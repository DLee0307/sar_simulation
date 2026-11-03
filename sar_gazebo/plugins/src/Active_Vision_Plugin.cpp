#include "Active_Vision_Plugin.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <sdf/Sensor.hh>
#include <sdf/Camera.hh>

using namespace gz;
using namespace sim;
using namespace systems;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultRateHz = 10.0;

// Runtime operating mode: mutually exclusive (no conflicting publishers).
enum class OperationMode { DetectionOnly, SweepTest, SearchTrack };
enum class TrackingState { Search, Track, Lost };

const char *StateName(TrackingState state)
{
  switch (state)
  {
    case TrackingState::Search: return "SEARCH";
    case TrackingState::Track:  return "TRACK";
    case TrackingState::Lost:   return "LOST";
  }
  return "UNKNOWN";
}

struct CameraConfig
{
  double horizontalFovRad{0.0};
  bool valid{false};
};

struct GimbalConfig
{
  double minPitchRad{0.0};
  double maxPitchRad{0.0};
  double maxVelocityRadS{0.0};
  bool valid{false};
};

struct ActiveVisionConfig
{
  OperationMode mode{OperationMode::SearchTrack};
  double rateHz{kDefaultRateHz};
  int targetTagId{0};
  bool enableDebugImage{true};

  // Algorithm parameters only. Physical limits / camera FOV are NOT duplicated.
  double searchMinPitchRad{0.0};
  double searchMaxPitchRad{kPi / 4.0};
  double searchPeriodSec{6.0};
  double trackingGain{0.65};
  double deadbandNormalized{0.02};  // fraction of image height
  double velocityFraction{0.6};    // fraction of joint's SDF velocity limit
  double trackingPitchSign{1.0};   // optional mounting-direction calibration
  double lostTimeoutSec{0.5};
  int reacquireFrames{2};
};

struct TagObservation
{
  bool received{false};   // fresh, valid camera image was processed
  bool detected{false};   // target marker ID found
  double u{0.0};
  double v{0.0};
  int width{0};
  int height{0};
};

void SetRgbPixel(std::string &rgb, int width, int height,
                 int x, int y, unsigned char red,
                 unsigned char green, unsigned char blue)
{
  if (x < 0 || y < 0 || x >= width || y >= height)
    return;
  const std::size_t idx = 3U * (static_cast<std::size_t>(y) * width + x);
  rgb[idx] = static_cast<char>(red);
  rgb[idx + 1] = static_cast<char>(green);
  rgb[idx + 2] = static_cast<char>(blue);
}

void DrawLine(std::string &rgb, int width, int height,
              int x0, int y0, int x1, int y1,
              unsigned char r, unsigned char g, unsigned char b)
{
  x0 = std::clamp(x0, 0, width - 1);
  x1 = std::clamp(x1, 0, width - 1);
  y0 = std::clamp(y0, 0, height - 1);
  y1 = std::clamp(y1, 0, height - 1);
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  while (true)
  {
    SetRgbPixel(rgb, width, height, x0, y0, r, g, b);
    SetRgbPixel(rgb, width, height, x0 + 1, y0, r, g, b);
    if (x0 == x1 && y0 == y1)
      break;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

void DrawCross(std::string &rgb, int width, int height,
               int u, int v, unsigned char r, unsigned char g,
               unsigned char b)
{
  for (int d = -3; d <= 3; ++d)
  {
    SetRgbPixel(rgb, width, height, u + d, v, r, g, b);
    SetRgbPixel(rgb, width, height, u, v + d, r, g, b);
  }
}

TagObservation DetectAndPublish(
    const gz::msgs::Image &frame,
    apriltag_detector_t *detector, int targetId,
    bool debugEnabled, gz::transport::Node::Publisher &debugPublisher)
{
  TagObservation result;
  const int width = static_cast<int>(frame.width());
  const int height = static_cast<int>(frame.height());
  if (width <= 0 || height <= 0 ||
      frame.pixel_format_type() != gz::msgs::L_INT8)
    return result;

  const std::size_t stride =
      frame.step() >= frame.width() ? frame.step() : frame.width();
  const std::size_t required = stride * static_cast<std::size_t>(height - 1) + width;
  if (frame.data().size() < required)
    return result;

  result.received = true;
  result.width = width;
  result.height = height;
  std::vector<std::uint8_t> gray(static_cast<std::size_t>(width) * height);
  for (int row = 0; row < height; ++row)
  {
    std::memcpy(gray.data() + static_cast<std::size_t>(row) * width,
                frame.data().data() + static_cast<std::size_t>(row) * stride,
                static_cast<std::size_t>(width));
  }

  // Ubuntu's AprilTag headers use const width/height/stride members.
  image_u8_t image{width, height, width, gray.data()};
  zarray_t *detections = detector ? apriltag_detector_detect(detector, &image)
                                  : nullptr;
  apriltag_detection_t *target = nullptr;
  if (detections)
  {
    for (int i = 0; i < zarray_size(detections); ++i)
    {
      apriltag_detection_t *tag = nullptr;
      zarray_get(detections, i, &tag);
      if (tag && tag->id == targetId &&
          std::isfinite(tag->c[0]) && std::isfinite(tag->c[1]))
      {
        target = tag;
        result.detected = true;
        result.u = tag->c[0];
        result.v = tag->c[1];
        break;
      }
    }
  }

  if (debugEnabled && debugPublisher)
  {
    std::string rgb(static_cast<std::size_t>(width) * height * 3U, '\0');
    for (std::size_t p = 0; p < gray.size(); ++p)
    {
      rgb[3 * p] = static_cast<char>(gray[p]);
      rgb[3 * p + 1] = static_cast<char>(gray[p]);
      rgb[3 * p + 2] = static_cast<char>(gray[p]);
    }
    // Blue cross: camera center, green cross + outline: detected tag center.
    DrawCross(rgb, width, height, (width - 1) / 2, (height - 1) / 2,
              0, 0, 255);
    if (target)
    {
      for (int j = 0; j < 4; ++j)
      {
        const int next = (j + 1) % 4;
        const double x0 = target->p[j][0];
        const double y0 = target->p[j][1];
        const double x1 = target->p[next][0];
        const double y1 = target->p[next][1];
        if (std::isfinite(x0) && std::isfinite(y0) &&
            std::isfinite(x1) && std::isfinite(y1))
        {
          DrawLine(rgb, width, height,
                   static_cast<int>(std::lround(x0)),
                   static_cast<int>(std::lround(y0)),
                   static_cast<int>(std::lround(x1)),
                   static_cast<int>(std::lround(y1)), 0, 255, 0);
        }
      }
      DrawCross(rgb, width, height,
                static_cast<int>(std::lround(result.u)),
                static_cast<int>(std::lround(result.v)), 0, 255, 0);
    }

    gz::msgs::Image out;
    out.mutable_header()->CopyFrom(frame.header());
    out.set_width(frame.width());
    out.set_height(frame.height());
    out.set_step(static_cast<std::uint32_t>(width * 3));
    out.set_pixel_format_type(gz::msgs::RGB_INT8);
    out.set_data(rgb);
    debugPublisher.Publish(out);
  }

  if (detections)
    apriltag_detections_destroy(detections);
  return result;
}
} // namespace

class gz::sim::systems::Active_Vision_PluginPrivate
{
  public: std::string cameraTopic{"/camera"};
  public: std::string jointStateTopic{"/SAR_Base/gimbal/joint_state"};
  public: std::string pitchCommandTopic{"/SAR_Base/gimbal/pitch_cmd"};
  public: std::string debugImageTopic{"/camera/active_vision"};
  // Display-only RGB8 stream. Optional fixed gamma changes GUI brightness only.
  public: std::string rawDisplayTopic{"/camera/raw_rgb"};
  public: double displayGamma{1.0};  // 1.0 = original; < 1.0 = brighter
  public: std::array<std::uint8_t, 256> displayGammaLut{};
  public: std::string cameraLinkName{"Camera"};
  public: std::string cameraSensorName{"camera"};
  public: std::string pitchJointName{"Camera_Pitch_Joint"};

  public: CameraConfig camera;
  public: GimbalConfig gimbal;
  public: ActiveVisionConfig cfg;

  public: std::mutex imageMutex;
  public: gz::msgs::Image latestImage;
  public: bool haveImage{false};
  public: std::uint64_t imageCount{0};
  public: std::uint64_t processedImageCount{0};

  // Transport callbacks are asynchronous relative to the Gazebo simulation thread.
  public: std::atomic<double> actualPitchRad{0.0};
  public: std::atomic<double> actualVelocityRadS{0.0};
  public: std::atomic<bool> haveJointState{false};
  public: std::atomic<double> commandedPitchRad{0.0};

  public: gz::transport::Node::Publisher pitchPublisher;
  public: gz::transport::Node::Publisher debugPublisher;
  public: gz::transport::Node::Publisher rawDisplayPublisher;
  public: apriltag_family_t *tagFamily{nullptr};
  public: apriltag_detector_t *tagDetector{nullptr};

  // Simulation-thread-only runtime state.
  public: bool clockInitialized{false};
  public: std::chrono::steady_clock::duration startSimTime{0};
  public: double nextVisionTickSec{0.0};
  public: std::uint64_t visionTickCount{0};
  public: TrackingState trackingState{TrackingState::Search};
  public: double lastSeenSec{-1.0};
  public: double lostStartSec{0.0};
  public: int detectionStreak{0};
  public: double searchStartSec{0.0};
  public: bool searchPhaseInitialized{false};

  public: void EnterState(TrackingState next)
  {
    if (this->trackingState != next)
    {
      gzwarn << "[Active_Vision_Plugin] " << StateName(this->trackingState)
             << " -> " << StateName(next) << std::endl;
      this->trackingState = next;
    }
  }

  public: void InitializeSearchPhase(double elapsedSec, double actualPitch)
  {
    const double lo = std::max(cfg.searchMinPitchRad, gimbal.minPitchRad);
    const double hi = std::min(cfg.searchMaxPitchRad, gimbal.maxPitchRad);
    const double ratio = std::clamp((actualPitch - lo) / (hi - lo), 0.0, 1.0);
    double phi = std::acos(std::clamp(1.0 - 2.0 * ratio, -1.0, 1.0));
    // Continue decreasing if the servo was previously moving downward.
    if (actualVelocityRadS.load() < -0.01)
      phi = 2.0 * kPi - phi;
    searchStartSec = elapsedSec - phi * cfg.searchPeriodSec / (2.0 * kPi);
    searchPhaseInitialized = true;
  }

  public: double SafeCommand(double requested, double actual) const
  {
    // Joint SDF is the sole source of the physical position/velocity limits.
    const double lo = gimbal.minPitchRad;
    const double hi = gimbal.maxPitchRad;
    const double maxStep = gimbal.maxVelocityRadS * cfg.velocityFraction /
                           cfg.rateHz;
    const double bounded = std::clamp(requested, lo, hi);
    return std::clamp(actual + std::clamp(bounded - actual, -maxStep, maxStep),
                      lo, hi);
  }

  public: void PublishPitch(double requested, double actual)
  {
    if (!pitchPublisher || !gimbal.valid)
      return;
    const double angle = SafeCommand(requested, actual);
    gz::msgs::Double command;
    command.set_data(angle);
    pitchPublisher.Publish(command);
    commandedPitchRad.store(angle);
  }
};

Active_Vision_Plugin::Active_Vision_Plugin()
    : dataPtr(std::make_unique<Active_Vision_PluginPrivate>())
{
}

Active_Vision_Plugin::~Active_Vision_Plugin()
{
  if (dataPtr->tagDetector)
    apriltag_detector_destroy(dataPtr->tagDetector);
  if (dataPtr->tagFamily)
    tag36h11_destroy(dataPtr->tagFamily);
}

void Active_Vision_Plugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager & /*_eventMgr*/)
{
  if (_sdf)
  {
    auto &p = *this->dataPtr;
    if (_sdf->HasElement("camera_topic"))
      p.cameraTopic = _sdf->Get<std::string>("camera_topic");
    if (_sdf->HasElement("joint_state_topic"))
      p.jointStateTopic = _sdf->Get<std::string>("joint_state_topic");
    if (_sdf->HasElement("pitch_command_topic"))
      p.pitchCommandTopic = _sdf->Get<std::string>("pitch_command_topic");
    if (_sdf->HasElement("debug_image_topic"))
      p.debugImageTopic = _sdf->Get<std::string>("debug_image_topic");
    if (_sdf->HasElement("raw_display_topic"))
      p.rawDisplayTopic = _sdf->Get<std::string>("raw_display_topic");
    if (_sdf->HasElement("display_gamma"))
      p.displayGamma = _sdf->Get<double>("display_gamma");
    if (_sdf->HasElement("camera_link_name"))
      p.cameraLinkName = _sdf->Get<std::string>("camera_link_name");
    if (_sdf->HasElement("camera_sensor_name"))
      p.cameraSensorName = _sdf->Get<std::string>("camera_sensor_name");
    if (_sdf->HasElement("pitch_joint_name"))
      p.pitchJointName = _sdf->Get<std::string>("pitch_joint_name");

    if (_sdf->HasElement("mode"))
    {
      const auto modeName = _sdf->Get<std::string>("mode");
      if (modeName == "search_track") p.cfg.mode = OperationMode::SearchTrack;
      else if (modeName == "sweep_test") p.cfg.mode = OperationMode::SweepTest;
      else if (modeName == "detection_only") p.cfg.mode = OperationMode::DetectionOnly;
      else
      {
        gzerr << "[Active_Vision_Plugin] Invalid mode '" << modeName
              << "'; disabling motion (detection_only).\n";
        p.cfg.mode = OperationMode::DetectionOnly;
      }
    }
    if (_sdf->HasElement("active_vision_rate_hz"))
      p.cfg.rateHz = _sdf->Get<double>("active_vision_rate_hz");
    if (_sdf->HasElement("target_tag_id"))
      p.cfg.targetTagId = _sdf->Get<int>("target_tag_id");
    if (_sdf->HasElement("enable_debug_image"))
      p.cfg.enableDebugImage = _sdf->Get<bool>("enable_debug_image");
    if (_sdf->HasElement("search_min_pitch_rad"))
      p.cfg.searchMinPitchRad = _sdf->Get<double>("search_min_pitch_rad");
    if (_sdf->HasElement("search_max_pitch_rad"))
      p.cfg.searchMaxPitchRad = _sdf->Get<double>("search_max_pitch_rad");
    if (_sdf->HasElement("search_period_sec"))
      p.cfg.searchPeriodSec = _sdf->Get<double>("search_period_sec");
    if (_sdf->HasElement("tracking_gain"))
      p.cfg.trackingGain = _sdf->Get<double>("tracking_gain");
    if (_sdf->HasElement("tracking_deadband_normalized"))
      p.cfg.deadbandNormalized = _sdf->Get<double>("tracking_deadband_normalized");
    if (_sdf->HasElement("tracking_velocity_fraction"))
      p.cfg.velocityFraction = _sdf->Get<double>("tracking_velocity_fraction");
    if (_sdf->HasElement("tracking_pitch_sign"))
      p.cfg.trackingPitchSign = _sdf->Get<double>("tracking_pitch_sign");
    if (_sdf->HasElement("lost_timeout_sec"))
      p.cfg.lostTimeoutSec = _sdf->Get<double>("lost_timeout_sec");
    if (_sdf->HasElement("reacquire_frames"))
      p.cfg.reacquireFrames = _sdf->Get<int>("reacquire_frames");
  }

  auto &p = *this->dataPtr;
  if (!std::isfinite(p.cfg.rateHz) || p.cfg.rateHz <= 0.0)
  {
    gzerr << "[Active_Vision_Plugin] Invalid rate; using 10 Hz.\n";
    p.cfg.rateHz = kDefaultRateHz;
  }
  if (!std::isfinite(p.cfg.searchPeriodSec) || p.cfg.searchPeriodSec <= 0.0)
    p.cfg.searchPeriodSec = 6.0;
  if (!std::isfinite(p.cfg.lostTimeoutSec) || p.cfg.lostTimeoutSec < 0.0)
    p.cfg.lostTimeoutSec = 0.5;
  if (!std::isfinite(p.cfg.trackingGain) || p.cfg.trackingGain <= 0.0)
    p.cfg.trackingGain = 0.65;
  if (!std::isfinite(p.cfg.deadbandNormalized) || p.cfg.deadbandNormalized < 0.0)
    p.cfg.deadbandNormalized = 0.02;
  if (!std::isfinite(p.cfg.velocityFraction) || p.cfg.velocityFraction <= 0.0 ||
      p.cfg.velocityFraction > 1.0)
    p.cfg.velocityFraction = 0.6;
  if (p.cfg.reacquireFrames < 1)
    p.cfg.reacquireFrames = 2;
  if (p.cfg.trackingPitchSign != -1.0 && p.cfg.trackingPitchSign != 1.0)
    p.cfg.trackingPitchSign = 1.0;

  // Fixed gamma lookup table: generated once, not per pixel or per frame.
  // It is used ONLY for /camera/raw_rgb visualization; /camera is untouched.
  if (!std::isfinite(p.displayGamma) ||
      p.displayGamma < 0.05 || p.displayGamma > 5.0)
  {
    gzwarn << "[Active_Vision_Plugin] Invalid display_gamma; "
              "using 1.0 (no display correction).\n";
    p.displayGamma = 1.0;
  }
  for (std::size_t i = 0; i < p.displayGammaLut.size(); ++i)
  {
    const double normalized = static_cast<double>(i) / 255.0;
    const double corrected = 255.0 * std::pow(normalized, p.displayGamma);
    p.displayGammaLut[i] = static_cast<std::uint8_t>(
        std::lround(std::clamp(corrected, 0.0, 255.0)));
  }

  // Read physical parameters FROM existing camera / joint SDF via ECM.
  const gz::sim::Model model(_entity);
  const auto jointEntity = model.JointByName(_ecm, p.pitchJointName);
  if (jointEntity != gz::sim::kNullEntity)
  {
    const gz::sim::Joint joint(jointEntity);
    const auto axes = joint.Axis(_ecm);
    if (axes && !axes->empty())
    {
      const auto &axis = axes->front();
      p.gimbal.minPitchRad = axis.Lower();
      p.gimbal.maxPitchRad = axis.Upper();
      p.gimbal.maxVelocityRadS = axis.MaxVelocity();
      p.gimbal.valid =
          std::isfinite(p.gimbal.minPitchRad) &&
          std::isfinite(p.gimbal.maxPitchRad) &&
          std::isfinite(p.gimbal.maxVelocityRadS) &&
          p.gimbal.minPitchRad < p.gimbal.maxPitchRad &&
          p.gimbal.maxVelocityRadS > 0.0;
    }
  }
  if (!p.gimbal.valid)
    gzerr << "[Active_Vision_Plugin] Cannot read valid '" << p.pitchJointName
          << "' limits from ECM. Pitch commands disabled.\n";

  const auto linkEntity = model.LinkByName(_ecm, p.cameraLinkName);
  if (linkEntity != gz::sim::kNullEntity)
  {
    // Gazebo Sim 8.6: Link::SensorByName() does not exist.
    // Match the sensor by parent Link entity, unscoped name, and Sensor marker.
    const auto sensorEntity = _ecm.EntityByComponents(
        gz::sim::components::ParentEntity(linkEntity),
        gz::sim::components::Name(p.cameraSensorName),
        gz::sim::components::Sensor());
    if (sensorEntity != gz::sim::kNullEntity)
    {
      const auto *comp = _ecm.Component<gz::sim::components::Camera>(sensorEntity);
      if (comp)
      {
        const auto *cameraSensor = comp->Data().CameraSensor();
        if (cameraSensor)
        {
          p.camera.horizontalFovRad = cameraSensor->HorizontalFov().Radian();
          p.camera.valid = std::isfinite(p.camera.horizontalFovRad) &&
                           p.camera.horizontalFovRad > 0.0 &&
                           p.camera.horizontalFovRad < kPi;
        }
      }
    }
  }
  if (!p.camera.valid)
    gzerr << "[Active_Vision_Plugin] Cannot read camera HFOV from ECM. "
          << "Tracking disabled; check camera link / sensor names.\n";

  if (p.gimbal.valid)
  {
    const double sLo = std::max(p.cfg.searchMinPitchRad, p.gimbal.minPitchRad);
    const double sHi = std::min(p.cfg.searchMaxPitchRad, p.gimbal.maxPitchRad);
    if (!(std::isfinite(sLo) && std::isfinite(sHi) && sLo < sHi))
    {
      gzerr << "[Active_Vision_Plugin] Search range must overlap joint limits. "
            << "Disabling actuation.\n";
      p.gimbal.valid = false;
    }
  }

  p.tagFamily = tag36h11_create();
  p.tagDetector = apriltag_detector_create();
  if (p.tagFamily && p.tagDetector)
  {
    apriltag_detector_add_family(p.tagDetector, p.tagFamily);
    p.tagDetector->nthreads = 1;
    p.tagDetector->quad_decimate = 1.0f;
    p.tagDetector->quad_sigma = 0.0f;
    p.tagDetector->refine_edges = 1;
  }
  else
    gzerr << "[Active_Vision_Plugin] AprilTag detector init failed.\n";

  const bool cameraOk = node.Subscribe(p.cameraTopic,
                                      &Active_Vision_Plugin::CameraMsg, this);
  const bool jointOk = node.Subscribe(p.jointStateTopic,
                                     &Active_Vision_Plugin::JointStateMsg, this);
  // The original /camera topic is not modified; display uses its own topic.
  if (!p.rawDisplayTopic.empty() && p.rawDisplayTopic != p.cameraTopic &&
      p.rawDisplayTopic != p.debugImageTopic)
  {
    p.rawDisplayPublisher =
        node.Advertise<gz::msgs::Image>(p.rawDisplayTopic);
  }
  else if (!p.rawDisplayTopic.empty())
  {
    gzerr << "[Active_Vision_Plugin] raw_display_topic must differ "
             "from camera_topic and debug_image_topic.\n";
  }
  p.pitchPublisher = node.Advertise<gz::msgs::Double>(p.pitchCommandTopic);
  if (p.cfg.enableDebugImage)
    p.debugPublisher = node.Advertise<gz::msgs::Image>(p.debugImageTopic);

  gzwarn << "[Active_Vision_Plugin] mode="
         << (p.cfg.mode == OperationMode::SearchTrack ? "search_track" :
             p.cfg.mode == OperationMode::SweepTest ? "sweep_test" : "detection_only")
         << ", rate=" << p.cfg.rateHz << "Hz, camera HFOV="
         << p.camera.horizontalFovRad << "rad, joint=["
         << p.gimbal.minPitchRad << "," << p.gimbal.maxPitchRad
         << "]rad, velocity=" << p.gimbal.maxVelocityRadS << "rad/s"
         << ", subscribed=" << cameraOk << "/" << jointOk
         << ", raw RGB topic=" << p.rawDisplayTopic
         << ", display gamma=" << p.displayGamma
         << ", raw RGB publisher=" << static_cast<bool>(p.rawDisplayPublisher)
         << std::endl;

  if (!cameraOk || !jointOk ||
      (!p.rawDisplayTopic.empty() && !p.rawDisplayPublisher) ||
      (p.cfg.enableDebugImage && !p.debugPublisher) ||
      (p.cfg.mode != OperationMode::DetectionOnly && !p.pitchPublisher))
    gzerr << "[Active_Vision_Plugin] Gazebo Transport setup error.\n";
}

void Active_Vision_Plugin::CameraMsg(const gz::msgs::Image &_msg)
{
  if (_msg.width() == 0 || _msg.height() == 0)
    return;
  auto &p = *dataPtr;

  // Save the original L8 frame for 10-Hz AprilTag detection before any GUI work.
  // A slow display subscriber must not hold the frame-cache mutex.
  {
    std::lock_guard<std::mutex> lock(p.imageMutex);
    p.latestImage.CopyFrom(_msg);
    p.haveImage = true;
    ++p.imageCount;
  }

  // Publish display-only RGB8 at the CAMERA sensor rate (100 Hz).
  // A fixed gamma LUT improves visibility without per-frame normalization.
  // Optical-flow and AprilTag inputs continue using the untouched /camera L8.
  if (!p.rawDisplayPublisher ||
      _msg.pixel_format_type() != gz::msgs::L_INT8)
    return;

  const std::size_t width = static_cast<std::size_t>(_msg.width());
  const std::size_t height = static_cast<std::size_t>(_msg.height());
  const std::size_t stride =
      _msg.step() >= _msg.width() ? static_cast<std::size_t>(_msg.step()) : width;
  if (height == 0 || _msg.data().size() < stride * (height - 1) + width)
    return;

  std::string rgb(width * height * 3U, '\0');
  for (std::size_t y = 0; y < height; ++y)
  {
    const std::size_t srcRow = y * stride;
    const std::size_t dstRow = 3U * y * width;
    for (std::size_t x = 0; x < width; ++x)
    {
      const auto luminance = static_cast<std::uint8_t>(
          static_cast<unsigned char>(_msg.data()[srcRow + x]));
      const char shown = static_cast<char>(p.displayGammaLut[luminance]);
      const std::size_t dst = dstRow + 3U * x;
      rgb[dst] = shown;
      rgb[dst + 1] = shown;
      rgb[dst + 2] = shown;
    }
  }

  gz::msgs::Image display;
  display.mutable_header()->CopyFrom(_msg.header());
  display.set_width(_msg.width());
  display.set_height(_msg.height());
  display.set_step(static_cast<std::uint32_t>(3U * width));
  display.set_pixel_format_type(gz::msgs::RGB_INT8);
  display.set_data(std::move(rgb));
  p.rawDisplayPublisher.Publish(display);
}

void Active_Vision_Plugin::JointStateMsg(const gz::msgs::Model &_msg)
{
  for (const auto &joint : _msg.joint())
  {
    if (joint.name() != dataPtr->pitchJointName || !joint.has_axis1())
      continue;
    dataPtr->actualPitchRad.store(joint.axis1().position());
    dataPtr->actualVelocityRadS.store(joint.axis1().velocity());
    dataPtr->haveJointState.store(true);
    break;
  }
}

void Active_Vision_Plugin::PreUpdate(const UpdateInfo &_info,
                                      EntityComponentManager & /*_ecm*/)
{
  auto &p = *dataPtr;
  if (_info.paused)
    return;

  if (!p.clockInitialized || _info.simTime < p.startSimTime)
  {
    p.clockInitialized = true;
    p.startSimTime = _info.simTime;
    p.nextVisionTickSec = 0.0;
    p.visionTickCount = 0;
    p.trackingState = TrackingState::Search;
    p.searchPhaseInitialized = false;
    p.lastSeenSec = -1.0;
    p.detectionStreak = 0;
  }
  const double elapsedSec =
      std::chrono::duration<double>(_info.simTime - p.startSimTime).count();
  if (elapsedSec + 1e-9 < p.nextVisionTickSec)
    return;
  const double period = 1.0 / p.cfg.rateHz;
  p.nextVisionTickSec += period;
  if (p.nextVisionTickSec <= elapsedSec)
    p.nextVisionTickSec = (std::floor(elapsedSec / period) + 1) * period;
  ++p.visionTickCount;

  // Read only the newest of the 100-Hz frames. No backlogged processing.
  gz::msgs::Image frame;
  bool newFrame = false;
  {
    std::lock_guard<std::mutex> lock(p.imageMutex);
    if (p.haveImage && p.imageCount != p.processedImageCount)
    {
      frame.CopyFrom(p.latestImage);
      p.processedImageCount = p.imageCount;
      newFrame = true;
    }
  }

  TagObservation obs;
  if (newFrame)
    obs = DetectAndPublish(frame, p.tagDetector, p.cfg.targetTagId,
                           p.cfg.enableDebugImage, p.debugPublisher);

  if (p.visionTickCount % 10 == 0)
  {
    gzmsg << "[Active_Vision_Plugin] tick=" << p.visionTickCount
          << " state=" << StateName(p.trackingState)
          << " new_frame=" << obs.received
          << " tag=" << obs.detected
          << " pitch=" << p.actualPitchRad.load() << std::endl;
  }

  if (p.cfg.mode == OperationMode::DetectionOnly)
    return;
  if (!p.gimbal.valid || !p.haveJointState.load() || !p.pitchPublisher)
    return;

  const double actualPitch = p.actualPitchRad.load();
  const double searchLo = std::max(p.cfg.searchMinPitchRad, p.gimbal.minPitchRad);
  const double searchHi = std::min(p.cfg.searchMaxPitchRad, p.gimbal.maxPitchRad);

  if (p.cfg.mode == OperationMode::SweepTest)
  {
    if (!p.searchPhaseInitialized)
      p.InitializeSearchPhase(elapsedSec, actualPitch);
    const double phase = 2.0 * kPi *
        (elapsedSec - p.searchStartSec) / p.cfg.searchPeriodSec;
    const double cmd = searchLo + 0.5 * (searchHi - searchLo) *
        (1.0 - std::cos(phase));
    p.PublishPitch(cmd, actualPitch);
    return;
  }

  // SEARCH / TRACK / LOST state machine.
  if (obs.received && obs.detected)
  {
    p.lastSeenSec = elapsedSec;
    ++p.detectionStreak;
  }
  else
  {
    p.detectionStreak = 0;
  }

  if (p.trackingState == TrackingState::Search)
  {
    if (p.detectionStreak >= p.cfg.reacquireFrames)
    {
      p.EnterState(TrackingState::Track);
      p.searchPhaseInitialized = false;
    }
    else
    {
      if (!p.searchPhaseInitialized)
        p.InitializeSearchPhase(elapsedSec, actualPitch);
      const double phase = 2.0 * kPi *
          (elapsedSec - p.searchStartSec) / p.cfg.searchPeriodSec;
      const double sweep = searchLo + 0.5 * (searchHi - searchLo) *
          (1.0 - std::cos(phase));
      // Hold briefly when first seen to obtain a confirming detection.
      p.PublishPitch(obs.detected ? actualPitch : sweep, actualPitch);
      return;
    }
  }

  if (p.trackingState == TrackingState::Track)
  {
    if (!obs.received || !obs.detected)
    {
      p.EnterState(TrackingState::Lost);
      p.lostStartSec = elapsedSec;
      p.PublishPitch(actualPitch, actualPitch);
      return;
    }

    // 1-DOF pitch can correct image V, not image U.
    if (!p.camera.valid)
    {
      p.PublishPitch(actualPitch, actualPitch);
      return;
    }
    const double imageCenterV = 0.5 * (obs.height - 1.0);
    const double errorV = obs.v - imageCenterV;
    const double normalizedError = errorV / static_cast<double>(obs.height);
    if (std::abs(normalizedError) <= p.cfg.deadbandNormalized)
    {
      p.PublishPitch(actualPitch, actualPitch);
      return;
    }
    // Square-pixel pinhole camera: fy = fx from the camera's original HFOV.
    const double focalPx = (0.5 * obs.width) /
        std::tan(0.5 * p.camera.horizontalFovRad);
    const double angularError = std::atan2(errorV, focalPx);
    const double requested = actualPitch +
        p.cfg.trackingPitchSign * p.cfg.trackingGain * angularError;
    p.PublishPitch(requested, actualPitch);
    return;
  }

  // LOST: hold the current pitch to reject brief detection dropouts.
  // Reacquire with consecutive valid frames; otherwise resume scanning.
  if (p.trackingState == TrackingState::Lost)
  {
    if (p.detectionStreak >= p.cfg.reacquireFrames)
    {
      p.EnterState(TrackingState::Track);
      // Tracking begins on next 10 Hz tick.
    }
    else if (elapsedSec - p.lostStartSec >= p.cfg.lostTimeoutSec)
    {
      p.EnterState(TrackingState::Search);
      p.searchPhaseInitialized = false;
      p.detectionStreak = 0;
    }
    p.PublishPitch(actualPitch, actualPitch);
  }

}

GZ_ADD_PLUGIN(
    Active_Vision_Plugin,
    System,
    Active_Vision_Plugin::ISystemConfigure,
    Active_Vision_Plugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    Active_Vision_Plugin,
    "gz::sim::systems::Active_Vision_Plugin")