#include "Rolling_Shutter_Plugin.h"


using namespace gz;
using namespace sim;
using namespace systems;

static sensor_msgs::msg::Image makeImageMsg(
    const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &img,
    const rclcpp::Node::SharedPtr &node,
    const std::string &frame_id)
{
  sensor_msgs::msg::Image msg;
  msg.header.stamp = node->get_clock()->now();
  msg.header.frame_id = frame_id;          // From frame_id
  msg.height = img.rows();
  msg.width  = img.cols();
  msg.encoding = "mono8";
  msg.is_bigendian = false;
  msg.step = msg.width;

  msg.data.resize(msg.height * msg.step);
  for (int r = 0; r < img.rows(); ++r)
    std::memcpy(&msg.data[r * msg.step], &img(r, 0), msg.step);
  return msg;
}


class gz::sim::systems::Rolling_Shutter_PluginPrivate
{
public:
  std::mutex mtx;
  std::condition_variable cv;
  std::vector<uint8_t> lastData;
  uint32_t lastWidth{0}, lastHeight{0}, lastStep{0};
  gz::msgs::PixelFormatType lastFmt{gz::msgs::PixelFormatType::L_INT8};
  uint64_t lastSeq{0};
  bool newFrame{false};

  // Derived variables for timing
  int NB{0};                    // number of blocks = ceil(outH / blockRows)
  int ms_per_block{1};          // readout_ms / NB (>= 1)
  std::vector<int> perBlockMs;  // per-block readout time (ms)

  int outW{80};                 // <rs_width>
  int outH{80};                 // <rs_height>
  int blockRows{10};            // <block_rows>
  int readout_ms{8};            // <readout_ms>
  int blank_ms{2};              // <blank_ms>
  int stepsPerMs{10};           // <steps_per_ms>
  std::string worldName{"empty"};               
  std::string cameraTopic{"/camera"};           
  std::string outTopic{"/camera/rolling_shutter"};
  std::string frame_id{"rolling_shutter_camera"};

  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rolling;
  int blockIdx{0};

  // ROS communications
  std::shared_ptr<rclcpp::Node> ros_node;
  rclcpp::Subscription<sar_msgs::msg::CtrlData>::SharedPtr Rolling_Shutter_Flag_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_; 

  // Internal state
  bool rolling_shutter_flag{false};
  bool prev_rolling_shutter_flag{false};
  bool rs_active{false};
  bool scanning{false};
  bool blanking{false};

  bool waitAfterPublish{false};
  std::chrono::steady_clock::time_point resumeAt;

};


Rolling_Shutter_Plugin::Rolling_Shutter_Plugin() : System(), dataPtr(std::make_unique<Rolling_Shutter_PluginPrivate>())
{
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  this->dataPtr->ros_node = std::make_shared<rclcpp::Node>("Rolling_Shutter_Node");

}



void Rolling_Shutter_Plugin::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm, EventManager &)
{
  (void)_entity;

  // Auto-detect world name (as before)
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (worldEntity != kNullEntity)
    if (auto name = _ecm.Component<components::Name>(worldEntity))
      this->dataPtr->worldName = name->Data();

  // --- Read all keys from SDF (override defaults when present) ---
  if (_sdf)
  {
    if (_sdf->HasElement("world_name"))
      this->dataPtr->worldName = _sdf->Get<std::string>("world_name");

    if (_sdf->HasElement("camera_topic"))
      this->dataPtr->cameraTopic = _sdf->Get<std::string>("camera_topic");

    if (_sdf->HasElement("steps_per_ms"))
      this->dataPtr->stepsPerMs = _sdf->Get<int>("steps_per_ms");

    if (_sdf->HasElement("rs_width"))
      this->dataPtr->outW = _sdf->Get<int>("rs_width");

    if (_sdf->HasElement("rs_height"))
      this->dataPtr->outH = _sdf->Get<int>("rs_height");

    if (_sdf->HasElement("block_rows"))
      this->dataPtr->blockRows = std::max(1, _sdf->Get<int>("block_rows"));

    if (_sdf->HasElement("readout_ms"))
      this->dataPtr->readout_ms = std::max(1, _sdf->Get<int>("readout_ms"));

    if (_sdf->HasElement("blank_ms"))
      this->dataPtr->blank_ms = std::max(0, _sdf->Get<int>("blank_ms"));

    if (_sdf->HasElement("out_topic"))
      this->dataPtr->outTopic = _sdf->Get<std::string>("out_topic");

    if (_sdf->HasElement("frame_id"))
      this->dataPtr->frame_id = _sdf->Get<std::string>("frame_id");
  }

  // Compute derived variables
  this->dataPtr->NB = (this->dataPtr->outH + this->dataPtr->blockRows - 1) / this->dataPtr->blockRows; // ceil
  this->dataPtr->ms_per_block = std::max(1, this->dataPtr->readout_ms / std::max(1, this->dataPtr->NB));

  // Distribute readout_ms across blocks exactly (no residual drift)
  this->dataPtr->perBlockMs.assign(this->dataPtr->NB, 0);

  if (this->dataPtr->NB > 0)
  {
    const int base = this->dataPtr->readout_ms / this->dataPtr->NB;   // base time per block
    int rem = this->dataPtr->readout_ms % this->dataPtr->NB;          // remaining milliseconds
    
    for (int i = 0; i < this->dataPtr->NB; ++i)
    {
      this->dataPtr->perBlockMs[i] = base + (i < rem ? 1 : 0);
      // Add one of the remaining milliseconds to the first 'rem' blocks
    }
  }

  // Prepare RS output buffer
  this->dataPtr->rolling.resize(this->dataPtr->outH, this->dataPtr->outW);
  this->dataPtr->rolling.setZero();
  this->dataPtr->blockIdx = 0;

  // Subscribe to camera
  bool ok = this->node.Subscribe(this->dataPtr->cameraTopic, &Rolling_Shutter_Plugin::OnImage, this);
  if (!ok)
    gzerr << "[RS] Failed to subscribe camera topic: " << this->dataPtr->cameraTopic << "\n";

  // Subscribe to /stats (optional)
  const std::string statsTopic = "/world/" + this->dataPtr->worldName + "/stats";
  this->node.Subscribe(statsTopic, &Rolling_Shutter_Plugin::OnStats, this);

  // Prepare ROS node/publisher
  if (!this->dataPtr->ros_node)
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    this->dataPtr->ros_node = std::make_shared<rclcpp::Node>("Rolling_Shutter_Node");
  }

  this->dataPtr->Rolling_Shutter_Flag_subscriber =
      this->dataPtr->ros_node->create_subscription<sar_msgs::msg::CtrlData>(
          "/CTRL/data", rclcpp::QoS(1),
          std::bind(&Rolling_Shutter_Plugin::Rolling_Shutter_Flag_Callback, this, std::placeholders::_1));

  auto qos = rclcpp::QoS(10).transient_local().reliable();
  this->dataPtr->image_pub_ = this->dataPtr->ros_node->create_publisher<sensor_msgs::msg::Image>(
      this->dataPtr->outTopic, qos);

  gzmsg << "[RS] Configured. world=" << this->dataPtr->worldName
        << " cam=" << this->dataPtr->cameraTopic
        << " stepsPerMs=" << this->dataPtr->stepsPerMs
        << " out=" << this->dataPtr->outW << "x" << this->dataPtr->outH
        << " blockRows=" << this->dataPtr->blockRows
        << " NB=" << this->dataPtr->NB
        << " readout_ms=" << this->dataPtr->readout_ms
        << " blank_ms=" << this->dataPtr->blank_ms
        << " ms_per_block=" << this->dataPtr->ms_per_block
        << " outTopic=" << this->dataPtr->outTopic
        << " frame_id=" << this->dataPtr->frame_id << "\n";
}

void Rolling_Shutter_Plugin::PreUpdate(const UpdateInfo &_info, 
                                EntityComponentManager &_ecm)
{
  GZ_PROFILE("Rolling_Shutter_Plugin::PreUpdate");
  //std::cout << "Rolling_Shutter_Plugin is started" << std::endl;
  if (this->dataPtr->ros_node) rclcpp::spin_some(this->dataPtr->ros_node);

  static uint64_t lastIter = 0;
  if (lastIter && _info.iterations != lastIter + 1) {
    gzdbg << "[RS-DBG] iteration jump: " << (_info.iterations - lastIter) << "\n";
  }
  lastIter = _info.iterations;  
  
}

//////////////////////////////////////////////////
void Rolling_Shutter_Plugin::PostUpdate(const UpdateInfo &, const EntityComponentManager &)
{
  GZ_PROFILE("Rolling_Shutter_Plugin::PostUpdate");

  // Edge detection for rolling shutter flag remains the same as before
  auto &flag_now  = this->dataPtr->rolling_shutter_flag;
  auto &flag_prev = this->dataPtr->prev_rolling_shutter_flag;

  if (flag_now != flag_prev)
  {
    if (flag_now && !this->dataPtr->rs_active)
    {
      gzmsg << "[RS] RS flag ON: PauseWorld(true) & start scanning\n";
      this->PauseWorld(true);
      this->dataPtr->rs_active = true;
      this->dataPtr->scanning = true;
      this->dataPtr->blanking = false;
      this->dataPtr->rolling.setZero();
      this->dataPtr->blockIdx = 0;
    }
    else if (!flag_now && this->dataPtr->rs_active)
    {
      gzmsg << "[RS] RS flag OFF: PauseWorld(false)\n";
      this->PauseWorld(false);
      this->dataPtr->rs_active = false;
      this->dataPtr->scanning = false;
      this->dataPtr->blanking = false;
    }
    flag_prev = flag_now;
  }

  // 3초 대기 상태라면 먼저 확인
  if (this->dataPtr->waitAfterPublish) {
    // 월드는 현재 unpause(false) 상태여야 PostUpdate가 호출됩니다.
    if (std::chrono::steady_clock::now() >= this->dataPtr->resumeAt) {
      // 다음 RS 사이클 시작: 다시 pause(true)로 전환하고 step-control 재개
      this->PauseWorld(true);
      this->dataPtr->waitAfterPublish = false;

      // 다음 주기 rolling shutter 재시작
      this->dataPtr->rs_active = true;
      this->dataPtr->scanning = true;
      this->dataPtr->blanking = false;
      this->dataPtr->blockIdx = 0;
      this->dataPtr->rolling.setZero();

      gzmsg << "[RS] 3s wait done → restart RS cycle (paused=true, step-controlled).\n";
    }
    // 아직 3초가 안 지났으면 그냥 대기
    return;
  }

  if (!this->dataPtr->rs_active)
    return;

  // (1) Scanning: advance ms_per_block (ms) per block → wait for frame → copy block
  if (this->dataPtr->scanning && this->dataPtr->blockIdx < this->dataPtr->NB)
  {
    int ms = (this->dataPtr->perBlockMs.empty() ? this->dataPtr->ms_per_block
                                                : this->dataPtr->perBlockMs[this->dataPtr->blockIdx]);
    if (ms > 0) {
      this->MultiStep(ms * this->dataPtr->stepsPerMs);
    }
    auto f = this->WaitLatestFrame(std::max(20, 3 * ms));
    
    if (!f) { gzdbg << "[RS] WaitLatestFrame timeout at block " << this->dataPtr->blockIdx << "\n"; return; }

    this->CopyBlockIntoRolling(*f, this->dataPtr->blockIdx);
    this->dataPtr->blockIdx++;

    if (this->dataPtr->blockIdx >= this->dataPtr->NB)
    {
      this->dataPtr->scanning = false;
      this->dataPtr->blanking = true;
    }
    return;
  }

  // (2) Blanking: advance by blank_ms, then publish
  if (this->dataPtr->blanking)
  {
    this->MultiStep(std::max(0, this->dataPtr->blank_ms) * this->dataPtr->stepsPerMs);

    auto msg = makeImageMsg(this->dataPtr->rolling, this->dataPtr->ros_node, this->dataPtr->frame_id);
    if (this->dataPtr->image_pub_) this->dataPtr->image_pub_->publish(msg);

    int readout_sum = this->dataPtr->readout_ms;
    if (this->dataPtr->perBlockMs.size() == static_cast<size_t>(this->dataPtr->NB)) {
      readout_sum = 0;
      for (int v : this->dataPtr->perBlockMs) readout_sum += v;
    }

    gzmsg << "[RS] Published RS frame (" << this->dataPtr->outW << "x" << this->dataPtr->outH
          << "). readout " << readout_sum
          << "ms + blank " << this->dataPtr->blank_ms << "ms\n";

    // 다음 사이클 준비
    this->dataPtr->rolling.setZero();
    this->dataPtr->blockIdx = 0;

    // 이번 사이클 종료
    this->dataPtr->scanning = false;
    this->dataPtr->blanking = false;
    this->dataPtr->rs_active = false;

    // **여기서 3초 대기 시작: 월드를 unpause 시켜서 PostUpdate가 계속 돌게 함**
    this->PauseWorld(false);  // 중요!
    this->dataPtr->waitAfterPublish = true;
    this->dataPtr->resumeAt = std::chrono::steady_clock::now() + std::chrono::seconds(3);

    gzmsg << "[RS] Unpaused world. Waiting 3s before next RS cycle.\n";
    return;
  }

}



void Rolling_Shutter_Plugin::Rolling_Shutter_Flag_Callback(const sar_msgs::msg::CtrlData::SharedPtr msg)
{
  this->dataPtr->rolling_shutter_flag = msg->rolling_shutter_flag;
  // std::cout << "rolling_shutter_flag: " << this->dataPtr->rolling_shutter_flag << std::endl;
  gzmsg << "[RS] /CTRL/data rolling_shutter_flag=" << std::boolalpha << this->dataPtr->rolling_shutter_flag << "\n";
}

void Rolling_Shutter_Plugin::OnImage(const gz::msgs::Image &msg)
{
  std::lock_guard<std::mutex> lk(this->dataPtr->mtx);
  this->dataPtr->lastWidth  = msg.width();
  this->dataPtr->lastHeight = msg.height();
  this->dataPtr->lastStep   = msg.step();
  this->dataPtr->lastFmt = msg.pixel_format_type();
  //this->dataPtr->lastSeq    = msg.header().sequence();

  const std::string &blob = msg.data();
  this->dataPtr->lastData.assign(blob.begin(), blob.end());

  this->dataPtr->newFrame = true;
  this->dataPtr->cv.notify_all();
}

void Rolling_Shutter_Plugin::PauseWorld(bool pause)
{
  const std::string svc = "/world/" + this->dataPtr->worldName + "/control";

  gz::msgs::WorldControl req;
  req.set_pause(pause);

  // Retry parameters
  const int maxAttempts = 20;             // Up to 20 attempts (~2 seconds total)
  const unsigned int timeoutMs = 100;     // 100 ms timeout per request
  const auto backoff = std::chrono::milliseconds(100);

  for (int attempt = 1; attempt <= maxAttempts; ++attempt)
  {
    gz::msgs::Boolean rep;
    bool result = false;

    const bool ok = this->node.Request(svc, req, timeoutMs, rep, result);

    gzmsg << "[RS] PauseWorld(" << std::boolalpha << pause << ") "
          << "attempt=" << attempt
          << " ok=" << ok
          << " transport_result=" << result
          << " reply_data=" << rep.data()
          << " svc=" << svc << "\n";

    if (ok && result && rep.data())
    {
      // Success
      return;
    }

    // Failed: wait briefly before retrying
    std::this_thread::sleep_for(backoff);
  }

  // If it reaches here, all retries failed
  gzerr << "[RS] PauseWorld failed after retries. svc=" << svc
        << " pause=" << std::boolalpha << pause << "\n";
}

void Rolling_Shutter_Plugin::OnStats(const gz::msgs::WorldStatistics &msg)
{
  // std::cout << "[RS] stats paused=" << std::boolalpha << msg.paused()
  //           << " sim_time="  << msg.sim_time().sec()  << "." << msg.sim_time().nsec()
  //           << " real_time=" << msg.real_time().sec() << "." << msg.real_time().nsec()
  //           << "\n";
}

void Rolling_Shutter_Plugin::MultiStep(int steps)
{
  const std::string svc = "/world/" + this->dataPtr->worldName + "/control";

  gz::msgs::WorldControl req;
  req.set_pause(true);           // Keep the simulation paused
  req.set_multi_step(steps);     // Advance exactly 'steps' steps while remaining paused

  gz::msgs::Boolean rep;
  bool result = false;
  unsigned int timeoutMs = 1000;

  bool ok = this->node.Request(svc, req, timeoutMs, rep, result);
  if (!ok || !result || !rep.data())
  {
    gzerr << "[RS] MultiStep(" << steps << ") failed (ok=" << ok
          << ", result=" << result << ", rep=" << rep.data() << ")\n";
  }

}


std::optional<Rolling_Shutter_Plugin::Frame>
Rolling_Shutter_Plugin::WaitLatestFrame(int timeout_ms)
{
  std::unique_lock<std::mutex> lk(this->dataPtr->mtx);
  if (!this->dataPtr->cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                                  [&]{ return this->dataPtr->newFrame; }))
    return std::nullopt;

  this->dataPtr->newFrame = false;
  if (this->dataPtr->lastData.empty())
    return std::nullopt;

  Frame f;
  f.buf    = this->dataPtr->lastData;  // Deep copy
  f.width  = this->dataPtr->lastWidth;
  f.height = this->dataPtr->lastHeight;
  f.step   = this->dataPtr->lastStep;
  f.fmt    = this->dataPtr->lastFmt;
  return f;
}

void Rolling_Shutter_Plugin::CopyBlockIntoRolling(const Frame &f, int blockIdx)
{
  if (f.fmt != gz::msgs::PixelFormatType::L_INT8)
  {
    gzerr << "[RS] Only L8 handled in this example\n";
    return;
  }

  const int H = this->dataPtr->outH;
  const int W = this->dataPtr->outW;
  const int B = this->dataPtr->blockRows;
  const int y0 = blockIdx * B;
  const int y1 = std::min(y0 + B, H);

  if (f.height == 0 || f.width == 0) {
    gzerr << "[RS] Empty source frame\n";
    return;
  }  
  
  for (int yOut = y0; yOut < y1; ++yOut)
  {
    int ySrc = std::min(yOut, static_cast<int>(f.height) - 1);

    const uint8_t* row = f.buf.data() + ySrc * f.step;

    if (static_cast<int>(f.width) == W)
    {
      std::memcpy(&this->dataPtr->rolling(yOut, 0), row, W);
    }
    else if (static_cast<int>(f.width) > W)
    {
      int x0 = (static_cast<int>(f.width) - W) / 2; // Center crop
      std::memcpy(&this->dataPtr->rolling(yOut, 0), row + x0, W);
    }
    else
    {
      int copyW = std::min(W, static_cast<int>(f.width));
      std::memcpy(&this->dataPtr->rolling(yOut, 0), row, copyW);
      std::memset(&this->dataPtr->rolling(yOut, copyW), 0, W - copyW); // Zero padding
    }
  }
}



GZ_ADD_PLUGIN(Rolling_Shutter_Plugin,
                    System,
                    Rolling_Shutter_Plugin::ISystemConfigure,
                    Rolling_Shutter_Plugin::ISystemPreUpdate,
                    //Rolling_Shutter_Plugin::ISystemUpdate,
                    Rolling_Shutter_Plugin::ISystemPostUpdate)
                    

GZ_ADD_PLUGIN_ALIAS(Rolling_Shutter_Plugin, "gz::sim::systems::Rolling_Shutter_Plugin")
