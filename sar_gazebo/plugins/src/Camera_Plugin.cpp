#include "Camera_Plugin.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>

using namespace gz;
using namespace sim;
using namespace systems;

namespace {
// 기존 너 코드와 동일한 기본값/클램프 유지
constexpr float kDefaultTau   = 5.0f;
constexpr float kDefaultTheta = 20.0f;

constexpr float kTauClampAbs   = 5.0f;
constexpr float kThetaClampAbs = 20.0f;

// ===== OpenMV 설정과 맞추기 위한 카메라 파라미터 =====
// OpenMV 코드: w=6.0e-6, f=2.8e-3
constexpr double kPixelW   = 6.0e-6;   // [m] pixel pitch
constexpr double kFocalLen = 2.8e-3;   // [m] focal length (OpenMV script f)

// 100Hz
constexpr double kDeltaT = 0.01;

// SVD singular value threshold
constexpr double kSvdEps = 1e-6;

// ===== Windowing(중앙 crop) =====
// OpenMV: sensor.set_windowing((64,64))
constexpr int kCropW = 64;
constexpr int kCropH = 64;
}  // namespace

class gz::sim::systems::Camera_PluginPrivate
{
public:
  // RowMajor로 두면 row 단위 memcpy가 깔끔함
  using ImageU8 = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  // 여기에는 "crop된 64x64"만 저장할 것
  ImageU8 cur_img;
  ImageU8 prev_img;
  bool have_prev = false;

  std::chrono::time_point<std::chrono::high_resolution_clock> last_time;

  bool Optical_Flow_Flag = false;
  float Tau_DH = 0.0f;
  float Theta_x_DH = 0.0f;
  float Theta_y_DH = 0.0f;

  // ROS2
  std::shared_ptr<rclcpp::Node> ros_node;
  rclcpp::Publisher<sar_msgs::msg::OpticalFlowData>::SharedPtr opticalflow_publisher;
  rclcpp::Subscription<sar_msgs::msg::CtrlData>::SharedPtr Optical_Flow_Flag_subscriber;
};

Camera_Plugin::Camera_Plugin()
  : System(), dataPtr(std::make_unique<Camera_PluginPrivate>())
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  this->dataPtr->ros_node = std::make_shared<rclcpp::Node>("Opticalflow_Publisher_Node");

  this->dataPtr->opticalflow_publisher =
      this->dataPtr->ros_node->create_publisher<sar_msgs::msg::OpticalFlowData>("Opticalflow/data", 1);

  this->dataPtr->Optical_Flow_Flag_subscriber =
      this->dataPtr->ros_node->create_subscription<sar_msgs::msg::CtrlData>(
          "/CTRL/data", 1,
          std::bind(&Camera_Plugin::Optical_Flow_Flag_Callback, this, std::placeholders::_1));
}

void Camera_Plugin::PublishOpticalFlow()
{
  sar_msgs::msg::OpticalFlowData msg;
  msg.tau = this->dataPtr->Tau_DH;
  msg.theta_x = this->dataPtr->Theta_x_DH;
  msg.theta_y = this->dataPtr->Theta_y_DH;
  this->dataPtr->opticalflow_publisher->publish(msg);
}

void Camera_Plugin::SetDefaultAndPublish()
{
  this->dataPtr->Tau_DH = kDefaultTau;
  this->dataPtr->Theta_x_DH = kDefaultTheta;
  this->dataPtr->Theta_y_DH = kDefaultTheta;
  PublishOpticalFlow();
}

//////////////////////////////////////////////////
void Camera_Plugin::CameraMsg(const gz::msgs::Image &_msg)
{
  auto current_time = std::chrono::high_resolution_clock::now();
  dataPtr->last_time = current_time;

  const int fullW  = static_cast<int>(_msg.width());   // e.g., 160
  const int fullH  = static_cast<int>(_msg.height());  // e.g., 120
  const auto &data = _msg.data();

  if (fullW <= 0 || fullH <= 0) {
    SetDefaultAndPublish();
    return;
  }

  const std::size_t expected = static_cast<std::size_t>(fullW) * static_cast<std::size_t>(fullH);
  if (data.size() < expected) {
    SetDefaultAndPublish();
    return;
  }

  // ---- 중앙 crop 시작점 (OpenMV windowing과 동일: center crop) ----
  int cropW = kCropW;
  int cropH = kCropH;

  // 혹시라도 해상도가 더 작으면 안전하게 축소
  if (cropW > fullW) cropW = fullW;
  if (cropH > fullH) cropH = fullH;

  const int u0 = (fullW - cropW) / 2;  // 160 -> (160-64)/2 = 48
  const int v0 = (fullH - cropH) / 2;  // 120 -> (120-64)/2 = 28

  // ---- cur/prev는 "crop 크기"로만 유지 ----
  if (dataPtr->cur_img.rows() != cropH || dataPtr->cur_img.cols() != cropW) {
    dataPtr->cur_img.resize(cropH, cropW);
    dataPtr->prev_img.resize(cropH, cropW);
    dataPtr->have_prev = false;
  }

  if (dataPtr->have_prev) {
    dataPtr->prev_img.swap(dataPtr->cur_img);
  }

  // ---- 160x120 원본에서 중앙 64x64만 row-by-row copy ----
  const uint8_t *src = reinterpret_cast<const uint8_t*>(data.data());
  uint8_t *dst = dataPtr->cur_img.data();

  for (int r = 0; r < cropH; ++r) {
    const std::size_t src_idx = static_cast<std::size_t>(v0 + r) * static_cast<std::size_t>(fullW)
                              + static_cast<std::size_t>(u0);
    std::memcpy(dst + static_cast<std::size_t>(r) * static_cast<std::size_t>(cropW),
                src + src_idx,
                static_cast<std::size_t>(cropW));
  }

  // 첫 프레임이면 prev가 없어서 계산 불가
  if (!dataPtr->have_prev) {
    dataPtr->have_prev = true;
    SetDefaultAndPublish();
    return;
  }

  if (this->dataPtr->Optical_Flow_Flag) {
    this->OF_Calc_Opt_Sep();   // 이제 "crop된 64x64"만 들어있음
  } else {
    SetDefaultAndPublish();
  }
}

//////////////////////////////////////////////////
// Direct gradient TTC (case2) 계산
// - 여기서는 dataPtr->cur_img/prev_img가 이미 64x64 crop임
void Camera_Plugin::OF_Calc_Opt_Sep()
{
  const int H = dataPtr->cur_img.rows(); // 64
  const int W = dataPtr->cur_img.cols(); // 64

  if (H < 3 || W < 3 || !dataPtr->have_prev) {
    SetDefaultAndPublish();
    return;
  }

  const int O_up = W / 2;
  const int O_vp = H / 2;

  const auto &I = dataPtr->cur_img;
  const auto &P = dataPtr->prev_img;

  double sum_gv2   = 0.0;
  double sum_gu2   = 0.0;
  double sum_gugv  = 0.0;
  double sum_grgv  = 0.0;
  double sum_grgu  = 0.0;
  double sum_gr2   = 0.0;

  double sum_gtgv  = 0.0;
  double sum_gtgu  = 0.0;
  double sum_gtgr  = 0.0;

  for (int v = 1; v < H - 1; ++v) {
    for (int u = 1; u < W - 1; ++u) {

      const double i00 = static_cast<double>(I(v-1, u-1));
      const double i01 = static_cast<double>(I(v-1, u));
      const double i02 = static_cast<double>(I(v-1, u+1));
      const double i10 = static_cast<double>(I(v,   u-1));
      const double i12 = static_cast<double>(I(v,   u+1));
      const double i20 = static_cast<double>(I(v+1, u-1));
      const double i21 = static_cast<double>(I(v+1, u));
      const double i22 = static_cast<double>(I(v+1, u+1));

      // Sobel G_up, G_vp (너 코드와 동치)
      const double G_up = (i02 + 2.0*i12 + i22) - (i00 + 2.0*i10 + i20);
      const double G_vp = (i20 + 2.0*i21 + i22) - (i00 + 2.0*i01 + i02);

      const double G_rp = (2.0*(u - O_up) + 1.0) * G_up + (2.0*(v - O_vp) + 1.0) * G_vp;
      const double G_tp = static_cast<double>(I(v, u)) - static_cast<double>(P(v, u));

      sum_gv2  += G_vp * G_vp;
      sum_gu2  += G_up * G_up;
      sum_gugv += G_up * G_vp;

      sum_grgv += G_rp * G_vp;
      sum_grgu += G_rp * G_up;
      sum_gr2  += G_rp * G_rp;

      sum_gtgv += G_tp * G_vp;
      sum_gtgu += G_tp * G_up;
      sum_gtgr += G_tp * G_rp;
    }
  }

  Eigen::Matrix3d X;
  X(0,0) =  kFocalLen * sum_gv2;
  X(0,1) = -kFocalLen * sum_gugv;
  X(0,2) = -(kPixelW / 2.0) * sum_grgv;

  X(1,0) =  kFocalLen * sum_gugv;
  X(1,1) = -kFocalLen * sum_gu2;
  X(1,2) = -(kPixelW / 2.0) * sum_grgu;

  X(2,0) =  kFocalLen * sum_grgv;
  X(2,1) = -kFocalLen * sum_grgu;
  X(2,2) = -(kPixelW / 2.0) * sum_gr2;

  Eigen::Vector3d y;
  y(0) = sum_gtgv;
  y(1) = sum_gtgu;
  y(2) = sum_gtgr;
  y *= (8.0 * kPixelW / kDeltaT);

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(X, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Vector3d s = svd.singularValues();

  Eigen::Matrix3d S_inv = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; ++i) {
    if (s(i) > kSvdEps) S_inv(i,i) = 1.0 / s(i);
  }

  const Eigen::Matrix3d pinv_X = svd.matrixV() * S_inv * svd.matrixU().transpose();
  const Eigen::Vector3d b = pinv_X * y;

  float Tau  = kDefaultTau;
  float Thx  = static_cast<float>(b(0));
  float Thy  = static_cast<float>(b(1));

  const double denom = b(2);
  if (std::isfinite(denom) && std::abs(denom) > 1e-12) {
    Tau = static_cast<float>(1.0 / denom);
  }

  if (!std::isfinite(Tau)) Tau = kDefaultTau;
  if (!std::isfinite(Thx)) Thx = kDefaultTheta;
  if (!std::isfinite(Thy)) Thy = kDefaultTheta;

  Tau = std::clamp(Tau, -kTauClampAbs,  kTauClampAbs);
  Thx = std::clamp(Thx, -kThetaClampAbs, kThetaClampAbs);
  Thy = std::clamp(Thy, -kThetaClampAbs, kThetaClampAbs);

  this->dataPtr->Tau_DH = Tau;
  this->dataPtr->Theta_x_DH = Thx;
  this->dataPtr->Theta_y_DH = Thy;

  PublishOpticalFlow();
}

//////////////////////////////////////////////////
void Camera_Plugin::Configure(const Entity &,
                             const std::shared_ptr<const sdf::Element> &,
                             EntityComponentManager &,
                             EventManager &)
{
  dataPtr->cur_img.resize(0, 0);
  dataPtr->prev_img.resize(0, 0);
  dataPtr->have_prev = false;
  dataPtr->last_time = std::chrono::high_resolution_clock::now();

  this->node.Subscribe("/camera", &Camera_Plugin::CameraMsg, this);
}

void Camera_Plugin::PreUpdate(const UpdateInfo &,
                              EntityComponentManager &)
{
  GZ_PROFILE("Camera_Plugin::PreUpdate");
  rclcpp::spin_some(this->dataPtr->ros_node);
}

void Camera_Plugin::PostUpdate(const UpdateInfo &,
                               const EntityComponentManager &)
{
  GZ_PROFILE("Camera_Plugin::PostUpdate");
}

void Camera_Plugin::Optical_Flow_Flag_Callback(const sar_msgs::msg::CtrlData::SharedPtr msg)
{
  this->dataPtr->Optical_Flow_Flag = msg->optical_flow_flag;
}

GZ_ADD_PLUGIN(Camera_Plugin,
              System,
              Camera_Plugin::ISystemConfigure,
              Camera_Plugin::ISystemPreUpdate,
              Camera_Plugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(Camera_Plugin, "gz::sim::systems::Camera_Plugin")