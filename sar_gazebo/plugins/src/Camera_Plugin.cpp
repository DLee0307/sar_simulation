#include "Camera_Plugin.h"


using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::Camera_PluginPrivate
{
public:
  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> cur_img;
  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> prev_img;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_time;

  ////!! ROS2 Publisher
  public: std::shared_ptr<rclcpp::Node> ros_node;
  public: rclcpp::Publisher<sar_msgs::msg::OpticalFlowData>::SharedPtr opticalflow_publisher;

};

Camera_Plugin::Camera_Plugin() : System(), dataPtr(std::make_unique<Camera_PluginPrivate>())
{

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  this->dataPtr->ros_node = std::make_shared<rclcpp::Node>("Opticalflow_Publisher_Node");
  this->dataPtr->opticalflow_publisher = this->dataPtr->ros_node->create_publisher<sar_msgs::msg::OpticalFlowData>("Opticalflow/data", 1);

}

//////////////////////////////////////////////////
void Camera_Plugin::CameraMsg(const gz::msgs::Image &_msg)
{
  auto current_time = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - dataPtr->last_time).count();
  //std::cout << "Time between cycles: " << duration << "ms" << std::endl;

  dataPtr->last_time = current_time;

  auto width = _msg.width();
  auto height = _msg.height();
  const auto &data = _msg.data();

  if (dataPtr->cur_img.size() > 0)
  {
    dataPtr->prev_img = dataPtr->cur_img;
  }

  dataPtr->cur_img.resize(height, width);

  for (size_t y = 0; y < height; ++y)
  {
    for (size_t x = 0; x < width; ++x)
    {
        size_t index = y * width + x;
        dataPtr->cur_img(y, x) = data[index];
    }
  }
  this->OF_Calc_Opt_Sep();
}


void Camera_Plugin::OF_Calc_Opt_Sep()
{
    int I_0 = 255; //Brightness value (0-255)
    float FoV = 82.22; //Field of View [deg]

    float w = 3.6e-6;
    float f = 0.033e-3;
    float delta_t = 0.01;

    int HEIGHT_PIXELS = 16;
    int WIDTH_PIXELS = 16;
    int O_up = WIDTH_PIXELS / 2;
    int O_vp = HEIGHT_PIXELS / 2;

    Eigen::MatrixXd cur_img = Eigen::MatrixXd::Zero(HEIGHT_PIXELS, WIDTH_PIXELS);
    Eigen::MatrixXd prev_img = Eigen::MatrixXd::Zero(HEIGHT_PIXELS, WIDTH_PIXELS);

    if (dataPtr->cur_img.size() > 0) {
        cur_img = dataPtr->cur_img.cast<double>();
    }

    if (dataPtr->prev_img.size() > 0) {
        prev_img = dataPtr->prev_img.cast<double>();
    }

    Eigen::Matrix<double, 1, 3> Ku_1;
    Ku_1 << -1, 0, 1;
    Eigen::Matrix<double, 3, 1> Ku_2;
    Ku_2 << 1, 2, 1;

    Eigen::Matrix<double, 1, 3> Kv_1;
    Kv_1 << 1, 2, 1;
    Eigen::Matrix<double, 3, 1> Kv_2;
    Kv_2 << -1, 0, 1;

    Eigen::MatrixXd G_up = Eigen::MatrixXd::Zero(HEIGHT_PIXELS, WIDTH_PIXELS);
    Eigen::MatrixXd G_vp = Eigen::MatrixXd::Zero(HEIGHT_PIXELS, WIDTH_PIXELS);
    Eigen::MatrixXd G_rp = Eigen::MatrixXd::Zero(HEIGHT_PIXELS, WIDTH_PIXELS);
    Eigen::MatrixXd G_tp = Eigen::MatrixXd::Zero(HEIGHT_PIXELS, WIDTH_PIXELS);

    // Calculate image gradients
    for (int v_p = 1; v_p < HEIGHT_PIXELS - 1; ++v_p)
    {
        for (int u_p = 1; u_p < WIDTH_PIXELS - 1; ++u_p)
        {
            Eigen::MatrixXd patch = cur_img.block<3, 3>(v_p - 1, u_p - 1);

            G_vp(v_p, u_p) = (Ku_1 * patch).dot(Ku_2);
            G_up(v_p, u_p) = (Kv_1 * patch).dot(Kv_2);

            G_rp(v_p, u_p) = (2 * (u_p - O_up) + 1) * G_up(v_p, u_p) + (2 * (v_p - O_vp) + 1) * G_vp(v_p, u_p);
            G_tp(v_p, u_p) = cur_img(v_p, u_p) - prev_img(v_p, u_p);
        }
    }

    Eigen::MatrixXd X(3, 3);
    X(0, 0) = f * G_vp.array().square().sum();
    X(0, 1) = -f * (G_up.array() * G_vp.array()).sum();
    X(0, 2) = -(w / 2) * (G_rp.array() * G_vp.array()).sum();

    X(1, 0) = f * (G_vp.array() * G_up.array()).sum();
    X(1, 1) = -f * G_up.array().square().sum();
    X(1, 2) = -(w / 2) * (G_rp.array() * G_up.array()).sum();

    X(2, 0) = f * (G_vp.array() * G_rp.array()).sum();
    X(2, 1) = -f * (G_up.array() * G_rp.array()).sum();
    X(2, 2) = -(w / 2) * G_rp.array().square().sum();

    Eigen::Vector3d y;
    y(0) = (G_tp.array() * G_vp.array()).sum();
    y(1) = (G_tp.array() * G_up.array()).sum();
    y(2) = (G_tp.array() * G_rp.array()).sum();
    y *= (8 * w / delta_t);

    // Use JacobiSVD to calculate pseudo-inverse manually
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd S = svd.singularValues().asDiagonal();
    Eigen::MatrixXd S_inv = S;
    for (int i = 0; i < S.rows(); ++i) {
        if (S(i, i) > 1e-6) {
            S_inv(i, i) = 1.0 / S(i, i);  // Inverse of non-zero singular values
        } else {
            S_inv(i, i) = 0;  // Zero out small singular values to avoid instability
        }
    }

    Eigen::MatrixXd pinv_X = svd.matrixV() * S_inv * svd.matrixU().transpose();  // Pseudo-inverse calculation

    Eigen::Vector3d b = pinv_X * y;

    //std::cout << "Solution vector b: " << b.transpose() << std::endl;
    std::cout << "Solution vector b: " << 1/b[2] << std::endl;

    sar_msgs::msg::OpticalFlowData msg;

    msg.tau = 1/b[2];

    this->dataPtr->opticalflow_publisher->publish(msg);

}


void Camera_Plugin::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm, EventManager &)
{
  dataPtr->cur_img = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>();
  dataPtr->prev_img = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>();
  dataPtr->last_time = std::chrono::high_resolution_clock::now();
  
  this->node.Subscribe("/camera", &Camera_Plugin::CameraMsg, this);

  //this->OF_Calc_Opt_Sep();


}

void Camera_Plugin::PreUpdate(const UpdateInfo &_info, 
                                EntityComponentManager &_ecm)
{
  GZ_PROFILE("Camera_Plugin::PreUpdate");
  
  
}

//////////////////////////////////////////////////
void Camera_Plugin::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Camera_Plugin::PostUpdate");
  
}



GZ_ADD_PLUGIN(Camera_Plugin,
                    System,
                    Camera_Plugin::ISystemConfigure,
                    Camera_Plugin::ISystemPreUpdate,
                    //Camera_Plugin::ISystemUpdate,
                    Camera_Plugin::ISystemPostUpdate)
                    

GZ_ADD_PLUGIN_ALIAS(Camera_Plugin, "gz::sim::systems::Camera_Plugin")
