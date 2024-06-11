#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include <vector>
// ros2 launch sar_launch sim_settings_launch.py params_file:=/home/dlee/ros2_ws/src/sar_simulation/sar_config/Sim_Settings.yaml
// ros2 launch sar_launch sim_settings_launch.py sim_settings_file:=/home/dlee/ros2_ws/src/sar_simulation/sar_config/Sim_Settings.yaml model_types_file:=/home/dlee/ros2_ws/src/sar_simulation/sar_config/Model_Types.yaml

class SimSettingsNode : public rclcpp::Node
{
public:
  SimSettingsNode()
  : Node("sim_settings_node")
  {
    // Declare PARAMETERS from Model_Types.yaml
    // Get PARAMETERS from Model_Types.yaml
    // Print PARAMETERS to verify from Model_Types.yaml

    // Declare SAR PARAMETERS from Sim_Settings.yaml
    this->declare_parameter("DATA_TYPE", "None");
    this->declare_parameter("SAR_SETTINGS.SAR_Type", "None");
    this->declare_parameter("SAR_SETTINGS.SAR_Config", "None");

    // Get SAR PARAMETERS from Sim_Settings.yaml
    std::string DATA_TYPE = this->get_parameter("DATA_TYPE").as_string();
    std::string SAR_Type = this->get_parameter("SAR_SETTINGS.SAR_Type").as_string();
    std::string SAR_Config = this->get_parameter("SAR_SETTINGS.SAR_Config").as_string();

    // Print SAR PARAMETERS to verify from Sim_Settings.yaml
    RCLCPP_INFO(this->get_logger(), "DATA_TYPE: %s", DATA_TYPE.c_str());
    RCLCPP_INFO(this->get_logger(), "SAR_Type: %s", SAR_Type.c_str());
    RCLCPP_INFO(this->get_logger(), "SAR_Config: %s", SAR_Config.c_str());

    // Declare INERTIAL PARAMETERS from Model_Types.yaml
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Mass", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Ixx", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Iyy", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Izz", 0.0);

    // Get INERTIAL PARAMETERS from Model_Types.yaml
    double Ref_Mass = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Mass").as_double();
    double Ref_Ixx = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Ixx").as_double();
    double Ref_Iyy = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Iyy").as_double();
    double Ref_Izz = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Izz").as_double();

    // Print INERTIAL PARAMETERS to verify from Model_Types.yaml
    //RCLCPP_INFO(this->get_logger(), "Ref_Mass: %f", Ref_Mass);
    //RCLCPP_INFO(this->get_logger(), "Ref_Ixx: %f", Ref_Ixx);
    //RCLCPP_INFO(this->get_logger(), "Ref_Iyy: %f", Ref_Iyy);
    //RCLCPP_INFO(this->get_logger(), "Ref_Izz: %f", Ref_Izz);

    // Declare SYSTEM PARAMETERS from Model_Types.yaml
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Thrust_max", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.C_tf", 0.0);

    // Get SYSTEM PARAMETERS from Model_Types.yaml
    double Thrust_max = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Thrust_max").as_double();
    double C_tf = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.C_tf").as_double();

    // Print SYSTEM PARAMETERS to verify from Model_Types.yaml
    //RCLCPP_INFO(this->get_logger(), "Thrust_max: %f", Thrust_max);
    //RCLCPP_INFO(this->get_logger(), "C_tf: %f", C_tf);

    // Declare GEOMETRIC PARAMETERS from Model_Types.yaml
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Forward_Reach", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".L_eff", 0.0);

    // Get GEOMETRIC PARAMETERS from Model_Types.yaml
    double Forward_Reach = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Forward_Reach").as_double();
    double L_eff = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".L_eff").as_double();

    // Print GEOMETRIC PARAMETERS to verify from Model_Types.yaml
    //RCLCPP_INFO(this->get_logger(), "Forward_Reach: %f", Forward_Reach);
    //RCLCPP_INFO(this->get_logger(), "L_eff: %f", L_eff);
    
    // Declare PROP DISTANCES from Model_Types.yaml
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Prop_Front", std::vector<double>{});
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Prop_Rear", std::vector<double>{});

    // Get PROP DISTANCES from Model_Types.yaml
    std::vector<double> Prop_Front_Vec = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Prop_Front").as_double_array();
    double Prop_14_x = Prop_Front_Vec[0];
    double Prop_14_y = Prop_Front_Vec[1];
    std::vector<double> Prop_Rear_Vec = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Prop_Rear").as_double_array();
    double Prop_23_x = Prop_Rear_Vec[0];
    double Prop_23_y = Prop_Rear_Vec[1];    
    
    // Print PROP DISTANCES to verify from Model_Types.yaml
    //RCLCPP_INFO(this->get_logger(), "Prop_Front_Vec: [%f, %f]", Prop_Front_Vec[0], Prop_Front_Vec[1]);
    //RCLCPP_INFO(this->get_logger(), "Prop_Rear_Vec: [%f, %f]", Prop_Rear_Vec[0], Prop_Rear_Vec[1]);

    // Declare UPDATE CTRL GAINS from Model_Types.yaml
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kp_xy", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kd_xy", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_ki_xy", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_xy", 0.0);

    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kp_z", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kd_z", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_ki_z", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_z", 0.0);

    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kp_xy", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kd_xy", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_ki_xy", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_R_xy", 0.0);

    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kp_z", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kd_z", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_ki_z", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_R_z", 0.0);

    // Get UPDATE CTRL GAINS from Model_Types.yaml
    double P_kp_xy = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kp_xy").as_double();
    double P_kd_xy = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kd_xy").as_double();
    double P_ki_xy = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_ki_xy").as_double();
    double i_range_xy = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_xy").as_double();

    double P_kp_z = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kp_z").as_double();
    double P_kd_z = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kd_z").as_double();
    double P_ki_z = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_ki_z").as_double();
    double i_range_z = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_z").as_double();

    double R_kp_xy = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kp_xy").as_double();
    double R_kd_xy = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kd_xy").as_double();
    double R_ki_xy = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_ki_xy").as_double();
    double i_range_R_xy = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_R_xy").as_double();

    double R_kp_z = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kp_z").as_double();
    double R_kd_z = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kd_z").as_double();
    double R_ki_z = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_ki_z").as_double();
    double i_range_R_z = this->get_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_R_z").as_double();

    // Print UPDATE CTRL GAINS to verify from Model_Types.yaml
    //RCLCPP_INFO(this->get_logger(), "P_kp_xy: %f", P_kp_xy);
    //RCLCPP_INFO(this->get_logger(), "P_kd_xy: %f", P_kd_xy);
    //RCLCPP_INFO(this->get_logger(), "P_ki_xy: %f", P_ki_xy);
    //RCLCPP_INFO(this->get_logger(), "i_range_xy: %f", i_range_xy);

    //RCLCPP_INFO(this->get_logger(), "P_kp_z: %f", P_kp_z);
    //RCLCPP_INFO(this->get_logger(), "P_kd_z: %f", P_kd_z);
    //RCLCPP_INFO(this->get_logger(), "P_ki_z: %f", P_ki_z);
    //RCLCPP_INFO(this->get_logger(), "i_range_z: %f", i_range_z);

    //RCLCPP_INFO(this->get_logger(), "R_kp_xy: %f", R_kp_xy);
    //RCLCPP_INFO(this->get_logger(), "R_kd_xy: %f", R_kd_xy);
    //RCLCPP_INFO(this->get_logger(), "R_ki_xy: %f", R_ki_xy);
    //RCLCPP_INFO(this->get_logger(), "i_range_R_xy: %f", i_range_R_xy);

    //RCLCPP_INFO(this->get_logger(), "R_kp_z: %f", R_kp_z);
    //RCLCPP_INFO(this->get_logger(), "R_kd_z: %f", R_kd_z);
    //RCLCPP_INFO(this->get_logger(), "R_ki_z: %f", R_ki_z);
    //RCLCPP_INFO(this->get_logger(), "i_range_R_z: %f", i_range_R_z);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimSettingsNode>());
  rclcpp::shutdown();
  return 0;
}