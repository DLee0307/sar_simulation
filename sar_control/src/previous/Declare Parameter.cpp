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


    // Declare and Get INERTIAL PARAMETERS from Model_Types.yaml
    double Ref_Mass = get_and_declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Mass", "Ref_Mass");
    double Ref_Ixx = get_and_declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Ixx", "Ref_Ixx");
    double Ref_Iyy = get_and_declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Iyy", "Ref_Iyy");
    double Ref_Izz = get_and_declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Izz", "Ref_Izz");

    // Print INERTIAL PARAMETERS to verify from Model_Types.yaml
    RCLCPP_INFO(this->get_logger(), "Ref_Mass: %f", Ref_Mass);
    RCLCPP_INFO(this->get_logger(), "Ref_Ixx: %f", Ref_Ixx);
    RCLCPP_INFO(this->get_logger(), "Ref_Iyy: %f", Ref_Iyy);
    RCLCPP_INFO(this->get_logger(), "Ref_Izz: %f", Ref_Izz);


    // Declare and Get SYSTEM PARAMETERS from Model_Types.yaml
    double Thrust_max = get_and_declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Thrust_max", "Thrust_max");
    double C_tf = get_and_declare_parameter("SAR_Type." + SAR_Type + ".System_Params.C_tf", "C_tf");

    // Print SYSTEM PARAMETERS to verify from Model_Types.yaml
    RCLCPP_INFO(this->get_logger(), "Thrust_max: %f", Thrust_max);
    RCLCPP_INFO(this->get_logger(), "C_tf: %f", C_tf);


    // Declare and Get GEOMETRIC PARAMETERS from Model_Types.yaml
    double Forward_Reach = get_and_declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Forward_Reach", "Forward_Reach");
    double L_eff = get_and_declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".L_eff", "L_eff");

    // Print GEOMETRIC PARAMETERS to verify from Model_Types.yaml
    RCLCPP_INFO(this->get_logger(), "Forward_Reach: %f", Forward_Reach);
    RCLCPP_INFO(this->get_logger(), "L_eff: %f", L_eff);
    

    // Declare and Get PROP DISTANCES from Model_Types.yaml
    std::vector<double> Prop_Front_Vec = get_and_declare_parameter_array("SAR_Type." + SAR_Type + ".System_Params.Prop_Front", "Prop_Front");
    double Prop_14_x = Prop_Front_Vec[0];
    double Prop_14_y = Prop_Front_Vec[1];
    std::vector<double> Prop_Rear_Vec = get_and_declare_parameter_array("SAR_Type." + SAR_Type + ".System_Params.Prop_Rear", "Prop_Rear");
    double Prop_23_x = Prop_Rear_Vec[0];
    double Prop_23_y = Prop_Rear_Vec[1];    
    
    // Print PROP DISTANCES to verify from Model_Types.yaml
    RCLCPP_INFO(this->get_logger(), "Prop_Front_Vec: [%f, %f]", Prop_Front_Vec[0], Prop_Front_Vec[1]);
    RCLCPP_INFO(this->get_logger(), "Prop_Rear_Vec: [%f, %f]", Prop_Rear_Vec[0], Prop_Rear_Vec[1]);


    // Declare and Get UPDATE CTRL GAINS from Model_Types.yaml
    double P_kp_xy = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kp_xy", "P_kp_xy");
    double P_kd_xy = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kd_xy", "P_kd_xy");
    double P_ki_xy = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_ki_xy", "P_ki_xy");
    double i_range_xy = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_xy", "i_range_xy");

    double P_kp_z = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kp_z", "P_kp_z");
    double P_kd_z = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_kd_z", "P_kd_z");
    double P_ki_z = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.P_ki_z", "P_ki_z");
    double i_range_z = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_z", "i_range_z");

    double R_kp_xy = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kp_xy", "R_kp_xy");
    double R_kd_xy = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kd_xy", "R_kd_xy");
    double R_ki_xy = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_ki_xy", "R_ki_xy");
    double i_range_R_xy = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_R_xy", "i_range_R_xy");

    double R_kp_z = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kp_z", "R_kp_z");
    double R_kd_z = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_kd_z", "R_kd_z");
    double R_ki_z = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.R_ki_z", "R_ki_z");
    double i_range_R_z = get_and_declare_parameter("SAR_Type." + SAR_Type + ".CtrlGains.i_range_R_z", "i_range_R_z");

    // Print UPDATE CTRL GAINS to verify from Model_Types.yaml
    RCLCPP_INFO(this->get_logger(), "P_kp_xy: %f", P_kp_xy);
    RCLCPP_INFO(this->get_logger(), "P_kd_xy: %f", P_kd_xy);
    RCLCPP_INFO(this->get_logger(), "P_ki_xy: %f", P_ki_xy);
    RCLCPP_INFO(this->get_logger(), "i_range_xy: %f", i_range_xy);

    RCLCPP_INFO(this->get_logger(), "P_kp_z: %f", P_kp_z);
    RCLCPP_INFO(this->get_logger(), "P_kd_z: %f", P_kd_z);
    RCLCPP_INFO(this->get_logger(), "P_ki_z: %f", P_ki_z);
    RCLCPP_INFO(this->get_logger(), "i_range_z: %f", i_range_z);

    RCLCPP_INFO(this->get_logger(), "R_kp_xy: %f", R_kp_xy);
    RCLCPP_INFO(this->get_logger(), "R_kd_xy: %f", R_kd_xy);
    RCLCPP_INFO(this->get_logger(), "R_ki_xy: %f", R_ki_xy);
    RCLCPP_INFO(this->get_logger(), "i_range_R_xy: %f", i_range_R_xy);

    RCLCPP_INFO(this->get_logger(), "R_kp_z: %f", R_kp_z);
    RCLCPP_INFO(this->get_logger(), "R_kd_z: %f", R_kd_z);
    RCLCPP_INFO(this->get_logger(), "R_ki_z: %f", R_ki_z);
    RCLCPP_INFO(this->get_logger(), "i_range_R_z: %f", i_range_R_z);
    }

private:
  double get_and_declare_parameter(const std::string &full_name, const std::string &simple_name)
  {
    double value = this->declare_parameter(full_name, 0.0);
    this->declare_parameter(simple_name, value);
    return value;
  }

  std::vector<double> get_and_declare_parameter_array(const std::string &full_name, const std::string &simple_name)
  {
    std::vector<double> value = this->declare_parameter(full_name, std::vector<double>{});
    this->declare_parameter(simple_name, value);
    return value;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimSettingsNode>());
  rclcpp::shutdown();
  return 0;
}