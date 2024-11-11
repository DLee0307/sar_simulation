#include "stabilizer.h"


// ros2 launch sar_launch set_parameter_launch.py sim_settings_file:=/home/dlee/ros2_ws/src/sar_simulation/sar_config/Sim_Settings.yaml model_types_file:=/home/dlee/ros2_ws/src/sar_simulation/sar_config/Model_Types.yaml
Controller::Controller()
: Node("SAR_Controller")
{

    // ===========================
    //     ROS TOPICS/SERVICES
    // ===========================

    // EXTERNAL SENSOR SUBSCRIBERS
    subscriber_Vicon = this->create_subscription<sar_msgs::msg::ViconData>("Vicon/data", 1, std::bind(&Controller::Vicon_Update_Callback, this, std::placeholders::_1));
    
    // INTERNAL SENSOR SUBSCRIBERS
    subscriber_IMU = this->create_subscription<sar_msgs::msg::IMUData>("Imu/data", 1, std::bind(&Controller::IMU_Update_Callback, this, std::placeholders::_1));

    // OpticalFlow SUBSCRIBERS
    subscriber_OpticalFlow = this->create_subscription<sar_msgs::msg::OpticalFlowData>("Opticalflow/data", 1, std::bind(&Controller::OpticalFlow_Update_Callback, this, std::placeholders::_1));

    // MISC SERVICES/PUBLISHERS
    CTRL_Data_Publisher = this->create_publisher<sar_msgs::msg::CtrlData>("/CTRL/data", 1);
    CTRL_Debug_Publisher = this->create_publisher<sar_msgs::msg::CtrlDebug>("/CTRL/debug", 1);

    CMD_Output_Service = this->create_service<sar_msgs::srv::CTRLCmdSrv>("/CTRL/Cmd_ctrl", std::bind(&Controller::CMD_Service_Resp, this, std::placeholders::_1, std::placeholders::_2));
    Get_Obs_Service = this->create_service<sar_msgs::srv::CTRLGetObs>("/CTRL/Get_Obs", std::bind(&Controller::Get_Obs_Resp, this, std::placeholders::_1, std::placeholders::_2));

    // ROS2 PARAMETERS PUBLISHERS
    ROS_Parmas_Publisher = this->create_publisher<sar_msgs::msg::ROSParams>("/ROS2/PARAMETER", 1);


    // Thread main controller loop so other callbacks can work fine
    appThread = std::thread(&Controller::appLoop, this);
    controllerThread = std::thread(&Controller::stabilizerLoop, this);
}
/*
void Controller::SAR_DC_ExtImpact_Callback(sar_msgs::SAR_ImpactData::ConstPtr const &msg)
{
    Impact_Flag_Ext = msg->Impact_Flag_Ext;
}

bool Controller::Get_Obs_Resp(sar_msgs::CTRL_Get_Obs::Request &req, sar_msgs::CTRL_Get_Obs::Response &res)
{
    res.Tick = tick;
    res.Tau = Tau;
    res.Tau_CR = Tau_CR;
    res.Theta_x = Theta_x;
    res.D_perp = D_perp;
    res.D_perp_CR = D_perp_CR;
    res.Plane_Angle_deg = Plane_Angle_deg;

    return 1;
}
*/

//From : SAR_DataConverter.h TO : CTRL_Command(&CTRL_Cmd);
void Controller::CMD_Service_Resp(const sar_msgs::srv::CTRLCmdSrv::Request::SharedPtr request,
                     sar_msgs::srv::CTRLCmdSrv::Response::SharedPtr response) {
    std::cout << "service is requested in controller : " <<  std::endl;
    
    // RESPOND THE SRV WAS RECIEVED
    response->srv_success = true;

    // UPDATE CTRL_Cmd STRUCT VALUES
    CTRL_Cmd.cmd_type = request->cmd_type;
    CTRL_Cmd.cmd_val1 = request->cmd_vals.x;
    CTRL_Cmd.cmd_val2 = request->cmd_vals.y;
    CTRL_Cmd.cmd_val3 = request->cmd_vals.z;
    CTRL_Cmd.cmd_flag = request->cmd_flag;
    CTRL_Cmd.cmd_rx = request->cmd_rx;
/**/
    std::cout << request->cmd_type <<  std::endl;                    
    std::cout << "cmd_type: " << request->cmd_type <<  std::endl;
    std::cout << "cmd_val1: " << request->cmd_vals.x <<  std::endl;
    std::cout << "cmd_val2: " << request->cmd_vals.y <<  std::endl;
    std::cout << "cmd_val3: " << request->cmd_vals.z <<  std::endl;
    std::cout << "cmd_flag: " << request->cmd_flag <<  std::endl;
    std::cout << "cmd_rx: " << request->cmd_rx <<  std::endl;


}

void Controller::Get_Obs_Resp(const sar_msgs::srv::CTRLGetObs::Request::SharedPtr request,
                     sar_msgs::srv::CTRLGetObs::Response::SharedPtr response) {

    response->tick = tick;
    response->tau = Tau;
    response->tau_cr = Tau_CR;
    response->theta_x = Theta_x;
    response->d_perp = D_perp;
    response->d_perp_cr = D_perp_CR;
    response->plane_angle_deg = Plane_Angle_deg;

}

void Controller::Vicon_Update_Callback(const sar_msgs::msg::ViconData::SharedPtr msg) {
    Vicon_Data = msg;

    // UPDATE POSE FROM VICON SYSTEM
    state.position.x = msg->pose.position.x;
    state.position.y = msg->pose.position.y;
    state.position.z = msg->pose.position.z;

    // UPDATE VELOCITIES FROM VICON SYSTEM
    state.velocity.x = msg->vel.x;
    state.velocity.y = msg->vel.y;
    state.velocity.z = msg->vel.z;

}

void Controller::IMU_Update_Callback(const sar_msgs::msg::IMUData::SharedPtr msg) {
    Imu_Data = msg;

    // Convert to Gs to match crazyflie sensors
    sensorData.acc.x = -msg->linear_acceleration.x/9.8066;
    sensorData.acc.y = -msg->linear_acceleration.y/9.8066;
    sensorData.acc.z = msg->linear_acceleration.z/9.8066;

    // Convert to deg/s to match crazyflie sensors
    sensorData.gyro.x = msg->angular_velocity.x*180.0/M_PI;
    sensorData.gyro.y = msg->angular_velocity.y*180.0/M_PI;
    sensorData.gyro.z = msg->angular_velocity.z*180.0/M_PI;

    state.attitudeQuaternion.x = msg->orientation.x;
    state.attitudeQuaternion.y = msg->orientation.y;
    state.attitudeQuaternion.z = msg->orientation.z;
    state.attitudeQuaternion.w = msg->orientation.w;

    // Convert to Gs to match crazyflie sensors
    state.acc.x = -msg->linear_acceleration.x/9.8066;
    state.acc.y = -msg->linear_acceleration.y/9.8066;
    state.acc.z = msg->linear_acceleration.z/9.8066;

    //std::cout << "orientation.x: " << Imu_Data->orientation.x << std::endl;
    /*std::cout << "orientation(x) : " << msg->orientation.x << std::endl;
    //std::cout << "orientation(y) : " << msg->orientation.y << std::endl;
    std::cout << "orientation(z) : " << msg->orientation.z << std::endl;
    std::cout << "orientation(w) : " << msg->orientation.w << std::endl;
    std::cout << "angular_velocity(x) : " << msg->angular_velocity.x << std::endl;
    std::cout << "angular_velocity(y) : " << msg->angular_velocity.y << std::endl;
    std::cout << "angular_velocity(z) : " << msg->angular_velocity.z << std::endl;
    std::cout << "linear_acceleration(x) : " << msg->linear_acceleration.x << std::endl;
    std::cout << "linear_acceleration(y) : " << msg->linear_acceleration.y << std::endl;
    std::cout << "linear_acceleration(z) : " << msg->linear_acceleration.z << std::endl;*/
}

void Controller::OpticalFlow_Update_Callback(const sar_msgs::msg::OpticalFlowData::SharedPtr msg) {
    OpticalFlow_Data = msg;
    Tau_DH = msg->tau;
    Tau_DIFF = Tau - Tau_DH;
    double Tau_difference2 = Tau_CR - Tau_DH;
    //!!!std::cout << "Tau - Tau_DH : " << Tau_DIFF << std::endl;
    //std::cout << "Tau_CR - Tau_DH : " << Tau_difference2 << std::endl;

    //!!!std::cout << "Tau_DH : " << Tau_DH << std::endl;
    //!!!std::cout << "Tau : " << Tau << std::endl;
    //std::cout << "Tau_CR : " << Tau_CR << std::endl;

}

// LOAD VALUES FROM ROSPARAM SERVER INTO CONTROLLER
void Controller::loadInitParams()
{
    printf("Updating Parameters\n");
    
    // Declare PARAMETERS from Sim_Settings.yaml
    this->declare_parameter("DATA_TYPE", "None");
    this->declare_parameter("SAR_SETTINGS.SAR_Type", "None");
    this->declare_parameter("SAR_SETTINGS.SAR_Config", "None");
    this->declare_parameter("SAR_SETTINGS.Policy_Type", "None");
   
    this->declare_parameter("CAM_SETTINGS.Cam_Config", "None");
    this->declare_parameter("CAM_SETTINGS.Cam_Active", false);
    
    this->declare_parameter("SIM_SETTINGS.Vicon_Delay", 0);
    //Ext_Position_msgBuffer.set_capacity(Vicon_Delay_ms);

    this->declare_parameter("PLANE_SETTINGS.Plane_Type", "None");
    this->declare_parameter("PLANE_SETTINGS.Plane_Config", "None");
    this->declare_parameter("PLANE_SETTINGS.Pos_X_init", 0.0);
    this->declare_parameter("PLANE_SETTINGS.Pos_Y_init", 0.0);
    this->declare_parameter("PLANE_SETTINGS.Pos_Z_init", 0.0);
    this->declare_parameter("PLANE_SETTINGS.Plane_Angle_init", 0.0);

    this->declare_parameter("SIM_SETTINGS.Sim_Speed", 0.0);
    this->declare_parameter("SIM_SETTINGS.Sim_Slowdown_Speed", 0.0);
    this->declare_parameter("SIM_SETTINGS.Landing_Slowdown_Flag", true);

    this->declare_parameter("SAR_DC_SETTINGS.Logging_Rate", 0);
    this->declare_parameter("SAR_DC_SETTINGS.Console_Output", true);

    // Load parameters from YAML file
    loadParametersFromSim_SettingsFile("/home/dlee/ros2_ws/src/sar_simulation/sar_config/Sim_Settings.yaml");
    

    // Get SAR PARAMETERS from loaded parameters
    DATA_TYPE = this->get_parameter("DATA_TYPE").as_string();
    SAR_Type = this->get_parameter("SAR_SETTINGS.SAR_Type").as_string();
    SAR_Config = this->get_parameter("SAR_SETTINGS.SAR_Config").as_string();
    POLICY_TYPE_STR = this->get_parameter("SAR_SETTINGS.Policy_Type").as_string();

    Cam_Config = this->get_parameter("CAM_SETTINGS.Cam_Config").as_string();
    CamActive_Flag = this->get_parameter("CAM_SETTINGS.Cam_Active").as_bool();

    POLICY_TYPE = this->get_parameter("SAR_SETTINGS.Policy_Type").as_string();
    Vicon_Delay_ms = this->get_parameter("SIM_SETTINGS.Vicon_Delay").as_int();
    //Ext_Position_msgBuffer.set_capacity(Vicon_Delay_ms);

    Plane_Type = this->get_parameter("PLANE_SETTINGS.Plane_Type").as_string();
    Plane_Config = this->get_parameter("PLANE_SETTINGS.Plane_Config").as_string();
    r_P_O.x = this->get_parameter("PLANE_SETTINGS.Pos_X_init").as_double();
    r_P_O.y = this->get_parameter("PLANE_SETTINGS.Pos_Y_init").as_double();
    r_P_O.z = this->get_parameter("PLANE_SETTINGS.Pos_Z_init").as_double();
    Plane_Angle_deg = this->get_parameter("PLANE_SETTINGS.Plane_Angle_init").as_double();

    SIM_SPEED = this->get_parameter("SIM_SETTINGS.Sim_Speed").as_double();
    SIM_SLOWDOWN_SPEED = this->get_parameter("SIM_SETTINGS.Sim_Slowdown_Speed").as_double();
    LANDING_SLOWDOWN_FLAG = this->get_parameter("SIM_SETTINGS.Landing_Slowdown_Flag").as_bool();

    LOGGING_RATE = this->get_parameter("SAR_DC_SETTINGS.Logging_Rate").as_int();
    SHOW_CONSOLE = this->get_parameter("SAR_DC_SETTINGS.Console_Output").as_bool();

/*
    // Print SAR PARAMETERS to verify from Sim_Settings.yaml
    RCLCPP_INFO(this->get_logger(), "DATA_TYPE: %s", DATA_TYPE.c_str());
    RCLCPP_INFO(this->get_logger(), "SAR_Type: %s", SAR_Type.c_str());
    RCLCPP_INFO(this->get_logger(), "SAR_Config: %s", SAR_Config.c_str());
    RCLCPP_INFO(this->get_logger(), "POLICY_TYPE_STR: %s", POLICY_TYPE_STR.c_str());

    RCLCPP_INFO(this->get_logger(), "Cam_Config: %s", Cam_Config.c_str());
    RCLCPP_INFO(this->get_logger(), "Cam_Active: %d", Cam_Active);

    RCLCPP_INFO(this->get_logger(), "POLICY_TYPE: %s", POLICY_TYPE.c_str());
    RCLCPP_INFO(this->get_logger(), "Vicon_Delay_ms: %d", Vicon_Delay_ms);

    RCLCPP_INFO(this->get_logger(), "Plane_Type: %s", Plane_Type.c_str());
    RCLCPP_INFO(this->get_logger(), "Plane_Config: %s", Plane_Config.c_str());
    RCLCPP_INFO(this->get_logger(), "Plane_Angle_deg: %d", Plane_Angle_deg);
    RCLCPP_INFO(this->get_logger(), "r_P_O.x: %f", r_P_O.x);
    RCLCPP_INFO(this->get_logger(), "r_P_O.y: %f", r_P_O.y);
    RCLCPP_INFO(this->get_logger(), "r_P_O.z: %f", r_P_O.z);

    RCLCPP_INFO(this->get_logger(), "SIM_SPEED: %f", SIM_SPEED);
    RCLCPP_INFO(this->get_logger(), "SIM_SLOWDOWN_SPEED: %f", SIM_SLOWDOWN_SPEED);
    RCLCPP_INFO(this->get_logger(), "LANDING_SLOWDOWN_FLAG: %d", LANDING_SLOWDOWN_FLAG);

    RCLCPP_INFO(this->get_logger(), "LOGGING_RATE: %d", LOGGING_RATE);
    RCLCPP_INFO(this->get_logger(), "SHOW_CONSOLE: %d", SHOW_CONSOLE);
*/

    if (strcmp(POLICY_TYPE_STR.c_str(),"PARAM_OPTIM")==0)
    {
        Policy = PARAM_OPTIM;
    }
    else if (strcmp(POLICY_TYPE_STR.c_str(),"DEEP_RL_ONBOARD")==0)
    {
        Policy = DEEP_RL_ONBOARD;
    }
    else if (strcmp(POLICY_TYPE_STR.c_str(),"DEEP_RL_SB3")==0)
    {
        Policy = DEEP_RL_SB3;
    }    


    /// Declare PARAMETERS from Model_Types.yaml
    // UPDATE INERTIAL PARAMETERS
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Mass", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Ixx", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Iyy", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Izz", 0.0);

    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Mass", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Ixx", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Iyy", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Izz", 0.0);

    // UPDATE SYSTEM PARAMETERS
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Thrust_max", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.C_tf", 0.0);

    std::vector<double> default_TrajAcc_Max = {0.0, 0.0, 0.0};
    std::vector<double> default_TrajJerk_Max = {0.0, 0.0, 0.0};
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Thrust_Coeff", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Torque_Coeff", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Tau_up", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Tau_down", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.TrajAcc_Max", default_TrajAcc_Max);
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.TrajJerk_Max", default_TrajJerk_Max);

    // UPDATE GEOMETRIC PARAMETERS
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Forward_Reach", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Gamma_eff", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".L_eff", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Leg_Params.K_Pitch", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Leg_Params.K_Yaw", 0.0);

    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Leg_Length", 0.0);
    this->declare_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Leg_Angle", 0.0);

    std::vector<double> default_prop_front = {0.0, 0.0};
    std::vector<double> default_prop_rear = {0.0, 0.0};
    // UPDATE PROP DISTANCES
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Prop_Front", default_prop_front);
    this->declare_parameter("SAR_Type." + SAR_Type + ".System_Params.Prop_Rear", default_prop_rear);

    // UPDATE CTRL GAINS
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


    /// Load parameters from YAML file
    loadParametersFromModel_TypesFile("/home/dlee/ros2_ws/src/sar_simulation/sar_config/Model_Types.yaml");


    /// Get PARAMETERS from Model_Types.yaml
    // UPDATE INERTIAL PARAMETERS
    m = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Mass").as_double();
    Ixx = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Ixx").as_double();
    Iyy = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Iyy").as_double();
    Izz = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Izz").as_double();

    Base_Mass = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Mass").as_double();
    Base_Ixx = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Ixx").as_double();    
    Base_Iyy = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Iyy").as_double();
    Base_Izz = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Izz").as_double();

    // UPDATE SYSTEM PARAMETERS
    Thrust_max = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Thrust_max").as_double();
    C_tf = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.C_tf").as_double();

    Thrust_Coeff = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Thrust_Coeff").as_double();
    Torque_Coeff = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Torque_Coeff").as_double();
    Tau_up = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Tau_up").as_double();
    Tau_down = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Tau_down").as_double();
    TrajAcc_Max = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.TrajAcc_Max").as_double_array();
    TrajJerk_Max = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.TrajJerk_Max").as_double_array();

    // UPDAT GEOMETRIC PARAMETERS
    Forward_Reach = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Forward_Reach").as_double();
    L_eff = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".L_eff").as_double();
    Gamma_eff = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Gamma_eff").as_double();
    K_Pitch = this->get_parameter("SAR_Type." + SAR_Type + ".Leg_Params.K_Pitch").as_double();
    K_Yaw = this->get_parameter("SAR_Type." + SAR_Type + ".Leg_Params.K_Yaw").as_double();

    Leg_Length = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Leg_Length").as_double();
    Leg_Angle = this->get_parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Leg_Angle").as_double();

    // UPDATE PROP DISTANCES
    Prop_Front_Vec = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Prop_Front").as_double_array();
    Prop_Rear_Vec = this->get_parameter("SAR_Type." + SAR_Type + ".System_Params.Prop_Rear").as_double_array();
    Prop_14_x = Prop_Front_Vec[0];
    Prop_14_y = Prop_Front_Vec[1];
    Prop_23_x = Prop_Rear_Vec[0];
    Prop_23_y = Prop_Rear_Vec[1];    

    // UPDATE CTRL GAINS
    P_kp_xy = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_kp_xy").as_double();
    P_kd_xy = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_kd_xy").as_double();
    P_ki_xy = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_ki_xy").as_double();
    i_range_xy = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.i_range_xy").as_double();

    P_kp_z = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_kp_z").as_double();
    P_kd_z = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_kd_z").as_double();
    P_ki_z = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_ki_z").as_double();
    i_range_z = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.i_range_z").as_double();

    R_kp_xy = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_kp_xy").as_double();
    R_kd_xy = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_kd_xy").as_double();
    R_ki_xy = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_ki_xy").as_double();
    i_range_R_xy = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.i_range_R_xy").as_double();

    R_kp_z = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_kp_z").as_double();
    R_kd_z = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_kd_z").as_double();
    R_ki_z = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_ki_z").as_double();
    i_range_R_z = this->get_parameter("SAR_Type."+ SAR_Type +".CtrlGains.i_range_R_z").as_double();

/*
    /// Print INERTIAL PARAMETERS to verify from Model_Types.yaml
    // UPDATE INERTIAL PARAMETERS
    RCLCPP_INFO(this->get_logger(), "m: %f", m);
    RCLCPP_INFO(this->get_logger(), "Ixx: %f", Ixx);
    RCLCPP_INFO(this->get_logger(), "Iyy: %f", Iyy);
    RCLCPP_INFO(this->get_logger(), "Izz: %f", Izz);

    RCLCPP_INFO(this->get_logger(), "Base_Mass: %f", Base_Mass);
    RCLCPP_INFO(this->get_logger(), "Base_Ixx: %f", Base_Ixx);
    RCLCPP_INFO(this->get_logger(), "Base_Iyy: %f", Base_Iyy);
    RCLCPP_INFO(this->get_logger(), "Base_Izz: %f", Base_Izz);

    // UPDATE SYSTEM PARAMETERS
    RCLCPP_INFO(this->get_logger(), "Thrust_max: %f", Thrust_max);
    RCLCPP_INFO(this->get_logger(), "C_tf: %f", C_tf);

    RCLCPP_INFO(this->get_logger(), "Thrust_Coeff: %f", Thrust_Coeff);
    RCLCPP_INFO(this->get_logger(), "Torque_Coeff: %f", Torque_Coeff);
    RCLCPP_INFO(this->get_logger(), "Tau_up: %f", Tau_up);
    RCLCPP_INFO(this->get_logger(), "Tau_down: %f", Tau_down);
    RCLCPP_INFO(this->get_logger(), "TrajAcc_Max: [%f, %f, %f]", TrajAcc_Max[0], TrajAcc_Max[1], TrajAcc_Max[2]);
    RCLCPP_INFO(this->get_logger(), "TrajJerk_Max: [%f, %f, %f]", TrajJerk_Max[0], TrajJerk_Max[1], TrajJerk_Max[2]);

    // UPDAT GEOMETRIC PARAMETERS
    RCLCPP_INFO(this->get_logger(), "Forward_Reach: %f", Forward_Reach);
    RCLCPP_INFO(this->get_logger(), "Gamma_eff: %f", Gamma_eff);
    RCLCPP_INFO(this->get_logger(), "L_eff: %f", L_eff);
    RCLCPP_INFO(this->get_logger(), "K_Pitch: %f", K_Pitch);
    RCLCPP_INFO(this->get_logger(), "K_Yaw: %f", K_Yaw);

    RCLCPP_INFO(this->get_logger(), "Leg_Length: %f", Leg_Length);
    RCLCPP_INFO(this->get_logger(), "Leg_Angle: %d", Leg_Angle);

    // UPDATE PROP DISTANCES
    RCLCPP_INFO(this->get_logger(), "Prop_Front_Vec: [%f, %f]", Prop_Front_Vec[0], Prop_Front_Vec[1]);
    RCLCPP_INFO(this->get_logger(), "Prop_Rear_Vec: [%f, %f]", Prop_Rear_Vec[0], Prop_Rear_Vec[1]);

    // UPDATE CTRL GAINS
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
*/

}

void Controller::loadParametersFromSim_SettingsFile(const std::string &file_path) {
    YAML::Node config = YAML::LoadFile(file_path);

    if (config["SAR_Controller"]["ros__parameters"]) {
        auto params = config["SAR_Controller"]["ros__parameters"];

        this->set_parameter(rclcpp::Parameter("DATA_TYPE", params["DATA_TYPE"].as<std::string>()));
        this->set_parameter(rclcpp::Parameter("SAR_SETTINGS.SAR_Type", params["SAR_SETTINGS"]["SAR_Type"].as<std::string>()));
        this->set_parameter(rclcpp::Parameter("SAR_SETTINGS.SAR_Config", params["SAR_SETTINGS"]["SAR_Config"].as<std::string>()));
        this->set_parameter(rclcpp::Parameter("SAR_SETTINGS.Policy_Type", params["SAR_SETTINGS"]["Policy_Type"].as<std::string>()));
        this->set_parameter(rclcpp::Parameter("SIM_SETTINGS.Vicon_Delay", params["SIM_SETTINGS"]["Vicon_Delay"].as<int>()));

        this->set_parameter(rclcpp::Parameter("CAM_SETTINGS.Cam_Config", params["CAM_SETTINGS"]["Cam_Config"].as<std::string>()));
        this->set_parameter(rclcpp::Parameter("CAM_SETTINGS.Cam_Active", params["CAM_SETTINGS"]["Cam_Active"].as<bool>()));

        this->set_parameter(rclcpp::Parameter("PLANE_SETTINGS.Plane_Type", params["PLANE_SETTINGS"]["Plane_Type"].as<std::string>()));
        this->set_parameter(rclcpp::Parameter("PLANE_SETTINGS.Plane_Config", params["PLANE_SETTINGS"]["Plane_Config"].as<std::string>()));
        this->set_parameter(rclcpp::Parameter("PLANE_SETTINGS.Plane_Angle_init", params["PLANE_SETTINGS"]["Plane_Angle_init"].as<double>()));
        this->set_parameter(rclcpp::Parameter("PLANE_SETTINGS.Pos_X_init", params["PLANE_SETTINGS"]["Pos_X_init"].as<double>()));
        this->set_parameter(rclcpp::Parameter("PLANE_SETTINGS.Pos_Y_init", params["PLANE_SETTINGS"]["Pos_Y_init"].as<double>()));
        this->set_parameter(rclcpp::Parameter("PLANE_SETTINGS.Pos_Z_init", params["PLANE_SETTINGS"]["Pos_Z_init"].as<double>()));

        this->set_parameter(rclcpp::Parameter("SIM_SETTINGS.Sim_Speed", params["SIM_SETTINGS"]["Sim_Speed"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SIM_SETTINGS.Sim_Slowdown_Speed", params["SIM_SETTINGS"]["Sim_Slowdown_Speed"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SIM_SETTINGS.Landing_Slowdown_Flag", params["SIM_SETTINGS"]["Landing_Slowdown_Flag"].as<bool>()));

        this->set_parameter(rclcpp::Parameter("SAR_DC_SETTINGS.Logging_Rate", params["SAR_DC_SETTINGS"]["Logging_Rate"].as<int>()));
        this->set_parameter(rclcpp::Parameter("SAR_DC_SETTINGS.Console_Output", params["SAR_DC_SETTINGS"]["Console_Output"].as<bool>()));

    }
}

void Controller::loadParametersFromModel_TypesFile(const std::string &file_path) {
    YAML::Node config = YAML::LoadFile(file_path);

    if (config["SAR_Controller"]["ros__parameters"]) {
        auto params = config["SAR_Controller"]["ros__parameters"];

        // UPDATE INERTIAL PARAMETERS
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Mass", params["SAR_Type"][SAR_Type]["Config"][SAR_Config]["Ref_Mass"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Ixx", params["SAR_Type"][SAR_Type]["Config"][SAR_Config]["Ref_Ixx"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Iyy", params["SAR_Type"][SAR_Type]["Config"][SAR_Config]["Ref_Iyy"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Ref_Izz", params["SAR_Type"][SAR_Type]["Config"][SAR_Config]["Ref_Izz"].as<double>()));

        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Mass", params["SAR_Type"][SAR_Type]["System_Params"]["Base_Mass"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Ixx", params["SAR_Type"][SAR_Type]["System_Params"]["Base_Ixx"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Iyy", params["SAR_Type"][SAR_Type]["System_Params"]["Base_Iyy"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Base_Izz", params["SAR_Type"][SAR_Type]["System_Params"]["Base_Izz"].as<double>()));

        // UPDATE SYSTEM PARAMETERS
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Thrust_max", params["SAR_Type"][SAR_Type]["System_Params"]["Thrust_max"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.C_tf", params["SAR_Type"][SAR_Type]["System_Params"]["C_tf"].as<double>()));

        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Thrust_Coeff", params["SAR_Type"][SAR_Type]["System_Params"]["Thrust_Coeff"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Torque_Coeff", params["SAR_Type"][SAR_Type]["System_Params"]["Torque_Coeff"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Tau_up", params["SAR_Type"][SAR_Type]["System_Params"]["Tau_up"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Tau_down", params["SAR_Type"][SAR_Type]["System_Params"]["Tau_down"].as<double>()));

        std::vector<double> TrajAcc_Max;
        for (auto it : params["SAR_Type"][SAR_Type]["System_Params"]["TrajAcc_Max"]) {
            TrajAcc_Max.push_back(it.as<double>());
        }
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.TrajAcc_Max", TrajAcc_Max));

        std::vector<double> TrajJerk_Max;
        for (auto it : params["SAR_Type"][SAR_Type]["System_Params"]["TrajJerk_Max"]) {
            TrajJerk_Max.push_back(it.as<double>());
        }
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.TrajJerk_Max", TrajJerk_Max));

        // UPDAT GEOMETRIC PARAMETERS
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Forward_Reach", params["SAR_Type"][SAR_Type]["System_Params"]["Forward_Reach"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Gamma_eff", params["SAR_Type"][SAR_Type]["Config"][SAR_Config]["Gamma_eff"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".L_eff", params["SAR_Type"][SAR_Type]["Config"][SAR_Config]["L_eff"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".Leg_Params.K_Pitch", params["SAR_Type"][SAR_Type]["Leg_Params"]["K_Pitch"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".Leg_Params.K_Yaw", params["SAR_Type"][SAR_Type]["Leg_Params"]["K_Yaw"].as<double>()));

        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Leg_Length", params["SAR_Type"][SAR_Type]["Config"][SAR_Config]["Leg_Length"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".Config." + SAR_Config + ".Leg_Angle", params["SAR_Type"][SAR_Type]["Config"][SAR_Config]["Leg_Angle"].as<double>()));


        // UPDATE PROP DISTANCES
        std::vector<double> prop_front;
        for (auto it : params["SAR_Type"][SAR_Type]["System_Params"]["Prop_Front"]) {
            prop_front.push_back(it.as<double>());
        }
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Prop_Front", prop_front));

        std::vector<double> prop_rear;
        for (auto it : params["SAR_Type"][SAR_Type]["System_Params"]["Prop_Rear"]) {
            prop_rear.push_back(it.as<double>());
        }
        this->set_parameter(rclcpp::Parameter("SAR_Type." + SAR_Type + ".System_Params.Prop_Rear", prop_rear));

        // UPDATE CTRL GAINS
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_kp_xy", params["SAR_Type"][SAR_Type]["CtrlGains"]["P_kp_xy"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_kd_xy", params["SAR_Type"][SAR_Type]["CtrlGains"]["P_kd_xy"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_ki_xy", params["SAR_Type"][SAR_Type]["CtrlGains"]["P_ki_xy"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.i_range_xy", params["SAR_Type"][SAR_Type]["CtrlGains"]["i_range_xy"].as<double>()));

        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_kp_z", params["SAR_Type"][SAR_Type]["CtrlGains"]["P_kp_z"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_kd_z", params["SAR_Type"][SAR_Type]["CtrlGains"]["P_kd_z"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.P_ki_z", params["SAR_Type"][SAR_Type]["CtrlGains"]["P_ki_z"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.i_range_z", params["SAR_Type"][SAR_Type]["CtrlGains"]["i_range_z"].as<double>()));

        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_kp_xy", params["SAR_Type"][SAR_Type]["CtrlGains"]["R_kp_xy"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_kd_xy", params["SAR_Type"][SAR_Type]["CtrlGains"]["R_kd_xy"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_ki_xy", params["SAR_Type"][SAR_Type]["CtrlGains"]["R_ki_xy"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.i_range_R_xy", params["SAR_Type"][SAR_Type]["CtrlGains"]["i_range_R_xy"].as<double>()));

        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_kp_z", params["SAR_Type"][SAR_Type]["CtrlGains"]["R_kp_z"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_kd_z", params["SAR_Type"][SAR_Type]["CtrlGains"]["R_kd_z"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.R_ki_z", params["SAR_Type"][SAR_Type]["CtrlGains"]["R_ki_z"].as<double>()));
        this->set_parameter(rclcpp::Parameter("SAR_Type."+ SAR_Type +".CtrlGains.i_range_R_z", params["SAR_Type"][SAR_Type]["CtrlGains"]["i_range_R_z"].as<double>()));
        
    }
}

//!!!!! NN output need to solved
// PUBLISH CONTROLLER DATA ON ROS TOPIC
void Controller::publishCtrlData()
{
    CtrlData_msg.tick = tick;

    // STATE DATA WRT ORIGIN
    CtrlData_msg.pose_b_o.position.x = Pos_B_O.x;
    CtrlData_msg.pose_b_o.position.y = Pos_B_O.y;
    CtrlData_msg.pose_b_o.position.z = Pos_B_O.z;

    CtrlData_msg.pose_b_o.orientation.x = Quat_B_O.x;
    CtrlData_msg.pose_b_o.orientation.y = Quat_B_O.y;
    CtrlData_msg.pose_b_o.orientation.z = Quat_B_O.z;
    CtrlData_msg.pose_b_o.orientation.w = Quat_B_O.w;

    CtrlData_msg.twist_b_o.linear.x = Vel_B_O.x;
    CtrlData_msg.twist_b_o.linear.y = Vel_B_O.y;
    CtrlData_msg.twist_b_o.linear.z = Vel_B_O.z;

    CtrlData_msg.twist_b_o.angular.x = Omega_B_O.x;
    CtrlData_msg.twist_b_o.angular.y = Omega_B_O.y;
    CtrlData_msg.twist_b_o.angular.z = Omega_B_O.z;

    CtrlData_msg.accel_b_o.linear.x = Accel_B_O.x;
    CtrlData_msg.accel_b_o.linear.y = Accel_B_O.y;
    CtrlData_msg.accel_b_o.linear.z = Accel_B_O.z;

    CtrlData_msg.accel_b_o.angular.x = dOmega_B_O.x;
    CtrlData_msg.accel_b_o.angular.y = dOmega_B_O.y;
    CtrlData_msg.accel_b_o.angular.z = dOmega_B_O.z;

    CtrlData_msg.accel_b_o_mag = Accel_B_O_Mag;


    // STATE DATA WRT PLANE
    CtrlData_msg.pose_p_b.position.x = Pos_P_B.x;
    CtrlData_msg.pose_p_b.position.y = Pos_P_B.y;
    CtrlData_msg.pose_p_b.position.z = Pos_P_B.z;

    CtrlData_msg.pose_p_b.orientation.x = Quat_P_B.x;
    CtrlData_msg.pose_p_b.orientation.y = Quat_P_B.y;
    CtrlData_msg.pose_p_b.orientation.z = Quat_P_B.z;
    CtrlData_msg.pose_p_b.orientation.w = Quat_P_B.w;

    CtrlData_msg.twist_b_p.linear.x = Vel_B_P.x;
    CtrlData_msg.twist_b_p.linear.y = Vel_B_P.y;
    CtrlData_msg.twist_b_p.linear.z = Vel_B_P.z;

    CtrlData_msg.twist_b_p.angular.x = Omega_B_P.x;
    CtrlData_msg.twist_b_p.angular.y = Omega_B_P.y;
    CtrlData_msg.twist_b_p.angular.z = Omega_B_P.z;


    // PLANE RELATIVE STATES
    CtrlData_msg.d_perp = D_perp;
    CtrlData_msg.d_perp_cr = D_perp_CR;
    CtrlData_msg.vel_mag_b_p = Vel_mag_B_P;
    CtrlData_msg.vel_angle_b_p = Vel_angle_B_P;

    // PLANE STATES
    CtrlData_msg.plane_pos.x = r_P_O.x;
    CtrlData_msg.plane_pos.y = r_P_O.y;
    CtrlData_msg.plane_pos.z = r_P_O.z;
    CtrlData_msg.plane_angle_deg = Plane_Angle_deg;
 
    // OPTICAL FLOW DATA
    CtrlData_msg.optical_flow.x = Theta_x;
    CtrlData_msg.optical_flow.y = Theta_y;
    CtrlData_msg.optical_flow.z = Tau;
    CtrlData_msg.tau_cr = Tau_CR;
    

    // ESTIMATED OPTICAL FLOW DATA
    CtrlData_msg.optical_flow_cam.x = Theta_x_Cam;
    CtrlData_msg.optical_flow_cam.y = Theta_y_Cam;
    CtrlData_msg.optical_flow_cam.z = Tau_Cam;

    // POLICY ACTIONS
    CtrlData_msg.nn_output = {tanhf(Y_output->data[0][0]),tanhf(Y_output->data[1][0]),Y_output->data[2][0],Y_output->data[3][0]};
    CtrlData_msg.a_trg = a_Trg;
    CtrlData_msg.a_rot = a_Rot;

    // CONTROL ACTIONS
    CtrlData_msg.fm = {F_thrust,M.x*1.0e3,M.y*1.0e3,M.z*1.0e3};
    CtrlData_msg.motorthrusts = {M1_thrust,M2_thrust,M3_thrust,M4_thrust};
    CtrlData_msg.motor_cmd = {M1_CMD,M2_CMD,M3_CMD,M4_CMD};

    CtrlData_msg.x_d.x = x_d.x;
    CtrlData_msg.x_d.y = x_d.y;
    CtrlData_msg.x_d.z = x_d.z;

    CtrlData_msg.v_d.x = v_d.x;
    CtrlData_msg.v_d.y = v_d.y;
    CtrlData_msg.v_d.z = v_d.z;

    CtrlData_msg.a_d.x = a_d.x;
    CtrlData_msg.a_d.y = a_d.y;
    CtrlData_msg.a_d.z = a_d.z;



    // ==========================
    //  STATES AT POLICY TRIGGER
    // ==========================
    CtrlData_msg.trg_flag = Trg_Flag;

    // STATE DATA WRT ORIGIN
    CtrlData_msg.pose_b_o_trg.position.x = Pos_B_O_trg.x;
    CtrlData_msg.pose_b_o_trg.position.y = Pos_B_O_trg.y;
    CtrlData_msg.pose_b_o_trg.position.z = Pos_B_O_trg.z;

    CtrlData_msg.pose_b_o_trg.orientation.x = Quat_B_O_trg.x;
    CtrlData_msg.pose_b_o_trg.orientation.y = Quat_B_O_trg.y;
    CtrlData_msg.pose_b_o_trg.orientation.z = Quat_B_O_trg.z;
    CtrlData_msg.pose_b_o_trg.orientation.w = Quat_B_O_trg.w;

    CtrlData_msg.twist_b_o_trg.linear.x = Vel_B_O_trg.x;
    CtrlData_msg.twist_b_o_trg.linear.y = Vel_B_O_trg.y;
    CtrlData_msg.twist_b_o_trg.linear.z = Vel_B_O_trg.z;

    CtrlData_msg.twist_b_o_trg.angular.x = Omega_B_O_trg.x;
    CtrlData_msg.twist_b_o_trg.angular.y = Omega_B_O_trg.y;
    CtrlData_msg.twist_b_o_trg.angular.z = Omega_B_O_trg.z;

    // STATE DATA WRT PLANE
    CtrlData_msg.pose_p_b_trg.position.x = Pos_P_B_trg.x;
    CtrlData_msg.pose_p_b_trg.position.y = Pos_P_B_trg.y;
    CtrlData_msg.pose_p_b_trg.position.z = Pos_P_B_trg.z;

    CtrlData_msg.pose_p_b_trg.orientation.x = Quat_P_B_trg.x;
    CtrlData_msg.pose_p_b_trg.orientation.y = Quat_P_B_trg.y;
    CtrlData_msg.pose_p_b_trg.orientation.z = Quat_P_B_trg.z;
    CtrlData_msg.pose_p_b_trg.orientation.w = Quat_P_B_trg.w;

    CtrlData_msg.twist_b_p_trg.linear.x = Vel_B_P_trg.x;
    CtrlData_msg.twist_b_p_trg.linear.y = Vel_B_P_trg.y;    
    CtrlData_msg.twist_b_p_trg.linear.z = Vel_B_P_trg.z;

    CtrlData_msg.twist_b_p_trg.angular.x = Omega_B_P_trg.x;
    CtrlData_msg.twist_b_p_trg.angular.y = Omega_B_P_trg.y;
    CtrlData_msg.twist_b_p_trg.angular.z = Omega_B_P_trg.z;

    // PLANE RELATIVE STATES
    CtrlData_msg.d_perp_trg = D_perp_trg;
    CtrlData_msg.d_perp_cr_trg = D_perp_CR_trg;
    CtrlData_msg.vel_mag_b_p_trg = Vel_mag_B_P_trg;
    CtrlData_msg.vel_angle_b_p_trg = Vel_angle_B_P_trg;


    // OPTICAL FLOW DATA (TRIGGER)
    CtrlData_msg.optical_flow_trg.x = Theta_x_trg;
    CtrlData_msg.optical_flow_trg.y = Theta_y_trg;
    CtrlData_msg.optical_flow_trg.z = Tau_trg;
    CtrlData_msg.tau_cr_trg = Tau_CR_trg;

    // POLICY ACTIONS (TRIGGER)
    CtrlData_msg.nn_output_trg = {tanhf(Y_output_trg[0]),tanhf(Y_output_trg[1]),Y_output_trg[2],Y_output_trg[3]};
    CtrlData_msg.a_trg_trg = a_Trg_trg;
    CtrlData_msg.a_rot_trg = a_Rot_trg;


    // ==========================
    //      STATES AT IMPACT
    // ==========================
    CtrlData_msg.impact_flag_ob = Impact_Flag_OB;

    CtrlData_msg.vel_mag_b_p_impact_ob = Vel_mag_B_P_impact_OB;
    CtrlData_msg.vel_angle_b_p_impact_ob = Vel_angle_B_P_impact_OB;

    CtrlData_msg.pose_b_o_impact_ob.orientation.x = Quat_B_O_impact_OB.x;
    CtrlData_msg.pose_b_o_impact_ob.orientation.y = Quat_B_O_impact_OB.y;
    CtrlData_msg.pose_b_o_impact_ob.orientation.z = Quat_B_O_impact_OB.z;
    CtrlData_msg.pose_b_o_impact_ob.orientation.w = Quat_B_O_impact_OB.w;


    CtrlData_msg.twist_b_p_impact_ob.angular.x = Omega_B_O_impact_OB.x;
    CtrlData_msg.twist_b_p_impact_ob.angular.y = Omega_B_O_impact_OB.y;
    CtrlData_msg.twist_b_p_impact_ob.angular.z = Omega_B_O_impact_OB.z;

    CtrlData_msg.domega_b_o_y_impact_ob = dOmega_B_O_impact_OB.y;
    
    CTRL_Data_Publisher->publish(CtrlData_msg);
}

//!!!!! completed
void Controller::publishCtrlDebug()
{
    CtrlDebug_msg.tumbled_flag = Tumbled_Flag;
    CtrlDebug_msg.tumbledetect_flag = TumbleDetect_Flag;
    CtrlDebug_msg.motorstop_flag = MotorStop_Flag;
    CtrlDebug_msg.angaccel_flag = AngAccel_Flag; 
    CtrlDebug_msg.armed_flag = Armed_Flag;
    CtrlDebug_msg.customthrust_flag = CustomThrust_Flag;
    CtrlDebug_msg.custommotorcmd_flag = CustomMotorCMD_Flag;

    CtrlDebug_msg.pos_ctrl_flag = (bool)kp_xf;
    CtrlDebug_msg.vel_ctrl_flag = (bool)kd_xf;
    CtrlDebug_msg.policy_armed_flag = Policy_Armed_Flag; 
    CtrlDebug_msg.camactive_flag = CamActive_Flag;

    CTRL_Debug_Publisher->publish(CtrlDebug_msg);
    //std::cout << "CtrlDebug_msgs are published" << std::endl;
}


void Controller::publishROSParamData(){

    ROSParams_msg.data_type = DATA_TYPE;
    ROSParams_msg.sar_type = SAR_Type;
    ROSParams_msg.sar_config = SAR_Config;
    ROSParams_msg.policy_type_str = POLICY_TYPE_STR;

    ROSParams_msg.cam_config = Cam_Config;
    ROSParams_msg.camactive_flag = CamActive_Flag;

    ROSParams_msg.policy_type = POLICY_TYPE;
    ROSParams_msg.vicon_delay_ms = Vicon_Delay_ms;

    ROSParams_msg.plane_type = Plane_Type;
    ROSParams_msg.plane_config = Plane_Config;
    ROSParams_msg.plane_angle_deg = Plane_Angle_deg;
    ROSParams_msg.pos_x = r_P_O.x;
    ROSParams_msg.pos_y = r_P_O.y;
    ROSParams_msg.pos_z = r_P_O.z;

    ROSParams_msg.sim_speed = SIM_SPEED;
    ROSParams_msg.sim_slowdown_speed = SIM_SLOWDOWN_SPEED;
    ROSParams_msg.landing_slowdown_flag = LANDING_SLOWDOWN_FLAG;

    ROSParams_msg.logging_rate = LOGGING_RATE;
    ROSParams_msg.show_console = SHOW_CONSOLE;

    ROSParams_msg.ref_mass = m;
    ROSParams_msg.ref_ixx = Ixx;
    ROSParams_msg.ref_iyy = Iyy;
    ROSParams_msg.ref_izz = Izz;

    ROSParams_msg.base_mass = Base_Mass;
    ROSParams_msg.base_ixx = Base_Ixx;
    ROSParams_msg.base_iyy = Base_Iyy;
    ROSParams_msg.base_izz = Base_Izz;    

    ROSParams_msg.thrust_max = Thrust_max;
    ROSParams_msg.c_tf = C_tf;

    ROSParams_msg.thrust_coeff = Thrust_Coeff;
    ROSParams_msg.torque_coeff = Torque_Coeff;
    ROSParams_msg.tau_up = Tau_up;
    ROSParams_msg.tau_down = Tau_down;
    ROSParams_msg.trajacc_max[0] = TrajAcc_Max[0];
    ROSParams_msg.trajacc_max[1] = TrajAcc_Max[1];
    ROSParams_msg.trajacc_max[2] = TrajAcc_Max[2];
    ROSParams_msg.trajjerck_max[0] = TrajJerk_Max[0];
    ROSParams_msg.trajjerck_max[1] = TrajJerk_Max[1];
    ROSParams_msg.trajjerck_max[2] = TrajJerk_Max[2];

    ROSParams_msg.forward_reach = Forward_Reach;
    ROSParams_msg.gamma_eff = Gamma_eff;
    ROSParams_msg.l_eff = L_eff;
    ROSParams_msg.k_pitch = K_Pitch;
    ROSParams_msg.k_yaw = K_Yaw;

    ROSParams_msg.leg_length = Leg_Length;
    ROSParams_msg.leg_angle = Leg_Angle;

    ROSParams_msg.prop_front_vec[0] = Prop_Front_Vec[0];
    ROSParams_msg.prop_front_vec[1] = Prop_Front_Vec[1];
    ROSParams_msg.prop_rear_vec[0] = Prop_Rear_Vec[0];
    ROSParams_msg.prop_rear_vec[1] = Prop_Rear_Vec[1];


    ROSParams_msg.p_kp_xy = P_kp_xy;
    ROSParams_msg.p_kd_xy = P_kd_xy;
    ROSParams_msg.p_ki_xy = P_ki_xy;
    ROSParams_msg.i_range_xy = i_range_xy;

    ROSParams_msg.p_kp_z = P_kp_z;
    ROSParams_msg.p_kd_z = P_kd_z;
    ROSParams_msg.p_ki_z = P_ki_z;
    ROSParams_msg.i_range_z = i_range_z;

    ROSParams_msg.r_kp_xy = R_kp_xy;
    ROSParams_msg.r_kd_xy = R_kd_xy;
    ROSParams_msg.r_ki_xy = R_ki_xy;
    ROSParams_msg.i_range_r_xy = i_range_R_xy;

    ROSParams_msg.r_kp_z = R_kp_z;
    ROSParams_msg.r_kd_z = R_kd_z;
    ROSParams_msg.r_ki_z = R_ki_z;
    ROSParams_msg.i_range_r_z = i_range_R_z;

    ROSParams_msg.tau_diff = Tau_DIFF;

    ROS_Parmas_Publisher->publish(ROSParams_msg);
    //std::cout << "ROS2 prameters are published" << std::endl;
}

void Controller::appLoop()
{
    rclcpp::Rate rate(1000);

    // RUN STABILIZER LOOP
    while (rclcpp::ok())
    {
        appMain();
        
        rate.sleep();
    }
}
/*
void Controller::stabilizerLoop() // MAIN CONTROLLER LOOP
{
    rclcpp::Rate rate(1000);
    loadInitParams();
    //std::cout << "stabilizerLoop is executed" << std::endl;

    // INITIATE CONTROLLER
    controllerOutOfTreeInit();
    Armed_Flag = true;

    // RUN STABILIZER LOOP
    while (rclcpp::ok())
    {
        controllerOutOfTree(&control, &setpoint, &sensorData, &state, tick);
        //std::cout << "controllerOutOfTree is executed" << std::endl;

        Controller::publishCtrlData();
        Controller::publishCtrlDebug();
        Controller::publishROSParamData();

        rate.sleep(); // Process holds here till next tick
        tick++;
    }

}*/
//https://github.com/ros-navigation/navigation2/issues/3586
//https://gazebosim.org/api/sim/8/distributedsimulation.html
/**/
void Controller::stabilizerLoop() 
{
    bool sim_time_enabled = this->get_parameter("use_sim_time").as_bool();
    if (sim_time_enabled) {
        RCLCPP_INFO(this->get_logger(), "Simulation time enabled.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Real time (wall clock) enabled.");
    }

    // Time for staring simulation
    rclcpp::Time last_time = this->get_clock()->now(); 

    loadInitParams();
    controllerOutOfTreeInit();
    Armed_Flag = true;

    while (rclcpp::ok())
    {
        // Current simulation time
        rclcpp::Time current_time = this->get_clock()->now();
        
        //
        rclcpp::Duration time_diff = current_time - last_time;

        //
        if (time_diff.seconds() >= 0.001) {
            controllerOutOfTree(&control, &setpoint, &sensorData, &state, tick);

            
            Controller::publishCtrlData();
            Controller::publishCtrlDebug();
            Controller::publishROSParamData();

            tick++;
            last_time = current_time;  // Update the last_time
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}