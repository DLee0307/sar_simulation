#include "SAR_DataConverter.h"
SAR_DataConverter::SAR_DataConverter()
    : Node("SAR_DataConverter_Node") {

        cmd_input_service_ = this->create_service<sar_msgs::srv::CTRLCmdSrv>("/SAR_DC/CMD_Input", std::bind(&SAR_DataConverter::CMD_SAR_DC_Callback, this, std::placeholders::_1, std::placeholders::_2));
        cmd_output_client_ = this->create_client<sar_msgs::srv::CTRLCmdSrv>("/CTRL/Cmd_ctrl"); //To Stabilizer
        
        // INTERNAL SENSOR SUBSCRIBERS
        CTRL_Data_Sub = this->create_subscription<sar_msgs::msg::CtrlData>("/CTRL/data", 1, std::bind(&SAR_DataConverter::CtrlData_Callback, this, std::placeholders::_1));
        CTRL_Debug_Sub = this->create_subscription<sar_msgs::msg::CtrlDebug>("/CTRL/debug", 1, std::bind(&SAR_DataConverter::CtrlDebug_Callback, this, std::placeholders::_1));

        // ROS2 PARAMETER
        ROS_Parmas_Sub = this->create_subscription<sar_msgs::msg::ROSParams>("/ROS2/PARAMETER", 1, std::bind(&SAR_DataConverter::ROSParams_Callback, this, std::placeholders::_1));

        //@@@ Clock 객체를 초기화
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        // INITIALIZE STATE DATA PUBLISHERS
        StateData_Pub = this->create_publisher<sar_msgs::msg::SARStateData>("/SAR_DC/StateData", 1);
        TriggerData_Pub = this->create_publisher<sar_msgs::msg::SARTriggerData>("/SAR_DC/TriggerData", 1);
        ImpactData_Pub = this->create_publisher<sar_msgs::msg::SARImpactData>("/SAR_DC/ImpactData", 1);
        MiscData_Pub = this->create_publisher<sar_msgs::msg::SARMiscData>("/SAR_DC/MiscData", 1);

        // INITIALIZE SAR_DC THREADS
        SAR_DC_Thread = std::thread(&SAR_DataConverter::MainLoop, this);
        ConsoleOutput_Thread = std::thread(&SAR_DataConverter::ConsoleLoop, this);


}

bool SAR_DataConverter::CMD_SAR_DC_Callback(const sar_msgs::srv::CTRLCmdSrv::Request::SharedPtr request,
                                            sar_msgs::srv::CTRLCmdSrv::Response::SharedPtr response)
{
    // SIMULATION: SEND COMMAND VALUES TO SIM CONTROLLER (SEND AS SERVICE REQUEST)
    auto req_copy = std::make_shared<sar_msgs::srv::CTRLCmdSrv::Request>();
    req_copy->cmd_type = request->cmd_type;
    req_copy->cmd_vals = request->cmd_vals;
    req_copy->cmd_flag = request->cmd_flag;
    req_copy->cmd_rx = request->cmd_rx;
    std::cout << "service is requested in DataConverter" <<  std::endl;
    std::cout << "cmd_type: " << request->cmd_type << std::endl;
    //std::cout << "cmd_type: " << req_copy->cmd_type << std::endl;

    auto result = cmd_output_client_->async_send_request(req_copy);

    return request->cmd_rx;

}

void SAR_DataConverter::ROSParams_Callback(const sar_msgs::msg::ROSParams::SharedPtr msg)
{
    //std::cout << "ROSParams_Callback is run" << std::endl;

    DATA_TYPE = msg->data_type;
    SAR_Type = msg->sar_type;
    SAR_Config = msg->sar_config;

    Mass = msg->ref_mass;
    Ixx = msg->ref_ixx;
    Iyy = msg->ref_ixx;
    Izz = msg->ref_izz;

    Gamma_eff = msg->gamma_eff;
    L_eff = msg->l_eff;
    K_Pitch = msg->k_pitch;
    K_Yaw = msg->k_yaw;

    Plane_Config = msg->plane_config;
    //Plane_Angle_deg = msg->plane_angle_deg;
    //Plane_Pos.x = msg->pos_x;
    //Plane_Pos.y = msg->pos_y;
    //Plane_Pos.z = msg->pos_z;

     /*   
    // DATA SETTINGS
    if(DATA_TYPE.compare("SIM") == 0)
    {
        ros::param::set("/use_sim_time",true);
    }
    else
    {
        ros::param::set("/use_sim_time",false);
    }
    */

   P_kp_xy = msg->p_kp_xy;
   P_kd_xy = msg->p_kd_xy;
   P_ki_xy = msg->p_ki_xy;

   P_kp_z = msg->p_kp_z;
   P_kd_z = msg->p_kd_z;
   P_ki_z = msg->p_ki_z;

   R_kp_xy = msg->r_kp_xy;
   R_kd_xy = msg->r_kd_xy;
   R_ki_xy = msg->r_ki_xy;

   R_kp_z = msg->r_kp_z;
   R_kd_z = msg->r_kd_z;
   R_ki_z = msg->r_ki_z;

   POLICY_TYPE = msg->policy_type;

   SIM_SPEED = msg->sim_speed;
   SIM_SLOWDOWN_SPEED = msg->sim_slowdown_speed;
   LANDING_SLOWDOWN_FLAG  = msg->landing_slowdown_flag;

   LOGGING_RATE = msg->logging_rate;
   SHOW_CONSOLE = msg->show_console; 

}

void SAR_DataConverter::MainInit()
{
    //loadInitParams();
    //adjustSimSpeed(SIM_SPEED);
    //@@@rclcpp::Clock::SharedPtr clock = this->get_clock();
    //@@@Time_start = clock->now();    
    Time_start = clock_->now();    
}

void SAR_DataConverter::MainLoop()
{

    MainInit();
    const int refresh_rate = 50; // 20 Hz
    const int delay_time_us = 1000000 / refresh_rate;


    
    while (rclcpp::ok())
    { 
        //checkSlowdown();
        
        // PUBLISH ORGANIZED DATA
        Publish_StateData();
        Publish_TriggerData();
        Publish_ImpactData();
        Publish_MiscData();

        rclcpp::sleep_for(std::chrono::microseconds(delay_time_us));
    }


}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SAR_DataConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}