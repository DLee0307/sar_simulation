#include "SAR_DataConverter.h"
SAR_DataConverter::SAR_DataConverter()
    : Node("SAR_DataConverter_Node") {

        cmd_input_service_ = this->create_service<sar_msgs::srv::CTRLCmdSrv>("/SAR_DC/CMD_Input", std::bind(&SAR_DataConverter::CMD_SAR_DC_Callback, this, std::placeholders::_1, std::placeholders::_2));
        CMD_Output_Service_Sim = this->create_client<sar_msgs::srv::CTRLCmdSrv>("/CTRL/Cmd_ctrl"); //To Stabilizer
        CMD_Output_Service_Exp = this->create_client<crazyflie_interfaces::srv::CTRLCmdSrv>("/cf1/Cmd_ctrl"); // To crazyflie_server.cpp
        /*
        if (DATA_TYPE == "SIM")
        {      
            
        }
        else{ 
            // /cf231/Cmd_ctrl 서비스가 사용 가능한지 확인
            while (!CMD_Output_Service_Exp->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "/cf231/Cmd_ctrl service not available, waiting again...");
            }
            RCLCPP_INFO(this->get_logger(), "/cf231/Cmd_ctrl service is available");
        }*/

        // INTERNAL SENSOR SUBSCRIBERS
        CTRL_Data_Sub = this->create_subscription<sar_msgs::msg::CtrlData>("/CTRL/data", 1, std::bind(&SAR_DataConverter::CtrlData_Callback, this, std::placeholders::_1));
        CTRL_Debug_Sub = this->create_subscription<sar_msgs::msg::CtrlDebug>("/CTRL/debug", 1, std::bind(&SAR_DataConverter::CtrlDebug_Callback, this, std::placeholders::_1));

        // CRAZYSWARM PIPELINE
        cf1_States_B_O_Sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>("/cf1/States_B_O", 1, std::bind(&SAR_DataConverter::cf1_States_B_O_Callback, this, std::placeholders::_1));
        cf1_States_B_P_Sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>("/cf1/States_B_P", 1, std::bind(&SAR_DataConverter::cf1_States_B_P_Callback, this, std::placeholders::_1));
        cf1_TrgState_Sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>("/cf1/TrgState", 1, std::bind(&SAR_DataConverter::cf1_TrgState_Callback, this, std::placeholders::_1));
        cf1_ImpactOBState_Sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>("/cf1/ImpactOBState", 1, std::bind(&SAR_DataConverter::cf1_Impact_OB_Callback, this, std::placeholders::_1));
        cf1_CTRL_Output_Sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>("/cf1/CTRL_Output", 1, std::bind(&SAR_DataConverter::cf1_CTRL_Output_Callback, this, std::placeholders::_1));
        cf1_SetPoints_Sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>("/cf1/SetPoints", 1, std::bind(&SAR_DataConverter::cf1_SetPoints_Callback, this, std::placeholders::_1));
        cf1_Flags_Sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>("/cf1/Flags", 1, std::bind(&SAR_DataConverter::cf1_Flags_Callback, this, std::placeholders::_1));
        cf1_Misc_Sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>("/cf1/Misc", 1, std::bind(&SAR_DataConverter::cf1_Misc_Callback, this, std::placeholders::_1));
        cf1_Practice_Sub = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>("/cf1/Practice", 1, std::bind(&SAR_DataConverter::cf1_Practice_Callback, this, std::placeholders::_1));

        // ROS2 PARAMETER
        ROS_Parmas_Sub = this->create_subscription<sar_msgs::msg::ROSParams>("/ROS2/PARAMETER", 1, std::bind(&SAR_DataConverter::ROSParams_Callback, this, std::placeholders::_1));

        //@@@ Clock 객체를 초기화
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        // INITIALIZE STATE DATA PUBLISHERS
        StateData_Pub = this->create_publisher<sar_msgs::msg::SARStateData>("/SAR_DC/StateData", 1);
        TriggerData_Pub = this->create_publisher<sar_msgs::msg::SARTriggerData>("/SAR_DC/TriggerData", 1);
        ImpactData_Pub = this->create_publisher<sar_msgs::msg::SARImpactData>("/SAR_DC/ImpactData", 1);
        MiscData_Pub = this->create_publisher<sar_msgs::msg::SARMiscData>("/SAR_DC/MiscData", 1);

        // LOGGING 
        Logging_Service = this->create_service<sar_msgs::srv::LoggingCMD>("/SAR_DC/DataLogging", std::bind(&SAR_DataConverter::DataLogging_Callback, this, std::placeholders::_1, std::placeholders::_2));

        // INITIALIZE SAR_DC THREADS
        SAR_DC_Thread = std::thread(&SAR_DataConverter::MainLoop, this);
        ConsoleOutput_Thread = std::thread(&SAR_DataConverter::ConsoleLoop, this);
        Logging_Thread = std::thread(&SAR_DataConverter::LoggingLoop, this);

}

bool SAR_DataConverter::CMD_SAR_DC_Callback(const sar_msgs::srv::CTRLCmdSrv::Request::SharedPtr request,
                                            sar_msgs::srv::CTRLCmdSrv::Response::SharedPtr response)
{
    auto req_copy_exp = std::make_shared<crazyflie_interfaces::srv::CTRLCmdSrv::Request>();
    auto req_copy_sim = std::make_shared<sar_msgs::srv::CTRLCmdSrv::Request>();

    req_copy_exp->cmd_type = request->cmd_type;
    req_copy_exp->cmd_vals = request->cmd_vals;
    req_copy_exp->cmd_flag = request->cmd_flag;
    req_copy_exp->cmd_rx = request->cmd_rx;

    req_copy_sim->cmd_type = request->cmd_type;
    req_copy_sim->cmd_vals = request->cmd_vals;
    req_copy_sim->cmd_flag = request->cmd_flag;
    req_copy_sim->cmd_rx = request->cmd_rx;
/*
    std::cout << "Service is requested in DataConverter" << std::endl;
    std::cout << "cmd_type: " << request->cmd_type << std::endl;
    std::cout << "cmd_vals: " << request->cmd_vals.x << std::endl;
    std::cout << "cmd_vals: " << request->cmd_vals.y << std::endl;
    std::cout << "cmd_vals: " << request->cmd_vals.z << std::endl;
    std::cout << "cmd_flag: " << request->cmd_flag << std::endl;
    std::cout << "cmd_rx: " << request->cmd_rx << std::endl;
*/
    if (DATA_TYPE == "SIM")
    {
        //!!! is for experiment work
        //!!!std::cout << "Sending request to CMD_Output_Service_Sim" << std::endl;
        auto result = CMD_Output_Service_Sim->async_send_request(req_copy_sim);
/*
        // 동기적 서비스 호출 처리
        if (rclcpp::spin_until_future_complete(shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto sim_response = result.get();
            std::cout << "Service call completed with result: " << std::boolalpha << sim_response->srv_success << std::endl;
            if (sim_response->srv_success) {
                std::cout << "Service call to SIM succeeded" << std::endl;
            } else {
                std::cout << "Service call to SIM failed" << std::endl;
            }
        } else {
            std::cerr << "Service call to SIM failed to complete" << std::endl;
        }*/
        //!!!std::cout << "CMD_SAR_DC_Callback in SAR_DataConverter.cpp is completed" << std::endl;
        return request->cmd_rx;
    }
    else {
        //std::cout << "Sending request to CMD_Output_Service_Exp" << std::endl;
        auto result = CMD_Output_Service_Exp->async_send_request(req_copy_exp);
/*
        // 동기적 서비스 호출 처리
        if (rclcpp::spin_until_future_complete(shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto exp_response = result.get();
            std::cout << "Service call completed with result: " << std::boolalpha << exp_response->srv_success << std::endl;
            if (exp_response->srv_success) {
                std::cout << "Service call to Exp succeeded" << std::endl;
            } else {
                std::cout << "Service call to Exp failed" << std::endl;
            }
        } else {
            std::cerr << "Service call to Exp failed to complete" << std::endl;
        }*/
        //std::cout << "CMD_SAR_DC_Callback in SAR_DataConverter.cpp is completed" << std::endl;
        return request->cmd_rx;
    }
    /*
    // SIMULATION: SEND COMMAND VALUES TO SIM CONTROLLER (SEND AS SERVICE REQUEST)
    auto req_copy_exp = std::make_shared<crazyflie_interfaces::srv::CTRLCmdSrv::Request>();
    auto req_copy_sim = std::make_shared<sar_msgs::srv::CTRLCmdSrv::Request>();

    req_copy_exp->cmd_type = request->cmd_type;
    req_copy_exp->cmd_vals = request->cmd_vals;
    req_copy_exp->cmd_flag = request->cmd_flag;
    req_copy_exp->cmd_rx = request->cmd_rx;
    
    req_copy_sim->cmd_type = request->cmd_type;
    req_copy_sim->cmd_vals = request->cmd_vals;
    req_copy_sim->cmd_flag = request->cmd_flag;
    req_copy_sim->cmd_rx = request->cmd_rx;

    std::cout << "service is requested in DataConverter" <<  std::endl;
    std::cout << "cmd_type: " << request->cmd_type << std::endl;
    std::cout << "cmd_vals: " << request->cmd_vals.x << std::endl;
    std::cout << "cmd_vals: " << request->cmd_vals.y << std::endl;
    std::cout << "cmd_vals: " << request->cmd_vals.z << std::endl;
    std::cout << "cmd_flag: " << request->cmd_flag << std::endl;
    std::cout << "cmd_rx: " << request->cmd_rx << std::endl;
    
    //std::cout << "cmd_type: " << req_copy->cmd_type << std::endl;
    //std::cout << "DATA_TYPE: " << DATA_TYPE.compare("SIM") << std::endl;


    if (DATA_TYPE.compare("SIM") == 0)
    {
        auto result = CMD_Output_Service_Sim->async_send_request(req_copy_sim);
        std::cout << "CMD_Output_Service_Sim" << std::endl;
        return request->cmd_rx;
    }
    try {
        auto result = CMD_Output_Service_Exp->async_send_request(req_copy_exp);
        std::cout << "CMD_Output_Service_Exp request sent" << std::endl;

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            std::cout << "Service call succeeded" << std::endl;
        } else {
            std::cout << "Service call failed" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }
*/
    /*else
    {
        auto result = CMD_Output_Service_Exp->async_send_request(req_copy_exp);
        std::cout << "CMD_Output_Service_Exp" << std::endl;

        return request->cmd_rx;
    }
*/
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