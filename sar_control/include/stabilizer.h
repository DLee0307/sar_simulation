#ifndef STABILIZER_H
#define STABILIZER_H

#include <thread>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iomanip>

#include "sar_msgs/msg/imu_data.hpp"
#include "sar_msgs/msg/vicon_data.hpp"
#include "sar_msgs/msg/ctrl_data.hpp"
#include "sar_msgs/msg/ctrl_debug.hpp"
#include "sar_msgs/msg/ros_params.hpp"

#include "sar_msgs/srv/ctrl_cmd_srv.hpp"

#include "math3d.h"
#include <cmath>
#include <yaml-cpp/yaml.h>

// FIRMWARE INCLUDES
#include "app.h"
#include "controller.h"

#include "Shared_Lib.h"
#include "Traj_Funcs.h"
#include "Controller_GTC.h"

//#include "sar_msgs/msg/ms.hpp"
//#include "sar_msgs/msg/motor_thrust.hpp"
//#include "sar_msgs/srv/set_trigger.hpp"


#define PWM_MAX 60000
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)
#define Deg2Rad (float)M_PI/180.0f
#define Rad2Deg 180.0f/(float)M_PI

//const float RATE_100_HZ = 100.0f; define in crazyflie firmware

class Controller : public rclcpp::Node
{
public:
    Controller();

    // DEFINE THREAD OBJECTS
    std::thread appThread;
    std::thread controllerThread;

    // FIRMWARE VARIABLES FOR CONTROLLER
    setpoint_t setpoint;
    sensorData_t sensorData;
    state_t state;
    control_t control;
    uint32_t tick = 2;
    bool ResetTickDelay_Flag = false;

    // ROS PARAMS
    std::string DATA_TYPE;
    std::string SAR_Type;
    std::string SAR_Config;
    std::string Cam_Config;
    std::string POLICY_TYPE_STR;

    // QUAD GEOMETRY PARAMETERS
    std::vector<double> Prop_Front_Vec;
    std::vector<double> Prop_Rear_Vec;
    //!! can be problem previous state was float
    //std::vector<float> Prop_Front_Vec;
    //std::vector<float> Prop_Rear_Vec;

    void publishCtrlData(rclcpp::Publisher<sar_msgs::msg::CtrlData>::SharedPtr& publisher);

    void Vicon_Update_Callback(const sar_msgs::msg::ViconData::SharedPtr msg);
    void IMU_Update_Callback(const sar_msgs::msg::IMUData::SharedPtr msg);


    void CMD_Service_Resp(const sar_msgs::srv::CTRLCmdSrv::Request::SharedPtr request,
                    sar_msgs::srv::CTRLCmdSrv::Response::SharedPtr response);


    void appLoop();
    void stabilizerLoop();

    void loadInitParams();
    void publishCtrlData();
    void publishCtrlDebug();
    void publishROSParamData();

    void loadParametersFromSim_SettingsFile(const std::string &file_path);
    void loadParametersFromModel_TypesFile(const std::string &file_path);

private:
    // ROS2 Topic Publisher
    rclcpp::Publisher<sar_msgs::msg::CtrlData>::SharedPtr CTRL_Data_Publisher;
    rclcpp::Publisher<sar_msgs::msg::CtrlDebug>::SharedPtr CTRL_Debug_Publisher;
    rclcpp::Publisher<sar_msgs::msg::ROSParams>::SharedPtr ROS_Parmas_Publisher;

    // ROS2 Topic Subscriber
    rclcpp::Subscription<sar_msgs::msg::ViconData>::SharedPtr subscriber_Vicon;
    rclcpp::Subscription<sar_msgs::msg::IMUData>::SharedPtr subscriber_IMU;

    // ROS2 Service
    rclcpp::Service<sar_msgs::srv::CTRLCmdSrv>::SharedPtr CMD_Output_Service;


    // MESSAGES
    // MESSAGES for Publisher
    sar_msgs::msg::CtrlData CtrlData_msg;
    sar_msgs::msg::CtrlDebug CtrlDebug_msg;
    sar_msgs::msg::ROSParams ROSParams_msg;

    // MESSAGES for Subscriber
    sar_msgs::msg::IMUData::SharedPtr Imu_Data;
    sar_msgs::msg::ViconData::SharedPtr Vicon_Data;

};




#endif // STABILIZER_H