#ifndef STABILIZER_H
#define STABILIZER_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "sar_msgs/msg/ms.hpp"
#include "sar_msgs/msg/imu_data.hpp"
#include "sar_msgs/msg/vicon_data.hpp"
#include "sar_msgs/msg/motor_thrust.hpp"
#include "sar_msgs/srv/set_trigger.hpp"

#include "math3d.h"
#include <cmath>

#include <yaml-cpp/yaml.h>
#include <fstream>

class Controller
{
    public:
        Controller(): Node("controller")
        {
            // ===========================
            //     ROS TOPICS/SERVICES
            // ===========================

            // EXTERNAL SENSOR SUBSCRIBERS
            subscriber_Vicon = this->create_subscription<sar_msgs::msg::ViconData>("Vicon/data", 1, std::bind(&ThrustController::ThrustControl, this, std::placeholders::_1));

            // INTERNAL SENSOR SUBSCRIBERS
            subscriber_IMU = this->create_subscription<sar_msgs::msg::IMUData>("imu/data", 1, std::bind(&ThrustController::IMU_Update_Callback, this, std::placeholders::_1));
        }

        // EXTERNAL SENSOR SUBSCRIBERS
        rclcpp::Subscription<sar_msgs::msg::ViconData>::SharedPtr subscriber_Vicon;
        
        // INTERNAL SENSOR SUBSCRIBERS
        rclcpp::Subscription<sar_msgs::msg::IMUData>::SharedPtr subscriber_IMU;

        // FUNCTION PROTOTYPES
        void IMU_Update_Callback(const sar_msgs::msg::IMUData::SharedPtr msg);
        void Vicon_Update_Callback(const sar_msgs::msg::ViconData::SharedPtr msg);

        void loadInitParams();

};

void Controller::IMU_Update_Callback(const sar_msgs::msg::IMUData::SharedPtr msg) {
    Imu_Data = msg;
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

void Controller::Vicon_Update_Callback(const sar_msgs::msg::ViconData::SharedPtr msg) {
    Vicon_Data = msg;
    std::cout << "position(x)_Vicon : " << Vicon_Data->pose.position.x  << std::endl;
    std::cout << "position(x)_Vicon : " << msg->pose.position.x << std::endl;
    std::cout << "position(y)_Vicon : " << msg->pose.position.y << std::endl;
    std::cout << "position(z)_Vicon : " << msg->pose.position.z << std::endl;
    std::cout << "orientation(x)_Vicon : " << msg->pose.orientation.x << std::endl;
    std::cout << "orientation(y)_Vicon : " << msg->pose.orientation.y << std::endl;
    std::cout << "orientation(z)_Vicon : " << msg->pose.orientation.z << std::endl;   
    std::cout << "vel(x)_Vicon : " << msg->vel.x << std::endl;
    std::cout << "vel(y)_Vicon : " << msg->vel.y << std::endl;
    std::cout << "vel(z)_Vicon : " << msg->vel.z << std::endl;
}

// LOAD VALUES FROM ROSPARAM SERVER INTO CONTROLLER
void Controller::loadInitParams()
{
    printf("Updating Parameters\n");
    std::string Sim_Setting_file_path = "/home/dlee/ros2_ws/src/sar_simulation/sar_config/Sim_Settings.yaml";
    try {
        YAML::Node config = YAML::LoadFile(Sim_Setting_file_path);

        // Declare parameter to parameter server
        this->declare_parameter("SAR_Type", config["SAR_SETTINGS"]["SAR_Type"].as<std::string>());
        this->declare_parameter("SAR_Config", config["SAR_SETTINGS"]["SAR_Config"].as<std::string>());
        this->declare_parameter("POLICY_TYPE_STR", config["SAR_SETTINGS"]["Policy_Type"].as<std::string>());
        this->get_parameter("SAR_Type", SAR_Type);
        this->get_parameter("SAR_Config", SAR_Config);
        this->get_parameter("POLICY_TYPE_STR", POLICY_TYPE_STR);
        //std::cout << "SAR_Type:" << SAR_Type << std::endl;
        //std::cout << "SAR_Config:" << SAR_Config << std::endl;
        //std::cout << "POLICY_TYPE_STR:" << POLICY_TYPE_STR << std::endl;
        
        RCLCPP_INFO(this->get_logger(), "Loaded Sim_Settings from YAML");

    } catch (const YAML::BadFile& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load Sim_Settings file: %s", e.what());
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Sim_Settings.YAML exception: %s", e.what());
    }
    std::string Model_Types_file_path = "/home/dlee/ros2_ws/src/sar_simulation/sar_config/Model_Types.yaml";
    try {
        YAML::Node config = YAML::LoadFile(Model_Types_file_path);
        
        // UPDATE INERTIAL PARAMETERS
        this->declare_parameter("Ref_Mass", config["SAR_Type"][SAR_Type]["Config"][SAR_Config]["Ref_Mass"].as<float>());
        this->declare_parameter("Ref_Ixx", config["SAR_Type"][SAR_Type]["Config"][SAR_Config]["Ref_Ixx"].as<float>());
        this->declare_parameter("Ref_Iyy", config["SAR_Type"][SAR_Type]["Config"][SAR_Config]["Ref_Iyy"].as<float>());
        this->declare_parameter("Ref_Izz", config["SAR_Type"][SAR_Type]["Config"][SAR_Config]["Ref_Izz"].as<float>());
        this->get_parameter("Ref_Mass", Ref_Mass);
        this->get_parameter("Ref_Ixx", Ref_Ixx);
        this->get_parameter("Ref_Iyy", Ref_Iyy);
        this->get_parameter("Ref_Izz", Ref_Izz);

        // UPDATE SYSTEM PARAMETERS
        this->declare_parameter("Thrust_max", config["SAR_Type"][SAR_Type]["System_Params"]["Thrust_max"].as<float>());
        this->declare_parameter("C_tf", config["SAR_Type"][SAR_Type]["System_Params"]["C_tf"].as<float>());
        this->get_parameter("Thrust_max", Thrust_max);
        this->get_parameter("C_tf", C_tf);

        // UPDAT GEOMETRIC PARAMETERS
        this->declare_parameter("Forward_Reach", config["SAR_Type"][SAR_Type]["System_Params"]["Forward_Reach"].as<float>());
        this->declare_parameter("L_eff", config["SAR_Type"][SAR_Type]["Config"][SAR_Config]["L_eff"].as<float>());
        this->get_parameter("Forward_Reach", Forward_Reach);
        this->get_parameter("L_eff", L_eff);

        // UPDATE PROP DISTANCES
        this->declare_parameter("Prop_Front_Vec", config["SAR_Type"][SAR_Type]["System_Params"]["Prop_Front"].as<std::vector<float>>());
        this->get_parameter("Prop_Front_Vec", Prop_Front_Vec);
        Prop_14_x = Prop_Front_Vec[0];
        Prop_14_y = Prop_Front_Vec[1];
        this->declare_parameter("Prop_Rear_Vec", config["SAR_Type"][SAR_Type]["System_Params"]["Prop_Rear"].as<std::vector<float>>());
        this->get_parameter("Prop_Rear_Vec", Prop_Rear_Vec);
        Prop_23_x = Prop_Rear_Vec[0];
        Prop_23_y = Prop_Rear_Vec[1];

        // UPDATE CTRL GAINS
        this->declare_parameter("P_kp_xy", config["SAR_Type"][SAR_Type]["CtrlGains"]["P_kp_xy"].as<float>());
        this->declare_parameter("P_kd_xy", config["SAR_Type"][SAR_Type]["CtrlGains"]["P_kd_xy"].as<float>());
        this->declare_parameter("P_ki_xy", config["SAR_Type"][SAR_Type]["CtrlGains"]["P_ki_xy"].as<float>());
        this->declare_parameter("i_range_xy", config["SAR_Type"][SAR_Type]["CtrlGains"]["i_range_xy"].as<float>());
        this->get_parameter("P_kp_xy", P_kp_xy);
        this->get_parameter("P_kd_xy", P_kd_xy);
        this->get_parameter("P_ki_xy", P_ki_xy);
        this->get_parameter("i_range_xy", i_range_xy);

        this->declare_parameter("P_kp_z", config["SAR_Type"][SAR_Type]["CtrlGains"]["P_kp_z"].as<float>());
        this->declare_parameter("P_kd_z", config["SAR_Type"][SAR_Type]["CtrlGains"]["P_kd_z"].as<float>());
        this->declare_parameter("P_ki_z", config["SAR_Type"][SAR_Type]["CtrlGains"]["P_ki_z"].as<float>());
        this->declare_parameter("i_range_z", config["SAR_Type"][SAR_Type]["CtrlGains"]["i_range_z"].as<float>());
        this->get_parameter("P_kp_z", P_kp_z);
        this->get_parameter("P_kd_z", P_kd_z);
        this->get_parameter("P_ki_z", P_ki_z);
        this->get_parameter("i_range_z", i_range_z);

        this->declare_parameter("R_kp_xy", config["SAR_Type"][SAR_Type]["CtrlGains"]["R_kp_xy"].as<float>());
        this->declare_parameter("R_kd_xy", config["SAR_Type"][SAR_Type]["CtrlGains"]["R_kd_xy"].as<float>());
        this->declare_parameter("R_ki_xy", config["SAR_Type"][SAR_Type]["CtrlGains"]["R_ki_xy"].as<float>());
        this->declare_parameter("i_range_R_xy", config["SAR_Type"][SAR_Type]["CtrlGains"]["i_range_R_xy"].as<float>());
        this->get_parameter("R_kp_xy", R_kp_xy);
        this->get_parameter("R_kd_xy", R_kd_xy);
        this->get_parameter("R_ki_xy", R_ki_xy);
        this->get_parameter("i_range_R_xy", i_range_R_xy);

        this->declare_parameter("R_kp_z", config["SAR_Type"][SAR_Type]["CtrlGains"]["R_kp_z"].as<float>());
        this->declare_parameter("R_kd_z", config["SAR_Type"][SAR_Type]["CtrlGains"]["R_kd_z"].as<float>());
        this->declare_parameter("R_ki_z", config["SAR_Type"][SAR_Type]["CtrlGains"]["R_ki_z"].as<float>());
        this->declare_parameter("i_range_R_z", config["SAR_Type"][SAR_Type]["CtrlGains"]["i_range_R_z"].as<float>());
        this->get_parameter("R_kp_z", R_kp_z);
        this->get_parameter("R_kd_z", R_kd_z);
        this->get_parameter("R_ki_z", R_ki_z);
        this->get_parameter("i_range_R_z", i_range_R_z);

        RCLCPP_INFO(this->get_logger(), "Loaded Model_Types from YAML");

    } catch (const YAML::BadFile& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load Model_Types file: %s", e.what());
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Model_Types.YAML exception: %s", e.what());
    }
}

#endif // STABILIZER_H