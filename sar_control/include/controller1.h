#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "sar_msgs/msg/ms.hpp"
#include "sar_msgs/msg/imu_data.hpp"
#include "sar_msgs/msg/vicon_data.hpp"
#include "sar_msgs/msg/motor_thrust.hpp"
#include "sar_msgs/srv/set_trigger.hpp"
#include "sar_msgs/srv/ctrl_cmd_srv.hpp"

#include "math3d.h"
#include <cmath>

#include <yaml-cpp/yaml.h>
#include <fstream>

#define PWM_MAX 60000
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)
#define Deg2Rad (float)M_PI/180.0f
#define Rad2Deg 180.0f/(float)M_PI

const float RATE_100_HZ = 100.0f;

class ThrustController : public rclcpp::Node
{
public:
    ThrustController();

    // CTRL COMMAND PACKETS
    struct CTRL_CmdPacket{
        uint8_t cmd_type; 
        float cmd_val1;
        float cmd_val2;
        float cmd_val3;
        float cmd_flag;
        bool  cmd_rx;
    } __attribute__((packed));
    
    struct CTRL_CmdPacket CTRL_Cmd;

    void CTRL_Command(struct CTRL_CmdPacket *CTRL_Cmd);

    // =================================
    //    INITIAL SYSTEM PARAMETERS
    // =================================
    //float m = 0.036585f;             // [kg]
    float m = 46.62E-3f;             // [kg]
    float Ixx = 33.40E-6f;           // [kg*m^2]
    float Iyy = 33.90E-6f;           // [kg*m^2]
    float Izz = 56.00E-6f;           // [kg*m^2]
    struct mat33 J;             // Rotational Inertia Matrix [kg*m^2]

    float Ref_Mass;
    float Ref_Ixx;
    float Ref_Iyy;
    float Ref_Izz;

    float C_tf = 6.18e-3f;          // Moment Coeff [Nm/N]
    float Thrust_max = 15.0f;         // Max thrust per motor [g]

    const float g = 9.81f;                        // Gravity [m/s^2]
    const struct vec e_3 = {0.0f, 0.0f, 1.0f};    // Global z-axis

    float dt = (float)(1.0f/RATE_100_HZ);

    // =================================
    //       GEOMETRIC PARAMETERS
    // =================================

    // QUAD GEOMETRY PARAMETERS
    std::vector<double> Prop_Front_Vec;
    std::vector<double> Prop_Rear_Vec;

    float Prop_14_x = 33.0e-3f;         // Front Prop Distance - x-axis [m]
    float Prop_14_y = 33.0e-3f;         // Front Prop Distance - y-axis [m]
    float Prop_23_x = 33.0e-3f;         // Rear  Prop Distance - x-axis [m]
    float Prop_23_y = 33.0e-3f;         // Rear  Prop Distance - y-axis [m]

    float L_eff;             // Effective Leg Length [m]
    float Forward_Reach;     // Forward Reach [m]
    float Collision_Radius;  // Collision Radius [m]
    
    // =================================
    //    CONTROL GAIN INITIALIZATION
    // =================================
    // XY POSITION PID
    float P_kp_xy = 0.3f; // Crazy flie P_kp_xy
    float P_kd_xy = 0.22f; // Crazy flie P_kd_xy
    float P_ki_xy = 0.0f; // Crazy flie P_ki_xy
    float i_range_xy = 0.1f; // Crazy flie i_range_xy

    // Z POSITION PID
    float P_kp_z = 0.5f; // Crazy flie P_kp_z
    float P_kd_z = 0.28f; // Crazy flie P_kd_z
    float P_ki_z = 0.0f; // Crazy flie P_ki_z
    float i_range_z = 0.1f; // Crazy flie i_range_z

    // XY ATTITUDE PID
    float R_kp_xy = 0.004f; // Crazy flie R_kp_xy
    float R_kd_xy = 0.0009f; // Crazy flie R_kd_xy
    float R_ki_xy = 0.0f; // Crazy flie R_ki_xy
    float i_range_R_xy = 0.1f; // Crazy flie i_range_R_xy

    // Z ATTITUDE PID
    float R_kp_z = 0.003f; // Crazy flie R_kp_xy
    float R_kd_z = 0.0009f; // Crazy flie R_kp_xy
    float R_ki_z = 0.0003f; // Crazy flie R_kp_xy
    float i_range_R_z = 0.1f; // Crazy flie R_kp_xy

    // INIT CTRL GAIN VECTORS 
    struct vec Kp_p; // Pos. Proportional Gains 
    struct vec Kd_p; // Pos. Derivative Gains
    struct vec Ki_p; // Pos. Integral Gains  

    struct vec Kp_R; // Rot. Proportional Gains
    struct vec Kd_R; // Rot. Derivative Gains
    struct vec Ki_R; // Rot. Integral Gains

    // INIT GAIN FLAGS
    float kp_xf = 1;    // Pos. Gain Flag
    float kd_xf = 1;    // Pos. Derivative Gain Flag

    // =================================
    //     BODY WRT ORIGIN STATES
    // =================================
    struct vec Pos_B_O = {0.0f,0.0f,0.0f};          // Pos [m]
    struct vec Vel_B_O = {0.0f,0.0f,0.0f};          // Vel [m/s]
    struct vec Accel_B_O = {0.0f,0.0f,0.0f};        // Linear Accel. [m/s^2]
    float Accel_B_O_Mag = 0.0f;                     // Linear Acceleration Magnitude [m/s^2]

    struct quat Quat_B_O = {0.0f,0.0f,0.0f,1.0f};   // Orientation
    struct vec Omega_B_O = {0.0f,0.0f,0.0f};        // Angular Rate [rad/s]
    struct vec Omega_B_O_prev  = {0.0f,0.0f,0.0f};  // Prev Angular Rate [rad/s^2]
    struct vec dOmega_B_O = {0.0f,0.0f,0.0f};       // Angular Accel [rad/s^2]

    struct mat33 R;                                 // Orientation as rotation matrix
    struct vec b3 = {0.0f,0.0f,1.0f};               // Current body z-axis in global coord.

    // =================================
    //         DESIRED STATES
    // =================================    
    struct vec x_d = {0.0f,0.0f,0.0f};              // Pos-desired [m]
    struct vec x_d_d = {0.0f,0.0f,0.0f};  
    struct vec v_d = {0.0f,0.0f,0.0f};              // Vel-desired [m/s]
    struct vec a_d = {0.0f,0.0f,0.0f};              // Acc-desired [m/s^2]

    struct vec b1_d = {1.0f,0.0f,0.0f};             // Desired body x-axis in global coord. 
    struct vec b2_d = {0.0f,1.0f,0.0f};             // Desired body y-axis in global coord.
    struct vec b3_d = {0.0f,0.0f,1.0f};             // Desired body z-axis in global coord.
    struct mat33 R_d;                               // Desired rotational matrix from b_d vectors

    struct quat quat_d = {0.0f,0.0f,0.0f,1.0f};     // Orientation-desired [qx,qy,qz,qw] !! need to check value
    struct vec omega_d = {0.0f,0.0f,0.0f};          // Omega-desired [rad/s] !! need to check value
    struct vec domega_d = {0.0f,0.0f,0.0f};         // Ang. Acc-desired [rad/s^2] !! need to check value

    // =================================
    //         STATE ERRORS
    // =================================
    struct vec e_x;  // Pos-error [m]
    struct vec e_v;  // Vel-error [m/s]
    struct vec e_PI; // Pos. Integral-error [m*s]

    struct vec e_R;  // Rotation-error [rad]
    struct vec e_w;  // Omega-error [rad/s]
    struct vec e_RI; // Rot. Integral-error [rad*s]

    // =================================
    //       CONTROLLER ACTUATIONS
    // =================================
    struct vec F_thrust_ideal = {0.0f,0.0f,1.0f};   // Ideal thrust vector [N]
    float F_thrust = 0.0f; 
    struct vec M = {0.0f,0.0f,0.0f};                // Implemented body moments [N*m]
    struct vec M_d = {0.0f,0.0f,0.0f};              // Desired moment [N*m]

    // MOTOR THRUSTS
    float M1_thrust;
    float M2_thrust;
    float M3_thrust;
    float M4_thrust;

    // =================================
    //   TEMPORARY CALC VECS/MATRICES
    // =================================
    struct vec temp1_v; 
    struct vec temp2_v;
    struct vec temp3_v;
    struct vec temp4_v;
    struct mat33 temp1_m;  

    struct vec P_effort; // Effort by positional PID
    struct vec R_effort; // Effort by rotational PID

    struct mat33 RdT_R;  // Rd' * R
    struct mat33 RT_Rd;  // R' * Rd
    struct vec Gyro_dyn;

    // FIRMWARE VARIABLES FOR CONTROLLER
    //state_t state;
    //sensorData_t sensorData;

    struct mat33 hat(struct vec v);
    struct vec dehat(struct mat33 m);
    struct vec quat2eul(struct quat q);
    void printvec(struct vec v);
    void printquat(struct quat q);
    void printmat(struct mat33 m);

    //void controlOutput(const state_t *state, const sensorData_t *sensors)
    std::array<double, 4> controlOutput();

    void ThrustControl(const sar_msgs::msg::ViconData::SharedPtr msg);
    void IMU_Update_Callback(const sar_msgs::msg::IMUData::SharedPtr msg);

    void CMD_Service_Resp(const sar_msgs::srv::CTRLCmdSrv::Request::SharedPtr request,
                     sar_msgs::srv::CTRLCmdSrv::Response::SharedPtr response);

    void set_trigger(const sar_msgs::srv::SetTrigger::Request::SharedPtr request,
                     sar_msgs::srv::SetTrigger::Response::SharedPtr response);

    void loadInitParams();

    // ROS PARAMS
    std::string SAR_Type;
    std::string SAR_Config;
    std::string Cam_Config;
    std::string POLICY_TYPE_STR;

    float A = 0;
    //Traj_Function
    // =================================
    //     TRAJECTORY INITIALIZATION
    // =================================

    typedef enum {
        NONE = 0,
        P2P = 1,
        CONST_VEL = 2,
        CONST_VEL_GZ = 3,
    } Trajectory_Type;

    typedef enum {
        x_axis = 0, 
        y_axis = 1,
        z_axis = 2
    } axis_direction;

    static Trajectory_Type Traj_Type;
    static axis_direction axis;

    struct traj_vec {
        struct {
            float x;
            float y;
            float z;
        } values;
    };

    bool Traj_Active[3] = {false,false,false};
    float s_0_t[3] = {0.0f, 0.0f, 0.0f};   // Traj Start Point [m]
    float s_f_t[3] = {0.0f, 0.0f, 0.0f};   // Traj End Point [m]
    float v_t[3] = {0.0f, 0.0f, 0.0f};     // Traj Vel [m/s]
    float a_t[3] = {0.0f, 0.0f, 0.0f};     // Traj Accel [m/s^2]
    float T[3] = {0.0f, 0.0f, 0.0f};       // Traj completion time [s]
    float t_traj[3] = {0.0f, 0.0f, 0.0f};  // Traj time counter [s]
    float TrajAcc_Max[3] = {1.0f, 1.0f, 3.1f}; // Max Accel for trajectory generation [m/s^2]
    //float TrajAcc_Max[3] = {1.0f, 1.0f, 3.1f}; // Max Accel for trajectory generation [m/s^2]

    void set_vec_element(struct vec *v, int index, float value);
    void point2point_Traj();
    void const_velocity_Traj();
    void const_velocity_GZ_Traj();



private:

    void publish_thrust_input(rclcpp::Publisher<sar_msgs::msg::MotorThrust>::SharedPtr& publisher, std::array<double, 4> thrust_value);

    rclcpp::Publisher<sar_msgs::msg::MotorThrust>::SharedPtr publisher;
    rclcpp::Subscription<sar_msgs::msg::ViconData>::SharedPtr subscriber_Vicon;
    rclcpp::Subscription<sar_msgs::msg::IMUData>::SharedPtr subscriber_IMU;
    rclcpp::Service<sar_msgs::srv::SetTrigger>::SharedPtr service_;
    rclcpp::Service<sar_msgs::srv::CTRLCmdSrv>::SharedPtr CMD_Output_Service;
    int trigger_ = 0;
    struct vec value_ = {0.0f, 0.0f, 0.0f};
    sar_msgs::msg::IMUData::SharedPtr Imu_Data;
    sar_msgs::msg::ViconData::SharedPtr Vicon_Data;
};


// LOAD VALUES FROM ROSPARAM SERVER INTO CONTROLLER
void ThrustController::loadInitParams()
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

#endif // CONTROLLER_H