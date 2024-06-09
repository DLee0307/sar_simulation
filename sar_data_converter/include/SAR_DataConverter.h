#pragma once

// STANDARD INCLUDES
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <math.h> 
#include <thread>

#include <ncurses.h>

#include "sar_msgs/msg/ctrl_data.hpp"
#include "sar_msgs/msg/ctrl_debug.hpp"
#include "sar_msgs/msg/sar_state_data.hpp"
#include "sar_msgs/msg/ros_params.hpp"

#include "sar_msgs/srv/ctrl_cmd_srv.hpp"


class SAR_DataConverter : public rclcpp::Node
{
public:
    SAR_DataConverter();

    void MainInit();
    void MainLoop();
    void ConsoleLoop();

    // =======================
    //     GAZEBO CALLBACKS
    // =======================
    void CtrlData_Callback(const sar_msgs::msg::CtrlData::SharedPtr msg);
    void CtrlDebug_Callback(const sar_msgs::msg::CtrlDebug::SharedPtr msg);

    // =======================
    //     ROS2 PARAMETER
    // =======================
    void ROSParams_Callback(const sar_msgs::msg::ROSParams::SharedPtr msg);

    // =============================
    //     CTRL COMMAND CALLBACKS
    // =============================
    bool CMD_SAR_DC_Callback(const sar_msgs::srv::CTRLCmdSrv::Request::SharedPtr request,
                             sar_msgs::srv::CTRLCmdSrv::Response::SharedPtr response);

    // =================================
    //     ORGANIZED DATA PUBLISHERS
    // =================================
    void Publish_StateData();

    // =======================
    //     MISC. FUNCTIONS
    // =======================
    inline void quat2euler(float quat[],float eul[]);
    inline void euler2quat(float quat[],float eul[]);

private:

    std::thread SAR_DC_Thread;
    std::thread ConsoleOutput_Thread;

    // =====================
    //     SYSTEM PARAMS
    // =====================
    std::string DATA_TYPE;    // Sim or Experiment Flag
    rclcpp::Time Time_start;  // Initial time in UNIX notation
    int LOGGING_RATE = 20;    // Default Logging Rate
    bool SHOW_CONSOLE = true;    

    // ==================
    //     SAR PARAMS
    // ==================
    std::string SAR_Type;
    std::string SAR_Config;
    std::string SAR_Type_str;
    std::string SAR_Config_str;
    
    std::string POLICY_TYPE;

    // DEFAULT INERTIA VALUES FOR BASE CRAZYFLIE
    float Mass = NAN; // [kg]
    float Ixx = NAN;  // [kg*m^2]
    float Iyy = NAN;  // [kg*m^2]
    float Izz = NAN;  // [kg*m^2]

    float P_kp_xy,P_kd_xy,P_ki_xy;
    float P_kp_z,P_kd_z,P_ki_z;
    float R_kp_xy,R_kd_xy,R_ki_xy;     
    float R_kp_z,R_kd_z,R_ki_z;

    float Gamma_eff = NAN;
    float L_eff = NAN;
    float K_Pitch = NAN;
    float K_Yaw = NAN;


    // ============================
    //     LANDING PLANE PARAMS
    // ============================
    std::string Plane_Config;
    geometry_msgs::msg::Vector3 Plane_Pos; // Initial Plane Position
    float Plane_Angle_deg = NAN; // Initial Plane Angle [Deg]



    // ====================
    //     SIM VARIABLES
    // ====================
    int SLOWDOWN_TYPE = 0;
    bool LANDING_SLOWDOWN_FLAG;
    float SIM_SPEED = 0.5; 
    float SIM_SLOWDOWN_SPEED = 0.5;

        // =======================
        //     GAZEBO CALLBACKS
        // =======================
    rclcpp::Subscription<sar_msgs::msg::CtrlData>::SharedPtr CTRL_Data_Sub;
    rclcpp::Subscription<sar_msgs::msg::CtrlDebug>::SharedPtr CTRL_Debug_Sub;

    // ===========================
    //     ROS2 Parameter
    // ===========================
    rclcpp::Subscription<sar_msgs::msg::ROSParams>::SharedPtr ROS_Parmas_Sub;

    // ===========================
    //     CTRL COMMAND OBJECTS
    // ===========================
    rclcpp::Service<sar_msgs::srv::CTRLCmdSrv>::SharedPtr cmd_input_service_;
    rclcpp::Client<sar_msgs::srv::CTRLCmdSrv>::SharedPtr cmd_output_client_;

    // ===================
    //     FLIGHT DATA
    // ===================

    rclcpp::Time Time;
    rclcpp::Time Time_prev;

    geometry_msgs::msg::Pose Pose_B_O;
    geometry_msgs::msg::Twist Twist_B_O;
    geometry_msgs::msg::Accel Accel_B_O;
    geometry_msgs::msg::Vector3 Eul_B_O;
    double Vel_mag_B_O = NAN;
    double Vel_angle_B_O = NAN;
    float Accel_B_O_Mag = NAN;

    geometry_msgs::msg::Pose Pose_P_B;
    geometry_msgs::msg::Twist Twist_B_P;
    geometry_msgs::msg::Vector3 Eul_P_B;

    double Vel_mag_B_P = NAN;
    double Vel_angle_B_P = NAN;
    double D_perp = NAN;
    double D_perp_CR = NAN;
    double D_perp_min = INFINITY;   

    geometry_msgs::msg::Vector3 Optical_Flow;
    geometry_msgs::msg::Vector3 Optical_Flow_Cam;

    double Tau = NAN;
    double Tau_CR = NAN;
    double Theta_x = NAN;
    double Theta_y = NAN;

    double Tau_Cam = NAN;
    double Theta_x_Cam = NAN;
    double Theta_y_Cam = NAN;

    geometry_msgs::msg::Vector3 x_d;
    geometry_msgs::msg::Vector3 v_d;
    geometry_msgs::msg::Vector3 a_d;

    std::array<double, 4> FM{0, 0, 0, 0};
    std::array<double, 4> MotorThrusts{0, 0, 0, 0};
    std::array<uint16_t, 4> MS_PWM{0, 0, 0, 0};

    //boost::array<double,4> NN_Output{NAN,NAN,NAN,NAN};
    double a_Trg = NAN;
    double a_Rot = NAN;
    double Rot_Sum = 0.0;

    // ==========================
    //  STATES AT POLICY TRIGGER
    // ==========================

    bool Trg_Flag = false;
    bool OnceFlag_Trg = false;
    rclcpp::Time Time_trg;

    geometry_msgs::msg::Pose Pose_B_O_trg;
    geometry_msgs::msg::Twist Twist_B_O_trg;
    geometry_msgs::msg::Vector3 Eul_B_O_trg;
    double Vel_mag_B_O_trg = NAN;
    double Vel_angle_B_O_trg = NAN;

    geometry_msgs::msg::Pose Pose_P_B_trg;
    geometry_msgs::msg::Twist Twist_B_P_trg;
    geometry_msgs::msg::Vector3 Eul_P_B_trg;

    double Vel_mag_B_P_trg = NAN;
    double Vel_angle_B_P_trg = NAN;
    double D_perp_trg = NAN;
    double D_perp_CR_trg = NAN;

    geometry_msgs::msg::Vector3 Optical_Flow_trg;
    double Tau_trg = NAN;
    double Tau_CR_trg = NAN;
    double Theta_x_trg = NAN;
    double Theta_y_trg = NAN;

    //boost::array<double,4> NN_Output_trg{NAN,NAN,NAN,NAN};
    double a_Trg_trg = NAN;
    double a_Rot_trg = NAN;

    // =======================
    //   ONBOARD IMPACT DATA
    // =======================
    bool Impact_Flag_OB = false;
    bool OnceFlag_Impact_OB = false;
    rclcpp::Time Time_impact_OB;

    geometry_msgs::msg::Pose Pose_B_O_impact_OB;
    geometry_msgs::msg::Vector3 Eul_B_O_impact_OB;

    geometry_msgs::msg::Twist Twist_B_P_impact_OB;
    geometry_msgs::msg::Vector3 Eul_P_B_impact_OB;
    float Accel_B_O_Mag_impact_OB = NAN;


    // ==========================
    //    EXTERNAL IMPACT DATA
    // ==========================
    bool Impact_Flag_Ext = false;
    rclcpp::Time Time_impact_Ext;

    geometry_msgs::msg::Pose Pose_B_O_impact_Ext;
    geometry_msgs::msg::Vector3 Eul_B_O_impact_Ext;

    geometry_msgs::msg::Twist Twist_B_P_impact_Ext;
    geometry_msgs::msg::Vector3 Eul_P_B_impact_Ext;
    float Rot_Sum_impact_Ext = NAN;

    bool BodyContact_Flag = false;
    bool ForelegContact_Flag = false;
    bool HindlegContact_Flag = false;
    bool OnceFlag_Impact = false;
    std::string BodyCollision_str = "SAR_Body::Body_Collision_";
    std::string LegCollision_str = "Leg_Collision_";



    // ==================
    //     MISC DATA
    // ==================

    double V_battery = 4.0;

    uint8_t Pad1_Contact = 0;
    uint8_t Pad2_Contact = 0;
    uint8_t Pad3_Contact = 0;
    uint8_t Pad4_Contact = 0;

    uint8_t Pad_Connections = 0;

    // ============================
    //     DATA PUBLISH OBJECTS
    // ============================
    rclcpp::Publisher<sar_msgs::msg::SARStateData>::SharedPtr StateData_Pub;

    sar_msgs::msg::SARStateData StateData_msg;

    // ====================
    //     DEBUG VALUES
    // ====================

    bool Tumbled_Flag = false;
    bool TumbleDetect_Flag = false;
    bool MotorStop_Flag = false;
    bool AngAccel_Flag = false;
    bool Armed_Flag = false;
    bool CustomThrust_Flag = false;
    bool CustomMotorCMD_Flag = false;

    bool Pos_Ctrl_Flag = false;
    bool Vel_Ctrl_Flag = false;
    bool Policy_Armed_Flag = false;

    bool CamActive_Flag = false;

    // SIM
    bool Sticky_Flag = false;
};

// CONVERT QUATERNION TO EULER ANGLES (YZX NOTATION)
inline void SAR_DataConverter::quat2euler(float quat[], float eul[]){

    float R11,R21,R31,R22,R23;
    float phi,theta,psi; // (phi=>x,theta=>y,psi=>z)

    // CALC NEEDED ROTATION MATRIX COMPONENTS FROM QUATERNION
    R11 = 1.0 - 2.0*(pow(quat[1],2) + pow(quat[2],2) );
    R21 = 2.0*(quat[0]*quat[1] + quat[2]*quat[3]);
    R31 = 2.0*(quat[0]*quat[2] - quat[1]*quat[3]);

    R22 = 1.0 - 2.0*( pow(quat[0],2) + pow(quat[2],2) );
    R23 = 2.0*(quat[1]*quat[2] - quat[0]*quat[3]);

    // CONVERT ROTATION MATRIX COMPONENTS TO EULER ANGLES (YZX NOTATION)
    phi = atan2(-R23, R22);
    theta = atan2(-R31, R11);
    psi = asin(R21);

    eul[0] = phi;   // X-axis
    eul[1] = theta; // Y-axis
    eul[2] = psi;   // Z-axis
}

// CONVERTS A SET OF EULER ANGLES TO QUATERNION IN (ZYX NOTATION)
inline void SAR_DataConverter::euler2quat(float quat[],float eul[]) {

	// Abbreviations for the various angular functions

    double cx = cos(eul[0] * 0.5);
    double cy = cos(eul[1] * 0.5);
    double cz = cos(eul[2] * 0.5);

    double sx = sin(eul[0] * 0.5);
    double sy = sin(eul[1] * 0.5);
    double sz = sin(eul[2] * 0.5);

    quat[0] = sx * cy * cz - cx * sy * sz;
    quat[1] = cx * sy * cz + sx * cy * sz;
    quat[2] = cz * cz * sz - sx * sy * cz;
    quat[3] = cx * cy * cz + sx * sy * sz;

}

