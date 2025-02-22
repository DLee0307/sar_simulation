#include "SAR_DataConverter.h"


/*
void SAR_DataConverter::RL_Data_Callback(const sar_msgs::RL_Data::ConstPtr &msg)
{

    K_ep = msg->K_ep;
    K_run = msg->K_run;
    n_rollouts = msg->n_rollouts;

    mu = msg->mu;
    sigma = msg->sigma;
    // policy = msg->policy;

    reward = msg->reward;
    reward_vals = msg->reward_vals;

    // vel_d = msg->vel_d;


    if(msg->trialComplete_flag == true)
    {
        Time_start = ros::Time::now();
    }
}
*/
void SAR_DataConverter::Publish_StateData()
{

    // ===================
    //     FLIGHT DATA
    // ===================
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    //@@@rclcpp::Duration Time_delta = clock->now() - Time_start;
    rclcpp::Duration Time_delta = clock_->now() - Time_start;
    //rclcpp::Duration Time_delta = Time - Time_start;
    StateData_msg.time.sec = Time_delta.seconds();
    StateData_msg.time.nanosec = Time_delta.nanoseconds();

    //ros::Duration Time_delta(Time-Time_start);
    //StateData_msg.Time.data.sec = Time_delta.sec;
    //StateData_msg.Time.data.nsec = Time_delta.nsec;

    // STATES WRT ORIGIN
    StateData_msg.pose_b_o = Pose_B_O;
    StateData_msg.twist_b_o = Twist_B_O;
    StateData_msg.accel_b_o = Accel_B_O;
    StateData_msg.eul_b_o = Eul_B_O;
    StateData_msg.accel_b_o_mag = Accel_B_O_Mag;

    // STATES WRT PLANE
    StateData_msg.pose_p_b = Pose_P_B;
    StateData_msg.twist_b_p = Twist_B_P;
    StateData_msg.eul_p_b = Eul_P_B;
    StateData_msg.vel_mag_b_p = Vel_mag_B_P;
    StateData_msg.vel_angle_b_p = Vel_angle_B_P;
    StateData_msg.d_perp = D_perp;
    StateData_msg.d_perp_cr = D_perp_CR;
    StateData_msg.d_perp_pad = D_perp_pad;
    StateData_msg.d_perp_pad_min = D_perp_pad_min;


    // OPTICAL FLOW STATES
    StateData_msg.optical_flow = Optical_Flow;
    StateData_msg.optical_flow_cam = Optical_Flow_Cam;
    StateData_msg.tau_cr = Tau_CR;

    // STATE SETPOINTS
    StateData_msg.x_d = x_d;
    StateData_msg.v_d = v_d;
    StateData_msg.a_d = a_d;

    // CONTROL ACTIONS
    StateData_msg.fm = FM;
    StateData_msg.motorthrusts = MotorThrusts;
    StateData_msg.motor_cmd = Motor_CMD;

    // POLICY ACTIONS
    //StateData_msg.NN_Output = NN_Output;
    StateData_msg.a_trg = a_Trg;
    StateData_msg.a_rot = a_Rot;


    // PUBLISH STATE DATA RECEIVED FROM CONTROLLER
    StateData_Pub->publish(StateData_msg);
    //std::cout << "StateData is published: " << Pose_B_O.position.z << std::endl; 
}

void SAR_DataConverter::Publish_TriggerData()
{
    //ros::Duration Time_delta(Time_trg-Time_start);
    //TriggerData_msg.Time_trg.data.sec = Time_delta.sec;
    //TriggerData_msg.Time_trg.data.nsec = Time_delta.nsec;

    // STATES WRT ORIGIN
    TriggerData_msg.pose_b_o_trg = Pose_B_O_trg;
    TriggerData_msg.twist_b_o_trg = Twist_B_O_trg;
    TriggerData_msg.eul_b_o_trg = Eul_B_O_trg;

    TriggerData_msg.vel_mag_b_o_trg = Vel_mag_B_O_trg;
    TriggerData_msg.vel_angle_b_o_trg = Vel_angle_B_O_trg;

    // STATES WRT PLANE
    TriggerData_msg.pose_p_b_trg = Pose_P_B_trg;
    TriggerData_msg.twist_b_p_trg = Twist_B_P_trg;
    TriggerData_msg.eul_p_b_trg = Eul_P_B_trg;

    TriggerData_msg.vel_mag_b_p_trg = Vel_mag_B_P_trg;
    TriggerData_msg.vel_angle_b_p_trg = Vel_angle_B_P_trg;
    TriggerData_msg.d_perp_trg = D_perp_trg;
    TriggerData_msg.d_perp_cr_trg = D_perp_CR_trg;

    // OPTICAL FLOW STATES
    TriggerData_msg.optical_flow_trg = Optical_Flow_trg;
    TriggerData_msg.tau_cr_trg = Tau_CR_trg;
    TriggerData_msg.tau_cm_trg = Tau_CM_trg;
    TriggerData_msg.theta_x_cm_trg = Theta_x_CM_trg;
    TriggerData_msg.tau_dh_trg = Tau_DH_trg;

    // POLICY ACTIONS
    //TriggerData_msg.nn_output_trg = NN_Output_trg;
    TriggerData_msg.a_trg_trg = a_Trg_trg;
    TriggerData_msg.a_rot_trg = a_Rot_trg;

    TriggerData_Pub->publish(TriggerData_msg);

}

void SAR_DataConverter::Publish_ImpactData()
{
    ImpactData_msg.impact_flag = Impact_Flag;

    // ONBOARD IMPACT DATA
    //ros::Duration Time_delta_OB(Time_impact_OB-Time_start);
    //ImpactData_msg.Time_impact_OB.data.sec = Time_delta_OB.sec;
    //ImpactData_msg.Time_impact_OB.data.nsec = Time_delta_OB.nsec;

    ImpactData_msg.impact_flag_ob = Impact_Flag_OB;

    ImpactData_msg.pose_b_o_impact_ob = Pose_B_O_impact_OB;
    ImpactData_msg.eul_b_o_impact_ob = Eul_B_O_impact_OB;
    ImpactData_msg.twist_b_p_impact_ob = Twist_B_P_impact_OB;
    ImpactData_msg.eul_p_b_impact_ob = Eul_P_B_impact_OB;

    // EXTERNAL IMPACT DATA
    //ros::Duration Time_delta_Ext(Time_impact_Ext-Time_start);
    //ImpactData_msg.Time_impact_Ext.data.sec = Time_delta_Ext.sec;
    //ImpactData_msg.Time_impact_Ext.data.nsec = Time_delta_Ext.nsec;

    ImpactData_msg.impact_flag_ext = Impact_Flag_Ext;
    ImpactData_msg.bodycontact_flag = BodyContact_Flag;
    ImpactData_msg.cameracontact_flag = CameraContact_Flag;
    ImpactData_msg.forelegcontact_flag = ForelegContact_Flag;
    ImpactData_msg.hindlegcontact_flag = HindlegContact_Flag;

    ImpactData_msg.pose_b_o_impact_ext = Pose_B_O_impact_Ext;
    ImpactData_msg.eul_b_o_impact_ext = Eul_B_O_impact_Ext;
    
    ImpactData_msg.twist_b_p_impact_ext = Twist_B_P_impact_Ext;
    ImpactData_msg.eul_p_b_impact_ext = Eul_P_B_impact_Ext;
    ImpactData_msg.rot_sum_impact_ext = Rot_Sum_impact_Ext;

    // IMPACT FORCES
    ImpactData_msg.force_impact.x = Force_Impact_x;
    ImpactData_msg.force_impact.y = Force_Impact_y;
    ImpactData_msg.force_impact.z = Force_Impact_z;
    ImpactData_msg.impact_magnitude = Impact_Magnitude;


    // STICKY PAD CONTACTS
    ImpactData_msg.pad_connections = Pad_Connections;
    // std::cout << "Pad_Connections: " << Pad_Connections << std::endl; 

    ImpactData_msg.pad1_contact = Pad1_Contact;
    ImpactData_msg.pad2_contact = Pad2_Contact;
    ImpactData_msg.pad3_contact = Pad3_Contact;
    ImpactData_msg.pad4_contact = Pad4_Contact;
    // std::cout << "Pad1_Contact: " << Pad1_Contact << std::endl; 
    // std::cout << "Pad2_Contact: " << Pad2_Contact << std::endl; 
    // std::cout << "Pad3_Contact: " << Pad3_Contact << std::endl; 
    // std::cout << "Pad4_Contact: " << Pad4_Contact << std::endl; 


    ImpactData_Pub->publish(ImpactData_msg);
}

void SAR_DataConverter::Publish_MiscData()
{

    //MiscData_msg.header.stamp = ros::Time::now();
    MiscData_msg.battery_voltage = V_battery;
    MiscData_msg.plane_angle = Plane_Angle_deg;
    MiscData_msg.plane_pos.x = Plane_Pos.x;
    MiscData_msg.plane_pos.y = Plane_Pos.y;
    MiscData_msg.plane_pos.z = Plane_Pos.z;


    
    MiscData_Pub->publish(MiscData_msg);

}