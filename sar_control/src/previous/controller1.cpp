#include "controller1.h"

ThrustController::Trajectory_Type ThrustController::Traj_Type = ThrustController::NONE;
ThrustController::axis_direction ThrustController::axis = ThrustController::x_axis;


ThrustController::ThrustController()
: Node("thrust_controller"), trigger_(0)
{
    // ===========================
    //     ROS TOPICS/SERVICES
    // ===========================

    // EXTERNAL SENSOR SUBSCRIBERS
    subscriber_Vicon = this->create_subscription<sar_msgs::msg::ViconData>("Vicon/data", 1, std::bind(&ThrustController::ThrustControl, this, std::placeholders::_1));

    // INTERNAL SENSOR SUBSCRIBERS
    subscriber_IMU = this->create_subscription<sar_msgs::msg::IMUData>("imu/data", 1, std::bind(&ThrustController::IMU_Update_Callback, this, std::placeholders::_1));


    this->publisher = this->create_publisher<sar_msgs::msg::MotorThrust>("thrust_input_topic", 10);

/* Bryan
    // MISC SERVICES/PUBLISHERS
    CTRL_Data_Publisher = nh->advertise<sar_msgs::CTRL_Data>("/CTRL/data",1);
    CTRL_Debug_Publisher = nh->advertise<sar_msgs::CTRL_Debug>("CTRL/debug",1);
    CTRL_CMD_Service = nh->advertiseService("/CTRL/Cmd_ctrl",&Controller::CMD_Service_Resp,this);
    Get_Obs_Service = nh->advertiseService("/CTRL/Get_Obs",&Controller::Get_Obs_Resp,this);
    SAR_DC_Subscriber = nh->subscribe("/SAR_DC/MiscData",1,&Controller::Plane_Pose_Callback,this,ros::TransportHints().tcpNoDelay());



    // Thread main controller loop so other callbacks can work fine
    appThread = std::thread(&Controller::appLoop, this);
    controllerThread = std::thread(&Controller::stabilizerLoop, this);
*/

    service_ = this->create_service<sar_msgs::srv::SetTrigger>("set_trigger", std::bind(&ThrustController::set_trigger, this, std::placeholders::_1, std::placeholders::_2));
    CMD_Output_Service = this->create_service<sar_msgs::srv::CTRLCmdSrv>("/CTRL/Cmd_ctrl", std::bind(&ThrustController::CMD_Service_Resp, this, std::placeholders::_1, std::placeholders::_2));
}   

void ThrustController::ThrustControl(const sar_msgs::msg::ViconData::SharedPtr msg) {
    Vicon_Data = msg;
    std::array<double, 4> thrusts;

    if (trigger_ == 0) {
        //loadInitParams();
        x_d = {0,0,0};
      
        thrusts = controlOutput(); 
        std::cout << "trigger_ : 0 " << std::endl;
    } else if (trigger_ == 1) {
        thrusts = {0, 0, 0, 0};
        std::cout << "trigger_ : 1 " << std::endl;
    } else if (trigger_ == 2) {
        thrusts = {0, 10, 10, 0};
        std::cout << "trigger_ : 2 " << std::endl;
    } else if (trigger_ == 3){
        x_d = x_d_d;
        thrusts = controlOutput();   
        std::cout << "trigger_ : 3 " << std::endl;
    } else if (trigger_ == 10){
        std::cout << "trigger_ : 10 " << std::endl;
        if(A == 0.0f){
            t_traj[0] = 0.0f; // Reset timer
            t_traj[1] = 0.0f; // Reset timer
            t_traj[2] = 0.0f; // Reset timer
            s_0_t[0] = Vicon_Data->pose.position.x-0.0325; // Starting position [m]
            s_0_t[1] = Vicon_Data->pose.position.y+0.0325; // Starting position [m]
            s_0_t[2] = Vicon_Data->pose.position.z; // Starting position [m]              
        }
        
        Traj_Active[0] = true;
        Traj_Active[1] = true;
        Traj_Active[2] = true;
        


        s_f_t[0] = x_d_d.x;  // Ending position [m]
        s_f_t[1] = x_d_d.y;  // Ending position [m]
        s_f_t[2] = x_d_d.z;  // Ending position [m]

        a_t[0] = TrajAcc_Max[0];  // Peak acceleration [m/s^2]
        a_t[1] = TrajAcc_Max[1];  // Peak acceleration [m/s^2]
        a_t[2] = TrajAcc_Max[2];  // Peak acceleration [m/s^2]

        //t_traj[0] = 0.0f; // Reset timer
        //t_traj[1] = 0.0f; // Reset timer
        //t_traj[2] = 0.0f; // Reset timer            

        T[0] = sqrtf(6.0f/a_t[0]*fabsf(s_f_t[0] - s_0_t[0])); // Calc trajectory manuever time [s]
        std::cout << "T[0] : "<< T[0] << std::endl;
        T[1] = sqrtf(6.0f/a_t[1]*fabsf(s_f_t[1] - s_0_t[1])); // Calc trajectory manuever time [s]
        std::cout << "T[1] : "<< T[1] << std::endl;
        T[2] = sqrtf(6.0f/a_t[2]*fabsf(s_f_t[2] - s_0_t[2])); // Calc trajectory manuever time [s]
        std::cout << "T[2] : "<< T[2] << std::endl;


        point2point_Traj();
        thrusts = controlOutput();   
        A = 1.0f;

    } else if (trigger_ ==11){
        std::cout << "trigger_ : 11 " << std::endl;
        if(A == 0.0f){
            t_traj[0] = 0.0f; // Reset timer
            t_traj[1] = 0.0f; // Reset timer
            t_traj[2] = 0.0f; // Reset timer  
            s_0_t[0] = Vicon_Data->pose.position.x-0.0325; // Starting position [m]
            s_0_t[1] = Vicon_Data->pose.position.y+0.0325; // Starting position [m]
            s_0_t[2] = Vicon_Data->pose.position.z; // Starting position [m]
        }


        float x_d_d_y_rad = x_d_d.y * M_PI / 180.0;
        v_t[0] = x_d_d.x * std::cos(x_d_d_y_rad); // Desired velocity [m/s]
        std::cout << "v_t[0] : "<< v_t[0] << std::endl;
        v_t[1] = x_d_d.x * std::cos(x_d_d_y_rad);  // Desired velocity [m/s]
        v_t[2] = x_d_d.x * std::sin(x_d_d_y_rad);  // Desired velocity [m/s]
        std::cout << "v_t[2] : "<< v_t[2] << std::endl;

        a_t[0] = TrajAcc_Max[0];  // Peak acceleration [m/s^2]
        a_t[1] = TrajAcc_Max[1];  // Peak acceleration [m/s^2]
        a_t[2] = TrajAcc_Max[2];  // Peak acceleration [m/s^2]
        
        const_velocity_Traj();
        thrusts = controlOutput();   
        A = 1.0f;
        
    } else {
        //loadInitParams();
        x_d = {0,0,0};
        thrusts = controlOutput(); 
        std::cout << "trigger_ : else " << std::endl;        
    }
    
    
    publish_thrust_input(publisher, thrusts);
    /*std::cout << "position(x)_Vicon : " << msg->pose.position.x << std::endl;
    std::cout << "position(y)_Vicon : " << msg->pose.position.y << std::endl;
    std::cout << "position(z)_Vicon : " << msg->pose.position.z << std::endl;
    std::cout << "orientation(x)_Vicon : " << msg->pose.orientation.x << std::endl;
    std::cout << "orientation(y)_Vicon : " << msg->pose.orientation.y << std::endl;
    std::cout << "orientation(z)_Vicon : " << msg->pose.orientation.z << std::endl;   
    std::cout << "vel(x)_Vicon : " << msg->vel.x << std::endl;
    std::cout << "vel(y)_Vicon : " << msg->vel.y << std::endl;
    std::cout << "vel(z)_Vicon : " << msg->vel.z << std::endl;*/
}

void ThrustController::IMU_Update_Callback(const sar_msgs::msg::IMUData::SharedPtr msg) {
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
/*
void ThrustController::Vicon_Update_Callback(const sar_msgs::msg::ViconData::SharedPtr msg) {
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
*/
void ThrustController::publish_thrust_input(rclcpp::Publisher<sar_msgs::msg::MotorThrust>::SharedPtr& publisher, std::array<double, 4> thrust_value) {
    auto message = sar_msgs::msg::MotorThrust();
    message.motorthrust = thrust_value;
    publisher->publish(message);
}

void ThrustController::CMD_Service_Resp(const sar_msgs::srv::CTRLCmdSrv::Request::SharedPtr request,
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
                        
    std::cout << "cmd_type: " << request->cmd_type <<  std::endl;
    std::cout << "cmd_val1: " << request->cmd_vals.x <<  std::endl;
    std::cout << "cmd_val2: " << request->cmd_vals.y <<  std::endl;
    std::cout << "cmd_val3: " << request->cmd_vals.z <<  std::endl;
    std::cout << "cmd_flag: " << request->cmd_flag <<  std::endl;
    std::cout << "cmd_rx: " << request->cmd_rx <<  std::endl;

    
    CTRL_Command(&CTRL_Cmd);
}
void ThrustController::set_trigger(const sar_msgs::srv::SetTrigger::Request::SharedPtr request,
                     sar_msgs::srv::SetTrigger::Response::SharedPtr response) {
    trigger_ = request->data;
    response->success = true;
    response->message = "Trigger value set to " + std::to_string(trigger_);

    if (!request->vector.empty()) {
        RCLCPP_INFO(this->get_logger(), "Received vector: ");
        for (const auto& value : request->vector) {
            x_d_d = {static_cast<float>(request->vector[0]), static_cast<float>(request->vector[1]), static_cast<float>(request->vector[2])};
            RCLCPP_INFO(this->get_logger(), "%f ", value);
        }
    }
}

void ThrustController::CTRL_Command(struct CTRL_CmdPacket *CTRL_Cmd)
{
    switch(CTRL_Cmd->cmd_type){
        case 0: // Reset
            std::cout << "Case 0:" << std::endl;

            break;

        case 1: // Position
            x_d.x = CTRL_Cmd->cmd_val1;
            x_d.y = CTRL_Cmd->cmd_val2;
            x_d.z = CTRL_Cmd->cmd_val3;
            kp_xf = CTRL_Cmd->cmd_flag;
            break;
        
        case 2: // Velocity
            v_d.x = CTRL_Cmd->cmd_val1;
            v_d.y = CTRL_Cmd->cmd_val2;
            v_d.z = CTRL_Cmd->cmd_val3;
            kd_xf = CTRL_Cmd->cmd_flag;
            break;

        case 5: // Hard Set All Motorspeeds to Zero
            //std::cout << "Case 5:" << std::endl;

            break;   

        case 10: // Point-to-Point Trajectory
            std::cout << "Case 10:" << std::endl;

            break;  
    
    
    }
}

std::array<double, 4> ThrustController::controlOutput()
{
    //std::cout << "orientation.x: " << Imu_Data->orientation.x << std::endl;
    struct mat33 J;
    J.m[0][0] = Ixx;
    J.m[0][1] = 0;
    J.m[0][2] = 0;
    J.m[1][0] = 0;
    J.m[1][1] = Iyy;
    J.m[1][2] = 0;
    J.m[2][0] = 0;
    J.m[2][1] = 0;
    J.m[2][2] = Izz;

    //printmat(J);

    // CONTROL GAINS
    Kp_p = mkvec(P_kp_xy,P_kp_xy,P_kp_z);
    //printvec(Kp_p);
    Kd_p = mkvec(P_kd_xy,P_kd_xy,P_kd_z);
    //printvec(Kd_p);
    Ki_p = mkvec(P_ki_xy,P_ki_xy,P_ki_z);
    //printvec(Ki_p);

    Kp_R = mkvec(R_kp_xy,R_kp_xy,R_kp_z);
    Kd_R = mkvec(R_kd_xy,R_kd_xy,R_kd_z);
    Ki_R = mkvec(R_ki_xy,R_ki_xy,R_ki_z);


    //!! need to analyze
    // =========== STATE SETPOINTS =========== //
    omega_d = mkvec(0.0f,0.0f,0.0f);        // Omega-desired [rad/s]
    domega_d = mkvec(0.0f,0.0f,0.0f);       // Omega-Accl. [rad/s^2]
    quat_d = mkquat(0.0f,0.0f,0.0f,1.0f);   // Desired orientation 
    //!!
    
    if (!Imu_Data) {
        RCLCPP_WARN(this->get_logger(), "IMU Data is not initialized.");
        // IMU Data initialize manually.
        Imu_Data = std::make_shared<sar_msgs::msg::IMUData>();
        Imu_Data->orientation.x = 0.0;
        Imu_Data->orientation.y = 0.0;
        Imu_Data->orientation.z = 0.0;
        Imu_Data->orientation.w = 1.0;
    }

    // =========== ROTATION MATRIX =========== //
    // R changes Body axes to be in terms of Global axes
    // https://www.andre-gaschler.com/rotationconverter/
    Quat_B_O = mkquat(Imu_Data->orientation.x,Imu_Data->orientation.y,Imu_Data->orientation.z,Imu_Data->orientation.w);
    //std::cout << "orientation.x: " << Imu_Data->orientation.x << std::endl;
    //std::cout << "orientation.y: " << Imu_Data->orientation.y << std::endl;
    //std::cout << "orientation.z: " << Imu_Data->orientation.z << std::endl;
    //std::cout << "orientation.w: " << Imu_Data->orientation.w << std::endl;
    
    R = quat2rotmat(Quat_B_O); // Quaternion to Rotation Matrix Conversion
    //printmat(R);
    //!! R = IMU orientation data?
    b3 = mvmul(R, e_3);         // Current body vertical axis in terms of global axes | [b3 = R*e_3] 
    //printvec(b3);


    // =========== TRANSLATIONAL EFFORT =========== //
    Pos_B_O = mkvec(Vicon_Data->pose.position.x-0.0325,Vicon_Data->pose.position.y+0.0325,Vicon_Data->pose.position.z);
    //printvec(Pos_B_O);
    e_x = vsub(Pos_B_O, x_d); // [e_x = pos-x_d]    


    Vel_B_O = mkvec(Vicon_Data->vel.x,Vicon_Data->vel.y,Vicon_Data->vel.z);
    //printvec(Vel_B_O);
    e_v = vsub(Vel_B_O, v_d); // [e_v = vel-v_d]

    // POS. INTEGRAL ERROR
    e_PI.x += (e_x.x)*dt;
    e_PI.x = clamp(e_PI.x, -i_range_xy, i_range_xy);
    //std::cout << "dt: " << dt << std::endl;

    e_PI.y += (e_x.y)*dt;
    e_PI.y = clamp(e_PI.y, -i_range_xy, i_range_xy);

    e_PI.z += (e_x.z)*dt;
    e_PI.z = clamp(e_PI.z, -i_range_z, i_range_z);
    
    /* [F_thrust_ideal = -kp_x*e_x*(kp_x_flag) + -kd_x*e_v + -kI_x*e_PI*(kp_x_flag) + m*g*e_3 + m*a_d] */


    
    /*!!
    math3d.h : https://github.com/jpreiss/cmath3d/blob/master/math3d.h
    clamp = (A min max) if A < min, then min. if A > max, then max
    veltmul = element-wise vector multiply.
    vneg = -1 * vector
    vscl = scalar * vector
    vadd3 = Adding three vector
    mvmul = multiply a matrix by a vector.
    mmul = multiply two matrice.
    dehat = make from matrix to vector
    */  

    temp1_v = veltmul(vneg(Kp_p), e_x);
    temp1_v = vscl(kp_xf,temp1_v);
    temp2_v = veltmul(vneg(Kd_p), e_v);
    temp2_v = vscl(kd_xf,temp2_v); // typo?
    temp3_v = veltmul(vneg(Ki_p), e_PI);
    P_effort = vadd3(temp1_v,temp2_v,temp3_v);
    temp1_v = vscl(m*g, e_3); // Feed-forward term
    //std::cout << "m(mass): " << m << std::endl;
    temp2_v = vscl(m, a_d);

    F_thrust_ideal = vadd3(P_effort, temp1_v,temp2_v); 
    //printvec(F_thrust_ideal);
    // =========== DESIRED BODY AXES =========== // 
    b3_d = vnormalize(F_thrust_ideal);

    b2_d = vnormalize(vcross(b3_d, b1_d));      // [b3_d x b1_d] | body-fixed horizontal axis
    temp1_v = vnormalize(vcross(b2_d, b3_d));
    R_d = mcolumns(temp1_v, b2_d, b3_d);        // Desired rotation matrix from calculations

    // =========== ROTATIONAL ERRORS =========== // 
    RdT_R = mmul(mtranspose(R_d), R);       // [R_d'*R]
    RT_Rd = mmul(mtranspose(R), R_d);       // [R'*R_d]
    temp1_v = dehat(msub(RdT_R, RT_Rd));    // [dehat(R_d'*R - R'*R)]
    e_R = vscl(0.5f, temp1_v);              // Rotation error | [eR = 0.5*dehat(R_d'*R - R'*R)]

    // 회전 오차=1/2​(trace(RdT​R)−1)=1/2((aA+dD+gG)+(bB+eE+hH)+(cC+fF+iI)−1)

    temp1_v = mvmul(RT_Rd, omega_d);        // [R'*R_d*omega_d]
    Omega_B_O = mkvec(Imu_Data->angular_velocity.x,Imu_Data->angular_velocity.y,Imu_Data->angular_velocity.z);  
    //printvec(Omega_B_O);
    e_w = vsub(Omega_B_O, temp1_v);        // Ang. vel error | [e_w = omega - R'*R_d*omega_d] 

    // ROT. INTEGRAL ERROR
    e_RI.x += (e_R.x)*dt;
    e_RI.x = clamp(e_RI.x, -i_range_R_xy, i_range_R_xy);

    e_RI.y += (e_R.y)*dt;
    e_RI.y = clamp(e_RI.y, -i_range_R_xy, i_range_R_xy);

    e_RI.z += (e_R.z)*dt;
    e_RI.z = clamp(e_RI.z, -i_range_R_z, i_range_R_z);

    // =========== CONTROL EQUATIONS =========== // 
    /* [M = -kp_R*e_R - kd_R*e_w -ki_R*e_RI + Gyro_dyn] */

    temp1_v = veltmul(vneg(Kp_R), e_R);     // [-kp_R*e_R]
    temp2_v = veltmul(vneg(Kd_R), e_w);     // [-kd_R*e_w]
    temp3_v = veltmul(vneg(Ki_R), e_RI);    // [-ki_R*e_RI]
    R_effort = vadd3(temp1_v,temp2_v,temp3_v);
    
    /* Gyro_dyn = [omega x (J*omega)] - [J*( hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d )] */
    temp1_v = vcross(Omega_B_O, mvmul(J, Omega_B_O)); // [omega x J*omega]
    //printvec(Omega_B_O);

    temp1_m = mmul(hat(Omega_B_O), RT_Rd); //  hat(omega)*R'*R_d
    temp2_v = mvmul(temp1_m, omega_d);      // (hat(omega)*R'*R_d)*omega_d
    temp3_v = mvmul(RT_Rd, domega_d);       // (R'*R_d*domega_d)

    temp4_v = mvmul(J, vsub(temp2_v, temp3_v)); // J*(hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d)
    Gyro_dyn = vsub(temp1_v,temp4_v);

    F_thrust = vdot(F_thrust_ideal, b3);    // Project ideal thrust onto b3 vector [N]
    //std::cout << "F_thrust: " << F_thrust << std::endl; 
    M = vadd(R_effort,Gyro_dyn);            // Control moments [Nm]
    //printvec(M);

    // MOTOR MIXING (GTC_Derivation_V2.pdf) 
    M1_thrust = F_thrust * Prop_23_x/(Prop_14_x + Prop_23_x) - M.x * 1/(Prop_14_y + Prop_23_y) - M.y * 1/(Prop_14_x + Prop_23_x) - M.z * Prop_23_y/(C_tf*(Prop_14_y + Prop_23_y));
    M2_thrust = F_thrust * Prop_14_x/(Prop_14_x + Prop_23_x) - M.x * 1/(Prop_14_y + Prop_23_y) + M.y * 1/(Prop_14_x + Prop_23_x) + M.z * Prop_14_y/(C_tf*(Prop_14_y + Prop_23_y));
    M3_thrust = F_thrust * Prop_14_x/(Prop_14_x + Prop_23_x) + M.x * 1/(Prop_14_y + Prop_23_y) + M.y * 1/(Prop_14_x + Prop_23_x) - M.z * Prop_14_y/(C_tf*(Prop_14_y + Prop_23_y));
    M4_thrust = F_thrust * Prop_23_x/(Prop_14_x + Prop_23_x) + M.x * 1/(Prop_14_y + Prop_23_y) - M.y * 1/(Prop_14_x + Prop_23_x) + M.z * Prop_23_y/(C_tf*(Prop_14_y + Prop_23_y));
    
    // CLAMP AND CONVERT THRUST FROM [N] AND [N*M] TO [g]
    M1_thrust = clamp((M1_thrust/2.0f)*Newton2g,0.0f,Thrust_max);
    M2_thrust = clamp((M2_thrust/2.0f)*Newton2g,0.0f,Thrust_max);
    M3_thrust = clamp((M3_thrust/2.0f)*Newton2g,0.0f,Thrust_max);
    M4_thrust = clamp((M4_thrust/2.0f)*Newton2g,0.0f,Thrust_max);

    //std::cout << "M1_thrust: " << M1_thrust << std::endl; 
    //std::cout << "M2_thrust: " << M2_thrust << std::endl; 
    //std::cout << "M3_thrust: " << M3_thrust << std::endl; 
    //std::cout << "M4_thrust: " << M4_thrust << std::endl; 

    //M1_thrust = 10;
    //M2_thrust = 10;
    //M3_thrust = 10;
    //M4_thrust = 10;

    return {M1_thrust, M2_thrust, M3_thrust, M4_thrust};
}


struct mat33 ThrustController::hat(struct vec v) {
    struct mat33 m;
    m.m[0][0] = 0;
    m.m[0][1] = -v.z;
    m.m[0][2] = v.y;
    m.m[1][0] = v.z;
    m.m[1][1] = 0;
    m.m[1][2] = -v.x;
    m.m[2][0] = -v.y;
    m.m[2][1] = v.x;
    m.m[2][2] = 0;
    return m;
}

struct vec ThrustController::dehat(struct mat33 m) {
    struct vec v;
    v.x = m.m[2][1];
    v.y = m.m[0][2];
    v.z = m.m[1][0];
    return v;
}

struct vec ThrustController::quat2eul(struct quat q) {
    struct vec eul;
    float R11,R21,R31,R22,R23;

    R11 = 1.0f - 2.0f*( fsqr(q.y) + fsqr(q.z) );
    R21 = 2.0f*(q.x*q.y + q.z*q.w);
    R31 = 2.0f*(q.x*q.z - q.y*q.w);

    R22 = 1.0f - 2.0f*( fsqr(q.x) + fsqr(q.z) );
    R23 = 2.0f*(q.y*q.z - q.x*q.w);

    eul.x = atan2f(-R23,R22); 	// Roll
    eul.y = atan2f(-R31,R11); 	// Pitch
    eul.z = asinf(R21); 		// Yaw

    return eul;
}

void ThrustController::printvec(struct vec v){
    std::cout << v.x << ", " << v.y << ", " << v.z << std::endl;
}

void ThrustController::printquat(struct quat q){
    std::cout << q.x << ", " << q.y << ", " << q.z << ", " << q.w << std::endl;
}

void ThrustController::printmat(struct mat33 m){
    struct vec vrow_0 = mrow(m,0);
    struct vec vrow_1 = mrow(m,1);
    struct vec vrow_2 = mrow(m,2);

    printvec(vrow_0);
    printvec(vrow_1);
    printvec(vrow_2);
    std::cout << std::endl;
}

void ThrustController::set_vec_element(struct vec *v, int index, float value) {

    switch (index) {
        case 0:
            v->x = value;
            break;
        case 1:
            v->y = value;
            break;
        case 2:
            v->z = value;
            break;
        default:
            // Handle invalid index if necessary
            break;
    }
}

// Third-order polynomial time scaling.
void ThrustController::point2point_Traj()
{

    for(int i = 0; i<3; i++)
    {
        // CALCULATE ONLY DESIRED TRAJECTORIES
        if(Traj_Active[i] == true)
        {
            float t = t_traj[i];

            if(t_traj[i] <= T[i] && T[i] != 0.0f) // SKIP CALC IF ALREADY AT END POSITION
            {
                // CALCULATE TIME SCALING VALUE S(t)
                float s_t = (3*powf(t,2)/powf(T[i],2) - 2*powf(t,3)/powf(T[i],3));
                float ds_t = (6*t/powf(T[i],2) - 6*powf(t,2)/powf(T[i],3));
                float dds_t = (6/powf(T[i],2) - 12*t/powf(T[i],3));

                // CONVERT PATH VALUES X(S) TO TRAJECTORY VALUES X(S(t))
                float pos_val = s_0_t[i] +  s_t * (s_f_t[i] - s_0_t[i]);
                float vel_val = ds_t * (s_f_t[i] - s_0_t[i]);
                float acc_val = dds_t * (s_f_t[i] - s_0_t[i]);

                // UPDATE DESIRED STATE VECTORS
                set_vec_element(&x_d, i, pos_val);
                set_vec_element(&v_d, i, vel_val);
                set_vec_element(&a_d, i, acc_val);
                std::cout << "Working : " << i << std::endl;
            }
            else
            {
                // CONVERT PATH VALUES X(S) TO TRAJECTORY VALUES X(S(t))
                float pos_val = s_f_t[i];
                float vel_val = 0.0f;
                float acc_val = 0.0f;
                
                // UPDATE DESIRED STATE VECTORS
                set_vec_element(&x_d, i, pos_val);
                set_vec_element(&v_d, i, vel_val);
                set_vec_element(&a_d, i, acc_val);
                std::cout << "complete : " << i << std::endl;
            }

            // INCREMENT TIME COUNTER FOR TRAJECTORY CALCULATIONS
            t_traj[i] += dt;
            //std::cout << "t_traj[i]: " << t_traj[i] << std::endl;
        }
    }
}

void ThrustController::const_velocity_Traj()// In real experiment
{
   
    float t_x = v_t[0]/a_t[0];
    float t_z = v_t[2]/a_t[2];
    float t = t_traj[0];
    std::cout << "t_x: " << t_x << std::endl;
    std::cout << "t_z: " << t_z << std::endl;
    std::cout << "t: " << t << std::endl;

    // X-ACCELERATION
    if(t < t_x) 
    {
        x_d.x = 0.5f*a_t[0]*t*t + s_0_t[0]; // 0.5*a_x*t^2 + x_0
        v_d.x = a_t[0]*t;  // a_x*t
        a_d.x = a_t[0];    // a_x

        x_d.z = s_0_t[2]; // z_0
        v_d.z = 0.0f;
        a_d.z = 0.0f;
        std::cout << "X only: " << std::endl;

    }

    // Z-ACCELERATION (CONSTANT X-VELOCITY)
    else if(t_x <= t && t < (t_x+t_z))
    {
        x_d.x = v_t[0]*t - fsqr(v_t[0])/(2.0f*a_t[0]) + s_0_t[0]; // vx*t - (vx/(2*ax))^2 + x_0
        v_d.x = v_t[0]; // vx
        a_d.x = 0.0f;

        x_d.z = 0.5f*a_t[2]*fsqr(t-t_x) + s_0_t[2]; // 0.5*az*t^2 + z_0
        v_d.z = a_t[2]*(t-t_x); // az*t
        a_d.z = a_t[2]; // az
        std::cout << "Z only: " << std::endl;
    }

    // CONSTANT X-VELOCITY AND CONSTANT Z-VELOCITY
    else if((t_x+t_z) <= t )
    {
        x_d.x = v_t[0]*t - fsqr(v_t[0])/(2.0f*a_t[0]) + s_0_t[0]; // vx*t - (vx/(2*ax))^2 + x_0
        v_d.x = v_t[0]; // vx
        a_d.x = 0.0;

        x_d.z = v_t[2]*(t-t_x) - fsqr(v_t[2])/(2.0f*a_t[2]) + s_0_t[2]; // vz*t - (vz/(2*az))^2 + z_0
        v_d.z = v_t[2]; // vz
        a_d.z = 0.0f;
        std::cout << "Not X,Z: " << std::endl;
    }

    t_traj[0] += dt;
    std::cout << "t: " << t << std::endl;
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ThrustController>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    //node->controlOutput();
    return 0;
}