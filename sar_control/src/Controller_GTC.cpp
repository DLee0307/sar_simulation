#include "Controller_GTC.h"

#define max(a,b) ((a) > (b) ? (a) : (b))

/*void appMain() {

    while (1)
    {
        #ifdef CONFIG_SAR_SIM

            if (CTRL_Cmd.cmd_rx == true)
            {
                CTRL_Command(&CTRL_Cmd);
                CTRL_Cmd.cmd_rx = false;
            }


        #elif CONFIG_SAR_EXP

            if (appchannelReceiveDataPacket(&CTRL_Cmd,sizeof(CTRL_Cmd),APPCHANNEL_WAIT_FOREVER))
            {
                if (CTRL_Cmd.cmd_rx == true)
                {
                    CTRL_Command(&CTRL_Cmd);
                    CTRL_Cmd.cmd_rx = false;
                }
            }

        #endif
    }
    
}*/

void appMain() {

    while (1)
    {
        //std::cout << "CTRL_Cmd.cmd_rx: " << CTRL_Cmd.cmd_rx <<  std::endl;//!!! If remove this part, cannot run.

        if (CTRL_Cmd.cmd_rx == true)
        {
            CTRL_Command(&CTRL_Cmd);
            //std::cout << "11111111111: " <<  std::endl;
            CTRL_Cmd.cmd_rx = false;

        }

    }
    
}

bool controllerOutOfTreeTest() {

  return true;
}

void controllerOutOfTreeReset() {

    RCLCPP_INFO(rclcpp::get_logger("GTC_Controller"), "GTC Controller Reset");
    RCLCPP_INFO(rclcpp::get_logger("GTC_Controller"), "SAR_Type: %s",SAR_Type);
    RCLCPP_INFO(rclcpp::get_logger("GTC_Controller"), "Policy_Type: %d", Policy);
    
    J = mdiag(Ixx,Iyy,Izz);

    // RESET INTEGRATION ERRORS
    e_PI = vzero(); // Pos. Integral-error [m*s]
    e_RI = vzero(); // Rot. Integral-error [m*s]

    // TURN POS/VEL CONTROLLER FLAGS ON
    kp_xf = 1.0f;
    kd_xf = 1.0f;

    // RESET SETPOINTS TO HOME POSITION
    x_d = mkvec(0.0f,0.0f,0.4f);
    v_d = mkvec(0.0f,0.0f,0.0f);
    a_d = mkvec(0.0f,0.0f,0.0f);
    b1_d = mkvec(1.0f,0.0f,0.0f);

    // RESET SYSTEM FLAGS
    Tumbled_Flag = false;
    CustomThrust_Flag = false;
    CustomMotorCMD_Flag = false;
    AngAccel_Flag = false;

    // RESET TRAJECTORY FLAGS
    Traj_Type = NONE;
    resetTraj_Vals(0);
    resetTraj_Vals(1);
    resetTraj_Vals(2);

    // RESET POLICY FLAGS
    Policy_Armed_Flag = false;
    Trg_Flag = false;
    onceFlag = false;

    // UPDATE COLLISION RADIUS
    Collision_Radius = L_eff;

    // RESET LOGGED TRIGGER VALUES
    Trg_Flag = false;
    Pos_B_O_trg = vzero();
    Vel_B_O_trg = vzero();
    Quat_B_O_trg = mkquat(0.0f,0.0f,0.0f,1.0f);
    Omega_B_O_trg = vzero();

    Pos_P_B_trg = vzero();
    Vel_B_P_trg = vzero();
    Quat_P_B_trg = mkquat(0.0f,0.0f,0.0f,1.0f);
    Omega_B_P_trg = vzero();

    D_perp_trg = 0.0f;
    D_perp_CR_trg = 0.0f;

    Optical_Flow_Flag = false;
    Rolling_Shutter_Flag = false;

    Theta_x_trg = 0.0f;
    Theta_x_DH_trg = 0.0f;
    Theta_y_trg = 0.0f;
    Tau_trg = 0.0f;
    Tau_CR_trg = 0.0f;
    Tau_CM_trg = 0.0f;
    Theta_x_CM_trg = 0.0f;
    Tau_DH_trg = 0.0f;

    Y_output_trg[0] = 0.0f;
    Y_output_trg[1] = 0.0f;
    Y_output_trg[2] = 0.0f;
    Y_output_trg[3] = 0.0f;

    a_Trg_trg = 0.0f;
    a_Rot_trg = 0.0f;

    // RESET LOGGED IMPACT VALUES
    Impact_Flag_OB = false;
    Impact_Flag_Ext = false;
    Vel_mag_B_P_impact_OB = 0.0f;
    Vel_angle_B_P_impact_OB = 0.0f;
    Quat_B_O_impact_OB = mkquat(0.0f,0.0f,0.0f,1.0f);
    Omega_B_O_impact_OB = vzero();
    dOmega_B_O_impact_OB = vzero();


    // TURN OFF IMPACT LEDS
    #ifdef CONFIG_SAR_EXP
    ledSet(LED_GREEN_L, 0);
    ledSet(LED_BLUE_NRF, 0);
    #endif


}

void controllerOutOfTreeInit() {

    #ifdef CONFIG_SAR_EXP

    #endif

    controllerOutOfTreeReset();
    controllerOutOfTreeTest();
    
    if(Policy == DEEP_RL_ONBOARD_DH){
        // INIT DEEP RL NN POLICY
        //X_input = nml_mat_new(2,1);
        X_input = nml_mat_new(2,1);

        Y_output = nml_mat_new(4,1);

        // INIT DEEP RL NN POLICY
        NN_init(&NN_DeepRL,NN_Params_DeepRL_DH);

    }

    else{
        // INIT DEEP RL NN POLICY
        X_input = nml_mat_new(4,1);
        Y_output = nml_mat_new(4,1);

        // INIT DEEP RL NN POLICY
        NN_init(&NN_DeepRL,NN_Params_DeepRL);

    }


    // // INIT DEEP RL NN POLICY
    // X_input = nml_mat_new(4,1);
    // Y_output = nml_mat_new(4,1);

    // // INIT DEEP RL NN POLICY
    // NN_init(&NN_DeepRL,NN_Params_DeepRL);

    RCLCPP_INFO(rclcpp::get_logger("GTC_Controller"), "GTC Controller Initiated");
}



void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{
/*
    // CHECK FOR CRAZYSWARM SIGNAL
    #ifdef CONFIG_SAR_EXP
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
    {
        uint32_t now = xTaskGetTickCount();
        if (now - PrevCrazyswarmTick > 1000)
        {
            Armed_Flag = false;
        }
    }
    #endif
*/

    // POLICY UPDATES
    if (isOFUpdated == true) {

        isOFUpdated = false;

        if(Policy_Armed_Flag == true){

            switch (Policy)
            {
                case DEEP_RL_SB3:

                    // EXECUTE POLICY IF TRIGGERED
                    if(onceFlag == false){

                        onceFlag = true;

                        // UPDATE AND RECORD TRIGGER VALUES
                        Trg_Flag = true;  
                        Pos_B_O_trg = Pos_B_O;
                        Vel_B_O_trg = Vel_B_O;
                        Quat_B_O_trg = Quat_B_O;
                        Omega_B_O_trg = Omega_B_O;

                        Pos_P_B_trg = Pos_P_B;
                        Vel_B_P_trg = Vel_B_P;
                        Quat_P_B_trg = Quat_P_B;
                        Omega_B_P_trg = Omega_B_P;

                        Tau_trg = Tau;
                        Tau_CR_trg = Tau_CR;
                        Tau_CM_trg = Tau_CM;
                        Theta_x_CM_trg = Theta_x_CM;
                        Tau_DH_trg = Tau_DH;
                        Theta_x_trg = Theta_x;
                        Theta_x_DH_trg = Theta_x_DH;
                        Theta_y_trg = Theta_y;
                        D_perp_trg = D_perp;
                        D_perp_CR_trg = D_perp_CR;


                        a_Trg_trg = a_Trg;
                        a_Rot_trg = a_Rot;

                        M_d.x = 0.0f;
                        M_d.y = a_Rot*Iyy;
                        M_d.z = 0.0f;
                    }

                    break;

                case DEEP_RL_ONBOARD:

                    //std::cout << "Policy_Armed_Flag: " << Policy_Armed_Flag << std::endl;
                    // PASS OBSERVATION THROUGH POLICY NN
                    NN_forward(X_input,Y_output,&NN_DeepRL);

                    // printf("X_input: %.5f %.5f %.5f %.5f\n",X_input->data[0][0],X_input->data[1][0],X_input->data[2][0],X_input->data[3][0]);
                    // printf("Y_output: %.5f %.5f %.5f %.5f\n\n",Y_output->data[0][0],Y_output->data[1][0],Y_output->data[2][0],Y_output->data[3][0]);


                    // SAMPLE POLICY TRIGGER ACTION
                    a_Trg = GaussianSample(Y_output->data[0][0],Y_output->data[2][0]);
                    a_Rot = GaussianSample(Y_output->data[1][0],Y_output->data[3][0]);

                    // SCALE ACTIONS
                    a_Trg = scaleValue(tanhf(a_Trg),-1.0f,1.0f,-1.0f,1.0f);
                    a_Rot = scaleValue(tanhf(a_Rot),-1.0f,1.0f,a_Rot_bounds[0],a_Rot_bounds[1]);

                    // EXECUTE POLICY IF TRIGGERED
                    if(a_Trg >= 0.5f && onceFlag == false && abs(Tau_CR) <= 1.0f)
                    {
                        onceFlag = true;
                        //std::cout << "Trigger is generated." << std::endl;

                        // UPDATE AND RECORD TRIGGER VALUES
                        Trg_Flag = true;  
                        Pos_B_O_trg = Pos_B_O;
                        Vel_B_O_trg = Vel_B_O;
                        Quat_B_O_trg = Quat_B_O;
                        Omega_B_O_trg = Omega_B_O;

                        Pos_P_B_trg = Pos_P_B;
                        Vel_B_P_trg = Vel_B_P;
                        Quat_P_B_trg = Quat_P_B;
                        Omega_B_P_trg = Omega_B_P;

                        Tau_trg = Tau;
                        Tau_CR_trg = Tau_CR;
                        Tau_DH_trg = Tau_DH;
                        Theta_x_trg = Theta_x;
                        Theta_y_trg = Theta_y;
                        D_perp_trg = D_perp;
                        D_perp_CR_trg = D_perp_CR;

                        Y_output_trg[0] = Y_output->data[0][0];
                        Y_output_trg[1] = Y_output->data[1][0];
                        Y_output_trg[2] = Y_output->data[2][0];
                        Y_output_trg[3] = Y_output->data[3][0];

                        a_Trg_trg = a_Trg;
                        a_Rot_trg = a_Rot;

                        M_d.x = 0.0f;
                        M_d.y = a_Rot*Iyy;
                        M_d.z = 0.0f;
                    }
                        
                    break;


                case DEEP_RL_ONBOARD_DH:

                    //std::cout << "Policy_Armed_Flag: " << Policy_Armed_Flag << std::endl;
                    // PASS OBSERVATION THROUGH POLICY NN
                    NN_forward(X_input,Y_output,&NN_DeepRL);

                    // printf("X_input: %.5f %.5f %.5f %.5f\n",X_input->data[0][0],X_input->data[1][0],X_input->data[2][0],X_input->data[3][0]);
                    // printf("Y_output: %.5f %.5f %.5f %.5f\n\n",Y_output->data[0][0],Y_output->data[1][0],Y_output->data[2][0],Y_output->data[3][0]);


                    // SAMPLE POLICY TRIGGER ACTION
                    a_Trg = GaussianSample(Y_output->data[0][0],Y_output->data[2][0]);
                    a_Rot = GaussianSample(Y_output->data[1][0],Y_output->data[3][0]);

                    // SCALE ACTIONS
                    a_Trg = scaleValue(tanhf(a_Trg),-1.0f,1.0f,-1.0f,1.0f);
                    //a_Rot = scaleValue(tanhf(a_Rot),-1.0f,1.0f,a_Rot_bounds[0],a_Rot_bounds[1]);
                    //a_Rot = scaleValue(tanhf(a_Rot),-1.0f,1.0f,-197.835f,197.835f);
                    a_Rot = scaleValue(tanhf(a_Rot),-1.0f,1.0f,-90.0f,-80.0f);

                    // EXECUTE POLICY IF TRIGGERED
                    if(a_Trg >= 0.5f && onceFlag == false && abs(Tau_CR) <= 0.5f)
                    {
                        onceFlag = true;
                        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
                        //std::cout << "Trigger is generated." << std::endl;

                        // UPDATE AND RECORD TRIGGER VALUES
                        Trg_Flag = true;  
                        Pos_B_O_trg = Pos_B_O;
                        Vel_B_O_trg = Vel_B_O;
                        Quat_B_O_trg = Quat_B_O;
                        Omega_B_O_trg = Omega_B_O;

                        Pos_P_B_trg = Pos_P_B;
                        Vel_B_P_trg = Vel_B_P;
                        Quat_P_B_trg = Quat_P_B;
                        Omega_B_P_trg = Omega_B_P;

                        Tau_trg = Tau;
                        Tau_CR_trg = Tau_CR; //
                        // Tau_DH_trg = Tau_DH;
                        // std::cout << "Tau_DH_trg: " << Tau_DH_trg << std::endl;
                        
                        Theta_x_trg = Theta_x; //
                        Theta_x_DH_trg = Theta_x_DH;
                        Theta_y_trg = Theta_y;
                        D_perp_trg = D_perp;
                        D_perp_CR_trg = D_perp_CR;

                        Y_output_trg[0] = Y_output->data[0][0];
                        Y_output_trg[1] = Y_output->data[1][0];
                        Y_output_trg[2] = Y_output->data[2][0];
                        Y_output_trg[3] = Y_output->data[3][0];

                        a_Trg_trg = a_Trg;
                        a_Rot_trg = a_Rot;

                        M_d.x = 0.0f;
                        M_d.y = a_Rot*Iyy;
                        M_d.z = 0.0f;
                    }
                        
                    break;

                // case DEEP_RL_ONBOARD_DH: //manual flip

                //     //std::cout << "Policy_Armed_Flag: " << Policy_Armed_Flag << std::endl;
                //     // PASS OBSERVATION THROUGH POLICY NN
                //     ///// NN_forward(X_input,Y_output,&NN_DeepRL);

                //     // printf("X_input: %.5f %.5f %.5f %.5f\n",X_input->data[0][0],X_input->data[1][0],X_input->data[2][0],X_input->data[3][0]);
                //     // printf("Y_output: %.5f %.5f %.5f %.5f\n\n",Y_output->data[0][0],Y_output->data[1][0],Y_output->data[2][0],Y_output->data[3][0]);


                //     // SAMPLE POLICY TRIGGER ACTION
                //     ///// a_Trg = GaussianSample(Y_output->data[0][0],Y_output->data[2][0]);
                //     ///// a_Rot = GaussianSample(Y_output->data[1][0],Y_output->data[3][0]);

                //     // SCALE ACTIONS
                //     ///// a_Trg = scaleValue(tanhf(a_Trg),-1.0f,1.0f,-1.0f,1.0f);
                //     //a_Rot = scaleValue(tanhf(a_Rot),-1.0f,1.0f,a_Rot_bounds[0],a_Rot_bounds[1]);
                //     //a_Rot = scaleValue(tanhf(a_Rot),-1.0f,1.0f,-197.835f,197.835f);
                //     ////// a_Rot = scaleValue(tanhf(a_Rot),-1.0f,1.0f,-90.0f,-80.0f);
                //     a_Rot = -88.5f;

                //     // EXECUTE POLICY IF TRIGGERED
                //     ///// if(a_Trg >= 0.5f && onceFlag == false && abs(Tau_CR) <= 0.5f)
                //     ///// if(Tau_DH <= 0.27f && onceFlag == false && abs(Tau_CR) <= 0.5f)
                //     if(Tau_CR <= 0.255f && onceFlag == false && abs(Tau_CR) <= 0.5f)
                //     {
                //         onceFlag = true;
                //         //std::cout << "Trigger is generated." << std::endl;

                //         // UPDATE AND RECORD TRIGGER VALUES
                //         Trg_Flag = true;  
                //         Pos_B_O_trg = Pos_B_O;
                //         Vel_B_O_trg = Vel_B_O;
                //         Quat_B_O_trg = Quat_B_O;
                //         Omega_B_O_trg = Omega_B_O;

                //         Pos_P_B_trg = Pos_P_B;
                //         Vel_B_P_trg = Vel_B_P;
                //         Quat_P_B_trg = Quat_P_B;
                //         Omega_B_P_trg = Omega_B_P;

                //         Tau_trg = Tau;
                //         Tau_CR_trg = Tau_CR; //
                //         ///// Tau_DH_trg = Tau_DH;
                //         ///// std::cout << "Tau_DH_trg: " << Tau_DH_trg << std::endl;
                        
                //         Theta_x_trg = Theta_x; //
                //         Theta_x_DH_trg = Theta_x_DH;
                //         Theta_y_trg = Theta_y;
                //         D_perp_trg = D_perp;
                //         D_perp_CR_trg = D_perp_CR;

                //         ///// Y_output_trg[0] = Y_output->data[0][0];
                //         ///// Y_output_trg[1] = Y_output->data[1][0];
                //         ///// Y_output_trg[2] = Y_output->data[2][0];
                //         ///// Y_output_trg[3] = Y_output->data[3][0];

                //         ///// a_Trg_trg = a_Trg;
                //         ///// a_Rot_trg = a_Rot;

                //         M_d.x = 0.0f;
                //         M_d.y = a_Rot*Iyy;
                //         M_d.z = 0.0f;
                //     }
                        
                //     break;
                
                    
            default:
                break;
            }                    
        }
    }


    if (RATE_DO_EXECUTE(RATE_25_HZ, tick))
    {
        updateRotationMatrices();
    }


    // STATE UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

        float time_delta = (tick-prev_tick)/1000.0f;

        // CALC STATES WRT ORIGIN
        Pos_B_O = mkvec(state->position.x, state->position.y, state->position.z);               // [m]
        Vel_B_O = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);               // [m/s]
        Accel_B_O = mkvec(sensors->acc.x*9.81f, sensors->acc.y*9.81f, sensors->acc.z*9.81f);    // [m/s^2]
        Accel_B_O_Mag = firstOrderFilter(vmag(Accel_B_O),Accel_B_O_Mag,1.0f) - 9.81f;           // [m/s^2]



        Omega_B_O = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));   // [rad/s]

        // CALC AND FILTER ANGULAR ACCELERATION
        dOmega_B_O.x = firstOrderFilter((Omega_B_O.x - Omega_B_O_prev.x)/time_delta,dOmega_B_O.x,0.90f); // [rad/s^2]
        dOmega_B_O.y = firstOrderFilter((Omega_B_O.y - Omega_B_O_prev.y)/time_delta,dOmega_B_O.y,0.90f); // [rad/s^2]
        dOmega_B_O.z = firstOrderFilter((Omega_B_O.z - Omega_B_O_prev.z)/time_delta,dOmega_B_O.z,0.90f); // [rad/s^2]


        Quat_B_O = mkquat(state->attitudeQuaternion.x,
                        state->attitudeQuaternion.y,
                        state->attitudeQuaternion.z,
                        state->attitudeQuaternion.w);

        // CALC STATES WRT PLANE
        Pos_P_B = mvmul(R_WP,vsub(r_P_O,Pos_B_O));  ///=!! r_P_O plane position vector.
        Vel_B_P = mvmul(R_WP,Vel_B_O);
        Vel_mag_B_P = vmag(Vel_B_P);
        Vel_angle_B_P = atan2f(Vel_B_P.z,Vel_B_P.x)*Rad2Deg;
        Omega_B_P = Omega_B_O;


/*
        // LAGGING STATES TO RECORD IMPACT VALUES
        // NOTE: A circular buffer would be a true better option if time allows
        if (cycleCounter % 5 == 0)
        {
            Vel_mag_B_P_prev_N = Vel_mag_B_P;
            Vel_angle_B_P_prev_N = Vel_angle_B_P;
            Quat_B_O_prev_N = Quat_B_O;
            Omega_B_O_prev_N = Omega_B_O;
            dOmega_B_O_prev_N = dOmega_B_O;
        }
        cycleCounter++;
        

        // ONBOARD IMPACT DETECTION
        if (dOmega_B_O.y > 300.0f && Impact_Flag_OB == false)
        {
            Impact_Flag_OB = true;
            Vel_mag_B_P_impact_OB = Vel_mag_B_P_prev_N;
            Vel_angle_B_P_impact_OB = Vel_angle_B_P_prev_N;
            Quat_B_O_impact_OB = Quat_B_O_prev_N;
            Omega_B_O_impact_OB = Omega_B_O_prev_N;
            dOmega_B_O_impact_OB.y = dOmega_B_O_prev_N.y;

            // TURN ON IMPACT LEDS
            #ifdef CONFIG_SAR_EXP
            ledSet(LED_GREEN_R, 1);
            ledSet(LED_BLUE_NRF, 1);
            #endif
        }
*/

        // SAVE PREVIOUS VALUES
        Omega_B_O_prev = Omega_B_O;
        prev_tick = tick;
    }

    // OPTICAL FLOW UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
    {
        // UPDATE GROUND TRUTH OPTICAL FLOW
        updateOpticalFlowAnalytic(state,sensors);

        // POLICY VECTOR UPDATE
        if (CamActive_Flag == true)
        {
            // ONLY UPDATE WITH NEW OPTICAL FLOW DATA
            // isOFUpdated = updateOpticalFlowEst();

            // UPDATE POLICY VECTOR
            // X_input->data[0][0] = Tau_Cam;
            // X_input->data[1][0] = Theta_x_Cam;
            // X_input->data[2][0] = D_perp; 
            // X_input->data[3][0] = Plane_Angle_deg; 
        }
        else if (Policy == DEEP_RL_ONBOARD_DH)
            {
                // UPDATE AT THE ABOVE FREQUENCY
                isOFUpdated = true;

                //std::cout << "Tau_DH: " << Tau_DH << std::endl;
                //std::cout << "D_perp_CR: " << D_perp_CR << std::endl;


                // UPDATE POLICY VECTOR
                X_input->data[0][0] = scaleValue(Tau_DH,-5.0f,5.0f,-1.0f,1.0f);
                X_input->data[1][0] = scaleValue(Theta_x_DH,-20.0f,20.0f,-1.0f,1.0f); 
            }
        else
        {
            // UPDATE AT THE ABOVE FREQUENCY
            isOFUpdated = true;

            //std::cout << "Tau_CR: " << Tau_CR << std::endl;
            //std::cout << "Theta_x: " << Theta_x << std::endl;
            //std::cout << "D_perp_CR: " << D_perp_CR << std::endl;
            //std::cout << "Plane_Angle_deg: " << Plane_Angle_deg << std::endl;


            // UPDATE POLICY VECTOR
            X_input->data[0][0] = scaleValue(Tau_CR,-5.0f,5.0f,-1.0f,1.0f);
            X_input->data[1][0] = scaleValue(Theta_x,-20.0f,20.0f,-1.0f,1.0f);
            X_input->data[2][0] = scaleValue(D_perp_CR,-0.5f,2.0f,-1.0f,1.0f); 
            X_input->data[3][0] = scaleValue(Plane_Angle_deg,0.0f,180.0f,-1.0f,1.0f);
        }
    }

    // TRAJECTORY UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

        switch (Traj_Type)
        {
            case NONE:
                /* DO NOTHING */
                break;

            case P2P:
                point2point_Traj();
                //std::cout << "P2P" <<  std::endl;
                break;
                
            case CONST_VEL:
                const_velocity_Traj();
                //std::cout << "CONST_VEL" <<  std::endl;
                break;

            case GZ_CONST_VEL:
                const_velocity_GZ_Traj();
                break;
        }
    }    

        // CTRL UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {


        controlOutput(state,sensors);
        F_thrust = clamp(F_thrust,0.0f,Thrust_max*g2Newton*4*0.85f);

        if(AngAccel_Flag == true || Trg_Flag == true)
        {
            F_thrust = 0.0f;
            M = vscl(2.0f,M_d); // divide a vector by a scalar.
        }

        
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
        
        //std::cout << "controlOutput is executed" << std::endl;

         // TUMBLE DETECTION
        if(b3.z <= 0 && TumbleDetect_Flag == true){ // If b3 axis has a negative z-component (Quadrotor is inverted)
            Tumbled_Flag = true;
        }
/*
        
        if(CustomThrust_Flag) // REPLACE THRUST VALUES WITH CUSTOM VALUES
        {
            M1_thrust = thrust_override[0];
            M2_thrust = thrust_override[1];
            M3_thrust = thrust_override[2];
            M4_thrust = thrust_override[3];

            // CONVERT THRUSTS TO M_CMD SIGNALS
            M1_CMD = (int32_t)thrust2Motor_CMD(M1_thrust); 
            M2_CMD = (int32_t)thrust2Motor_CMD(M2_thrust);
            M3_CMD = (int32_t)thrust2Motor_CMD(M3_thrust);
            M4_CMD = (int32_t)thrust2Motor_CMD(M4_thrust);
        }
        else if(CustomMotorCMD_Flag)
        {
            M1_CMD = M_CMD_override[0]; 
            M2_CMD = M_CMD_override[1];
            M3_CMD = M_CMD_override[2];
            M4_CMD = M_CMD_override[3];
        }
        else 
        {
            // CONVERT THRUSTS TO M_CMD SIGNALS
            M1_CMD = (int32_t)thrust2Motor_CMD(M1_thrust); 
            M2_CMD = (int32_t)thrust2Motor_CMD(M2_thrust);
            M3_CMD = (int32_t)thrust2Motor_CMD(M3_thrust);
            M4_CMD = (int32_t)thrust2Motor_CMD(M4_thrust);
        }
*/
        if (!Armed_Flag || MotorStop_Flag || Tumbled_Flag || Impact_Flag_OB || Impact_Flag_Ext)
        {
            #ifndef CONFIG_SAR_EXP
            M1_thrust = 0.0f;
            M2_thrust = 0.0f;
            M3_thrust = 0.0f;
            M4_thrust = 0.0f;
            #endif

            M1_CMD = 0.0f; 
            M2_CMD = 0.0f;
            M3_CMD = 0.0f;
            M4_CMD = 0.0f;
        }
    }
    
}