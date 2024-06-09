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
        //std::cout << "CTRL_Cmd.cmd_rx: " << CTRL_Cmd.cmd_rx <<  std::endl;

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
    //RCLCPP_INFO(rclcpp::get_logger("GTC_Controller"), "SAR_Type: %d\n",SAR_Type);
    RCLCPP_INFO(rclcpp::get_logger("GTC_Controller"), "Policy_Type: %d\n", Policy);
    
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
    //Tumbled_Flag = false;
    //MotorStop_Flag = false;
    //CustomThrust_Flag = false;
    //CustomPWM_Flag = false;
    //AngAccel_Flag = false;

    // RESET TRAJECTORY FLAGS
    Traj_Type = NONE;

    // RESET POLICY FLAGS
    //Policy_Armed_Flag = false;
    //Trg_Flag = false;
    //onceFlag = false;

    // UPDATE COLLISION RADIUS
    //Collision_Radius = max(L_eff,Forward_Reach);


    // RESET LOGGED TRIGGER VALUES
    //Trg_Flag = false;
    //Pos_B_O_trg = vzero();
    //Vel_B_O_trg = vzero();
    //Quat_B_O_trg = mkquat(0.0f,0.0f,0.0f,1.0f);
    //Omega_B_O_trg = vzero();

    //Pos_P_B_trg = vzero();
    //Vel_B_P_trg = vzero();
    //Quat_P_B_trg = mkquat(0.0f,0.0f,0.0f,1.0f);
    //Omega_B_P_trg = vzero();

    //Theta_x_trg = 0.0f;
    //Theta_y_trg = 0.0f;
    //Tau_trg = 0.0f;
    //Tau_CR_trg = 0.0f;

    //Policy_Trg_Action_trg = 0.0f;
    //Policy_Rot_Action_trg = 0.0f;


}

void controllerOutOfTreeInit() {

    #ifdef CONFIG_SAR_EXP
    ledSet(LED_BLUE_L, 0);
    ledSet(LED_BLUE_NRF, 0);
    #endif

    controllerOutOfTreeReset();
    controllerOutOfTreeTest();

    // INIT DEEP RL NN POLICY
    //X_input = nml_mat_new(3,1);
    //Y_output = nml_mat_new(4,1);

    // INIT DEEP RL NN POLICY
    // NN_init(&NN_DeepRL,NN_Params_DeepRL);

    RCLCPP_INFO(rclcpp::get_logger("GTC_Controller"), "GTC Controller Initiated");
}



void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{

    // STATE UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

        float time_delta = (tick-prev_tick)/1000.0f;

        // CALC STATES WRT ORIGIN
        Pos_B_O = mkvec(state->position.x-0.0325, state->position.y+0.0325, state->position.z);          // [m]
        Vel_B_O = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);          // [m/s]
        Accel_B_O = mkvec(sensors->acc.x*9.81f, sensors->acc.y*9.81f, sensors->acc.z*9.81f); // [m/s^2]
        Accel_B_O_Mag = firstOrderFilter(vmag(Accel_B_O),Accel_B_O_Mag,0.5f);



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
        //Pos_P_B = mvmul(R_WP,vsub(r_P_O,Pos_B_O)); ///=!! r_P_O plane position vector.
        //Vel_B_P = mvmul(R_WP,Vel_B_O);
        //Vel_mag_B_P = vmag(Vel_B_P);
        //Vel_angle_B_P = atan2f(Vel_B_P.z,Vel_B_P.x)*Rad2Deg;
        //Omega_B_P = Omega_B_O;



        // if (Accel_B_O_Mag > 10.0f && Impact_Flag_OB == false)
        // {
        //     Impact_Flag_OB = true;
        //     Pos_B_O_impact_OB = Pos_B_O;
        //     Vel_B_P_impact_OB = Vel_B_P;
        //     Quat_B_O_impact_OB = Quat_B_O;
        //     Omega_B_P_impact_OB = Omega_B_P;
        //     Accel_B_O_Mag_impact_OB = Accel_B_O_Mag;
        // }


        // SAVE PREVIOUS VALUES
        //Omega_B_O_prev = Omega_B_O;
        prev_tick = tick;
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
                //std::cout << "11111111111: " <<  std::endl;
                break;
                
            case CONST_VEL:
                const_velocity_Traj();
                //std::cout << "CONST_VEL" <<  std::endl;
                break;
        }
    }    

        // CTRL UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {


        controlOutput(state,sensors);
/*
        if(AngAccel_Flag == true || Trg_Flag == true)
        {
            F_thrust = 0.0f;
            M = vscl(2.0f,M_d);
        }
*/
        
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
    }
}