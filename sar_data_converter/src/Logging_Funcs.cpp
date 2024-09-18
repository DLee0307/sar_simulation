#include "SAR_DataConverter.h"

#define formatBool(b) ((b) ? "True" : "False")

bool SAR_DataConverter::DataLogging_Callback(const sar_msgs::srv::LoggingCMD::Request::SharedPtr request,
                             sar_msgs::srv::LoggingCMD::Response::SharedPtr response)
{
    switch(request->logging_cmd){
        case 0:
/**/
            //!!!std::cout << "000" << std::endl;
            Logging_Flag = false;
            fPtr = fopen(request->file_path.c_str(), "w");
            create_CSV();

            break;

        case 1:
/**/
            //!!!std::cout << "111" << std::endl;
            Logging_Flag = true;
            fPtr = fopen(request->file_path.c_str(), "a");

            break;

        case 2:
/*
            std::cout << "222" << std::endl;
            Logging_Flag = false;

            fPtr = fopen(req.filePath.c_str(), "a");
            error_string = req.error_string;
            append_CSV_blank();
            append_CSV_misc();
            append_CSV_Trg();
            append_CSV_impact();
            append_CSV_blank();
*/
            break;
    }

    return 1;
}

void SAR_DataConverter::LoggingLoop()
{

    rclcpp::Rate rate(LOGGING_RATE);

    while (rclcpp::ok())
    {   
        if (Logging_Flag == true)
        {
            //append_CSV_states(); 
        }

        rate.sleep();
    }

}

void SAR_DataConverter::create_CSV()
{
    // POLICY DATA
    fprintf(fPtr,"K_ep,K_run,");
    fprintf(fPtr,"t,");
    fprintf(fPtr,"Trg_Action,Rot_Action,");
    fprintf(fPtr,"Mu,Sigma,Policy,");

    // STATE DATA
    fprintf(fPtr,"D_perp,Tau,Tau_CR,Theta_x,");
    fprintf(fPtr,"V_BO_Mag,V_BO_Angle,a_BO_Mag,");
    fprintf(fPtr,"V_BP_Mag,V_BP_Angle,Phi_PB,");
    fprintf(fPtr,"Trg_Flag,Impact_Flag_Ext,Impact_Flag_OB,");

    // STATE DATA
    fprintf(fPtr,"r_BO.x,r_BO.y,r_BO.z,");
    fprintf(fPtr,"V_BO.x,V_BO.y,V_BO.z,");
    fprintf(fPtr,"a_BO.x,a_BO.y,a_BO.z,");


    //  MISC STATE DATA
    fprintf(fPtr,"Eul_BO.x,Eul_BO.y,Eul_BO.z,");
    fprintf(fPtr,"W_BO.x,W_BO.y,W_BO.z,");
    fprintf(fPtr,"AngAcc_BO.y,");
    fprintf(fPtr,"F_thrust,Mx,My,Mz,");

    // SETPOINT VALUES
    fprintf(fPtr,"x_d.x,x_d.y,x_d.z,");
    fprintf(fPtr,"v_d.x,v_d.y,v_d.z,");
    fprintf(fPtr,"a_d.x,a_d.y,a_d.z,");

    // MISC VALUES
    fprintf(fPtr,"Error");
    fprintf(fPtr,"\n");


    fprintf(fPtr,"# DATA_TYPE: %s, ",DATA_TYPE.c_str());
    fprintf(fPtr,"SAR_SETTINGS: {Policy_Type: %s, SAR_Type: %s, SAR_Config: %s}, ",POLICY_TYPE.c_str(),SAR_Type.c_str(),SAR_Config.c_str());
    fprintf(fPtr,"LEG_SETTINGS: {L_eff: %.3f, Gamma_eff: %.0f, K_Pitch: %.1f, K_Yaw: %.1f}, ",L_eff,Gamma_eff,K_Pitch,K_Yaw);
    fprintf(fPtr,"\n");

    fflush(fPtr);

}