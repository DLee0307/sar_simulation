#include "SAR_DataConverter.h"

#define formatBool(b) ((b) ? "True" : "False")

bool SAR_DataConverter::DataLogging_Callback(const sar_msgs::srv::LoggingCMD::Request::SharedPtr request,
                             sar_msgs::srv::LoggingCMD::Response::SharedPtr response)
{
    switch(request->logging_cmd){
        case 0:
/*
            std::cout << "000" << std::endl;
            Logging_Flag = false;
            fPtr = fopen(req.filePath.c_str(), "w");
            create_CSV();
*/
            break;

        case 1:
/*
            std::cout << "111" << std::endl;
            Logging_Flag = true;
            fPtr = fopen(req.filePath.c_str(), "a");
*/
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

/*
    std::cout << "Service is requested in DataConverter" << std::endl;
    std::cout << "file_path: " << request->file_path << std::endl;
    std::cout << "logging_cmd: " << static_cast<int>(request->logging_cmd) << std::endl;
    std::cout << "error_string: " << request->error_string << std::endl;
*/

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