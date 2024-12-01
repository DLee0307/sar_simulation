#include "Gazebo_Funcs.h"

/**
 * @brief Function called that will service call the sticky foot plugin and join pad to landing surface
 *
 */
void SAR_DataConverter::activateStickyFeet()
{
    auto request = std::make_shared<sar_msgs::srv::ActivateStickyPads::Request>();
    request->sticky_flag = Sticky_Flag;
    auto result_1 = activate_stickypads_service_1->async_send_request(request);
    auto result_2 = activate_stickypads_service_2->async_send_request(request);
    auto result_3 = activate_stickypads_service_3->async_send_request(request);
    auto result_4 = activate_stickypads_service_4->async_send_request(request);

    //std::cout << "activateStickyFeet is run" << std::endl;


}

void SAR_DataConverter::Pad_Connections_Callback_1(const sar_msgs::msg::StickyPadConnect::SharedPtr msg) {
    if (msg->pad1_contact == 1)
        Pad1_Contact = 1;
    Pad_Connections = Pad1_Contact + Pad2_Contact + Pad3_Contact + Pad4_Contact;

    // if (ForelegContact_Flag == true)
    //     Pad_Connections = Pad1_Contact + Pad2_Contact + Pad3_Contact + Pad4_Contact;
}

void SAR_DataConverter::Pad_Connections_Callback_2(const sar_msgs::msg::StickyPadConnect::SharedPtr msg) {
    if (msg->pad2_contact == 1)
        Pad2_Contact = 1;
    Pad_Connections = Pad1_Contact + Pad2_Contact + Pad3_Contact + Pad4_Contact;
    
    // if(HindlegContact_Flag = true)
    //     Pad_Connections = Pad1_Contact + Pad2_Contact + Pad3_Contact + Pad4_Contact;
}

void SAR_DataConverter::Pad_Connections_Callback_3(const sar_msgs::msg::StickyPadConnect::SharedPtr msg) {
    if (msg->pad3_contact == 1)
        Pad3_Contact = 1;
}

void SAR_DataConverter::Pad_Connections_Callback_4(const sar_msgs::msg::StickyPadConnect::SharedPtr msg) {
    if (msg->pad4_contact == 1)
        Pad4_Contact = 1;
}

void SAR_DataConverter::Surface_Contact_Callback(const gz::msgs::Contacts &msg)
{
    for (const auto &contact : msg.contact()) {
        //std::cout << "Contact : " << contact.collision2().name() << std::endl;
        std::string collision_name = contact.collision2().name();
        if (collision_name.find("Body_Collision") != std::string::npos) {
            BodyContact_Flag = true;
            //std::cout << "Body Contact Detected" << std::endl;
        }

        if (collision_name.find("Camera_Collision") != std::string::npos) {
            BodyContact_Flag = true;
            //std::cout << "Body Contact Detected" << std::endl;
        }

        //!!! Need to Change if ForelegContact_Flag is true, cant HindlegContact_Flag be true? > need to check with rewardfunction in SAR_Sim_DeepRL.py
        // Check if the msg contain "Foot_Collision"
        if (ForelegContact_Flag == false && HindlegContact_Flag == false){
            
            if (collision_name.find("Foot_Collision") != std::string::npos) {
                //std::cout << "Foot Collision Detected: " << collision_name << std::endl;
                if (collision_name.back() == '1' || collision_name.back() == '4') {
                    ForelegContact_Flag = true;
                    //std::cout << "Foreleg Contact Detected" << std::endl;
                }
                else if (collision_name.back() == '2' || collision_name.back() == '3') {
                    HindlegContact_Flag = true;
                    //std::cout << "Hindleg Contact Detected" << std::endl;
                }
            }

        }
        // LOCK IN STATE DATA WHEN INITIAL IMPACT DETECTED
        if (Impact_Flag_Ext == false && (BodyContact_Flag == true || ForelegContact_Flag == true || HindlegContact_Flag == true))
        {
            Impact_Flag_Ext = true;

            // RECORD IMPACT STATE DATA FROM END OF CIRCULAR BUFFER WHEN IMPACT FLAGGED
            //Time_impact_Ext = ros::Time::now();
            Time_impact_Ext = clock_->now();
            Pose_B_O_impact_Ext = Pose_B_O_impact_buff.front();
            Eul_B_O_impact_Ext = Eul_B_O_impact_buff.front();
            Twist_B_P_impact_Ext = Twist_P_B_impact_buff.front();
            Eul_P_B_impact_Ext = Eul_P_B_impact_buff.front();
            Rot_Sum_impact_Ext = Rot_Sum;

            //std::cout << "Impact_Flag_Ext = "<< Impact_Flag_Ext << std::endl;

        }

    }
    //std::cout << "Surface_Contact_Callback works well" << std::endl;
}

void SAR_DataConverter::ExtractCollisionName(const std::string& full_name) {
    // Find the last position of "::".
    std::size_t pos = full_name.find_last_of("::");
    
    // If "::" is found, extract the substring from the next character to the end.
    std::string collision_name;
    if (pos != std::string::npos) {
        collision_name = full_name.substr(pos + 1);
    } else {
        collision_name = full_name; // If "::" is not found, use the entire string.
    }

    //std::cout << "Extracted collision name: " << collision_name << std::endl;
}