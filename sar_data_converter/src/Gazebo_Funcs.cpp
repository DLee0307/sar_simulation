#include "Gazebo_Funcs.h"

void SAR_DataConverter::Pad_Connections_Callback_1(const sar_msgs::msg::StickyPadConnect::SharedPtr msg) {
    if (msg->pad1_contact == 1)
        Pad1_Contact = 1;
    Pad_Connections = Pad1_Contact + Pad2_Contact + Pad3_Contact + Pad4_Contact;
}

void SAR_DataConverter::Pad_Connections_Callback_2(const sar_msgs::msg::StickyPadConnect::SharedPtr msg) {
    if (msg->pad2_contact == 1)
        Pad2_Contact = 1;
    Pad_Connections = Pad1_Contact + Pad2_Contact + Pad3_Contact + Pad4_Contact;
}

void SAR_DataConverter::Pad_Connections_Callback_3(const sar_msgs::msg::StickyPadConnect::SharedPtr msg) {
    if (msg->pad3_contact == 1)
        Pad3_Contact = 1;
}

void SAR_DataConverter::Pad_Connections_Callback_4(const sar_msgs::msg::StickyPadConnect::SharedPtr msg) {
    if (msg->pad4_contact == 1)
        Pad4_Contact = 1;
}
