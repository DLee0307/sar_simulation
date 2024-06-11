#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "sar_msgs/msg/ms.hpp"
#include <std_srvs/srv/set_bool.hpp>

class ThrustController : public rclcpp::Node
{
public:
    ThrustController()
    : Node("thrust_controller")
    {
        this->publisher_1 = this->create_publisher<std_msgs::msg::Float64>("thrust_input_topic_Prop_1_Joint", 10);
        this->publisher_2 = this->create_publisher<std_msgs::msg::Float64>("thrust_input_topic_Prop_2_Joint", 10);
        this->publisher_3 = this->create_publisher<std_msgs::msg::Float64>("thrust_input_topic_Prop_3_Joint", 10);
        this->publisher_4 = this->create_publisher<std_msgs::msg::Float64>("thrust_input_topic_Prop_4_Joint", 10);

        subscriber_1 = this->create_subscription<sar_msgs::msg::MS>(
            "Prop_1_Jointpose_x",
            10,
            std::bind(&ThrustController::handle_prop_1_pose, this, std::placeholders::_1));
        subscriber_2 = this->create_subscription<sar_msgs::msg::MS>(
            "Prop_2_Jointpose_x",
            10,
            std::bind(&ThrustController::handle_prop_2_pose, this, std::placeholders::_1));
        subscriber_3 = this->create_subscription<sar_msgs::msg::MS>(
            "Prop_3_Jointpose_x",
            10,
            std::bind(&ThrustController::handle_prop_3_pose, this, std::placeholders::_1));
        subscriber_4 = this->create_subscription<sar_msgs::msg::MS>(
            "Prop_4_Jointpose_x",
            10,
            std::bind(&ThrustController::handle_prop_4_pose, this, std::placeholders::_1));
    }

private:
    void handle_prop_1_pose(const sar_msgs::msg::MS::SharedPtr msg) {
        double thrust = 10.0;
        if (msg->pose.position.z > 2) {
            thrust = 8.0;
        } else if (msg->pose.position.z > 1) {
            thrust = 10.0;
        } else {
            thrust = 10.0;
        }
        publish_thrust_input(publisher_1, thrust);
    }
    void handle_prop_2_pose(const sar_msgs::msg::MS::SharedPtr msg) {
        double thrust = 10.0;
        if (msg->pose.position.z > 2) {
            thrust = 8.0;
        } else if (msg->pose.position.z > 1) {
            thrust = 10.0;
        } else {
            thrust = 10.0;
        }
        publish_thrust_input(publisher_2, thrust);
    }
    void handle_prop_3_pose(const sar_msgs::msg::MS::SharedPtr msg) {
        double thrust = 10.0;
        if (msg->pose.position.z > 2) {
            thrust = 8.0;
        } else if (msg->pose.position.z > 1) {
            thrust = 10.0;
        } else {
            thrust = 10.0;
        }
        publish_thrust_input(publisher_3, thrust);
    }
    void handle_prop_4_pose(const sar_msgs::msg::MS::SharedPtr msg) {
        double thrust = 10.0;
        if (msg->pose.position.z > 2) {
            thrust = 8.0;
        } else if (msg->pose.position.z > 1) {
            thrust = 10.0;
        } else {
            thrust = 10.0;
        }
        publish_thrust_input(publisher_4, thrust);
    }

    void publish_thrust_input(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& publisher, double thrust_value) {
        auto message = std_msgs::msg::Float64();
        message.data = thrust_value;
        publisher->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_1, publisher_2, publisher_3, publisher_4;
    rclcpp::Subscription<sar_msgs::msg::MS>::SharedPtr subscriber_1, subscriber_2, subscriber_3, subscriber_4;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrustController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}