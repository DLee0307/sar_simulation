#include "Step_World_Plugin.h"
#include <cstdlib>
//https://github.com/gazebosim/gz-sim/blob/990f1c27d4f2ef69740bab433be6e8b7206d39ac/src/SimulationRunner.cc#L1195

using namespace gz;
using namespace sim;
using namespace systems;

Step_World_Plugin::Step_World_Plugin() : System(), dataPtr(std::make_unique<Step_World_PluginPrivate>())
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  this->dataPtr->ros_node = std::make_shared<rclcpp::Node>("step_world_plugin_node");
}

Step_World_Plugin::~Step_World_Plugin()
{
  rclcpp::shutdown();
}


void Step_World_Plugin::Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &/*_eventMgr*/)
{
    //std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
    RCLCPP_INFO(this->dataPtr->ros_node->get_logger(), "Configuring Step_World_Plugin");
    world_ = gz::sim::World(_entity);
    this->step_world_service_ = this->dataPtr->ros_node->create_service<sar_msgs::srv::WorldStep>(
        "/ENV/World_Step", std::bind(&Step_World_Plugin::Step_World, this, std::placeholders::_1, std::placeholders::_2));

    this->dataPtr->gz_node = std::make_unique<gz::transport::Node>();
}

void Step_World_Plugin::Step_World(
    const std::shared_ptr<sar_msgs::srv::WorldStep::Request> request,
    std::shared_ptr<sar_msgs::srv::WorldStep::Response> response)
{
    RCLCPP_INFO(this->dataPtr->ros_node->get_logger(), "Step_World service called with n_steps: %d", request->n_steps);
    std::cout << "Service request received for Step_World with n_steps: " << request->n_steps << std::endl;
    //std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;

    RCLCPP_INFO(this->dataPtr->ros_node->get_logger(), "Step_World service called with n_steps: %d", request->n_steps);

    gz::msgs::WorldControl world_control_msg;
    world_control_msg.set_pause(true);
    world_control_msg.set_multi_step(request->n_steps);

    bool success = this->dataPtr->gz_node->Request("/world/empty/control", world_control_msg);
    response->srv_success = success;

    if (success) {
        RCLCPP_INFO(this->dataPtr->ros_node->get_logger(), "Successfully requested pause and step");
    } else {
        RCLCPP_WARN(this->dataPtr->ros_node->get_logger(), "Failed to send WorldControl request");
    }


/*
    std::string cmd;
    cmd = "gz service -s /world/empty/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'";
    std::system(cmd.c_str());

    gz::transport::Node node;
    gz::msgs::WorldControl world_control_msg;
    world_control_msg.set_pause(true);
    world_control_msg.set_multi_step(request->n_steps);
    node.Request("/world/empty/control", world_control_msg);

    response->srv_success = true;
*/
}

void Step_World_Plugin::PreUpdate(const UpdateInfo & /*_info*/, EntityComponentManager &_ecm)
{
    rclcpp::spin_some(this->dataPtr->ros_node);
}

GZ_ADD_PLUGIN(Step_World_Plugin, System,
  Step_World_Plugin::ISystemConfigure,
  Step_World_Plugin::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(Step_World_Plugin, "gz::sim::systems::Step_World_Plugin")