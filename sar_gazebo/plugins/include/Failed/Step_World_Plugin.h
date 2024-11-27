#ifndef GZ_SIM_SYSTEMS_STEP_WORLD_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_STEP_WORLD_PLUGIN_HH_

// STANDARD INCLUDES
#include <iostream>
#include <thread>
#include <memory>

// ROS AND GAZEBO INCLUDES
#include <gz/sim/System.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <memory>
#include <sdf/Element.hh>
#include <gz/sim/Model.hh>

// CUSTOM INCLUDE
#include "sar_msgs/srv/world_step.hpp"

#include <rclcpp/rclcpp.hpp>
#include <gz/sim/World.hh>

#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  class Step_World_PluginPrivate
  {
    public:
      std::shared_ptr<rclcpp::Node> ros_node;
      std::unique_ptr<gz::transport::Node> gz_node; 
  };

  class Step_World_Plugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    public: explicit Step_World_Plugin();
    
    public: ~Step_World_Plugin() override;

    public: void Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &_eventMgr) override;

    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    public: void Step_World(const std::shared_ptr<sar_msgs::srv::WorldStep::Request> request,
                        std::shared_ptr<sar_msgs::srv::WorldStep::Response> response);

    private: std::unique_ptr<Step_World_PluginPrivate> dataPtr;

    private: gz::sim::World world_;
    private: rclcpp::Service<sar_msgs::srv::WorldStep>::SharedPtr step_world_service_;


  };
}
}
}  // namespace sim
}  // namespace gz
#endif