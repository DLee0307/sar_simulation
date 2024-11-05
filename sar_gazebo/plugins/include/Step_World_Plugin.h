#ifndef GZ_SIM_SYSTEMS_STEP_WORLD_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_STEP_WORLD_PLUGIN_HH_

// STANDARD INCLUDES
#include <iostream>
#include <thread>


// ROS AND GAZEBO INCLUDES
#include <gz/sim/System.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <memory>
#include <sdf/Element.hh>
#include <gz/sim/Model.hh>

// CUSTOM INCLUDE
//#include "sar_msgs/World_Step.h"


namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  class Step_World_PluginPrivate;

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

    private: std::unique_ptr<Step_World_PluginPrivate> dataPtr;


  };
}
}
}  // namespace sim
}  // namespace gz
#endif