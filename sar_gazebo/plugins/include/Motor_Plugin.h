#ifndef GZ_SIM_SYSTEMS_MOTOR_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_MOTOR_PLUGIN_HH_

// For making plugin INCLUDES  
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

// For sdf_model
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>

//ROS2 message
#include "sar_msgs/msg/ctrl_data.hpp"

// 
#include <algorithm>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <memory>

//
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

// For LiftDrag
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"

// For ApplyJointForce
#include <gz/sim/System.hh>
#include <gz/msgs/double.pb.h>
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class Motor_PluginPrivate;

  class Motor_Plugin
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        //public ISystemUpdate,
        public ISystemPostUpdate
  {
    public: Motor_Plugin();

    public: ~Motor_Plugin() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;
    /*
    // Documentation inherited
    public: void Update(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;                           
    */

    // Documentation inherited
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                           const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<Motor_PluginPrivate> dataPtr;
  };
  }
}
}
}

#endif
