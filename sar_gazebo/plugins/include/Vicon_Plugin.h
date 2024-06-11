#ifndef GZ_SIM_SYSTEMS_VICON_HH_
#define GZ_SIM_SYSTEMS_VICON_HH_

// For making plugin INCLUDES  
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

// For sdf_model
#include <sdf/Element.hh>

#include <memory>
#include <cmath>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>
#include "gz/sim/Link.hh" ///
#include "gz/sim/Model.hh"
#include <gz/sim/Util.hh>

#include <gz/sim/components/Link.hh>
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/LinearVelocity.hh"

// For ROS2 Topic
#include <rclcpp/rclcpp.hpp>
#include "sar_msgs/msg/vicon_data.hpp"


namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class Vicon_PluginPrivate;

  /// \class Imu Imu.hh gz/sim/systems/Imu.hh
  /// \brief This system manages all IMU sensors in simulation.
  /// Each IMU sensor eports vertical position, angular velocity
  /// and lienar acceleration readings over Gazebo Transport.
  class Vicon_Plugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit Vicon_Plugin();

    /// \brief Destructor
    public: ~Vicon_Plugin() override;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<Vicon_PluginPrivate> dataPtr;
  };
  }
}
}
}
#endif
