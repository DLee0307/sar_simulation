#ifndef GZ_SIM_SYSTEMS_IMU_HH_
#define GZ_SIM_SYSTEMS_IMU_HH_

#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>
#include "sar_msgs/msg/imu_data.hpp"

#include <cmath>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class Imu_PluginPrivate;

  /// \class Imu Imu.hh gz/sim/systems/Imu.hh
  /// \brief This system manages all IMU sensors in simulation.
  /// Each IMU sensor eports vertical position, angular velocity
  /// and lienar acceleration readings over Gazebo Transport.
  class Imu_Plugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit Imu_Plugin();

    /// \brief Destructor
    public: ~Imu_Plugin() override;

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
    private: std::unique_ptr<Imu_PluginPrivate> dataPtr;
  };
  }
}
}
}
#endif