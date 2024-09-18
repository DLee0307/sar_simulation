#ifndef GZ_SIM_SYSTEMS_CAMERA_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_CAMERA_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <memory>
#include <sdf/Element.hh>
#include <gz/sim/Model.hh>
#include <rclcpp/rclcpp.hpp>

#include <gz/msgs.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  class Camera_PluginPrivate;

  class Camera_Plugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    public: Camera_Plugin();
    
    public: ~Camera_Plugin() override = default;

    /// \brief Gazebo transport node
    public: transport::Node node;

    public: void Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &_eventMgr) override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                           const gz::sim::EntityComponentManager &_ecm) override;

    private: void CameraMsg(const gz::msgs::Image &_msg);

    private: std::unique_ptr<Camera_PluginPrivate> dataPtr;
  };
}
}
}  // namespace sim
}  // namespace gz
#endif
