#ifndef GZ_SIM_SYSTEMS_ACTIVE_VISION_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_ACTIVE_VISION_PLUGIN_HH_

#include <cstdint>
#include <memory>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{

class Active_Vision_PluginPrivate;

/// AprilTag 36h11 detector and pitch-only search / track controller.
/// The original L8 camera stream remains at its sensor rate (100 Hz in SDF).
/// Publishes a separate, unnormalized RGB8 display copy at /camera/raw_rgb.
/// AprilTag detection and gimbal control run at a separately configured rate.
/// Camera FOV and gimbal limits are loaded from Gazebo ECM.
class Active_Vision_Plugin : public System,
                             public ISystemConfigure,
                             public ISystemPreUpdate
{
  public: Active_Vision_Plugin();
  public: ~Active_Vision_Plugin() override;

  public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) override;

  public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

  private: void CameraMsg(const gz::msgs::Image &_msg);
  private: void JointStateMsg(const gz::msgs::Model &_msg);

  private: gz::transport::Node node;
  private: std::unique_ptr<Active_Vision_PluginPrivate> dataPtr;
};

}  // namespace systems
}  // inline namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_SYSTEMS_ACTIVE_VISION_PLUGIN_HH_