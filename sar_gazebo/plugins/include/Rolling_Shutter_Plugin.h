#ifndef GZ_SIM_SYSTEMS_ROLLING_SHUTTER_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_ROLLING_SHUTTER_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <memory>
#include <sdf/Element.hh>
#include <gz/sim/Model.hh>
#include <rclcpp/rclcpp.hpp>

#include "sar_msgs/msg/ctrl_data.hpp"

#include <gz/msgs.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <Eigen/Dense>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <vector>
#include <thread>   // std::this_thread::sleep_for
#include <chrono>   // std::chrono::milliseconds

#include <sensor_msgs/msg/image.hpp>

#include <cstring>

#include <algorithm>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  class Rolling_Shutter_PluginPrivate;

  class Rolling_Shutter_Plugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    public: Rolling_Shutter_Plugin();
    
    public: ~Rolling_Shutter_Plugin() override = default;

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

    public: void Rolling_Shutter_Flag_Callback(const sar_msgs::msg::CtrlData::SharedPtr msg);

    private: std::unique_ptr<Rolling_Shutter_PluginPrivate> dataPtr;
  


    private: void OnImage(const gz::msgs::Image &msg);

    private: struct Frame {
      std::vector<uint8_t> buf;  // 프레임 바이트를 안전하게 소유(딥카피)
      uint32_t width{0}, height{0}, step{0};
      gz::msgs::PixelFormatType fmt{};
    };

    private: std::optional<Frame> WaitLatestFrame(int timeout_ms);

    private: void CopyBlockIntoRolling(const Frame &f, int blockIdx);

    private: void PauseWorld(bool pause);
    private: void MultiStep(int steps);

    private: void OnStats(const gz::msgs::WorldStatistics &msg);

  };
}
}
}  // namespace sim
}  // namespace gz
#endif
