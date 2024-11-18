#ifndef GZ_SIM_SYSTEMS_SET_VELOCITY_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_SET_VELOCITY_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <sdf/Element.hh>
#include <memory>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
  class Set_Velocity_PluginPrivate
  {
    public: 
      gz::sim::Model model;

    /// \brief 설정할 선속도
    public: math::Vector3d linearVelocity;
  };

  class Set_Velocity_Plugin : public System, public ISystemConfigure
  {
    public: Set_Velocity_Plugin();
    public: ~Set_Velocity_Plugin() override = default;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    private: std::unique_ptr<Set_Velocity_PluginPrivate> dataPtr;
  };
}
}
}
}

#endif
