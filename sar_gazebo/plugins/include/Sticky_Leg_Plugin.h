#ifndef GZ_SIM_SYSTEMS_STICKY_LEG_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_STICKY_LEG_PLUGIN_HH_

// For registering plugin
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

// For sdf_model
#include <sdf/Element.hh>

#include "gz/sim/Model.hh"

#include <memory>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>


namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class Sticky_Leg_PluginPrivate;

  class Sticky_Leg_Plugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit Sticky_Leg_Plugin();

    /// \brief Destructor
    public: ~Sticky_Leg_Plugin() override;

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

    // Initialize the plugin
    public: void Load(const EntityComponentManager &_ecm,
                      const sdf::ElementPtr &_sdf);

    /// \brief Private data pointer.
    private: 
      std::unique_ptr<Sticky_Leg_PluginPrivate> dataPtr;

  };
  }
}
}
}
#endif
