#ifndef GZ_SIM_SYSTEMS_PRACTICE_HH_
#define GZ_SIM_SYSTEMS_PRACTICE_HH_

#include <gz/sim/System.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <memory>
#include <sdf/Element.hh>
#include <gz/sim/Model.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  class PracticePrivate;

  class Practice:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    public: explicit Practice();
    
    public: ~Practice() override;

    public: void Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &_eventMgr) override;

    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    private: std::unique_ptr<PracticePrivate> dataPtr;

    private: void OnDetachRequest(const gz::msgs::Empty &);
  };
}
}
}  // namespace sim
}  // namespace gz
#endif
