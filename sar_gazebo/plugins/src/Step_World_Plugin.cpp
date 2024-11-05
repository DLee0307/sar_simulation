#include "Step_World_Plugin.h"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::Step_World_PluginPrivate
{

};

Step_World_Plugin::Step_World_Plugin() : System(), dataPtr(std::make_unique<Step_World_PluginPrivate>())
{
}

Step_World_Plugin::~Step_World_Plugin() = default;


void Step_World_Plugin::Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &/*_eventMgr*/)
{
    //std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
}

void Step_World_Plugin::PreUpdate(const UpdateInfo & /*_info*/, EntityComponentManager &_ecm)
{
    
}

GZ_ADD_PLUGIN(Step_World_Plugin, System,
  Step_World_Plugin::ISystemConfigure,
  Step_World_Plugin::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(Step_World_Plugin, "gz::sim::systems::Step_World_Plugin")