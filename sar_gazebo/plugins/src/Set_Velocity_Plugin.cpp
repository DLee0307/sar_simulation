#include "Set_Velocity_Plugin.h"
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/EntityComponentManager.hh>

using namespace gz;
using namespace sim;
using namespace systems;

Set_Velocity_Plugin::Set_Velocity_Plugin()
    : System(), dataPtr(std::make_unique<Set_Velocity_PluginPrivate>())
{
}

void Set_Velocity_Plugin::Configure(const Entity &_entity,
                                    const std::shared_ptr<const sdf::Element> &_sdf,
                                    EntityComponentManager &_ecm,
                                    EventManager &)
{
    std::cout << "Set_Velocity_Plugin is run" << std::endl;
    // 모델 초기화
    this->dataPtr->model = Model(_entity);

    if (!this->dataPtr->model.Valid(_ecm))
    {
        gzerr << "Set_Velocity_Plugin: Invalid model entity." << std::endl;
        return;
    }

    // SDF에서 선속도 읽기
    if (_sdf->HasElement("linear_velocity"))
    {
        this->dataPtr->linearVelocity = _sdf->Get<math::Vector3d>("linear_velocity");
        gzdbg << "Set linear velocity to " << this->dataPtr->linearVelocity << std::endl;
    }
    else
    {
        gzdbg << "No linear_velocity specified in SDF. Using default (0, 0, 0)." << std::endl;
        this->dataPtr->linearVelocity = math::Vector3d(0, 0, 0);
    }

    // `LinearVelocity` 컴포넌트 설정
    auto linearVelComp = _ecm.Component<components::LinearVelocity>(_entity);
    if (!linearVelComp)
    {
        _ecm.CreateComponent(_entity, components::LinearVelocity(this->dataPtr->linearVelocity));
        std::cout << "LinearVelocity is set" << std::endl;
    }
    else
    {
        *linearVelComp = components::LinearVelocity(this->dataPtr->linearVelocity);
        std::cout << "LinearVelocity is set" << std::endl;
    }
}

GZ_ADD_PLUGIN(Set_Velocity_Plugin,
              System,
              Set_Velocity_Plugin::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(Set_Velocity_Plugin, "gz::sim::systems::Set_Velocity_Plugin")
