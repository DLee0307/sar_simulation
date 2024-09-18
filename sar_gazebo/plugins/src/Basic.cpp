#include "Camera_Plugin.h"


using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::Camera_PluginPrivate
{


};

Camera_Plugin::Camera_Plugin() : System(), dataPtr(std::make_unique<Camera_PluginPrivate>())
{

}

//////////////////////////////////////////////////
void Camera_Plugin::CameraMsg(const gz::msgs::Image &_msg)
{
  auto width = _msg.width();
  std::cout << "width: "<< width << std::endl;
  auto height = _msg.height();
  std::cout << "height: "<< height << std::endl;

}

void Camera_Plugin::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm, EventManager &)
{
  this->node.Subscribe("/camera", &Camera_Plugin::CameraMsg, this);
}

void Camera_Plugin::PreUpdate(const UpdateInfo &_info, 
                                EntityComponentManager &_ecm)
{
  GZ_PROFILE("Camera_Plugin::PreUpdate");
}

//////////////////////////////////////////////////
void Camera_Plugin::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Camera_Plugin::PostUpdate");

}


GZ_ADD_PLUGIN(Camera_Plugin,
                    System,
                    Camera_Plugin::ISystemConfigure,
                    Camera_Plugin::ISystemPreUpdate,
                    //Motor_Plugin::ISystemUpdate,
                    Camera_Plugin::ISystemPostUpdate)
                    

GZ_ADD_PLUGIN_ALIAS(Camera_Plugin, "gz::sim::systems::Camera_Plugin")
