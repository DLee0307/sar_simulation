#include "Sticky_Leg_Plugin.h"


using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private Imu data class.
class gz::sim::systems::Sticky_Leg_PluginPrivate
{
    /// \brief Model interface
    public: Model model{kNullEntity};

    /// \brief Copy of the sdf configuration used for this plugin
    public: sdf::ElementPtr sdfConfig;

    /// \brief Initialization flag
    public: bool initialized{false};

    // SDF PARAMS
    public: std::string Joint_Name;
    public: std::string Link_Name;
    public: int Leg_Number;

    ////!! ROS2 Service
    public: std::shared_ptr<rclcpp::Node> ros_node;
    public: rclcpp::Service<sar_msgs::srv::ActivateStickyPads>::SharedPtr Leg_Connect_Service;

    /// ROS2 Callback for thrust subscription \param[in] _msg thrust message
    public: bool Service_Callback(const sar_msgs::srv::ActivateStickyPads::Request::SharedPtr request,
                                        sar_msgs::srv::ActivateStickyPads::Response::SharedPtr response);

    public: void Load(const EntityComponentManager &_ecm,
                      const sdf::ElementPtr &_sdf);

};

//////////////////////////////////////////////////
Sticky_Leg_Plugin::Sticky_Leg_Plugin() : System(), dataPtr(std::make_unique<Sticky_Leg_PluginPrivate>())
{

}

//////////////////////////////////////////////////
Sticky_Leg_Plugin::~Sticky_Leg_Plugin() = default;

//////////////////////////////////////////////////
void Sticky_Leg_PluginPrivate::Load(const EntityComponentManager &_ecm,
                                    const sdf::ElementPtr &_sdf)
{ 
  // GRAB THE JOINT FROM SDF
  if (_sdf->HasElement("JointName"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("JointName");
    Joint_Name = elem->Get<std::string>();
    std::cout << "Joint_Name: " << this->Joint_Name << std::endl; 
  }

  // GRAB THE LINK FROM SDF
  if (_sdf->HasElement("LinkName"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("LinkName");
    Link_Name = elem->Get<std::string>();
    std::cout << "Link_Name: " << this->Link_Name << std::endl; 
  }

  // GRAB THE LEGNUMBER FROM SDF
  if (_sdf->HasElement("LegNumber"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("LegNumber");
    Leg_Number = elem->Get<int>();
    std::cout << "Leg_Number: " << this->Leg_Number << std::endl; 
  }

}

//////////////////////////////////////////////////
void Sticky_Leg_Plugin::Configure(const Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    EntityComponentManager &_ecm, EventManager &)
{ 
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Touch plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void Sticky_Leg_Plugin::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Sticky_Leg_Plugin::PreUpdate");
  //std::cout << "initialized: " << this->dataPtr->initialized << std::endl; 
  //std::cout << "sdfConfig: " << this->dataPtr->sdfConfig << std::endl; 
  //this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
  if ((!this->dataPtr->initialized) && this->dataPtr->sdfConfig)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;
  }
}

//////////////////////////////////////////////////
void Sticky_Leg_Plugin::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Sticky_Leg_Plugin::PostUpdate");

}
/*
//////////////////////////////////////////////////
void Sticky_Leg_PluginPrivate::Update(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Sticky_Leg_PluginPrivate::Update");

}
*/

//////////////////////////////////////////////////////////////
bool Sticky_Leg_PluginPrivate::Service_Callback(const sar_msgs::srv::ActivateStickyPads::Request::SharedPtr request,
                                                sar_msgs::srv::ActivateStickyPads::Response::SharedPtr response)
{

}

GZ_ADD_PLUGIN(Sticky_Leg_Plugin, System,
  Sticky_Leg_Plugin::ISystemConfigure,
  Sticky_Leg_Plugin::ISystemPreUpdate,
  Sticky_Leg_Plugin::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(Sticky_Leg_Plugin, "gz::sim::systems::Sticky_Leg_Plugin")