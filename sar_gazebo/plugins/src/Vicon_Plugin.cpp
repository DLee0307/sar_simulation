#include "Vicon_Plugin.h"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private Imu data class.
class gz::sim::systems::Vicon_PluginPrivate
{
  /// Model
  public: Model model{kNullEntity};

  /// Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdfConfig;

  /// Link
  public: Entity linkEntity;
  public: std::string linkName;

  public: void Update(const EntityComponentManager &_ecm);


  ////!! ROS2 Publisher
  public: std::shared_ptr<rclcpp::Node> ros_node;
  public: rclcpp::Publisher<sar_msgs::msg::ViconData>::SharedPtr pose_publisher;
};

//////////////////////////////////////////////////
Vicon_Plugin::Vicon_Plugin() : System(), dataPtr(std::make_unique<Vicon_PluginPrivate>())
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  this->dataPtr->ros_node = std::make_shared<rclcpp::Node>("Vicon_Publisher_Node");
  this->dataPtr->pose_publisher = this->dataPtr->ros_node->create_publisher<sar_msgs::msg::ViconData>("Vicon/data", 1);
}

//////////////////////////////////////////////////
Vicon_Plugin::~Vicon_Plugin() = default;

//////////////////////////////////////////////////
void Vicon_Plugin::Configure(const Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    EntityComponentManager &_ecm, EventManager &)
{ 
  // GRAP THE MODEL
  this->dataPtr->model = Model(_entity);

  // CHECK THE MODEL
  std::cout << "Start Vicon_Plugin, Model Name: " << this->dataPtr->model.Name(_ecm) << std::endl;
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Vicon_Plugin plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->sdfConfig = _sdf->Clone();
  auto sdfClone = _sdf->Clone();

  if (_sdf->HasElement("link_name"))
  {
    std::shared_ptr<sdf::Element> sdfCopy = std::const_pointer_cast<sdf::Element>(_sdf);
    sdf::ElementPtr elem = sdfCopy->GetElement("link_name");
    //std::cout << "linkName1: " << elem << std::endl;
    auto linkName = elem->Get<std::string>();
    //std::cout << "linkName2: " << linkName << std::endl;
    auto entities = entitiesFromScopedName(linkName, _ecm, this->dataPtr->model.Entity());
    //std::cout << "linkName3: " << linkName << std::endl;
    // CHECK IF ENTITY TYPE IS EMPTY
    if (entities.empty()){
      gzerr << "Link with name[" << linkName << "] not found. ";
      //this->dataPtr->validConfig = false;
      return;
    }

    // CHECK IF ENTITY TYPE'S SIZE IS ONLY ONE
    else if (entities.size() > 1){
      gzwarn << "Multiple link entities with name[" << linkName << "] found. "
             << "Using the first one.\n";
    }
    //std::cout << "linkName4: " << linkName << std::endl;
    // USE FIRST ONE
    this->dataPtr->linkEntity = *entities.begin();
    //std::cout << "linkEntity: " << this->dataPtr->linkEntity << std::endl; 


    // CHECK IF ENTITY TYPE IS LINK
    if (!_ecm.EntityHasComponentType(this->dataPtr->linkEntity,
                                     components::Link::typeId)){
      this->dataPtr->linkEntity = kNullEntity;
      gzerr << "Entity with name[" << linkName << "] is not a link\n";
      //this->dataPtr->validConfig = false;
      return;
    }
  }

  //std::cout << "linkName: " << this->dataPtr->linkName << std::endl;
  //std::cout << "linkEntity: " << this->dataPtr->linkEntity << std::endl;
}

//////////////////////////////////////////////////
void Vicon_Plugin::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Vicon_Plugin::PreUpdate");

}

//////////////////////////////////////////////////
void Vicon_Plugin::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Vicon_Plugin::PostUpdate");
  // CHECK THE SIMULATION IS PAUSED
  if (_info.paused)
    return;

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }
  
  auto modelEntity = this->dataPtr->linkEntity;
  //std::cout << "linkName: " << this->dataPtr->linkName << std::endl;
  //std::cout << "linkEntity: " << this->dataPtr->linkEntity << std::endl;
  //std::cout << "modelEntity: " << modelEntity << std::endl;
  
  auto poseComp = _ecm.Component<components::WorldPose>(modelEntity); 
  if (poseComp == nullptr) {
    gzerr << "Pose component not found for entity: " << modelEntity << std::endl;
    return;
  }  

  //std::cout << "poseComp: " << poseComp->Data() << std::endl; 
  auto _poseComp = poseComp->Data();
  //std::cout << "poseComp: " << _poseComp << std::endl; 
  /*std::cout << "_poseComp.Pos().(X): " << _poseComp.Pos().X() << std::endl; 
  std::cout << "_poseComp.Pos().(Y): " << _poseComp.Pos().Y() << std::endl;
  std::cout << "_poseComp.Pos().(Z): " << _poseComp.Pos().Z() << std::endl;
  std::cout << "_poseComp.Rot().(X): " << _poseComp.Rot().X() << std::endl;
  std::cout << "_poseComp.Rot().(Y): " << _poseComp.Rot().Y() << std::endl;
  std::cout << "_poseComp.Rot().(Z): " << _poseComp.Rot().Z() << std::endl;*/

  auto VelComp = _ecm.Component<components::WorldLinearVelocity>(modelEntity);
  auto _VelComp = VelComp->Data();
  //std::cout << "VelComp: " << _VelComp << std::endl; 
  /*std::cout << "_VelComp.(X): " << _VelComp.X() << std::endl; 
  std::cout << "_VelComp.(Y): " << _VelComp.Y() << std::endl; 
  std::cout << "_VelComp.(Z): " << _VelComp.Z() << std::endl;*/

  sar_msgs::msg::ViconData msg;
  
  //msg.motorthrust = _poseComp.Pos().Z();
  msg.pose.position.x = _poseComp.Pos().X();
  msg.pose.position.y = _poseComp.Pos().Y();
  msg.pose.position.z = _poseComp.Pos().Z();
  msg.pose.orientation.x = _poseComp.Rot().X();
  msg.pose.orientation.y = _poseComp.Rot().Y();
  msg.pose.orientation.z = _poseComp.Rot().Z();
  msg.vel.x = _VelComp.X();
  msg.vel.y = _VelComp.Y();
  msg.vel.z = _VelComp.Z();
  

  this->dataPtr->pose_publisher->publish(msg);
/**/
}



//////////////////////////////////////////////////
void Vicon_PluginPrivate::Update(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Vicon_PluginPrivate::Update");

}


GZ_ADD_PLUGIN(Vicon_Plugin, System,
  Vicon_Plugin::ISystemConfigure,
  Vicon_Plugin::ISystemPreUpdate,
  Vicon_Plugin::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(Vicon_Plugin, "gz::sim::systems::Vicon_Plugin")
