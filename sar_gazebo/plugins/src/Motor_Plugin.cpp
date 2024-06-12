///Joint API : https://github.com/gazebosim/gz-sim/blob/760ba21f685f65f436a0107103a68bcf4170a17b/tutorials/migration_joint_api.md?plain=1#L128

#include "Motor_Plugin.h"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::Motor_PluginPrivate
{
  /// Initialize the system
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  // Compute thrust and update the corresponding
  public: void Update(const UpdateInfo &_info, EntityComponentManager &_ecm);

  /// Model
  public: Model model{kNullEntity};

  /// Joint
  public: Entity jointEntity;
  public: Entity jointParentLink;
  public: Entity jointParentLinkEntity;
  public: std::string jointName;
  public: double JointVelocityCmd;
  public: double jointForceCmd;
  public: std::mutex jointForceCmdMutex;
  
  /// Link
  public: Entity linkEntity;
  public: std::string linkName;
  
  /*
  /// canonical_link
  public: Entity canonicallinkEntity;
  public: std::string canonicallinkName;
  */

  /// MOTOR PARAMETERS
  public: int Turn_Direction;
  public: std::string turningDirection;  
  public: double Thrust_Coeff = 2.2e-8;  /// Thrust Coeff [N/(rad/s)]
  public: double Torque_Coeff = 135.96e-12; /// Torque Coeff [N*m/(rad/s)]
  public: double C_tf = 6.18e-3 ; /// Torque-Thrust Coeff [N*m/N]

  /// FIRST ORDER FILTER BEHAVIOR
  public: float Thrust_input = 0.0f;  // Desired Thrust [N]
  public: double Tau_up = 0.05 ; /// Motor Time Constant (Up) [s]
  public: double Tau_down = 0.15; /// Motor Time Constant (Down) [s]
  public: double Sampling_time;
  public: double Prev_Sim_time = 0.0;
  public: double Prev_Thrust = 0.0;  

  /// CACULATED VALUES
  public: double Motor_Number;    // Slowed-down Rotational Velocity [rad/s]  
  public: double Thrust;              // Calculated Thrust [N]
  public: double Torque;              // Calculated Torque [N*m]
  public: double Rot_Vel = 0.0f;      // Rotational Velocity [rad/s]
  public: double Rot_Vel_Sim = 0.0f;      // Rotational Velocity in Simulation [rad/s]
  public: double Rot_Vel_Slowdown = 200.0;    // Slowed-down Rotational Velocity [rad/s]  
  
  /// Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdfConfig;
  
  /// Set during Load to true if the configuration for the system is valid and the post-update can run
  public: bool validConfig{false};
  /// Initialization flag
  public: bool initialized{false};
  
  ////!! ROS2 Publisher
  public: std::shared_ptr<rclcpp::Node> ros_node;
  public: rclcpp::Subscription<sar_msgs::msg::CtrlData>::SharedPtr pose_subscriber;

  /// ROS2 Callback for thrust subscription \param[in] _msg thrust message
  public: void Callback(const sar_msgs::msg::CtrlData::SharedPtr msg);


};


//////////////////////////////////////////////////
Motor_Plugin::Motor_Plugin()
    : System(), dataPtr(std::make_unique<Motor_PluginPrivate>())
{
}

//////////////////////////////////////////////////
void Motor_PluginPrivate::Load(const EntityComponentManager &_ecm,
                           const sdf::ElementPtr &_sdf)
{ 
  GZ_PROFILE("Motor_Plugin::Load");
  //std::cout << "Load is activated" << std::endl;

  // GRAB THE LINK FROM SDF
  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    auto linkName = elem->Get<std::string>();
    auto entities = entitiesFromScopedName(linkName, _ecm, this->model.Entity());

    // CHECK IF ENTITY TYPE IS EMPTY
    if (entities.empty()){
      gzerr << "Link with name[" << linkName << "] not found. ";
      this->validConfig = false;
      return;
    }

    // CHECK IF ENTITY TYPE'S SIZE IS ONLY ONE
    else if (entities.size() > 1){
      gzwarn << "Multiple link entities with name[" << linkName << "] found. "
             << "Using the first one.\n";
    }

    // USE FIRST ONE
    this->linkEntity = *entities.begin();
    //std::cout << "linkEntity: " << this->linkEntity << std::endl; 


    // CHECK IF ENTITY TYPE IS LINK
    if (!_ecm.EntityHasComponentType(this->linkEntity,
                                     components::Link::typeId)){
      this->linkEntity = kNullEntity;
      gzerr << "Entity with name[" << linkName << "] is not a link\n";
      this->validConfig = false;
      return;
    }
  }

  else{
    gzerr << "The Motor_plugin system requires the 'link_name' parameter\n";
    this->validConfig = false;
    return;
  }
/*
  // GRAB THE CANONICALLINK FROM SDF
  if (_sdf->HasElement("canonicallink_name"))
  {
    sdf::ElementPtr canonicalelem = _sdf->GetElement("canonicallink_name");
    auto canonicallinkName = canonicalelem->Get<std::string>();
    auto canonicalentities = entitiesFromScopedName(canonicallinkName, _ecm, this->model.Entity());

    // CHECK IF ENTITY TYPE IS EMPTY
    if (canonicalentities.empty()){
      gzerr << "Canonicallink with name[" << canonicallinkName << "] not found. ";
      this->validConfig = false;
      return;
    }

    // CHECK IF ENTITY TYPE'S SIZE IS ONLY ONE
    else if (canonicalentities.size() > 1){
      gzwarn << "Multiple Canonicallink entities with name[" << canonicallinkName << "] found. "
             << "Using the first one.\n";
    }

    // USE FIRST ONE
    this->canonicallinkEntity = *canonicalentities.begin();
    std::cout << "canonicallinkEntity: " << this->canonicallinkEntity << std::endl; 


    // CHECK IF ENTITY TYPE IS LINK
    if (!_ecm.EntityHasComponentType(this->canonicallinkEntity,
                                     components::Link::typeId)){
      this->canonicallinkEntity = kNullEntity;
      gzerr << "Entity with name[" << canonicallinkName << "] is not a link\n";
      this->validConfig = false;
      return;
    }
    
  }

  else{
    gzerr << "The Motor_plugin system requires the 'canonicallink_name' parameter\n";
    this->validConfig = false;
    return;
  }
*/

  // If we reached here, we have a valid configuration
  this->validConfig = true;

}

//////////////////////////////////////////////////
void Motor_PluginPrivate::Update(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm)
{
  GZ_PROFILE("Motor_Plugin::Update");

  // UPDATE THRUST VIA FIRST ORDER FILTER (INSTANTANEOUS THRUSTS ARE NOT POSSIBLE)
  if (this->Thrust_input >= this->Prev_Thrust)
  {
      // x(k) = alpha*x(k-1) + (1-alpha)*u(k)
      double alpha_up = exp(-0.000001 * this->Sampling_time / this->Tau_up);
      this->Thrust = alpha_up * this->Prev_Thrust + (1-alpha_up)*this->Thrust_input;

      //std::cout << "Thrust: " << this->Thrust << std::endl;
      //std::cout << "Prev_Thrust: " << this->Prev_Thrust << std::endl;
      //std::cout << "Sampling_time: " << this->Sampling_time << std::endl;
      //std::cout << "Tau_up: " << this->Tau_up << std::endl;
      //std::cout << "alpha_up: " << alpha_up << std::endl;
  }
  else // Thrust_input < prev_thrust
  {
      // x(k) = alpha*x(k-1) + (1-alpha)*u(k)
      double alpha_down = exp(-0.000001 * this->Sampling_time / this->Tau_down);//!!!!! need to analyze
      this->Thrust = alpha_down * this->Prev_Thrust + (1-alpha_down)*this->Thrust_input;

      //std::cout << "alpha_down: " << alpha_down << std::endl;
  }
  //std::cout << this->jointName << "'s thrust: " << this->Thrust << std::endl;

  /// APPLY THE FORCE TO PROPELLER(LINK) ON LINK'S FRAME

  // GET PROPELLER'S(LINK) POSE DIRECTION IN WORLD FRAME
  const auto worldPose = _ecm.Component<components::WorldPose>(this->linkEntity);
  //std::cout << "this->linkEntity" << this->linkEntity << std::endl;  
  //std::cout << "worldPose: " << worldPose << std::endl;  
  const auto &pose = worldPose->Data();
  //std::cout << "pose" << pose << std::endl; 
  
  // GET PROPELLER'S(LINK) Velocity IN WORLD FRAME
  //const auto worldVelocity = _ecm.Component<components::WorldLinearVelocity>(this->linkEntity);
  //std::cout << "this->linkEntity" << this->linkEntity << std::endl;  
  //std::cout << "worldVelocity: " << worldVelocity << std::endl;  
  //const auto &velocty = worldVelocity->Data();
  //std::cout << "velocty" << velocty  << std::endl; 

  //!!Torque need to solve!!!!!!!!!
  //!! GET PROPELLER'S(LINK) POSE DIRECTION IN WORLD FRAME
  //!!const auto ParentworldPose = _ecm.Component<components::WorldPose>(this->jointParentLinkEntity);
  //!!std::cout << "this->jointParentLinkEntity" << this->jointParentLinkEntity << std::endl;  
  //!!std::cout << "worldPose1: " << ParentworldPose << std::endl;  
  //!!const auto &Parentpose = ParentworldPose->Data();
  //!!std::cout << "Parentpose" << Parentpose << std::endl; 

  // CALCULATE THE FORCE ON WORLD FRAME
  gz::math::v7::Vector3d force0(0, 0, this->Thrust * g2Newton);
  //std::cout << "worldPose.Data" << this->Thrust_input << std::endl; 
  //std::cout << "force0 : " << this->jointName << force0 << std::endl;
  
  //ROTATE FORCE FROM WORLD FRAME TO LINK'S FRAME
  const auto force = pose.Rot().RotateVector(force0);
  //std::cout << "force : " << this->jointName << force << std::endl;

  //!!Torque CALCULATE THE TORQUE ON WORLD FRAME
  //!!gz::math::v7::Vector3d torque0(0, 0, -1 * this->Turn_Direction * this->C_tf * this->Thrust * g2Newton);
  //!!const auto torque = pose1.Rot().RotateVector(torque0);

  gz::math::v7::Vector3d zeros(0, 0, 0);

  // MAKE LINK CLASS INSTANCE 
  Link link(this->linkEntity);
  //!!Link link1(this->jointParentLinkEntity);

  // APPLY THE WRENCH ON PROPELLER(LINK)
  link.AddWorldWrench(_ecm, force, zeros);
  //!!link1.AddWorldWrench(_ecm, zeros, torque0);
  
  // UPDATE THE Prev_Thrust
  this->Prev_Thrust = this->Thrust;


  //std::cout << "Thrust_input: " << this->Thrust_input << std::endl; 

  

}

//////////////////////////////////////////////////
void Motor_Plugin::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm, EventManager &)
{ 
  
  //!! Initialize the RO2 node.
  if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
  }
  
  // GRAP THE MODEL
  this->dataPtr->model = Model(_entity);

  // CHECK THE MODEL
  std::cout << "Start Motor_Plugin, Model Name: " << this->dataPtr->model.Name(_ecm) << std::endl;
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Motor_Plugin plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->sdfConfig = _sdf->Clone();
  auto sdfClone = _sdf->Clone();

  // GRAB THE MOTOR JOINT FROM SDF
  auto sdfElem = sdfClone->GetElement("joint_name");
  if (sdfElem)
  {
    this->dataPtr->jointName = sdfElem->Get<std::string>();
  }

  if (this->dataPtr->jointName == "")
  {
    gzerr << "Motor_Plugin found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  // If the joint hasn't been identified yet, look for it
  //! [findJoint]
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
  }
  //! [findJoint]

  if (this->dataPtr->jointEntity == kNullEntity)
    return;

  ///!!Torque Parent model
  //!!const auto jointParentName = _ecm.Component<components::ParentEntity>(this->dataPtr->jointEntity)->Data();
  //!!const auto jointParentName = _ecm.Component<components::ParentEntity>(this->dataPtr->jointEntity);
  //!!const auto jointParentName = _ecm.Component<components::ParentLinkName>(this->dataPtr->jointEntity)->Data();
  //!!std::cout << "jointParentName : " << jointParentName << std::endl;
  //!!auto jointParentLinkEntity = entitiesFromScopedName(jointParentName, _ecm, this->dataPtr->model.Entity());
  //!!auto parententities = entitiesFromScopedName(jointParentName, _ecm, this->dataPtr->model.Entity());
  //!!this->dataPtr->jointParentLinkEntity = *parententities.begin();
  //!!std::cout << "jointParentLinkEntity : " << this->dataPtr->jointParentLinkEntity  << std::endl;

  // COLLECT THE Motor_Number FROM SDF
  this->dataPtr->Motor_Number = _sdf->Get<double>("Motor_Number");

  // COLLECT THE Rot_Vel_Slowdown FROM SDF
  //auto Rot_Vel_Slowdown = sdfClone->GetElement("Visual_Slowdown");
  this->dataPtr->Rot_Vel_Slowdown = _sdf->Get<double>("Visual_Slowdown");
  //std::cout << "Rot_Vel_Slowdown: " << this->dataPtr->Rot_Vel_Slowdown << std::endl;
  
  // COLLECT MOTOR TURNING DIRECTION
  std::string turningDirection = _sdf->Get<std::string>("Turning_Direction");
  if (turningDirection == "ccw")
  {
      this->dataPtr->Turn_Direction = 1;
  }
  else if (turningDirection == "cw")
  {
      this->dataPtr->Turn_Direction = -1;
  }
  else
  {
      gzerr << "[gz_motor_model] Please only use 'cw' or 'ccw' as Turning_Direction" << std::endl;
  }
  //std::cout << "turningDirection: " << turningDirection << std::endl;
  //std::cout << "this->dataPtr->Turn_Direction: " << this->dataPtr->Turn_Direction << std::endl;

  //!!!!Initialize ROS2 node and Publisher
  auto unique_name = "motor_plugin_node_" + this->dataPtr->jointName;
  //std::cout << "unique_name: " << unique_name << std::endl;
  this->dataPtr->ros_node = std::make_shared<rclcpp::Node>(unique_name);

  //auto unique_name1 = "motor_plugin_node_subscriber_" + this->dataPtr->jointName;
  //this->dataPtr->ros_node = std::make_shared<rclcpp::Node>(unique_name1);
  auto topic_name = "/CTRL/data";
  //std::cout << "topic_name: " << topic_name << std::endl;
  auto callback = [this](const sar_msgs::msg::CtrlData::SharedPtr msg) {
    this->dataPtr->Callback(msg);
  };
  this->dataPtr->pose_subscriber = this->dataPtr->ros_node->create_subscription<sar_msgs::msg::CtrlData>(
    topic_name, 1, callback);

}

//////////////////////////////////////////////////
void Motor_Plugin::PreUpdate(const UpdateInfo &_info, 
                                EntityComponentManager &_ecm)
{
  GZ_PROFILE("Motor_Plugin::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // CHECK IF SYSTEM IS INITIALIZED
  // This is not an "else" because "initialized" can be set in the if block
  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;

    if (this->dataPtr->validConfig)
    {
      Link link(this->dataPtr->linkEntity);
      link.EnableVelocityChecks(_ecm, true);

    }
  }

  // CHECK THE SIMULATION IS PAUSED
  if (_info.paused)
    return;

  // MEASUREE SAMPLING TIME
  this->dataPtr->Sampling_time = std::chrono::duration_cast<std::chrono::microseconds>(_info.simTime).count()-this->dataPtr->Prev_Sim_time;
  this->dataPtr->Prev_Sim_time = std::chrono::duration_cast<std::chrono::microseconds>(_info.simTime).count();

  // IF SYSTEM IS INITIALIED AND VALIDATED, RUN UPDATE FUNCTION
  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_info, _ecm);
    //std::cout << "Update is activated1" << std::endl;

    rclcpp::spin_some(this->dataPtr->ros_node);
    //rclcpp::shutdown();
  }

  /// Rotating Propeller
  // SET VISUAL VELOCITY OF ROTOR
  this->dataPtr->Rot_Vel = sqrt(this->dataPtr->Thrust/this->dataPtr->Thrust_Coeff);
  this->dataPtr->Rot_Vel_Sim = (this->dataPtr->Turn_Direction)*(this->dataPtr->Rot_Vel)/(this->dataPtr->Rot_Vel_Slowdown);
  //std::cout << "this->dataPtr->Turn_Direction: " << this->dataPtr->Turn_Direction << std::endl;
  //std::cout << "Rot_Vel_Slowdown: " << this->dataPtr->Rot_Vel_Slowdown << std::endl;
  //std::cout << "Rot_Vel_Sim: " << this->dataPtr->Rot_Vel_Sim << std::endl;

  // Update joint velocity
  Entity joint = this->dataPtr->jointEntity;
  _ecm.SetComponentData<components::JointVelocityCmd>(joint, {this->dataPtr->Rot_Vel_Sim});


}


//////////////////////////////////////////////////////////////
void Motor_PluginPrivate::Callback(const sar_msgs::msg::CtrlData::SharedPtr msg)
{
    //std::lock_guard<std::mutex> lock(this->jointForceCmdMutex);
    this->Thrust_input = msg->motorthrusts[this->Motor_Number-1];
    //std::cout << "Received Thrust_input: " << this->jointName << ": " << this->Thrust_input << std::endl;
}

/*
void Motor_Plugin::Update(const UpdateInfo &_info,
                                EntityComponentManager &_ecm)
{
}
*/

void Motor_Plugin::PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm)
{
  
}

//////////////////////////////////////////////////
// Register Plugin
GZ_ADD_PLUGIN(Motor_Plugin,
                    System,
                    Motor_Plugin::ISystemConfigure,
                    Motor_Plugin::ISystemPreUpdate,
                    //Motor_Plugin::ISystemUpdate,
                    Motor_Plugin::ISystemPostUpdate)
                    
GZ_ADD_PLUGIN_ALIAS(Motor_Plugin, "gz::sim::systems::Motor_Plugin")
