#include "Sticky_Leg_Plugin.h"
//joint_entity = creator_->CreateEntities(&joint);

#include <std_msgs/msg/string.hpp>

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

    public: bool contact_flag{false};
    public: bool attach_flag{false};

    public: std::string Child_Name;
    public: std::string Child_Model_Name;
    public: std::string Child_Link_Name;

    public: std::string Parent_Name;
    public: std::string Parent_Model_Name;
    public: std::string Parent_Link_Name;

    public: Entity parentLinkEntity{kNullEntity};
    public: Entity childLinkEntity{kNullEntity};
    public: Entity detachableJointEntity{kNullEntity};

    public: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    public: rclcpp::Publisher<sar_msgs::msg::StickyPadConnect>::SharedPtr Sticky_Pad_Connect_Publisher;
    public: sar_msgs::msg::StickyPadConnect Sticky_Leg_Connect_msg;
    
/*
    /// \brief Create sensors that correspond to entities in the simulation
    /// \param[in] _ecm Mutable reference to ECM.
    public: void CreateSensors(EntityComponentManager &_ecm);

    /// \brief Update and publish sensor data
    /// \param[in] _ecm Immutable reference to ECM.
    public: void UpdateSensors(const UpdateInfo &_info,
                              const EntityComponentManager &_ecm);

    /// \brief Remove sensors if their entities have been removed from
    /// simulation.
    /// \param[in] _ecm Immutable reference to ECM.
    public: void RemoveSensors(const EntityComponentManager &_ecm);

    /// \brief A map of Contact entity to its Contact sensor.
    public: std::unordered_map<Entity,
        std::unique_ptr<Sticky_Leg_Plugin>> entitySensorMap;
*/


};

//////////////////////////////////////////////////
Sticky_Leg_Plugin::Sticky_Leg_Plugin() : System(), dataPtr(std::make_unique<Sticky_Leg_PluginPrivate>())
{
  
}

//////////////////////////////////////////////////
Sticky_Leg_Plugin::~Sticky_Leg_Plugin() = default;
/*
//////////////////////////////////////////////////
void Sticky_Leg_Plugin::Load(const sdf::ElementPtr &_sdf, const std::string &_topic,
                         const std::vector<Entity> &_collisionEntities)
{
  this->collisionEntities = _collisionEntities;

  auto contactElem = _sdf->GetElement("contact");
  auto tmpTopic = contactElem->Get<std::string>("topic", "__default_topic__").first;

  if (tmpTopic == "__default_topic__")
  {
    // use default topic for sensor
    this->topic = _topic;
  }
  else
  {
    this->topic = tmpTopic;
  }

  gzmsg << "Contact system publishing on " << this->topic << std::endl;
  std::cout << "topic is made: " << std::endl;
  this->pub = this->node.Advertise<gz::msgs::Contacts>(this->topic);

}

//////////////////////////////////////////////////
void Sticky_Leg_Plugin::AddContacts(
    const std::chrono::steady_clock::duration &_stamp,
    const msgs::Contacts &_contacts)
{
  std::cout << "AddContacts Loop: " << std::endl;
  auto stamp = convert<msgs::Time>(_stamp);
  for (const auto &contact : _contacts.contact())
  {
      std::cout << "Collision 1:!!!!!! " << std::endl;
      //std::cout << "Collision 1: " << contact.collision1().DebugString() << std::endl;
      //std::cout << "Collision 2: " << contact.collision2() << std::endl;
      //std::cout << "Number of positions: " << contact.position_size() << std::endl;

      auto *newContact = this->contactsMsg.add_contact();
      newContact->CopyFrom(contact);
      newContact->mutable_header()->mutable_stamp()->CopyFrom(stamp);
  }

  this->contactsMsg.mutable_header()->mutable_stamp()->CopyFrom(stamp);
}

//////////////////////////////////////////////////
void Sticky_Leg_Plugin::Publish()
{
  // Only publish if there are contacts
  //std::cout << "Publish function called" << std::endl;
  //std::cout << "contactsMsg.contact_size: " << this->contactsMsg.contact_size() << std::endl;
  if (this->contactsMsg.contact_size() > 0)
  {
    this->pub.Publish(this->contactsMsg);
    this->contactsMsg.Clear();
  }
}

//////////////////////////////////////////////////
void Sticky_Leg_PluginPrivate::CreateSensors(EntityComponentManager &_ecm)
{
  GZ_PROFILE("Sticky_Leg_PluginPrivate::CreateSensors");
  // Run
  //std::cout << "CreateSensors loop: " << std::endl;

  _ecm.EachNew<components::ContactSensor>(
      [&](const Entity &_entity,
          const components::ContactSensor *_contact) -> bool
      {
          auto *parentEntity = _ecm.Component<components::ParentEntity>(_entity);
          if (nullptr == parentEntity)
              return true;

          auto *linkComp = _ecm.Component<components::Link>(parentEntity->Data());
          if (nullptr == linkComp)
          {
              return true;
          }

          auto collisionElem = _contact->Data()->GetElement("contact")->GetElement("collision");

          std::vector<Entity> collisionEntities;
          for (; collisionElem; collisionElem = collisionElem->GetNextElement("collision"))
          {
              auto collisionName = collisionElem->Get<std::string>();
              auto childEntities = _ecm.ChildrenByComponents(
                  parentEntity->Data(), components::Collision(),
                  components::Name(collisionName));

              if (!childEntities.empty())
              {
                  collisionEntities.push_back(childEntities.front());
                  _ecm.CreateComponent(childEntities.front(),
                                        components::ContactSensorData());
              }
          }

          std::string defaultTopic = scopedName(_entity, _ecm, "/") + "/contact";
          std::cout << "defaultTopic : " << defaultTopic << std::endl;
          
          auto sensor = std::make_unique<Sticky_Leg_Plugin>();
          std::cout << "Load : " << std::endl;
          sensor->Load(_contact->Data(), defaultTopic, collisionEntities);
          this->entitySensorMap.insert(
              std::make_pair(_entity, std::move(sensor)));

          return true;
      });
}
//////////////////////////////////////////////////


void Sticky_Leg_PluginPrivate::UpdateSensors(const UpdateInfo &_info,
                                            const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Sticky_Leg_PluginPrivate::UpdateSensors");
  //std::cout << "UpdateSensors Loop : " << std::endl;
  for (const auto &item : this->entitySensorMap)
  {
    for (const Entity &entity : item.second->collisionEntities)
    {
      auto contacts = _ecm.Component<components::ContactSensorData>(entity);

      if (contacts->Data().contact_size() > 0)
      {
        std::cout << "AddContacts : " << std::endl;
        item.second->AddContacts(_info.simTime, contacts->Data());
      }
    }
  }
}

//////////////////////////////////////////////////
void Sticky_Leg_PluginPrivate::RemoveSensors(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("ContactPrivate::RemoveSensors");
  _ecm.EachRemoved<components::ContactSensor>(
    [&](const Entity &_entity,
        const components::ContactSensor *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing Contact sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}
*/
//////////////////////////////////////////////////
void Sticky_Leg_Plugin::SubscribeToContacts()
{
    //this->node.Subscribe("/SAR_Internal/Leg_Connections_3", &Sticky_Leg_Plugin::OnContactsMsg, this);
    this->node.Subscribe("/SAR_Internal/Leg_Connections_" + std::to_string(this->dataPtr->Leg_Number), &Sticky_Leg_Plugin::OnContactsMsg, this);
}

//////////////////////////////////////////////////
void Sticky_Leg_Plugin::OnContactsMsg(const gz::msgs::Contacts &_msg)
{
    std::cout << "Received contacts message:" << this->dataPtr->Leg_Number << std::endl;
    

    for (const auto &contact : _msg.contact())
    {
        //std::cout << "Collision 1 Name: " << contact.collision1().name() << std::endl;
        //std::cout << "Collision 2 id: " << contact.collision2().id() << std::endl;
        //std::cout << "Collision 2 Name: " << contact.collision2().name() << std::endl;
        this->dataPtr->Child_Name = contact.collision2().name();
        //std::cout << "Child_Name: " << this->dataPtr->Child_Name << std::endl;

        this->dataPtr->contact_flag = true;

        // String splitting logic
        std::string delimiter = "::";
        size_t pos = 0;
        std::string token;
        std::vector<std::string> components;

        std::string name = this->dataPtr->Child_Name;
        while ((pos = name.find(delimiter)) != std::string::npos) {
            token = name.substr(0, pos);
            components.push_back(token);
            name.erase(0, pos + delimiter.length());
        }
        components.push_back(name);

        // Print Output
        for (const auto &part : components) {
            //std::cout << "Component: " << part << std::endl;
        }
        //std::cout << "Component 1: " << components[0] << std::endl;
        //std::cout << "Component 2: " << components[1] << std::endl;
        //std::cout << "Component 3: " << components[2] << std::endl;

        this->dataPtr->Parent_Model_Name = components[0];
        this->dataPtr->Parent_Link_Name = components[1];

        // components[0] = "Desert_Pattern", components[1] = "Ground_Pattern", components[2] = "Surface_Collision"

        for (int i = 0; i < contact.position_size(); ++i)
        {
            /*std::cout << "Position " << i << ": (" 
                      << contact.position(i).x() << ", " 
                      << contact.position(i).y() << ", " 
                      << contact.position(i).z() << ")" << std::endl;*/
        }

        //std::cout << "Depth: " << contact.depth(0) << std::endl;
    }
}


//////////////////////////////////////////////////
void Sticky_Leg_Plugin::Configure(const Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  EntityComponentManager &_ecm,
                                  EventManager &/*_eventMgr*/)
{ 
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "DetachableJoint should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();

  // Clone _sdf to modify and extract elements
  sdf::ElementPtr sdfClone = _sdf->Clone();

  // GRAB THE JOINT FROM SDF
  if (sdfClone->HasElement("JointName"))
  {
    sdf::ElementPtr elem = sdfClone->GetElement("JointName");
    this->dataPtr->Joint_Name = elem->Get<std::string>();
    std::cout << "Joint_Name: " << this->dataPtr->Joint_Name << std::endl; 
  }

  // GRAB THE LINK FROM SDF
  if (sdfClone->HasElement("LinkName"))
  {
    sdf::ElementPtr elem = sdfClone->GetElement("LinkName");
    this->dataPtr->Link_Name = elem->Get<std::string>();
    std::cout << "Link_Name: " << this->dataPtr->Link_Name << std::endl; 
  }

  // GRAB THE LEGNUMBER FROM SDF
  if (sdfClone->HasElement("LegNumber"))
  {
    sdf::ElementPtr elem = sdfClone->GetElement("LegNumber");
    this->dataPtr->Leg_Number = elem->Get<int>();
    std::cout << "Leg_Number: " << this->dataPtr->Leg_Number << std::endl; 
  }

  if (_sdf->HasElement("child_link"))
  {
    auto childLinkName = _sdf->Get<std::string>("child_link");
    this->dataPtr->childLinkEntity = this->dataPtr->model.LinkByName(_ecm, childLinkName);
    if (kNullEntity == this->dataPtr->childLinkEntity)
    {
      gzerr << "Link with name " << childLinkName
            << " not found in model " << this->dataPtr->model.Name(_ecm)
            << ". Failed to initialize.\n";
      return;
    }
  }

  if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
  }
  this->dataPtr->ros_node = std::make_shared<rclcpp::Node>("sticky_leg_plugin_node_" + std::to_string(this->dataPtr->Leg_Number));
  this->dataPtr->Sticky_Pad_Connect_Publisher = this->dataPtr->ros_node->create_publisher<sar_msgs::msg::StickyPadConnect>("/SAR_Internal/Leg_Connections" + std::to_string(this->dataPtr->Leg_Number), 5);
  this->SubscribeToContacts();
}

//////////////////////////////////////////////////
void Sticky_Leg_Plugin::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Sticky_Leg_Plugin::PreUpdate");
  //std::cout << "contact_flag : " << this->dataPtr->contact_flag << std::endl;
  
  // Run
  //std::cout << "CreateSensors : " << std::endl;
  //this->dataPtr->CreateSensors(_ecm);
  if (this->dataPtr->contact_flag && !this->dataPtr->attach_flag)
  {
    Entity modelEntity = _ecm.EntityByComponents(
        components::Model(), components::Name(this->dataPtr->Parent_Model_Name));
    //std::cout << "modelEntity : " << modelEntity << std::endl;
    
    this->dataPtr->parentLinkEntity = _ecm.EntityByComponents(
        components::Link(), components::ParentEntity(modelEntity),
        components::Name(this->dataPtr->Parent_Link_Name)); 
    //std::cout << "childLinkEntity : " << this->dataPtr->childLinkEntity << std::endl;

    //std::cout << "parentLinkEntity : " << this->dataPtr->parentLinkEntity << std::endl;

    if (kNullEntity != this->dataPtr->parentLinkEntity)
    {
      this->dataPtr->detachableJointEntity = _ecm.CreateEntity();
      _ecm.CreateComponent(
          this->dataPtr->detachableJointEntity,
          components::DetachableJoint({this->dataPtr->parentLinkEntity,
                                       this->dataPtr->childLinkEntity, "fixed"}));

      this->dataPtr->attach_flag = true;
      gzdbg << "Entity attached." << std::endl;

      switch(this->dataPtr->Leg_Number)
      {
          case 1:
            this->dataPtr->Sticky_Leg_Connect_msg.pad1_contact = 1;
              break;
          case 2:
            this->dataPtr->Sticky_Leg_Connect_msg.pad2_contact = 1;
              break;
          case 3:
            this->dataPtr->Sticky_Leg_Connect_msg.pad3_contact = 1;
              break;
          case 4:
            this->dataPtr->Sticky_Leg_Connect_msg.pad4_contact = 1;
              break;
      }

      //std::cout << "Sticky_Leg_Connect_msg : " << Sticky_Leg_Connect_msg.pad1_contact << std::endl;
      /**/
      this->dataPtr->Sticky_Pad_Connect_Publisher->publish(this->dataPtr->Sticky_Leg_Connect_msg);
      }

  }


    
}

//////////////////////////////////////////////////
void Sticky_Leg_Plugin::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Sticky_Leg_Plugin::PostUpdate");
  //std::cout << "PostUpdate Loop : " << std::endl;

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }
/*
  if (!_info.paused)
  {
    //std::cout << "UpdateSensors : " << std::endl;
    this->dataPtr->UpdateSensors(_info, _ecm);
    //std::cout << "entitySensorMap size: " << this->dataPtr->entitySensorMap.size() << std::endl;

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Publish sensor data
      it.second->Publish();
    }
  }

  this->dataPtr->RemoveSensors(_ecm);*/
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
  return true;
}

GZ_ADD_PLUGIN(Sticky_Leg_Plugin, System,
  Sticky_Leg_Plugin::ISystemConfigure,
  Sticky_Leg_Plugin::ISystemPreUpdate,
  Sticky_Leg_Plugin::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(Sticky_Leg_Plugin, "gz::sim::systems::Sticky_Leg_Plugin")