#include "Sticky_Leg_Plugin.h"
//joint_entity = creator_->CreateEntities(&joint);

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



};

//////////////////////////////////////////////////
Sticky_Leg_Plugin::Sticky_Leg_Plugin() : System(), dataPtr(std::make_unique<Sticky_Leg_PluginPrivate>())
{
}

//////////////////////////////////////////////////
Sticky_Leg_Plugin::~Sticky_Leg_Plugin() = default;

//////////////////////////////////////////////////
void Sticky_Leg_Plugin::Load(const sdf::ElementPtr &_sdf, const std::string &_topic,
                         const std::vector<Entity> &_collisionEntities)
{
  this->collisionEntities = _collisionEntities;

  auto contactElem = _sdf->GetElement("contact");
  auto tmpTopic =
      contactElem->Get<std::string>("topic", "__default_topic__").first;

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
  this->pub = this->node.Advertise<gz::msgs::Contacts>(this->topic);

  } 
//////////////////////////////////////////////////
  void Sticky_Leg_Plugin::AddContacts(
      const std::chrono::steady_clock::duration &_stamp,
      const msgs::Contacts &_contacts)
  {
    auto stamp = convert<msgs::Time>(_stamp);
    for (const auto &contact : _contacts.contact())
    {
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

            auto sensor = std::make_unique<Sticky_Leg_Plugin>();
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
      for (const auto &item : this->entitySensorMap)
      {
          for (const Entity &entity : item.second->collisionEntities)
          {
              auto contacts = _ecm.Component<components::ContactSensorData>(entity);

              if (contacts->Data().contact_size() > 0)
              {
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
}

//////////////////////////////////////////////////
void Sticky_Leg_Plugin::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Sticky_Leg_Plugin::PreUpdate");
  this->dataPtr->CreateSensors(_ecm);
}

//////////////////////////////////////////////////
void Sticky_Leg_Plugin::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Contact::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (!_info.paused)
  {
    this->dataPtr->UpdateSensors(_info, _ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Publish sensor data
      it.second->Publish();
    }
  }

  this->dataPtr->RemoveSensors(_ecm);
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