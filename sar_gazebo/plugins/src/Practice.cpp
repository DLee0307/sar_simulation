#include "Practice.h"
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/msgs/stringmsg.pb.h>

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::PracticePrivate
{
  public: Model model{kNullEntity};
  public: Entity parentLinkEntity{kNullEntity};
  public: std::string childModelName;
  public: std::string childLinkName;
  public: Entity childLinkEntity{kNullEntity};
  public: bool isAttached{false};
  public: bool attachRequested{true};
  public: bool detachRequested{false};
  public: Entity detachableJointEntity{kNullEntity};
  public: transport::Node node;
  public: std::string detachTopic;
  public: std::string attachTopic;
  public: std::string outputTopic;
  public: transport::Node::Publisher outputPub;
};

Practice::Practice() : System(), dataPtr(std::make_unique<PracticePrivate>())
{
}

Practice::~Practice() = default;

std::string validTopic(const std::vector<std::string> &topics)
{
  for (const auto &topic : topics)
  {
    if (!topic.empty())
    {
      return topic;
    }
  }
  return std::string();
}

void Practice::Configure(const Entity &_entity,
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

  if (_sdf->HasElement("parent_link"))
  {
    auto parentLinkName = _sdf->Get<std::string>("parent_link");
    this->dataPtr->parentLinkEntity = this->dataPtr->model.LinkByName(_ecm, parentLinkName);
    if (kNullEntity == this->dataPtr->parentLinkEntity)
    {
      gzerr << "Link with name " << parentLinkName
            << " not found in model " << this->dataPtr->model.Name(_ecm)
            << ". Failed to initialize.\n";
      return;
    }
  }

  if (_sdf->HasElement("child_model"))
  {
    this->dataPtr->childModelName = _sdf->Get<std::string>("child_model");
  }

  if (_sdf->HasElement("child_link"))
  {
    this->dataPtr->childLinkName = _sdf->Get<std::string>("child_link");
  }

  std::vector<std::string> detachTopics;
  if (_sdf->HasElement("detach_topic"))
  {
    detachTopics.push_back(_sdf->Get<std::string>("detach_topic"));
  }
  this->dataPtr->detachTopic = validTopic(detachTopics);
  this->dataPtr->node.Subscribe(this->dataPtr->detachTopic, &Practice::OnDetachRequest, this);

  std::vector<std::string> attachTopics;
  if (_sdf->HasElement("attach_topic"))
  {
    attachTopics.push_back(_sdf->Get<std::string>("attach_topic"));
  }
  this->dataPtr->attachTopic = validTopic(attachTopics);

  auto msgCb = std::function<void(const gz::transport::ProtoMsg &)>(
    [this](const auto &)
    {
      if (!this->dataPtr->isAttached)
      {
        this->dataPtr->attachRequested = true;
      }
    });

  this->dataPtr->node.Subscribe(this->dataPtr->attachTopic, msgCb);

  std::vector<std::string> outputTopics;
  if (_sdf->HasElement("output_topic"))
  {
    outputTopics.push_back(_sdf->Get<std::string>("output_topic"));
  }
  this->dataPtr->outputTopic = validTopic(outputTopics);
  this->dataPtr->outputPub = this->dataPtr->node.Advertise<gz::msgs::StringMsg>(this->dataPtr->outputTopic);
}

void Practice::PreUpdate(const UpdateInfo & /*_info*/, EntityComponentManager &_ecm)
{
  //std::cout << "attachRequested: " << this->dataPtr->attachRequested << std::endl;
  //std::cout << "isAttached: " << this->dataPtr->isAttached << std::endl;

  if (this->dataPtr->attachRequested && !this->dataPtr->isAttached)
  {
    Entity modelEntity = _ecm.EntityByComponents(
        components::Model(), components::Name(this->dataPtr->childModelName));
    this->dataPtr->childLinkEntity = _ecm.EntityByComponents(
        components::Link(), components::ParentEntity(modelEntity),
        components::Name(this->dataPtr->childLinkName));

    if (kNullEntity != this->dataPtr->childLinkEntity)
    {
      this->dataPtr->detachableJointEntity = _ecm.CreateEntity();
      _ecm.CreateComponent(
          this->dataPtr->detachableJointEntity,
          components::DetachableJoint({this->dataPtr->parentLinkEntity,
                                       this->dataPtr->childLinkEntity, "fixed"}));

      std::cout << "parentLinkEntity: " << this->dataPtr->parentLinkEntity << std::endl;
      std::cout << "parentLinkEntity: " << this->dataPtr->childLinkEntity << std::endl;

      this->dataPtr->attachRequested = false;
      this->dataPtr->isAttached = true;
      gzdbg << "Entity attached." << std::endl;
    }
  }

  if (this->dataPtr->detachRequested && this->dataPtr->isAttached)
  {
    _ecm.RequestRemoveEntity(this->dataPtr->detachableJointEntity);
    this->dataPtr->detachableJointEntity = kNullEntity;
    this->dataPtr->detachRequested = false;
    this->dataPtr->isAttached = false;
    gzdbg << "Entity detached." << std::endl;
  }
}

void Practice::OnDetachRequest(const gz::msgs::Empty &)
{
  if (!this->dataPtr->isAttached)
  {
    gzdbg << "Already detached" << std::endl;
    return;
  }
  this->dataPtr->detachRequested = true;
}

GZ_ADD_PLUGIN(Practice, System,
  Practice::ISystemConfigure,
  Practice::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(Practice, "gz::sim::systems::Practice")
