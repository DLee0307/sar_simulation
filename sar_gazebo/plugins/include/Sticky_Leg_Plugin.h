#ifndef GZ_SIM_SYSTEMS_STICKY_LEG_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_STICKY_LEG_PLUGIN_HH_

// For registering plugin
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

// For sdf_model
#include <sdf/Element.hh>

#include <unordered_map>

#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/contact.pb.h>

#include "gz/sim/Model.hh"
#include <gz/sim/Util.hh> 
#include "gz/sim/Conversions.hh"

#include "gz/sim/components/DetachableJoint.hh"
#include <gz/sim/components/Model.hh>
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"

#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/ContactSensorData.hh"

#include <memory>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>

#include "sar_msgs/msg/sticky_pad_connect.hpp"

#include "sar_msgs/srv/activate_sticky_pads.hpp"


namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class Sticky_Leg_PluginPrivate;

  class Sticky_Leg_Plugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit Sticky_Leg_Plugin();

    /// \brief Destructor
    public: ~Sticky_Leg_Plugin() override;

    /// \brief Gazebo transport node
    public: transport::Node node;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// ROS2 Callback for thrust subscription \param[in] _msg thrust message
    public: bool Service_Callback(const sar_msgs::srv::ActivateStickyPads::Request::SharedPtr request,
                                        sar_msgs::srv::ActivateStickyPads::Response::SharedPtr response);
/*
    public: void Load(const sdf::ElementPtr &_sdf, const std::string &_topic,
                      const std::vector<Entity> &_collisionEntities);

    public: void AddContacts(const std::chrono::steady_clock::duration &_stamp,
                            const msgs::Contacts &_contacts);

    /// \brief Publish sensor data over gz transport
    public: void Publish();

    /// \brief Topic to publish data to
    public: std::string topic;

    /// \brief Message to publish
    public: msgs::Contacts contactsMsg;


    /// \brief Gazebo transport publisher
    public: transport::Node::Publisher pub;

    /// \brief Entities for which this sensor publishes data
    public: std::vector<Entity> collisionEntities;
*/
    public: void SubscribeToContacts();

    private: void OnContactsMsg(const gz::msgs::Contacts &_msg);
    
    /// \brief Private data pointer.
    private: std::unique_ptr<Sticky_Leg_PluginPrivate> dataPtr;

    private: bool Sticky_Flag = false;
    private: bool Attached_Flag = false;


  };
  }
}
}
}
#endif
