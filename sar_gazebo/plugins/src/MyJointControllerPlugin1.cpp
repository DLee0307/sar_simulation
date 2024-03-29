#include "MyJointControllerPlugin.h"



//using namespace ignition;
//using namespace gazebo;
//using namespace systems;

//https://gist.github.com/nullpo24/146f253f0d789cb10c680786e7582940
MyJointControllerPlugin::MyJointControllerPlugin()
    : dataPtr(std::make_unique<MyJointControllerPluginPrivate>())
{
}

//Configure called once when the simulation starts or when the plugin is first loaded.
void MyJointControllerPlugin::Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr)
{
    this->dataPtr->model = ignition::gazebo::Model(_entity);

    // Get the canonical link
    std::vector<ignition::gazebo::Entity> links = _ecm.ChildrenByComponents(
        this->dataPtr->model.Entity(), ignition::gazebo::components::CanonicalLink());
    
    if (!links.empty())
        this->dataPtr->canonicalLink = ignition::gazebo::Link(links[0]);
    
    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "DiffDrive plugin should be attached to a model entity. "
            << "Failed to initialize." << std::endl;
        return;
    }
    auto ptr = const_cast<sdf::Element *>(_sdf.get());

    // Get params from SDF
    sdf::ElementPtr sdfElem = ptr->GetElement("left_joint");
    while (sdfElem)
    {
        this->dataPtr->leftJointNames.push_back(sdfElem->Get<std::string>());
        sdfElem = sdfElem->GetNextElement("left_joint");
    }

    sdfElem = ptr->GetElement("right_joint");
    while (sdfElem)
    {
        this->dataPtr->rightJointNames.push_back(sdfElem->Get<std::string>());
        sdfElem = sdfElem->GetNextElement("right_joint");
    }

    //DH:: can be problem in this part
    sdfElem = ptr->GetElement("model");
    while (sdfElem)
    {
        this->dataPtr->modelNames.push_back(sdfElem->Get<std::string>());
        sdfElem = sdfElem->GetNextElement("model");
    }

    this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
        this->dataPtr->wheelSeparation).first;
    this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
        this->dataPtr->wheelRadius).first;

    // Parse speed limiter parameters.
    bool hasVelocityLimits     = false;
    bool hasAccelerationLimits = false;
    bool hasJerkLimits         = false;
    double minVel              = std::numeric_limits<double>::lowest();
    double maxVel              = std::numeric_limits<double>::max();
    double minAccel            = std::numeric_limits<double>::lowest();
    double maxAccel            = std::numeric_limits<double>::max();
    double minJerk             = std::numeric_limits<double>::lowest();
    double maxJerk             = std::numeric_limits<double>::max();

    if (_sdf->HasElement("min_velocity"))
    {
        minVel = _sdf->Get<double>("min_velocity");
        hasVelocityLimits = true;
    }
    if (_sdf->HasElement("max_velocity"))
    {
        maxVel = _sdf->Get<double>("max_velocity");
        hasVelocityLimits = true;
    }
    if (_sdf->HasElement("min_acceleration"))
    {
        minAccel = _sdf->Get<double>("min_acceleration");
        hasAccelerationLimits = true;
    }
    if (_sdf->HasElement("max_acceleration"))
    {
        maxAccel = _sdf->Get<double>("max_acceleration");
        hasAccelerationLimits = true;
    }
    if (_sdf->HasElement("min_jerk"))
    {
        minJerk = _sdf->Get<double>("min_jerk");
        hasJerkLimits = true;
    }
    if (_sdf->HasElement("max_jerk"))
    {
        maxJerk = _sdf->Get<double>("max_jerk");
        hasJerkLimits = true;
    }
    
    // Instantiate the speed limiters.
    /*this->dataPtr->limiterLin = std::make_unique<ignition::gazebo::systems::SpeedLimiter>(
        hasVelocityLimits, hasAccelerationLimits, hasJerkLimits,
        minVel, maxVel, minAccel, maxAccel, minJerk, maxJerk);
    
    this->dataPtr->limiterAng = std::make_unique<ignition::gazebo::systems::SpeedLimiter>(
        hasVelocityLimits, hasAccelerationLimits, hasJerkLimits,
        minVel, maxVel, minAccel, maxAccel, minJerk, maxJerk);*/
    
    double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
    if (odomFreq > 0)
    {
        std::chrono::duration<double> odomPer{1 / odomFreq};
        this->dataPtr->odomPubPeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
    }


    // Setup odometry.
    this->dataPtr->odom.SetWheelParams(this->dataPtr->wheelSeparation,
        this->dataPtr->wheelRadius, this->dataPtr->wheelRadius);

    // Subscribe to commands
    std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
    if (_sdf->HasElement("topic"))
        topic = _sdf->Get<std::string>("topic");
    std::cout << "topic:" << topic << std::endl;
    this->dataPtr->node.Subscribe(topic, &MyJointControllerPluginPrivate::OnCmdVel,
        this->dataPtr.get());

    std::string odomTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
        "/odometry"};
    this->dataPtr->odomPub = this->dataPtr->node.Advertise<ignition::msgs::Odometry>(
        odomTopic);


    // Subscribe to pose commands
    // this->dataPtr->node.Subscribe("/PosePublisher/geometry_msgs/Pose", &DiffDrivePrivate::OnCmdPose, this->dataPtr.get());
    this->dataPtr->node.Subscribe("/pose_publisher", &MyJointControllerPluginPrivate::OnCmdPose, this->dataPtr.get());

    if (_sdf->HasElement("frame_id"))
        this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

    if (_sdf->HasElement("child_frame_id"))
        this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");

    ignmsg << "DiffDrive subscribing to twist messages on [" << topic << "]"
            << std::endl;

    std::cout << "\t Link Entity Name:\t" << ptr << std::endl;
}




void MyJointControllerPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("DiffDrive::PreUpdate");

    // \TODO(anyone) Support rewind
    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
        ignwarn << "Detected jump back in time ["
            << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
            << "s]. System may not work properly." << std::endl;
    }

    // If the joints haven't been identified yet, look for them
    static std::set<std::string> warnedModels;
    auto modelName = this->dataPtr->model.Name(_ecm);
    if (this->dataPtr->leftJoints.empty() ||
        this->dataPtr->rightJoints.empty() ||
        this->dataPtr->models.empty())
    {
        bool warned{false};
        for (const std::string &name : this->dataPtr->leftJointNames)
        {
            ignition::gazebo::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
            if (joint != ignition::gazebo::kNullEntity)
                this->dataPtr->leftJoints.push_back(joint);
            else if (warnedModels.find(modelName) == warnedModels.end())
            {
                ignwarn << "Failed to find left joint [" << name << "] for model ["
                        << modelName << "]" << std::endl;
                warned = true;
            }
        }
        
        for (const std::string &name : this->dataPtr->rightJointNames)
        {
            ignition::gazebo::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
            if (joint != ignition::gazebo::kNullEntity)
                this->dataPtr->rightJoints.push_back(joint);
            else if (warnedModels.find(modelName) == warnedModels.end())
            {
                ignwarn << "Failed to find right joint [" << name << "] for model ["
                        << modelName << "]" << std::endl;
                warned = true;
            }
        }
        for (const std::string &name : this->dataPtr->modelNames)
        {
            ignition::gazebo::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
            if (joint != ignition::gazebo::kNullEntity)
                this->dataPtr->models.push_back(joint);
        }

        if (warned)
        {
            warnedModels.insert(modelName);
        }

    }
    
    if (this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty())
        return;


    if (warnedModels.find(modelName) != warnedModels.end())
    {
        ignmsg << "Found joints for model [" << modelName
            << "], plugin will start working." << std::endl;
        warnedModels.erase(modelName);
    }


    // Nothing left to do if paused.
    if (_info.paused)
        return;

    for (ignition::gazebo::Entity joint : this->dataPtr->leftJoints)
    {
        // Update wheel velocity
        auto vel = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint);

        if (vel == nullptr)
        {
        _ecm.CreateComponent(
            joint, ignition::gazebo::components::JointVelocityCmd({this->dataPtr->leftJointSpeed}));
        }
        else
        {
        *vel = ignition::gazebo::components::JointVelocityCmd({this->dataPtr->leftJointSpeed});
        }
    }

    for (ignition::gazebo::Entity joint : this->dataPtr->rightJoints)
    {
        // Update wheel velocity
        auto vel = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint);

        if (vel == nullptr)
        {
        _ecm.CreateComponent(joint,
            ignition::gazebo::components::JointVelocityCmd({this->dataPtr->rightJointSpeed}));
        }
        else
        {
        *vel = ignition::gazebo::components::JointVelocityCmd({this->dataPtr->rightJointSpeed});
        }
    }

    // Create the left and right side joint position components if they
    // don't exist.
    auto leftPos = _ecm.Component<ignition::gazebo::components::JointPosition>(
        this->dataPtr->leftJoints[0]);
    if (!leftPos)
    {
        _ecm.CreateComponent(this->dataPtr->leftJoints[0],
            ignition::gazebo::components::JointPosition());
    }

    auto rightPos = _ecm.Component<ignition::gazebo::components::JointPosition>(
        this->dataPtr->rightJoints[0]);
    if (!rightPos)
    {
        _ecm.CreateComponent(this->dataPtr->rightJoints[0],
            ignition::gazebo::components::JointPosition());
    }

    auto pose = ignition::gazebo::convert<ignition::math::Pose3d>(this->dataPtr->targetPose);
    this->dataPtr->model.SetWorldPoseCmd(_ecm, pose);
}


void MyJointControllerPlugin::Update(const ignition::gazebo::UpdateInfo &_info, 
                        ignition::gazebo::EntityComponentManager &_ecm)
{
}

void MyJointControllerPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info, 
                                        const ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("DiffDrive::PostUpdate");
    // Nothing left to do if paused.
    if (_info.paused)
        return;

    this->dataPtr->UpdateVelocity(_info, _ecm);
    this->dataPtr->UpdateOdometry(_info, _ecm);
    this->dataPtr->UpdatePose(_info, _ecm);  
}

void MyJointControllerPluginPrivate::UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
                                                    const ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("DiffDrive::UpdateOdometry");
    // Initialize, if not already initialized.
    if (!this->odom.Initialized())
    {
        this->odom.Init(std::chrono::steady_clock::time_point(_info.simTime));
        return;
    }

    if (this->leftJoints.empty() || this->rightJoints.empty())
        return;

    // Get the first joint positions for the left and right side.
    auto leftPos = _ecm.Component<ignition::gazebo::components::JointPosition>(this->leftJoints[0]);
    auto rightPos = _ecm.Component<ignition::gazebo::components::JointPosition>(this->rightJoints[0]);

    // Abort if the joints were not found or just created.
    if (!leftPos || !rightPos || leftPos->Data().empty() || rightPos->Data().empty())
    {
        return;
    }

    this->odom.Update(leftPos->Data()[0], rightPos->Data()[0],
      std::chrono::steady_clock::time_point(_info.simTime));

    // Throttle publishing
    auto diff = _info.simTime - this->lastOdomPubTime;
    if (diff > std::chrono::steady_clock::duration::zero() &&
        diff < this->odomPubPeriod)
    {
        return;
    }
    this->lastOdomPubTime = _info.simTime;

    // Construct the odometry message and publish it.
    ignition::msgs::Odometry msg;
    msg.mutable_pose()->mutable_position()->set_x(this->odom.X());
    msg.mutable_pose()->mutable_position()->set_y(this->odom.Y());

    ignition::math::Quaterniond orientation(0, 0, *this->odom.Heading());
    ignition::msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

    msg.mutable_twist()->mutable_linear()->set_x(this->odom.LinearVelocity());
    msg.mutable_twist()->mutable_angular()->set_z(*this->odom.AngularVelocity());

    // Set the time stamp in the header
    msg.mutable_header()->mutable_stamp()->CopyFrom(
        ignition::gazebo::convert<ignition::msgs::Time>(_info.simTime));

    // Set the frame id.
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    if (this->sdfFrameId.empty())
    {
        frame->add_value(this->model.Name(_ecm) + "/odom");
    }
    else
    {
        frame->add_value(this->sdfFrameId);
    }

    std::optional<std::string> linkName = this->canonicalLink.Name(_ecm);
    if (this->sdfChildFrameId.empty())
    {
        if (linkName)
        {
        auto childFrame = msg.mutable_header()->add_data();
        childFrame->set_key("child_frame_id");
        childFrame->add_value(this->model.Name(_ecm) + "/" + *linkName);
        }
    }
    else
    {
        auto childFrame = msg.mutable_header()->add_data();
        childFrame->set_key("child_frame_id");
        childFrame->add_value(this->sdfChildFrameId);
    }

    // Construct the Pose_V/tf message and publish it.
    ignition::msgs::Pose_V tfMsg;
    ignition::msgs::Pose *tfMsgPose = nullptr;
    tfMsgPose = tfMsg.add_pose();
    tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
    tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
    tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());

    // Publish the messages
    this->odomPub.Publish(msg);
    this->tfPub.Publish(tfMsg);
    
}

void MyJointControllerPluginPrivate::UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
                                                    const ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("DiffDrive::UpdateVelocity");

    double linVel;
    double angVel;
    {
        std::lock_guard<std::mutex> lock(this->mutex);
        linVel = this->targetVel.linear().x();
        angVel = this->targetVel.angular().z();
    }


    const double dt = std::chrono::duration<double>(_info.dt).count();

    // Limit the target velocity if needed.
    //this->limiterLin->Limit(linVel, this->last0Cmd.lin, this->last1Cmd.lin, dt);
    //this->limiterAng->Limit(angVel, this->last0Cmd.ang, this->last1Cmd.ang, dt);

    // Update history of commands.
    this->last1Cmd = last0Cmd;
    this->last0Cmd.lin = linVel;
    this->last0Cmd.ang = angVel;

    // Convert the target velocities to joint velocities.
    this->rightJointSpeed =
        (linVel + angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
    this->leftJointSpeed =
        (linVel - angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
}

void MyJointControllerPluginPrivate::UpdatePose(const ignition::gazebo::UpdateInfo &_info,
                                                const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  IGN_PROFILE("DiffDrive::UpdatePose");

  auto pose = ignition::gazebo::convert<ignition::math::Pose3d>(this->targetPose);
  // math::Pose3d pose;
  // this->dataPtr->model.SetWorldPoseCmd(_ecm, math::Pose3d(this->targetPose.position().x, 2, 3, 0, 0, 0, 0));
  // this->model.SetWorldPoseCmd(_ecm, pose);

}


void MyJointControllerPluginPrivate::OnCmdVel(const ignition::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}


void MyJointControllerPluginPrivate::OnCmdPose(const ignition::msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetPose = _msg;
}

IGNITION_ADD_PLUGIN(MyJointControllerPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate,
                    ignition::gazebo::ISystemUpdate,
                    ignition::gazebo::ISystemPostUpdate)
                    
IGNITION_ADD_PLUGIN_ALIAS(MyJointControllerPlugin, "my_namespace::MyJointControllerPlugin")