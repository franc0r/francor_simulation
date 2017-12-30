#include "gazebo-francor-morty-motor-controller-plugin.h"

namespace nrbdl {

GZ_REGISTER_MODEL_PLUGIN(GazeboFrancorMortyMotorController)

GazeboFrancorMortyMotorController::~GazeboFrancorMortyMotorController(void)
{
    alive_ = false;
    callback_queue_thread_.join();
}

void GazeboFrancorMortyMotorController::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    // Get the joints of each motor. All 6 joints are required. The order of the joint tag list must correspond to the
    // enum class Wheel!
    const std::vector<std::string> sdfJointTags{ "motorLeftFrontJoint", "motorLeftMidJoint",
        "motorLeftRearJoint", "motorRightFrontJoint", "motorRightMidJoint", "motorRightRearJoint" };

    for (std::size_t i = 0; i < static_cast<std::size_t>(Wheel::COUNT_WHEELS); ++i)
    {
        if (sdf->HasElement(sdfJointTags[i]))
        {
            const std::string jointName(sdf->GetElement(sdfJointTags[i])->Get<std::string>());
            auto joint = model->GetJoint(jointName);

            if (joint == nullptr)
            {
                ROS_ERROR("GazeboFrancorMortyMotorController can't get a valid pointer of the joint \"%s\"."
                          " The plugin loading has been canceled.", jointName.c_str());
                return;
            }

            motor_joints_[i] = joint;
            ROS_INFO("GazeboFrancorMortyMotorController: Added joint \"%s\" as motor joint.",
                     joint->GetScopedName().c_str());
        }
        else
        {
            ROS_ERROR("GazeboFrancorMortyMotorController can't find the tag \"%s\" in the SDF file.",
                      sdfJointTags[i].c_str());
        }
    }

    // Get the position of each motor center. The tag list must correspond to enum class Wheel!
    const std::vector<std::string> sdfPosTags{ "motorLeftFrontPos", "motorLeftMidPos", "motorLeftRearPos",
        "motorRightFrontPos", "motorRightMidPos", "motorRightRearPos" };
    std::vector<gazebo::math::Vector3> posWheels(sdfPosTags.size());

    for (std::size_t i = 0; i < static_cast<std::size_t>(Wheel::COUNT_WHEELS); ++i)
    {
        if (sdf->HasElement(sdfPosTags[i]))
        {
            posWheels[i] = sdf->GetElement(sdfPosTags[i])->Get<gazebo::math::Vector3>();
        }
        else
        {
            ROS_ERROR("GazeboFrancorMortyMotorController can't find the tag \"%s\". The position of the wheel is "
                      "required. The plugin loading has been canceled.", sdfPosTags[i]);
            return;
        }
    }

    // Get the diameter of each wheel. The tag list must correspond to enum class Wheel!
    const std::vector<std::string> sdfDiameterTag{ "motorLeftFrontDiameter", "motorLeftMidDiameter",
        "motorLeftRearDiameter", "motorRightFrontDiameter", "motorRightMidDiameter", "motorRightRearDiameter" };
    std::vector<double> diameterWheels(sdfDiameterTag.size());

    for (std::size_t i = 0; i < static_cast<std::size_t>(Wheel::COUNT_WHEELS); ++i)
    {
        if (sdf->HasElement(sdfDiameterTag[i]))
        {
            diameterWheels[i] = sdf->GetElement(sdfDiameterTag[i])->Get<double>();
        }
        else
        {
            ROS_ERROR("GazeboFrancorMortyMotorController can't find the tag \"%s\". The diameter of the wheel is "
                      "required. The plugin loading has been canceled.", sdfDiameterTag[i]);
            return;
        }
    }

    // Get the parameter for the ROS part.
    std::string velocityTopic("velocity");
    std::string odometryTopic("odometry");
    std::string rosNamespace("morty");

    if (sdf->HasElement("velocityTopic"))
    {
        velocityTopic = sdf->GetElement("velocityTopic")->Get<std::string>();
        ROS_INFO("GazeboFrancorMortyMotorController has changed the velocity topic to \"%s\".",
                 velocityTopic.c_str());
    }
    else
    {
        ROS_INFO("GazeboFrancorLaserLevelingPlugin is using \"%s\" as default velocity topic.",
                 velocityTopic.c_str());
    }
    if (sdf->HasElement("odometryTopic"))
    {
        odometryTopic = sdf->GetElement("odometryTopic")->Get<std::string>();
        ROS_INFO("GazeboFrancorMortyMotorController has changed the odometry topic to \"%s\".",
                 odometryTopic.c_str());
    }
    else
    {
        ROS_INFO("GazeboFrancorLaserLevelingPlugin is using \"%s\" as default odometry topic.",
                 odometryTopic.c_str());
    }
    if (sdf->HasElement("rosNamespace"))
    {
        rosNamespace = sdf->GetElement("rosNamespace")->Get<std::string>();
        ROS_INFO("GazeboFrancorMortyMotorController has changed the ros namespace to \"%s\".",
                 rosNamespace.c_str());
    }
    else
    {
        ROS_INFO("GazeboFrancorLaserLevelingPlugin is using \"%s\" as default ros namespace.",
                 rosNamespace.c_str());
    }

    // Initialize the joint controller and the robot kinematic.
    joint_controller_ = std::make_shared<gazebo::physics::JointController>(model);
    // A copy of this PID controller is used by each motor joint.
    // TODO: find good parameter for the controller.
    gazebo::common::PID pid(1.0, 0.0, 0.0);

    for (std::size_t i = 0; i < static_cast<std::size_t>(Wheel::COUNT_WHEELS); ++i)
    {
        // Limit the joint. TODO: The limits should be configurable by SDF file.
        motor_joints_[i]->SetVelocityLimit(0, 10.0);
        motor_joints_[i]->SetParam("fmax", 0, 30.0);

        // Add joint as wheel to the kinematic.
        kinematic_.addWheel(motor_joints_[i]->GetScopedName(), diameterWheels[i], posWheels[i]);

        // Add the joint to joint controller and configure it.
        joint_controller_->AddJoint(motor_joints_[i]);
        joint_controller_->SetVelocityPID(motor_joints_[i]->GetScopedName(), pid);
        joint_controller_->SetVelocityTarget(motor_joints_[i]->GetScopedName(), 0.0);
    }

    // Initialize ROS part.
    ros_nodehandle_ = std::make_shared<ros::NodeHandle>();
    sub_velocity_ = ros_nodehandle_->subscribe(velocityTopic,
                                               1,
                                               &GazeboFrancorMortyMotorController::receiveTwistMsg,
                                               this);

    alive_ = true;
    callback_queue_thread_ = std::thread(&GazeboFrancorMortyMotorController::rosQueueThread, this);

    // Initialize model plugin.
    model_ = model;
    update_connection_ =
     gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboFrancorMortyMotorController::update, this));
}

void GazeboFrancorMortyMotorController::Reset(void)
{

}

void GazeboFrancorMortyMotorController::update(void)
{
    // The update method of the joint controller has to be called in each iteration.
    std::lock_guard<std::mutex> lock(mutex_ros_msgs_);
    joint_controller_->Update();
}

void GazeboFrancorMortyMotorController::rosQueueThread(void)
{
    constexpr double timeout = 0.01;

    while (alive_ && ros_nodehandle_->ok())
        ros_msg_queue_.callAvailable(ros::WallDuration(timeout));
}

void GazeboFrancorMortyMotorController::receiveTwistMsg(const geometry_msgs::Twist& msg)
{
    // Lock mutex to manipulate the target velocities of the wheels.
    std::lock_guard<std::mutex> lock(mutex_ros_msgs_);
    kinematic_.calculate(msg.linear.x, msg.angular.z); // Respect only linear speed in x direction and yaw rotating.

    for (std::size_t i = 0; i < static_cast<std::size_t>(Wheel::COUNT_WHEELS); ++i)
        joint_controller_->SetVelocityTarget(motor_joints_[i]->GetScopedName(), kinematic_.getWheelRotatingSpeed(i));
}

} // end namespace nrbdl
