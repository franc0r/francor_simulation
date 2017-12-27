#include "gazebo-francor-morty-motor-controller-plugin.h"

namespace nrbdl {

GazeboFrancorMortyMotorController::~GazeboFrancorMortyMotorController(void)
{

}

void GazeboFrancorMortyMotorController::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    // Get the joints of each motor. All 6 joints are required.
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

    // Initialize the joint controller.
    joint_controller_ = std::make_shared<gazebo::physics::JointController>(model);
    // This PID controller is used by each motor joint.
    gazebo::common::PID pid(1.0, 0.0, 0.0);

    for (auto& joint : motor_joints_)
    {
        // Limit the joint.
        joint->SetVelocityLimit(0, 10.0);
        joint->SetParam("fmax", 0, 30.0);

        // Add the joint to joint controller and configure it.
        joint_controller_->AddJoint(joint);
        joint_controller_->SetVelocityPID(joint->GetScopedName(), pid);
        joint_controller_->SetVelocityTarget(joint->GetScopedName(), 0.0);
    }

    // Initialize model plugin.
    model_ = model;
    update_connection_ =
     gazebo::event::Events::ConnectWorldUpdateBegin(std::bing(&GazeboFrancorMortyMotorController::update, this));
}

void GazeboFrancorMortyMotorController::Reset(void)
{

}

void GazeboFrancorMortyMotorController::update(void)
{
    // The update method of the joint controller has to be called in each iteration.
    joint_controller_->Update();
}

void GazeboFrancorMortyMotorController::rosQueueThread(void)
{

}

} // end namespace nrbdl
