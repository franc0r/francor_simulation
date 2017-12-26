#include "nrbdl_robots/gazebo-francor-laser-leveling-plugin.h"

#include <ros/ros.h>

namespace nrbdl {

void GazeboFrancorLaserLevelingPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    // Get the roll and pitch joint from the SDF model file. The joints are checked. If there are no joint names
    // provided or a joint doesn't exist, the plugin loading will be cancelled.
    if (sdf->HasElement("jointServoRoll"))
    {
        const std::string jointName(sdf->GetElement("jointServoRoll")->Get<std::string>());
        joint_roll_ = model->GetJoint(jointName);

        if (joint_roll_ == nullptr)
        {
            ROS_ERROR("Provided joint \"%s\" dosen't exist. Loading of GazeboFrancorLaserLeveling plugin has been "
                      "cancelled.", jointName.c_str());
            return;
        }
    }
    else
    {
        ROS_ERROR("GazeboFrancorLaserLeveling is missing <jointServoRoll>. Loading of plugin has been cancelled.");
        return;
    }
    if (sdf->HasElement("jointServoPitch"))
    {
        const std::string jointName(sdf->GetElement("jointServoPitch")->Get<std::string>());
        joint_pitch_ = model->GetJoint(jointName);

        if (joint_pitch_ == nullptr)
        {
            ROS_ERROR("Provided joint \"%s\" dosen't exist. Loading of GazeboFrancorLaserLeveling plugin has been "
                      "cancelled.", jointName.c_str());
            return;
        }
    }
    else
    {
        ROS_ERROR("GazeboFrancorLaserLeveling is missing <jointServoPitch>. Loading of plugin has been cancelled.");
        return;
    }

    // Initialize the joint controller. The controller handles both joint together.
    joint_controller_ = std::make_shared<gazebo::physics::JointController>(model);
    joint_controller_->AddJoint(joint_roll_);
    joint_controller_->SetPositionPID(joint_roll_->GetName(), gazebo::common::PID(20.0, 0.1, 0.0));
    joint_controller_->SetPositionTarget(joint_roll_->GetName(), 0.0);

    joint_controller_->AddJoint(joint_pitch_);
    joint_controller_->SetPositionPID(joint_pitch_->GetName(), gazebo::common::PID(20.0, 0.1, 0.0));
    joint_controller_->SetPositionTarget(joint_pitch_->GetName(), 0.0);

    // Connect Gazebo event with callback.
    update_connection_
        = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboFrancorLaserLevelingPlugin::levelLaser, this));

    ROS_INFO("GazeboFrancorLaserLeveling has been started.");
}

void GazeboFrancorLaserLevelingPlugin::levelLaser(void)
{
    // Updates the controller (trigger internal calculations).
    joint_controller_->Update();
}

} // end namespace nrbdl
