#include "nrbdl_robots/gazebo-francor-laser-leveling-plugin.h"

#include <ros/ros.h>

namespace nrbdl {

GZ_REGISTER_MODEL_PLUGIN(GazeboFrancorLaserLevelingPlugin)

void GazeboFrancorLaserLevelingPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    ROS_INFO("GazeboFrancorLaserLeveling plugin is being loaded.");

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

    // Initialize the joint controllers.
    controller_roll_.setLimits(-0.01, 0.01);  // Set the limit to 10 Nm.
    controller_roll_.setTarget(0.0);          // Set the target value to 0 rad.
    controller_roll_.setOutputToValue(0.0);   // Set the output to the target value.

    controller_pitch_.setLimits(-0.01, 0.01); // Set the limit to 10 Nm.
    controller_pitch_.setTarget(0.0);         // Set the target value to 0 rad.
    controller_pitch_.setOutputToValue(0.0);  // Set the output to the target value.

    // Connect Gazebo event with callback.
    update_connection_
        = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboFrancorLaserLevelingPlugin::levelLaser, this));

    ROS_INFO("GazeboFrancorLaserLeveling has been started.");
}

void GazeboFrancorLaserLevelingPlugin::levelLaser(void)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    // Get current pose of the model and use the orientation as target values.
    gazebo::math::Quaternion rotModel(joint_roll_->GetParent()->GetWorldPose().rot);
    const gazebo::math::Angle targetRoll(-rotModel.GetRoll());
    const gazebo::math::Angle targetPitch(-rotModel.GetPitch());
    std::cout << "model roll angle = " << rotModel.GetRoll() << std::endl;
    std::cout << "model pitch angle = " << rotModel.GetPitch() << std::endl;
    std::cout << "target roll angle = " << targetRoll.Radian() << std::endl;
    std::cout << "target pitch angle = " << targetPitch.Radian() << std::endl;
    controller_roll_.setTarget(targetRoll.Radian());
    controller_pitch_.setTarget(targetPitch.Radian());


    // Get the current angles of the servos as input value for the controller.
    //const unsigned int idxAxisRoll = (joint_roll_->GetLocalAxis(0).x ? 0 : joint_roll_->GetLocalAxis(0).y ? 1 : 2);
    //const unsigned int idxAxisPitch = (joint_pitch_->GetLocalAxis(0).x ? 0 : joint_pitch_->GetLocalAxis(0).y ? 1 : 2);
    const gazebo::math::Angle rollJointAngle(joint_roll_->GetAngle(0));
    const gazebo::math::Angle pitchJointAngle(joint_pitch_->GetAngle(0));

    controller_roll_.process(rollJointAngle.Radian());
    controller_pitch_.process(pitchJointAngle.Radian());


    // Set the output of the controllers to the model joints (torque in Nm).
    joint_roll_->SetVelocityLimit(0, 0.1);
    joint_roll_->SetForce(0, controller_roll_.output());
    joint_pitch_->SetVelocityLimit(0, 0.1);
    joint_pitch_->SetForce(0, controller_pitch_.output());
//    joint_roll_->SetForce(0, 0.01);
//    joint_pitch_->SetForce(0, 0.01);
    std::cout << "roll controller output = " << controller_roll_.output() << std::endl;
    std::cout << "pitch controller output = " << controller_pitch_.output() << std::endl;
    std::cout << "force of joint " << joint_roll_->GetScopedName() << " is " << joint_roll_->GetForce(0) << std::endl;
}

} // end namespace nrbdl
