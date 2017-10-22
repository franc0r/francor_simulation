#include "gazebo_morty_motor_controller.h"

namespace nrbdl_robots {

GazeboMortyMotorController::~GazeboMortyMotorController(void)
{
    running_ = false;
    callbackThread_.join();
}

enum Wheel {
    LEFT_FRONT = 0,
    LEFT_MID,
    LEFT_REAR,
    RIGHT_FRONT,
    RIGHT_MID,
    RIGHT_REAR
};

void GazeboMortyMotorController::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    model_ = model;

    std::string robotNamespace = "";
    std::string twistTopic = "cmd_vel";

    rosNodeHandle_ = std::make_shared<ros::NodeHandle>(robotNamespace);
    rosNodeHandle_->setCallbackQueue(&queue_);
    subTwist_ = rosNodeHandle_->subscribe(twistTopic, 1, &GazeboMortyMotorController::callbackTwistCommand, this);

    callbackThread_ = std::thread(&GazeboMortyMotorController::queueThread, this);

    auto joints = model_->GetJoints();

    for (auto & joint : joints)
        std::cout << "joint: " << joint->GetName() << std::endl;

    joints_.resize(6);
    joints_[LEFT_FRONT]  = model_->GetJoint("left_bogie_wheel_front");
    joints_[LEFT_MID]    = model_->GetJoint("left_bogie_wheel_middle");
    joints_[LEFT_REAR]   = model_->GetJoint("left_bogie_wheel_rear");
    joints_[RIGHT_FRONT] = model_->GetJoint("right_bogie_wheel_front");
    joints_[RIGHT_MID]   = model_->GetJoint("right_bogie_wheel_middle");
    joints_[RIGHT_REAR]  = model_->GetJoint("right_bogie_wheel_rear");

    updateConnection_ =
        gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboMortyMotorController::calculateWheelRotation, this));
}

void GazeboMortyMotorController::callbackTwistCommand(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    std::lock_guard<std::mutex> lock(syncMutex_);
    yaw_ = cmd_msg->angular.z;
    velocity_ = cmd_msg->linear.x;

    if (yaw_ > M_PI * 0.5)
        yaw_ = M_PI * 0.5;
    else if (yaw_ < -M_PI * 0.5)
        yaw_ = -M_PI * 0.5;
}

void GazeboMortyMotorController::calculateWheelRotation(void)
{
    std::lock_guard<std::mutex> lock(syncMutex_);
    const double velRot = 5.0 * std::sin(yaw_);
    double velLeft = velocity_ - velRot;
    double velRight = velocity_ + velRot;
    velLeft = velLeft / (wheelRadius_ * 4.0 * M_PI);
    velRight = velRight / (wheelRadius_ * 4.0 * M_PI);
    std::cout << "vel left = " << velLeft << std::endl;
    std::cout << "vel right = " << velRight << std::endl;

    joints_[LEFT_FRONT]->SetParam("fmax", 0, 200.0);
    joints_[LEFT_FRONT]->SetParam("vel", 0, velLeft);
    joints_[LEFT_MID]->SetParam("fmax", 0, 200.0);
    joints_[LEFT_MID]->SetParam("vel", 0, velLeft);
    joints_[LEFT_REAR]->SetParam("fmax", 0, 200.0);
    joints_[LEFT_REAR]->SetParam("vel", 0, velLeft);
    joints_[RIGHT_FRONT]->SetParam("fmax", 0, 200.0);
    joints_[RIGHT_FRONT]->SetParam("vel", 0, velRight);
    joints_[RIGHT_MID]->SetParam("fmax", 0, 200.0);
    joints_[RIGHT_MID]->SetParam("vel", 0, velRight);
    joints_[RIGHT_REAR]->SetParam("fmax", 0, 200.0);
    joints_[RIGHT_REAR]->SetParam("vel", 0, velRight);
}

void GazeboMortyMotorController::queueThread(void)
{
    constexpr double timeout = 0.01;

    while (running_ && rosNodeHandle_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMortyMotorController)

} // end namespace nrbdl_robots
