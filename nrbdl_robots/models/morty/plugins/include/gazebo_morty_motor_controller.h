#ifndef ___GAZEBO_MORTY_MOTOR_CONTROLLER_H___
#define ___GAZEBO_MORTY_MOTOR_CONTROLLER_H___

#include <thread>
#include <mutex>
#include <atomic>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace nrbdl_robots {

class GazeboMortyMotorController : public gazebo::ModelPlugin
{
public:
    GazeboMortyMotorController(void) = default;
    virtual ~GazeboMortyMotorController(void);

    virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

private:
    void callbackTwistCommand(const geometry_msgs::Twist::ConstPtr& cmd_msg);
    void calculateWheelRotation(void);
    void queueThread(void);

    // Gazebo callback.
    gazebo::physics::ModelPtr model_;
    gazebo::event::ConnectionPtr updateConnection_;
    std::vector<gazebo::physics::JointPtr> joints_;

    // Custom Callback Queue
    std::shared_ptr<ros::NodeHandle> rosNodeHandle_;
    ros::Subscriber subTwist_;
    ros::CallbackQueue queue_;
    std::thread callbackThread_;
    std::mutex syncMutex_;

    std::atomic<bool> running_{true};
    double wheelRadius_ = 0.11;
    double velocity_;
    double yaw_;
};

} // end namespace nrbdl_robots

#endif
