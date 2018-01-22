#ifndef ___GAZEBO_FRANCOR_MORTY_MOTOR_CONTROLLER_H___
#define ___GAZEBO_FRANCOR_MORTY_MOTOR_CONTROLLER_H___

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <thread>
#include <array>

#include "RobotKinematicDifferential.h"

namespace francor {

class GazeboFrancorMortyMotorController : public gazebo::ModelPlugin
{
public:
    GazeboFrancorMortyMotorController(void) = default;
    GazeboFrancorMortyMotorController(const GazeboFrancorMortyMotorController&) = delete;
    GazeboFrancorMortyMotorController(GazeboFrancorMortyMotorController&&) = default;
    virtual ~GazeboFrancorMortyMotorController(void);

    GazeboFrancorMortyMotorController& operator=(const GazeboFrancorMortyMotorController&) = delete;
    GazeboFrancorMortyMotorController& operator=(GazeboFrancorMortyMotorController&&) = default;

    virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
    virtual void Reset(void);

private:
    void update(void);
    void rosQueueThread(void);
    void receiveTwistMsg(const geometry_msgs::Twist& msg);

    enum class Wheel : std::size_t {
        LEFT_FRONT = 0,
        LEFT_MID,
        LEFT_REAR,
        RIGHT_FRONT,
        RIGHT_MID,
        RIGHT_REAR,
        COUNT_WHEELS
    };

    // Gazebo Model Members
    gazebo::physics::ModelPtr model_;
    gazebo::event::ConnectionPtr update_connection_;
    std::shared_ptr<gazebo::physics::JointController> joint_controller_;
    std::array<gazebo::physics::JointPtr, static_cast<std::size_t>(Wheel::COUNT_WHEELS)> motor_joints_;

    // ROS objects
    ros::CallbackQueue ros_msg_queue_;
    std::thread callback_queue_thread_;
    ros::Publisher pub_odometry_;
    ros::Subscriber sub_velocity_;
    std::shared_ptr<ros::NodeHandle> ros_nodehandle_;
    std::atomic<bool> alive_{false};
    std::mutex mutex_ros_msgs_;

    // Members
    RobotKinematicDifferential kinematic_;
};

} // end namespace francor

#endif 
