#include "RobotKinematicDifferential.h"

#include <cmath>

#include <ros/ros.h>

namespace francor {

int RobotKinematicDifferential::addWheel(const std::string& name,
                                         const double diameter,
                                         const gazebo::math::Vector3& position)
{
    wheel_rotation_speeds_.push_back(0.0);
    wheels_.push_back(Wheel(name, diameter, position));

    return wheels_.size() - 1; // The wheel is the last element...
}

int RobotKinematicDifferential::getWheelIndex(const std::string& wheel) const
{
    for (std::size_t i = 0; i < wheels_.size(); ++i)
        if (wheels_[i].name() == wheel)
            return i;

    return -1;
}

void RobotKinematicDifferential::modifyWheelPosition(const std::size_t wheel, const gazebo::math::Vector3& position)
{
    wheels_[wheel].setPosition(position);
    wheel_rotation_speeds_[wheel] = 0.0; // The last calculated speed is not longer valid. I dont't know if zero is
                                         // the right choise!!!
}

void RobotKinematicDifferential::modifyWheelPosition(const std::string& wheel, const gazebo::math::Vector3& position)
{
    const int index = this->getWheelIndex(wheel);

    if (index < 0)
    {
        ROS_ERROR("RobotKinematicDifferential class can't modify the position of wheel \"s\". The provided wheel name"
                  " is not valid.", wheel.c_str());
        return;
    }

    this->modifyWheelPosition (index, position);
}

double RobotKinematicDifferential::getWheelRotatingSpeed(const std::string& wheel) const
{
    const int index = this->getWheelIndex(wheel);

    if (index < 0)
    {
        ROS_ERROR("RobotKinematicDifferential can't return the rotation speed of wheel \"s\". The provided wheel name"
                  " is not valid.", wheel.c_str());
        return std::nan("0");
    }

    return wheel_rotation_speeds_[index];
}

void RobotKinematicDifferential::calculate (const double linearVelocity, const gazebo::math::Angle rotationSpeed)
{
    std::cout << "RobotKinematicDifferential calculates rotating speeds of the wheels." << std::endl;
    for (std::size_t i = 0; i < wheels_.size(); ++i)
    {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Each wheel rotates on a circle around the robot center. The radius of this cricle is the distance to the
        // robot center (0): r = length(p_wheel - 0). The wheel's velocity (v) on the circle depends on the rotating
        // speed (dyaw / dt) of the robot: dyaw / dt = v / r.
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        const double v_circle = rotationSpeed.Radian() * wheels_[i].distanceToCenter();

        // Resulting velocity is the linear + neccesary velocity for rotation.
        // For wheels on the left side flip the rotation velocity.
        const double d_v = (wheels_[i].position().y < 0.0 ? -1.0 : 1.0);
        const double v_wheel = v_circle * d_v + linearVelocity;

        wheel_rotation_speeds_[i] = wheels_[i].rotationSpeed(v_wheel);

        std::cout << wheels_[i].name() << ":" << std::endl;
        std::cout << "rotating speed = " << rotationSpeed.Radian() << std::endl;
        std::cout << "v_circle = " << v_circle << std::endl;
        std::cout << "v_wheel  = " << v_wheel << std::endl;
    }
}

double RobotKinematicDifferential::Wheel::rotationSpeed (const double linearSpeed) const
{
    // Return rad/sec.
    const double speed = (2.0 * M_PI * linearSpeed) / (M_PI * diameter_);
    // Respects if a wheel is on the left or right side of the robot.
    return (position_.y < 0.0 ? speed : speed * -1.0);
}

} // end namespace francor
