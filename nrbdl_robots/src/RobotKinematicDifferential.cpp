#include "RobotKinematicDifferential.h"

namespace nrbdl {

void RobotKinematicDifferential::addWheel(const double diameter, const gazebo::math::Vector3& position)
{
    wheel_rotation_speeds_.push_back(0.0);
    wheels_.push_back(Wheel(diameter, position));
}

void RobotKinematicDifferential::calculate (const double linearVelocity, const gazebo::math::Angle rotationSpeed)
{

}

double RobotKinematicDifferential::Wheel::rotationSpeed (const double linearSpeed) const
{
    // Return rad/sec.
    return linearSpeed / (M_PI * diameter_);
}

} // end namespace nrbdl
