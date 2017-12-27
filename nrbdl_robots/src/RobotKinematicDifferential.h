#ifndef ___FRANCOR_ROBOT_KINEMATIC_DIFFERENTIAL_H___
#define ___FRANCOR_ROBOT_KINEMATIC_DIFFERENTIAL_H___

#include <vector>

#include <gazebo/math/gzmath.hh>

namespace nrbdl {

class RobotKinematicDifferential
{
public:
    RobotKinematicDifferential(void) = default;
    ~RobotKinematicDifferential(void) = default;

    /**
     * \brief The method adds a wheel to this kinematic class. The coordinates of the wheel must obtain to the
     *        rotating center of the robot. The diameter unit is meter.
     * \param diameter The diameter of the wheel in meter.
     * \param position The position of the wheel (center of the wheel pointed to the ground). The origin is the
     *                 rotating center of the robot.
     */
    void addWheel(const double diameter, const gazebo::math::Vector3& position);
    inline const std::vector<double>& getWheelRotatingSpeeds(void) const { return wheel_rotation_speeds_; }
    /**
     * \brief Calculates the wheel rotation speeds according the provided linear and rotation velocity. The results
     *        can be accessed by getter functions.
     * \param linearVelocity The velocity in x-direction.
     * \param rotationSpeed The rotation speed (rad/sec).
     */
    void calculate(const double linearVelocity, const gazebo::math::Angle rotationSpeed);

private:

    class Wheel
    {
    public:
        Wheel(void) = default;
        Wheel(const double diameter, const gazebo::math::Vector3& position) : diameter_(diameter), position_(position) { }
        Wheel(const Wheel&) = default;
        Wheel(Wheel&&) = default;

        inline double diameter(void) const { return diameter_; }
        inline gazebo::math::Vector3 position(void) const { return position_; }
        double rotationSpeed(const double linearSpeed) const;

        double distanceTo(const Wheel& wheel) const { return (position_ - wheel.position_).GetLength(); }

    private:
        double diameter_ = -1.0;
        gazebo::math::Vector3 position_;
    };

    std::vector<double> wheel_rotation_speeds_;
    std::vector<Wheel> wheels_;
};

} // end namespace nrbdl

#endif
