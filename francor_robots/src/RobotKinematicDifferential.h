#ifndef ___FRANCOR_ROBOT_KINEMATIC_DIFFERENTIAL_H___
#define ___FRANCOR_ROBOT_KINEMATIC_DIFFERENTIAL_H___

#include <vector>

#include <gazebo/math/gzmath.hh>

namespace francor {

class RobotKinematicDifferential
{
public:
    RobotKinematicDifferential(void) = default;
    ~RobotKinematicDifferential(void) = default;

    /**
     * \brief The method adds a wheel to this kinematic. The coordinates of the wheel must obtain to the
     *        rotating center of the robot. The diameter unit is meter.
     * \param name The name of the new wheel. For example "left_front".
     * \param diameter The diameter of the wheel in meter.
     * \param position The position of the wheel (center of the wheel pointed to the ground). The origin is the
     *                 rotating center of the robot.
     * \return The index of the added wheel. It can be used to modify a wheel.
     */
    int addWheel(const std::string& name, const double diameter, const gazebo::math::Vector3& position);
    /**
     * \brief Gets the wheel's index by name.
     * \param wheel The name of the wheel.
     * \return The index of the wheel. Returns -1 if the name is not contained in this kinematic.
     */
    int getWheelIndex(const std::string& wheel) const;
    /**
     * \brief Modifies a wheel's position.
     * \param wheel The index of the wheel that will be modified.
     * \param position The new position of the wheel.
     */
    void modifyWheelPosition(const std::size_t wheel, const gazebo::math::Vector3& position);
    /**
     * \brief Modifies a wheel's position.
     * \param wheel The name of the wheel that will be modified.
     * \param position The new postion of the wheel.
     */
    void modifyWheelPosition(const std::string& wheel, const gazebo::math::Vector3& position);
    /**
     * \brief Gets the last calculated rotation speed of a wheel.
     * \param wheel The index of the wheel.
     * \return The last calculated rotation speed of the selected wheel.
     */
    inline double getWheelRotatingSpeed(const std::size_t wheel) const { return wheel_rotation_speeds_[wheel]; }
    /**
     * \brief Gets the last calculated rotation speed of a wheel.
     * \param wheel The name of the wheel.
     * \return The last calculated rotation speed of the selected wheel.
     */
    double getWheelRotatingSpeed(const std::string& wheel) const;
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
        Wheel(const std::string& name, const double diameter, const gazebo::math::Vector3& position)
          : name_(name), diameter_(diameter), position_(position), distance_to_center_(position.GetLength()) { }
        Wheel(const Wheel&) = default;
        Wheel(Wheel&&) = default;

        inline double diameter(void) const { return diameter_; }
        inline const gazebo::math::Vector3& position(void) const { return position_; }
        inline const std::string& name(void) const { return name_; }

        inline void setPosition(const gazebo::math::Vector3& position) { position_ = position; }

        double rotationSpeed(const double linearSpeed) const;
        inline double distanceTo(const Wheel& wheel) const { return (position_ - wheel.position_).GetLength(); }
        inline double distanceToCenter(void) const { return distance_to_center_; }

    private:
        std::string name_;
        double diameter_ = -1.0;
        gazebo::math::Vector3 position_;
        double distance_to_center_ = 0.0;
    };


    std::vector<double> wheel_rotation_speeds_;
    std::vector<Wheel> wheels_;
};

} // end namespace francor

#endif
