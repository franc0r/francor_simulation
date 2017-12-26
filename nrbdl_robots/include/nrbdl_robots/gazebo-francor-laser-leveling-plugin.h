#ifndef ___GAZEBO_FRANCOR_LASER_LEVELING_PLUGIN_H___
#define ___GAZEBO_FRANCOR_LASER_LEVELING_PLUGIN_H___

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include "PidController.h"

namespace nrbdl {

class GazeboFrancorLaserLevelingPlugin : public gazebo::ModelPlugin
{
public:
    GazeboFrancorLaserLevelingPlugin(void) = default;
    virtual ~GazeboFrancorLaserLevelingPlugin(void) = default;

    virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

private:
    void levelLaser(void);

    // Gazebo Model Members.
    gazebo::physics::ModelPtr model_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::physics::JointPtr joint_roll_;
    gazebo::physics::JointPtr joint_pitch_;

    // Joint controllers.
    PidController controller_roll_{0.00001, 0.000001, 0.0};
    PidController controller_pitch_{0.00001, 0.000001, 0.0};
};


} // end namespace nrbdl

#endif
