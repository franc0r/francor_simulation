#ifndef ___GAZEBO_FRANCOR_LASER_LEVELING_PLUGIN_H___
#define ___GAZEBO_FRANCOR_LASER_LEVELING_PLUGIN_H___

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace nrbdl {

class GazeboFrancorLaserLevelingPlugin : public gazebo::ModelPlugin
{
public:
    GazeboFrancorLaserLevelingPlugin(void) = default;
    virtual ~GazeboFrancorLaserLevelingPlugin(void) = default;

    virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

private:
    void levelLaser(void);

    gazebo::physics::ModelPtr model_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::physics::JointPtr joint_roll_;
    gazebo::physics::JointPtr joint_pitch_;
    std::shared_ptr<gazebo::physics::JointController> joint_controller_;
};


} // end namespace nrbdl

#endif
