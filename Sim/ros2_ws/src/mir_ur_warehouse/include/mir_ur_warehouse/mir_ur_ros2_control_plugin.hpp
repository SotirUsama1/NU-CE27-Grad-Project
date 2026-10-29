#ifndef MIR_UR_WAREHOUSE__MIR_UR_ROS2_CONTROL_PLUGIN_HPP_
#define MIR_UR_WAREHOUSE__MIR_UR_ROS2_CONTROL_PLUGIN_HPP_

#include <memory>

#include <gazebo/common/Plugin.hh>

namespace mir_ur_warehouse
{

class GazeboRosControlPrivate;

// gazebo_ros2_control writes every robot's parameters into one process-wide rcl context.
// A second robot replaces the first robot's parameters, so this plugin keeps each
// controller manager's arguments on that manager alone.
class MirUrRos2ControlPlugin : public gazebo::ModelPlugin
{
public:
  MirUrRos2ControlPlugin();
  ~MirUrRos2ControlPlugin() override;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void Reset() override;

private:
  std::unique_ptr<GazeboRosControlPrivate> impl_;
};

}  // namespace mir_ur_warehouse

#endif
