// Hold the MiR base still while parked. Arm joints use kinematic SetPosition, which
// nudges the chassis every physics step; pinning the base world pose (not only velocity)
// while cmd_vel is idle removes that chatter.

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

namespace mir_ur_warehouse
{

class ParkBase : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    // Prefer base_footprint: Gazebo folds base_link into it via the fixed joint.
    const std::string base_name =
      sdf->HasElement("base_link") ? sdf->Get<std::string>("base_link") : "base_footprint";
    const std::string left_name =
      sdf->HasElement("left_wheel_joint") ? sdf->Get<std::string>("left_wheel_joint")
                                         : "left_wheel_joint";
    const std::string right_name =
      sdf->HasElement("right_wheel_joint") ? sdf->Get<std::string>("right_wheel_joint")
                                          : "right_wheel_joint";
    cmd_eps_ = sdf->HasElement("cmd_eps") ? sdf->Get<double>("cmd_eps") : 1e-3;
    release_s_ = sdf->HasElement("release_s") ? sdf->Get<double>("release_s") : 0.05;

    base_ = FindLink(base_name, "base_footprint");
    if (!base_) {
      base_ = FindLink(base_name, "base_link");
    }
    left_ = FindJoint(left_name, "left_wheel_joint");
    right_ = FindJoint(right_name, "right_wheel_joint");
    if (!base_) {
      gzerr << "[park_base] no base link on model " << model_->GetName() << " (wanted "
            << base_name << "). links:";
      for (const auto & link : model_->GetLinks()) {
        gzerr << " " << link->GetName();
      }
      gzerr << "\n";
      return;
    }
    gzmsg << "[park_base] " << model_->GetName() << " locking " << base_->GetName() << "\n";

    node_ = gazebo_ros::Node::Get(sdf);
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "diff_cont/cmd_vel_unstamped", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Twist & msg) {
        const double a = std::max(std::abs(msg.linear.x), std::abs(msg.angular.z));
        std::lock_guard<std::mutex> lock(mutex_);
        last_cmd_ = a;
        last_cmd_time_ = model_->GetWorld()->SimTime();
      });

    // After physics + ros2_control, snap the base back if parked.
    update_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&ParkBase::OnUpdate, this));
  }

private:
  gazebo::physics::LinkPtr FindLink(const std::string & want, const std::string & suffix) const
  {
    if (auto link = model_->GetLink(want)) {
      return link;
    }
    for (const auto & link : model_->GetLinks()) {
      const std::string & n = link->GetName();
      if (n == want) {
        return link;
      }
      if (n.size() >= suffix.size() &&
          n.compare(n.size() - suffix.size(), suffix.size(), suffix) == 0 &&
          n.find("inertia") == std::string::npos)
      {
        return link;
      }
    }
    return nullptr;
  }

  gazebo::physics::JointPtr FindJoint(const std::string & want, const std::string & suffix) const
  {
    if (auto joint = model_->GetJoint(want)) {
      return joint;
    }
    for (const auto & joint : model_->GetJoints()) {
      const std::string & n = joint->GetName();
      if (n == want) {
        return joint;
      }
      if (n.size() >= suffix.size() &&
          n.compare(n.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        return joint;
      }
    }
    return nullptr;
  }

  bool IsParked()
  {
    double cmd;
    gazebo::common::Time t_cmd;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = last_cmd_;
      t_cmd = last_cmd_time_;
    }
    const double age = (model_->GetWorld()->SimTime() - t_cmd).Double();
    const bool driving = cmd > cmd_eps_ && age >= 0.0 && age < release_s_ + 0.25;
    return !driving;
  }

  void OnUpdate()
  {
    if (!base_) {
      return;
    }
    if (!IsParked()) {
      have_pose_ = false;
      return;
    }

    if (!have_pose_) {
      pose_ = base_->WorldPose();
      have_pose_ = true;
    }

    // Pin pose: SetPosition on the arm still displaces the base slightly each step.
    base_->SetWorldPose(pose_, true, true);
    base_->SetLinearVel(ignition::math::Vector3d::Zero);
    base_->SetAngularVel(ignition::math::Vector3d::Zero);
    if (left_) {
      left_->SetVelocity(0, 0.0);
    }
    if (right_) {
      right_->SetVelocity(0, 0.0);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_;
  gazebo::physics::JointPtr left_;
  gazebo::physics::JointPtr right_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  gazebo::event::ConnectionPtr update_;
  std::mutex mutex_;
  double last_cmd_{0.0};
  gazebo::common::Time last_cmd_time_;
  double cmd_eps_{1e-3};
  double release_s_{0.05};
  bool have_pose_{false};
  ignition::math::Pose3d pose_;
};

GZ_REGISTER_MODEL_PLUGIN(ParkBase)

}  // namespace mir_ur_warehouse
