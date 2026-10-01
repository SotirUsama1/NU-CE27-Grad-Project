// Drives a warehouse robot and its load handling from ROS 2 (topics under the model's <ros><namespace>).
//
// Base, both roles (differential drive: only linear.x and angular.z are used):
//   cmd_vel  geometry_msgs/Twist       odom  nav_msgs/Odometry (frame <ns>/odom, starts at the spawn pose)
//   pose     geometry_msgs/PoseStamped in the world frame "map"
//   TF: map -> <ns>/odom -> <ns>/base_link -> <ns>/laser
// Load handling:
//   handler/command  std_msgs/String, one command per message
//   handler/state    std_msgs/String, JSON: busy flag, carried boxes, result of the last command
//
// Picker (<role>picker</role>): a lifting tray with two-stage telescopic side arms, plus storage slots.
//   lift <z>                 move the tray top to height z [m]
//   pick <left|right>        pull the box beside the tray (bottom within 10 cm of the tray top) onto the tray
//   store [n]                move the tray box into storage slot n (default: first free slot)
//   retrieve <n>             move the box in slot n onto the tray
//   place <left|right> [y]   push the tray box out until its centre is y m from the robot centre line
//                            (default: onto a shelf when the robot drives in the middle of an aisle);
//                            the box is set down on whatever is at the height of the last lift command
// Transport (<role>transport</role>): a powered roller deck.
//   unload <front|back>      run the rollers until every box has left the deck
//   Boxes that come to rest on the deck are fixed to it automatically.
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ignition/math/Pose3.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo
{
namespace
{
using ignition::math::Pose3d;
using ignition::math::Quaterniond;
using ignition::math::Vector3d;

// Boxes currently handled by any robot in this server, so two robots never grab the same box.
std::mutex g_heldMutex;
std::set<const physics::Link *> g_held;

bool Claim(const physics::LinkPtr &link)
{
  std::lock_guard<std::mutex> lock(g_heldMutex);
  return g_held.insert(link.get()).second;
}

void Unclaim(const physics::LinkPtr &link)
{
  std::lock_guard<std::mutex> lock(g_heldMutex);
  g_held.erase(link.get());
}

bool IsHeld(const physics::LinkPtr &link)
{
  std::lock_guard<std::mutex> lock(g_heldMutex);
  return g_held.count(link.get()) > 0;
}

Vector3d BoxSize(const physics::LinkPtr &link)
{
  for (const auto &collision : link->GetCollisions()) {
    auto box = boost::dynamic_pointer_cast<physics::BoxShape>(collision->GetShape());
    if (box) {
      return box->Size();
    }
  }
  return Vector3d::Zero;
}

geometry_msgs::msg::Transform ToTransform(const Pose3d &p)
{
  geometry_msgs::msg::Transform t;
  t.translation.x = p.Pos().X();
  t.translation.y = p.Pos().Y();
  t.translation.z = p.Pos().Z();
  t.rotation.x = p.Rot().X();
  t.rotation.y = p.Rot().Y();
  t.rotation.z = p.Rot().Z();
  t.rotation.w = p.Rot().W();
  return t;
}

geometry_msgs::msg::Pose ToPose(const Pose3d &p)
{
  geometry_msgs::msg::Pose m;
  m.position.x = p.Pos().X();
  m.position.y = p.Pos().Y();
  m.position.z = p.Pos().Z();
  m.orientation.x = p.Rot().X();
  m.orientation.y = p.Rot().Y();
  m.orientation.z = p.Rot().Z();
  m.orientation.w = p.Rot().W();
  return m;
}

std::string JsonName(const physics::LinkPtr &link)
{
  return link ? "\"" + link->GetModel()->GetName() + "\"" : "null";
}

double Approach(double value, double target, double step)
{
  return value + std::clamp(target - value, -step, step);
}
}  // namespace

class CustomRobotPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    base_ = model->GetLink(sdf->Get<std::string>("base_link", "base_link").first);
    if (!base_) {
      gzerr << "[custom_robot] " << model->GetName() << ": base link not found\n";
      return;
    }
    role_ = sdf->Get<std::string>("role", "transport").first;
    maxSpeed_ = sdf->Get<double>("max_speed", 1.0).first;
    maxTurn_ = sdf->Get<double>("max_turn_rate", 1.2).first;
    accel_ = sdf->Get<double>("acceleration", 0.8).first;
    loadPrefix_ = sdf->Get<std::string>("load_prefix", "box_").first;
    laserPose_ = sdf->Get<Pose3d>("laser_pose", Pose3d()).first;

    if (role_ == "picker") {
      lift_ = model->GetJoint(sdf->Get<std::string>("lift_joint", "lift_joint").first);
      reach_ = model->GetJoint(sdf->Get<std::string>("reach_joint", "reach_joint").first);
      reach2_ = model->GetJoint(sdf->Get<std::string>("reach2_joint", "reach2_joint").first);
      arm_ = model->GetLink(sdf->Get<std::string>("arm_link", "arm").first);
      if (!lift_ || !reach_ || !arm_) {
        gzerr << "[custom_robot] " << model->GetName() << ": picker needs lift_joint, reach_joint and arm\n";
        return;
      }
      trayX_ = sdf->Get<double>("tray_x", 0.3).first;
      trayZero_ = sdf->Get<double>("tray_zero", 0.1).first;
      reachMax_ = sdf->Get<double>("reach_max", 1.45).first;
      slotX_ = sdf->Get<double>("slot_x", -0.4).first;
      aisleHalf_ = sdf->Get<double>("aisle_half", 0.78).first;
      std::istringstream slots(sdf->Get<std::string>("slot_heights", "0.3 0.85 1.4 1.95").first);
      for (double z; slots >> z;) {
        slotTops_.push_back(z);
      }
      slots_.resize(slotTops_.size());
      liftTarget_ = liftCommand_ = trayZero_;
    } else {
      deckLength_ = sdf->Get<double>("deck_length", 1.2).first;
      deckWidth_ = sdf->Get<double>("deck_width", 0.75).first;
      deckTop_ = sdf->Get<double>("deck_top", 0.64).first;
      rollerSpeed_ = sdf->Get<double>("roller_speed", 0.4).first;
    }

    ros_ = gazebo_ros::Node::Get(sdf);
    std::string ns = ros_->get_namespace();
    prefix_ = ns == "/" ? "" : ns.substr(1) + "/";

    cmdSub_ = ros_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        cmdV_ = std::clamp(msg->linear.x, -maxSpeed_, maxSpeed_);
        cmdW_ = std::clamp(msg->angular.z, -maxTurn_, maxTurn_);
        lastCmd_ = world_->SimTime().Double();
      });
    commandSub_ = ros_->create_subscription<std_msgs::msg::String>(
      "handler/command", 10, [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        commands_.push_back(msg->data);
      });
    odomPub_ = ros_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    posePub_ = ros_->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    statePub_ = ros_->create_publisher<std_msgs::msg::String>("handler/state", rclcpp::QoS(1).transient_local());
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_);
    staticTf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(ros_);

    start_ = base_->WorldPose();
    PublishStaticFrames();
    update_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CustomRobotPlugin::OnUpdate, this, std::placeholders::_1));
    gzmsg << "[custom_robot] " << model->GetName() << " (" << role_ << ") on " << ns << "\n";
  }

private:
  struct Step
  {
    enum Kind { kLift, kReach, kGrip, kDrop, kSlide, kStow, kUnstow, kDone } kind;
    double value{0.0};          // joint target, or slide duration
    physics::LinkPtr box;
    int slot{-1};
    Vector3d to;                // kSlide: box centre in the base frame
    bool free{false};           // kDrop: hand the box back to the world
    std::string message;
  };

  void PublishStaticFrames()
  {
    auto stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(world_->SimTime());
    std::vector<geometry_msgs::msg::TransformStamped> frames(2);
    frames[0].header.stamp = stamp;
    frames[0].header.frame_id = "map";
    frames[0].child_frame_id = prefix_ + "odom";
    frames[0].transform = ToTransform(start_);
    frames[1].header.stamp = stamp;
    frames[1].header.frame_id = prefix_ + "base_link";
    frames[1].child_frame_id = prefix_ + "laser";
    frames[1].transform = ToTransform(laserPose_);
    staticTf_->sendTransform(frames);
  }

  void OnUpdate(const common::UpdateInfo &info)
  {
    const double now = info.simTime.Double();
    const double dt = std::max(0.0, now - lastUpdate_);
    lastUpdate_ = now;

    std::deque<std::string> commands;
    double cmdV, cmdW;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      commands.swap(commands_);
      const bool fresh = now - lastCmd_ < 0.5;
      cmdV = fresh ? cmdV_ : 0.0;
      cmdW = fresh ? cmdW_ : 0.0;
    }
    Drive(cmdV, cmdW, dt);
    for (const auto &c : commands) {
      Command(c);
    }
    if (role_ == "picker") {
      RunSteps(dt);
      HoldJoint(lift_, liftTarget_ - trayZero_, 0.4, 3000.0);
      const double stages = reach2_ ? 2.0 : 1.0;
      HoldJoint(reach_, reachTarget_ / stages, 0.5 / stages, 800.0);
      if (reach2_) {
        HoldJoint(reach2_, reachTarget_ / stages, 0.5 / stages, 800.0);
      }
    } else {
      RunDeck();
    }
    if (now - lastOdom_ >= 1.0 / 30.0) {
      lastOdom_ = now;
      PublishPose(info.simTime);
    }
    if (stateDirty_ || now - lastState_ >= 0.5) {
      lastState_ = now;
      stateDirty_ = false;
      PublishState();
    }
  }

  // ------------------------------------------------------------------ base

  void Drive(double cmdV, double cmdW, double dt)
  {
    v_ = Approach(v_, cmdV, accel_ * dt);
    w_ = Approach(w_, cmdW, 2.0 * accel_ * dt);
    const double yaw = base_->WorldPose().Rot().Yaw();
    const double vz = base_->WorldLinearVel().Z();
    base_->SetLinearVel(Vector3d(v_ * std::cos(yaw), v_ * std::sin(yaw), std::min(vz, 0.0)));
    base_->SetAngularVel(Vector3d(0, 0, w_));
  }

  void PublishPose(const common::Time &time)
  {
    const auto stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(time);
    const Pose3d world = base_->WorldPose();
    const Pose3d odom = start_.Inverse() * world;

    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = prefix_ + "odom";
    o.child_frame_id = prefix_ + "base_link";
    o.pose.pose = ToPose(odom);
    o.twist.twist.linear.x = v_;
    o.twist.twist.angular.z = w_;
    odomPub_->publish(o);

    geometry_msgs::msg::TransformStamped t;
    t.header = o.header;
    t.child_frame_id = o.child_frame_id;
    t.transform = ToTransform(odom);
    tf_->sendTransform(t);

    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = stamp;
    p.header.frame_id = "map";
    p.pose = ToPose(world);
    posePub_->publish(p);
  }

  // -------------------------------------------------------------- commands

  void Command(const std::string &line)
  {
    std::istringstream in(line);
    std::string verb, arg;
    in >> verb >> arg;
    if (role_ == "picker" && !steps_.empty()) {
      return Finish("error: busy, ignored '" + line + "'");
    }
    if (role_ == "transport" && !unloading_.empty()) {
      return Finish("error: busy, ignored '" + line + "'");
    }
    try {
      if (role_ == "picker" && verb == "lift") {
        liftCommand_ = std::clamp(std::stod(arg), trayZero_, trayZero_ + lift_->UpperLimit(0));
        steps_.push_back({Step::kLift, liftCommand_});
        steps_.push_back(Done("ok: tray at " + std::to_string(liftCommand_).substr(0, 5) + " m"));
      } else if (role_ == "picker" && verb == "pick") {
        PlanPick(Side(arg));
      } else if (role_ == "picker" && verb == "store") {
        PlanStore(arg.empty() ? -1 : std::stoi(arg));
      } else if (role_ == "picker" && verb == "retrieve") {
        PlanRetrieve(std::stoi(arg));
      } else if (role_ == "picker" && verb == "place") {
        double y = -1.0;
        in >> y;
        PlanPlace(Side(arg), y);
      } else if (role_ == "transport" && verb == "unload") {
        StartUnload(arg == "back" ? -1.0 : 1.0, arg);
      } else {
        Finish("error: unknown command '" + line + "'");
      }
    } catch (const std::exception &) {
      steps_.clear();
      Finish("error: bad arguments in '" + line + "'");
    }
    busy_ = !steps_.empty() || !unloading_.empty();
    stateDirty_ = true;
  }

  static double Side(const std::string &s)
  {
    if (s == "left") return 1.0;
    if (s == "right") return -1.0;
    throw std::invalid_argument(s);
  }

  static Step Done(const std::string &message)
  {
    Step s{Step::kDone};
    s.message = message;
    return s;
  }

  void Finish(const std::string &result)
  {
    result_ = result;
    busy_ = !steps_.empty() || !unloading_.empty();
    stateDirty_ = true;
    gzmsg << "[custom_robot] " << model_->GetName() << ": " << result << "\n";
  }

  // ---------------------------------------------------------------- picker

  double TrayTop() const { return trayZero_ + lift_->Position(0); }

  // Offset of the arm centre from the tray centre line along the robot's y axis.
  double Reach() const { return reach_->Position(0) + (reach2_ ? reach2_->Position(0) : 0.0); }

  // Bottom of the tray box relative to the tray top (it rides slightly above after a pick).
  double BoxOffset(const physics::LinkPtr &box) const
  {
    const double bottom = box->WorldPose().Pos().Z() - BoxSize(box).Z() / 2;
    return bottom - (base_->WorldPose().Pos().Z() + TrayTop());
  }

  Vector3d InBase(const Vector3d &world) const
  {
    const Pose3d b = base_->WorldPose();
    return b.Rot().RotateVectorReverse(world - b.Pos());
  }

  // Half extent of a box along the robot's y axis.
  double HalfDepth(const physics::LinkPtr &box) const
  {
    const Vector3d size = BoxSize(box);
    const double yaw = (base_->WorldPose().Rot().Inverse() * box->WorldPose().Rot()).Yaw();
    return (std::abs(std::sin(yaw)) * size.X() + std::abs(std::cos(yaw)) * size.Y()) / 2;
  }

  void PlanPick(double side)
  {
    if (tray_) {
      return Finish("error: tray is not empty, store or place its box first");
    }
    physics::LinkPtr best;
    double bestGap = 1e9, bestBottom = 0;
    for (const auto &m : world_->Models()) {
      if (m->IsStatic() || m == model_ || m->GetName().rfind(loadPrefix_, 0) != 0) {
        continue;
      }
      auto link = m->GetLink();
      if (!link || IsHeld(link)) {
        continue;
      }
      const Vector3d p = InBase(link->WorldPose().Pos());
      const double bottom = p.Z() - BoxSize(link).Z() / 2;
      const double gap = side * p.Y() - HalfDepth(link);
      if (std::abs(p.X() - trayX_) > 0.25 || gap < 0.4 || side * p.Y() > reachMax_ ||
          std::abs(bottom - TrayTop()) > 0.1) {
        continue;
      }
      if (gap < bestGap) {
        best = link;
        bestGap = gap;
        bestBottom = bottom;
      }
    }
    if (!best || !Claim(best)) {
      return Finish("error: no free box beside the tray on the " + std::string(side > 0 ? "left" : "right") +
                    " at tray height " + std::to_string(TrayTop()).substr(0, 4) + " m");
    }
    const double y = InBase(best->WorldPose().Pos()).Y();
    steps_.push_back({Step::kLift, bestBottom});   // align the tray with the box bottom
    steps_.push_back({Step::kReach, y});            // arms on both sides of the box
    Step grip{Step::kGrip};
    grip.box = best;
    steps_.push_back(grip);
    steps_.push_back({Step::kLift, bestBottom + 0.02});
    steps_.push_back({Step::kReach, 0.0});
    steps_.push_back(Done("ok: picked " + best->GetModel()->GetName()));
  }

  void PlanStore(int slot)
  {
    if (!tray_) {
      return Finish("error: tray is empty");
    }
    if (slot < 0) {
      slot = static_cast<int>(std::find(slots_.begin(), slots_.end(), nullptr) - slots_.begin());
    }
    if (slot >= static_cast<int>(slots_.size()) || slots_[slot]) {
      return Finish("error: slot " + std::to_string(slot) + " is not free");
    }
    auto box = tray_;
    const double h = BoxSize(box).Z();
    steps_.push_back({Step::kLift, slotTops_[slot] + 0.005 - BoxOffset(box)});
    Step drop{Step::kDrop};
    steps_.push_back(drop);
    Step slide{Step::kSlide, 1.5};
    slide.box = box;
    slide.to = Vector3d(slotX_, 0, slotTops_[slot] + h / 2 + 0.005);
    steps_.push_back(slide);
    Step stow{Step::kStow};
    stow.box = box;
    stow.slot = slot;
    steps_.push_back(stow);
    steps_.push_back({Step::kReach, 0.0});
    steps_.push_back(Done("ok: stored " + box->GetModel()->GetName() + " in slot " + std::to_string(slot)));
  }

  void PlanRetrieve(int slot)
  {
    if (tray_) {
      return Finish("error: tray is not empty");
    }
    if (slot < 0 || slot >= static_cast<int>(slots_.size()) || !slots_[slot]) {
      return Finish("error: slot " + std::to_string(slot) + " is empty");
    }
    auto box = slots_[slot];
    const double h = BoxSize(box).Z();
    steps_.push_back({Step::kLift, slotTops_[slot]});
    steps_.push_back({Step::kReach, 0.0});
    Step unstow{Step::kUnstow};
    unstow.slot = slot;
    steps_.push_back(unstow);
    Step slide{Step::kSlide, 1.5};
    slide.box = box;
    slide.to = Vector3d(trayX_, 0, slotTops_[slot] + h / 2 + 0.005);
    steps_.push_back(slide);
    Step grip{Step::kGrip};
    grip.box = box;
    steps_.push_back(grip);
    steps_.push_back(Done("ok: retrieved " + box->GetModel()->GetName() + " from slot " + std::to_string(slot)));
  }

  void PlanPlace(double side, double y)
  {
    if (!tray_) {
      return Finish("error: tray is empty");
    }
    auto box = tray_;
    const double depth = HalfDepth(box);
    if (y < 0) {
      y = aisleHalf_ + depth;
    }
    const double offset = BoxOffset(box);
    const double shift = side * y - InBase(box->WorldPose().Pos()).Y();
    const double target = Reach() + shift;
    if (std::abs(target) > reachMax_) {
      return Finish("error: " + std::to_string(y).substr(0, 4) + " m is out of reach");
    }
    steps_.push_back({Step::kLift, liftCommand_ + 0.02 - offset});   // box bottom 2 cm above the surface
    steps_.push_back({Step::kReach, target});
    steps_.push_back({Step::kLift, liftCommand_ + 0.003 - offset});
    Step drop{Step::kDrop};
    drop.free = true;
    steps_.push_back(drop);
    steps_.push_back({Step::kReach, 0.0});
    steps_.push_back({Step::kLift, liftCommand_});
    steps_.push_back(Done("ok: placed " + box->GetModel()->GetName()));
  }

  physics::JointPtr Fix(const physics::LinkPtr &parent, const physics::LinkPtr &child)
  {
    auto joint = world_->Physics()->CreateJoint("fixed", model_);
    joint->SetName(model_->GetName() + "_holds_" + child->GetModel()->GetName());
    joint->Load(parent, child, Pose3d());
    joint->Init();
    return joint;
  }

  static void Unfix(physics::JointPtr &joint)
  {
    if (joint) {
      joint->Detach();
      joint.reset();
    }
  }

  void RunSteps(double dt)
  {
    while (!steps_.empty()) {
      Step &s = steps_.front();
      switch (s.kind) {
        case Step::kLift:
          liftTarget_ = s.value;
          if (std::abs(TrayTop() - s.value) > 0.003) {
            return Timeout(dt);
          }
          break;
        case Step::kReach:
          reachTarget_ = s.value;
          if (std::abs(Reach() - s.value) > 0.003) {
            return Timeout(dt);
          }
          break;
        case Step::kGrip:
          tray_ = s.box;
          trayJoint_ = Fix(arm_, s.box);
          break;
        case Step::kDrop:
          Unfix(trayJoint_);
          if (s.free) {
            Unclaim(tray_);
          }
          tray_.reset();
          break;
        case Step::kSlide: {
          if (slideTime_ == 0.0) {
            slideFrom_ = InBase(s.box->WorldPose().Pos());
            slideRot_ = base_->WorldPose().Rot().Inverse() * s.box->WorldPose().Rot();
          }
          slideTime_ = std::min(slideTime_ + dt, s.value);
          const double a = slideTime_ / s.value;
          const Pose3d b = base_->WorldPose();
          const Vector3d p = slideFrom_ + (s.to - slideFrom_) * (a * a * (3 - 2 * a));
          s.box->GetModel()->SetWorldPose(Pose3d(b.Pos() + b.Rot().RotateVector(p), b.Rot() * slideRot_));
          s.box->SetLinearVel(base_->WorldLinearVel());
          s.box->SetAngularVel(Vector3d::Zero);
          if (slideTime_ < s.value) {
            return;
          }
          slideTime_ = 0.0;
          break;
        }
        case Step::kStow:
          slots_[s.slot] = s.box;
          slotJoints_.resize(slots_.size());
          slotJoints_[s.slot] = Fix(base_, s.box);
          break;
        case Step::kUnstow:
          slotJoints_.resize(slots_.size());
          Unfix(slotJoints_[s.slot]);
          slots_[s.slot].reset();
          break;
        case Step::kDone: {
          const std::string message = s.message;
          steps_.pop_front();
          stepTime_ = 0.0;
          Finish(message);
          continue;
        }
      }
      steps_.pop_front();
      stepTime_ = 0.0;
      stateDirty_ = true;
    }
  }

  void Timeout(double dt)
  {
    stepTime_ += dt;
    if (stepTime_ > 15.0) {
      steps_.clear();
      stepTime_ = 0.0;
      Finish("error: the handler got stuck (blocked by something?)");
    }
  }

  static void HoldJoint(const physics::JointPtr &joint, double target, double speed, double force)
  {
    const double error = target - joint->Position(0);
    joint->SetParam("fmax", 0, force);
    joint->SetParam("vel", 0, std::clamp(4.0 * error, -speed, speed));
  }

  // ------------------------------------------------------------- transport

  bool OnDeck(const Vector3d &p, double height, double margin) const
  {
    const double bottom = p.Z() - height / 2;
    return std::abs(p.X()) < deckLength_ / 2 + margin && std::abs(p.Y()) < deckWidth_ / 2 + margin &&
           bottom > deckTop_ - 0.02 && bottom < deckTop_ + 0.08;
  }

  void RunDeck()
  {
    if (!unloading_.empty()) {
      const Pose3d b = base_->WorldPose();
      const Vector3d push = b.Rot().RotateVector(Vector3d(unloadDir_ * rollerSpeed_, 0, 0));
      for (auto it = unloading_.begin(); it != unloading_.end();) {
        const Vector3d p = InBase((*it)->WorldPose().Pos());
        const Vector3d size = BoxSize(*it);
        const double yaw = (b.Rot().Inverse() * (*it)->WorldPose().Rot()).Yaw();
        const double halfX = (std::abs(std::cos(yaw)) * size.X() + std::abs(std::sin(yaw)) * size.Y()) / 2;
        const double bottom = p.Z() - size.Z() / 2;
        // keep rolling until the whole box has left the deck, so the next conveyor takes it over
        if (std::abs(p.X()) - halfX > deckLength_ / 2 + 0.02 || bottom < deckTop_ - 0.1 || bottom > deckTop_ + 0.1) {
          Unclaim(*it);
          it = unloading_.erase(it);
          continue;
        }
        const double vz = (*it)->WorldLinearVel().Z();
        (*it)->SetEnabled(true);
        (*it)->SetLinearVel(base_->WorldLinearVel() + Vector3d(push.X(), push.Y(), vz));
        ++it;
      }
      if (unloading_.empty()) {
        quietUntil_ = world_->SimTime().Double() + 2.0;
        Finish("ok: unloaded " + std::to_string(unloadCount_) + " box(es) " + unloadSide_);
      }
      return;
    }
    if (deckScan_++ % 25 != 0 || world_->SimTime().Double() < quietUntil_) {
      return;
    }
    for (const auto &m : world_->Models()) {
      if (m->IsStatic() || m == model_ || m->GetName().rfind(loadPrefix_, 0) != 0) {
        continue;
      }
      auto link = m->GetLink();
      if (!link || IsHeld(link)) {
        continue;
      }
      const Vector3d p = InBase(link->WorldPose().Pos());
      if (!OnDeck(p, BoxSize(link).Z(), 0.0) ||
          (link->WorldLinearVel() - base_->WorldLinearVel()).Length() > 0.05 || !Claim(link)) {
        continue;
      }
      deck_.push_back(link);
      deckJoints_.push_back(Fix(base_, link));
      stateDirty_ = true;
    }
  }

  void StartUnload(double dir, const std::string &side)
  {
    if (deck_.empty()) {
      return Finish("error: the deck is empty");
    }
    unloadDir_ = dir;
    unloadSide_ = side.empty() ? "front" : side;
    unloadCount_ = static_cast<int>(deck_.size());
    for (auto &j : deckJoints_) {
      Unfix(j);
    }
    unloading_ = deck_;
    deck_.clear();
    deckJoints_.clear();
    busy_ = true;
  }

  // ----------------------------------------------------------------- state

  void PublishState()
  {
    std::ostringstream s;
    s << "{\"robot\": \"" << model_->GetName() << "\", \"role\": \"" << role_ << "\", \"busy\": "
      << (busy_ ? "true" : "false");
    if (role_ == "picker") {
      s << ", \"tray_height\": " << std::round(TrayTop() * 1000) / 1000 << ", \"tray\": " << JsonName(tray_)
        << ", \"slots\": [";
      for (size_t i = 0; i < slots_.size(); ++i) {
        s << (i ? ", " : "") << JsonName(slots_[i]);
      }
      s << "]";
    } else {
      s << ", \"deck\": [";
      for (size_t i = 0; i < deck_.size(); ++i) {
        s << (i ? ", " : "") << JsonName(deck_[i]);
      }
      s << "]";
    }
    s << ", \"result\": \"" << result_ << "\"}";
    std_msgs::msg::String msg;
    msg.data = s.str();
    statePub_->publish(msg);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr base_;
  std::string role_, prefix_, loadPrefix_, result_{"ready"};
  double maxSpeed_{1.0}, maxTurn_{1.2}, accel_{0.8};
  Pose3d start_, laserPose_;

  gazebo_ros::Node::SharedPtr ros_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandSub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statePub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> staticTf_;
  event::ConnectionPtr update_;

  std::mutex mutex_;
  std::deque<std::string> commands_;
  double cmdV_{0}, cmdW_{0}, lastCmd_{-1e9};
  double v_{0}, w_{0};
  double lastUpdate_{0}, lastOdom_{-1e9}, lastState_{-1e9};
  bool busy_{false}, stateDirty_{true};

  // picker
  physics::JointPtr lift_, reach_, reach2_;
  physics::LinkPtr arm_;
  double trayX_{0.3}, trayZero_{0.1}, reachMax_{1.45}, slotX_{-0.4}, aisleHalf_{0.78};
  double liftTarget_{0.1}, liftCommand_{0.1}, reachTarget_{0.0};
  std::vector<double> slotTops_;
  std::deque<Step> steps_;
  double stepTime_{0.0}, slideTime_{0.0};
  Vector3d slideFrom_;
  Quaterniond slideRot_;
  physics::LinkPtr tray_;
  physics::JointPtr trayJoint_;
  std::vector<physics::LinkPtr> slots_;
  std::vector<physics::JointPtr> slotJoints_;

  // transport
  double deckLength_{1.2}, deckWidth_{0.75}, deckTop_{0.64}, rollerSpeed_{0.4};
  std::vector<physics::LinkPtr> deck_, unloading_;
  std::vector<physics::JointPtr> deckJoints_;
  double unloadDir_{1.0}, quietUntil_{0.0};
  std::string unloadSide_;
  int unloadCount_{0};
  unsigned deckScan_{0};
};

GZ_REGISTER_MODEL_PLUGIN(CustomRobotPlugin)
}  // namespace gazebo
