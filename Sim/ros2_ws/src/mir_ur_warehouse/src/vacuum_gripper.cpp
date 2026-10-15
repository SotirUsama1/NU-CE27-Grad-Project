// Multi-cup vacuum gripper for Gazebo Classic, modelled on an industrial carton gripper: a plate
// of bellows cups, each behind its own flow valve, on one vacuum generator. No joint is created;
// boxes are held only by physics.
//   - Each cup is a bellows standing stroke in front of the plate. It seals on a flat box face
//     once its lip touches it (with seal_gap of leakage beyond that), less with the tilt between
//     cup and face and the part of its rim that hangs over an edge.
//   - A cup's vacuum approaches max_vacuum * seal with time constant evacuation_time while the
//     valve is on, and vents with release_time when it is off or the seal is lost. The flow
//     valves keep a leaking cup from pulling down the others.
//   - Each cup pulls its box towards the plate with vacuum * cup area. The bellows push back as a
//     spring-damper along the cup axis when compressed, so the box seats on the cups rather than
//     on the plate. The reactions are not applied to the plate: the arm is position controlled
//     (moved kinematically), and Gazebo keeps the velocity such forces give its links, so they
//     would build up and slam the plate into the box.
//   - In the plane of the face each sealed cup is a spring-damper (the bellows lip) anchored to
//     the face. Its force is limited to friction * vacuum * cup area; beyond that the lip slips
//     and the anchor moves. Weight, inertia and twist are carried this way, so overloaded,
//     badly gripped or jerked boxes slip, peel off or fall. The cup velocity is taken from its
//     motion, so this also works when the arm is position controlled (moved kinematically).
// Interface (like the valve output and vacuum switch of a real gripper)
//   /suction/set_vacuum   (std_srvs/SetBool)   valve on / off (blow-off)
//   /suction/vacuum       (std_msgs/Float64)   mean vacuum of the cups in kPa below atmosphere
//   /suction/part_present (std_msgs/Bool)      vacuum switch, on above switch_ratio * max_vacuum
// Only box collision shapes are gripped.

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace mir_ur_warehouse
{

class VacuumGripper : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);

    pad_ = model->GetLink(sdf->Get<std::string>("pad_link", "suction_pad").first);
    if (!pad_) {
      RCLCPP_ERROR(node_->get_logger(), "vacuum_gripper: pad_link not found");
      return;
    }
    pad_offset_ = sdf->Get<double>("pad_offset", pad_offset_).first;
    cup_radius_ = sdf->Get<double>("cup_radius", cup_radius_).first;
    max_vacuum_ = sdf->Get<double>("max_vacuum", max_vacuum_).first;
    stroke_ = sdf->Get<double>("stroke", stroke_).first;
    seal_gap_ = sdf->Get<double>("seal_gap", seal_gap_).first;
    bellows_stiffness_ = sdf->Get<double>("bellows_stiffness", bellows_stiffness_).first;
    bellows_damping_ = sdf->Get<double>("bellows_damping", bellows_damping_).first;
    full_seal_angle_ = sdf->Get<double>("full_seal_angle", full_seal_angle_).first;
    max_seal_angle_ = sdf->Get<double>("max_seal_angle", max_seal_angle_).first;
    evacuation_time_ = sdf->Get<double>("evacuation_time", evacuation_time_).first;
    release_time_ = sdf->Get<double>("release_time", release_time_).first;
    switch_ratio_ = sdf->Get<double>("switch_ratio", switch_ratio_).first;
    friction_ = sdf->Get<double>("friction", friction_).first;
    lip_stiffness_ = sdf->Get<double>("lip_stiffness", lip_stiffness_).first;
    lip_damping_ = sdf->Get<double>("lip_damping", lip_damping_).first;

    // <cups>x1 y1 x2 y2 ...</cups>, cup centres on the plate face in the pad link frame.
    std::istringstream cups(sdf->Get<std::string>("cups", "0 0").first);
    double x, y;
    while (cups >> x >> y) {
      cups_.push_back(Cup{ignition::math::Vector3d(x, y, pad_offset_)});
    }

    vacuum_pub_ = node_->create_publisher<std_msgs::msg::Float64>("suction/vacuum", 10);
    present_pub_ = node_->create_publisher<std_msgs::msg::Bool>("suction/part_present", 10);
    valve_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "suction/set_vacuum",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        valve_ = req->data;
        res->success = true;
        res->message = req->data ? "vacuum on" : "vacuum off";
      });

    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&VacuumGripper::OnUpdate, this, std::placeholders::_1));
  }

private:
  struct Cup
  {
    ignition::math::Vector3d offset;
    gazebo::physics::CollisionPtr target;
    double vacuum{0.0};
    bool anchored{false};
    ignition::math::Vector3d anchor;   // on the face, in the target link frame
    ignition::math::Vector3d last_centre;
    ignition::math::Vector3d velocity;
  };

  struct Seal
  {
    double quality{0.0};
    double gap{1.0};
    ignition::math::Vector3d point;
  };

  // Seal of a cup (face centre c, axis n pointing out of the cup) on one box collision.
  Seal SealOn(const gazebo::physics::CollisionPtr & col, const ignition::math::Vector3d & c,
    const ignition::math::Vector3d & n) const
  {
    Seal s;
    auto box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(col->GetShape());
    if (!box) {
      return s;
    }
    const auto pose = col->WorldPose();
    const auto half = box->Size() / 2;
    const auto lc = pose.Rot().RotateVectorReverse(c - pose.Pos());
    const auto ln = pose.Rot().RotateVectorReverse(n);

    // The face the cup looks at is the one whose outward normal is most opposed to the cup axis.
    int axis = 0;
    for (int i = 1; i < 3; ++i) {
      if (std::abs(ln[i]) > std::abs(ln[axis])) {
        axis = i;
      }
    }
    const double out = ln[axis] > 0 ? -1.0 : 1.0;
    const double tilt = std::acos(std::min(std::abs(ln[axis]), 1.0));
    s.gap = out * lc[axis] - half[axis];
    ignition::math::Vector3d lp = lc;
    lp[axis] = out * half[axis];
    s.point = pose.Pos() + pose.Rot().RotateVector(lp);
    if (tilt >= max_seal_angle_ || s.gap > stroke_ + seal_gap_ || s.gap < -stroke_) {
      return s;
    }
    const double angle_factor =
      std::clamp((max_seal_angle_ - tilt) / (max_seal_angle_ - full_seal_angle_), 0.0, 1.0);
    const double gap_factor = std::clamp((stroke_ + seal_gap_ - s.gap) / seal_gap_, 0.0, 1.0);

    // Rim on the face: an overhanging rim leaks, so the seal falls off steeply.
    const int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
    int on_face = 0;
    constexpr int kRim = 24;
    for (int k = 0; k < kRim; ++k) {
      const double u = 2 * M_PI * k / kRim;
      const double p1 = lc[a1] + cup_radius_ * std::cos(u);
      const double p2 = lc[a2] + cup_radius_ * std::sin(u);
      if (std::abs(p1) <= half[a1] && std::abs(p2) <= half[a2]) {
        ++on_face;
      }
    }
    const double rim = static_cast<double>(on_face) / kRim;

    s.quality = std::pow(rim, 4) * angle_factor * gap_factor;
    return s;
  }

  // Best seal for a cup on any movable box near it.
  Seal Search(const ignition::math::Vector3d & c, const ignition::math::Vector3d & n,
    gazebo::physics::CollisionPtr & target) const
  {
    Seal best;
    const auto margin = ignition::math::Vector3d::One * (cup_radius_ + stroke_ + seal_gap_);
    for (const auto & m : world_->Models()) {
      if (m == model_ || m->IsStatic()) {
        continue;
      }
      for (const auto & link : m->GetLinks()) {
        auto bb = link->BoundingBox();
        bb.Min() -= margin;
        bb.Max() += margin;
        if (!bb.Contains(c)) {
          continue;
        }
        for (const auto & col : link->GetCollisions()) {
          const auto s = SealOn(col, c, n);
          if (s.quality > best.quality) {
            best = s;
            target = col;
          }
        }
      }
    }
    return best;
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double dt = (info.simTime - last_time_).Double();
    last_time_ = info.simTime;
    if (dt <= 0.0 || dt > 0.1) {
      return;
    }

    const auto pad_pose = pad_->WorldPose();
    const auto n = pad_pose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);
    const double area = M_PI * cup_radius_ * cup_radius_;
    double total = 0.0;

    for (auto & cup : cups_) {
      const auto c = pad_pose.CoordPositionAdd(cup.offset);
      // Differentiated position, low-pass filtered (20 ms): the kinematically moved arm jitters
      // by fractions of a millimetre per step, which would otherwise read as large velocities.
      cup.velocity += ((c - cup.last_centre) / dt - cup.velocity) * (dt / (0.02 + dt));
      cup.last_centre = c;
      const auto cup_vel = cup.velocity;
      Seal seal;
      if (cup.target) {
        seal = SealOn(cup.target, c, n);
        if (seal.gap > stroke_ + seal_gap_) {
          cup.target.reset();
          cup.anchored = false;
        }
      }
      if (!cup.target) {
        seal = Search(c, n, cup.target);
      }

      const double target_vacuum = valve_ && cup.target ? max_vacuum_ * seal.quality : 0.0;
      const double tau = target_vacuum > cup.vacuum ? evacuation_time_ : release_time_;
      cup.vacuum += (target_vacuum - cup.vacuum) * (1.0 - std::exp(-dt / tau));
      total += cup.vacuum;

      if (!cup.target) {
        cup.anchored = false;
        continue;
      }
      const auto link = cup.target->GetLink();
      const auto link_pose = link->WorldPose();
      const double suction = cup.vacuum * area;
      // Bellows: pushes the face away once compressed, damped by the closing speed.
      const double compression = stroke_ - seal.gap;
      double push = 0.0;
      if (compression > 0.0) {
        const double closing = (cup_vel - link->WorldLinearVel(link_pose.Rot().RotateVectorReverse(
            seal.point - link_pose.Pos()))).Dot(n);
        push = std::max(0.0, bellows_stiffness_ * compression + bellows_damping_ * closing);
      }
      link->AddForceAtWorldPosition(n * (push - suction), seal.point);
      if (cup.vacuum < 1.0) {
        cup.anchored = false;
        continue;
      }
      // Lip: spring-damper in the face plane, limited by friction.
      auto to_link = [&link_pose](const ignition::math::Vector3d & p) {
          return link_pose.Rot().RotateVectorReverse(p - link_pose.Pos());
        };
      if (!cup.anchored) {
        // The lip grips once it touches the face; until then only the suction pulls.
        if (seal.quality <= 0.0 || compression < 0.0) {
          continue;
        }
        cup.anchor = to_link(seal.point);
        cup.anchored = true;
      }
      const auto a = link_pose.CoordPositionAdd(cup.anchor);
      auto stretch = c - a;
      stretch -= n * stretch.Dot(n);
      const auto a_vel = link->WorldLinearVel(cup.anchor);
      auto slide = cup_vel - a_vel;
      slide -= n * slide.Dot(n);
      auto lip = stretch * lip_stiffness_ + slide * lip_damping_;
      const double limit = friction_ * suction;
      if (lip.Length() > limit) {
        lip *= limit / lip.Length();
        const double len = stretch.Length();
        if (len > limit / lip_stiffness_) {
          const auto moved = a + stretch * (1.0 - limit / lip_stiffness_ / len);
          cup.anchor = to_link(moved);
        }
      }
      link->AddForceAtWorldPosition(lip, a);
    }

    if ((info.simTime - last_publish_).Double() >= 0.05) {
      last_publish_ = info.simTime;
      const double mean = cups_.empty() ? 0.0 : total / cups_.size();
      std_msgs::msg::Float64 v;
      v.data = mean / 1000.0;
      vacuum_pub_->publish(v);
      const double ratio = mean / max_vacuum_;
      present_ = ratio > switch_ratio_ || (present_ && ratio > switch_ratio_ - 0.1);
      std_msgs::msg::Bool p;
      p.data = present_;
      present_pub_->publish(p);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr pad_;
  gazebo::event::ConnectionPtr update_;
  gazebo::common::Time last_time_, last_publish_;
  std::vector<Cup> cups_;

  double pad_offset_{0.01};
  double cup_radius_{0.03};
  double max_vacuum_{40000.0};
  double stroke_{0.01};
  double seal_gap_{0.002};
  double bellows_stiffness_{20000.0};
  double bellows_damping_{150.0};
  double full_seal_angle_{0.09};
  double max_seal_angle_{0.26};
  double evacuation_time_{0.15};
  double release_time_{0.25};
  double switch_ratio_{0.6};
  double friction_{0.6};
  double lip_stiffness_{5000.0};
  double lip_damping_{50.0};

  std::atomic<bool> valve_{false};
  bool present_{false};

  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vacuum_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr present_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr valve_srv_;
};

GZ_REGISTER_MODEL_PLUGIN(VacuumGripper)

}  // namespace mir_ur_warehouse
