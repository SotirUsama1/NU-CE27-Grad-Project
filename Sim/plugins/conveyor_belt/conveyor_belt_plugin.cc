// Moves every dynamic link resting on the belt along the belt's +x axis.
//
// SDF parameters (all in the model frame, belt centred on the model origin):
//   <belt_length>  belt length along x              [m]
//   <belt_width>   usable width along y             [m]
//   <belt_top>     height of the belt surface       [m]
//   <reach>        how far above the surface a link centre may be to count as "on the belt" [m]
//   <speed>        belt speed, negative runs backwards [m/s]
//   <overrun>      keep pushing this far past the belt end so boxes clear it [m]
//
// The speed can be changed at runtime:
//   gz topic -p /gazebo/default/<model name>/speed -m 'data: "0.5"'
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class ConveyorBeltPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    length_ = sdf->Get<double>("belt_length", 6.0).first;
    width_ = sdf->Get<double>("belt_width", 0.9).first;
    top_ = sdf->Get<double>("belt_top", 0.7).first;
    reach_ = sdf->Get<double>("reach", 0.6).first;
    speed_ = sdf->Get<double>("speed", 0.3).first;
    overrun_ = sdf->Get<double>("overrun", 0.3).first;

    node_ = transport::NodePtr(new transport::Node());
    node_->Init(world_->Name());
    speedSub_ = node_->Subscribe("~/" + model_->GetName() + "/speed", &ConveyorBeltPlugin::OnSpeed, this);

    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ConveyorBeltPlugin::OnUpdate, this));
    gzmsg << "[conveyor_belt] " << model_->GetName() << ": " << length_ << " m belt at " << speed_
          << " m/s, speed topic ~/" << model_->GetName() << "/speed\n";
  }

private:
  void OnSpeed(ConstGzStringPtr &msg)
  {
    try {
      speed_ = std::stod(msg->data());
      gzmsg << "[conveyor_belt] " << model_->GetName() << " speed set to " << speed_ << " m/s\n";
    } catch (const std::exception &) {
      gzerr << "[conveyor_belt] speed must be a number, got '" << msg->data() << "'\n";
    }
  }

  // Scanning every model of a large world each step is costly, so the links near the belt
  // are collected every kRescanSteps steps with a margin wide enough for anything that can
  // move onto the belt in the meantime.
  void Rescan(const ignition::math::Pose3d &pose)
  {
    nearby_.clear();
    const double radius = length_ / 2 + width_ + reach_ + 1.0;
    for (const auto &model : world_->Models()) {
      if (model == model_ || model->IsStatic() ||
          (model->WorldPose().Pos() - pose.Pos()).Length() > radius) {
        continue;
      }
      for (const auto &link : model->GetLinks()) {
        nearby_.push_back(link);
      }
    }
  }

  void OnUpdate()
  {
    if (speed_ == 0.0) {
      return;
    }
    const auto pose = model_->WorldPose();
    if (steps_++ % kRescanSteps == 0) {
      Rescan(pose);
    }
    const auto dir = pose.Rot().RotateVector(ignition::math::Vector3d::UnitX);
    for (const auto &link : nearby_) {
      const auto p = pose.Rot().RotateVectorReverse(link->WorldPose().Pos() - pose.Pos());
      if (p.X() < -length_ / 2 || p.X() > length_ / 2 + overrun_ || std::abs(p.Y()) > width_ / 2 ||
          p.Z() < top_ || p.Z() > top_ + reach_) {
        continue;
      }
      const auto v = link->WorldLinearVel();
      link->SetEnabled(true);
      link->SetLinearVel(dir * speed_ + ignition::math::Vector3d(0, 0, v.Z()));
    }
  }

  static constexpr unsigned kRescanSteps = 50;
  std::vector<physics::LinkPtr> nearby_;
  unsigned steps_{0};

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  transport::NodePtr node_;
  transport::SubscriberPtr speedSub_;
  event::ConnectionPtr update_;
  double length_{6.0};
  double width_{0.9};
  double top_{0.7};
  double reach_{0.6};
  double speed_{0.3};
  double overrun_{0.3};
};

GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)
}  // namespace gazebo
