// Gazebo Classic validation plugin: collect only Husky contacts. Subscribing to
// all warehouse contacts serializes hundreds of resting boxes and slows physics.
// Build with -shared -fPIC and pkg-config --cflags --libs gazebo, then add a
// plugin pointing at the resulting .so to a temporary test world's <world>.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>
#include <set>
#include <string>
#include <vector>

namespace gazebo {
class Nav2ContactMonitor : public WorldPlugin {
  physics::WorldPtr world;
  event::ConnectionPtr update;
  transport::NodePtr node;
  transport::SubscriberPtr sub;
  std::set<std::string> observed;
  unsigned long messages = 0, robot_contacts = 0, obstacle_contacts = 0;

  void Contacts(ConstContactsPtr &msg) {
    for (int i = 0; i < msg->contact_size(); ++i) {
      const auto &c = msg->contact(i);
      const auto a = c.collision1(), b = c.collision2();
      const bool ha = a.find("husky::") == 0, hb = b.find("husky::") == 0;
      if (ha == hb) continue;
      ++robot_contacts;
      const auto other = ha ? b : a;
      const bool floor = other.find("ground_plane::") == 0 ||
        (other.size() >= 7 && other.substr(other.size()-7) == "::floor");
      if (!floor) ++obstacle_contacts;
      const auto pair = a + " | " + b;
      if (observed.insert(pair).second)
        std::cout << "NAV2_CONTACT " << (floor ? "GROUND " : "OBSTACLE ")
                  << c.time().sec() << "." << c.time().nsec() << " " << pair << std::endl;
    }
    if (++messages % 5000 == 0)
      std::cout << "NAV2_CONTACT COUNTS messages=" << messages << " robot_contacts="
                << robot_contacts << " obstacle_contacts=" << obstacle_contacts << std::endl;
  }

  void Update() {
    if (sub) return;
    auto robot = world->ModelByName("husky");
    if (!robot) return;
    std::vector<std::string> names;
    for (auto link : robot->GetLinks())
      for (auto collision : link->GetCollisions()) names.push_back(collision->GetScopedName());
    const auto topic = world->Physics()->GetContactManager()->CreateFilter("nav2_validation", names);
    sub = node->Subscribe(topic, &Nav2ContactMonitor::Contacts, this);
    std::cout << "NAV2_CONTACT READY collisions=" << names.size() << std::endl;
  }

public:
  void Load(physics::WorldPtr w, sdf::ElementPtr) override {
    world = w;
    node.reset(new transport::Node());
    node->Init(world->Name());
    update = event::Events::ConnectWorldUpdateBegin(std::bind(&Nav2ContactMonitor::Update, this));
  }
};
GZ_REGISTER_WORLD_PLUGIN(Nav2ContactMonitor)
}
