#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace h1_2_moveit_demo
{

class IncubatorLatchSystem final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    const gz::sim::Model model(entity);
    if (!model.Valid(ecm)) {
      gzerr << "IncubatorLatchSystem must be loaded on a model entity.\n";
      return;
    }

    const std::string door_joint_name =
      sdf->Get<std::string>("door_joint", "door_hinge_joint").first;
    const std::string handle_joint_name =
      sdf->Get<std::string>("handle_joint", "handle_latch_joint").first;
    unlock_angle_ = sdf->Get<double>("unlock_angle", 1.553343034275).first;
    relock_handle_angle_ =
      sdf->Get<double>("relock_handle_angle", 0.174532925199).first;
    relock_door_angle_ =
      sdf->Get<double>("relock_door_angle", 0.034906585040).first;
    handle_detent_torque_ =
      sdf->Get<double>("handle_detent_torque", 0.08).first;
    locked_handle_return_torque_ =
      sdf->Get<double>("locked_handle_return_torque", 0.40).first;
    handle_detent_damping_ =
      sdf->Get<double>("handle_detent_damping", 0.01).first;
    state_topic_ = sdf->Get<std::string>(
      "state_topic", "/incubator/latch/state").first;

    door_joint_.ResetEntity(model.JointByName(ecm, door_joint_name));
    handle_joint_.ResetEntity(model.JointByName(ecm, handle_joint_name));
    if (!door_joint_.Valid(ecm) || !handle_joint_.Valid(ecm)) {
      gzerr << "IncubatorLatchSystem could not resolve joints ["
            << door_joint_name << "] and [" << handle_joint_name << "].\n";
      return;
    }

    door_joint_.EnablePositionCheck(ecm);
    door_joint_.EnableVelocityCheck(ecm);
    handle_joint_.EnablePositionCheck(ecm);
    handle_joint_.EnableVelocityCheck(ecm);

    state_publisher_ = node_.Advertise<gz::msgs::StringMsg>(state_topic_);
    if (!state_publisher_) {
      gzerr << "IncubatorLatchSystem could not advertise ["
            << state_topic_ << "].\n";
      return;
    }

    locked_ = true;
    locked_return_enabled_ = false;
    configured_ = true;
    last_state_publish_ = std::chrono::steady_clock::now();
    PublishState("locked");
    gzmsg << "IncubatorLatchSystem locked door at 0 rad with handle down; "
          << "the handle must turn counterclockwise from the robot/front view to "
          << unlock_angle_
          << " rad (up) before the door hinge is released; "
          << "handle detent / locked return torque / damping are "
          << handle_detent_torque_ << " N m / "
          << locked_handle_return_torque_ << " N m / "
          << handle_detent_damping_ << " N m s per rad.\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!configured_ || info.paused) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_state_publish_ >= std::chrono::milliseconds(500)) {
      PublishState(locked_ ? "locked" : "unlocked");
      last_state_publish_ = now;
    }

    const auto door_position = door_joint_.Position(ecm);
    const auto handle_position = handle_joint_.Position(ecm);
    const auto handle_velocity = handle_joint_.Velocity(ecm);
    if (!door_position || door_position->empty() ||
      !handle_position || handle_position->empty())
    {
      if (locked_) {
        HoldDoorClosed(ecm);
      }
      return;
    }

    const double door_angle = door_position->front();
    const double handle_angle = handle_position->front();
    const double handle_speed =
      handle_velocity && !handle_velocity->empty() ? handle_velocity->front() : 0.0;
    // A smooth two-position detent has stable equilibria at q=0 and q=180 deg,
    // with the separatrix at q=90 deg.  It prevents the light handle from
    // falling under gravity after a small touch while remaining compliant to
    // a deliberate robot push through the midpoint.
    // The stronger return is deliberately limited to the existing relock
    // region *after* an unlocked -> locked transition.  The initially locked
    // handle therefore keeps the verified 0.08 N m dynamics over its entire
    // 0 -> 90 deg unlock motion; after door closure, the stronger return can
    // overcome static friction near q=0.
    const double effective_detent_torque =
      locked_ && locked_return_enabled_ &&
      handle_angle <= relock_handle_angle_ ?
      locked_handle_return_torque_ : handle_detent_torque_;
    const double handle_detent_force =
      -effective_detent_torque * std::sin(2.0 * handle_angle) -
      handle_detent_damping_ * handle_speed;
    handle_joint_.SetForce(ecm, std::vector<double>{handle_detent_force});
    if (locked_ && handle_angle >= unlock_angle_) {
      locked_ = false;
      PublishState("unlocked");
      last_state_publish_ = now;
      gzmsg << "Incubator latch unlocked at handle angle "
            << handle_angle << " rad.\n";
    } else if (!locked_ && door_angle <= relock_door_angle_ &&
      handle_angle <= relock_handle_angle_)
    {
      locked_ = true;
      locked_return_enabled_ = true;
      PublishState("locked");
      last_state_publish_ = now;
      gzmsg << "Incubator latch relocked with door angle "
            << door_angle << " rad and handle angle "
            << handle_angle << " rad.\n";
    }

    if (locked_) {
      HoldDoorClosed(ecm);
    }
  }

private:
  void HoldDoorClosed(gz::sim::EntityComponentManager & ecm)
  {
    door_joint_.ResetPosition(ecm, std::vector<double>{0.0});
    door_joint_.ResetVelocity(ecm, std::vector<double>{0.0});
  }

  void PublishState(const std::string & state)
  {
    gz::msgs::StringMsg message;
    message.set_data(state);
    state_publisher_.Publish(message);
  }

  gz::sim::Joint door_joint_;
  gz::sim::Joint handle_joint_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher state_publisher_;
  std::string state_topic_;
  double unlock_angle_{1.553343034275};
  double relock_handle_angle_{0.174532925199};
  double relock_door_angle_{0.034906585040};
  double handle_detent_torque_{0.08};
  double locked_handle_return_torque_{0.40};
  double handle_detent_damping_{0.01};
  bool locked_{true};
  bool locked_return_enabled_{false};
  bool configured_{false};
  std::chrono::steady_clock::time_point last_state_publish_{};
};

}  // namespace h1_2_moveit_demo

GZ_ADD_PLUGIN(
  h1_2_moveit_demo::IncubatorLatchSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  h1_2_moveit_demo::IncubatorLatchSystem,
  "h1_2::IncubatorLatchSystem")
