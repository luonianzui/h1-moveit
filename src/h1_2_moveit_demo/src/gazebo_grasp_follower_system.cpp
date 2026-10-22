#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace h1_2_moveit_demo
{

class GazeboCompliantGraspSystem final :
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
    child_model_entity_ = entity;
    const gz::sim::Model child_model(child_model_entity_);
    if (!child_model.Valid(ecm)) {
      gzerr << "GazeboCompliantGraspSystem must be loaded on a model entity.\n";
      return;
    }

    const std::string parent_scope =
      sdf->Get<std::string>("parent_scope", "h1_2::right_wrist_yaw_link").first;
    gravity_compensation_ =
      sdf->Get<bool>("gravity_compensation", true).first;
    const bool initially_attached =
      sdf->Get<bool>("initially_attached", true).first;
    detach_topic_ = sdf->Get<std::string>(
      "detach_topic", "/h1_2/right_pose_grasp/detach").first;
    attach_topic_ = sdf->Get<std::string>(
      "attach_topic", "/h1_2/right_pose_grasp/attach").first;
    state_topic_ = sdf->Get<std::string>(
      "output_topic", "/h1_2/right_pose_grasp/state").first;

    const std::unordered_set<gz::sim::Entity> candidates =
      gz::sim::entitiesFromScopedName(parent_scope, ecm);
    for (const auto candidate : candidates) {
      if (ecm.Component<gz::sim::components::Link>(candidate) != nullptr) {
        parent_link_entity_ = candidate;
        break;
      }
    }
    if (parent_link_entity_ == gz::sim::kNullEntity) {
      gzerr << "GazeboCompliantGraspSystem could not resolve parent link ["
            << parent_scope << "].\n";
      return;
    }

    child_pose_entity_ = child_model_entity_;
    if (sdf->HasElement("child_scope")) {
      child_scope_ = sdf->Get<std::string>("child_scope");
      if (child_scope_.empty()) {
        gzerr << "GazeboCompliantGraspSystem child_scope must not be empty.\n";
        return;
      }

      const std::unordered_set<gz::sim::Entity> child_candidates =
        gz::sim::entitiesFromScopedName(child_scope_, ecm);
      std::size_t link_candidate_count = 0U;
      for (const auto candidate : child_candidates) {
        if (ecm.Component<gz::sim::components::Link>(candidate) == nullptr) {
          continue;
        }
        child_link_entity_ = candidate;
        ++link_candidate_count;
      }
      if (link_candidate_count != 1U) {
        gzerr << "GazeboCompliantGraspSystem child_scope [" << child_scope_
              << "] must resolve to exactly one link, but resolved "
              << link_candidate_count << ".\n";
        return;
      }
      if (!IsDescendantOf(child_link_entity_, child_model_entity_, ecm)) {
        gzerr << "GazeboCompliantGraspSystem child_scope [" << child_scope_
              << "] is not a link within its host model.\n";
        return;
      }
      child_pose_entity_ = child_link_entity_;
    } else {
      child_link_entity_ = child_model.CanonicalLink(ecm);
      if (child_link_entity_ == gz::sim::kNullEntity) {
        gzerr << "GazeboCompliantGraspSystem could not resolve the child canonical link.\n";
        return;
      }
    }

    if (initially_attached) {
      CaptureCurrentRelativePose(ecm);
    }

    const gz::sim::Link child_link(child_link_entity_);
    const gz::sim::Link parent_link(parent_link_entity_);
    child_link.EnableVelocityChecks(ecm);
    parent_link.EnableVelocityChecks(ecm);
    if (const auto inertial = child_link.WorldInertial(ecm)) {
      child_mass_ = inertial->MassMatrix().Mass();
    }
    if (!std::isfinite(child_mass_) || child_mass_ <= 0.0) {
      gzerr << "GazeboCompliantGraspSystem child link mass must be positive and finite.\n";
      return;
    }
    maximum_force_ = sdf->Get<double>(
      "max_force", child_mass_ * kMaximumLinearAcceleration).first;
    maximum_torque_ = sdf->Get<double>(
      "max_torque", kDefaultMaximumTorque).first;
    if (!IsPositiveFinite(maximum_force_) || !IsPositiveFinite(maximum_torque_)) {
      gzerr << "GazeboCompliantGraspSystem max_force and max_torque must be "
            << "positive finite values.\n";
      return;
    }

    state_publisher_ = node_.Advertise<gz::msgs::StringMsg>(state_topic_);
    const bool detach_ok = node_.Subscribe(
      detach_topic_, &GazeboCompliantGraspSystem::OnDetach, this);
    const bool attach_ok = node_.Subscribe(
      attach_topic_, &GazeboCompliantGraspSystem::OnAttach, this);
    if (!state_publisher_ || !detach_ok || !attach_ok) {
      gzerr << "GazeboCompliantGraspSystem could not establish transport topics.\n";
      return;
    }

    attached_.store(initially_attached);
    configured_.store(true);
    requested_state_.store(
      initially_attached ? kPublishAttached : kPublishDetached);
    state_heartbeat_steps_ = 0U;
    if (initially_attached) {
      gzmsg << "GazeboCompliantGraspSystem attached independent model ["
            << child_model.Name(ecm) << "] to [" << parent_scope
            << "] using child body ["
            << (child_scope_.empty() ? "<model/canonical-link>" : child_scope_)
            << "], gravity compensation "
            << (gravity_compensation_ ? "enabled" : "disabled")
            << ", max force " << maximum_force_ << " N, max torque "
            << maximum_torque_
            << " N m, while preserving cross-model collisions.\n";
    } else {
      gzmsg << "GazeboCompliantGraspSystem preloaded independent model ["
            << child_model.Name(ecm) << "] detached from [" << parent_scope
            << "] using child body ["
            << (child_scope_.empty() ? "<model/canonical-link>" : child_scope_)
            << "]; no support wrench is active.\n";
    }
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!configured_.load()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(attachment_mutex_);
      if (attach_requested_) {
        // A preloaded detached plugin must bind the pose at the physical grasp,
        // not at Configure time.  Suppress all wrench application until this
        // simulation-thread ECM sample has established the new relative pose.
        CaptureCurrentRelativePose(ecm);
        attach_requested_ = false;
        attached_.store(true);
        requested_state_.store(kPublishAttached);
      }
    }

    if (attached_.load()) {
      const gz::math::Pose3d parent_world = gz::sim::worldPose(parent_link_entity_, ecm);
      const gz::math::Pose3d target_world = parent_world * parent_to_child_;
      const gz::math::Pose3d child_world = gz::sim::worldPose(child_pose_entity_, ecm);
      const gz::sim::Link parent_link(parent_link_entity_);
      const gz::sim::Link child_link(child_link_entity_);

      const gz::math::Vector3d parent_linear_velocity =
        parent_link.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
      const gz::math::Vector3d parent_angular_velocity =
        parent_link.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
      const gz::math::Vector3d child_linear_velocity =
        child_link.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
      const gz::math::Vector3d child_angular_velocity =
        child_link.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
      const gz::math::Vector3d world_offset =
        parent_world.Rot().RotateVector(parent_to_child_.Pos());
      const gz::math::Vector3d target_linear_velocity =
        parent_linear_velocity + parent_angular_velocity.Cross(world_offset);

      gz::math::Vector3d linear_acceleration =
        (target_world.Pos() - child_world.Pos()) * kLinearStiffness +
        (target_linear_velocity - child_linear_velocity) * kLinearDamping;
      if (gravity_compensation_) {
        linear_acceleration += gz::math::Vector3d(0.0, 0.0, 9.8);
      }
      LimitMagnitude(linear_acceleration, kMaximumLinearAcceleration);
      gz::math::Vector3d force = linear_acceleration * child_mass_;
      LimitMagnitude(force, maximum_force_);

      gz::math::Quaterniond rotation_error =
        (target_world.Rot() * child_world.Rot().Inverse()).Normalized();
      gz::math::Vector3d rotation_axis;
      double rotation_angle = 0.0;
      rotation_error.AxisAngle(rotation_axis, rotation_angle);
      if (rotation_angle > GZ_PI) {
        rotation_angle -= 2.0 * GZ_PI;
      }
      gz::math::Vector3d angular_acceleration =
        rotation_axis * (rotation_angle * kAngularStiffness) +
        (parent_angular_velocity - child_angular_velocity) * kAngularDamping;
      LimitMagnitude(angular_acceleration, kMaximumAngularAcceleration);
      gz::math::Vector3d torque = angular_acceleration * kFallbackRotationalInertia;
      if (const auto inertia = child_link.WorldInertiaMatrix(ecm)) {
        torque = (*inertia) * angular_acceleration;
      }
      LimitMagnitude(torque, maximum_torque_);
      child_link.AddWorldWrench(ecm, force, torque);
    }

    const int requested = requested_state_.exchange(kPublishNone);
    if (requested != kPublishNone) {
      // A transition acknowledgement always wins over the periodic heartbeat.
      // Resetting the counter also gives every fresh state a full quiet period.
      state_heartbeat_steps_ = 0U;
      PublishState(requested == kPublishAttached ? "attached" : "detached");
    } else {
      ++state_heartbeat_steps_;
      if (state_heartbeat_steps_ >= kStateHeartbeatPeriodSteps) {
        state_heartbeat_steps_ = 0U;
        PublishState(attached_.load() ? "attached" : "detached");
      }
    }
  }

private:
  void OnDetach(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> lock(attachment_mutex_);
    attach_requested_ = false;
    attached_.store(false);
    requested_state_.store(kPublishDetached);
  }

  void OnAttach(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> lock(attachment_mutex_);
    // Transport callbacks do not have an EntityComponentManager.  Defer the
    // actual attachment to PreUpdate and disable any stale wrench meanwhile.
    attached_.store(false);
    attach_requested_ = true;
  }

  void CaptureCurrentRelativePose(const gz::sim::EntityComponentManager & ecm)
  {
    const gz::math::Pose3d parent_world =
      gz::sim::worldPose(parent_link_entity_, ecm);
    const gz::math::Pose3d child_world =
      gz::sim::worldPose(child_pose_entity_, ecm);
    parent_to_child_ = parent_world.Inverse() * child_world;
  }

  void PublishState(const std::string & state)
  {
    gz::msgs::StringMsg message;
    message.set_data(state);
    state_publisher_.Publish(message);
  }

  static constexpr int kPublishNone = 0;
  static constexpr int kPublishAttached = 1;
  static constexpr int kPublishDetached = 2;
  static constexpr std::uint32_t kStateHeartbeatPeriodSteps = 250U;
  static constexpr double kLinearStiffness = 50000.0;
  static constexpr double kLinearDamping = 450.0;
  static constexpr double kAngularStiffness = 20000.0;
  static constexpr double kAngularDamping = 280.0;
  static constexpr double kMaximumLinearAcceleration = 1000.0;
  static constexpr double kMaximumAngularAcceleration = 4000.0;
  static constexpr double kDefaultMaximumTorque = 0.2;
  static constexpr double kFallbackRotationalInertia = 2.0e-5;

  static bool IsDescendantOf(
    const gz::sim::Entity entity,
    const gz::sim::Entity ancestor,
    const gz::sim::EntityComponentManager & ecm)
  {
    std::unordered_set<gz::sim::Entity> visited;
    gz::sim::Entity current = entity;
    while (current != gz::sim::kNullEntity && visited.insert(current).second) {
      if (current == ancestor) {
        return true;
      }
      current = ecm.ParentEntity(current);
    }
    return false;
  }

  static bool IsPositiveFinite(const double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  static void LimitMagnitude(gz::math::Vector3d & value, const double maximum)
  {
    const double magnitude = value.Length();
    if (magnitude > maximum && magnitude > 0.0) {
      value *= maximum / magnitude;
    }
  }

  gz::sim::Entity child_model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity child_pose_entity_{gz::sim::kNullEntity};
  gz::sim::Entity child_link_entity_{gz::sim::kNullEntity};
  gz::sim::Entity parent_link_entity_{gz::sim::kNullEntity};
  gz::math::Pose3d parent_to_child_;
  double child_mass_{0.012};
  double maximum_force_{12.0};
  double maximum_torque_{kDefaultMaximumTorque};
  bool gravity_compensation_{true};
  gz::transport::Node node_;
  gz::transport::Node::Publisher state_publisher_;
  std::string detach_topic_;
  std::string attach_topic_;
  std::string state_topic_;
  std::string child_scope_;
  std::mutex attachment_mutex_;
  bool attach_requested_{false};
  std::atomic<bool> configured_{false};
  std::atomic<bool> attached_{false};
  std::atomic<int> requested_state_{kPublishNone};
  std::uint32_t state_heartbeat_steps_{0U};
};

}  // namespace h1_2_moveit_demo

GZ_ADD_PLUGIN(
  h1_2_moveit_demo::GazeboCompliantGraspSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  h1_2_moveit_demo::GazeboCompliantGraspSystem,
  "h1_2::GazeboCompliantGraspSystem")
