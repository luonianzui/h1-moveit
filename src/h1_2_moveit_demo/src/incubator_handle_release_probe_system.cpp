#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
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

// Gazebo-only diagnostic.  It resets the passive handle once and never applies
// a force or position command afterwards.  The ordinary latch system, joint
// friction, damping and gravity are therefore the only dynamics during the
// measured release interval.
class IncubatorHandleReleaseProbeSystem final :
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
      gzerr << "IncubatorHandleReleaseProbeSystem must be loaded on a model.\n";
      return;
    }

    const std::string joint_name =
      sdf->Get<std::string>("handle_joint", "handle_latch_joint").first;
    target_angle_ = sdf->Get<double>("target_angle", 1.562069680535).first;
    unlock_angle_ = sdf->Get<double>("unlock_angle", 1.553343034275).first;
    duration_seconds_ = sdf->Get<double>("duration_seconds", 5.0).first;
    grace_seconds_ = sdf->Get<double>("grace_seconds", 0.5).first;
    maximum_drift_ = sdf->Get<double>("maximum_drift", 0.008726646260).first;
    maximum_speed_ = sdf->Get<double>("maximum_speed", 0.01).first;
    latch_topic_ = sdf->Get<std::string>(
      "latch_topic", "/incubator/latch/state").first;
    output_topic_ = sdf->Get<std::string>(
      "output_topic", "/h1_2/handle_release_probe/result").first;

    const bool numeric_ok =
      std::isfinite(target_angle_) && std::isfinite(unlock_angle_) &&
      std::isfinite(duration_seconds_) && std::isfinite(grace_seconds_) &&
      std::isfinite(maximum_drift_) && std::isfinite(maximum_speed_) &&
      target_angle_ >= unlock_angle_ && target_angle_ <= 3.141592653590 &&
      duration_seconds_ > 0.0 && grace_seconds_ >= 0.0 &&
      grace_seconds_ < duration_seconds_ && maximum_drift_ > 0.0 &&
      maximum_speed_ > 0.0;
    if (!numeric_ok || latch_topic_.empty() || output_topic_.empty()) {
      gzerr << "IncubatorHandleReleaseProbeSystem configuration is invalid.\n";
      return;
    }

    handle_joint_.ResetEntity(model.JointByName(ecm, joint_name));
    if (!handle_joint_.Valid(ecm)) {
      gzerr << "IncubatorHandleReleaseProbeSystem could not resolve joint ["
            << joint_name << "].\n";
      return;
    }
    handle_joint_.EnablePositionCheck(ecm);
    handle_joint_.EnableVelocityCheck(ecm);

    output_publisher_ = node_.Advertise<gz::msgs::StringMsg>(output_topic_);
    const bool latch_ok = node_.Subscribe(
      latch_topic_, &IncubatorHandleReleaseProbeSystem::OnLatchState, this);
    if (!output_publisher_ || !latch_ok) {
      gzerr << "IncubatorHandleReleaseProbeSystem transport setup failed.\n";
      return;
    }
    configured_ = true;
    gzmsg << "IncubatorHandleReleaseProbeSystem armed for a one-shot passive "
          << "release test at " << target_angle_ << " rad; no post-reset "
          << "joint force or position command will be applied.\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!configured_ || finished_ || info.paused) {
      return;
    }
    const double simulation_seconds =
      std::chrono::duration<double>(info.simTime).count();
    if (!reset_done_) {
      handle_joint_.ResetPosition(ecm, std::vector<double>{target_angle_});
      handle_joint_.ResetVelocity(ecm, std::vector<double>{0.0});
      started_at_seconds_ = simulation_seconds;
      reset_done_ = true;
      return;
    }

    const auto positions = handle_joint_.Position(ecm);
    const auto velocities = handle_joint_.Velocity(ecm);
    if (!positions || positions->size() != 1U ||
      !velocities || velocities->size() != 1U ||
      !std::isfinite(positions->front()) ||
      !std::isfinite(velocities->front()))
    {
      invalid_sample_ = true;
    } else {
      latest_position_ = positions->front();
      latest_velocity_ = velocities->front();
      const double elapsed = simulation_seconds - started_at_seconds_;
      if (elapsed >= grace_seconds_) {
        minimum_position_ = std::min(minimum_position_, latest_position_);
        maximum_position_ = std::max(maximum_position_, latest_position_);
        maximum_absolute_speed_ = std::max(
          maximum_absolute_speed_, std::abs(latest_velocity_));
        ++measured_samples_;
      }
    }

    if (simulation_seconds - started_at_seconds_ < duration_seconds_) {
      return;
    }
    const std::string latch = LatestLatchState();
    const double drift = measured_samples_ == 0U ?
      std::numeric_limits<double>::infinity() :
      std::max(
        std::abs(minimum_position_ - target_angle_),
        std::abs(maximum_position_ - target_angle_));
    const bool passed =
      !invalid_sample_ && measured_samples_ > 0U &&
      latch == "unlocked" &&
      minimum_position_ >= unlock_angle_ &&
      drift <= maximum_drift_ &&
      maximum_absolute_speed_ <= maximum_speed_;

    std::ostringstream payload;
    payload << std::boolalpha << std::setprecision(15)
            << "{\"schema\":\"h1_handle_release_probe/v1\","
            << "\"status\":\"" << (passed ? "PASS" : "FAIL") << "\","
            << "\"passive_after_one_shot_reset\":true,"
            << "\"controller_commands_issued\":0,"
            << "\"target_angle_rad\":" << target_angle_ << ','
            << "\"unlock_angle_rad\":" << unlock_angle_ << ','
            << "\"duration_sim_s\":" << duration_seconds_ << ','
            << "\"grace_sim_s\":" << grace_seconds_ << ','
            << "\"samples\":" << measured_samples_ << ','
            << "\"minimum_angle_rad\":" << minimum_position_ << ','
            << "\"maximum_angle_rad\":" << maximum_position_ << ','
            << "\"maximum_drift_rad\":" << drift << ','
            << "\"maximum_abs_speed_rad_s\":" << maximum_absolute_speed_ << ','
            << "\"final_angle_rad\":" << latest_position_ << ','
            << "\"final_speed_rad_s\":" << latest_velocity_ << ','
            << "\"latch_state\":\"" << latch << "\"}";
    gz::msgs::StringMsg message;
    message.set_data(payload.str());
    output_publisher_.Publish(message);
    if (passed) {
      gzmsg << "HANDLE_RELEASE_PROBE_RESULT=PASS " << payload.str() << "\n";
    } else {
      gzerr << "HANDLE_RELEASE_PROBE_RESULT=FAIL " << payload.str() << "\n";
    }
    finished_ = true;
  }

private:
  void OnLatchState(const gz::msgs::StringMsg & message)
  {
    std::lock_guard<std::mutex> lock(latch_mutex_);
    latch_state_ = message.data();
  }

  std::string LatestLatchState() const
  {
    std::lock_guard<std::mutex> lock(latch_mutex_);
    return latch_state_;
  }

  gz::sim::Joint handle_joint_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher output_publisher_;
  std::string latch_topic_;
  std::string output_topic_;
  mutable std::mutex latch_mutex_;
  std::string latch_state_{"unknown"};
  double target_angle_{1.562069680535};
  double unlock_angle_{1.553343034275};
  double duration_seconds_{5.0};
  double grace_seconds_{0.5};
  double maximum_drift_{0.008726646260};
  double maximum_speed_{0.01};
  double started_at_seconds_{0.0};
  double minimum_position_{std::numeric_limits<double>::infinity()};
  double maximum_position_{-std::numeric_limits<double>::infinity()};
  double maximum_absolute_speed_{0.0};
  double latest_position_{std::numeric_limits<double>::quiet_NaN()};
  double latest_velocity_{std::numeric_limits<double>::quiet_NaN()};
  std::size_t measured_samples_{0U};
  bool configured_{false};
  bool reset_done_{false};
  bool invalid_sample_{false};
  bool finished_{false};
};

}  // namespace h1_2_moveit_demo

GZ_ADD_PLUGIN(
  h1_2_moveit_demo::IncubatorHandleReleaseProbeSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  h1_2_moveit_demo::IncubatorHandleReleaseProbeSystem,
  "h1_2::IncubatorHandleReleaseProbeSystem")
