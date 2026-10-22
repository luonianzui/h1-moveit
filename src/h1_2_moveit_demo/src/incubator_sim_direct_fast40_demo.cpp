#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_plugin_v.pb.h>
#include <gz/msgs/joint_trajectory.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

namespace
{
using namespace std::chrono_literals;

constexpr char kJointStateTopic[] = "/world/h1_world/model/incubator/joint_state";
constexpr char kDynamicPoseTopic[] = "/world/h1_world/dynamic_pose/info";
constexpr char kLatchStateTopic[] = "/incubator/latch/state";
constexpr char kWorldStatsTopic[] = "/world/h1_world/stats";
constexpr char kSystemAddService[] = "/world/h1_world/entity/system/add";
constexpr char kCommandTopic[] =
  "/h1_2/incubator/simulation_direct_joint_trajectory";
constexpr char kIncubatorModelName[] = "incubator";
constexpr char kHandleJointName[] = "handle_latch_joint";
constexpr char kDoorJointName[] = "door_hinge_joint";

constexpr double kPi = 3.14159265358979323846;
// A 2 deg command compensation overcomes the measured low-gain tracking
// offset.  The physical feedback gate still requires a fresh unlocked latch;
// the observed handle endpoint is expected to remain near 90 deg.
constexpr double kHandleTarget = 92.0 * kPi / 180.0;
constexpr double kDoorTarget40 = 40.0 * kPi / 180.0;
constexpr double kDoorTarget60 = 60.0 * kPi / 180.0;
// Gazebo's JointTrajectoryController changes set-points at the stamped times;
// it does not provide the continuous spline semantics of ros2_control.  Dense
// targets therefore bound the commanded rate below the unchanged SDF limits:
// handle 1 deg / 0.06 sim-s = 0.291 rad/s < 0.35 rad/s, and door
// 0.1 deg / 0.01 sim-s = 0.175 rad/s < 0.5 rad/s.
constexpr double kHandleStepDegrees = 1.0;
constexpr double kHandleStepSeconds = 0.06;
constexpr double kDoorStepDegrees = 0.1;
constexpr double kDoorStepSeconds = 0.01;
constexpr double kFirstPointSeconds = 0.1;
constexpr double kMaximumWallSeconds = 120.0;
constexpr std::size_t kRequiredRealTimeFactorSamples = 5U;
constexpr double kMinimumRealTimeFactor = 0.18;

double degrees(const double radians)
{
  return radians * 180.0 / kPi;
}

std::string xml_number(const double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(12) << value;
  return stream.str();
}

void set_point_time(gz::msgs::JointTrajectoryPoint * point, const double seconds)
{
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  const auto total_nanoseconds = static_cast<std::int64_t>(
    std::llround(seconds * static_cast<double>(kNanosecondsPerSecond)));
  const auto whole_seconds = total_nanoseconds / kNanosecondsPerSecond;
  const auto nanoseconds = static_cast<std::int32_t>(
    total_nanoseconds % kNanosecondsPerSecond);
  point->mutable_time_from_start()->set_sec(whole_seconds);
  point->mutable_time_from_start()->set_nsec(nanoseconds);
}

struct FeedbackSnapshot
{
  std::uint64_t incubator_entity_id = 0U;
  double door_angle = std::numeric_limits<double>::quiet_NaN();
  double handle_angle = std::numeric_limits<double>::quiet_NaN();
  double door_velocity = std::numeric_limits<double>::quiet_NaN();
  double handle_velocity = std::numeric_limits<double>::quiet_NaN();
  double real_time_factor = std::numeric_limits<double>::quiet_NaN();
  double minimum_recent_real_time_factor =
    std::numeric_limits<double>::quiet_NaN();
  std::size_t real_time_factor_samples = 0U;
  double simulation_seconds = std::numeric_limits<double>::quiet_NaN();
  std::string latch_state;
  double joint_age_seconds = std::numeric_limits<double>::infinity();
  double pose_age_seconds = std::numeric_limits<double>::infinity();
  double stats_age_seconds = std::numeric_limits<double>::infinity();
  double latch_age_seconds = std::numeric_limits<double>::infinity();
};

class GazeboFeedback
{
public:
  bool start()
  {
    const bool joint_ok = transport_.Subscribe(
      kJointStateTopic, &GazeboFeedback::joint_callback, this);
    const bool pose_ok = transport_.Subscribe(
      kDynamicPoseTopic, &GazeboFeedback::pose_callback, this);
    const bool latch_ok = transport_.Subscribe(
      kLatchStateTopic, &GazeboFeedback::latch_callback, this);
    const bool stats_ok = transport_.Subscribe(
      kWorldStatsTopic, &GazeboFeedback::stats_callback, this);
    return joint_ok && pose_ok && latch_ok && stats_ok;
  }

  std::optional<FeedbackSnapshot> snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!incubator_entity_id_.has_value() || !door_angle_.has_value() ||
      !handle_angle_.has_value() || !real_time_factor_.has_value() ||
      !simulation_seconds_.has_value())
    {
      return std::nullopt;
    }

    const auto now = std::chrono::steady_clock::now();
    FeedbackSnapshot result;
    result.incubator_entity_id = *incubator_entity_id_;
    result.door_angle = *door_angle_;
    result.handle_angle = *handle_angle_;
    result.door_velocity = door_velocity_.value_or(
      std::numeric_limits<double>::quiet_NaN());
    result.handle_velocity = handle_velocity_.value_or(
      std::numeric_limits<double>::quiet_NaN());
    result.real_time_factor = *real_time_factor_;
    result.real_time_factor_samples = recent_real_time_factors_.size();
    if (!recent_real_time_factors_.empty()) {
      result.minimum_recent_real_time_factor = *std::min_element(
        recent_real_time_factors_.begin(), recent_real_time_factors_.end());
    }
    result.simulation_seconds = *simulation_seconds_;
    result.latch_state = latch_state_.value_or("unknown");
    result.joint_age_seconds =
      std::chrono::duration<double>(now - joint_received_at_).count();
    result.pose_age_seconds =
      std::chrono::duration<double>(now - pose_received_at_).count();
    result.stats_age_seconds =
      std::chrono::duration<double>(now - stats_received_at_).count();
    result.latch_age_seconds =
      std::chrono::duration<double>(now - latch_received_at_).count();
    return result;
  }

  gz::transport::Node & transport()
  {
    return transport_;
  }

private:
  void joint_callback(const gz::msgs::Model & model)
  {
    std::optional<double> door_angle;
    std::optional<double> handle_angle;
    std::optional<double> door_velocity;
    std::optional<double> handle_velocity;
    for (const auto & joint : model.joint()) {
      if (!joint.has_axis1()) {
        continue;
      }
      if (joint.name() == kDoorJointName) {
        door_angle = joint.axis1().position();
        door_velocity = joint.axis1().velocity();
      } else if (joint.name() == kHandleJointName) {
        handle_angle = joint.axis1().position();
        handle_velocity = joint.axis1().velocity();
      }
    }
    if (!door_angle.has_value() || !handle_angle.has_value()) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    door_angle_ = *door_angle;
    handle_angle_ = *handle_angle;
    door_velocity_ = door_velocity;
    handle_velocity_ = handle_velocity;
    joint_received_at_ = std::chrono::steady_clock::now();
  }

  void pose_callback(const gz::msgs::Pose_V & poses)
  {
    for (const auto & pose : poses.pose()) {
      if (pose.name() != kIncubatorModelName) {
        continue;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      incubator_entity_id_ = pose.id();
      pose_received_at_ = std::chrono::steady_clock::now();
      return;
    }
  }

  void latch_callback(const gz::msgs::StringMsg & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latch_state_ = message.data();
    latch_received_at_ = std::chrono::steady_clock::now();
  }

  void stats_callback(const gz::msgs::WorldStatistics & message)
  {
    if (!message.has_sim_time()) {
      return;
    }
    const double simulation_seconds =
      static_cast<double>(message.sim_time().sec()) +
      static_cast<double>(message.sim_time().nsec()) * 1.0e-9;
    std::lock_guard<std::mutex> lock(mutex_);
    real_time_factor_ = message.real_time_factor();
    if (std::isfinite(message.real_time_factor()) &&
      message.real_time_factor() > 0.0)
    {
      recent_real_time_factors_.push_back(message.real_time_factor());
      while (recent_real_time_factors_.size() >
        kRequiredRealTimeFactorSamples)
      {
        recent_real_time_factors_.pop_front();
      }
    } else {
      recent_real_time_factors_.clear();
    }
    simulation_seconds_ = simulation_seconds;
    stats_received_at_ = std::chrono::steady_clock::now();
  }

  gz::transport::Node transport_;
  mutable std::mutex mutex_;
  std::optional<std::uint64_t> incubator_entity_id_;
  std::optional<double> door_angle_;
  std::optional<double> handle_angle_;
  std::optional<double> door_velocity_;
  std::optional<double> handle_velocity_;
  std::optional<double> real_time_factor_;
  std::optional<double> simulation_seconds_;
  std::optional<std::string> latch_state_;
  std::deque<double> recent_real_time_factors_;
  std::chrono::steady_clock::time_point joint_received_at_{};
  std::chrono::steady_clock::time_point pose_received_at_{};
  std::chrono::steady_clock::time_point stats_received_at_{};
  std::chrono::steady_clock::time_point latch_received_at_{};
};

bool feedback_is_fresh(const FeedbackSnapshot & state)
{
  return state.joint_age_seconds <= 1.0 && state.pose_age_seconds <= 1.0 &&
         state.stats_age_seconds <= 1.0 && state.latch_age_seconds <= 1.0 &&
         std::isfinite(state.door_angle) &&
         std::isfinite(state.handle_angle) &&
         std::isfinite(state.real_time_factor) &&
         state.real_time_factor > 0.0;
}

std::optional<FeedbackSnapshot> wait_for_fresh_feedback(
  const GazeboFeedback & feedback, const std::chrono::seconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const auto state = feedback.snapshot();
    if (state.has_value() && feedback_is_fresh(*state) &&
      state->real_time_factor_samples >= kRequiredRealTimeFactorSamples &&
      std::isfinite(state->minimum_recent_real_time_factor) &&
      state->minimum_recent_real_time_factor >= kMinimumRealTimeFactor)
    {
      return state;
    }
    std::this_thread::sleep_for(20ms);
  }
  return std::nullopt;
}

bool add_direct_joint_controller(
  gz::transport::Node & transport, const FeedbackSnapshot & initial,
  const rclcpp::Logger & logger)
{
  gz::msgs::EntityPlugin_V request;
  auto * entity = request.mutable_entity();
  entity->set_id(initial.incubator_entity_id);
  entity->set_name(kIncubatorModelName);
  entity->set_type(gz::msgs::Entity::MODEL);

  auto * plugin = request.add_plugins();
  plugin->set_name("gz::sim::systems::JointTrajectoryController");
  plugin->set_filename("gz-sim-joint-trajectory-controller-system");
  plugin->set_innerxml(
    std::string("<topic>") + kCommandTopic + "</topic>" +
    "<joint_name>" + kHandleJointName + "</joint_name>" +
    "<initial_position>" + xml_number(initial.handle_angle) +
    "</initial_position>" +
    "<position_p_gain>2</position_p_gain>" +
    "<position_i_gain>0</position_i_gain>" +
    "<position_d_gain>0.05</position_d_gain>" +
    "<position_cmd_min>-5</position_cmd_min>" +
    "<position_cmd_max>5</position_cmd_max>" +
    "<joint_name>" + kDoorJointName + "</joint_name>" +
    "<initial_position>" + xml_number(initial.door_angle) +
    "</initial_position>" +
    "<position_p_gain>20</position_p_gain>" +
    "<position_i_gain>0</position_i_gain>" +
    "<position_d_gain>1</position_d_gain>" +
    "<position_cmd_min>-50</position_cmd_min>" +
    "<position_cmd_max>50</position_cmd_max>");

  gz::msgs::Boolean reply;
  bool service_result = false;
  const bool completed = transport.Request(
    kSystemAddService, request, 10000u, reply, service_result);
  if (!completed || !service_result || !reply.data()) {
    RCLCPP_ERROR(
      logger,
      "Gazebo rejected the runtime direct-joint controller "
      "(completed=%d, result=%d, reply=%d).",
      completed, service_result, reply.data());
    return false;
  }
  return true;
}

gz::msgs::JointTrajectory make_dense_handle_trajectory(
  const double initial_handle, const double initial_door)
{
  gz::msgs::JointTrajectory trajectory;
  trajectory.add_joint_names(kHandleJointName);
  trajectory.add_joint_names(kDoorJointName);

  auto * initial_point = trajectory.add_points();
  initial_point->add_positions(initial_handle);
  initial_point->add_positions(initial_door);
  set_point_time(initial_point, kFirstPointSeconds);

  const int step_count = std::max(
    1, static_cast<int>(std::ceil(
      std::abs(degrees(kHandleTarget - initial_handle)) /
      kHandleStepDegrees)));
  for (int step = 1; step <= step_count; ++step) {
    const double fraction =
      static_cast<double>(step) / static_cast<double>(step_count);
    auto * point = trajectory.add_points();
    point->add_positions(
      initial_handle + fraction * (kHandleTarget - initial_handle));
    point->add_positions(initial_door);
    set_point_time(
      point, kFirstPointSeconds +
      static_cast<double>(step) * kHandleStepSeconds);
  }
  return trajectory;
}

gz::msgs::JointTrajectory make_dense_door_trajectory(
  const double held_handle, const double initial_door, const double target_door)
{
  gz::msgs::JointTrajectory trajectory;
  trajectory.add_joint_names(kHandleJointName);
  trajectory.add_joint_names(kDoorJointName);

  auto * initial_point = trajectory.add_points();
  initial_point->add_positions(held_handle);
  initial_point->add_positions(initial_door);
  set_point_time(initial_point, kFirstPointSeconds);

  const int step_count = std::max(
    1, static_cast<int>(std::ceil(
      std::abs(degrees(target_door - initial_door)) / kDoorStepDegrees)));
  for (int step = 1; step <= step_count; ++step) {
    const double fraction =
      static_cast<double>(step) / static_cast<double>(step_count);
    auto * point = trajectory.add_points();
    point->add_positions(held_handle);
    point->add_positions(
      initial_door + fraction * (target_door - initial_door));
    set_point_time(
      point, kFirstPointSeconds +
      static_cast<double>(step) * kDoorStepSeconds);
  }
  return trajectory;
}

gz::msgs::JointTrajectory make_hold_trajectory(
  const double handle, const double door)
{
  gz::msgs::JointTrajectory trajectory;
  trajectory.add_joint_names(kHandleJointName);
  trajectory.add_joint_names(kDoorJointName);
  auto * point = trajectory.add_points();
  point->add_positions(handle);
  point->add_positions(door);
  set_point_time(point, kFirstPointSeconds);
  return trajectory;
}

double trajectory_duration_seconds(const gz::msgs::JointTrajectory & trajectory)
{
  if (trajectory.points().empty()) {
    return 0.0;
  }
  const auto & duration = trajectory.points().rbegin()->time_from_start();
  return static_cast<double>(duration.sec()) +
         static_cast<double>(duration.nsec()) * 1.0e-9;
}

bool latch_is_unlocked(const FeedbackSnapshot & state)
{
  return state.latch_state.find("unlocked") != std::string::npos;
}

int fail(const rclcpp::Logger & logger, const std::string & reason)
{
  RCLCPP_ERROR(logger, "SIMULATION_DIRECT_JOINT_FAST40_RESULT=FAIL: %s", reason.c_str());
  RCLCPP_ERROR(
    logger,
    "This was an uncertified Gazebo experiment; no candidate-3 robot path "
    "was calculated or executed.");
  rclcpp::shutdown();
  return 1;
}

int fail_after_controller_added(
  const rclcpp::Logger & logger,
  gz::transport::Node::Publisher & publisher,
  const GazeboFeedback & feedback,
  const std::string & reason)
{
  const auto state = feedback.snapshot();
  if (state.has_value() && feedback_is_fresh(*state) && publisher.HasConnections()) {
    const auto hold = make_hold_trajectory(state->handle_angle, state->door_angle);
    if (publisher.Publish(hold)) {
      RCLCPP_ERROR(
        logger,
        "Post-add failure: a Gazebo hold at the latest measured handle/door "
        "state was requested.");
    }
  }
  RCLCPP_ERROR(
    logger,
    "The runtime Gazebo controller remains loaded. Restart Gazebo before "
    "retrying or performing later contact validation.");
  return fail(logger, reason);
}

int fail_fast60_continuation(
  const rclcpp::Logger & logger, const std::string & reason)
{
  RCLCPP_ERROR(
    logger, "SIMULATION_DIRECT_JOINT_FAST60_CONTINUATION_RESULT=FAIL: %s",
    reason.c_str());
  RCLCPP_ERROR(
    logger,
    "This was an uncertified Gazebo continuation; no candidate-3 robot path "
    "was calculated or executed.");
  rclcpp::shutdown();
  return 1;
}

int fail_fast60_after_controller(
  const rclcpp::Logger & logger,
  gz::transport::Node::Publisher & publisher,
  const GazeboFeedback & feedback,
  const std::string & reason)
{
  const auto state = feedback.snapshot();
  if (state.has_value() && feedback_is_fresh(*state) && publisher.HasConnections()) {
    const auto hold = make_hold_trajectory(state->handle_angle, state->door_angle);
    if (publisher.Publish(hold)) {
      RCLCPP_ERROR(
        logger,
        "Fast60 continuation failure: a Gazebo hold at the latest measured "
        "handle/door state was requested.");
    }
  }
  RCLCPP_ERROR(
    logger,
    "The existing runtime Gazebo controller remains loaded. Restart Gazebo "
    "before retrying or performing contact validation.");
  return fail_fast60_continuation(logger, reason);
}

int run_fast60_continuation(
  const rclcpp::Logger & logger, GazeboFeedback & feedback,
  const FeedbackSnapshot & initial)
{
  constexpr double kDoorStartTolerance = 0.75 * kPi / 180.0;
  constexpr double kHandleTolerance = 1.5 * kPi / 180.0;
  if (std::abs(initial.door_angle - kDoorTarget40) > kDoorStartTolerance) {
    return fail_fast60_continuation(
      logger, "door must start at the stable 40 deg direct-joint endpoint");
  }
  if (std::abs(initial.handle_angle - kHandleTarget) > kHandleTolerance ||
    !latch_is_unlocked(initial))
  {
    return fail_fast60_continuation(
      logger, "fresh compensated handle plus unlocked latch start gate failed");
  }
  if (!std::isfinite(initial.handle_velocity) ||
    !std::isfinite(initial.door_velocity) ||
    std::abs(initial.handle_velocity) > 0.02 ||
    std::abs(initial.door_velocity) > 0.01)
  {
    return fail_fast60_continuation(
      logger, "handle and door must be stationary before the 40-to-60 continuation");
  }

  auto publisher = feedback.transport().Advertise<gz::msgs::JointTrajectory>(
    kCommandTopic);
  if (!publisher) {
    return fail_fast60_continuation(
      logger, "could not advertise the existing Gazebo-only command topic");
  }
  const auto connection_deadline = std::chrono::steady_clock::now() + 5s;
  while (rclcpp::ok() && !publisher.HasConnections() &&
    std::chrono::steady_clock::now() < connection_deadline)
  {
    std::this_thread::sleep_for(20ms);
  }
  if (!publisher.HasConnections()) {
    return fail_fast60_continuation(
      logger,
      "the prior fast40 Gazebo controller is absent; no new controller was added");
  }

  const auto before_command = feedback.snapshot();
  if (!before_command.has_value() || !feedback_is_fresh(*before_command) ||
    std::abs(before_command->door_angle - kDoorTarget40) > kDoorStartTolerance ||
    std::abs(before_command->handle_angle - kHandleTarget) > kHandleTolerance ||
    !std::isfinite(before_command->handle_velocity) ||
    !std::isfinite(before_command->door_velocity) ||
    std::abs(before_command->handle_velocity) > 0.02 ||
    std::abs(before_command->door_velocity) > 0.01 ||
    !latch_is_unlocked(*before_command))
  {
    return fail_fast60_after_controller(
      logger, publisher, feedback,
      "fresh stationary q40/unlocked gate changed before publication");
  }

  const auto trajectory = make_dense_door_trajectory(
    kHandleTarget, before_command->door_angle, kDoorTarget60);
  const auto started_at = std::chrono::steady_clock::now();
  const double started_simulation_seconds = before_command->simulation_seconds;
  if (!publisher.Publish(trajectory)) {
    return fail_fast60_after_controller(
      logger, publisher, feedback,
      "Gazebo dense 40-to-60 door trajectory publication failed");
  }
  RCLCPP_WARN(
    logger,
    "Gazebo-only DOOR CONTINUATION started using the existing controller: "
    "%d dense targets, %.3f -> 60 deg, duration %.3f sim-s. The compensated "
    "handle set-point remains 92 deg and no ROS robot trajectory is used.",
    trajectory.points_size(), degrees(before_command->door_angle),
    trajectory_duration_seconds(trajectory));

  FeedbackSnapshot latest_state = *before_command;
  std::optional<std::chrono::steady_clock::time_point> stable_since;
  auto next_progress = started_at;
  while (true) {
    if (!rclcpp::ok()) {
      return fail_fast60_after_controller(
        logger, publisher, feedback, "interrupted during the 40-to-60 continuation");
    }
    const auto now = std::chrono::steady_clock::now();
    const double wall_seconds =
      std::chrono::duration<double>(now - started_at).count();
    if (wall_seconds >= kMaximumWallSeconds) {
      return fail_fast60_after_controller(
        logger, publisher, feedback,
        "60 deg stable feedback was not reached within 120 wall seconds");
    }

    const auto state = feedback.snapshot();
    if (!state.has_value() || !feedback_is_fresh(*state)) {
      stable_since.reset();
      std::this_thread::sleep_for(20ms);
      continue;
    }
    latest_state = *state;
    const bool angle_ok =
      std::abs(state->handle_angle - kHandleTarget) <= kHandleTolerance &&
      std::abs(state->door_angle - kDoorTarget60) <= kDoorStartTolerance &&
      latch_is_unlocked(*state);
    const bool velocity_ok =
      std::isfinite(state->handle_velocity) &&
      std::isfinite(state->door_velocity) &&
      std::abs(state->handle_velocity) <= 0.02 &&
      std::abs(state->door_velocity) <= 0.01;
    if (angle_ok && velocity_ok) {
      if (!stable_since.has_value()) {
        stable_since = now;
      }
      if (std::chrono::duration<double>(now - *stable_since).count() >= 2.0) {
        break;
      }
    } else {
      stable_since.reset();
    }

    if (now >= next_progress) {
      RCLCPP_INFO(
        logger,
        "FAST60 progress wall=%.1f s sim=%.1f s handle=%.2f deg door=%.2f "
        "deg latch='%s' RTF=%.3f.",
        wall_seconds,
        state->simulation_seconds - started_simulation_seconds,
        degrees(state->handle_angle), degrees(state->door_angle),
        state->latch_state.c_str(), state->real_time_factor);
      next_progress = now + 2s;
    }
    std::this_thread::sleep_for(20ms);
  }

  const double wall_seconds = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started_at).count();
  const double simulation_seconds =
    latest_state.simulation_seconds - started_simulation_seconds;
  RCLCPP_INFO(
    logger,
    "SIMULATION_DIRECT_JOINT_FAST60_CONTINUATION_RESULT=PASS wall=%.3f s "
    "sim=%.3f s handle=%.4f deg door=%.4f deg handle_vel=%.6f rad/s "
    "door_vel=%.6f rad/s latch='%s'.",
    wall_seconds, simulation_seconds, degrees(latest_state.handle_angle),
    degrees(latest_state.door_angle), latest_state.handle_velocity,
    latest_state.door_velocity, latest_state.latch_state.c_str());
  RCLCPP_WARN(
    logger,
    "Candidate-3 parameter gate only: no candidate-3 robot path was calculated "
    "or executed. This is not a collision/contact/safety certificate. The "
    "existing runtime controller remains loaded to hold the visible 60 deg state.");
  rclcpp::shutdown();
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("incubator_sim_direct_fast40_demo");
  const auto logger = node->get_logger();

  const bool enabled = node->declare_parameter<bool>(
    "door_handoff_simulation_direct_joint_fast40", false);
  const bool continue_to_60 = node->declare_parameter<bool>(
    "door_handoff_simulation_direct_joint_fast60_continuation", false);
  const int only_candidate = node->declare_parameter<int>(
    "door_handoff_only_candidate", 0);
  bool use_sim_time = false;
  if (!node->get_parameter("use_sim_time", use_sim_time)) {
    use_sim_time = node->declare_parameter<bool>("use_sim_time", false);
  }

  RCLCPP_WARN(
    logger,
    "SIMULATION-ONLY EXPERIMENT: candidate-3 parameter gate only; no "
    "candidate-3 robot path will be calculated or executed; this is not a "
    "collision/contact/safety certificate.");
  const auto fail_current_mode = [&](const std::string & reason) {
      return continue_to_60 ?
             fail_fast60_continuation(logger, reason) : fail(logger, reason);
    };
  if (enabled == continue_to_60) {
    return fail_current_mode(
      "exactly one fast40 or fast60-continuation opt-in must be true");
  }
  if (!use_sim_time) {
    return fail_current_mode("use_sim_time must be true");
  }
  if (only_candidate != 3) {
    return fail_current_mode("door_handoff_only_candidate must equal 3");
  }

  std::size_t direct_demo_node_count = 0U;
  for (const auto & name : node->get_node_names()) {
    if (name.find("incubator_door_open_demo") != std::string::npos) {
      return fail_current_mode("another incubator_door_open_demo node is active");
    }
    if (name.find("incubator_sim_direct_fast40_demo") != std::string::npos) {
      ++direct_demo_node_count;
    }
  }
  if (direct_demo_node_count > 1U) {
    return fail_current_mode("another simulation-direct node is active");
  }

  GazeboFeedback feedback;
  if (!feedback.start()) {
    return fail_current_mode(
      "could not subscribe to all required Gazebo feedback topics");
  }
  const auto initial = wait_for_fresh_feedback(feedback, 5s);
  if (!initial.has_value()) {
    return fail_current_mode(
      "fresh incubator/latch/world feedback with a stable five-sample RTF "
      "window was not available");
  }

  if (continue_to_60) {
    return run_fast60_continuation(logger, feedback, *initial);
  }

  constexpr double kStartTolerance = 2.0 * kPi / 180.0;
  const bool handle_is_down = std::abs(initial->handle_angle) <= kStartTolerance;
  const bool handle_is_up =
    std::abs(initial->handle_angle - kHandleTarget) <= kStartTolerance;
  if (std::abs(initial->door_angle) > kStartTolerance) {
    return fail(
      logger,
      "door must start near 0 deg; restart Gazebo instead of resetting a live scene");
  }
  if (!handle_is_down && !handle_is_up) {
    return fail(logger, "handle must start near 0 deg or the compensated unlocked endpoint");
  }
  if (handle_is_up && !latch_is_unlocked(*initial)) {
    return fail(logger,
      "compensated handle feedback is present but the fresh latch is not unlocked");
  }
  if (!std::isfinite(initial->minimum_recent_real_time_factor) ||
    initial->minimum_recent_real_time_factor < kMinimumRealTimeFactor)
  {
    return fail(
      logger,
      "five-sample Gazebo real-time-factor window is below the 0.18 timing gate");
  }

  RCLCPP_INFO(
    logger,
    "Start gate PASS: entity=%llu door=%.3f deg handle=%.3f deg latch='%s' "
    "RTF latest/min5=%.3f/%.3f. No ROS controller trajectory publisher exists "
    "in this executable.",
    static_cast<unsigned long long>(initial->incubator_entity_id),
    degrees(initial->door_angle), degrees(initial->handle_angle),
    initial->latch_state.c_str(), initial->real_time_factor,
    initial->minimum_recent_real_time_factor);

  auto publisher = feedback.transport().Advertise<gz::msgs::JointTrajectory>(
    kCommandTopic);
  if (!publisher) {
    return fail(logger, "could not advertise the Gazebo-only joint trajectory topic");
  }

  // A subscriber on this private topic can only be a controller left in the
  // running Gazebo process by an earlier attempt.  Refuse to stack another
  // force controller on the same two joints.
  const auto existing_controller_deadline =
    std::chrono::steady_clock::now() + 1s;
  while (rclcpp::ok() && std::chrono::steady_clock::now() <
    existing_controller_deadline)
  {
    if (publisher.HasConnections()) {
      return fail(
        logger,
        "an existing simulation-direct controller is already subscribed; "
        "restart Gazebo before retrying");
    }
    std::this_thread::sleep_for(20ms);
  }
  if (!rclcpp::ok()) {
    return fail(logger, "interrupted before adding the Gazebo controller");
  }
  if (publisher.HasConnections()) {
    return fail(
      logger,
      "an existing simulation-direct controller appeared at the end of the "
      "subscriber probe; restart Gazebo before retrying");
  }
  direct_demo_node_count = 0U;
  for (const auto & name : node->get_node_names()) {
    if (name.find("incubator_door_open_demo") != std::string::npos) {
      return fail(
        logger,
        "an incubator_door_open_demo node appeared during the add probe");
    }
    if (name.find("incubator_sim_direct_fast40_demo") != std::string::npos) {
      ++direct_demo_node_count;
    }
  }
  if (direct_demo_node_count > 1U) {
    return fail(
      logger,
      "another simulation-direct fast40 node appeared during the add probe");
  }

  if (!add_direct_joint_controller(feedback.transport(), *initial, logger)) {
    RCLCPP_ERROR(
      logger,
      "The add-service outcome may be ambiguous if its reply was lost. Restart "
      "Gazebo before retrying.");
    return fail(logger, "runtime JointTrajectoryController could not be added");
  }
  if (!rclcpp::ok()) {
    return fail_after_controller_added(
      logger, publisher, feedback,
      "interrupted immediately after adding the Gazebo controller");
  }

  const auto connection_deadline = std::chrono::steady_clock::now() + 5s;
  while (rclcpp::ok() && !publisher.HasConnections() &&
    std::chrono::steady_clock::now() < connection_deadline)
  {
    std::this_thread::sleep_for(20ms);
  }
  if (!publisher.HasConnections()) {
    return fail_after_controller_added(
      logger, publisher, feedback,
      "the runtime Gazebo controller did not subscribe");
  }

  const auto command_start_state = feedback.snapshot();
  if (!command_start_state.has_value() ||
    !feedback_is_fresh(*command_start_state))
  {
    return fail_after_controller_added(
      logger, publisher, feedback,
      "fresh feedback disappeared before the first command");
  }
  const auto started_at = std::chrono::steady_clock::now();
  const double started_simulation_seconds =
    command_start_state->simulation_seconds;
  FeedbackSnapshot latest_state = *command_start_state;
  auto next_progress = started_at;

  if (handle_is_down) {
    const auto handle_trajectory = make_dense_handle_trajectory(
      initial->handle_angle, initial->door_angle);
    if (!rclcpp::ok()) {
      return fail_after_controller_added(
        logger, publisher, feedback,
        "interrupted before publishing the handle stage");
    }
    if (!publisher.Publish(handle_trajectory)) {
      return fail_after_controller_added(
        logger, publisher, feedback,
        "Gazebo dense handle trajectory publication failed");
    }
    RCLCPP_WARN(
      logger,
      "Gazebo-only HANDLE stage started: %d dense targets, %.3f -> 92 deg "
      "compensated command (expected feedback near 90 deg), "
      "duration %.3f sim-s. Door remains at its measured start angle.",
      handle_trajectory.points_size(), degrees(initial->handle_angle),
      trajectory_duration_seconds(handle_trajectory));

    std::optional<std::chrono::steady_clock::time_point> handle_stable_since;
    bool handle_stage_complete = false;
    while (!handle_stage_complete) {
      if (!rclcpp::ok()) {
        return fail_after_controller_added(
          logger, publisher, feedback,
          "interrupted during the handle stage");
      }
      const auto now = std::chrono::steady_clock::now();
      const double wall_seconds =
        std::chrono::duration<double>(now - started_at).count();
      if (wall_seconds >= kMaximumWallSeconds) {
        return fail_after_controller_added(
          logger, publisher, feedback,
          "handle unlock was not confirmed within 120 wall seconds");
      }

      const auto state = feedback.snapshot();
      if (!state.has_value() || !feedback_is_fresh(*state)) {
        handle_stable_since.reset();
        std::this_thread::sleep_for(20ms);
        continue;
      }
      latest_state = *state;
      const bool ready =
        std::abs(state->handle_angle - kHandleTarget) <= 1.5 * kPi / 180.0 &&
        std::isfinite(state->handle_velocity) &&
        std::abs(state->handle_velocity) <= 0.02 &&
        latch_is_unlocked(*state);
      if (ready) {
        if (!handle_stable_since.has_value()) {
          handle_stable_since = now;
        }
        handle_stage_complete =
          std::chrono::duration<double>(now - *handle_stable_since).count() >= 0.5;
      } else {
        handle_stable_since.reset();
      }

      if (now >= next_progress) {
        RCLCPP_INFO(
          logger,
          "HANDLE progress wall=%.1f s sim=%.1f s handle=%.2f deg "
          "door=%.2f deg latch='%s' RTF=%.3f.",
          wall_seconds,
          state->simulation_seconds - started_simulation_seconds,
          degrees(state->handle_angle), degrees(state->door_angle),
          state->latch_state.c_str(), state->real_time_factor);
        next_progress = now + 5s;
      }
      std::this_thread::sleep_for(20ms);
    }
  }

  const auto before_door = feedback.snapshot();
  if (!before_door.has_value() || !feedback_is_fresh(*before_door) ||
    std::abs(before_door->handle_angle - kHandleTarget) > 1.5 * kPi / 180.0 ||
    !latch_is_unlocked(*before_door))
  {
    return fail_after_controller_added(
      logger, publisher, feedback,
      "fresh compensated handle plus unlocked latch gate failed before the door stage");
  }
  latest_state = *before_door;

  const auto door_trajectory = make_dense_door_trajectory(
    kHandleTarget, before_door->door_angle, kDoorTarget40);
  if (!rclcpp::ok()) {
    return fail_after_controller_added(
      logger, publisher, feedback,
      "interrupted before publishing the door stage");
  }
  if (!publisher.Publish(door_trajectory)) {
    return fail_after_controller_added(
      logger, publisher, feedback,
      "Gazebo dense door trajectory publication failed");
  }
  RCLCPP_WARN(
    logger,
    "Gazebo-only DOOR stage started only after measured handle/latch gate PASS: "
    "%d dense targets, %.3f -> 40 deg about the existing hinge, duration "
    "%.3f sim-s.",
    door_trajectory.points_size(), degrees(before_door->door_angle),
    trajectory_duration_seconds(door_trajectory));

  std::optional<std::chrono::steady_clock::time_point> stable_since;
  bool door_stage_complete = false;
  while (!door_stage_complete) {
    if (!rclcpp::ok()) {
      return fail_after_controller_added(
        logger, publisher, feedback,
        "interrupted during the door stage");
    }
    const auto now = std::chrono::steady_clock::now();
    const double wall_seconds =
      std::chrono::duration<double>(now - started_at).count();
    if (wall_seconds >= kMaximumWallSeconds) {
      return fail_after_controller_added(
        logger, publisher, feedback,
        "40 deg stable feedback was not reached within 120 wall seconds");
    }

    const auto state = feedback.snapshot();
    if (!state.has_value() || !feedback_is_fresh(*state)) {
      stable_since.reset();
      std::this_thread::sleep_for(20ms);
      continue;
    }
    latest_state = *state;

    const bool angle_ok =
      std::abs(state->handle_angle - kHandleTarget) <= 1.5 * kPi / 180.0 &&
      std::abs(state->door_angle - kDoorTarget40) <= 0.75 * kPi / 180.0 &&
      latch_is_unlocked(*state);
    const bool velocity_ok =
      std::isfinite(state->handle_velocity) &&
      std::isfinite(state->door_velocity) &&
      std::abs(state->handle_velocity) <= 0.02 &&
      std::abs(state->door_velocity) <= 0.01;
    if (angle_ok && velocity_ok) {
      if (!stable_since.has_value()) {
        stable_since = now;
      }
      door_stage_complete =
        std::chrono::duration<double>(now - *stable_since).count() >= 2.0;
    } else {
      stable_since.reset();
    }

    if (now >= next_progress) {
      RCLCPP_INFO(
        logger,
        "DOOR progress wall=%.1f s sim=%.1f s handle=%.2f deg door=%.2f deg "
        "latch='%s' RTF=%.3f.",
        wall_seconds,
        state->simulation_seconds - started_simulation_seconds,
        degrees(state->handle_angle), degrees(state->door_angle),
        state->latch_state.c_str(), state->real_time_factor);
      next_progress = now + 5s;
    }
    std::this_thread::sleep_for(20ms);
  }

  const auto finished_at = std::chrono::steady_clock::now();
  if (!rclcpp::ok()) {
    return fail_after_controller_added(
      logger, publisher, feedback,
      "interrupted after the final stability window");
  }
  const double wall_seconds =
    std::chrono::duration<double>(finished_at - started_at).count();
  const double simulation_seconds =
    latest_state.simulation_seconds - started_simulation_seconds;
  if (wall_seconds > kMaximumWallSeconds) {
    return fail_after_controller_added(
      logger, publisher, feedback,
      "completion exceeded the two-minute wall-time requirement");
  }

  RCLCPP_INFO(
    logger,
    "SIMULATION_DIRECT_JOINT_FAST40_RESULT=PASS wall=%.3f s sim=%.3f s "
    "handle=%.4f deg door=%.4f deg handle_vel=%.6f rad/s "
    "door_vel=%.6f rad/s latch='%s'.",
    wall_seconds, simulation_seconds, degrees(latest_state.handle_angle),
    degrees(latest_state.door_angle), latest_state.handle_velocity,
    latest_state.door_velocity, latest_state.latch_state.c_str());
  RCLCPP_WARN(
    logger,
    "Candidate-3 parameter gate only: no candidate-3 robot path was calculated "
    "or executed. This is not a collision/contact/safety certificate. The "
    "runtime controller intentionally remains loaded to hold the visible final "
    "state; restart Gazebo before later contact validation.");

  rclcpp::shutdown();
  return 0;
}
