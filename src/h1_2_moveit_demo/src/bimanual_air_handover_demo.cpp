#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <future>
#include <functional>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <fcl/fcl.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_plugin_v.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <octomap/OcTree.h>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace
{
using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

constexpr char kPlanningFrame[] = "pelvis";
constexpr char kObjectId[] = "green_air_handover_block";
constexpr char kGazeboObjectName[] = "green_cube";
constexpr char kGazeboPoseTopic[] = "/world/h1_world/dynamic_pose/info";
constexpr char kGazeboSystemAddService[] = "/world/h1_world/entity/system/add";
constexpr char kGazeboRemoveEntityService[] = "/world/h1_world/remove/blocking";
constexpr char kRightGraspDetachTopic[] = "/h1_2/right_pose_grasp/detach";
constexpr char kRightGraspAttachTopic[] = "/h1_2/right_pose_grasp/attach";
constexpr char kRightGraspStateTopic[] = "/h1_2/right_pose_grasp/state";
constexpr char kLeftGraspDetachTopic[] = "/h1_2/left_pose_grasp/detach";
constexpr char kLeftGraspAttachTopic[] = "/h1_2/left_pose_grasp/attach";
constexpr char kLeftGraspStateTopic[] = "/h1_2/left_pose_grasp/state";
constexpr char kIncubatorJointStateTopic[] = "/incubator/joint_states";
constexpr char kIncubatorBodyId[] = "incubator_body_exact";
constexpr char kIncubatorAttachedDoorId[] = "incubator_door_attached";
constexpr char kIncubatorAttachedHandleId[] = "incubator_handle_attached";
constexpr double kCartesianEefStep = 0.005;
constexpr double kMinCartesianFraction = 0.995;
constexpr double kDex1PadGapAtZero = 0.050061;
// Simulation contact compliance routinely divides the two finger strokes a
// few millimetres asymmetrically even when the two-hand support geometry is
// centered.  These are spatial contact gates, so use the project's uniform
// 10 mm minimum distance tolerance while retaining the hard finger-limit gate.
constexpr double kGripperContactGapTolerance = 0.010;
constexpr double kGripperPinchCenterTolerance = 0.010;
constexpr double kGripperContactLowerLimit = -0.019;
// Measured recovery limit for the low, empty left-receiver posture.  A 200 mm
// fixed-orientation lift reached only 70% of its Cartesian path, whereas this
// 120 mm lift solved completely and left enough table clearance for a verified
// joint-space park.  Keep recovery independent from the approach distance used
// by grasp trajectories.
constexpr double kLeftStartupRecoveryClearance = 0.120;
constexpr auto kDetectionPollPeriod = 100ms;
// Gazebo can run far below real time when the GUI, camera and physics are active.
// These limits bound wall-clock waiting only; every hold and stability gate still
// measures the requested duration against /clock.
constexpr double kSlowSimulationWallFactor = 120.0;
constexpr double kMinimumSimulationWallTimeout = 30.0;

std::optional<geometry_msgs::msg::PoseStamped> g_detected_object_pose;
std::mutex g_object_pose_mutex;

struct GazeboPoseSample
{
  geometry_msgs::msg::Pose pose;
  std::uint64_t sequence{0};
  std::chrono::steady_clock::time_point received_at;
};

struct JointMotionLimit
{
  double max_velocity;
  double max_acceleration;
};

struct IncubatorJointSample
{
  double door{std::numeric_limits<double>::quiet_NaN()};
  double handle{std::numeric_limits<double>::quiet_NaN()};
  double door_velocity{std::numeric_limits<double>::quiet_NaN()};
  double handle_velocity{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point received_at;
};

class IncubatorJointMonitor
{
public:
  explicit IncubatorJointMonitor(const rclcpp::Node::SharedPtr & node)
  {
    subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
      kIncubatorJointStateTopic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        IncubatorJointSample sample;
        for (size_t index = 0; index < message->name.size(); ++index) {
          if (index >= message->position.size()) {
            continue;
          }
          const double velocity = index < message->velocity.size() ?
            message->velocity[index] : std::numeric_limits<double>::quiet_NaN();
          if (message->name[index] == "door_hinge_joint") {
            sample.door = message->position[index];
            sample.door_velocity = velocity;
          } else if (message->name[index] == "handle_latch_joint") {
            sample.handle = message->position[index];
            sample.handle_velocity = velocity;
          }
        }
        if (!std::isfinite(sample.door) || !std::isfinite(sample.handle)) {
          return;
        }
        sample.received_at = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        sample_ = sample;
      });
  }

  std::optional<IncubatorJointSample> latest(const double maximum_wall_age = 0.5) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sample_.has_value() ||
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - sample_->received_at).count() > maximum_wall_age)
    {
      return std::nullopt;
    }
    return sample_;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  mutable std::mutex mutex_;
  std::optional<IncubatorJointSample> sample_;
};

using JointMotionLimits = std::map<std::string, JointMotionLimit>;

std::optional<JointMotionLimits> load_synchronized_joint_limits(
  const rclcpp::Node::SharedPtr & node,
  const std::vector<std::string> & joint_names)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(logger, "MoveGroup parameter service is unavailable for joint-limit loading.");
    return std::nullopt;
  }

  std::vector<std::string> parameter_names;
  parameter_names.reserve(joint_names.size() * 4);
  for (const auto & joint_name : joint_names) {
    const std::string prefix =
      "robot_description_planning.joint_limits." + joint_name + ".";
    parameter_names.push_back(prefix + "has_velocity_limits");
    parameter_names.push_back(prefix + "max_velocity");
    parameter_names.push_back(prefix + "has_acceleration_limits");
    parameter_names.push_back(prefix + "max_acceleration");
  }

  const auto future = client->get_parameters(parameter_names);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(logger, "Timed out while loading synchronized joint limits from MoveGroup.");
    return std::nullopt;
  }
  const auto parameters = future.get();
  if (parameters.size() != parameter_names.size()) {
    RCLCPP_ERROR(
      logger, "MoveGroup returned %zu/%zu synchronized joint-limit parameters.",
      parameters.size(), parameter_names.size());
    return std::nullopt;
  }

  JointMotionLimits limits;
  for (size_t index = 0; index < joint_names.size(); ++index) {
    const auto & has_velocity = parameters[index * 4];
    const auto & max_velocity = parameters[index * 4 + 1];
    const auto & has_acceleration = parameters[index * 4 + 2];
    const auto & max_acceleration = parameters[index * 4 + 3];
    if (has_velocity.get_type() != rclcpp::ParameterType::PARAMETER_BOOL ||
      max_velocity.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
      has_acceleration.get_type() != rclcpp::ParameterType::PARAMETER_BOOL ||
      max_acceleration.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
      !has_velocity.as_bool() || !has_acceleration.as_bool() ||
      !std::isfinite(max_velocity.as_double()) || max_velocity.as_double() <= 0.0 ||
      !std::isfinite(max_acceleration.as_double()) || max_acceleration.as_double() <= 0.0)
    {
      RCLCPP_ERROR(
        logger, "MoveGroup has incomplete motion limits for joint '%s'.",
        joint_names[index].c_str());
      return std::nullopt;
    }
    limits.emplace(
      joint_names[index],
      JointMotionLimit{max_velocity.as_double(), max_acceleration.as_double()});
  }
  RCLCPP_INFO(
    logger, "Loaded authoritative velocity/acceleration limits for %zu synchronized joints.",
    limits.size());
  return limits;
}

class GazeboObjectPoseMonitor
{
public:
  bool start()
  {
    return node_.Subscribe(
      kGazeboPoseTopic, &GazeboObjectPoseMonitor::pose_callback, this);
  }

  std::optional<GazeboPoseSample> latest(const double max_wall_age = 1.0) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sample_.has_value() ||
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - sample_->received_at).count() > max_wall_age)
    {
      return std::nullopt;
    }
    return sample_;
  }

private:
  void pose_callback(const gz::msgs::Pose_V & poses)
  {
    for (const auto & pose : poses.pose()) {
      if (pose.name() != kGazeboObjectName) {
        continue;
      }
      GazeboPoseSample sample;
      sample.pose.position.x = pose.position().x();
      sample.pose.position.y = pose.position().y();
      sample.pose.position.z = pose.position().z();
      sample.pose.orientation.x = pose.orientation().x();
      sample.pose.orientation.y = pose.orientation().y();
      sample.pose.orientation.z = pose.orientation().z();
      sample.pose.orientation.w = pose.orientation().w();
      sample.received_at = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        sample.sequence = ++sequence_;
        sample_ = sample;
      }
      return;
    }
  }

  gz::transport::Node node_;
  mutable std::mutex mutex_;
  std::optional<GazeboPoseSample> sample_;
  std::uint64_t sequence_{0};
};

class GazeboIndependentGrasp
{
public:
  GazeboIndependentGrasp()
  {
    state_subscribed_ = node_.Subscribe(
      kRightGraspStateTopic, &GazeboIndependentGrasp::state_callback, this);
    left_state_subscribed_ = node_.Subscribe(
      kLeftGraspStateTopic, &GazeboIndependentGrasp::left_state_callback, this);
    pose_subscribed_ = node_.Subscribe(
      kGazeboPoseTopic, &GazeboIndependentGrasp::pose_callback, this);
  }

  bool attach_right_to_cube()
  {
    const auto logger = rclcpp::get_logger("h1_2_air_handover");
    if (!state_subscribed_ || !pose_subscribed_) {
      RCLCPP_ERROR(logger, "Could not subscribe to the Gazebo grasp-constraint state topic.");
      return false;
    }

    const auto cube_model_entity_id = wait_for_cube_model_entity_id(5.0);
    if (!cube_model_entity_id.has_value()) {
      RCLCPP_ERROR(
        logger,
        "Could not resolve Gazebo model entity '%s' from '%s'; refusing to add the grasp "
        "constraint to an unresolved entity.",
        kGazeboObjectName, kGazeboPoseTopic);
      return false;
    }

    std::uint64_t initial_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      initial_sequence = sequence_;
    }

    gz::msgs::EntityPlugin_V request;
    auto * entity = request.mutable_entity();
    entity->set_id(*cube_model_entity_id);
    entity->set_name(kGazeboObjectName);
    entity->set_type(gz::msgs::Entity::MODEL);
    auto * plugin = request.add_plugins();
    plugin->set_name("h1_2::GazeboCompliantGraspSystem");
    plugin->set_filename(H1_GAZEBO_GRASP_FOLLOWER_LIBRARY);
    plugin->set_innerxml(
      std::string("<parent_scope>h1_2::right_wrist_yaw_link</parent_scope>") +
      "<detach_topic>" + kRightGraspDetachTopic + "</detach_topic>" +
      "<attach_topic>" + kRightGraspAttachTopic + "</attach_topic>" +
      "<output_topic>" + kRightGraspStateTopic + "</output_topic>");

    gz::msgs::Boolean reply;
    bool service_result = false;
    const bool request_completed = node_.Request(
      kGazeboSystemAddService, request, 10000u, reply, service_result);
    if (!request_completed || !service_result || !reply.data()) {
      RCLCPP_ERROR(
        logger,
        "Gazebo rejected the runtime independent grasp-follower system request "
        "(completed=%d, result=%d, reply=%d).",
        request_completed, service_result, reply.data());
      return false;
    }
    if (!wait_for_state("attached", initial_sequence, 10.0)) {
      RCLCPP_ERROR(
        logger,
        "Gazebo did not confirm the right-hand grasp constraint as attached.");
      return false;
    }
    attached_by_this_process_ = true;
    RCLCPP_INFO(
      logger,
      "Gazebo independent collision-body grasp support confirmed attached after "
      "symmetric real-finger contact.");
    return true;
  }

  bool attach_left_to_cube()
  {
    const auto logger = rclcpp::get_logger("h1_2_air_handover");
    if (!left_state_subscribed_ || !pose_subscribed_) {
      RCLCPP_ERROR(logger, "Could not subscribe to the left Gazebo grasp-support state topic.");
      return false;
    }
    const auto cube_model_entity_id = wait_for_cube_model_entity_id(5.0);
    if (!cube_model_entity_id.has_value()) {
      RCLCPP_ERROR(logger, "Could not resolve the Gazebo block before left support transfer.");
      return false;
    }

    std::uint64_t initial_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      initial_sequence = left_sequence_;
    }

    gz::msgs::EntityPlugin_V request;
    auto * entity = request.mutable_entity();
    entity->set_id(*cube_model_entity_id);
    entity->set_name(kGazeboObjectName);
    entity->set_type(gz::msgs::Entity::MODEL);
    auto * plugin = request.add_plugins();
    plugin->set_name("h1_2::GazeboCompliantGraspSystem");
    plugin->set_filename(H1_GAZEBO_GRASP_FOLLOWER_LIBRARY);
    plugin->set_innerxml(
      std::string("<parent_scope>h1_2::left_wrist_yaw_link</parent_scope>") +
      "<detach_topic>" + kLeftGraspDetachTopic + "</detach_topic>" +
      "<attach_topic>" + kLeftGraspAttachTopic + "</attach_topic>" +
      "<output_topic>" + kLeftGraspStateTopic + "</output_topic>");

    gz::msgs::Boolean reply;
    bool service_result = false;
    const bool request_completed = node_.Request(
      kGazeboSystemAddService, request, 10000u, reply, service_result);
    if (!request_completed || !service_result || !reply.data()) {
      RCLCPP_ERROR(
        logger,
        "Gazebo rejected the left compliant grasp-support request "
        "(completed=%d, result=%d, reply=%d).",
        request_completed, service_result, reply.data());
      return false;
    }
    if (!wait_for_left_state("attached", initial_sequence, 10.0)) {
      RCLCPP_ERROR(logger, "Gazebo did not confirm left-wrist grasp support as attached.");
      return false;
    }
    left_attached_by_this_process_ = true;
    RCLCPP_INFO(
      logger,
      "Gazebo independent collision-body support transferred onto the verified left grasp.");
    return true;
  }

  bool detach_right_from_cube(const bool require_confirmation)
  {
    const auto logger = rclcpp::get_logger("h1_2_air_handover");
    if (!state_subscribed_) {
      return !require_confirmation;
    }
    std::uint64_t initial_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_state_ == "detached") {
        attached_by_this_process_ = false;
        return true;
      }
      initial_sequence = sequence_;
    }

    auto publisher = node_.Advertise<gz::msgs::Empty>(kRightGraspDetachTopic);
    if (!publisher) {
      RCLCPP_ERROR(logger, "Could not advertise the Gazebo grasp detach command.");
      return !require_confirmation;
    }
    gz::msgs::Empty message;
    for (int attempt = 0; attempt < 5; ++attempt) {
      publisher.Publish(message);
      if (wait_for_state("detached", initial_sequence, 2.0)) {
        attached_by_this_process_ = false;
        RCLCPP_INFO(logger, "Gazebo grasp support confirmed detached.");
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_ERROR(
      logger,
      "Gazebo did not confirm the right-hand grasp constraint as detached; "
      "the gripper will not open.");
    return !require_confirmation;
  }

  bool reattach_right_to_cube()
  {
    const auto logger = rclcpp::get_logger("h1_2_air_handover");
    if (!state_subscribed_) {
      return false;
    }
    std::uint64_t initial_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      initial_sequence = sequence_;
    }
    auto publisher = node_.Advertise<gz::msgs::Empty>(kRightGraspAttachTopic);
    if (!publisher) {
      RCLCPP_ERROR(logger, "Could not advertise the Gazebo grasp reattach command.");
      return false;
    }
    gz::msgs::Empty message;
    for (int attempt = 0; attempt < 5; ++attempt) {
      publisher.Publish(message);
      if (wait_for_state("attached", initial_sequence, 2.0)) {
        attached_by_this_process_ = true;
        RCLCPP_WARN(
          logger,
          "Restored right-wrist simulation support after the airborne detach gate failed.");
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_ERROR(logger, "Could not restore right-wrist simulation support.");
    return false;
  }

  bool detach_left_from_cube(const bool require_confirmation)
  {
    const auto logger = rclcpp::get_logger("h1_2_air_handover");
    if (!left_state_subscribed_) {
      return !require_confirmation;
    }
    std::uint64_t initial_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (left_last_state_ == "detached") {
        left_attached_by_this_process_ = false;
        return true;
      }
      initial_sequence = left_sequence_;
    }
    auto publisher = node_.Advertise<gz::msgs::Empty>(kLeftGraspDetachTopic);
    if (!publisher) {
      RCLCPP_ERROR(logger, "Could not advertise the left Gazebo grasp-support detach command.");
      return !require_confirmation;
    }
    gz::msgs::Empty message;
    for (int attempt = 0; attempt < 5; ++attempt) {
      publisher.Publish(message);
      if (wait_for_left_state("detached", initial_sequence, 2.0)) {
        left_attached_by_this_process_ = false;
        RCLCPP_INFO(logger, "Gazebo left-wrist grasp support confirmed detached.");
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_ERROR(
      logger,
      "Gazebo did not confirm left-wrist support as detached; the left gripper will not open.");
    return !require_confirmation;
  }

  bool attached_by_this_process() const
  {
    return attached_by_this_process_;
  }

  bool remove_cube_model_for_reset()
  {
    const auto logger = rclcpp::get_logger("h1_2_air_handover");
    const auto entity_id = wait_for_cube_model_entity_id(5.0);
    if (!entity_id.has_value()) {
      RCLCPP_ERROR(logger, "Could not resolve green_cube for the failed-trial reset.");
      return false;
    }
    gz::msgs::Entity request;
    request.set_id(*entity_id);
    request.set_name(kGazeboObjectName);
    request.set_type(gz::msgs::Entity::MODEL);
    gz::msgs::Boolean reply;
    bool service_result = false;
    const bool request_completed = node_.Request(
      kGazeboRemoveEntityService, request, 10000u, reply, service_result);
    if (!request_completed || !service_result || !reply.data()) {
      RCLCPP_ERROR(
        logger,
        "Gazebo failed to remove the stopped trial cube "
        "(completed=%d, result=%d, reply=%d).",
        request_completed, service_result, reply.data());
      return false;
    }
    RCLCPP_INFO(logger, "Gazebo removed the stopped trial cube for a clean respawn.");
    return true;
  }

private:
  void pose_callback(const gz::msgs::Pose_V & poses)
  {
    for (const auto & pose : poses.pose()) {
      if (pose.name() != kGazeboObjectName || pose.id() == 0u) {
        continue;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      cube_model_entity_id_ = pose.id();
      return;
    }
  }

  void state_callback(const gz::msgs::StringMsg & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_state_ = message.data();
    ++sequence_;
  }

  void left_state_callback(const gz::msgs::StringMsg & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    left_last_state_ = message.data();
    ++left_sequence_;
  }

  bool wait_for_state(
    const std::string & expected,
    const std::uint64_t minimum_sequence,
    const double wall_timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(wall_timeout);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (sequence_ > minimum_sequence && last_state_ == expected) {
          return true;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  bool wait_for_left_state(
    const std::string & expected,
    const std::uint64_t minimum_sequence,
    const double wall_timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(wall_timeout);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (left_sequence_ > minimum_sequence && left_last_state_ == expected) {
          return true;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  std::optional<std::uint64_t> wait_for_cube_model_entity_id(const double wall_timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(wall_timeout);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (cube_model_entity_id_.has_value()) {
          return cube_model_entity_id_;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return std::nullopt;
  }

  gz::transport::Node node_;
  mutable std::mutex mutex_;
  std::string last_state_;
  std::string left_last_state_;
  std::uint64_t sequence_{0};
  std::uint64_t left_sequence_{0};
  std::optional<std::uint64_t> cube_model_entity_id_;
  bool state_subscribed_{false};
  bool left_state_subscribed_{false};
  bool pose_subscribed_{false};
  bool attached_by_this_process_{false};
  bool left_attached_by_this_process_{false};
};

void object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(g_object_pose_mutex);
  g_detected_object_pose = *msg;
}

bool set_bounded_start_state(
  MoveGroupInterface & move_group,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  auto start_state = move_group.getCurrentState(10.0);
  if (!start_state) {
    RCLCPP_ERROR(logger, "No current state is available for stage '%s'.", stage_name.c_str());
    return false;
  }

  constexpr double kJointLimitMargin = 1e-6;
  for (const auto & joint : move_group.getActiveJoints()) {
    const auto & bounds = start_state->getRobotModel()->getVariableBounds(joint);
    if (!bounds.position_bounded_) {
      continue;
    }
    const double span = bounds.max_position_ - bounds.min_position_;
    const double margin = std::min(kJointLimitMargin, span * 0.25);
    start_state->setVariablePosition(
      joint,
      std::clamp(
        start_state->getVariablePosition(joint),
        bounds.min_position_ + margin,
        bounds.max_position_ - margin));
  }
  start_state->update();

  moveit_msgs::msg::RobotState start_state_msg;
  start_state_msg.is_diff = true;
  for (const auto & joint : move_group.getActiveJoints()) {
    start_state_msg.joint_state.name.push_back(joint);
    start_state_msg.joint_state.position.push_back(start_state->getVariablePosition(joint));
  }
  move_group.setStartState(start_state_msg);
  return true;
}

bool plan_and_execute_joint(
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & target,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  if (!set_bounded_start_state(move_group, stage_name)) {
    return false;
  }
  if (!move_group.setJointValueTarget(target)) {
    RCLCPP_ERROR(logger, "Joint target violates a limit at stage '%s'.", stage_name.c_str());
    return false;
  }

  MoveGroupInterface::Plan plan;
  RCLCPP_INFO(logger, "Planning stage: %s", stage_name.c_str());
  if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning failed at stage '%s'.", stage_name.c_str());
    return false;
  }
  RCLCPP_INFO(
    logger, "Stage '%s' planned with %zu points; executing...",
    stage_name.c_str(), plan.trajectory.joint_trajectory.points.size());
  if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed at stage '%s'.", stage_name.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "Stage completed: %s", stage_name.c_str());
  return true;
}

bool execute_prevalidated_joint_target(
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & target,
  const double duration,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  const auto current = move_group.getCurrentState(10.0);
  const auto joint_names = move_group.getActiveJoints();
  if (!current || joint_names.empty() || !std::isfinite(duration) || duration < 0.2) {
    return false;
  }
  if (target.size() != joint_names.size()) {
    RCLCPP_ERROR(
      logger, "Direct target '%s' must cover exactly %zu active joints.",
      stage_name.c_str(), joint_names.size());
    return false;
  }
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = joint_names;
  constexpr size_t kSegments = 10U;
  trajectory.joint_trajectory.points.reserve(kSegments + 1U);
  for (size_t step = 0U; step <= kSegments; ++step) {
    const double alpha = static_cast<double>(step) / static_cast<double>(kSegments);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(joint_names.size());
    for (const auto & joint : joint_names) {
      const auto target_joint = target.find(joint);
      if (target_joint == target.end() || !std::isfinite(target_joint->second)) {
        return false;
      }
      const auto & bounds = current->getRobotModel()->getVariableBounds(joint);
      if (bounds.position_bounded_ &&
        (target_joint->second < bounds.min_position_ - 1e-9 ||
        target_joint->second > bounds.max_position_ + 1e-9))
      {
        return false;
      }
      point.positions.push_back(
        current->getVariablePosition(joint) +
        alpha * (target_joint->second - current->getVariablePosition(joint)));
    }
    point.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(std::max(0.05, alpha * duration)));
    trajectory.joint_trajectory.points.push_back(std::move(point));
  }
  RCLCPP_INFO(
    logger,
    "Executing prevalidated direct joint target '%s' with %zu points over %.2f s.",
    stage_name.c_str(), trajectory.joint_trajectory.points.size(), duration);
  const bool success = move_group.execute(trajectory) ==
    moveit::core::MoveItErrorCode::SUCCESS;
  if (!success) {
    RCLCPP_WARN(
      logger,
      "Direct target '%s' reported controller failure; physical feedback gate will decide.",
      stage_name.c_str());
  }
  return success;
}

bool joint_target_is_reached(
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & target,
  const double tolerance,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  constexpr double kFeedbackSettleTimeout = 10.0;
  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::duration<double>(kFeedbackSettleTimeout);
  double maximum_error = 0.0;
  std::string worst_joint;
  bool received_feedback = false;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto state = move_group.getCurrentState(1.0);
    if (state) {
      received_feedback = true;
      maximum_error = 0.0;
      worst_joint.clear();
      for (const auto & [joint_name, expected] : target) {
        const double error = std::abs(state->getVariablePosition(joint_name) - expected);
        if (error > maximum_error) {
          maximum_error = error;
          worst_joint = joint_name;
        }
      }
      if (maximum_error <= tolerance) {
        const double settle_wall_seconds =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        RCLCPP_INFO(
          logger,
          "Joint feedback gate '%s': maximum error %.4f after %.2f wall seconds.",
          stage_name.c_str(), maximum_error, settle_wall_seconds);
        return true;
      }
    }
    std::this_thread::sleep_for(50ms);
  }
  if (!received_feedback) {
    RCLCPP_ERROR(logger, "No joint feedback is available after '%s'.", stage_name.c_str());
  } else {
    RCLCPP_WARN(
      logger,
      "Joint feedback gate '%s' failed after %.1f wall seconds: maximum error %.4f "
      "at '%s' (limit %.4f).",
      stage_name.c_str(), kFeedbackSettleTimeout, maximum_error,
      worst_joint.c_str(), tolerance);
  }
  return false;
}

bool gripper_open_target_is_reached(
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & target,
  const double tolerance,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  const auto state = move_group.getCurrentState(10.0);
  if (!state) {
    RCLCPP_ERROR(logger, "No gripper feedback is available after '%s'.", stage_name.c_str());
    return false;
  }

  double maximum_not_open_enough = 0.0;
  std::string worst_joint;
  for (const auto & [joint_name, expected] : target) {
    // Dex1 becomes more open as the finger coordinate decreases.  A finger
    // clamped slightly below the requested open target is safer, not an error.
    const double not_open_enough =
      state->getVariablePosition(joint_name) - expected;
    if (not_open_enough > maximum_not_open_enough) {
      maximum_not_open_enough = not_open_enough;
      worst_joint = joint_name;
    }
  }
  if (maximum_not_open_enough <= tolerance) {
    RCLCPP_INFO(
      logger,
      "One-sided open feedback gate '%s' passed: maximum shortfall %.3f mm.",
      stage_name.c_str(), maximum_not_open_enough * 1000.0);
    return true;
  }
  RCLCPP_WARN(
    logger,
    "One-sided open feedback gate '%s' failed: '%s' is %.3f mm short of the "
    "requested opening (limit %.3f mm).",
    stage_name.c_str(), worst_joint.c_str(), maximum_not_open_enough * 1000.0,
    tolerance * 1000.0);
  return false;
}

bool verify_gripper_contact_feedback(
  MoveGroupInterface & gripper,
  const std::string & side,
  const double commanded_position,
  const double object_width,
  const bool enforce_contact_gate,
  const std::string & stage_name,
  const double pinch_center_tolerance = kGripperPinchCenterTolerance)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  const auto state = gripper.getCurrentState(10.0);
  if (!state) {
    RCLCPP_ERROR(
      logger, "No gripper joint feedback is available after '%s'.", stage_name.c_str());
    return false;
  }

  const std::string finger_1_joint = side + "_dex1_finger_1_joint";
  const std::string finger_2_joint = side + "_dex1_finger_2_joint";
  const double q1 = state->getVariablePosition(finger_1_joint);
  const double q2 = state->getVariablePosition(finger_2_joint);
  const double pad_gap = kDex1PadGapAtZero - (q1 + q2);
  const double pinch_center_offset = (q2 - q1) / 2.0;
  if (!std::isfinite(q1) || !std::isfinite(q2) || !std::isfinite(pad_gap) ||
    !std::isfinite(pinch_center_offset))
  {
    RCLCPP_ERROR(
      logger, "Gripper contact diagnostic '%s' received non-finite joint feedback.",
      stage_name.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger,
    "Gripper contact diagnostic '%s': command %.3f mm, q1 %.3f mm, q2 %.3f mm, "
    "pad gap %.3f mm, pinch center offset %.3f mm.",
    stage_name.c_str(), commanded_position * 1000.0, q1 * 1000.0, q2 * 1000.0,
    pad_gap * 1000.0, pinch_center_offset * 1000.0);

  if (!enforce_contact_gate) {
    return true;
  }

  const double gap_error = std::abs(pad_gap - object_width);
  const bool finger_at_lower_limit =
    q1 < kGripperContactLowerLimit || q2 < kGripperContactLowerLimit;
  const bool pinch_center_outside_tolerance =
    std::abs(pinch_center_offset) > pinch_center_tolerance;
  if (gap_error > kGripperContactGapTolerance || finger_at_lower_limit ||
    pinch_center_outside_tolerance)
  {
    RCLCPP_ERROR(
      logger,
      "Gripper contact gate '%s' failed: pad gap %.3f mm vs object %.3f mm "
      "(error %.3f mm, limit %.3f mm), center offset %.3f mm (limit %.3f mm), "
      "q1/q2 %.3f/%.3f mm. Refusing to lift with asymmetric contact.",
      stage_name.c_str(), pad_gap * 1000.0, object_width * 1000.0,
      gap_error * 1000.0, kGripperContactGapTolerance * 1000.0,
      pinch_center_offset * 1000.0, pinch_center_tolerance * 1000.0,
      q1 * 1000.0, q2 * 1000.0);
    return false;
  }

  RCLCPP_INFO(
    logger,
    "Gripper contact gate '%s' passed: pad gap error %.3f mm (limit %.3f mm).",
    stage_name.c_str(), gap_error * 1000.0, kGripperContactGapTolerance * 1000.0);
  return true;
}

bool plan_and_execute_pose(
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  move_group.clearPoseTargets();
  move_group.setPoseTarget(target);

  MoveGroupInterface::Plan plan;
  RCLCPP_INFO(logger, "Planning pose stage: %s", stage_name.c_str());
  if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Pose planning failed at stage '%s'.", stage_name.c_str());
    return false;
  }
  RCLCPP_INFO(
    logger, "Stage '%s' planned with %zu points; executing...",
    stage_name.c_str(), plan.trajectory.joint_trajectory.points.size());
  if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed at stage '%s'.", stage_name.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "Stage completed: %s", stage_name.c_str());
  return true;
}

bool plan_and_execute_pose_with_cartesian_continuation(
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & pose_target,
  const geometry_msgs::msg::Pose & continuation_target,
  const std::string & stage_name,
  const int max_candidates = 24,
  const std::vector<geometry_msgs::msg::Pose> & branch_probe_targets = {},
  const std::function<bool(const trajectory_msgs::msg::JointTrajectory &)> &
  candidate_gate = {})
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  for (int candidate = 1; candidate <= max_candidates; ++candidate) {
    if (!set_bounded_start_state(move_group, stage_name)) {
      return false;
    }
    move_group.clearPoseTargets();
    move_group.setPoseTarget(pose_target);

    MoveGroupInterface::Plan pose_plan;
    if (move_group.plan(pose_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
      pose_plan.trajectory.joint_trajectory.points.empty())
    {
      RCLCPP_WARN(
        logger, "IK-branch candidate %d/%d could not reach '%s'.",
        candidate, max_candidates, stage_name.c_str());
      continue;
    }

    moveit_msgs::msg::RobotState candidate_start;
    candidate_start.is_diff = true;
    candidate_start.joint_state.name = pose_plan.trajectory.joint_trajectory.joint_names;
    candidate_start.joint_state.position =
      pose_plan.trajectory.joint_trajectory.points.back().positions;
    move_group.setStartState(candidate_start);
    moveit_msgs::msg::RobotTrajectory continuation;
    moveit_msgs::msg::MoveItErrorCodes continuation_error;
    const double fraction = move_group.computeCartesianPath(
      {continuation_target}, kCartesianEefStep, continuation, true, &continuation_error);
    const auto point_count = continuation.joint_trajectory.points.size();
    RCLCPP_INFO(
      logger,
      "IK-branch candidate %d/%d for '%s': continuation %.1f%% (%zu points).",
      candidate, max_candidates, stage_name.c_str(), fraction * 100.0, point_count);
    if (fraction < kMinCartesianFraction || point_count == 0) {
      continue;
    }
    if (candidate_gate && !candidate_gate(continuation.joint_trajectory)) {
      RCLCPP_INFO(
        logger, "IK-branch candidate %d/%d for '%s' failed its endpoint gate.",
        candidate, max_candidates, stage_name.c_str());
      continue;
    }

    if (!branch_probe_targets.empty()) {
      moveit_msgs::msg::RobotState probe_start;
      probe_start.is_diff = true;
      probe_start.joint_state.name = continuation.joint_trajectory.joint_names;
      probe_start.joint_state.position =
        continuation.joint_trajectory.points.back().positions;
      move_group.setStartState(probe_start);
      moveit_msgs::msg::RobotTrajectory branch_probe;
      moveit_msgs::msg::MoveItErrorCodes branch_probe_error;
      const double probe_fraction = move_group.computeCartesianPath(
        branch_probe_targets, kCartesianEefStep,
        branch_probe, false, &branch_probe_error);
      const auto probe_points = branch_probe.joint_trajectory.points.size();
      RCLCPP_INFO(
        logger,
        "IK-branch candidate %d/%d for '%s': contact-and-lift probe %.1f%% (%zu points).",
        candidate, max_candidates, stage_name.c_str(), probe_fraction * 100.0, probe_points);
      if (probe_fraction < kMinCartesianFraction || probe_points < 2) {
        continue;
      }
    }

    RCLCPP_INFO(
      logger, "Selected candidate %d for '%s'; executing empty-hand pose and continuation.",
      candidate, stage_name.c_str());
    if (move_group.execute(pose_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Pose execution failed at branch-selection stage '%s'.", stage_name.c_str());
      return false;
    }
    if (move_group.execute(continuation) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        logger, "Cartesian continuation execution failed at stage '%s'.", stage_name.c_str());
      return false;
    }
    RCLCPP_INFO(logger, "Branch-selection stage completed: %s", stage_name.c_str());
    return true;
  }
  move_group.setStartStateToCurrentState();
  RCLCPP_ERROR(
    logger, "No pose candidate provided a complete Cartesian continuation for '%s'.",
    stage_name.c_str());
  return false;
}

std::optional<std::map<std::string, double>> compute_seeded_ik_joint_target(
  const rclcpp::Node::SharedPtr & node,
  MoveGroupInterface & move_group,
  const rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr & ik_client,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  const geometry_msgs::msg::Pose & pose_target,
  const std::string & ik_link_name,
  const std::map<std::string, double> & seed,
  const std::string & stage_name,
  const std::function<bool(const std::map<std::string, double> &)> & solution_gate = {},
  const bool avoid_collisions = true)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  auto seed_state = move_group.getCurrentState(10.0);
  if (!seed_state) {
    RCLCPP_ERROR(logger, "No current state is available for seeded IK stage '%s'.", stage_name.c_str());
    return std::nullopt;
  }
  const auto & model_variable_names = seed_state->getVariableNames();
  const auto model_has_variable = [&model_variable_names](const std::string & variable_name) {
      return std::find(
        model_variable_names.begin(), model_variable_names.end(), variable_name) !=
        model_variable_names.end();
    };
  const auto active_joint_names = move_group.getActiveJoints();
  const std::set<std::string> active_joints(
    active_joint_names.begin(), active_joint_names.end());
  std::set<std::string> seeded_joints;
  for (const auto & [joint_name, unused] : seed) {
    static_cast<void>(unused);
    seeded_joints.insert(joint_name);
  }
  if (seeded_joints != active_joints) {
    RCLCPP_ERROR(
      logger, "Seed for '%s' must cover exactly all %zu active joints (received %zu).",
      stage_name.c_str(), active_joints.size(), seeded_joints.size());
    return std::nullopt;
  }
  for (const auto & [joint_name, position] : seed) {
    if (!model_has_variable(joint_name) || !std::isfinite(position)) {
      RCLCPP_ERROR(logger, "Invalid seeded IK joint '%s' for '%s'.", joint_name.c_str(), stage_name.c_str());
      return std::nullopt;
    }
    seed_state->setVariablePosition(joint_name, position);
  }
  seed_state->update();
  if (!seed_state->satisfiesBounds(1e-6)) {
    RCLCPP_ERROR(logger, "Seed state is outside robot bounds for '%s'.", stage_name.c_str());
    return std::nullopt;
  }
  if (!ik_client->wait_for_service(3s)) {
    RCLCPP_ERROR(logger, "MoveGroup IK service is unavailable for '%s'.", stage_name.c_str());
    return std::nullopt;
  }

  auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  request->ik_request.group_name = move_group.getName();
  request->ik_request.robot_state.is_diff = true;
  request->ik_request.robot_state.joint_state.name = seed_state->getVariableNames();
  request->ik_request.robot_state.joint_state.position.reserve(
    request->ik_request.robot_state.joint_state.name.size());
  for (const auto & variable_name : request->ik_request.robot_state.joint_state.name) {
    request->ik_request.robot_state.joint_state.position.push_back(
      seed_state->getVariablePosition(variable_name));
  }
  request->ik_request.avoid_collisions = avoid_collisions;
  request->ik_request.ik_link_name = ik_link_name;
  request->ik_request.pose_stamped.header.frame_id = kPlanningFrame;
  request->ik_request.pose_stamped.header.stamp = node->now();
  request->ik_request.pose_stamped.pose = pose_target;
  request->ik_request.timeout = rclcpp::Duration::from_seconds(2.0);

  auto future = ik_client->async_send_request(request);
  if (future.wait_for(3s) != std::future_status::ready) {
    RCLCPP_ERROR(logger, "Seeded IK request timed out for '%s'.", stage_name.c_str());
    return std::nullopt;
  }
  const auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(
      logger, "Seeded IK failed for '%s' with MoveIt error %d.",
      stage_name.c_str(), response->error_code.val);
    return std::nullopt;
  }

  const auto & solution = response->solution.joint_state;
  if (solution.name.size() != solution.position.size()) {
    RCLCPP_ERROR(logger, "Seeded IK returned a malformed state for '%s'.", stage_name.c_str());
    return std::nullopt;
  }
  std::map<std::string, double> solution_by_name;
  for (size_t index = 0; index < solution.name.size(); ++index) {
    solution_by_name.emplace(solution.name[index], solution.position[index]);
    if (model_has_variable(solution.name[index])) {
      seed_state->setVariablePosition(solution.name[index], solution.position[index]);
    }
  }
  seed_state->update();
  if (!seed_state->satisfiesBounds(1e-6)) {
    RCLCPP_ERROR(logger, "Seeded IK returned an out-of-bounds state for '%s'.", stage_name.c_str());
    return std::nullopt;
  }

  std::map<std::string, double> joint_target;
  for (const auto & joint_name : move_group.getActiveJoints()) {
    const auto found = solution_by_name.find(joint_name);
    if (found == solution_by_name.end() || !std::isfinite(found->second)) {
      RCLCPP_ERROR(
        logger, "Seeded IK omitted active joint '%s' for '%s'.",
        joint_name.c_str(), stage_name.c_str());
      return std::nullopt;
    }
    joint_target.emplace(joint_name, found->second);
  }
  if (solution_gate && !solution_gate(joint_target)) {
    RCLCPP_ERROR(logger, "Seeded IK response failed its branch gate for '%s'.", stage_name.c_str());
    return std::nullopt;
  }
  const auto & actual_transform = seed_state->getGlobalLinkTransform(ik_link_name);
  geometry_msgs::msg::PoseStamped actual_model_pose;
  actual_model_pose.header.frame_id = seed_state->getRobotModel()->getModelFrame();
  actual_model_pose.pose.position.x = actual_transform.translation().x();
  actual_model_pose.pose.position.y = actual_transform.translation().y();
  actual_model_pose.pose.position.z = actual_transform.translation().z();
  const Eigen::Quaterniond actual_orientation(actual_transform.linear());
  actual_model_pose.pose.orientation.x = actual_orientation.x();
  actual_model_pose.pose.orientation.y = actual_orientation.y();
  actual_model_pose.pose.orientation.z = actual_orientation.z();
  actual_model_pose.pose.orientation.w = actual_orientation.w();
  geometry_msgs::msg::PoseStamped actual_planning_pose;
  try {
    tf_buffer->transform(
      actual_model_pose, actual_planning_pose, kPlanningFrame,
      tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger, "Cannot transform seeded IK FK from '%s' to '%s' for '%s': %s",
      actual_model_pose.header.frame_id.c_str(), kPlanningFrame,
      stage_name.c_str(), ex.what());
    return std::nullopt;
  }
  const Eigen::Vector3d expected_translation(
    pose_target.position.x, pose_target.position.y, pose_target.position.z);
  Eigen::Quaterniond expected_orientation(
    pose_target.orientation.w, pose_target.orientation.x,
    pose_target.orientation.y, pose_target.orientation.z);
  expected_orientation.normalize();
  const double translation_error =
    (Eigen::Vector3d(
      actual_planning_pose.pose.position.x,
      actual_planning_pose.pose.position.y,
      actual_planning_pose.pose.position.z) - expected_translation).norm();
  const Eigen::Quaterniond actual_planning_orientation(
    actual_planning_pose.pose.orientation.w,
    actual_planning_pose.pose.orientation.x,
    actual_planning_pose.pose.orientation.y,
    actual_planning_pose.pose.orientation.z);
  const double rotation_error = Eigen::AngleAxisd(
    expected_orientation.toRotationMatrix().transpose() *
    actual_planning_orientation.normalized().toRotationMatrix()).angle();
  constexpr double kIkFkTranslationTolerance = 0.0005;
  constexpr double kIkFkRotationTolerance = 0.004363323129985824;
  if (translation_error > kIkFkTranslationTolerance ||
    rotation_error > kIkFkRotationTolerance)
  {
    RCLCPP_ERROR(
      logger, "Seeded IK FK mismatch for '%s': %.3f mm / %.3f deg.",
      stage_name.c_str(), translation_error * 1000.0,
      rotation_error * 57.29577951308232);
    return std::nullopt;
  }
  RCLCPP_INFO(
    logger, "Seeded IK produced a complete %s target for '%s'.",
    avoid_collisions ? "collision-checked" : "kinematic-only",
    stage_name.c_str());
  return joint_target;
}

std::optional<std::map<std::string, double>> compute_nearest_seeded_ik_joint_target(
  const rclcpp::Node::SharedPtr & node,
  MoveGroupInterface & move_group,
  const rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr & ik_client,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  const geometry_msgs::msg::Pose & pose_target,
  const std::string & ik_link_name,
  const std::map<std::string, double> & seed,
  const std::string & stage_name,
  const int candidate_count,
  const double maximum_total_travel = std::numeric_limits<double>::infinity(),
  const double minimum_joint_limit_margin = 0.05,
  const std::function<bool(const std::map<std::string, double> &)> &
  solution_gate = {})
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  const bool valid_finite_limit =
    std::isfinite(maximum_total_travel) && maximum_total_travel > 0.0;
  const bool unlimited = maximum_total_travel == std::numeric_limits<double>::infinity();
  if (candidate_count <= 0 || (!valid_finite_limit && !unlimited) ||
    !std::isfinite(minimum_joint_limit_margin) || minimum_joint_limit_margin <= 0.0)
  {
    RCLCPP_ERROR(logger, "Invalid nearest-IK search settings for '%s'.", stage_name.c_str());
    return std::nullopt;
  }

  std::optional<std::map<std::string, double>> best_candidate;
  double best_total_travel = std::numeric_limits<double>::infinity();
  const auto scoring_state = move_group.getCurrentState(10.0);
  if (!scoring_state) {
    RCLCPP_ERROR(logger, "No robot model is available to score nearest IK for '%s'.", stage_name.c_str());
    return std::nullopt;
  }
  for (int candidate_index = 1; candidate_index <= candidate_count; ++candidate_index) {
    const auto candidate = compute_seeded_ik_joint_target(
      node, move_group, ik_client, tf_buffer, pose_target, ik_link_name, seed,
      stage_name + " candidate " + std::to_string(candidate_index));
    if (!candidate.has_value()) {
      continue;
    }
    if (solution_gate && !solution_gate(*candidate)) {
      RCLCPP_INFO(
        logger, "Nearest-IK candidate %d/%d for '%s' failed its posture gate.",
        candidate_index, candidate_count, stage_name.c_str());
      continue;
    }
    double total_travel = 0.0;
    double minimum_bound_margin = std::numeric_limits<double>::infinity();
    std::string closest_bound_joint;
    for (const auto & [joint_name, seed_position] : seed) {
      const auto found = candidate->find(joint_name);
      if (found == candidate->end()) {
        total_travel = std::numeric_limits<double>::infinity();
        break;
      }
      total_travel += std::abs(found->second - seed_position);
      const auto & bounds = scoring_state->getRobotModel()->getVariableBounds(joint_name);
      if (bounds.position_bounded_) {
        const double margin = std::min(
          found->second - bounds.min_position_, bounds.max_position_ - found->second);
        if (margin < minimum_bound_margin) {
          minimum_bound_margin = margin;
          closest_bound_joint = joint_name;
        }
      }
    }
    RCLCPP_INFO(
      logger,
      "Nearest-IK candidate %d/%d for '%s': total parked-state travel %.3f rad, "
      "minimum bound margin %.3f rad at '%s' (%s).",
      candidate_index, candidate_count, stage_name.c_str(), total_travel,
      minimum_bound_margin, closest_bound_joint.c_str(),
      minimum_bound_margin >= minimum_joint_limit_margin ? "accepted" : "rejected");
    if (minimum_bound_margin < minimum_joint_limit_margin) {
      continue;
    }
    if (total_travel < best_total_travel) {
      best_total_travel = total_travel;
      best_candidate = candidate;
    }
  }

  if (!best_candidate.has_value()) {
    RCLCPP_ERROR(logger, "No collision-free nearest-IK candidate exists for '%s'.", stage_name.c_str());
    return std::nullopt;
  }
  if (best_total_travel > maximum_total_travel) {
    RCLCPP_ERROR(
      logger,
      "Nearest IK for '%s' still requires %.3f rad total travel (limit %.3f rad); "
      "refusing the visually excessive branch.",
      stage_name.c_str(), best_total_travel, maximum_total_travel);
    return std::nullopt;
  }
  RCLCPP_INFO(
    logger, "Selected nearest IK for '%s': %.3f rad total travel from parked state.",
    stage_name.c_str(), best_total_travel);
  return best_candidate;
}

bool plan_seeded_joint_target_with_retries(
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & joint_target,
  const std::string & stage_name,
  MoveGroupInterface::Plan & output_plan,
  const std::function<bool(const trajectory_msgs::msg::JointTrajectory &)> &
  plan_gate = {})
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  constexpr int kMaximumPlanningAttempts = 3;
  for (int attempt = 1; attempt <= kMaximumPlanningAttempts; ++attempt) {
    if (!set_bounded_start_state(move_group, stage_name)) {
      RCLCPP_WARN(
        logger, "Could not refresh the start state for seeded plan '%s' (attempt %d/%d).",
        stage_name.c_str(), attempt, kMaximumPlanningAttempts);
      continue;
    }
    move_group.clearPoseTargets();
    if (!move_group.setJointValueTarget(joint_target)) {
      RCLCPP_ERROR(logger, "Seeded joint target violates a limit for '%s'.", stage_name.c_str());
      return false;
    }
    MoveGroupInterface::Plan candidate_plan;
    if (move_group.plan(candidate_plan) == moveit::core::MoveItErrorCode::SUCCESS &&
      !candidate_plan.trajectory.joint_trajectory.points.empty())
    {
      if (plan_gate && !plan_gate(candidate_plan.trajectory.joint_trajectory)) {
        RCLCPP_WARN(
          logger, "Seeded joint plan for '%s' failed its path gate (attempt %d/%d).",
          stage_name.c_str(), attempt, kMaximumPlanningAttempts);
        continue;
      }
      output_plan = std::move(candidate_plan);
      if (attempt > 1) {
        RCLCPP_INFO(
          logger, "Seeded joint planning for '%s' recovered on attempt %d/%d.",
          stage_name.c_str(), attempt, kMaximumPlanningAttempts);
      }
      return true;
    }
    RCLCPP_WARN(
      logger, "Seeded joint planning for '%s' failed before execution (attempt %d/%d); "
      "refreshing current state and retrying.",
      stage_name.c_str(), attempt, kMaximumPlanningAttempts);
  }
  RCLCPP_ERROR(
    logger, "Joint planning to the seeded IK target failed for '%s' after %d attempts.",
    stage_name.c_str(), kMaximumPlanningAttempts);
  return false;
}

bool verify_direct_joint_path(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::string & stage_name,
  const double maximum_single_joint_excess = 0.35,
  const double maximum_total_excess = 0.70)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  if (trajectory.points.size() < 2 || trajectory.joint_names.empty()) {
    RCLCPP_ERROR(logger, "Direct-path gate '%s' received an empty trajectory.", stage_name.c_str());
    return false;
  }
  const auto & first = trajectory.points.front().positions;
  const auto & last = trajectory.points.back().positions;
  if (first.size() != trajectory.joint_names.size() ||
    last.size() != trajectory.joint_names.size())
  {
    RCLCPP_ERROR(logger, "Direct-path gate '%s' received malformed endpoints.", stage_name.c_str());
    return false;
  }

  double total_direct_travel = 0.0;
  double total_cumulative_travel = 0.0;
  double maximum_joint_excess = 0.0;
  std::string worst_joint;
  for (size_t joint_index = 0; joint_index < trajectory.joint_names.size(); ++joint_index) {
    double cumulative_travel = 0.0;
    double previous = first[joint_index];
    if (!std::isfinite(previous) || !std::isfinite(last[joint_index])) {
      return false;
    }
    for (size_t point_index = 1; point_index < trajectory.points.size(); ++point_index) {
      const auto & positions = trajectory.points[point_index].positions;
      if (positions.size() != trajectory.joint_names.size() ||
        !std::isfinite(positions[joint_index]))
      {
        return false;
      }
      cumulative_travel += std::abs(positions[joint_index] - previous);
      previous = positions[joint_index];
    }
    const double direct_travel = std::abs(last[joint_index] - first[joint_index]);
    const double excess = std::max(0.0, cumulative_travel - direct_travel);
    total_direct_travel += direct_travel;
    total_cumulative_travel += cumulative_travel;
    if (excess > maximum_joint_excess) {
      maximum_joint_excess = excess;
      worst_joint = trajectory.joint_names[joint_index];
    }
  }

  const double total_excess =
    std::max(0.0, total_cumulative_travel - total_direct_travel);
  const bool accepted = maximum_joint_excess <= maximum_single_joint_excess &&
    total_excess <= maximum_total_excess;
  RCLCPP_INFO(
    logger,
    "Direct path '%s': %zu points, direct/cumulative %.3f/%.3f rad, total excess "
    "%.3f rad, worst '%s' excess %.3f rad (%s).",
    stage_name.c_str(), trajectory.points.size(), total_direct_travel,
    total_cumulative_travel, total_excess, worst_joint.c_str(), maximum_joint_excess,
    accepted ? "accepted" : "rejected");
  return accepted;
}

bool verify_trajectory_joint_bound_margin(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const moveit::core::RobotModelConstPtr & robot_model,
  const double required_margin,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  if (!robot_model || trajectory.points.empty() || trajectory.joint_names.empty()) {
    RCLCPP_ERROR(logger, "Joint-bound gate '%s' lacks a model or trajectory.", stage_name.c_str());
    return false;
  }
  double minimum_margin = std::numeric_limits<double>::infinity();
  std::string closest_joint;
  for (const auto & point : trajectory.points) {
    if (point.positions.size() != trajectory.joint_names.size()) {
      RCLCPP_ERROR(logger, "Joint-bound gate '%s' received a malformed point.", stage_name.c_str());
      return false;
    }
    for (size_t index = 0; index < trajectory.joint_names.size(); ++index) {
      const double position = point.positions[index];
      if (!std::isfinite(position)) {
        return false;
      }
      const auto & bounds = robot_model->getVariableBounds(trajectory.joint_names[index]);
      if (!bounds.position_bounded_) {
        continue;
      }
      const double margin = std::min(
        position - bounds.min_position_, bounds.max_position_ - position);
      if (margin < minimum_margin) {
        minimum_margin = margin;
        closest_joint = trajectory.joint_names[index];
      }
    }
  }
  const bool accepted = minimum_margin >= required_margin;
  RCLCPP_INFO(
    logger, "Joint-bound path '%s': minimum margin %.3f rad at '%s' "
    "(required %.3f rad, %s).",
    stage_name.c_str(), minimum_margin, closest_joint.c_str(), required_margin,
    accepted ? "accepted" : "rejected");
  return accepted;
}

bool plan_and_execute_joint_with_cartesian_continuation(
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & joint_target,
  const geometry_msgs::msg::Pose & continuation_target,
  const std::string & stage_name,
  const std::vector<geometry_msgs::msg::Pose> & branch_probe_targets = {},
  const std::function<bool(const trajectory_msgs::msg::JointTrajectory &)> &
  candidate_gate = {},
  const std::function<bool(const trajectory_msgs::msg::JointTrajectory &)> &
  joint_plan_gate = {},
  const bool execute_motion = true,
  const bool probe_avoid_collisions = false,
  std::map<std::string, double> * branch_probe_endpoint = nullptr)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  MoveGroupInterface::Plan pose_plan;
  if (!plan_seeded_joint_target_with_retries(
      move_group, joint_target, stage_name, pose_plan, joint_plan_gate))
  {
    return false;
  }
  const auto & planned = pose_plan.trajectory.joint_trajectory;
  const auto & planned_endpoint = planned.points.back();
  double maximum_endpoint_error = 0.0;
  for (const auto & [joint_name, expected] : joint_target) {
    const auto found = std::find(planned.joint_names.begin(), planned.joint_names.end(), joint_name);
    if (found == planned.joint_names.end()) {
      RCLCPP_ERROR(logger, "Planned seeded trajectory omitted joint '%s'.", joint_name.c_str());
      return false;
    }
    const auto index = static_cast<size_t>(std::distance(planned.joint_names.begin(), found));
    if (index >= planned_endpoint.positions.size()) {
      RCLCPP_ERROR(logger, "Planned seeded trajectory has a malformed endpoint for '%s'.", stage_name.c_str());
      return false;
    }
    maximum_endpoint_error = std::max(
      maximum_endpoint_error, std::abs(planned_endpoint.positions[index] - expected));
  }
  constexpr double kSeededEndpointTolerance = 1e-4;
  if (maximum_endpoint_error > kSeededEndpointTolerance) {
    RCLCPP_ERROR(
      logger, "Planned endpoint for '%s' differs from seeded IK by %.6f rad.",
      stage_name.c_str(), maximum_endpoint_error);
    return false;
  }

  moveit_msgs::msg::RobotState candidate_start;
  candidate_start.is_diff = true;
  candidate_start.joint_state.name = planned.joint_names;
  candidate_start.joint_state.position = planned_endpoint.positions;
  move_group.setStartState(candidate_start);
  moveit_msgs::msg::RobotTrajectory continuation;
  moveit_msgs::msg::MoveItErrorCodes continuation_error;
  const double fraction = move_group.computeCartesianPath(
    {continuation_target}, kCartesianEefStep, continuation, true, &continuation_error);
  const auto point_count = continuation.joint_trajectory.points.size();
  RCLCPP_INFO(
    logger, "Seeded branch for '%s': collision-checked continuation %.1f%% (%zu points).",
    stage_name.c_str(), fraction * 100.0, point_count);
  if (fraction < kMinCartesianFraction || point_count < 2) {
    RCLCPP_ERROR(logger, "Seeded branch cannot complete the Cartesian continuation for '%s'.", stage_name.c_str());
    return false;
  }
  if (candidate_gate && !candidate_gate(continuation.joint_trajectory)) {
    RCLCPP_ERROR(logger, "Seeded branch failed its endpoint gate for '%s'.", stage_name.c_str());
    return false;
  }

  if (!branch_probe_targets.empty()) {
    moveit_msgs::msg::RobotState probe_start;
    probe_start.is_diff = true;
    probe_start.joint_state.name = continuation.joint_trajectory.joint_names;
    probe_start.joint_state.position = continuation.joint_trajectory.points.back().positions;
    move_group.setStartState(probe_start);
    moveit_msgs::msg::RobotTrajectory branch_probe;
    moveit_msgs::msg::MoveItErrorCodes branch_probe_error;
    // The probe may include the intended final finger contact.  It is never
    // executed; it only proves that this exact IK branch can continuously
    // reach the contact pose and complete the requested Cartesian lift.
    const double probe_fraction = move_group.computeCartesianPath(
      branch_probe_targets, kCartesianEefStep, branch_probe,
      probe_avoid_collisions, &branch_probe_error);
    const auto probe_points = branch_probe.joint_trajectory.points.size();
    RCLCPP_INFO(
      logger, "Seeded branch for '%s': contact-and-lift probe %.1f%% (%zu points).",
      stage_name.c_str(), probe_fraction * 100.0, probe_points);
    if (probe_fraction < kMinCartesianFraction || probe_points < 2) {
      RCLCPP_ERROR(logger, "Seeded branch failed the contact-and-lift probe for '%s'.", stage_name.c_str());
      return false;
    }
    if (candidate_gate && !candidate_gate(branch_probe.joint_trajectory)) {
      RCLCPP_ERROR(logger, "Seeded contact-and-lift probe left its branch for '%s'.", stage_name.c_str());
      // Preserve the full-path rejection above.  Recompute each waypoint
      // segment only to identify where the bound/signature loss occurs; these
      // diagnostic trajectories are never executed and cannot turn a failure
      // into an acceptance.
      moveit_msgs::msg::RobotState segment_start = probe_start;
      for (size_t segment_index = 0;
        segment_index < branch_probe_targets.size(); ++segment_index)
      {
        move_group.setStartState(segment_start);
        moveit_msgs::msg::RobotTrajectory segment_probe;
        moveit_msgs::msg::MoveItErrorCodes segment_error;
        const double segment_fraction = move_group.computeCartesianPath(
          {branch_probe_targets[segment_index]}, kCartesianEefStep,
          segment_probe, probe_avoid_collisions, &segment_error);
        const auto segment_points = segment_probe.joint_trajectory.points.size();
        RCLCPP_INFO(
          logger,
          "Failed-branch diagnostic segment %zu/%zu: %.1f%% (%zu points, error=%d).",
          segment_index + 1, branch_probe_targets.size(),
          segment_fraction * 100.0, segment_points, segment_error.val);
        if (segment_fraction < kMinCartesianFraction || segment_points < 2) {
          break;
        }
        static_cast<void>(candidate_gate(segment_probe.joint_trajectory));
        segment_start.is_diff = true;
        segment_start.joint_state.name = segment_probe.joint_trajectory.joint_names;
        segment_start.joint_state.position =
          segment_probe.joint_trajectory.points.back().positions;
      }
      return false;
    }
    if (branch_probe_endpoint != nullptr) {
      const auto & probe_joints = branch_probe.joint_trajectory;
      const auto & endpoint = probe_joints.points.back();
      if (probe_joints.joint_names.size() != endpoint.positions.size()) {
        RCLCPP_ERROR(
          logger, "Malformed branch-probe endpoint for '%s'.", stage_name.c_str());
        return false;
      }
      branch_probe_endpoint->clear();
      for (size_t index = 0; index < probe_joints.joint_names.size(); ++index) {
        if (!std::isfinite(endpoint.positions[index])) {
          return false;
        }
        branch_probe_endpoint->emplace(
          probe_joints.joint_names[index], endpoint.positions[index]);
      }
    }
  }

  if (!execute_motion) {
    move_group.setStartStateToCurrentState();
    RCLCPP_INFO(
      logger,
      "Seeded branch preflight passed for '%s'; no robot trajectory was executed.",
      stage_name.c_str());
    return true;
  }

  const auto point_as_target = [&logger, &stage_name](
      const trajectory_msgs::msg::JointTrajectory & trajectory,
      const trajectory_msgs::msg::JointTrajectoryPoint & point,
      const std::string & point_name)
    -> std::optional<std::map<std::string, double>>
    {
      if (trajectory.joint_names.size() != point.positions.size()) {
        RCLCPP_ERROR(
          logger, "Malformed %s point for seeded stage '%s'.",
          point_name.c_str(), stage_name.c_str());
        return std::nullopt;
      }
      std::map<std::string, double> target;
      for (size_t index = 0; index < trajectory.joint_names.size(); ++index) {
        if (!std::isfinite(point.positions[index])) {
          RCLCPP_ERROR(
            logger, "Non-finite %s point for seeded stage '%s'.",
            point_name.c_str(), stage_name.c_str());
          return std::nullopt;
        }
        target.emplace(trajectory.joint_names[index], point.positions[index]);
      }
      return target;
    };
  const auto pose_plan_start = point_as_target(
    planned, planned.points.front(), "joint-plan start");
  const auto continuation_start = point_as_target(
    continuation.joint_trajectory, continuation.joint_trajectory.points.front(),
    "Cartesian continuation start");
  const auto continuation_end = point_as_target(
    continuation.joint_trajectory, continuation.joint_trajectory.points.back(),
    "Cartesian continuation end");
  if (!pose_plan_start.has_value() || !continuation_start.has_value() ||
    !continuation_end.has_value())
  {
    return false;
  }

  RCLCPP_INFO(logger, "Executing verified seeded-IK branch for '%s'.", stage_name.c_str());
  if (!joint_target_is_reached(
      move_group, *pose_plan_start, 0.005, stage_name + " live joint-plan start"))
  {
    return false;
  }
  if (move_group.execute(pose_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Seeded joint-plan execution failed at '%s'.", stage_name.c_str());
    return false;
  }
  if (!joint_target_is_reached(
      move_group, *continuation_start, 0.005,
      stage_name + " live Cartesian start"))
  {
    return false;
  }
  if (move_group.execute(continuation) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Cartesian continuation execution failed at '%s'.", stage_name.c_str());
    return false;
  }
  if (!joint_target_is_reached(
      move_group, *continuation_end, 0.005,
      stage_name + " live Cartesian end"))
  {
    return false;
  }
  RCLCPP_INFO(logger, "Seeded branch-selection stage completed: %s", stage_name.c_str());
  return true;
}

bool plan_and_execute_seeded_joint_with_cartesian_probe(
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & joint_target,
  const std::vector<geometry_msgs::msg::Pose> & probe_targets,
  const std::string & stage_name,
  const std::function<bool(const trajectory_msgs::msg::JointTrajectory &)> &
  candidate_gate = {},
  const std::function<bool(const trajectory_msgs::msg::JointTrajectory &)> &
  joint_plan_gate = {})
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  if (probe_targets.empty()) {
    RCLCPP_ERROR(logger, "Seeded stage '%s' requires a Cartesian probe.", stage_name.c_str());
    return false;
  }
  MoveGroupInterface::Plan joint_plan;
  if (!plan_seeded_joint_target_with_retries(
      move_group, joint_target, stage_name, joint_plan, joint_plan_gate))
  {
    return false;
  }
  const auto & planned = joint_plan.trajectory.joint_trajectory;
  const auto & planned_endpoint = planned.points.back();
  double maximum_endpoint_error = 0.0;
  for (const auto & [joint_name, expected] : joint_target) {
    const auto found = std::find(planned.joint_names.begin(), planned.joint_names.end(), joint_name);
    if (found == planned.joint_names.end()) {
      RCLCPP_ERROR(logger, "Planned seeded trajectory omitted joint '%s'.", joint_name.c_str());
      return false;
    }
    const auto index = static_cast<size_t>(std::distance(planned.joint_names.begin(), found));
    if (index >= planned_endpoint.positions.size()) {
      RCLCPP_ERROR(logger, "Planned seeded trajectory has a malformed endpoint for '%s'.", stage_name.c_str());
      return false;
    }
    maximum_endpoint_error = std::max(
      maximum_endpoint_error, std::abs(planned_endpoint.positions[index] - expected));
  }
  constexpr double kSeededEndpointTolerance = 1e-4;
  if (maximum_endpoint_error > kSeededEndpointTolerance) {
    RCLCPP_ERROR(
      logger, "Planned endpoint for '%s' differs from seeded IK by %.6f rad.",
      stage_name.c_str(), maximum_endpoint_error);
    return false;
  }

  moveit_msgs::msg::RobotState probe_start;
  probe_start.is_diff = true;
  probe_start.joint_state.name = planned.joint_names;
  probe_start.joint_state.position = planned_endpoint.positions;
  move_group.setStartState(probe_start);
  moveit_msgs::msg::RobotTrajectory probe;
  moveit_msgs::msg::MoveItErrorCodes probe_error;
  // Contact with the real block is intentional.  This trajectory is never
  // executed here; it only proves that the exact seeded branch can enter the
  // grasp and then continue through the requested lift without a branch jump.
  const double probe_fraction = move_group.computeCartesianPath(
    probe_targets, kCartesianEefStep, probe, false, &probe_error);
  const auto probe_points = probe.joint_trajectory.points.size();
  RCLCPP_INFO(
    logger, "Seeded branch for '%s': contact-and-lift probe %.1f%% (%zu points).",
    stage_name.c_str(), probe_fraction * 100.0, probe_points);
  if (probe_fraction < kMinCartesianFraction || probe_points < 2) {
    // The physical sequence deliberately stops at contact to close the
    // gripper before starting the vertical lift.  A single multi-waypoint IK
    // interpolation can lose its seed across that stop even when both real
    // execution segments are individually complete.  Never execute the
    // partial combined probe; instead require every physical segment to solve
    // independently at the same 99.5% gate and retain the same branch gate.
    RCLCPP_WARN(
      logger,
      "Combined contact-and-lift probe for '%s' solved only %.1f%% (error=%d); "
      "checking the two physically separated Cartesian stages independently.",
      stage_name.c_str(), probe_fraction * 100.0, probe_error.val);
    moveit_msgs::msg::RobotState segment_start = probe_start;
    for (size_t segment_index = 0; segment_index < probe_targets.size(); ++segment_index) {
      move_group.setStartState(segment_start);
      moveit_msgs::msg::RobotTrajectory segment_probe;
      moveit_msgs::msg::MoveItErrorCodes segment_error;
      const double segment_fraction = move_group.computeCartesianPath(
        {probe_targets[segment_index]}, kCartesianEefStep,
        segment_probe, false, &segment_error);
      const auto segment_points = segment_probe.joint_trajectory.points.size();
      RCLCPP_INFO(
        logger,
        "Physical-stage probe %zu/%zu for '%s': %.1f%% (%zu points, error=%d).",
        segment_index + 1, probe_targets.size(), stage_name.c_str(),
        segment_fraction * 100.0, segment_points, segment_error.val);
      if (segment_fraction < kMinCartesianFraction || segment_points < 2 ||
        (candidate_gate && !candidate_gate(segment_probe.joint_trajectory)))
      {
        RCLCPP_ERROR(
          logger,
          "Physical-stage probe %zu/%zu failed for '%s'; no partial path will execute.",
          segment_index + 1, probe_targets.size(), stage_name.c_str());
        move_group.setStartStateToCurrentState();
        return false;
      }
      segment_start.is_diff = true;
      segment_start.joint_state.name = segment_probe.joint_trajectory.joint_names;
      segment_start.joint_state.position =
        segment_probe.joint_trajectory.points.back().positions;
    }
    move_group.setStartStateToCurrentState();
    RCLCPP_INFO(
      logger,
      "All %zu physically separated Cartesian probes passed for '%s'; the partial "
      "combined probe remains unexecuted.",
      probe_targets.size(), stage_name.c_str());
  } else if (candidate_gate && !candidate_gate(probe.joint_trajectory)) {
    RCLCPP_ERROR(logger, "Seeded contact-and-lift probe left its branch for '%s'.", stage_name.c_str());
    return false;
  }

  if (planned.joint_names.size() != planned.points.front().positions.size()) {
    RCLCPP_ERROR(logger, "Malformed joint-plan start for seeded stage '%s'.", stage_name.c_str());
    return false;
  }
  std::map<std::string, double> planned_start;
  for (size_t index = 0; index < planned.joint_names.size(); ++index) {
    const double position = planned.points.front().positions[index];
    if (!std::isfinite(position)) {
      RCLCPP_ERROR(logger, "Non-finite joint-plan start for seeded stage '%s'.", stage_name.c_str());
      return false;
    }
    planned_start.emplace(planned.joint_names[index], position);
  }

  move_group.setStartStateToCurrentState();
  if (!joint_target_is_reached(
      move_group, planned_start, 0.005, stage_name + " live joint-plan start"))
  {
    return false;
  }
  RCLCPP_INFO(logger, "Executing verified seeded-IK target for '%s'.", stage_name.c_str());
  if (move_group.execute(joint_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Seeded joint-plan execution failed at '%s'.", stage_name.c_str());
    return false;
  }
  if (!joint_target_is_reached(
      move_group, joint_target, 0.005, stage_name + " live seeded endpoint"))
  {
    return false;
  }
  RCLCPP_INFO(logger, "Seeded target stage completed: %s", stage_name.c_str());
  return true;
}

bool plan_and_execute_cartesian(
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target,
  const std::string & stage_name,
  const bool fallback_to_pose_planning = false,
  const std::function<bool(const trajectory_msgs::msg::JointTrajectory &)> &
  candidate_gate = {},
  const std::vector<geometry_msgs::msg::Pose> & intermediate_waypoints = {},
  const bool avoid_collisions = true)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  if (!set_bounded_start_state(move_group, stage_name)) {
    return false;
  }
  move_group.clearPoseTargets();

  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  auto waypoints = intermediate_waypoints;
  waypoints.push_back(target);
  RCLCPP_INFO(
    logger, "Planning Cartesian stage: %s (%zu waypoint(s), eef_step=%.3f m)",
    stage_name.c_str(), waypoints.size(), kCartesianEefStep);
  const double fraction = move_group.computeCartesianPath(
    waypoints, kCartesianEefStep, trajectory, avoid_collisions, &error_code);
  const auto point_count = trajectory.joint_trajectory.points.size();

  if (fraction < kMinCartesianFraction || point_count == 0) {
    if (fallback_to_pose_planning) {
      RCLCPP_WARN(
        logger,
        "Cartesian stage '%s' solved %.1f%% (%zu points, error=%d); "
        "using collision-aware OMPL before executing any partial path.",
        stage_name.c_str(), fraction * 100.0, point_count, error_code.val);
      return plan_and_execute_pose(move_group, target, stage_name + " (OMPL fallback)");
    }
    RCLCPP_ERROR(
      logger,
      "Cartesian stage '%s' failed: fraction=%.1f%%, points=%zu, error=%d.",
      stage_name.c_str(), fraction * 100.0, point_count, error_code.val);
    return false;
  }

  if (candidate_gate && !candidate_gate(trajectory.joint_trajectory)) {
    RCLCPP_ERROR(logger, "Cartesian stage '%s' left its required IK branch.", stage_name.c_str());
    return false;
  }

  const auto point_as_target = [&logger, &stage_name](
      const trajectory_msgs::msg::JointTrajectory & joint_trajectory,
      const trajectory_msgs::msg::JointTrajectoryPoint & point,
      const std::string & point_name)
    -> std::optional<std::map<std::string, double>>
    {
      if (joint_trajectory.joint_names.size() != point.positions.size()) {
        RCLCPP_ERROR(
          logger, "Malformed %s point for Cartesian stage '%s'.",
          point_name.c_str(), stage_name.c_str());
        return std::nullopt;
      }
      std::map<std::string, double> result;
      for (size_t index = 0; index < joint_trajectory.joint_names.size(); ++index) {
        if (!std::isfinite(point.positions[index])) {
          RCLCPP_ERROR(
            logger, "Non-finite %s point for Cartesian stage '%s'.",
            point_name.c_str(), stage_name.c_str());
          return std::nullopt;
        }
        result.emplace(joint_trajectory.joint_names[index], point.positions[index]);
      }
      return result;
    };
  const auto live_start = point_as_target(
    trajectory.joint_trajectory, trajectory.joint_trajectory.points.front(), "start");
  const auto live_end = point_as_target(
    trajectory.joint_trajectory, trajectory.joint_trajectory.points.back(), "end");
  if (!live_start.has_value() || !live_end.has_value()) {
    return false;
  }

  RCLCPP_INFO(
    logger, "Cartesian stage '%s' solved %.1f%% with %zu points; executing...",
    stage_name.c_str(), fraction * 100.0, point_count);
  if (!joint_target_is_reached(
      move_group, *live_start, 0.005, stage_name + " live Cartesian start"))
  {
    return false;
  }
  if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed at Cartesian stage '%s'.", stage_name.c_str());
    return false;
  }
  if (!joint_target_is_reached(
      move_group, *live_end, 0.005, stage_name + " live Cartesian end"))
  {
    return false;
  }
  RCLCPP_INFO(logger, "Cartesian stage completed: %s", stage_name.c_str());
  return true;
}

void diagnose_cartesian_collision(
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  auto common_state = move_group.getCurrentState(10.0);
  if (!common_state || !validity_client->wait_for_service(3s)) {
    RCLCPP_WARN(logger, "Cannot run collision diagnostics for '%s'.", stage_name.c_str());
    return;
  }
  common_state->update();
  move_group.setStartState(*common_state);
  moveit_msgs::msg::RobotTrajectory diagnostic_path;
  moveit_msgs::msg::MoveItErrorCodes diagnostic_error;
  const double diagnostic_fraction = move_group.computeCartesianPath(
    {target}, kCartesianEefStep, diagnostic_path, false, &diagnostic_error);
  move_group.setStartStateToCurrentState();
  const auto & joint_path = diagnostic_path.joint_trajectory;
  RCLCPP_INFO(
    logger,
    "Collision diagnostic '%s': kinematic-only path %.1f%% (%zu points, error=%d).",
    stage_name.c_str(), diagnostic_fraction * 100.0,
    joint_path.points.size(), diagnostic_error.val);
  if (diagnostic_fraction < kMinCartesianFraction || joint_path.points.empty()) {
    RCLCPP_WARN(
      logger,
      "The failed Cartesian path is kinematically incomplete, so no collision pair can "
      "explain the full failure for '%s'.",
      stage_name.c_str());
    return;
  }

  auto checked_state = *common_state;
  for (size_t point_index = 0; point_index < joint_path.points.size(); ++point_index) {
    const auto & point = joint_path.points[point_index];
    if (point.positions.size() != joint_path.joint_names.size()) {
      return;
    }
    checked_state.setVariablePositions(joint_path.joint_names, point.positions);
    checked_state.update();
    auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    request->group_name = "";
    request->robot_state.is_diff = true;
    request->robot_state.joint_state.name = checked_state.getVariableNames();
    request->robot_state.joint_state.position.reserve(
      request->robot_state.joint_state.name.size());
    for (const auto & variable_name : request->robot_state.joint_state.name) {
      request->robot_state.joint_state.position.push_back(
        checked_state.getVariablePosition(variable_name));
    }
    auto future = validity_client->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      RCLCPP_WARN(
        logger, "Collision diagnostic timed out at point %zu/%zu for '%s'.",
        point_index + 1, joint_path.points.size(), stage_name.c_str());
      return;
    }
    const auto response = future.get();
    if (response->valid) {
      continue;
    }
    RCLCPP_ERROR(
      logger,
      "Collision diagnostic found the first invalid state at point %zu/%zu (%.1f%%) "
      "for '%s'; %zu contacts reported.",
      point_index + 1, joint_path.points.size(),
      100.0 * static_cast<double>(point_index) /
      static_cast<double>(std::max<size_t>(1, joint_path.points.size() - 1)),
      stage_name.c_str(), response->contacts.size());
    const size_t contact_count = std::min<size_t>(response->contacts.size(), 8);
    for (size_t contact_index = 0; contact_index < contact_count; ++contact_index) {
      const auto & contact = response->contacts[contact_index];
      RCLCPP_ERROR(
        logger, "  contact %zu: '%s' <-> '%s', depth %.2f mm.",
        contact_index + 1, contact.contact_body_1.c_str(), contact.contact_body_2.c_str(),
        contact.depth * 1000.0);
    }
    return;
  }
  RCLCPP_WARN(
    logger,
    "Every sampled state of the kinematic-only path was valid for '%s'; the partial result "
    "is likely an IK branch-continuity issue rather than collision.",
    stage_name.c_str());
}

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond orientation(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  orientation.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = orientation.toRotationMatrix();
  return transform;
}

geometry_msgs::msg::Pose eigen_to_pose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond orientation(transform.linear());
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

std::optional<geometry_msgs::msg::Pose> link_pose_in_frame(
  const moveit::core::RobotState & state,
  const std::string & link_name,
  const std::string & target_frame,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  geometry_msgs::msg::PoseStamped model_pose;
  model_pose.header.frame_id = state.getRobotModel()->getModelFrame();
  model_pose.pose = eigen_to_pose(state.getGlobalLinkTransform(link_name));
  try {
    geometry_msgs::msg::PoseStamped target_pose;
    tf_buffer->transform(
      model_pose, target_pose, target_frame, tf2::durationFromSec(0.2));
    return target_pose.pose;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger, "Cannot transform link '%s' from '%s' to '%s': %s",
      link_name.c_str(), model_pose.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

std::optional<geometry_msgs::msg::Pose> current_link_pose_in_frame(
  MoveGroupInterface & move_group,
  const std::string & link_name,
  const std::string & target_frame,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  const auto state = move_group.getCurrentState(10.0);
  if (!state) {
    RCLCPP_ERROR(
      rclcpp::get_logger("h1_2_air_handover"),
      "No current state is available while reading link '%s'.", link_name.c_str());
    return std::nullopt;
  }
  state->update();
  return link_pose_in_frame(*state, link_name, target_frame, tf_buffer);
}

void fill_robot_state_request(
  const moveit::core::RobotState & state,
  moveit_msgs::msg::RobotState & request_state)
{
  request_state.is_diff = true;
  request_state.joint_state.name = state.getVariableNames();
  request_state.joint_state.position.clear();
  request_state.joint_state.position.reserve(request_state.joint_state.name.size());
  for (const auto & variable_name : request_state.joint_state.name) {
    request_state.joint_state.position.push_back(state.getVariablePosition(variable_name));
  }
}

std::shared_ptr<moveit_msgs::srv::GetStateValidity::Response> check_full_state_validity(
  const moveit::core::RobotState & state,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client)
{
  if (!validity_client->wait_for_service(3s)) {
    return nullptr;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  request->group_name = "";
  fill_robot_state_request(state, request->robot_state);
  auto future = validity_client->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    return nullptr;
  }
  return future.get();
}

bool apply_allowed_collision_matrix(
  const moveit_msgs::msg::AllowedCollisionMatrix & matrix,
  const rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr & apply_scene_client)
{
  if (!apply_scene_client->wait_for_service(3s)) {
    return false;
  }
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene.is_diff = true;
  request->scene.robot_state.is_diff = true;
  request->scene.allowed_collision_matrix = matrix;
  auto future = apply_scene_client->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    return false;
  }
  return future.get()->success;
}

std::optional<moveit_msgs::msg::AllowedCollisionMatrix> read_allowed_collision_matrix(
  const rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr & get_scene_client)
{
  if (!get_scene_client->wait_for_service(3s)) {
    return std::nullopt;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto future = get_scene_client->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    return std::nullopt;
  }
  return future.get()->scene.allowed_collision_matrix;
}

bool set_allowed_collision_pair(
  moveit_msgs::msg::AllowedCollisionMatrix & matrix,
  const std::string & first,
  const std::string & second,
  const bool allowed)
{
  const auto ensure_entry = [&matrix](const std::string & name) -> std::optional<size_t> {
      const size_t original_size = matrix.entry_names.size();
      if (matrix.entry_values.size() != original_size) {
        return std::nullopt;
      }
      for (const auto & row : matrix.entry_values) {
        if (row.enabled.size() != original_size) {
          return std::nullopt;
        }
      }
      const auto existing = std::find(matrix.entry_names.begin(), matrix.entry_names.end(), name);
      if (existing != matrix.entry_names.end()) {
        return static_cast<size_t>(std::distance(matrix.entry_names.begin(), existing));
      }
      for (auto & row : matrix.entry_values) {
        row.enabled.push_back(false);
      }
      moveit_msgs::msg::AllowedCollisionEntry new_row;
      new_row.enabled.resize(original_size + 1, false);
      matrix.entry_names.push_back(name);
      matrix.entry_values.push_back(std::move(new_row));
      return original_size;
    };

  const auto first_index = ensure_entry(first);
  const auto second_index = ensure_entry(second);
  if (!first_index.has_value() || !second_index.has_value()) {
    return false;
  }
  matrix.entry_values[*first_index].enabled[*second_index] = allowed;
  matrix.entry_values[*second_index].enabled[*first_index] = allowed;
  return true;
}

bool recover_left_from_table_safety_margin_if_needed(
  MoveGroupInterface & left_arm,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  const rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr & get_scene_client,
  const rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr & apply_scene_client,
  bool & recovery_performed)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  recovery_performed = false;
  auto start_state = left_arm.getCurrentState(10.0);
  if (!start_state) {
    RCLCPP_ERROR(logger, "No current state is available for left table-margin recovery.");
    return false;
  }
  start_state->update();
  const auto initial_validity = check_full_state_validity(*start_state, validity_client);
  if (!initial_validity) {
    RCLCPP_ERROR(logger, "Cannot query the current full-state validity before recovery.");
    return false;
  }
  if (initial_validity->valid) {
    return true;
  }

  constexpr char table_id[] = "environment_table_10mm_margin";
  const std::set<std::string> approved_left_finger_links = {
    "left_dex1_finger_1_link", "left_dex1_finger_1_pad_link",
    "left_dex1_finger_2_link", "left_dex1_finger_2_pad_link"};
  std::set<std::string> colliding_left_finger_links;
  if (initial_validity->contacts.empty()) {
    RCLCPP_ERROR(
      logger, "The current state is invalid but MoveIt reported no contacts; refusing recovery.");
    return false;
  }
  for (const auto & contact : initial_validity->contacts) {
    const bool first_is_table = contact.contact_body_1 == table_id;
    const bool second_is_table = contact.contact_body_2 == table_id;
    const std::string & other = first_is_table ? contact.contact_body_2 : contact.contact_body_1;
    if (first_is_table == second_is_table ||
      approved_left_finger_links.find(other) == approved_left_finger_links.end() ||
      !std::isfinite(contact.depth) || contact.depth < 0.0 || contact.depth > 0.010)
    {
      RCLCPP_ERROR(
        logger,
        "Recovery refused unexpected contact '%s' <-> '%s' at depth %.2f mm.",
        contact.contact_body_1.c_str(), contact.contact_body_2.c_str(),
        contact.depth * 1000.0);
      return false;
    }
    colliding_left_finger_links.insert(other);
  }
  RCLCPP_WARN(
    logger,
    "Current state lies only inside the 10 mm table safety shell at %zu approved left-finger "
    "link(s); preparing one-way vertical exit recovery.",
    colliding_left_finger_links.size());

  if (!get_scene_client->wait_for_service(3s)) {
    RCLCPP_ERROR(logger, "GetPlanningScene is unavailable for temporary recovery allowance.");
    return false;
  }
  auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  scene_request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto scene_future = get_scene_client->async_send_request(scene_request);
  if (scene_future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(logger, "Timed out while reading the authoritative collision matrix.");
    return false;
  }
  const auto original_matrix = scene_future.get()->scene.allowed_collision_matrix;
  auto recovery_matrix = original_matrix;
  for (const auto & link_name : colliding_left_finger_links) {
    if (!set_allowed_collision_pair(recovery_matrix, table_id, link_name, true)) {
      RCLCPP_ERROR(logger, "Malformed collision matrix; refusing temporary recovery allowance.");
      return false;
    }
  }
  if (!apply_allowed_collision_matrix(recovery_matrix, apply_scene_client)) {
    RCLCPP_ERROR(logger, "Failed to apply the temporary table-exit collision allowance.");
    return false;
  }

  bool success = false;
  const auto allowed_validity = check_full_state_validity(*start_state, validity_client);
  if (!allowed_validity || !allowed_validity->valid) {
    RCLCPP_ERROR(
      logger,
      "The recovery allowance did not make the current state valid against all other "
      "self, OctoMap and environment collisions.");
  } else {
    const auto start_pose = link_pose_in_frame(
      *start_state, "left_dex1_tcp", kPlanningFrame, tf_buffer);
    if (start_pose.has_value()) {
      auto target_pose = *start_pose;
      target_pose.position.z += kLeftStartupRecoveryClearance;
      const Eigen::Quaterniond start_orientation(
        start_pose->orientation.w, start_pose->orientation.x,
        start_pose->orientation.y, start_pose->orientation.z);
      const auto monotonic_vertical_gate =
        [start_state, start_pose, start_orientation, tf_buffer](
        const trajectory_msgs::msg::JointTrajectory & trajectory) -> bool
        {
          auto checked_state = *start_state;
          double previous_z = start_pose->position.z;
          double final_z = previous_z;
          for (const auto & point : trajectory.points) {
            if (point.positions.size() != trajectory.joint_names.size()) {
              return false;
            }
            checked_state.setVariablePositions(trajectory.joint_names, point.positions);
            checked_state.update();
            const auto pose = link_pose_in_frame(
              checked_state, "left_dex1_tcp", kPlanningFrame, tf_buffer);
            if (!pose.has_value()) {
              return false;
            }
            const Eigen::Quaterniond orientation(
              pose->orientation.w, pose->orientation.x,
              pose->orientation.y, pose->orientation.z);
            if (std::abs(pose->position.x - start_pose->position.x) > 0.001 ||
              std::abs(pose->position.y - start_pose->position.y) > 0.001 ||
              pose->position.z + 0.00025 < previous_z ||
              start_orientation.angularDistance(orientation) > 0.5 * M_PI / 180.0)
            {
              return false;
            }
            previous_z = pose->position.z;
            final_z = pose->position.z;
          }
          const double rise = final_z - start_pose->position.z;
          return rise >= kLeftStartupRecoveryClearance - 0.001 &&
                 rise <= kLeftStartupRecoveryClearance + 0.001;
        };
      success = plan_and_execute_cartesian(
        left_arm, target_pose,
        "startup safety-shell exit: raise empty left receiver 120 mm",
        false, monotonic_vertical_gate);
    }
  }

  const bool restored = apply_allowed_collision_matrix(original_matrix, apply_scene_client);
  if (!restored) {
    RCLCPP_ERROR(
      logger, "CRITICAL: failed to restore the authoritative collision matrix after recovery.");
    return false;
  }
  if (!success) {
    RCLCPP_ERROR(logger, "One-way table safety-shell exit recovery did not execute.");
    return false;
  }

  auto final_state = left_arm.getCurrentState(10.0);
  if (!final_state) {
    return false;
  }
  final_state->update();
  const auto final_validity = check_full_state_validity(*final_state, validity_client);
  if (!final_validity || !final_validity->valid) {
    RCLCPP_ERROR(
      logger,
      "The 10 mm table safety gate was restored, but the recovered endpoint is not fully valid.");
    return false;
  }
  recovery_performed = true;
  RCLCPP_INFO(
    logger,
    "Left receiver exited vertically by 120 mm; the full 10 mm table gate is restored and "
    "the endpoint is valid.");
  return true;
}

bool plan_and_execute_supported_object_vertical_segment(
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target,
  const bool leaving_support,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  const rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr & get_scene_client,
  const rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr & apply_scene_client,
  const std::string & stage_name,
  const double required_distance = 0.015)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  constexpr char table_id[] = "environment_table_10mm_margin";
  constexpr char octomap_id[] = "<octomap>";
  const std::set<std::string> approved_environment = {table_id, octomap_id};

  auto start_state = move_group.getCurrentState(10.0);
  if (!start_state) {
    RCLCPP_ERROR(logger, "No current state is available for supported segment '%s'.", stage_name.c_str());
    return false;
  }
  start_state->update();
  const std::string tcp_link =
    move_group.getName() == "left_arm" ? "left_dex1_tcp" : "right_dex1_tcp";
  const auto start_pose = link_pose_in_frame(
    *start_state, tcp_link, kPlanningFrame, tf_buffer);
  if (!start_pose.has_value()) {
    return false;
  }
  const double vertical_delta = target.position.z - start_pose->position.z;
  if (!std::isfinite(required_distance) || required_distance < 0.001 ||
    required_distance > 0.015 ||
    std::abs(std::abs(vertical_delta) - required_distance) > 0.001 ||
    std::abs(target.position.x - start_pose->position.x) > 0.001 ||
    std::abs(target.position.y - start_pose->position.y) > 0.001)
  {
    RCLCPP_ERROR(
      logger, "Supported segment '%s' is not the required %.1f mm vertical motion.",
      stage_name.c_str(), required_distance * 1000.0);
    return false;
  }
  if ((leaving_support && vertical_delta <= 0.0) ||
    (!leaving_support && vertical_delta >= 0.0))
  {
    RCLCPP_ERROR(logger, "Supported segment '%s' has the wrong vertical direction.", stage_name.c_str());
    return false;
  }

  const auto contact_environment = [&approved_environment](
      const moveit_msgs::msg::ContactInformation & contact) -> std::optional<std::string> {
      if (contact.contact_body_1 == kObjectId &&
        approved_environment.count(contact.contact_body_2) != 0)
      {
        return contact.contact_body_2;
      }
      if (contact.contact_body_2 == kObjectId &&
        approved_environment.count(contact.contact_body_1) != 0)
      {
        return contact.contact_body_1;
      }
      return std::nullopt;
    };
  const auto validate_contact_set =
    [&contact_environment, &logger, &stage_name](
      const std::shared_ptr<moveit_msgs::srv::GetStateValidity::Response> & response,
      std::map<std::string, double> * depths) -> bool
    {
      if (!response) {
        return false;
      }
      if (depths) {
        depths->clear();
      }
      if (response->valid) {
        return true;
      }
      if (response->contacts.empty()) {
        RCLCPP_ERROR(
          logger, "Invalid state without contacts in supported segment '%s'.",
          stage_name.c_str());
        return false;
      }
      for (const auto & contact : response->contacts) {
        const auto environment = contact_environment(contact);
        if (!environment.has_value() || !std::isfinite(contact.depth) ||
          contact.depth < 0.0 || contact.depth > 0.016)
        {
          RCLCPP_ERROR(
            logger,
            "Supported segment '%s' rejected contact '%s' <-> '%s' at %.2f mm.",
            stage_name.c_str(), contact.contact_body_1.c_str(),
            contact.contact_body_2.c_str(), contact.depth * 1000.0);
          return false;
        }
        if (depths) {
          auto & maximum_depth = (*depths)[*environment];
          maximum_depth = std::max(maximum_depth, contact.depth);
        }
      }
      return true;
    };

  std::map<std::string, double> previous_depths;
  const auto initial_validity = check_full_state_validity(*start_state, validity_client);
  if (!validate_contact_set(initial_validity, &previous_depths)) {
    return false;
  }
  if (leaving_support && initial_validity->valid) {
    RCLCPP_ERROR(
      logger, "Supported exit '%s' has no existing table contact to justify an allowance.",
      stage_name.c_str());
    return false;
  }
  if (!leaving_support && !initial_validity->valid) {
    RCLCPP_ERROR(
      logger, "Supported entry '%s' must begin outside the 10 mm safety shell.",
      stage_name.c_str());
    return false;
  }

  if (!set_bounded_start_state(move_group, stage_name)) {
    return false;
  }
  move_group.clearPoseTargets();
  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  const double fraction = move_group.computeCartesianPath(
    {target}, kCartesianEefStep, trajectory, false, &error_code);
  const auto & joint_path = trajectory.joint_trajectory;
  if (fraction < kMinCartesianFraction || joint_path.points.size() < 2) {
    RCLCPP_ERROR(
      logger, "Supported segment '%s' is kinematically incomplete: %.1f%%, %zu points.",
      stage_name.c_str(), fraction * 100.0, joint_path.points.size());
    return false;
  }

  auto checked_state = *start_state;
  const Eigen::Quaterniond start_orientation(
    start_pose->orientation.w, start_pose->orientation.x,
    start_pose->orientation.y, start_pose->orientation.z);
  double previous_z = start_pose->position.z;
  for (size_t point_index = 0; point_index < joint_path.points.size(); ++point_index) {
    const auto & point = joint_path.points[point_index];
    if (point.positions.size() != joint_path.joint_names.size()) {
      return false;
    }
    checked_state.setVariablePositions(joint_path.joint_names, point.positions);
    checked_state.update();
    const auto pose = link_pose_in_frame(
      checked_state, tcp_link, kPlanningFrame, tf_buffer);
    if (!pose.has_value()) {
      return false;
    }
    const Eigen::Quaterniond orientation(
      pose->orientation.w, pose->orientation.x,
      pose->orientation.y, pose->orientation.z);
    const bool z_is_monotonic = leaving_support ?
      pose->position.z + 0.00025 >= previous_z :
      pose->position.z <= previous_z + 0.00025;
    if (!z_is_monotonic ||
      std::abs(pose->position.x - start_pose->position.x) > 0.001 ||
      std::abs(pose->position.y - start_pose->position.y) > 0.001 ||
      start_orientation.angularDistance(orientation) > 0.5 * M_PI / 180.0)
    {
      RCLCPP_ERROR(
        logger, "Supported segment '%s' violated its fixed-pose vertical gate at point %zu.",
        stage_name.c_str(), point_index + 1);
      return false;
    }
    previous_z = pose->position.z;

    std::map<std::string, double> point_depths;
    const auto point_validity = check_full_state_validity(checked_state, validity_client);
    if (!validate_contact_set(point_validity, &point_depths)) {
      return false;
    }
    if (leaving_support) {
      for (const auto & [environment, depth] : point_depths) {
        const auto previous = previous_depths.find(environment);
        if (previous == previous_depths.end() || depth > previous->second + 0.00075) {
          RCLCPP_ERROR(
            logger,
            "Supported exit '%s' increased '%s' contact depth from %.2f to %.2f mm.",
            stage_name.c_str(), environment.c_str(),
            previous == previous_depths.end() ? 0.0 : previous->second * 1000.0,
            depth * 1000.0);
          return false;
        }
      }
      previous_depths = std::move(point_depths);
    } else {
      const double progress = std::abs(
        (pose->position.z - start_pose->position.z) / vertical_delta);
      if (!point_validity->valid && progress < 0.25) {
        RCLCPP_ERROR(
          logger, "Supported entry '%s' met the environment too early at %.1f%%.",
          stage_name.c_str(), progress * 100.0);
        return false;
      }
    }
  }
  if (std::abs(previous_z - target.position.z) > 0.001) {
    return false;
  }

  if (!get_scene_client->wait_for_service(3s)) {
    return false;
  }
  auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  scene_request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto scene_future = get_scene_client->async_send_request(scene_request);
  if (scene_future.wait_for(5s) != std::future_status::ready) {
    return false;
  }
  const auto original_matrix = scene_future.get()->scene.allowed_collision_matrix;
  auto supported_matrix = original_matrix;
  for (const auto & environment : approved_environment) {
    if (!set_allowed_collision_pair(supported_matrix, kObjectId, environment, true)) {
      return false;
    }
  }
  if (!apply_allowed_collision_matrix(supported_matrix, apply_scene_client)) {
    return false;
  }

  const auto point_as_target = [&joint_path](
      const trajectory_msgs::msg::JointTrajectoryPoint & point)
    -> std::map<std::string, double>
    {
      std::map<std::string, double> result;
      for (size_t index = 0; index < joint_path.joint_names.size(); ++index) {
        result.emplace(joint_path.joint_names[index], point.positions[index]);
      }
      return result;
    };
  bool success = joint_target_is_reached(
    move_group, point_as_target(joint_path.points.front()), 0.005,
    stage_name + " live supported start");
  if (success) {
    RCLCPP_INFO(
      logger,
      "Supported segment '%s' passed 100%% offline geometry/contact validation with %zu points; "
      "executing.",
      stage_name.c_str(), joint_path.points.size());
    success = move_group.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
  }
  if (success) {
    success = joint_target_is_reached(
      move_group, point_as_target(joint_path.points.back()), 0.005,
      stage_name + " live supported end");
  }
  const bool restored = apply_allowed_collision_matrix(original_matrix, apply_scene_client);
  if (!restored) {
    RCLCPP_ERROR(logger, "CRITICAL: failed to restore the collision matrix after '%s'.", stage_name.c_str());
    return false;
  }
  if (!success) {
    return false;
  }

  auto final_state = move_group.getCurrentState(10.0);
  if (!final_state) {
    return false;
  }
  final_state->update();
  const auto final_validity = check_full_state_validity(*final_state, validity_client);
  if (!validate_contact_set(final_validity, nullptr)) {
    return false;
  }
  if (leaving_support && !final_validity->valid) {
    RCLCPP_ERROR(
      logger, "Supported exit '%s' did not finish outside the full 10 mm safety shell.",
      stage_name.c_str());
    return false;
  }
  if (!leaving_support && final_validity->valid) {
    RCLCPP_ERROR(
      logger, "Supported entry '%s' did not reach the modeled table support shell.",
      stage_name.c_str());
    return false;
  }
  RCLCPP_INFO(
    logger, "Supported segment '%s' completed and the authoritative collision matrix is restored.",
    stage_name.c_str());
  return true;
}

std::optional<geometry_msgs::msg::Pose> transform_pose_to_planning_frame(
  const geometry_msgs::msg::Pose & world_pose,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  geometry_msgs::msg::PoseStamped source;
  source.header.frame_id = "world";
  source.pose = world_pose;
  try {
    geometry_msgs::msg::PoseStamped target;
    tf_buffer->transform(source, target, kPlanningFrame, tf2::durationFromSec(0.2));
    return target.pose;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger, "Cannot transform Gazebo object pose into %s: %s", kPlanningFrame, ex.what());
    return std::nullopt;
  }
}

bool compute_complete_cartesian(
  MoveGroupInterface & move_group,
  const moveit::core::RobotState & common_start,
  const geometry_msgs::msg::Pose & target,
  const std::string & stage_name,
  moveit_msgs::msg::RobotTrajectory & trajectory)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  // Both component paths must be solved from the exact same robot state.
  // Re-reading the state here lets controller feedback advance between the
  // right and left requests and breaks the point-by-point pairing invariant.
  move_group.setStartState(common_start);
  move_group.clearPoseTargets();

  moveit_msgs::msg::MoveItErrorCodes error_code;
  // Each component is intentionally solved without collision checking because
  // the opposite arm must move at the same time.  The merged 14-joint states
  // are checked against the full planning scene before anything is executed.
  const double fraction = move_group.computeCartesianPath(
    {target}, kCartesianEefStep, trajectory, false, &error_code);
  const auto point_count = trajectory.joint_trajectory.points.size();
  if (fraction < kMinCartesianFraction || point_count < 2) {
    RCLCPP_ERROR(
      logger,
      "Synchronized Cartesian component '%s' failed: fraction=%.1f%%, points=%zu, error=%d.",
      stage_name.c_str(), fraction * 100.0, point_count, error_code.val);
    return false;
  }
  RCLCPP_INFO(
    logger, "Synchronized component '%s' solved %.1f%% with %zu points.",
    stage_name.c_str(), fraction * 100.0, point_count);
  return true;
}

bool build_synchronized_cartesian_trajectory(
  MoveGroupInterface & right_arm,
  MoveGroupInterface & left_arm,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  const JointMotionLimits & motion_limits,
  const double vertical_distance,
  const double duration_seconds,
  const double velocity_scaling,
  const double acceleration_scaling,
  const std::string & stage_name,
  moveit_msgs::msg::RobotTrajectory & combined)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  auto common_start = right_arm.getCurrentState(10.0);
  if (!common_start) {
    RCLCPP_ERROR(logger, "No common current state is available for '%s'.", stage_name.c_str());
    return false;
  }
  common_start->update();

  // getCurrentPose() is expressed in RobotModel::getModelFrame() ("world"
  // for this fixed-base SRDF), regardless of setPoseReferenceFrame().  Derive
  // both TCP poses from this one shared state and retain the frame while
  // transforming them, otherwise a 5 mm lift becomes roughly a 1 m
  // world/pelvis offset.
  const auto right_start_pose = link_pose_in_frame(
    *common_start, "right_dex1_tcp", kPlanningFrame, tf_buffer);
  const auto left_start_pose = link_pose_in_frame(
    *common_start, "left_dex1_tcp", kPlanningFrame, tf_buffer);
  if (!right_start_pose.has_value() || !left_start_pose.has_value()) {
    return false;
  }
  auto right_target = *right_start_pose;
  auto left_target = *left_start_pose;
  right_target.position.z += vertical_distance;
  left_target.position.z += vertical_distance;
  RCLCPP_INFO(
    logger,
    "Synchronized '%s' starts in %s at right [%.3f, %.3f, %.3f], "
    "left [%.3f, %.3f, %.3f]; vertical delta %.1f mm.",
    stage_name.c_str(), kPlanningFrame,
    right_start_pose->position.x, right_start_pose->position.y, right_start_pose->position.z,
    left_start_pose->position.x, left_start_pose->position.y, left_start_pose->position.z,
    vertical_distance * 1000.0);

  moveit_msgs::msg::RobotTrajectory right_trajectory;
  moveit_msgs::msg::RobotTrajectory left_trajectory;
  if (!compute_complete_cartesian(
      right_arm, *common_start, right_target, stage_name + " / right", right_trajectory) ||
    !compute_complete_cartesian(
      left_arm, *common_start, left_target, stage_name + " / left", left_trajectory))
  {
    return false;
  }

  const auto & right_joint_trajectory = right_trajectory.joint_trajectory;
  const auto & left_joint_trajectory = left_trajectory.joint_trajectory;
  const size_t right_point_count = right_joint_trajectory.points.size();
  const size_t left_point_count = left_joint_trajectory.points.size();
  const auto component_is_well_formed = [](const auto & joint_trajectory) {
      if (joint_trajectory.points.size() < 2 || joint_trajectory.joint_names.empty()) {
        return false;
      }
      return std::all_of(
        joint_trajectory.points.begin(), joint_trajectory.points.end(),
        [&joint_trajectory](const auto & point) {
          return point.positions.size() == joint_trajectory.joint_names.size() &&
                 std::all_of(point.positions.begin(), point.positions.end(),
                   [](const double value) {return std::isfinite(value);});
        });
    };
  if (!component_is_well_formed(right_joint_trajectory) ||
    !component_is_well_formed(left_joint_trajectory))
  {
    RCLCPP_ERROR(logger, "Malformed Cartesian component for synchronized stage '%s'.", stage_name.c_str());
    return false;
  }

  const auto same_joint_set = [](const auto & first, const auto & second) {
      return std::set<std::string>(first.begin(), first.end()) ==
             std::set<std::string>(second.begin(), second.end());
    };
  if (!same_joint_set(right_joint_trajectory.joint_names, right_arm.getActiveJoints()) ||
    !same_joint_set(left_joint_trajectory.joint_names, left_arm.getActiveJoints()))
  {
    RCLCPP_ERROR(logger, "Cartesian component joint names do not match their planning groups for '%s'.", stage_name.c_str());
    return false;
  }

  std::set<std::string> unique_joint_names;
  for (const auto & joint_name : right_joint_trajectory.joint_names) {
    if (!unique_joint_names.insert(joint_name).second) {
      RCLCPP_ERROR(logger, "Duplicate right-arm joint '%s' in '%s'.", joint_name.c_str(), stage_name.c_str());
      return false;
    }
  }
  for (const auto & joint_name : left_joint_trajectory.joint_names) {
    if (!unique_joint_names.insert(joint_name).second) {
      RCLCPP_ERROR(logger, "Duplicate cross-arm joint '%s' in '%s'.", joint_name.c_str(), stage_name.c_str());
      return false;
    }
  }

  const auto rotation_distance = [](const geometry_msgs::msg::Pose & first,
      const geometry_msgs::msg::Pose & second) {
      const Eigen::Matrix3d delta =
        pose_to_eigen(first).linear().transpose() * pose_to_eigen(second).linear();
      return Eigen::AngleAxisd(delta).angle();
    };
  const auto translation_distance = [](const geometry_msgs::msg::Pose & first,
      const geometry_msgs::msg::Pose & second) {
      return (pose_to_eigen(first).translation() - pose_to_eigen(second).translation()).norm();
    };

  const auto extract_cartesian_progress = [&](
      const auto & joint_trajectory, const std::string & link_name,
      const geometry_msgs::msg::Pose & start_pose,
      const geometry_msgs::msg::Pose & target_pose,
      std::vector<double> & progresses) {
      constexpr double kEndpointTranslationTolerance = 0.001;
      constexpr double kPathTranslationTolerance = 0.001;
      constexpr double kOrientationTolerance = 0.008726646259971648;
      constexpr double kStartJointTolerance = 0.005;
      constexpr double kMaximumSourceJointStep = 0.35;
      constexpr double kProgressTolerance = 0.05;
      constexpr double kMinimumProgressStep = 1e-6;
      auto state = *common_start;
      progresses.clear();
      progresses.reserve(joint_trajectory.points.size());
      double previous_progress = -std::numeric_limits<double>::infinity();
      for (size_t index = 0; index < joint_trajectory.points.size(); ++index) {
        const auto & point = joint_trajectory.points[index];
        if (index == 0) {
          for (size_t joint_index = 0; joint_index < joint_trajectory.joint_names.size(); ++joint_index) {
            const double start_error = std::abs(
              point.positions[joint_index] -
              common_start->getVariablePosition(joint_trajectory.joint_names[joint_index]));
            if (start_error > kStartJointTolerance) {
              RCLCPP_ERROR(
                logger, "Cartesian source '%s' starts %.4f rad away from the common state at joint '%s'.",
                link_name.c_str(), start_error,
                joint_trajectory.joint_names[joint_index].c_str());
              return false;
            }
          }
        } else {
          const auto & previous_point = joint_trajectory.points[index - 1];
          for (size_t joint_index = 0; joint_index < point.positions.size(); ++joint_index) {
            const double joint_step = std::abs(
              point.positions[joint_index] - previous_point.positions[joint_index]);
            if (joint_step > kMaximumSourceJointStep) {
              RCLCPP_ERROR(
                logger, "Cartesian source '%s' has a %.3f rad IK branch jump at point %zu.",
                link_name.c_str(), joint_step, index + 1);
              return false;
            }
          }
        }

        state.setVariablePositions(joint_trajectory.joint_names, point.positions);
        state.update();
        if (!state.satisfiesBounds(1e-6)) {
          RCLCPP_ERROR(
            logger, "Cartesian source '%s' is outside robot bounds at point %zu.",
            link_name.c_str(), index + 1);
          return false;
        }
        const auto pose = link_pose_in_frame(state, link_name, kPlanningFrame, tf_buffer);
        if (!pose.has_value()) {
          return false;
        }
        const double raw_progress =
          (pose->position.z - start_pose.position.z) / vertical_distance;
        if (!std::isfinite(raw_progress) || raw_progress < -kProgressTolerance ||
          raw_progress > 1.0 + kProgressTolerance)
        {
          RCLCPP_ERROR(
            logger, "Cartesian source '%s' has invalid vertical progress %.6f at point %zu.",
            link_name.c_str(), raw_progress, index + 1);
          return false;
        }
        const bool is_first = index == 0;
        const bool is_last = index + 1 == joint_trajectory.points.size();
        if ((is_first && std::abs(raw_progress) > kProgressTolerance) ||
          (is_last && std::abs(raw_progress - 1.0) > kProgressTolerance) ||
          (!is_first && !is_last &&
          (raw_progress <= kMinimumProgressStep ||
          raw_progress >= 1.0 - kMinimumProgressStep)))
        {
          RCLCPP_ERROR(
            logger,
            "Cartesian source '%s' has out-of-order endpoint/interior progress %.6f at point %zu.",
            link_name.c_str(), raw_progress, index + 1);
          return false;
        }
        if (index > 0 && raw_progress <= previous_progress + kMinimumProgressStep) {
          RCLCPP_ERROR(
            logger, "Cartesian source '%s' is not strictly monotonic at point %zu (%.6f <= %.6f).",
            link_name.c_str(), index + 1, raw_progress, previous_progress);
          return false;
        }
        geometry_msgs::msg::Pose expected = start_pose;
        expected.position.z += raw_progress * vertical_distance;
        const double path_translation_error = translation_distance(expected, *pose);
        const double path_rotation_error = rotation_distance(start_pose, *pose);
        if (path_translation_error > kPathTranslationTolerance ||
          path_rotation_error > kOrientationTolerance)
        {
          RCLCPP_ERROR(
            logger,
            "Cartesian source '%s' leaves its vertical fixed-orientation line by %.2f mm / %.2f deg at point %zu.",
            link_name.c_str(), path_translation_error * 1000.0,
            path_rotation_error * 57.29577951308232, index + 1);
          return false;
        }
        progresses.push_back(raw_progress);
        previous_progress = raw_progress;
      }

      if (std::abs(progresses.front()) > kProgressTolerance ||
        std::abs(progresses.back() - 1.0) > kProgressTolerance)
      {
        RCLCPP_ERROR(
          logger, "Cartesian source '%s' does not span the full command (%.4f -> %.4f).",
          link_name.c_str(), progresses.front(), progresses.back());
        return false;
      }
      auto endpoint_state = *common_start;
      endpoint_state.setVariablePositions(
        joint_trajectory.joint_names, joint_trajectory.points.front().positions);
      endpoint_state.update();
      const auto actual_start = link_pose_in_frame(
        endpoint_state, link_name, kPlanningFrame, tf_buffer);
      endpoint_state.setVariablePositions(
        joint_trajectory.joint_names, joint_trajectory.points.back().positions);
      endpoint_state.update();
      const auto actual_target = link_pose_in_frame(
        endpoint_state, link_name, kPlanningFrame, tf_buffer);
      if (!actual_start.has_value() || !actual_target.has_value() ||
        translation_distance(*actual_start, start_pose) > kEndpointTranslationTolerance ||
        translation_distance(*actual_target, target_pose) > kEndpointTranslationTolerance ||
        rotation_distance(*actual_start, start_pose) > kOrientationTolerance ||
        rotation_distance(*actual_target, target_pose) > kOrientationTolerance)
      {
        RCLCPP_ERROR(logger, "Cartesian source '%s' endpoint validation failed for '%s'.", link_name.c_str(), stage_name.c_str());
        return false;
      }
      progresses.front() = 0.0;
      progresses.back() = 1.0;
      for (size_t index = 1; index < progresses.size(); ++index) {
        if (progresses[index] <= progresses[index - 1] + kMinimumProgressStep) {
          RCLCPP_ERROR(
            logger,
            "Cartesian source '%s' loses strict ordering after endpoint normalization at point %zu.",
            link_name.c_str(), index + 1);
          return false;
        }
      }
      return true;
    };

  std::vector<double> right_progresses;
  std::vector<double> left_progresses;
  if (!extract_cartesian_progress(
      right_joint_trajectory, "right_dex1_tcp", *right_start_pose,
      right_target, right_progresses) ||
    !extract_cartesian_progress(
      left_joint_trajectory, "left_dex1_tcp", *left_start_pose,
      left_target, left_progresses))
  {
    return false;
  }

  const auto interpolate_positions = [](const auto & joint_trajectory,
      const std::vector<double> & progresses, const double progress) {
      if (progress <= progresses.front()) {
        return joint_trajectory.points.front().positions;
      }
      if (progress >= progresses.back()) {
        return joint_trajectory.points.back().positions;
      }
      const auto upper = std::upper_bound(progresses.begin(), progresses.end(), progress);
      const size_t upper_index = static_cast<size_t>(std::distance(progresses.begin(), upper));
      const size_t lower_index = upper_index - 1;
      const double span = progresses[upper_index] - progresses[lower_index];
      const double alpha = (progress - progresses[lower_index]) / span;
      std::vector<double> positions(joint_trajectory.joint_names.size());
      for (size_t joint_index = 0; joint_index < positions.size(); ++joint_index) {
        const double lower = joint_trajectory.points[lower_index].positions[joint_index];
        const double upper = joint_trajectory.points[upper_index].positions[joint_index];
        positions[joint_index] = lower + alpha * (upper - lower);
      }
      return positions;
    };

  std::vector<double> common_progresses = right_progresses;
  common_progresses.insert(
    common_progresses.end(), left_progresses.begin(), left_progresses.end());
  std::sort(common_progresses.begin(), common_progresses.end());
  std::vector<double> progress_union;
  constexpr double kProgressMergeTolerance = 1e-6;
  for (const double progress : common_progresses) {
    if (progress_union.empty() || progress - progress_union.back() > kProgressMergeTolerance) {
      progress_union.push_back(progress);
    }
  }
  if (progress_union.size() < 2 || std::abs(progress_union.front()) > kProgressMergeTolerance ||
    std::abs(progress_union.back() - 1.0) > kProgressMergeTolerance)
  {
    RCLCPP_ERROR(logger, "Cannot construct a complete shared Cartesian progress grid for '%s'.", stage_name.c_str());
    return false;
  }

  constexpr double kDenseCartesianStep = 0.0005;
  constexpr double kDenseJointStep = 0.008726646259971648;
  std::vector<double> dense_progresses;
  for (size_t segment = 0; segment + 1 < progress_union.size(); ++segment) {
    const double start_progress = progress_union[segment];
    const double end_progress = progress_union[segment + 1];
    const double progress_span = end_progress - start_progress;
    if (progress_span <= 0.0) {
      RCLCPP_ERROR(logger, "Non-increasing shared progress grid for '%s'.", stage_name.c_str());
      return false;
    }
    const auto right_start_positions = interpolate_positions(
      right_joint_trajectory, right_progresses, start_progress);
    const auto right_end_positions = interpolate_positions(
      right_joint_trajectory, right_progresses, end_progress);
    const auto left_start_positions = interpolate_positions(
      left_joint_trajectory, left_progresses, start_progress);
    const auto left_end_positions = interpolate_positions(
      left_joint_trajectory, left_progresses, end_progress);
    double maximum_joint_step = 0.0;
    for (size_t index = 0; index < right_start_positions.size(); ++index) {
      maximum_joint_step = std::max(
        maximum_joint_step,
        std::abs(right_end_positions[index] - right_start_positions[index]));
    }
    for (size_t index = 0; index < left_start_positions.size(); ++index) {
      maximum_joint_step = std::max(
        maximum_joint_step,
        std::abs(left_end_positions[index] - left_start_positions[index]));
    }
    const size_t cartesian_subdivisions = static_cast<size_t>(std::ceil(
        std::abs(vertical_distance) * progress_span / kDenseCartesianStep));
    const size_t joint_subdivisions = static_cast<size_t>(std::ceil(
        maximum_joint_step / kDenseJointStep));
    const size_t subdivisions = std::max<size_t>(
      1, std::max(cartesian_subdivisions, joint_subdivisions));
    const size_t first_subdivision = segment == 0 ? 0 : 1;
    for (size_t subdivision = first_subdivision; subdivision <= subdivisions; ++subdivision) {
      const double alpha = static_cast<double>(subdivision) /
        static_cast<double>(subdivisions);
      dense_progresses.push_back(start_progress + alpha * progress_span);
    }
  }

  combined = moveit_msgs::msg::RobotTrajectory();
  auto & output = combined.joint_trajectory;
  output.joint_names = right_joint_trajectory.joint_names;
  output.joint_names.insert(
    output.joint_names.end(),
    left_joint_trajectory.joint_names.begin(), left_joint_trajectory.joint_names.end());
  output.points.reserve(progress_union.size());

  // The joint trajectory controller uses quintic interpolation whenever each
  // waypoint contains position, velocity and acceleration.  Put both
  // derivatives at zero on every preserved source knot.  Within one shared
  // knot interval, quintic smoothstep then traverses the exact same straight
  // joint-space segment for both arms, so the dense geometric checks below
  // cover the controller's complete path rather than just its message points.
  // Segment durations are enlarged analytically until the peak velocity and
  // acceleration of that quintic are inside the requested scaled limits.
  constexpr double kQuinticPeakVelocity = 1.875;
  constexpr double kQuinticPeakAcceleration = 5.773502691896258;
  double cumulative_duration = 0.0;
  for (size_t index = 0; index < progress_union.size(); ++index) {
    const double progress = progress_union[index];
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = interpolate_positions(
      right_joint_trajectory, right_progresses, progress);
    const auto left_positions = interpolate_positions(
      left_joint_trajectory, left_progresses, progress);
    point.positions.insert(
      point.positions.end(), left_positions.begin(), left_positions.end());
    point.velocities.assign(output.joint_names.size(), 0.0);
    point.accelerations.assign(output.joint_names.size(), 0.0);

    if (index > 0) {
      double segment_duration = duration_seconds *
        (progress_union[index] - progress_union[index - 1]);
      for (size_t joint_index = 0; joint_index < output.joint_names.size(); ++joint_index) {
        const auto limit = motion_limits.find(output.joint_names[joint_index]);
        if (limit == motion_limits.end()) {
          RCLCPP_ERROR(
            logger, "Joint '%s' lacks loaded motion limits for synchronized stage '%s'.",
            output.joint_names[joint_index].c_str(), stage_name.c_str());
          return false;
        }
        const double delta =
          point.positions[joint_index] - output.points.back().positions[joint_index];
        const double directional_velocity_limit =
          velocity_scaling * limit->second.max_velocity;
        const double symmetric_acceleration_limit =
          acceleration_scaling * limit->second.max_acceleration;
        if (!std::isfinite(directional_velocity_limit) ||
          !std::isfinite(symmetric_acceleration_limit) ||
          directional_velocity_limit <= 0.0 || symmetric_acceleration_limit <= 0.0)
        {
          RCLCPP_ERROR(
            logger, "Joint '%s' has unusable scaled motion limits for '%s'.",
            output.joint_names[joint_index].c_str(), stage_name.c_str());
          return false;
        }
        const double distance = std::abs(delta);
        segment_duration = std::max(
          segment_duration, kQuinticPeakVelocity * distance / directional_velocity_limit);
        segment_duration = std::max(
          segment_duration,
          std::sqrt(kQuinticPeakAcceleration * distance / symmetric_acceleration_limit));
      }
      if (!std::isfinite(segment_duration) || segment_duration <= 0.0) {
        RCLCPP_ERROR(
          logger, "Invalid synchronized quintic duration at segment %zu for '%s'.",
          index, stage_name.c_str());
        return false;
      }
      cumulative_duration += segment_duration;
    }
    point.time_from_start = rclcpp::Duration::from_seconds(cumulative_duration);
    output.points.push_back(std::move(point));
  }

  if (!validity_client->wait_for_service(3s)) {
    RCLCPP_ERROR(logger, "State-validity service is unavailable for '%s'.", stage_name.c_str());
    return false;
  }

  // Validate the exact joint-space line traversed by every controller quintic
  // segment.  The dense grid preserves every original IK fold, bounds each
  // checked TCP step to 0.5 mm (or each joint step to 0.5 degree), and fixes
  // every non-arm joint to the measured common state.
  auto paired_state = *common_start;
  const Eigen::Isometry3d initial_relative =
    paired_state.getGlobalLinkTransform("right_dex1_tcp").inverse() *
    paired_state.getGlobalLinkTransform("left_dex1_tcp");
  for (size_t index = 0; index < dense_progresses.size(); ++index) {
    auto dense_positions = interpolate_positions(
      right_joint_trajectory, right_progresses, dense_progresses[index]);
    const auto dense_left_positions = interpolate_positions(
      left_joint_trajectory, left_progresses, dense_progresses[index]);
    dense_positions.insert(
      dense_positions.end(), dense_left_positions.begin(), dense_left_positions.end());
    paired_state.setVariablePositions(output.joint_names, dense_positions);
    paired_state.update();
    if (!paired_state.satisfiesBounds(1e-6)) {
      RCLCPP_ERROR(logger, "Synchronized dense point %zu/%zu is outside robot bounds for '%s'.",
        index + 1, dense_progresses.size(), stage_name.c_str());
      return false;
    }
    const auto right_pose = link_pose_in_frame(
      paired_state, "right_dex1_tcp", kPlanningFrame, tf_buffer);
    const auto left_pose = link_pose_in_frame(
      paired_state, "left_dex1_tcp", kPlanningFrame, tf_buffer);
    if (!right_pose.has_value() || !left_pose.has_value()) {
      return false;
    }
    auto expected_right = *right_start_pose;
    auto expected_left = *left_start_pose;
    expected_right.position.z += dense_progresses[index] * vertical_distance;
    expected_left.position.z += dense_progresses[index] * vertical_distance;
    const double right_translation_error = translation_distance(*right_pose, expected_right);
    const double left_translation_error = translation_distance(*left_pose, expected_left);
    const double right_rotation_error = rotation_distance(*right_pose, expected_right);
    const double left_rotation_error = rotation_distance(*left_pose, expected_left);
    if (right_translation_error > 0.001 || left_translation_error > 0.001 ||
      right_rotation_error > 0.008726646259971648 ||
      left_rotation_error > 0.008726646259971648)
    {
      RCLCPP_ERROR(
        logger,
        "Synchronized dense point %zu/%zu leaves the absolute Cartesian lines: "
        "right %.2f mm / %.2f deg, left %.2f mm / %.2f deg for '%s'.",
        index + 1, dense_progresses.size(),
        right_translation_error * 1000.0, right_rotation_error * 57.29577951308232,
        left_translation_error * 1000.0, left_rotation_error * 57.29577951308232,
        stage_name.c_str());
      return false;
    }
    const Eigen::Isometry3d relative =
      paired_state.getGlobalLinkTransform("right_dex1_tcp").inverse() *
      paired_state.getGlobalLinkTransform("left_dex1_tcp");
    const double translation_drift =
      (relative.translation() - initial_relative.translation()).norm();
    const Eigen::Matrix3d rotation_delta =
      initial_relative.linear().transpose() * relative.linear();
    const double angular_drift = Eigen::AngleAxisd(rotation_delta).angle();
    if (translation_drift > 0.002 || angular_drift > 0.017453292519943295) {
      RCLCPP_ERROR(
        logger,
        "Synchronized point %zu/%zu changes the TCP pair by %.2f mm / %.2f deg for '%s'.",
        index + 1, dense_progresses.size(), translation_drift * 1000.0,
        angular_drift * 57.29577951308232, stage_name.c_str());
      return false;
    }

    auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    request->group_name = "";
    request->robot_state.is_diff = true;
    request->robot_state.joint_state.name = paired_state.getVariableNames();
    request->robot_state.joint_state.position.reserve(
      request->robot_state.joint_state.name.size());
    for (const auto & variable_name : request->robot_state.joint_state.name) {
      request->robot_state.joint_state.position.push_back(
        paired_state.getVariablePosition(variable_name));
    }
    auto future = validity_client->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready) {
      RCLCPP_ERROR(
        logger, "State-validity check timed out at synchronized point %zu for '%s'.",
        index, stage_name.c_str());
      return false;
    }
    if (!future.get()->valid) {
      RCLCPP_ERROR(
        logger, "Synchronized point %zu/%zu is in collision or outside bounds for '%s'.",
        index + 1, dense_progresses.size(), stage_name.c_str());
      return false;
    }
  }

  RCLCPP_INFO(
    logger,
    "Synchronized stage '%s' merged right=%zu and left=%zu source points into "
    "%zu shared quintic knots, densely validated %zu path states, and retimed to %.2f s.",
    stage_name.c_str(), right_point_count, left_point_count,
    progress_union.size(), dense_progresses.size(), cumulative_duration);
  return true;
}

bool execute_synchronized_trajectory(
  MoveGroupInterface & both_arms,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  if (trajectory.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(logger, "Synchronized stage '%s' has no points.", stage_name.c_str());
    return false;
  }
  const auto current = both_arms.getCurrentState(10.0);
  if (!current) {
    RCLCPP_ERROR(logger, "No current state is available before '%s'.", stage_name.c_str());
    return false;
  }
  const auto & first = trajectory.joint_trajectory.points.front();
  constexpr double kMaximumStartError = 0.003;
  for (size_t index = 0; index < trajectory.joint_trajectory.joint_names.size(); ++index) {
    const auto & joint_name = trajectory.joint_trajectory.joint_names[index];
    const double error = std::abs(
      current->getVariablePosition(joint_name) - first.positions[index]);
    if (error > kMaximumStartError) {
      RCLCPP_ERROR(
        logger,
        "Synchronized stage '%s' start differs by %.4f rad at '%s' (limit %.4f rad).",
        stage_name.c_str(), error, joint_name.c_str(), kMaximumStartError);
      return false;
    }
  }
  RCLCPP_INFO(
    logger, "Executing synchronized stage '%s' through both arm controllers...",
    stage_name.c_str());
  if (both_arms.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
    both_arms.stop();
    RCLCPP_ERROR(logger, "Synchronized execution failed at '%s'.", stage_name.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "Synchronized stage completed: %s", stage_name.c_str());
  return true;
}

moveit_msgs::msg::RobotTrajectory reverse_synchronized_trajectory(
  const moveit_msgs::msg::RobotTrajectory & forward)
{
  moveit_msgs::msg::RobotTrajectory reverse;
  reverse.joint_trajectory.joint_names = forward.joint_trajectory.joint_names;
  const size_t point_count = forward.joint_trajectory.points.size();
  reverse.joint_trajectory.points.reserve(point_count);
  if (point_count == 0) {
    return reverse;
  }
  const rclcpp::Duration total_duration(
    forward.joint_trajectory.points.back().time_from_start);
  for (size_t index = 0; index < point_count; ++index) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    const auto & source = forward.joint_trajectory.points[point_count - 1 - index];
    point.positions = source.positions;
    point.velocities = source.velocities;
    for (auto & velocity : point.velocities) {
      velocity = -velocity;
    }
    point.accelerations = source.accelerations;
    // The forward synchronized grid is deliberately non-uniform because it
    // preserves every source IK knot.  Mirror its exact timestamps so every
    // reverse segment has the same duration, velocity and acceleration
    // magnitude as the already validated forward segment.
    point.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
      total_duration - rclcpp::Duration(source.time_from_start));
    reverse.joint_trajectory.points.push_back(std::move(point));
  }
  return reverse;
}

geometry_msgs::msg::Pose make_object_pose(
  const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::Pose make_downward_tcp_pose(
  const geometry_msgs::msg::Pose & object_pose,
  const double y_offset,
  const double z_offset,
  const double left_outward_yaw = 0.0,
  const double right_outward_yaw = 0.0)
{
  geometry_msgs::msg::Pose pose = object_pose;
  pose.position.y += y_offset;
  pose.position.z += z_offset;
  const double yaw = y_offset < 0.0 ? right_outward_yaw : left_outward_yaw;
  const Eigen::Quaterniond orientation =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-1.5707963267948966, Eigen::Vector3d::UnitX());
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

geometry_msgs::msg::Pose make_object_aligned_downward_tcp_pose(
  const geometry_msgs::msg::Pose & object_pose,
  const double y_offset,
  const double z_offset,
  const double left_outward_yaw = 0.0,
  const double right_outward_yaw = 0.0)
{
  const Eigen::Isometry3d object_transform = pose_to_eigen(object_pose);
  geometry_msgs::msg::Pose pose = object_pose;
  const double object_yaw = std::atan2(
    object_transform.linear()(1, 0), object_transform.linear()(0, 0));
  const Eigen::Matrix3d planar_object_rotation =
    Eigen::AngleAxisd(object_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d world_offset =
    planar_object_rotation * Eigen::Vector3d(0.0, y_offset, z_offset);
  pose.position.x += world_offset.x();
  pose.position.y += world_offset.y();
  pose.position.z += world_offset.z();

  // Tabletop depth/PCA orientation is stable before either hand occludes the
  // block.  Use only its planar yaw: keep the TCP vertical while aligning the
  // jaws with the physical 80 mm object axis and interpreting y_offset in the
  // object's own 100 mm axis.
  const double outward_yaw = y_offset < 0.0 ? right_outward_yaw : left_outward_yaw;
  const Eigen::Quaterniond orientation =
    Eigen::AngleAxisd(object_yaw + outward_yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-1.5707963267948966, Eigen::Vector3d::UnitX());
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

void translate_pose_in_object_planar_frame(
  geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Pose & object_pose,
  const double local_x,
  const double local_y,
  const double local_z)
{
  const Eigen::Matrix3d object_rotation = pose_to_eigen(object_pose).linear();
  const double object_yaw = std::atan2(object_rotation(1, 0), object_rotation(0, 0));
  const Eigen::Vector3d world_offset =
    Eigen::AngleAxisd(object_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    Eigen::Vector3d(local_x, local_y, local_z);
  pose.position.x += world_offset.x();
  pose.position.y += world_offset.y();
  pose.position.z += world_offset.z();
}

geometry_msgs::msg::Pose make_inward_tcp_pose(
  const geometry_msgs::msg::Pose & object_pose,
  const double y_offset,
  const double z_offset,
  const double downward_tilt)
{
  const double half_tilt = downward_tilt / 2.0;
  const double sine = std::sin(half_tilt);
  const double cosine = std::cos(half_tilt);
  Eigen::Quaterniond relative_orientation;
  if (y_offset < 0.0) {
    // Right side: local +Y points inward (+Y) and downward.  The base is both
    // farther outside (-Y) and higher than the contact point.
    relative_orientation = Eigen::Quaterniond(cosine, -sine, 0.0, 0.0);
  } else {
    // Left side is the exact Rz(pi) mirror of the right-hand orientation.
    relative_orientation = Eigen::Quaterniond(0.0, 0.0, -sine, cosine);
  }
  Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
  relative.translation() = Eigen::Vector3d(0.0, y_offset, z_offset);
  relative.linear() = relative_orientation.toRotationMatrix();
  return eigen_to_pose(pose_to_eigen(object_pose) * relative);
}

moveit_msgs::msg::CollisionObject make_collision_object(
  const geometry_msgs::msg::Pose & pose,
  const double size_x,
  const double size_y,
  const double size_z)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = kPlanningFrame;
  object.id = kObjectId;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size_x, size_y, size_z};
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

std::vector<std::string> hand_touch_links(const std::string & side)
{
  return {
    side + "_wrist_yaw_link",
    side + "_dex1_adapter_link",
    side + "_dex1_base_link",
    side + "_dex1_finger_1_slider_link",
    side + "_dex1_finger_1_link",
    side + "_dex1_finger_1_pad_link",
    side + "_dex1_finger_2_slider_link",
    side + "_dex1_finger_2_link",
    side + "_dex1_finger_2_pad_link",
    side + "_dex1_tcp",
  };
}

std::vector<std::string> both_hand_touch_links()
{
  auto links = hand_touch_links("right");
  const auto left_links = hand_touch_links("left");
  links.insert(links.end(), left_links.begin(), left_links.end());
  return links;
}

std::vector<std::string> finger_touch_links(const std::string & side)
{
  return {
    side + "_dex1_finger_1_slider_link",
    side + "_dex1_finger_1_link",
    side + "_dex1_finger_1_pad_link",
    side + "_dex1_finger_2_slider_link",
    side + "_dex1_finger_2_link",
    side + "_dex1_finger_2_pad_link",
    side + "_dex1_tcp",
  };
}

std::vector<std::string> both_finger_touch_links()
{
  auto links = finger_touch_links("right");
  const auto left_links = finger_touch_links("left");
  links.insert(links.end(), left_links.begin(), left_links.end());
  return links;
}

std::optional<double> minimum_shape_world_z(
  const shapes::ShapeConstPtr & shape,
  const Eigen::Isometry3d & world_transform)
{
  if (!shape) {
    return std::nullopt;
  }
  const Eigen::Vector3d local_world_z =
    world_transform.linear().transpose() * Eigen::Vector3d::UnitZ();
  const double center_z = world_transform.translation().z();
  switch (shape->type) {
    case shapes::SPHERE:
      return center_z - static_cast<const shapes::Sphere *>(shape.get())->radius;
    case shapes::CYLINDER:
    {
      const auto * cylinder = static_cast<const shapes::Cylinder *>(shape.get());
      const double radial_projection = cylinder->radius * std::hypot(
        local_world_z.x(), local_world_z.y());
      const double axial_projection =
        0.5 * cylinder->length * std::abs(local_world_z.z());
      return center_z - radial_projection - axial_projection;
    }
    case shapes::CONE:
    {
      const auto * cone = static_cast<const shapes::Cone *>(shape.get());
      const double tip_projection = 0.5 * cone->length * local_world_z.z();
      const double base_projection =
        -0.5 * cone->length * local_world_z.z() -
        cone->radius * std::hypot(local_world_z.x(), local_world_z.y());
      return center_z + std::min(tip_projection, base_projection);
    }
    case shapes::BOX:
    {
      const auto * box = static_cast<const shapes::Box *>(shape.get());
      const double projection_radius = 0.5 * (
        std::abs(local_world_z.x()) * box->size[0] +
        std::abs(local_world_z.y()) * box->size[1] +
        std::abs(local_world_z.z()) * box->size[2]);
      return center_z - projection_radius;
    }
    case shapes::MESH:
    {
      const auto * mesh = static_cast<const shapes::Mesh *>(shape.get());
      if (mesh->vertex_count == 0 || mesh->vertices == nullptr) {
        return std::nullopt;
      }
      double minimum_z = std::numeric_limits<double>::infinity();
      for (unsigned int index = 0; index < mesh->vertex_count; ++index) {
        const Eigen::Vector3d vertex(
          mesh->vertices[index * 3], mesh->vertices[index * 3 + 1],
          mesh->vertices[index * 3 + 2]);
        minimum_z = std::min(minimum_z, (world_transform * vertex).z());
      }
      return minimum_z;
    }
    default:
      return std::nullopt;
  }
}

std::optional<double> minimum_hand_table_clearance(
  const moveit::core::RobotState & state,
  const double table_surface_z)
{
  double minimum_z = std::numeric_limits<double>::infinity();
  bool found_collision_geometry = false;
  auto links = hand_touch_links("right");
  const auto left_links = hand_touch_links("left");
  links.insert(links.end(), left_links.begin(), left_links.end());
  for (const auto & link_name : links) {
    const auto * link_model = state.getRobotModel()->getLinkModel(link_name);
    if (!link_model) {
      return std::nullopt;
    }
    const auto & shapes = link_model->getShapes();
    const auto & origins = link_model->getCollisionOriginTransforms();
    if (shapes.size() != origins.size()) {
      return std::nullopt;
    }
    const auto & link_transform = state.getGlobalLinkTransform(link_model);
    for (size_t index = 0; index < shapes.size(); ++index) {
      const auto shape_z = minimum_shape_world_z(
        shapes[index], link_transform * origins[index]);
      if (!shape_z.has_value()) {
        return std::nullopt;
      }
      minimum_z = std::min(minimum_z, *shape_z);
      found_collision_geometry = true;
    }
  }
  if (!found_collision_geometry || !std::isfinite(minimum_z)) {
    return std::nullopt;
  }
  return minimum_z - table_surface_z;
}

struct CenterClearanceReport
{
  double hand_to_hand{std::numeric_limits<double>::infinity()};
  double hand_to_table{std::numeric_limits<double>::infinity()};
  std::string nearest_right_link;
  std::string nearest_left_link;
};

std::shared_ptr<fcl::CollisionGeometryd> fcl_geometry_for_shape(
  const shapes::ShapeConstPtr & shape)
{
  static std::map<const shapes::Shape *, std::shared_ptr<fcl::CollisionGeometryd>> cache;
  if (!shape) {
    return nullptr;
  }
  const auto cached = cache.find(shape.get());
  if (cached != cache.end()) {
    return cached->second;
  }

  std::shared_ptr<fcl::CollisionGeometryd> geometry;
  switch (shape->type) {
    case shapes::SPHERE:
      geometry = std::make_shared<fcl::Sphered>(
        static_cast<const shapes::Sphere *>(shape.get())->radius);
      break;
    case shapes::CYLINDER:
    {
      const auto * cylinder = static_cast<const shapes::Cylinder *>(shape.get());
      geometry = std::make_shared<fcl::Cylinderd>(cylinder->radius, cylinder->length);
      break;
    }
    case shapes::CONE:
    {
      const auto * cone = static_cast<const shapes::Cone *>(shape.get());
      geometry = std::make_shared<fcl::Coned>(cone->radius, cone->length);
      break;
    }
    case shapes::BOX:
    {
      const auto * box = static_cast<const shapes::Box *>(shape.get());
      geometry = std::make_shared<fcl::Boxd>(box->size[0], box->size[1], box->size[2]);
      break;
    }
    case shapes::MESH:
    {
      const auto * mesh = static_cast<const shapes::Mesh *>(shape.get());
      if (mesh->vertex_count == 0 || mesh->triangle_count == 0 ||
        mesh->vertices == nullptr || mesh->triangles == nullptr)
      {
        return nullptr;
      }
      std::vector<fcl::Vector3d> vertices;
      vertices.reserve(mesh->vertex_count);
      for (unsigned int index = 0; index < mesh->vertex_count; ++index) {
        vertices.emplace_back(
          mesh->vertices[index * 3], mesh->vertices[index * 3 + 1],
          mesh->vertices[index * 3 + 2]);
      }
      std::vector<fcl::Triangle> triangles;
      triangles.reserve(mesh->triangle_count);
      for (unsigned int index = 0; index < mesh->triangle_count; ++index) {
        triangles.emplace_back(
          mesh->triangles[index * 3], mesh->triangles[index * 3 + 1],
          mesh->triangles[index * 3 + 2]);
      }
      auto mesh_geometry = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
      if (mesh_geometry->beginModel(
          static_cast<int>(mesh->triangle_count),
          static_cast<int>(mesh->vertex_count)) != fcl::BVH_OK ||
        mesh_geometry->addSubModel(vertices, triangles) != fcl::BVH_OK ||
        mesh_geometry->endModel() != fcl::BVH_OK)
      {
        return nullptr;
      }
      geometry = std::move(mesh_geometry);
      break;
    }
    default:
      return nullptr;
  }
  cache.emplace(shape.get(), geometry);
  return geometry;
}

struct ExactBoxGraspReport
{
  double pad1_distance{std::numeric_limits<double>::infinity()};
  double pad2_distance{std::numeric_limits<double>::infinity()};
  double nonpad_distance{std::numeric_limits<double>::infinity()};
  bool pad1_collision{false};
  bool pad2_collision{false};
  bool nonpad_collision{false};
  size_t pad1_contacts{0U};
  size_t pad2_contacts{0U};
  double pad1_contact_x_sum{0.0};
  double pad2_contact_x_sum{0.0};
  double pad1_contact_y_sum{0.0};
  double pad2_contact_y_sum{0.0};
  double minimum_contact_edge_margin{std::numeric_limits<double>::infinity()};
  std::string nearest_nonpad_link;
};

std::optional<ExactBoxGraspReport> exact_left_box_grasp_report(
  const moveit::core::RobotState & state,
  const geometry_msgs::msg::Pose & object_pose,
  const double size_x,
  const double size_y,
  const double size_z)
{
  if (!std::isfinite(size_x) || !std::isfinite(size_y) || !std::isfinite(size_z) ||
    size_x <= 0.0 || size_y <= 0.0 || size_z <= 0.0)
  {
    return std::nullopt;
  }
  auto box_geometry = std::make_shared<fcl::Boxd>(size_x, size_y, size_z);
  const Eigen::Isometry3d object_transform = pose_to_eigen(object_pose);
  fcl::CollisionObjectd box_object(box_geometry, object_transform);
  const std::string pad1 = "left_dex1_finger_1_pad_link";
  const std::string pad2 = "left_dex1_finger_2_pad_link";
  const std::vector<std::string> links{
    "left_wrist_yaw_link", "left_dex1_adapter_link", "left_dex1_base_link",
    "left_dex1_finger_1_slider_link", "left_dex1_finger_1_link", pad1,
    "left_dex1_finger_2_slider_link", "left_dex1_finger_2_link", pad2,
  };
  ExactBoxGraspReport report;
  for (const auto & link_name : links) {
    const auto * link = state.getRobotModel()->getLinkModel(link_name);
    if (!link) {
      return std::nullopt;
    }
    const auto & link_shapes = link->getShapes();
    const auto & origins = link->getCollisionOriginTransforms();
    if (link_shapes.size() != origins.size()) {
      return std::nullopt;
    }
    for (size_t index = 0; index < link_shapes.size(); ++index) {
      const auto geometry = fcl_geometry_for_shape(link_shapes[index]);
      if (!geometry) {
        return std::nullopt;
      }
      fcl::CollisionObjectd link_object(
        geometry, state.getGlobalLinkTransform(link) * origins[index]);
      const bool is_pad = link_name == pad1 || link_name == pad2;
      fcl::CollisionRequestd collision_request(is_pad ? 64U : 1U, is_pad);
      fcl::CollisionResultd collision_result;
      fcl::collide(&link_object, &box_object, collision_request, collision_result);
      fcl::DistanceRequestd distance_request(false, false, 0.0, 0.0, 1e-7);
      fcl::DistanceResultd distance_result;
      const double distance = fcl::distance(
        &link_object, &box_object, distance_request, distance_result);
      if (!std::isfinite(distance)) {
        return std::nullopt;
      }
      if (link_name == pad1) {
        report.pad1_distance = std::min(report.pad1_distance, distance);
        report.pad1_collision = report.pad1_collision || collision_result.isCollision();
      } else if (link_name == pad2) {
        report.pad2_distance = std::min(report.pad2_distance, distance);
        report.pad2_collision = report.pad2_collision || collision_result.isCollision();
      } else if (distance < report.nonpad_distance) {
        report.nonpad_distance = distance;
        report.nonpad_collision = collision_result.isCollision();
        report.nearest_nonpad_link = link_name;
      } else {
        report.nonpad_collision = report.nonpad_collision || collision_result.isCollision();
      }
      if (is_pad && collision_result.isCollision()) {
        std::vector<fcl::Contactd> contacts;
        collision_result.getContacts(contacts);
        for (const auto & contact : contacts) {
          const Eigen::Vector3d local = object_transform.inverse() * contact.pos;
          report.minimum_contact_edge_margin = std::min(
            report.minimum_contact_edge_margin,
            size_y * 0.5 - std::abs(local.y()));
          if (link_name == pad1) {
            ++report.pad1_contacts;
            report.pad1_contact_x_sum += local.x();
            report.pad1_contact_y_sum += local.y();
          } else {
            ++report.pad2_contacts;
            report.pad2_contact_x_sum += local.x();
            report.pad2_contact_y_sum += local.y();
          }
        }
      }
    }
  }
  return report;
}

moveit_msgs::msg::AttachedCollisionObject make_left_attached_box(
  const geometry_msgs::msg::Pose & object_pose_in_tcp,
  const double size_x,
  const double size_y,
  const double size_z)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "left_dex1_tcp";
  attached.touch_links = hand_touch_links("left");
  attached.object.header.frame_id = "left_dex1_tcp";
  attached.object.id = kObjectId;
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size_x, size_y, size_z};
  attached.object.primitives.push_back(primitive);
  attached.object.primitive_poses.push_back(object_pose_in_tcp);
  return attached;
}

bool remove_left_attached_box(
  moveit::planning_interface::PlanningSceneInterface & planning_scene)
{
  // A failed preflight can reach cleanup before the virtual box was ever
  // attached.  In that case the authoritative world box must be preserved,
  // and there is nothing to detach.
  if (planning_scene.getAttachedObjects({kObjectId}).empty()) {
    return true;
  }
  moveit_msgs::msg::AttachedCollisionObject removal;
  removal.link_name = "left_dex1_tcp";
  removal.object.header.frame_id = "left_dex1_tcp";
  removal.object.id = kObjectId;
  removal.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  if (!planning_scene.applyAttachedCollisionObject(removal)) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (planning_scene.getAttachedObjects({kObjectId}).empty()) {
      return true;
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

std::shared_ptr<planning_scene::PlanningScene> read_frozen_planning_scene(
  const rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr & client,
  const moveit::core::RobotModelConstPtr & robot_model,
  const std::string & stage_name,
  const geometry_msgs::msg::Pose * cube_ghost_pose_in_planning_frame = nullptr,
  const double cube_size_x = 0.0,
  const double cube_size_y = 0.0,
  const double cube_size_z = 0.0)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  if (!client->wait_for_service(3s)) {
    RCLCPP_ERROR(logger, "GetPlanningScene is unavailable for '%s'.", stage_name.c_str());
    return nullptr;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
    moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
    moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
    moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
    moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
    moveit_msgs::msg::PlanningSceneComponents::OCTOMAP |
    moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS |
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
    moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING;
  auto future = client->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(logger, "Timed out reading the frozen scene for '%s'.", stage_name.c_str());
    return nullptr;
  }
  auto scene_message = future.get()->scene;
  // MoveGroup serializes its fixed-transform cache with pelvis as target,
  // while a standalone PlanningScene expects the RobotModel frame (world).
  // Every task object and the OctoMap are already expressed in pelvis, so let
  // the freshly loaded RobotState resolve that fixed link instead of replaying
  // the incompatible cache.
  scene_message.fixed_frame_transforms.clear();
  auto frozen = std::make_shared<planning_scene::PlanningScene>(robot_model);
  frozen->setPlanningSceneMsg(scene_message);

  // Perception visual masks exist only to remove robot pixels from the depth
  // cloud.  They are not physical collision bodies and two left-finger masks
  // intentionally overlap by about 0.36 mm.  Remove only those masks from the
  // local audit clone; retain the exact right-attached door/handle and every
  // physical robot link unchanged.
  auto & frozen_state = frozen->getCurrentStateNonConst();
  std::map<std::string, const moveit::core::AttachedBody *> attached_bodies;
  frozen_state.getAttachedBodies(attached_bodies);
  std::vector<std::string> attached_masks;
  for (const auto & [id, body] : attached_bodies) {
    static_cast<void>(body);
    if (id.find("visual_mask") != std::string::npos) {
      attached_masks.push_back(id);
    }
  }
  for (const auto & id : attached_masks) {
    frozen_state.clearAttachedBody(id);
  }
  frozen_state.updateCollisionBodyTransforms();
  std::vector<std::string> world_masks;
  for (const auto & id : frozen->getWorld()->getObjectIds()) {
    if (id.find("visual_mask") != std::string::npos) {
      world_masks.push_back(id);
    }
  }
  for (const auto & id : world_masks) {
    frozen->getWorldNonConst()->removeObject(id);
  }

  // The depth camera necessarily observes the task cube before pickup.  Its
  // occupied voxels are therefore a duplicate of the exact collision box
  // above, and treating both representations as independent obstacles makes
  // the open fingers fail the 10 mm gate while approaching the real cube.
  // Filter only the measured cube volume in this frozen audit clone.  The live
  // MoveGroup OctoMap is never changed and every voxel outside that tightly
  // bounded volume remains collision- and distance-active.
  if (cube_ghost_pose_in_planning_frame != nullptr) {
    constexpr char kOctomapId[] = "<octomap>";
    const auto octomap_object = frozen->getWorld()->getObject(kOctomapId);
    if (octomap_object && !octomap_object->shapes_.empty()) {
      if (octomap_object->shapes_.size() != 1U ||
        octomap_object->shape_poses_.size() != 1U ||
        octomap_object->global_shape_poses_.size() != 1U)
      {
        RCLCPP_ERROR(
          logger, "Frozen OctoMap layout is unexpected for '%s'.", stage_name.c_str());
        return nullptr;
      }
      const auto * source_shape = dynamic_cast<const shapes::OcTree *>(
        octomap_object->shapes_.front().get());
      if (source_shape == nullptr || !source_shape->octree) {
        RCLCPP_ERROR(
          logger, "Frozen OctoMap shape is unavailable for '%s'.", stage_name.c_str());
        return nullptr;
      }
      auto filtered_tree = std::make_shared<octomap::OcTree>(*source_shape->octree);
      const Eigen::Isometry3d cube_global =
        frozen_state.getGlobalLinkTransform(kPlanningFrame) *
        pose_to_eigen(*cube_ghost_pose_in_planning_frame);
      const Eigen::Isometry3d octomap_global =
        octomap_object->global_shape_poses_.front();
      // 10 mm is the authorized task-object perception mask.  Add half of a
      // 5 mm leaf so a voxel whose center lies just outside the exact box but
      // whose volume overlaps it is removed as the same observed cube.
      constexpr double kCubeGhostPadding = 0.0125;
      const Eigen::Vector3d half_extent(
        0.5 * cube_size_x + kCubeGhostPadding,
        0.5 * cube_size_y + kCubeGhostPadding,
        0.5 * cube_size_z + kCubeGhostPadding);
      std::vector<octomap::OcTreeKey> removals;
      for (auto leaf = filtered_tree->begin_leafs();
        leaf != filtered_tree->end_leafs(); ++leaf)
      {
        if (!filtered_tree->isNodeOccupied(*leaf)) {
          continue;
        }
        const auto point = leaf.getCoordinate();
        const Eigen::Vector3d local = cube_global.inverse() *
          (octomap_global * Eigen::Vector3d(point.x(), point.y(), point.z()));
        if ((local.array().abs() <= half_extent.array()).all()) {
          removals.push_back(leaf.getKey());
        }
      }
      for (const auto & key : removals) {
        // Keep the tree topology valid and mark only this occupied leaf free.
        // Deleting mixed-depth leaves can invalidate parent/child ownership in
        // octomap 1.9 when the cloned tree is subsequently pruned.
        filtered_tree->setNodeValue(
          key, filtered_tree->getClampingThresMinLog(), true);
      }
      if (!removals.empty()) {
        filtered_tree->updateInnerOccupancy();
        filtered_tree->prune();
        const Eigen::Isometry3d object_pose = octomap_object->pose_;
        const Eigen::Isometry3d shape_pose = octomap_object->shape_poses_.front();
        frozen->getWorldNonConst()->removeObject(kOctomapId);
        frozen->getWorldNonConst()->addToObject(
          kOctomapId, object_pose,
          std::make_shared<shapes::OcTree>(filtered_tree), shape_pose);
      }
      RCLCPP_INFO(
        logger,
        "LEFT_RACK_LOCAL_OCTOMAP_FILTER '%s': removed %zu target-cube voxel(s); "
        "live OctoMap unchanged.",
        stage_name.c_str(), removals.size());
    }
  }
  return frozen;
}

bool compute_complete_left_cartesian_path(
  MoveGroupInterface & left_arm,
  const moveit::core::RobotState & start,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  const std::string & stage_name,
  moveit_msgs::msg::RobotTrajectory & trajectory,
  moveit::core::RobotState & endpoint,
  const bool avoid_collisions = true,
  const bool require_exact_fraction = false)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  if (waypoints.empty()) {
    return false;
  }
  left_arm.setStartState(start);
  left_arm.clearPoseTargets();
  moveit_msgs::msg::MoveItErrorCodes error;
  const double fraction = left_arm.computeCartesianPath(
    waypoints, kCartesianEefStep, trajectory, avoid_collisions, &error);
  left_arm.setStartStateToCurrentState();
  auto & path = trajectory.joint_trajectory;
  if (!path.points.empty() &&
    path.joint_names.size() == path.points.front().positions.size())
  {
    trajectory_msgs::msg::JointTrajectoryPoint exact_start;
    exact_start.positions.reserve(path.joint_names.size());
    for (const auto & joint : path.joint_names) {
      exact_start.positions.push_back(start.getVariablePosition(joint));
    }
    if (!path.points.front().velocities.empty()) {
      exact_start.velocities.assign(path.joint_names.size(), 0.0);
    }
    if (!path.points.front().accelerations.empty()) {
      exact_start.accelerations.assign(path.joint_names.size(), 0.0);
    }
    exact_start.time_from_start = builtin_interfaces::msg::Duration{};
    path.points.insert(path.points.begin(), std::move(exact_start));
    // GetCartesianPath timing varies with the active planning pipeline.  The
    // positions have already passed the dedicated joint-step audit, so give
    // every preserved knot an explicit monotonic simulation-friendly time.
    constexpr double kDirectKnotPeriod = 0.10;
    for (size_t index = 0U; index < path.points.size(); ++index) {
      auto & point = path.points[index];
      point.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
        rclcpp::Duration::from_seconds(kDirectKnotPeriod * static_cast<double>(index)));
      point.velocities.clear();
      point.accelerations.clear();
      point.effort.clear();
    }
  }
  RCLCPP_INFO(
    logger, "Left-rack Cartesian preflight '%s': %.3f%%, %zu point(s), error=%d.",
    stage_name.c_str(), fraction * 100.0, path.points.size(), error.val);
  const double required_fraction = require_exact_fraction ?
    (1.0 - 1e-9) : kMinCartesianFraction;
  if (fraction < required_fraction || path.points.size() < 2U ||
    path.joint_names.size() != path.points.back().positions.size())
  {
    RCLCPP_ERROR(
      logger, "Left-rack path '%s' is incomplete; partial execution is forbidden.",
      stage_name.c_str());
    return false;
  }
  endpoint = start;
  endpoint.setVariablePositions(path.joint_names, path.points.back().positions);
  endpoint.update();
  return endpoint.satisfiesBounds(1e-6);
}

bool compute_complete_left_cartesian_segments(
  MoveGroupInterface & left_arm,
  const moveit::core::RobotState & start,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  const std::string & stage_name,
  moveit_msgs::msg::RobotTrajectory & combined_trajectory,
  moveit::core::RobotState & endpoint,
  const bool avoid_collisions = true,
  const bool require_exact_fraction = false,
  std::vector<size_t> * segment_boundaries = nullptr)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  if (waypoints.empty()) {
    return false;
  }

  combined_trajectory = moveit_msgs::msg::RobotTrajectory{};
  if (segment_boundaries != nullptr) {
    segment_boundaries->clear();
  }
  endpoint = start;
  for (size_t index = 0U; index < waypoints.size(); ++index) {
    moveit_msgs::msg::RobotTrajectory segment;
    moveit::core::RobotState segment_endpoint(endpoint);
    const std::string segment_name = stage_name + " segment " +
      std::to_string(index + 1U) + "/" + std::to_string(waypoints.size());
    if (!compute_complete_left_cartesian_path(
        left_arm, endpoint, {waypoints[index]}, segment_name, segment,
        segment_endpoint, avoid_collisions, require_exact_fraction))
    {
      RCLCPP_ERROR(
        logger, "Segmented left-rack path '%s' failed at segment %zu/%zu.",
        stage_name.c_str(), index + 1U, waypoints.size());
      return false;
    }

    auto & combined = combined_trajectory.joint_trajectory;
    const auto & source = segment.joint_trajectory;
    if (combined.joint_names.empty()) {
      combined.joint_names = source.joint_names;
      combined.header = source.header;
    } else if (combined.joint_names != source.joint_names) {
      RCLCPP_ERROR(
        logger, "Segmented left-rack path '%s' changed joint ordering at segment %zu.",
        stage_name.c_str(), index + 1U);
      return false;
    }
    const size_t source_begin = combined.points.empty() ? 0U : 1U;
    if (source.points.size() <= source_begin) {
      return false;
    }
    combined.points.insert(
      combined.points.end(), source.points.begin() + source_begin,
      source.points.end());
    if (segment_boundaries != nullptr) {
      if (segment_boundaries->empty()) {
        segment_boundaries->push_back(0U);
      }
      segment_boundaries->push_back(combined.points.size() - 1U);
    }
    endpoint = segment_endpoint;
  }

  auto & combined = combined_trajectory.joint_trajectory;
  constexpr double kDirectKnotPeriod = 0.10;
  for (size_t index = 0U; index < combined.points.size(); ++index) {
    auto & point = combined.points[index];
    point.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(
        kDirectKnotPeriod * static_cast<double>(index)));
    point.velocities.clear();
    point.accelerations.clear();
    point.effort.clear();
  }
  RCLCPP_INFO(
    logger, "Segmented left-rack path '%s': all %zu segment(s) complete, "
    "%zu combined point(s).",
    stage_name.c_str(), waypoints.size(), combined.points.size());
  return combined.points.size() >= 2U && endpoint.satisfiesBounds(1e-6);
}

trajectory_msgs::msg::JointTrajectory reverse_left_joint_path(
  const trajectory_msgs::msg::JointTrajectory & forward)
{
  trajectory_msgs::msg::JointTrajectory reverse;
  reverse.joint_names = forward.joint_names;
  reverse.points.reserve(forward.points.size());
  for (auto point = forward.points.rbegin(); point != forward.points.rend(); ++point) {
    trajectory_msgs::msg::JointTrajectoryPoint copied = *point;
    copied.time_from_start = builtin_interfaces::msg::Duration{};
    copied.velocities.clear();
    copied.accelerations.clear();
    copied.effort.clear();
    reverse.points.push_back(std::move(copied));
  }
  return reverse;
}

struct LeftRackPositionAsset
{
  std::vector<std::string> joint_names;
  std::vector<double> fresh_start;
  trajectory_msgs::msg::JointTrajectory approach;
  trajectory_msgs::msg::JointTrajectory carry;
  trajectory_msgs::msg::JointTrajectory support;
  trajectory_msgs::msg::JointTrajectory retreat;
  std::vector<size_t> approach_boundaries;
  std::vector<size_t> carry_boundaries;
  std::vector<size_t> support_boundaries;
  std::vector<size_t> retreat_boundaries;
};

constexpr char kLeftRackCacheMagic[] = "H1_LEFT_RACK_POSITION_CACHE_V1";
constexpr size_t kLeftRackCacheMaximumBytes = 4U * 1024U * 1024U;
constexpr double kLeftRackCacheStartTolerance = 0.003;

std::uint64_t left_rack_fnv1a64(const std::string & bytes)
{
  std::uint64_t value = 14695981039346656037ULL;
  for (const unsigned char byte : bytes) {
    value ^= static_cast<std::uint64_t>(byte);
    value *= 1099511628211ULL;
  }
  return value;
}

bool left_rack_positions_are_finite(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<std::string> & expected_joints)
{
  if (trajectory.joint_names != expected_joints || trajectory.points.size() < 2U) {
    return false;
  }
  return std::all_of(
    trajectory.points.begin(), trajectory.points.end(),
    [&expected_joints](const auto & point) {
      return point.positions.size() == expected_joints.size() &&
             std::all_of(point.positions.begin(), point.positions.end(),
               [](const double value) {return std::isfinite(value);});
    });
}

bool left_rack_boundaries_are_valid(
  const std::vector<size_t> & boundaries,
  const size_t point_count)
{
  if (point_count < 2U || boundaries.size() < 2U || boundaries.front() != 0U ||
    boundaries.back() != point_count - 1U)
  {
    return false;
  }
  for (size_t index = 1U; index < boundaries.size(); ++index) {
    if (boundaries[index] <= boundaries[index - 1U] || boundaries[index] >= point_count) {
      return false;
    }
  }
  return true;
}

bool left_rack_joint_vectors_match(
  const std::vector<double> & first,
  const std::vector<double> & second,
  const double tolerance)
{
  if (first.size() != second.size()) {
    return false;
  }
  for (size_t index = 0U; index < first.size(); ++index) {
    if (!std::isfinite(first[index]) || !std::isfinite(second[index]) ||
      std::abs(first[index] - second[index]) > tolerance)
    {
      return false;
    }
  }
  return true;
}

bool validate_left_rack_position_asset(
  const LeftRackPositionAsset & asset,
  const std::vector<std::string> & expected_joints,
  const std::vector<double> * live_start,
  std::string & error)
{
  if (asset.joint_names != expected_joints || expected_joints.size() != 7U ||
    asset.fresh_start.size() != expected_joints.size() ||
    !std::all_of(asset.fresh_start.begin(), asset.fresh_start.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    error = "joint order or fresh start is invalid";
    return false;
  }
  const std::array<std::pair<const trajectory_msgs::msg::JointTrajectory *,
    const std::vector<size_t> *>, 4U> paths{{
      {&asset.approach, &asset.approach_boundaries},
      {&asset.carry, &asset.carry_boundaries},
      {&asset.support, &asset.support_boundaries},
      {&asset.retreat, &asset.retreat_boundaries},
    }};
  for (const auto & [trajectory, boundaries] : paths) {
    if (!left_rack_positions_are_finite(*trajectory, expected_joints) ||
      !left_rack_boundaries_are_valid(*boundaries, trajectory->points.size()))
    {
      error = "a path is non-finite, incomplete, or has invalid segment boundaries";
      return false;
    }
  }
  if (!left_rack_joint_vectors_match(
      asset.fresh_start, asset.approach.points.front().positions, 1e-12) ||
    !left_rack_joint_vectors_match(
      asset.approach.points.back().positions, asset.carry.points.front().positions, 1e-12) ||
    !left_rack_joint_vectors_match(
      asset.carry.points.back().positions, asset.support.points.front().positions, 1e-12) ||
    !left_rack_joint_vectors_match(
      asset.support.points.back().positions, asset.retreat.points.front().positions, 1e-12))
  {
    error = "a cached path seam or the cached fresh start is not exact";
    return false;
  }
  if (live_start != nullptr && !left_rack_joint_vectors_match(
      asset.fresh_start, *live_start, kLeftRackCacheStartTolerance))
  {
    error = "live left-arm start differs from the cached fresh start by more than 0.003 rad";
    return false;
  }
  return true;
}

std::string serialize_left_rack_position_asset(const LeftRackPositionAsset & asset)
{
  std::ostringstream stream;
  stream << std::setprecision(17) << kLeftRackCacheMagic << '\n';
  stream << "joint_count " << asset.joint_names.size() << '\n';
  stream << "joint_names";
  for (const auto & name : asset.joint_names) {
    stream << ' ' << name;
  }
  stream << '\n' << "fresh_start";
  for (const double value : asset.fresh_start) {
    stream << ' ' << value;
  }
  stream << '\n';
  // These fixed task fields bind the position asset to the exact fresh cube,
  // original middle-rack target, selected front corridor and q60 mechanism.
  // Keep the 160 mm cube edge parallel to the cabinet front-to-back axis at
  // release.  The former 70 degree target was visibly diagonal on the rack.
  // The final two fields bind the simulation-only 0.5 mm carry/retreat
  // self-clearance exception and the level 1.5 mm physics-settle release
  // height; the independent 10 mm environment-clearance gate stays strict.
  stream << "task_contract 0.08 0.16 0.03 0.334 0.377 0.83 90 60 90 "
         << "0.610890542898 -0.029527393333 0.995407082351 90 0.340 1.020 1 "
         << "32 0 75 0 0 0 -90 0.025 0.0005 0.0015\n";
  const auto write_path = [&stream](
    const std::string & name,
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<size_t> & boundaries)
    {
      stream << "stage " << name << ' ' << trajectory.points.size() << ' ' <<
        boundaries.size() << '\n';
      stream << "boundaries";
      for (const size_t boundary : boundaries) {
        stream << ' ' << boundary;
      }
      stream << '\n' << "start";
      for (const double value : trajectory.points.front().positions) {
        stream << ' ' << value;
      }
      stream << '\n' << "finish";
      for (const double value : trajectory.points.back().positions) {
        stream << ' ' << value;
      }
      stream << '\n' << "points\n";
      for (const auto & point : trajectory.points) {
        for (size_t index = 0U; index < point.positions.size(); ++index) {
          if (index > 0U) {
            stream << ' ';
          }
          stream << point.positions[index];
        }
        stream << '\n';
      }
    };
  write_path("approach", asset.approach, asset.approach_boundaries);
  write_path("carry", asset.carry, asset.carry_boundaries);
  write_path("support", asset.support, asset.support_boundaries);
  write_path("retreat", asset.retreat, asset.retreat_boundaries);
  stream << "end_cache\n";
  return stream.str();
}

bool write_new_left_rack_position_asset(
  const std::string & path,
  const LeftRackPositionAsset & asset,
  std::string & error)
{
  std::string validation_error;
  if (!validate_left_rack_position_asset(
      asset, asset.joint_names, nullptr, validation_error))
  {
    error = "refusing to write invalid asset: " + validation_error;
    return false;
  }
  std::string body = serialize_left_rack_position_asset(asset);
  std::ostringstream checksum;
  checksum << "checksum_fnv1a64 " << std::hex << std::setfill('0') << std::setw(16) <<
    left_rack_fnv1a64(body) << '\n';
  body += checksum.str();
  if (path.empty() || path.front() != '/' || body.size() > kLeftRackCacheMaximumBytes) {
    error = "cache path must be absolute and payload must be bounded";
    return false;
  }
  const int descriptor = ::open(
    path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0400);
  if (descriptor < 0) {
    error = "exclusive cache create failed: " + std::string(std::strerror(errno));
    return false;
  }
  size_t offset = 0U;
  bool written = true;
  while (offset < body.size()) {
    const ssize_t count = ::write(descriptor, body.data() + offset, body.size() - offset);
    if (count <= 0) {
      written = false;
      error = "cache write failed: " + std::string(std::strerror(errno));
      break;
    }
    offset += static_cast<size_t>(count);
  }
  if (written && (::fchmod(descriptor, 0400) != 0 || ::fsync(descriptor) != 0)) {
    written = false;
    error = "cache sealing failed: " + std::string(std::strerror(errno));
  }
  if (::close(descriptor) != 0 && written) {
    written = false;
    error = "cache close failed: " + std::string(std::strerror(errno));
  }
  // Never unlink a partially written exclusive target here: retaining it makes
  // the failure visible and guarantees a later export cannot silently replace it.
  return written;
}

std::optional<LeftRackPositionAsset> read_left_rack_position_asset(
  const std::string & path,
  const std::vector<std::string> & expected_joints,
  const std::vector<double> & live_start,
  std::string & error)
{
  struct stat path_status {};
  if (path.empty() || path.front() != '/' || ::lstat(path.c_str(), &path_status) != 0 ||
    !S_ISREG(path_status.st_mode) || S_ISLNK(path_status.st_mode) ||
    path_status.st_nlink != 1 || (path_status.st_mode & 0222) != 0 ||
    path_status.st_size <= 0 ||
    static_cast<std::uint64_t>(path_status.st_size) > kLeftRackCacheMaximumBytes)
  {
    error = "cache is missing, non-regular, writable, linked, or oversized";
    return std::nullopt;
  }
  const int descriptor = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (descriptor < 0) {
    error = "cache open failed: " + std::string(std::strerror(errno));
    return std::nullopt;
  }
  struct stat opened_status {};
  if (::fstat(descriptor, &opened_status) != 0 ||
    opened_status.st_dev != path_status.st_dev || opened_status.st_ino != path_status.st_ino ||
    opened_status.st_size != path_status.st_size)
  {
    ::close(descriptor);
    error = "cache path changed while opening";
    return std::nullopt;
  }
  std::string contents(static_cast<size_t>(opened_status.st_size), '\0');
  size_t offset = 0U;
  while (offset < contents.size()) {
    const ssize_t count = ::read(descriptor, contents.data() + offset, contents.size() - offset);
    if (count <= 0) {
      ::close(descriptor);
      error = "cache read failed or was truncated";
      return std::nullopt;
    }
    offset += static_cast<size_t>(count);
  }
  if (::close(descriptor) != 0) {
    error = "cache close failed";
    return std::nullopt;
  }
  constexpr char kChecksumMarker[] = "checksum_fnv1a64 ";
  const size_t checksum_offset = contents.rfind(kChecksumMarker);
  if (checksum_offset == std::string::npos || checksum_offset == 0U ||
    contents.back() != '\n')
  {
    error = "cache checksum record is missing";
    return std::nullopt;
  }
  const std::string body = contents.substr(0U, checksum_offset);
  const std::string checksum_line = contents.substr(checksum_offset);
  std::istringstream checksum_stream(checksum_line);
  std::string checksum_name;
  std::string checksum_hex;
  std::string checksum_trailing;
  checksum_stream >> checksum_name >> checksum_hex;
  if (checksum_name != "checksum_fnv1a64" || checksum_hex.size() != 16U ||
    (checksum_stream >> checksum_trailing))
  {
    error = "cache checksum record is malformed";
    return std::nullopt;
  }
  std::uint64_t expected_checksum = 0U;
  std::istringstream checksum_value(checksum_hex);
  checksum_value >> std::hex >> expected_checksum;
  if (!checksum_value || expected_checksum != left_rack_fnv1a64(body)) {
    error = "cache checksum mismatch";
    return std::nullopt;
  }

  std::istringstream stream(body);
  std::string token;
  size_t joint_count = 0U;
  LeftRackPositionAsset asset;
  if (!(stream >> token) || token != kLeftRackCacheMagic ||
    !(stream >> token >> joint_count) || token != "joint_count" || joint_count != 7U ||
    !(stream >> token) || token != "joint_names")
  {
    error = "cache header is invalid";
    return std::nullopt;
  }
  asset.joint_names.resize(joint_count);
  for (auto & name : asset.joint_names) {
    if (!(stream >> name) || name.size() > 128U) {
      error = "cache joint list is invalid";
      return std::nullopt;
    }
  }
  if (!(stream >> token) || token != "fresh_start") {
    error = "cache fresh start is missing";
    return std::nullopt;
  }
  asset.fresh_start.resize(joint_count);
  for (double & value : asset.fresh_start) {
    if (!(stream >> value) || !std::isfinite(value)) {
      error = "cache fresh start is invalid";
      return std::nullopt;
    }
  }
  const std::array<double, 26U> expected_contract{{
      0.08, 0.16, 0.03, 0.334, 0.377, 0.83, 90.0, 60.0,
      90.0, 0.610890542898, -0.029527393333, 0.995407082351,
      90.0, 0.340, 1.020, 1.0, 32.0, 0.0, 75.0, 0.0,
      0.0, 0.0, -90.0, 0.025, 0.0005, 0.0015}};
  if (!(stream >> token) || token != "task_contract") {
    error = "cache task contract is missing";
    return std::nullopt;
  }
  for (const double expected : expected_contract) {
    double value = 0.0;
    if (!(stream >> value) || !std::isfinite(value) || std::abs(value - expected) > 1e-12) {
      error = "cache task contract does not match this executable";
      return std::nullopt;
    }
  }
  const auto read_path = [&stream, joint_count, &error](
    const std::string & expected_name,
    trajectory_msgs::msg::JointTrajectory & trajectory,
    std::vector<size_t> & boundaries) -> bool
    {
      std::string local_token;
      std::string name;
      size_t point_count = 0U;
      size_t boundary_count = 0U;
      if (!(stream >> local_token >> name >> point_count >> boundary_count) ||
        local_token != "stage" || name != expected_name || point_count < 2U ||
        point_count > 10000U || boundary_count < 2U || boundary_count > 100U)
      {
        error = "cache stage header is invalid for " + expected_name;
        return false;
      }
      if (!(stream >> local_token) || local_token != "boundaries") {
        error = "cache boundaries are missing for " + expected_name;
        return false;
      }
      boundaries.resize(boundary_count);
      for (size_t & boundary : boundaries) {
        if (!(stream >> boundary)) {
          error = "cache boundary is invalid for " + expected_name;
          return false;
        }
      }
      std::vector<double> declared_start(joint_count, 0.0);
      std::vector<double> declared_finish(joint_count, 0.0);
      if (!(stream >> local_token) || local_token != "start") {
        error = "cache start endpoint is missing for " + expected_name;
        return false;
      }
      for (double & value : declared_start) {
        if (!(stream >> value) || !std::isfinite(value)) {
          return false;
        }
      }
      if (!(stream >> local_token) || local_token != "finish") {
        error = "cache finish endpoint is missing for " + expected_name;
        return false;
      }
      for (double & value : declared_finish) {
        if (!(stream >> value) || !std::isfinite(value)) {
          return false;
        }
      }
      if (!(stream >> local_token) || local_token != "points") {
        error = "cache points are missing for " + expected_name;
        return false;
      }
      trajectory.points.resize(point_count);
      for (auto & point : trajectory.points) {
        point.positions.resize(joint_count);
        for (double & value : point.positions) {
          if (!(stream >> value) || !std::isfinite(value)) {
            error = "cache contains a non-finite point for " + expected_name;
            return false;
          }
        }
      }
      if (!left_rack_joint_vectors_match(
          declared_start, trajectory.points.front().positions, 1e-12) ||
        !left_rack_joint_vectors_match(
          declared_finish, trajectory.points.back().positions, 1e-12))
      {
        error = "cache endpoint record disagrees with points for " + expected_name;
        return false;
      }
      return true;
    };
  if (!read_path("approach", asset.approach, asset.approach_boundaries) ||
    !read_path("carry", asset.carry, asset.carry_boundaries) ||
    !read_path("support", asset.support, asset.support_boundaries) ||
    !read_path("retreat", asset.retreat, asset.retreat_boundaries) ||
    !(stream >> token) || token != "end_cache")
  {
    if (error.empty()) {
      error = "cache body is incomplete";
    }
    return std::nullopt;
  }
  std::string trailing;
  if (stream >> trailing) {
    error = "cache has trailing fields";
    return std::nullopt;
  }
  for (auto * trajectory : {&asset.approach, &asset.carry, &asset.support, &asset.retreat}) {
    trajectory->joint_names = asset.joint_names;
  }
  if (!validate_left_rack_position_asset(
      asset, expected_joints, &live_start, error))
  {
    return std::nullopt;
  }
  return asset;
}

bool set_left_rack_endpoint_state(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const moveit::core::RobotState & start,
  moveit::core::RobotState & endpoint)
{
  if (!left_rack_positions_are_finite(trajectory, trajectory.joint_names)) {
    return false;
  }
  endpoint = start;
  endpoint.setVariablePositions(trajectory.joint_names, trajectory.points.back().positions);
  endpoint.update();
  return endpoint.satisfiesBounds(1e-6);
}

void assign_left_rack_uniform_timing(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const double period)
{
  for (size_t index = 0U; index < trajectory.points.size(); ++index) {
    auto & point = trajectory.points[index];
    point.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(period * static_cast<double>(index)));
    point.velocities.clear();
    point.accelerations.clear();
    point.effort.clear();
  }
}

std::optional<double> retime_left_rack_path_locally(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const JointMotionLimits & limits,
  const double minimum_period,
  const std::string & stage)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  constexpr double kTargetVelocityFraction = 0.34;
  constexpr double kTargetAccelerationFraction = 0.95;
  constexpr double kHardVelocityFraction = 0.35;
  constexpr double kHardAccelerationFraction = 1.00;
  constexpr size_t kMaximumIterations = 64U;
  if (!std::isfinite(minimum_period) || minimum_period < 0.01 ||
    minimum_period > 0.10 || trajectory.points.size() < 2U ||
    trajectory.joint_names.empty())
  {
    return std::nullopt;
  }
  const size_t segment_count = trajectory.points.size() - 1U;
  const size_t joint_count = trajectory.joint_names.size();
  std::vector<double> durations(segment_count, minimum_period);
  std::vector<std::vector<double>> differences(
    segment_count, std::vector<double>(joint_count, 0.0));
  for (size_t joint = 0U; joint < joint_count; ++joint) {
    if (limits.find(trajectory.joint_names[joint]) == limits.end()) {
      return std::nullopt;
    }
  }
  for (size_t segment = 0U; segment < segment_count; ++segment) {
    if (trajectory.points[segment].positions.size() != joint_count ||
      trajectory.points[segment + 1U].positions.size() != joint_count)
    {
      return std::nullopt;
    }
    for (size_t joint = 0U; joint < joint_count; ++joint) {
      differences[segment][joint] =
        trajectory.points[segment + 1U].positions[joint] -
        trajectory.points[segment].positions[joint];
      if (!std::isfinite(differences[segment][joint])) {
        return std::nullopt;
      }
    }
  }
  bool converged = false;
  size_t iterations = 0U;
  for (; iterations < kMaximumIterations; ++iterations) {
    std::vector<std::vector<double>> velocities(
      segment_count, std::vector<double>(joint_count, 0.0));
    std::vector<double> scales(segment_count, 1.0);
    bool adjustment = false;
    for (size_t segment = 0U; segment < segment_count; ++segment) {
      for (size_t joint = 0U; joint < joint_count; ++joint) {
        const auto & limit = limits.at(trajectory.joint_names[joint]);
        velocities[segment][joint] = differences[segment][joint] / durations[segment];
        const double ratio = std::abs(velocities[segment][joint]) /
          (limit.max_velocity * kTargetVelocityFraction);
        if (ratio > 1.0 + 1e-6) {
          scales[segment] = std::max(scales[segment], ratio);
          adjustment = true;
        }
      }
    }
    for (size_t knot = 0U; knot <= segment_count; ++knot) {
      const bool first = knot == 0U;
      const bool last = knot == segment_count;
      const size_t previous = first ? 0U : knot - 1U;
      const size_t next = last ? segment_count - 1U : knot;
      const double acceleration_dt = first ? durations.front() :
        (last ? durations.back() : 0.5 * (durations[previous] + durations[next]));
      for (size_t joint = 0U; joint < joint_count; ++joint) {
        const double prior_velocity = first ? 0.0 : velocities[previous][joint];
        const double next_velocity = last ? 0.0 : velocities[next][joint];
        const auto & limit = limits.at(trajectory.joint_names[joint]);
        const double ratio = std::abs(next_velocity - prior_velocity) /
          acceleration_dt / (limit.max_acceleration * kTargetAccelerationFraction);
        if (ratio > 1.0 + 1e-6) {
          const double scale = std::sqrt(ratio);
          scales[previous] = std::max(scales[previous], scale);
          scales[next] = std::max(scales[next], scale);
          adjustment = true;
        }
      }
    }
    if (!adjustment) {
      converged = true;
      break;
    }
    for (size_t segment = 0U; segment < segment_count; ++segment) {
      durations[segment] *= scales[segment];
    }
  }
  if (!converged) {
    RCLCPP_ERROR(logger, "Left-rack local timing '%s' did not converge.", stage.c_str());
    return std::nullopt;
  }
  double time = 0.0;
  for (size_t index = 0U; index < trajectory.points.size(); ++index) {
    auto & point = trajectory.points[index];
    point.time_from_start = rclcpp::Duration::from_seconds(time);
    point.velocities.clear();
    point.accelerations.clear();
    point.effort.clear();
    if (index < durations.size()) {
      time += durations[index];
    }
  }
  double peak_velocity_fraction = 0.0;
  double peak_acceleration_fraction = 0.0;
  std::vector<double> previous_velocity(joint_count, 0.0);
  double previous_dt = 0.0;
  for (size_t segment = 0U; segment < segment_count; ++segment) {
    const double dt = durations[segment];
    for (size_t joint = 0U; joint < joint_count; ++joint) {
      const auto & limit = limits.at(trajectory.joint_names[joint]);
      const double velocity = differences[segment][joint] / dt;
      peak_velocity_fraction = std::max(
        peak_velocity_fraction, std::abs(velocity) / limit.max_velocity);
      const double acceleration_dt = segment == 0U ? dt :
        0.5 * (previous_dt + dt);
      const double prior = segment == 0U ? 0.0 : previous_velocity[joint];
      peak_acceleration_fraction = std::max(
        peak_acceleration_fraction,
        std::abs(velocity - prior) / acceleration_dt / limit.max_acceleration);
      previous_velocity[joint] = velocity;
    }
    previous_dt = dt;
  }
  for (size_t joint = 0U; joint < joint_count; ++joint) {
    const auto & limit = limits.at(trajectory.joint_names[joint]);
    peak_acceleration_fraction = std::max(
      peak_acceleration_fraction,
      std::abs(previous_velocity[joint]) / previous_dt / limit.max_acceleration);
  }
  const bool accepted = peak_velocity_fraction <= kHardVelocityFraction + 1e-9 &&
    peak_acceleration_fraction <= kHardAccelerationFraction + 1e-9;
  RCLCPP_INFO(
    logger,
    "LEFT_RACK_FAST_TIMING '%s': %zu points, %.3f s, velocity/acceleration "
    "%.1f/%.1f%% (hard 35/100%%) -> %s.",
    stage.c_str(), trajectory.points.size(), time,
    peak_velocity_fraction * 100.0, peak_acceleration_fraction * 100.0,
    accepted ? "PASS" : "FAIL");
  return accepted ? std::optional<double>{time} : std::nullopt;
}

struct LeftRackPathAudit
{
  bool passed{false};
  size_t dense_samples{0U};
  double minimum_environment{std::numeric_limits<double>::infinity()};
  double minimum_self{std::numeric_limits<double>::infinity()};
  double maximum_joint_step{0.0};
};

LeftRackPathAudit audit_left_rack_path(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const moveit::core::RobotState & explicit_start,
  const planning_scene::PlanningScenePtr & frozen_scene,
  const double finger_position,
  const std::string & stage_name,
  const double required_environment_clearance = 0.010,
  const bool allow_cube_body_support = false,
  const bool allow_left_cube_grasp = false,
  const bool allow_left_body_brush = true,
  const double required_self_clearance = 0.005)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  LeftRackPathAudit output;
  if (!frozen_scene || trajectory.points.empty() || trajectory.joint_names.empty()) {
    return output;
  }
  auto acm = frozen_scene->getAllowedCollisionMatrix();
  // Fast simulation trial: the only observed cabinet interference on the
  // otherwise complete center-placement corridor is a brush by these two
  // original Dex1 links at the opening.  Keep every other robot/cube/body
  // pair active; remove this exception during the later clearance redesign.
  if (allow_left_body_brush) {
    for (const auto & link : hand_touch_links("left")) {
      acm.setEntry(kIncubatorBodyId, link, true);
    }
  }
  // The attached cube starts exactly supported by the already-expanded table
  // shell.  Permit that single support pair while it lifts away; all left-hand
  // links and every other environment object remain collision-active.
  acm.setEntry(kObjectId, "environment_table_10mm_margin", true);
  if (allow_cube_body_support) {
    acm.setEntry(kObjectId, kIncubatorBodyId, true);
  }
  if (allow_left_cube_grasp) {
    for (const auto & link : hand_touch_links("left")) {
      acm.setEntry(kObjectId, link, true);
    }
  }
  // The table and legacy perception cube guard are already expanded by the
  // project's full 10 mm margin.  Keep them collision-active in `acm` (zero
  // penetration is the 10 mm gate), but exclude them from the additional
  // exact-environment distance query so the margin is not counted twice.
  auto clearance_acm = acm;
  const auto * left_group = frozen_scene->getRobotModel()->getJointModelGroup("left_arm");
  if (!left_group) {
    return output;
  }
  // The planning group does not enumerate end-effector child links.  Apply
  // this no-double-count rule to every physical left-side link, including the
  // Dex1 fingers and pads; the expanded shells stay active in the ordinary
  // collision request above, so penetration is still rejected.
  for (const auto * link_model : frozen_scene->getRobotModel()->getLinkModels()) {
    const auto & link = link_model->getName();
    if (link.rfind("left_", 0) != 0U) {
      continue;
    }
    clearance_acm.setEntry("environment_table_10mm_margin", link, true);
    clearance_acm.setEntry("green_cube_10mm_guard", link, true);
  }
  moveit::core::RobotState state(frozen_scene->getCurrentState());
  state.setVariablePositions(explicit_start.getVariablePositions());
  state.setVariablePosition("left_dex1_finger_1_joint", finger_position);
  state.setVariablePosition("left_dex1_finger_2_joint", finger_position);
  state.update();

  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = "left_arm";
  collision_request.contacts = true;
  collision_request.max_contacts = 20U;
  collision_request.max_contacts_per_pair = 4U;
  collision_request.pad_environment_collisions = false;
  collision_request.pad_self_collisions = false;

  collision_detection::DistanceRequest environment_request;
  environment_request.group_name = "left_arm";
  environment_request.enableGroup(state.getRobotModel());
  environment_request.type = collision_detection::DistanceRequestTypes::GLOBAL;
  environment_request.enable_signed_distance = true;
  environment_request.distance_threshold = required_environment_clearance + 1e-6;
  environment_request.acm = &clearance_acm;
  collision_detection::DistanceRequest self_request = environment_request;
  self_request.acm = &acm;

  // Retain every Cartesian knot and audit every inter-knot line at no more
  // than 0.5 degree per arm joint.  The fixed-TCP redundancy bridge below is
  // part of this same position path and must not receive a coarser exception.
  constexpr double kDenseJointStep = 0.5 * M_PI / 180.0;
  constexpr double kMaximumAdjacentStep = 4.9 * M_PI / 180.0;
  if (!std::isfinite(required_self_clearance) || required_self_clearance < 0.0) {
    return output;
  }
  bool accepted = true;
  for (size_t point_index = 0; accepted && point_index < trajectory.points.size(); ++point_index) {
    const auto & point = trajectory.points[point_index];
    if (point.positions.size() != trajectory.joint_names.size()) {
      accepted = false;
      break;
    }
    moveit::core::RobotState next(state);
    next.setVariablePositions(trajectory.joint_names, point.positions);
    next.setVariablePosition("left_dex1_finger_1_joint", finger_position);
    next.setVariablePosition("left_dex1_finger_2_joint", finger_position);
    next.update();
    double step = 0.0;
    for (const auto & joint : trajectory.joint_names) {
      step = std::max(
        step, std::abs(next.getVariablePosition(joint) - state.getVariablePosition(joint)));
    }
    output.maximum_joint_step = std::max(output.maximum_joint_step, step);
    if (step > kMaximumAdjacentStep) {
      RCLCPP_ERROR(
        logger, "Left-rack path '%s' has %.3f deg adjacent change at point %zu "
        "(limit 4.900 deg).", stage_name.c_str(), step * 57.29577951308232,
        point_index + 1U);
      accepted = false;
      break;
    }
    const size_t subdivisions = std::max<size_t>(
      1U, static_cast<size_t>(std::ceil(step / kDenseJointStep)));
    for (size_t subdivision = 1U; accepted && subdivision <= subdivisions; ++subdivision) {
      moveit::core::RobotState sample(state);
      state.interpolate(
        next, static_cast<double>(subdivision) / static_cast<double>(subdivisions), sample);
      sample.setVariablePosition("left_dex1_finger_1_joint", finger_position);
      sample.setVariablePosition("left_dex1_finger_2_joint", finger_position);
      sample.updateCollisionBodyTransforms();
      ++output.dense_samples;
      if (!sample.satisfiesBounds(1e-6)) {
        accepted = false;
        break;
      }
      collision_detection::CollisionResult collision_result;
      frozen_scene->checkCollision(collision_request, collision_result, sample, acm);
      if (collision_result.collision) {
        RCLCPP_ERROR(
          logger, "Left-rack path '%s' collides at dense sample %zu, trajectory "
          "point %zu/%zu subdivision %zu/%zu.",
          stage_name.c_str(), output.dense_samples, point_index + 1U,
          trajectory.points.size(), subdivision, subdivisions);
        size_t reported = 0U;
        for (const auto & [pair, contacts] : collision_result.contacts) {
          if (reported++ >= 6U) {
            break;
          }
          RCLCPP_ERROR(
            logger, "  contact '%s' <-> '%s' (%zu point(s)).",
            pair.first.c_str(), pair.second.c_str(), contacts.size());
        }
        accepted = false;
        break;
      }
      collision_detection::DistanceResult environment_result;
      frozen_scene->getCollisionEnvUnpadded()->distanceRobot(
        environment_request, environment_result, sample);
      collision_detection::DistanceResult self_result;
      frozen_scene->getCollisionEnvUnpadded()->distanceSelf(
        self_request, self_result, sample);
      const double environment_distance = environment_result.minimum_distance.distance;
      const double self_distance = self_result.minimum_distance.distance;
      if (!std::isfinite(environment_distance) || !std::isfinite(self_distance)) {
        accepted = false;
        break;
      }
      output.minimum_environment = std::min(output.minimum_environment, environment_distance);
      output.minimum_self = std::min(output.minimum_self, self_distance);
      if (environment_distance + 1e-9 < required_environment_clearance ||
        self_distance + 1e-9 < required_self_clearance)
      {
        RCLCPP_ERROR(
          logger, "Left-rack path '%s' clearance failed at sample %zu, trajectory "
          "point %zu/%zu subdivision %zu/%zu: "
          "environment %.3f mm '%s' <-> '%s' (required %.3f), "
          "self %.3f mm '%s' <-> '%s' (required %.3f); environment nearest "
          "p0=[%.4f, %.4f, %.4f], p1=[%.4f, %.4f, %.4f].",
          stage_name.c_str(), output.dense_samples, point_index + 1U,
          trajectory.points.size(), subdivision, subdivisions,
          environment_distance * 1000.0,
          environment_result.minimum_distance.link_names[0].c_str(),
          environment_result.minimum_distance.link_names[1].c_str(),
          required_environment_clearance * 1000.0, self_distance * 1000.0,
          self_result.minimum_distance.link_names[0].c_str(),
          self_result.minimum_distance.link_names[1].c_str(),
          required_self_clearance * 1000.0,
          environment_result.minimum_distance.nearest_points[0].x(),
          environment_result.minimum_distance.nearest_points[0].y(),
          environment_result.minimum_distance.nearest_points[0].z(),
          environment_result.minimum_distance.nearest_points[1].x(),
          environment_result.minimum_distance.nearest_points[1].y(),
          environment_result.minimum_distance.nearest_points[1].z());
        accepted = false;
      }
    }
    state = next;
  }
  output.passed = accepted;
  RCLCPP_INFO(
    logger, "LEFT_RACK_DENSE_AUDIT '%s': %zu samples, environment/self "
    "minimum %.3f/%.3f mm, maximum adjacent change %.3f deg -> %s.",
    stage_name.c_str(), output.dense_samples,
    output.minimum_environment * 1000.0, output.minimum_self * 1000.0,
    output.maximum_joint_step * 57.29577951308232,
    output.passed ? "PASS" : "FAIL");
  return output;
}

std::optional<CenterClearanceReport> center_regrasp_clearance(
  const planning_scene::PlanningScene & geometric_scene,
  const moveit::core::RobotState & state,
  const double table_surface_z)
{
  static_cast<void>(geometric_scene);
  const auto right_links = hand_touch_links("right");
  const auto left_links = hand_touch_links("left");
  CenterClearanceReport report;
  for (const auto & right_link_name : right_links) {
    const auto * right_link = state.getRobotModel()->getLinkModel(right_link_name);
    if (!right_link) {
      return std::nullopt;
    }
    const auto & right_shapes = right_link->getShapes();
    const auto & right_origins = right_link->getCollisionOriginTransforms();
    if (right_shapes.size() != right_origins.size()) {
      return std::nullopt;
    }
    for (const auto & left_link_name : left_links) {
      const auto * left_link = state.getRobotModel()->getLinkModel(left_link_name);
      if (!left_link) {
        return std::nullopt;
      }
      const auto & left_shapes = left_link->getShapes();
      const auto & left_origins = left_link->getCollisionOriginTransforms();
      if (left_shapes.size() != left_origins.size()) {
        return std::nullopt;
      }
      for (size_t right_index = 0; right_index < right_shapes.size(); ++right_index) {
        const auto right_geometry = fcl_geometry_for_shape(right_shapes[right_index]);
        if (!right_geometry) {
          return std::nullopt;
        }
        const fcl::Transform3d right_transform =
          state.getGlobalLinkTransform(right_link) * right_origins[right_index];
        fcl::CollisionObjectd right_object(right_geometry, right_transform);
        for (size_t left_index = 0; left_index < left_shapes.size(); ++left_index) {
          const auto left_geometry = fcl_geometry_for_shape(left_shapes[left_index]);
          if (!left_geometry) {
            return std::nullopt;
          }
          const fcl::Transform3d left_transform =
            state.getGlobalLinkTransform(left_link) * left_origins[left_index];
          fcl::CollisionObjectd left_object(left_geometry, left_transform);
          const fcl::DistanceRequestd request(false, false, 0.0, 0.0, 1e-7);
          fcl::DistanceResultd result;
          const double distance = fcl::distance(
            &right_object, &left_object, request, result);
          if (!std::isfinite(distance)) {
            return std::nullopt;
          }
          if (distance < report.hand_to_hand) {
            report.hand_to_hand = distance;
            report.nearest_right_link = right_link_name;
            report.nearest_left_link = left_link_name;
          }
        }
      }
    }
  }
  const auto table_clearance = minimum_hand_table_clearance(state, table_surface_z);
  if (!table_clearance.has_value()) {
    return std::nullopt;
  }
  report.hand_to_table = *table_clearance;
  return report;
}

bool full_robot_state_is_valid(
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  const moveit::core::RobotState & state,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  request->group_name = "";
  request->robot_state.is_diff = true;
  request->robot_state.joint_state.name = state.getVariableNames();
  request->robot_state.joint_state.position.reserve(
    request->robot_state.joint_state.name.size());
  for (const auto & variable_name : request->robot_state.joint_state.name) {
    request->robot_state.joint_state.position.push_back(
      state.getVariablePosition(variable_name));
  }
  auto future = validity_client->async_send_request(request);
  if (future.wait_for(3s) != std::future_status::ready) {
    RCLCPP_ERROR(logger, "State-validity request timed out for '%s'.", stage_name.c_str());
    return false;
  }
  const auto response = future.get();
  if (response->valid) {
    return true;
  }
  RCLCPP_ERROR(
    logger, "Full-state collision gate failed for '%s' with %zu contact(s).",
    stage_name.c_str(), response->contacts.size());
  const size_t contact_count = std::min<size_t>(response->contacts.size(), 6);
  for (size_t index = 0; index < contact_count; ++index) {
    const auto & contact = response->contacts[index];
    RCLCPP_ERROR(
      logger, "  contact %zu: '%s' <-> '%s', depth %.2f mm.", index + 1,
      contact.contact_body_1.c_str(), contact.contact_body_2.c_str(),
      contact.depth * 1000.0);
  }
  return false;
}

struct CenterPathValidation
{
  double minimum_hand_clearance{std::numeric_limits<double>::infinity()};
  double minimum_table_clearance{std::numeric_limits<double>::infinity()};
  double minimum_joint_margin{std::numeric_limits<double>::infinity()};
  std::string minimum_joint_margin_name;
  std::map<std::string, double> endpoint;
};

std::optional<CenterPathValidation> validate_center_regrasp_path(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const moveit::core::RobotState & common_start,
  const planning_scene::PlanningScene & geometric_scene,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  const double table_surface_z,
  const std::string & stage_name,
  const bool enforce_clearance = true)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  constexpr double kRequiredCenterHandClearance = 0.005;
  constexpr double kRequiredTableClearance = 0.010;
  constexpr double kRequiredJointMargin = 0.020;
  constexpr double kDenseJointStep = 0.008726646259971648;
  if (trajectory.points.size() < 2 || trajectory.joint_names.empty()) {
    return std::nullopt;
  }
  if (!validity_client->wait_for_service(3s)) {
    RCLCPP_ERROR(logger, "State-validity service is unavailable for '%s'.", stage_name.c_str());
    return std::nullopt;
  }

  CenterPathValidation validation;
  auto checked_state = common_start;
  size_t dense_state_count = 0;
  for (size_t segment = 0; segment + 1 < trajectory.points.size(); ++segment) {
    const auto & start = trajectory.points[segment];
    const auto & end = trajectory.points[segment + 1];
    if (start.positions.size() != trajectory.joint_names.size() ||
      end.positions.size() != trajectory.joint_names.size())
    {
      return std::nullopt;
    }
    double maximum_step = 0.0;
    for (size_t joint = 0; joint < trajectory.joint_names.size(); ++joint) {
      maximum_step = std::max(
        maximum_step, std::abs(end.positions[joint] - start.positions[joint]));
    }
    const size_t subdivisions = std::max<size_t>(
      1, static_cast<size_t>(std::ceil(maximum_step / kDenseJointStep)));
    const size_t first = segment == 0 ? 0 : 1;
    for (size_t subdivision = first; subdivision <= subdivisions; ++subdivision) {
      const double alpha = static_cast<double>(subdivision) /
        static_cast<double>(subdivisions);
      std::vector<double> positions(trajectory.joint_names.size());
      for (size_t joint = 0; joint < positions.size(); ++joint) {
        positions[joint] = start.positions[joint] +
          alpha * (end.positions[joint] - start.positions[joint]);
        const auto & bounds = common_start.getRobotModel()->getVariableBounds(
          trajectory.joint_names[joint]);
        if (bounds.position_bounded_) {
          const double margin = std::min(
            positions[joint] - bounds.min_position_,
            bounds.max_position_ - positions[joint]);
          if (margin < validation.minimum_joint_margin) {
            validation.minimum_joint_margin = margin;
            validation.minimum_joint_margin_name = trajectory.joint_names[joint];
          }
        }
      }
      checked_state.setVariablePositions(trajectory.joint_names, positions);
      checked_state.update();
      ++dense_state_count;
      if (!checked_state.satisfiesBounds(1e-6) ||
        !full_robot_state_is_valid(
          validity_client, checked_state,
          stage_name + " dense state " + std::to_string(dense_state_count)))
      {
        return std::nullopt;
      }
      const auto clearance = center_regrasp_clearance(
        geometric_scene, checked_state, table_surface_z);
      if (!clearance.has_value()) {
        RCLCPP_ERROR(logger, "Could not evaluate mesh clearance for '%s'.", stage_name.c_str());
        return std::nullopt;
      }
      validation.minimum_hand_clearance = std::min(
        validation.minimum_hand_clearance, clearance->hand_to_hand);
      validation.minimum_table_clearance = std::min(
        validation.minimum_table_clearance, clearance->hand_to_table);
      if (clearance->hand_to_hand < kRequiredCenterHandClearance ||
        clearance->hand_to_table < kRequiredTableClearance)
      {
        if (enforce_clearance) {
          RCLCPP_INFO(
            logger,
            "Center path '%s' rejected at dense state %zu: hand/hand %.3f mm "
            "('%s' <-> '%s'), hand/table %.3f mm "
            "(required hand/table 5.000/10.000 mm).",
            stage_name.c_str(), dense_state_count,
            clearance->hand_to_hand * 1000.0,
            clearance->nearest_right_link.c_str(), clearance->nearest_left_link.c_str(),
            clearance->hand_to_table * 1000.0);
          return std::nullopt;
        }
      }
    }
  }
  if (validation.minimum_joint_margin < kRequiredJointMargin) {
    RCLCPP_INFO(
      logger, "Center path '%s' rejected: minimum joint margin %.3f rad at '%s' "
      "(required %.3f rad).",
      stage_name.c_str(), validation.minimum_joint_margin,
      validation.minimum_joint_margin_name.c_str(), kRequiredJointMargin);
    return std::nullopt;
  }
  const auto & endpoint = trajectory.points.back();
  for (size_t index = 0; index < trajectory.joint_names.size(); ++index) {
    validation.endpoint.emplace(trajectory.joint_names[index], endpoint.positions[index]);
  }
  RCLCPP_INFO(
    logger,
    "Center path '%s' %s %zu dense states: minimum hand/hand %.3f mm, "
    "hand/table %.3f mm, joint margin %.3f rad at '%s'.",
    stage_name.c_str(), enforce_clearance ? "passed" : "diagnosed",
    dense_state_count,
    validation.minimum_hand_clearance * 1000.0,
    validation.minimum_table_clearance * 1000.0,
    validation.minimum_joint_margin,
    validation.minimum_joint_margin_name.c_str());
  return validation;
}

struct CenterCandidateResult
{
  double y_offset{0.0};
  double z_offset{0.0};
  double tilt{0.0};
  bool segmented{false};
  CenterPathValidation validation;
};

std::optional<CenterCandidateResult> evaluate_center_regrasp_candidate(
  MoveGroupInterface & left_arm,
  const moveit::core::RobotState & common_start,
  const planning_scene::PlanningScene & geometric_scene,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  const geometry_msgs::msg::Pose & airborne_object,
  const double table_surface_z,
  const double y_offset,
  const double z_offset,
  const double tilt,
  const bool segmented,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  constexpr double kOriginalZ = 0.019114705;
  constexpr double kOriginalTilt = 50.0 * 0.017453292519943295;
  constexpr double kStagingY = 0.115;
  constexpr double kRequiredCenterHandClearance = 0.005;
  constexpr double kRequiredTableClearance = 0.010;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (segmented) {
    waypoints.push_back(make_inward_tcp_pose(
        airborne_object, kStagingY, kOriginalZ, kOriginalTilt));
    const bool adjusted_staging_differs =
      std::abs(z_offset - kOriginalZ) > 1e-9 ||
      std::abs(tilt - kOriginalTilt) > 1e-9;
    if (adjusted_staging_differs) {
      waypoints.push_back(make_inward_tcp_pose(
          airborne_object, kStagingY, z_offset, tilt));
    }
  }
  waypoints.push_back(make_inward_tcp_pose(
      airborne_object, y_offset, z_offset, tilt));

  left_arm.setStartState(common_start);
  left_arm.clearPoseTargets();
  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  const double fraction = left_arm.computeCartesianPath(
    waypoints, kCartesianEefStep, trajectory, true, &error_code);
  left_arm.setStartStateToCurrentState();
  RCLCPP_INFO(
    logger,
    "Center candidate '%s': local Y/Z %.1f/%.3f mm, tilt %.1f deg, "
    "%s path %.1f%% (%zu points, error=%d).",
    stage_name.c_str(), y_offset * 1000.0, z_offset * 1000.0,
    tilt * 57.29577951308232, segmented ? "segmented" : "direct",
    fraction * 100.0, trajectory.joint_trajectory.points.size(), error_code.val);
  if (fraction < kMinCartesianFraction ||
    trajectory.joint_trajectory.points.size() < 2)
  {
    left_arm.setStartState(common_start);
    moveit_msgs::msg::RobotTrajectory kinematic_trajectory;
    moveit_msgs::msg::MoveItErrorCodes kinematic_error;
    const double kinematic_fraction = left_arm.computeCartesianPath(
      waypoints, kCartesianEefStep, kinematic_trajectory, false, &kinematic_error);
    left_arm.setStartStateToCurrentState();
    RCLCPP_INFO(
      logger,
      "Center candidate '%s' diagnostic without collision checking: %.1f%% "
      "(%zu points, error=%d).",
      stage_name.c_str(), kinematic_fraction * 100.0,
      kinematic_trajectory.joint_trajectory.points.size(), kinematic_error.val);
    if (kinematic_fraction >= kMinCartesianFraction &&
      kinematic_trajectory.joint_trajectory.points.size() >= 2)
    {
      static_cast<void>(validate_center_regrasp_path(
          kinematic_trajectory.joint_trajectory, common_start,
          geometric_scene, validity_client, table_surface_z,
          stage_name + " collision diagnostic", false));
    }
    return std::nullopt;
  }
  auto validation = validate_center_regrasp_path(
    trajectory.joint_trajectory, common_start, geometric_scene,
    validity_client, table_surface_z, stage_name);
  if (!validation.has_value()) {
    return std::nullopt;
  }

  // The arm path is evaluated with the receiver fully open.  Also sample the
  // complete -19..+3 mm closing stroke at the endpoint so an apparently safe
  // open-hand pose cannot hide an opposing-hand collision during re-contact.
  auto closing_state = common_start;
  for (const auto & [joint_name, position] : validation->endpoint) {
    closing_state.setVariablePosition(joint_name, position);
  }
  for (int finger_mm = -19; finger_mm <= 3; ++finger_mm) {
    const double finger_position = static_cast<double>(finger_mm) / 1000.0;
    closing_state.setVariablePosition("left_dex1_finger_1_joint", finger_position);
    closing_state.setVariablePosition("left_dex1_finger_2_joint", finger_position);
    closing_state.update();
    if (!closing_state.satisfiesBounds(1e-6) ||
      !full_robot_state_is_valid(
        validity_client, closing_state,
        stage_name + " closing " + std::to_string(finger_mm) + " mm"))
    {
      return std::nullopt;
    }
    const auto clearance = center_regrasp_clearance(
      geometric_scene, closing_state, table_surface_z);
    if (!clearance.has_value() ||
      clearance->hand_to_hand < kRequiredCenterHandClearance ||
      clearance->hand_to_table < kRequiredTableClearance)
    {
      if (clearance.has_value()) {
        RCLCPP_INFO(
          logger,
          "Center candidate '%s' rejected while closing at %d mm: "
          "hand/hand %.3f mm, hand/table %.3f mm.",
          stage_name.c_str(), finger_mm,
          clearance->hand_to_hand * 1000.0,
          clearance->hand_to_table * 1000.0);
      }
      return std::nullopt;
    }
    validation->minimum_hand_clearance = std::min(
      validation->minimum_hand_clearance, clearance->hand_to_hand);
    validation->minimum_table_clearance = std::min(
      validation->minimum_table_clearance, clearance->hand_to_table);
  }

  CenterCandidateResult result;
  result.y_offset = y_offset;
  result.z_offset = z_offset;
  result.tilt = tilt;
  result.segmented = segmented;
  result.validation = std::move(*validation);
  RCLCPP_INFO(
    logger,
    "CENTER_CANDIDATE_PASS y=%.1f_mm z=%.3f_mm tilt=%.1f_deg path=%s "
    "min_hand=%.3f_mm min_table=%.3f_mm joint_margin=%.3f_rad.",
    result.y_offset * 1000.0, result.z_offset * 1000.0,
    result.tilt * 57.29577951308232,
    result.segmented ? "segmented" : "direct",
    result.validation.minimum_hand_clearance * 1000.0,
    result.validation.minimum_table_clearance * 1000.0,
    result.validation.minimum_joint_margin);
  return result;
}

std::optional<CenterCandidateResult> search_center_regrasp_candidates(
  MoveGroupInterface & left_arm,
  const moveit::core::RobotState & common_start,
  const planning_scene::PlanningScene & geometric_scene,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  const geometry_msgs::msg::Pose & airborne_object,
  const double table_surface_z,
  const bool z_sweep_only,
  const double fixed_tilt)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  constexpr double kBaseZ = 0.019114705;
  constexpr double kBaseTilt = 50.0 * 0.017453292519943295;
  double selected_tilt = fixed_tilt;
  if (!z_sweep_only) {
    static_cast<void>(evaluate_center_regrasp_candidate(
        left_arm, common_start, geometric_scene, validity_client,
        airborne_object, table_surface_z,
        0.010, kBaseZ, kBaseTilt, false, "direct baseline Y10"));

    const std::vector<double> tilt_degrees{
      45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0};
    std::vector<CenterCandidateResult> tilt_results;
    for (const double degrees : tilt_degrees) {
      const auto candidate = evaluate_center_regrasp_candidate(
        left_arm, common_start, geometric_scene, validity_client,
        airborne_object, table_surface_z,
        0.010, kBaseZ, degrees * 0.017453292519943295, true,
        "tilt sweep " + std::to_string(static_cast<int>(degrees)) + " deg");
      if (candidate.has_value()) {
        tilt_results.push_back(*candidate);
      }
    }
    if (tilt_results.empty()) {
      RCLCPP_ERROR(logger, "No segmented Y=+10 mm candidate passed the tilt-only sweep.");
      return std::nullopt;
    }
    const auto maximum_clearance = [](const auto & first, const auto & second) {
        return first.validation.minimum_hand_clearance <
               second.validation.minimum_hand_clearance;
      };
    const auto best_tilt = *std::max_element(
      tilt_results.begin(), tilt_results.end(), maximum_clearance);
    selected_tilt = best_tilt.tilt;
    RCLCPP_INFO(
      logger, "Tilt-only sweep selected %.1f deg at fixed Y/Z +10/%.3f mm.",
      selected_tilt * 57.29577951308232, kBaseZ * 1000.0);
  } else {
    RCLCPP_INFO(
      logger,
      "Resuming the single-variable search at Z with fixed Y/tilt +10 mm/%.1f deg.",
      selected_tilt * 57.29577951308232);
  }

  const auto maximum_clearance = [](const auto & first, const auto & second) {
      return first.validation.minimum_hand_clearance <
             second.validation.minimum_hand_clearance;
    };
  std::vector<double> z_offsets;
  if (z_sweep_only) {
    for (int tenths_mm = 201; tenths_mm <= 209; ++tenths_mm) {
      z_offsets.push_back(static_cast<double>(tenths_mm) / 10000.0);
    }
  } else {
    z_offsets = {0.018, 0.019, 0.019114705, 0.020, 0.021, 0.022};
  }
  std::vector<CenterCandidateResult> z_results;
  for (const double z_offset : z_offsets) {
    const auto candidate = evaluate_center_regrasp_candidate(
      left_arm, common_start, geometric_scene, validity_client,
      airborne_object, table_surface_z,
      0.010, z_offset, selected_tilt, true,
      "Z sweep " + std::to_string(z_offset * 1000.0) + " mm");
    if (candidate.has_value()) {
      z_results.push_back(*candidate);
    }
  }
  if (z_results.empty()) {
    RCLCPP_ERROR(logger, "No candidate passed the Z-only sweep at the selected tilt.");
    return std::nullopt;
  }
  const auto best_z = *std::max_element(
    z_results.begin(), z_results.end(), maximum_clearance);
  RCLCPP_INFO(
    logger, "Z-only sweep selected %.3f mm at fixed Y/tilt +10 mm/%.1f deg.",
    best_z.z_offset * 1000.0, best_z.tilt * 57.29577951308232);

  const std::vector<double> y_offsets{0.010, 0.008, 0.005};
  std::vector<CenterCandidateResult> y_results;
  for (const double y_offset : y_offsets) {
    const auto candidate = evaluate_center_regrasp_candidate(
      left_arm, common_start, geometric_scene, validity_client,
      airborne_object, table_surface_z,
      y_offset, best_z.z_offset, best_z.tilt, true,
      "Y sweep " + std::to_string(y_offset * 1000.0) + " mm");
    if (candidate.has_value()) {
      y_results.push_back(*candidate);
    }
  }
  if (y_results.empty()) {
    RCLCPP_ERROR(logger, "No +10/+8/+5 mm center target passed the final Y-only sweep.");
    return std::nullopt;
  }

  // Prefer +8 mm because it leaves 2 mm measurement/execution margin inside
  // the <=10 mm COM gate.  Fall back to +5 mm, then the exact +10 mm boundary.
  const std::vector<double> preference{0.008, 0.005, 0.010};
  for (const double preferred_y : preference) {
    const auto found = std::find_if(
      y_results.begin(), y_results.end(), [preferred_y](const auto & candidate) {
        return std::abs(candidate.y_offset - preferred_y) < 1e-9;
      });
    if (found != y_results.end()) {
      RCLCPP_INFO(
        logger,
        "CENTER_REGRASP_SEARCH_SELECTED y=%.1f_mm z=%.3f_mm tilt=%.1f_deg "
        "min_hand=%.3f_mm min_table=%.3f_mm joint_margin=%.3f_rad.",
        found->y_offset * 1000.0, found->z_offset * 1000.0,
        found->tilt * 57.29577951308232,
        found->validation.minimum_hand_clearance * 1000.0,
        found->validation.minimum_table_clearance * 1000.0,
        found->validation.minimum_joint_margin);
      return *found;
    }
  }
  return std::nullopt;
}

bool wait_for_world_object(
  moveit::planning_interface::PlanningSceneInterface & planning_scene,
  const std::string & object_id,
  const double timeout_seconds = 3.0)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (planning_scene.getAttachedObjects({object_id}).empty() &&
      planning_scene.getObjects({object_id}).count(object_id) == 1)
    {
      return true;
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

bool wait_for_attached_object(
  moveit::planning_interface::PlanningSceneInterface & planning_scene,
  const std::string & object_id,
  const std::string & link_name,
  const double timeout_seconds = 3.0)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto attached = planning_scene.getAttachedObjects({object_id});
    const auto object = attached.find(object_id);
    if (object != attached.end() && object->second.link_name == link_name) {
      return true;
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

bool wait_for_attached_object_with_touch_links(
  moveit::planning_interface::PlanningSceneInterface & planning_scene,
  const std::string & object_id,
  const std::string & link_name,
  std::vector<std::string> expected_touch_links,
  const double timeout_seconds = 3.0)
{
  std::sort(expected_touch_links.begin(), expected_touch_links.end());
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto attached = planning_scene.getAttachedObjects({object_id});
    const auto object = attached.find(object_id);
    if (object != attached.end() && object->second.link_name == link_name) {
      auto actual_touch_links = object->second.touch_links;
      std::sort(actual_touch_links.begin(), actual_touch_links.end());
      if (actual_touch_links == expected_touch_links) {
        return true;
      }
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

bool wait_for_object_absent(
  moveit::planning_interface::PlanningSceneInterface & planning_scene,
  const std::string & object_id,
  const double timeout_seconds = 3.0)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (planning_scene.getAttachedObjects({object_id}).empty() &&
      planning_scene.getObjects({object_id}).empty())
    {
      return true;
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

bool wait_for_sim_duration(
  const rclcpp::Node::SharedPtr & node,
  const double sim_seconds,
  const std::string & reason)
{
  const auto sim_start = node->now();
  const double wall_limit = std::max(
    kMinimumSimulationWallTimeout, sim_seconds * kSlowSimulationWallFactor);
  const auto wall_deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(wall_limit);
  while (std::chrono::steady_clock::now() < wall_deadline) {
    const auto elapsed = node->now() - sim_start;
    if (elapsed.seconds() >= sim_seconds) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  RCLCPP_ERROR(
    node->get_logger(),
    "Simulation clock did not advance %.2f seconds for '%s' within %.1f wall seconds.",
    sim_seconds, reason.c_str(), wall_limit);
  return false;
}

bool refresh_dynamic_octomap_at_stationary_pose(
  const rclcpp::Node::SharedPtr & node,
  MoveGroupInterface & both_arms,
  const rclcpp::Client<std_srvs::srv::Empty>::SharedPtr & clear_octomap_client,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  const double rebuild_seconds,
  const std::string & stage_name)
{
  if (!clear_octomap_client->wait_for_service(3s)) {
    RCLCPP_ERROR(
      node->get_logger(), "OctoMap clear service unavailable for '%s'.", stage_name.c_str());
    return false;
  }
  const auto future = clear_octomap_client->async_send_request(
    std::make_shared<std_srvs::srv::Empty::Request>());
  if (future.wait_for(3s) != std::future_status::ready) {
    RCLCPP_ERROR(
      node->get_logger(), "OctoMap refresh request timed out for '%s'.", stage_name.c_str());
    return false;
  }
  RCLCPP_INFO(
    node->get_logger(),
    "Cleared motion-generated dynamic OctoMap shadow at stationary stage '%s'; "
    "rebuilding from D435 for %.1f s before revalidating.",
    stage_name.c_str(), rebuild_seconds);
  if (!wait_for_sim_duration(node, rebuild_seconds, stage_name + " D435 rebuild")) {
    return false;
  }
  auto state = both_arms.getCurrentState(10.0);
  if (!state) {
    return false;
  }
  state->update();
  const bool valid = state->satisfiesBounds(1e-6) &&
    full_robot_state_is_valid(validity_client, *state, stage_name + " rebuilt scene");
  if (valid) {
    RCLCPP_INFO(
      node->get_logger(),
      "Stationary D435 OctoMap refresh gate passed for '%s'.", stage_name.c_str());
  }
  return valid;
}

double object_lowest_corner_z(
  const geometry_msgs::msg::Pose & pose,
  const double size_x,
  const double size_y,
  const double size_z)
{
  const Eigen::Matrix3d rotation = pose_to_eigen(pose).linear();
  const double vertical_radius =
    std::abs(rotation(2, 0)) * size_x * 0.5 +
    std::abs(rotation(2, 1)) * size_y * 0.5 +
    std::abs(rotation(2, 2)) * size_z * 0.5;
  return pose.position.z - vertical_radius;
}

double object_upright_tilt(const geometry_msgs::msg::Pose & pose)
{
  const Eigen::Vector3d object_up = pose_to_eigen(pose).linear().col(2);
  return std::acos(std::clamp(object_up.z(), -1.0, 1.0));
}

double pose_rotation_distance(
  const geometry_msgs::msg::Pose & first,
  const geometry_msgs::msg::Pose & second)
{
  const Eigen::Matrix3d delta =
    pose_to_eigen(first).linear().transpose() * pose_to_eigen(second).linear();
  return Eigen::AngleAxisd(delta).angle();
}

struct ObjectStabilityReport
{
  GazeboPoseSample final_sample;
  double bottom_clearance{0.0};
  double upright_tilt{0.0};
  double maximum_translation_drift{0.0};
  double maximum_rotation_drift{0.0};
};

bool verify_gazebo_pose_near_reference(
  const rclcpp::Node::SharedPtr & node,
  const GazeboObjectPoseMonitor & monitor,
  const geometry_msgs::msg::Pose & reference,
  const double maximum_translation,
  const double maximum_rotation,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  if (!wait_for_sim_duration(node, 0.20, stage_name + " settle")) {
    return false;
  }
  const auto current = monitor.latest();
  if (!current.has_value()) {
    RCLCPP_ERROR(logger, "No fresh Gazebo pose is available for gate '%s'.", stage_name.c_str());
    return false;
  }
  const Eigen::Vector3d reference_position(
    reference.position.x, reference.position.y, reference.position.z);
  const Eigen::Vector3d current_position(
    current->pose.position.x, current->pose.position.y, current->pose.position.z);
  const double translation = (current_position - reference_position).norm();
  const double rotation = pose_rotation_distance(reference, current->pose);
  constexpr double kRadiansToDegrees = 57.29577951308232;
  RCLCPP_INFO(
    logger, "Gazebo reference gate '%s': displacement %.2f mm / %.2f deg.",
    stage_name.c_str(), translation * 1000.0, rotation * kRadiansToDegrees);
  if (translation > maximum_translation || rotation > maximum_rotation) {
    RCLCPP_ERROR(
      logger,
      "Gazebo reference gate '%s' detected that an approach moved the block; "
      "refusing to close either gripper.", stage_name.c_str());
    return false;
  }
  return true;
}

std::optional<ObjectStabilityReport> verify_gazebo_object_stability(
  const rclcpp::Node::SharedPtr & node,
  const GazeboObjectPoseMonitor & monitor,
  const double table_surface_z,
  const double object_size_x,
  const double object_size_y,
  const double object_size_z,
  const double settle_seconds,
  const double observation_seconds,
  const double minimum_clearance,
  const double maximum_clearance,
  const double maximum_translation_drift,
  const double maximum_rotation_drift,
  const double maximum_upright_tilt,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_air_handover");
  if (!wait_for_sim_duration(node, settle_seconds, stage_name + " settle")) {
    return std::nullopt;
  }

  const auto first = monitor.latest();
  if (!first.has_value()) {
    RCLCPP_ERROR(logger, "No fresh Gazebo pose is available for gate '%s'.", stage_name.c_str());
    return std::nullopt;
  }

  ObjectStabilityReport report;
  report.final_sample = *first;
  const auto sim_start = node->now();
  const double wall_limit = std::max(
    kMinimumSimulationWallTimeout,
    observation_seconds * kSlowSimulationWallFactor);
  const auto wall_deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(wall_limit);
  std::uint64_t last_sequence = first->sequence;
  std::size_t fresh_sample_count = 0;
  while ((node->now() - sim_start).seconds() < observation_seconds &&
    std::chrono::steady_clock::now() < wall_deadline)
  {
    const auto sample = monitor.latest();
    if (sample.has_value() && sample->sequence != last_sequence) {
      last_sequence = sample->sequence;
      ++fresh_sample_count;
      report.final_sample = *sample;
      const Eigen::Vector3d first_position(
        first->pose.position.x, first->pose.position.y, first->pose.position.z);
      const Eigen::Vector3d position(
        sample->pose.position.x, sample->pose.position.y, sample->pose.position.z);
      report.maximum_translation_drift = std::max(
        report.maximum_translation_drift, (position - first_position).norm());
      report.maximum_rotation_drift = std::max(
        report.maximum_rotation_drift,
        pose_rotation_distance(first->pose, sample->pose));
    }
    std::this_thread::sleep_for(20ms);
  }
  if ((node->now() - sim_start).seconds() < observation_seconds) {
    RCLCPP_ERROR(logger, "Gazebo stability gate '%s' timed out.", stage_name.c_str());
    return std::nullopt;
  }
  const auto final_fresh_sample = monitor.latest(0.50);
  if (!final_fresh_sample.has_value()) {
    RCLCPP_ERROR(
      logger, "Gazebo pose stream became stale before gate '%s' finished.",
      stage_name.c_str());
    return std::nullopt;
  }
  const Eigen::Vector3d first_position(
    first->pose.position.x, first->pose.position.y, first->pose.position.z);
  const Eigen::Vector3d final_position(
    final_fresh_sample->pose.position.x,
    final_fresh_sample->pose.position.y,
    final_fresh_sample->pose.position.z);
  report.maximum_translation_drift = std::max(
    report.maximum_translation_drift, (final_position - first_position).norm());
  report.maximum_rotation_drift = std::max(
    report.maximum_rotation_drift,
    pose_rotation_distance(first->pose, final_fresh_sample->pose));
  report.final_sample = *final_fresh_sample;
  if (fresh_sample_count < 2) {
    RCLCPP_ERROR(
      logger, "Gazebo pose stream did not provide fresh samples for gate '%s'.",
      stage_name.c_str());
    return std::nullopt;
  }

  report.bottom_clearance = object_lowest_corner_z(
    report.final_sample.pose, object_size_x, object_size_y, object_size_z) -
    table_surface_z;
  report.upright_tilt = object_upright_tilt(report.final_sample.pose);
  constexpr double kRadiansToDegrees = 57.29577951308232;
  RCLCPP_INFO(
    logger,
    "Gazebo gate '%s': clearance %.2f mm, tilt %.2f deg, drift %.2f mm / %.2f deg.",
    stage_name.c_str(), report.bottom_clearance * 1000.0,
    report.upright_tilt * kRadiansToDegrees,
    report.maximum_translation_drift * 1000.0,
    report.maximum_rotation_drift * kRadiansToDegrees);

  if (report.bottom_clearance < minimum_clearance ||
    report.bottom_clearance > maximum_clearance ||
    report.maximum_translation_drift > maximum_translation_drift ||
    report.maximum_rotation_drift > maximum_rotation_drift ||
    report.upright_tilt > maximum_upright_tilt)
  {
    RCLCPP_ERROR(
      logger,
      "Gazebo gate '%s' rejected the physical block state; stopping before the next load step.",
      stage_name.c_str());
    return std::nullopt;
  }
  return report;
}

std::optional<geometry_msgs::msg::Pose> wait_for_stable_object_pose(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  const double timeout,
  const double max_age,
  const double max_spread,
  const int64_t sample_count,
  const bool require_orientation_stability)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout));
  std::optional<int64_t> last_stamp;
  std::vector<geometry_msgs::msg::Pose> samples;

  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(kDetectionPollPeriod);
    std::optional<geometry_msgs::msg::PoseStamped> detected;
    {
      std::lock_guard<std::mutex> lock(g_object_pose_mutex);
      detected = g_detected_object_pose;
    }
    if (!detected.has_value()) {
      continue;
    }

    const int64_t stamp = rclcpp::Time(detected->header.stamp).nanoseconds();
    if (last_stamp.has_value() && stamp == *last_stamp) {
      continue;
    }
    last_stamp = stamp;
    const double age = (node->now() - rclcpp::Time(detected->header.stamp)).seconds();
    if (age < 0.0 || age > max_age) {
      continue;
    }

    try {
      geometry_msgs::msg::PoseStamped transformed;
      tf_buffer->transform(
        *detected, transformed, kPlanningFrame, tf2::durationFromSec(0.1));
      const auto & position = transformed.pose.position;
      if (position.x < 0.20 || position.x > 0.40 ||
        position.y < -0.40 || position.y > 0.05 ||
        position.z < -0.19 || position.z > -0.06)
      {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "Ignoring visual sample outside initial pickup region: [%.3f, %.3f, %.3f].",
          position.x, position.y, position.z);
        continue;
      }
      samples.push_back(transformed.pose);
      if (samples.size() > static_cast<size_t>(sample_count)) {
        samples.erase(samples.begin());
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000, "TF transform failed: %s", ex.what());
      continue;
    }

    if (samples.size() != static_cast<size_t>(sample_count)) {
      continue;
    }
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    for (const auto & sample : samples) {
      xs.push_back(sample.position.x);
      ys.push_back(sample.position.y);
      zs.push_back(sample.position.z);
    }
    const auto [x_min, x_max] = std::minmax_element(xs.begin(), xs.end());
    const auto [y_min, y_max] = std::minmax_element(ys.begin(), ys.end());
    const auto [z_min, z_max] = std::minmax_element(zs.begin(), zs.end());
    const double spread = std::hypot(
      std::hypot(*x_max - *x_min, *y_max - *y_min), *z_max - *z_min);
    if (spread > max_spread) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "Visual samples spread %.1f mm; waiting for <= %.1f mm.",
        spread * 1000.0, max_spread * 1000.0);
      continue;
    }
    Eigen::Quaterniond reference_orientation(
      samples.front().orientation.w, samples.front().orientation.x,
      samples.front().orientation.y, samples.front().orientation.z);
    reference_orientation.normalize();
    Eigen::Vector4d quaternion_sum = Eigen::Vector4d::Zero();
    std::vector<Eigen::Quaterniond> sample_orientations;
    sample_orientations.reserve(samples.size());
    for (const auto & sample : samples) {
      Eigen::Quaterniond orientation(
        sample.orientation.w, sample.orientation.x,
        sample.orientation.y, sample.orientation.z);
      orientation.normalize();
      if (reference_orientation.dot(orientation) < 0.0) {
        orientation.coeffs() *= -1.0;
      }
      quaternion_sum += Eigen::Vector4d(
        orientation.w(), orientation.x(), orientation.y(), orientation.z());
      sample_orientations.push_back(orientation);
    }
    if (quaternion_sum.norm() < 1e-9) {
      continue;
    }
    quaternion_sum.normalize();
    const Eigen::Quaterniond averaged_orientation(
      quaternion_sum[0], quaternion_sum[1], quaternion_sum[2], quaternion_sum[3]);
    double maximum_orientation_spread = 0.0;
    for (const auto & orientation : sample_orientations) {
      maximum_orientation_spread = std::max(
        maximum_orientation_spread,
        averaged_orientation.angularDistance(orientation));
    }
    constexpr double kMaximumVisualOrientationSpread =
      5.0 * 0.017453292519943295;
    if (maximum_orientation_spread > kMaximumVisualOrientationSpread) {
      if (require_orientation_stability) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "Visual orientation samples spread %.1f deg; waiting for <= 5.0 deg.",
          maximum_orientation_spread * 57.29577951308232);
        continue;
      }
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "Airborne visual orientation samples spread %.1f deg; retaining the real-camera "
        "position while treating the hand-occluded angle as diagnostic only.",
        maximum_orientation_spread * 57.29577951308232);
    }
    const auto median = [](std::vector<double> values) {
        const auto middle = values.begin() + values.size() / 2;
        std::nth_element(values.begin(), middle, values.end());
        return *middle;
      };
    auto pose = make_object_pose(median(xs), median(ys), median(zs));
    pose.orientation.x = averaged_orientation.x();
    pose.orientation.y = averaged_orientation.y();
    pose.orientation.z = averaged_orientation.z();
    pose.orientation.w = averaged_orientation.w();
    RCLCPP_INFO(
      node->get_logger(),
      "Stable object detection (%ld frames, %.1f mm / %.1f deg spread): "
      "[%.3f, %.3f, %.3f]",
      static_cast<long>(sample_count), spread * 1000.0,
      maximum_orientation_spread * 57.29577951308232,
      pose.position.x, pose.position.y, pose.position.z);
    return pose;
  }
  return std::nullopt;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  options.append_parameter_override("use_sim_time", true);
  auto node = rclcpp::Node::make_shared("h1_2_bimanual_air_handover_demo", options);

  const auto get_double = [&node](const std::string & name, const double default_value) {
      if (!node->has_parameter(name)) {
        return node->declare_parameter<double>(name, default_value);
      }
      const auto parameter = node->get_parameter(name);
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return parameter.as_double();
      }
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        return static_cast<double>(parameter.as_int());
      }
      return std::numeric_limits<double>::quiet_NaN();
    };
  const auto get_bool = [&node](const std::string & name, const bool default_value) {
      if (!node->has_parameter(name)) {
        return node->declare_parameter<bool>(name, default_value);
      }
      const auto parameter = node->get_parameter(name);
      return parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL ?
             parameter.as_bool() : default_value;
    };
  const auto get_string = [&node](
    const std::string & name, const std::string & default_value)
    {
      if (!node->has_parameter(name)) {
        return node->declare_parameter<std::string>(name, default_value);
      }
      const auto parameter = node->get_parameter(name);
      return parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING ?
             parameter.as_string() : default_value;
    };
  std::string mode = "handover";
  if (node->has_parameter("mode")) {
    node->get_parameter("mode", mode);
  } else {
    mode = node->declare_parameter<std::string>("mode", "handover");
  }

  const double velocity_scaling = get_double("velocity_scaling", 0.25);
  const double acceleration_scaling = get_double("acceleration_scaling", 0.20);
  // Empty-space motions may use a faster override, but contact, lift, load
  // transfer and placement retain their separately validated timing.
  const double transfer_motion_velocity_scaling = get_double(
    "transfer_motion_velocity_scaling", 0.20);
  const double transfer_motion_acceleration_scaling = get_double(
    "transfer_motion_acceleration_scaling", 0.20);
  const double receiver_empty_velocity_scaling = get_double(
    "receiver_empty_velocity_scaling", 0.50);
  const double receiver_empty_acceleration_scaling = get_double(
    "receiver_empty_acceleration_scaling", 0.40);
  const double gripper_scaling = get_double("gripper_scaling", 0.20);
  const double object_size_x = get_double("object_size_x", 0.08);
  const double object_size_y = get_double("object_size_y", 0.16);
  const double object_size_z = get_double("object_size_z", 0.03);
  const double right_grasp_y_offset = get_double("right_grasp_y_offset", -0.064);
  const double left_grasp_y_offset = get_double(
    "left_grasp_y_offset", mode == "sequential_center_regrasp_test" ? 0.008 : 0.035);
  const double right_gripper_outward_yaw_degrees = get_double(
    "right_gripper_outward_yaw_degrees", 0.0);
  const double right_gripper_outward_yaw =
    right_gripper_outward_yaw_degrees * 0.017453292519943295;
  const double left_gripper_outward_yaw_degrees = get_double(
    "left_gripper_outward_yaw_degrees", 0.0);
  const double left_gripper_outward_yaw =
    left_gripper_outward_yaw_degrees * 0.017453292519943295;
  // Keep 50 mm between the two TCPs while making their support midpoint
  // coincide with the current small block's local-Y center of mass.
  const double cooperative_grasp_y_offset = get_double("cooperative_grasp_y_offset", 0.025);
  const double transfer_receiver_y_offset = get_double("transfer_receiver_y_offset", 0.025);
  const double transfer_receiver_z_offset = get_double("transfer_receiver_z_offset", 0.0205);
  const double transfer_donor_x_offset = get_double("transfer_donor_x_offset", 0.0);
  // The long-block serial donor uses the physically verified deeper grasp.
  // Its exact fully-open Dex1/table clearance at 60 degrees is 10.541 mm.
  const double grasp_z_offset = get_double("grasp_z_offset", 0.0185);
  const double cooperative_grasp_z_offset = get_double("cooperative_grasp_z_offset", 0.0205);
  const double cooperative_inward_tilt_degrees = get_double(
    "cooperative_inward_tilt_degrees", 75.0);
  const double transfer_receiver_tilt_degrees = get_double(
    "transfer_receiver_tilt_degrees", 80.0);
  const double transfer_receiver_staging_tilt_degrees = get_double(
    "transfer_receiver_staging_tilt_degrees", transfer_receiver_tilt_degrees);
  const double transfer_receiver_staging_z_offset = get_double(
    "transfer_receiver_staging_z_offset", transfer_receiver_z_offset);
  const double transfer_receiver_slide_start_y = get_double(
    "transfer_receiver_slide_start_y", 0.060);
  const double transfer_receiver_staging_y = get_double(
    "transfer_receiver_staging_y", 0.145);
  const double transfer_receiver_retreat = get_double("transfer_receiver_retreat", 0.080);
  const double serial_receiver_prehandover_y_offset = get_double(
    "serial_receiver_prehandover_y_offset", 0.115);
  const double approach_clearance = get_double("approach_clearance", 0.10);
  const double cooperative_donor_approach_clearance = get_double(
    "cooperative_donor_approach_clearance", 0.050);
  const double handover_lift = get_double("handover_lift", 0.125);
  const double pickup_test_lift = get_double("pickup_test_lift", 0.05);
  // Preserve the required 50 mm pickup test.  The long block can settle a few
  // millimetres during the subsequent horizontal carry, so center-regrasp mode
  // may command a small extra Z target only after the pickup gate has passed.
  const double serial_carry_z_compensation = get_double(
    "serial_carry_z_compensation", 0.0);
  const double cooperative_test_lift = get_double("cooperative_test_lift", 0.050);
  const double cooperative_motion_duration = get_double("cooperative_motion_duration", 3.00);
  const double handover_x = get_double("handover_x", 0.315);
  const double handover_y = get_double("handover_y", 0.0);
  const double place_x = get_double("place_x", 0.315);
  const double place_y = get_double("place_y", 0.05);
  const double tabletop_center_z = get_double("tabletop_center_z", -0.170);
  const double soft_contact = get_double("gripper_soft_contact_position", 0.001);
  // The 80 mm gripping width remained symmetric at 3 mm preload; 7 mm drove
  // one right finger to its lower limit and produced a 5 mm pinch-center error.
  const double preload = get_double(
    "gripper_preload_position",
    (mode == "sequential_vision_handover" ||
    mode == "sequential_center_regrasp_test" ||
    mode == "left_cube_to_incubator_rack") ? 0.003 : 0.007);
  // Use the same measured symmetric preload for the small-block serial receiver.
  // The older cooperative-transfer mode retains its separately proven 9 mm default.
  const double transfer_receiver_preload = get_double(
    "transfer_receiver_preload_position",
    (mode == "sequential_vision_handover" ||
    mode == "sequential_center_regrasp_test") ? 0.003 : 0.009);
  const double dual_hold_seconds = get_double("dual_hold_seconds", 2.0);
  const double receiver_hold_seconds = get_double("receiver_hold_seconds", 2.0);
  const double transfer_gate_observation = get_double("transfer_gate_observation", 1.0);
  const double transfer_gate_max_drift = get_double("transfer_gate_max_drift", 0.010);
  // Gazebo contact compliance can create several millimetres of harmless pose
  // noise.  Use a uniform 10 mm distance tolerance for adjacent stages, the
  // accumulated airborne anchor, and the commanded lift height (40..60 mm for
  // the default 50 mm lift).  Angular and table-contact gates remain separate.
  const double transfer_gate_max_anchor_displacement = get_double(
    "transfer_gate_max_anchor_displacement", 0.010);
  const double transfer_gate_air_height_tolerance = get_double(
    "transfer_gate_air_height_tolerance", 0.010);
  const double transfer_gate_max_angle_degrees = get_double(
    "transfer_gate_max_angle_degrees", 5.0);
  const double transfer_gate_max_tilt_degrees = get_double(
    "transfer_gate_max_tilt_degrees", 10.0);
  // The current 80 x 160 x 30 mm block's aligned RGB-D estimate agrees with
  // the Gazebo center to sub-millimetre accuracy.  The old -3 mm correction
  // belonged to the former 100 mm-tall block and made the small block's pinch
  // asymmetric, so keep the current default centered on the real detection.
  const double vision_grasp_x_correction = get_double(
    "vision_grasp_x_correction", 0.0);
  const double airborne_vision_x_correction = get_double(
    "airborne_vision_x_correction", 0.014);
  const double airborne_vision_y_correction = get_double(
    "airborne_vision_y_correction", -0.010);
  const double airborne_vision_z_correction = get_double(
    "airborne_vision_z_correction", -0.002);
  const double detection_timeout = get_double("detection_timeout", 12.0);
  const double detection_max_age = get_double("detection_max_age", 0.50);
  const double detection_max_spread = get_double("detection_max_spread", 0.008);
  const double recovery_table_surface_z = get_double("recovery_table_surface_z", 0.815);
  const bool donor_preflight_only = get_bool("donor_preflight_only", false);
  const bool receiver_preflight_only = get_bool("receiver_preflight_only", false);
  const bool center_regrasp_preflight_only = get_bool(
    "center_regrasp_preflight_only", false);
  const bool center_regrasp_z_sweep_only = get_bool(
    "center_regrasp_z_sweep_only", false);
  // Dedicated left-only rack placement is deliberately preflight-only unless
  // this second, explicit opt-in is true.  The mode string alone never sends
  // a controller command.
  const bool left_rack_execute = get_bool("left_rack_execute", false);
  // The position-only cache is opt-in in both directions.  Export is a
  // no-controller collector from the exact fresh q60/left-clear state; a
  // cache-required run fails closed and never falls back to a new IK search.
  const bool left_rack_cache_export_only = get_bool(
    "left_rack_cache_export_only", false);
  const bool left_rack_cache_required = get_bool(
    "left_rack_cache_required", false);
  // The yaw=90 cabinet-front route passes through a narrow 7-DOF wrist
  // redundancy region while the cube center crosses the front centerline.
  // Keep the TCP and therefore the attached cube fixed while selecting the
  // outward wrist branch; this is position generation only and never grants
  // execution by itself.
  const bool left_rack_fixed_tcp_redundancy_bridge = get_bool(
    "left_rack_fixed_tcp_redundancy_bridge", false);
  const double left_rack_fixed_tcp_bridge_direction_trial_value = get_double(
    "left_rack_fixed_tcp_bridge_direction_trial",
    left_rack_fixed_tcp_redundancy_bridge ? 2.0 : 0.0);
  const int left_rack_fixed_tcp_bridge_direction_trial =
    static_cast<int>(std::llround(left_rack_fixed_tcp_bridge_direction_trial_value));
  const std::string left_rack_cache_path = get_string(
    "left_rack_cache_path",
    "/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/"
    "left_rack_post_door_q60_positions_v1.cache");
  // Fast timing is simulation-only and changes timestamps, never positions.
  // The complete cube-arm phase has a hard 35 simulation-second ceiling.
  const bool left_rack_fast_timing = get_bool("left_rack_fast_timing", false);
  const double left_rack_fast_minimum_knot_period = get_double(
    "left_rack_fast_minimum_knot_period", 0.020);
  const double left_rack_fast_maximum_arm_seconds = get_double(
    "left_rack_fast_maximum_arm_seconds", 35.0);
  // Explicit task-scoped reset for a stopped left-rack simulation trial.  It
  // clears only the failed left carry and never commands the right support.
  const bool left_rack_reset_failed_trial = get_bool(
    "left_rack_reset_failed_trial", false);
  const double center_regrasp_search_tilt_degrees = get_double(
    "center_regrasp_search_tilt_degrees", 55.0);
  const double center_regrasp_overpass_height = get_double(
    "center_regrasp_overpass_height", 0.015);
  const bool cooperative_transfer_lift_only = get_bool(
    "cooperative_transfer_lift_only", false);
  const bool use_gazebo_grasp_constraint = get_bool(
    "use_gazebo_grasp_constraint",
    mode == "sequential_center_regrasp_test" || mode == "right_holder_place_recovery" ||
    mode == "left_cube_to_incubator_rack");

  const bool left_cube_to_incubator_rack_mode =
    mode == "left_cube_to_incubator_rack";
  const bool cooperative_lift_mode = mode == "cooperative_lift_test";
  const bool center_regrasp_mode = mode == "center_regrasp_test";
  const bool cooperative_transfer_mode =
    mode == "cooperative_transfer_test" || center_regrasp_mode;
  const bool receiver_place_recovery_mode = mode == "receiver_place_recovery";
  const bool dual_hold_place_recovery_mode = mode == "dual_hold_place_recovery";
  const bool right_holder_place_recovery_mode = mode == "right_holder_place_recovery";
  const bool center_regrasp_abort_recovery_mode =
    mode == "center_regrasp_abort_recovery";
  const bool sequential_vision_handover_mode = mode == "sequential_vision_handover";
  const bool donor_release_test_mode = mode == "donor_release_test";
  const bool sequential_center_regrasp_test_mode =
    mode == "sequential_center_regrasp_test" || donor_release_test_mode;
  const bool sequential_vision_mode =
    sequential_vision_handover_mode || sequential_center_regrasp_test_mode;
  const bool physical_right_pick_mode =
    mode == "right_pick_test" || sequential_vision_mode;
  const bool full_handover_mode = mode == "handover" || sequential_vision_mode;
  const bool cooperative_mode = cooperative_lift_mode || cooperative_transfer_mode;
  const bool transfer_preflight_only =
    receiver_preflight_only || center_regrasp_preflight_only;
  // The donor and receiver intentionally occupy different local-Y stations so
  // both grippers can fit on the object's local-Y side.  Their combined
  // support, not either hand in isolation, must be centered on the block COM.
  // Keep this as an execution-blocking geometric invariant: a visually valid
  // IK pose is not sufficient if the two support lines create a net moment.
  const double cooperative_support_span =
    cooperative_grasp_y_offset + transfer_receiver_y_offset;
  const double cooperative_support_midpoint_x = transfer_donor_x_offset / 2.0;
  const double cooperative_support_midpoint_y =
    (transfer_receiver_y_offset - cooperative_grasp_y_offset) / 2.0;
  const double cooperative_support_midpoint_z =
    (cooperative_grasp_z_offset + transfer_receiver_z_offset) / 2.0;
  const double generic_cooperative_support_height_mismatch =
    std::abs(cooperative_grasp_z_offset - transfer_receiver_z_offset);
  // Exact STL-vertex analysis of the fully open Dex1 gives a worst-case
  // projection of 25.37 mm below the TCP over the allowed 20..90 degree
  // inward-tilt range.  Round upward to 25.5 mm and require the user's 10 mm
  // mechanical table clearance before any IK or trajectory can be executed.
  constexpr double kOpenDex1MinimumBelowTcp = 0.0255;
  // Exact fully-open Dex1 mesh projection at the serial donor's fixed
  // 60-degree inward tilt.  Unlike the worst-case constant above (which spans
  // every allowed tilt), this preserves the 10 mm table gate while permitting
  // the physically verified 2 mm deeper pad engagement.
  constexpr double kOpenDex1MinimumBelowTcpAt60Degrees = 0.022959104;
  const bool serial_donor_uses_exact_60_degree_profile =
    sequential_center_regrasp_test_mode &&
    std::abs(cooperative_inward_tilt_degrees - 60.0) <= 1e-6;
  const double serial_donor_open_finger_table_clearance =
    object_size_z / 2.0 + grasp_z_offset -
    (serial_donor_uses_exact_60_degree_profile ?
    kOpenDex1MinimumBelowTcpAt60Degrees : kOpenDex1MinimumBelowTcp);
  const double donor_open_finger_table_clearance =
    object_size_z / 2.0 + cooperative_grasp_z_offset - kOpenDex1MinimumBelowTcp;
  const double receiver_open_finger_table_clearance =
    object_size_z / 2.0 + transfer_receiver_z_offset - kOpenDex1MinimumBelowTcp;
  const double receiver_staging_open_finger_table_clearance =
    object_size_z / 2.0 + transfer_receiver_staging_z_offset -
    kOpenDex1MinimumBelowTcp;
  // The centered segmented-transfer profile below was evaluated directly on
  // the installed Dex1 STL meshes, including the complete donor entry,
  // receiver 80->50 degree outer rotation and fixed-50-degree inward slide.
  // These measurements are deliberately enabled only for the exact geometry
  // that produced them.  Any changed dimension or waypoint fails closed into
  // the generic conservative checks above and must be re-evaluated.
  const auto near_profile_value = [](const double value, const double expected,
      const double tolerance)
    {
      return std::isfinite(value) && std::abs(value - expected) <= tolerance;
    };
  const bool mesh_validated_segmented_transfer_profile =
    cooperative_transfer_mode &&
    near_profile_value(object_size_x, 0.080, 1e-6) &&
    near_profile_value(object_size_y, 0.100, 1e-6) &&
    near_profile_value(object_size_z, 0.030, 1e-6) &&
    near_profile_value(transfer_donor_x_offset, 0.0, 1e-6) &&
    near_profile_value(cooperative_grasp_y_offset, 0.035, 1e-6) &&
    near_profile_value(transfer_receiver_y_offset, 0.035, 1e-6) &&
    near_profile_value(cooperative_grasp_z_offset, 0.0205, 1e-6) &&
    near_profile_value(transfer_receiver_z_offset, 0.019114705, 1e-6) &&
    near_profile_value(transfer_receiver_staging_z_offset, 0.0205, 1e-6) &&
    near_profile_value(cooperative_inward_tilt_degrees, 60.0, 1e-6) &&
    near_profile_value(transfer_receiver_tilt_degrees, 50.0, 1e-6) &&
    near_profile_value(transfer_receiver_staging_tilt_degrees, 80.0, 1e-6) &&
    near_profile_value(transfer_receiver_slide_start_y, 0.083, 1e-6) &&
    near_profile_value(transfer_receiver_staging_y, 0.145, 1e-6);
  constexpr double kSegmentedMinimumBaseClearance = 0.0108453;
  constexpr double kSegmentedDonorMinimumTableClearance = 0.012540896;
  constexpr double kSegmentedReceiverMinimumTableClearance = 0.010263252;
  constexpr double kSegmentedDonorPadArea = 468.37698e-6;
  constexpr double kSegmentedReceiverPadArea = 496.19019e-6;
  constexpr double kSegmentedPadAreaRatio = 0.943946;
  constexpr double kSegmentedPadHeightMismatch = 0.0;
  constexpr double kSegmentedSupportMidpointError = 0.000401829;
  const bool mesh_validated_segmented_support_is_safe =
    kSegmentedMinimumBaseClearance >= 0.010 &&
    kSegmentedDonorMinimumTableClearance >= 0.010 &&
    kSegmentedReceiverMinimumTableClearance >= 0.010 &&
    kSegmentedDonorPadArea >= 400.0e-6 &&
    kSegmentedReceiverPadArea >= 400.0e-6 &&
    kSegmentedPadAreaRatio >= 0.80 &&
    kSegmentedPadHeightMismatch <= 0.001 &&
    kSegmentedSupportMidpointError <= 0.010;
  // Keep at least roughly 450 mm^2 of side-pad overlap on the current Dex1
  // geometry.  For a 30 mm-tall block this caps the TCP offset near 22 mm.
  const double maximum_contact_z_offset = object_size_z / 2.0 + 0.007;
  const bool generic_cooperative_support_is_centered =
    cooperative_support_span >= 0.045 && cooperative_support_span <= 0.070 &&
    std::abs(cooperative_support_midpoint_x) <= 0.002 &&
    std::abs(cooperative_support_midpoint_y) <= 0.001 &&
    generic_cooperative_support_height_mismatch <= 0.001 &&
    donor_open_finger_table_clearance >= 0.010 &&
    receiver_open_finger_table_clearance >= 0.010 &&
    receiver_staging_open_finger_table_clearance >= 0.010 &&
    cooperative_grasp_z_offset <= maximum_contact_z_offset &&
    transfer_receiver_z_offset <= maximum_contact_z_offset;
  const bool cooperative_support_is_centered =
    mesh_validated_segmented_transfer_profile ?
    mesh_validated_segmented_support_is_safe : generic_cooperative_support_is_centered;
  if (sequential_vision_handover_mode) {
    RCLCPP_INFO(
      node->get_logger(),
      "Using serial vision handover: right pickup/lift, central carry, fresh D435 detection, "
      "then left takeover and placement.");
    // Reuse the proven legacy ownership/release/place tail after the serial
    // mode-specific pickup, physical gates and fresh receiver pose are set.
    mode = "handover";
  } else if (donor_release_test_mode) {
    RCLCPP_INFO(
      node->get_logger(),
      "Using serial donor-release test: execute the proven center regrasp, transfer the "
      "independent collision-body support from right to left, unload the right donor in "
      "1 mm steps, then verify left-only placement.");
  } else if (sequential_center_regrasp_test_mode) {
    RCLCPP_INFO(
      node->get_logger(),
      "Using safe serial center-regrasp test: right pickup/lift and central carry, "
      "then left COM regrasp while the right donor never releases.");
    // Reuse the dual-contact tail: after the physical two-hand hold, the left
    // receiver unloads and retreats while the right donor remains the owner.
    mode = "dual_contact_test";
  }

  if ((mode != "reset_only" && mode != "pose_test" && mode != "right_pick_test" &&
    !left_cube_to_incubator_rack_mode &&
    !receiver_place_recovery_mode && !dual_hold_place_recovery_mode &&
    !right_holder_place_recovery_mode && !center_regrasp_abort_recovery_mode &&
    !cooperative_mode && mode != "dual_contact_test" && !full_handover_mode) ||
    !std::isfinite(velocity_scaling) || velocity_scaling <= 0.0 || velocity_scaling > 1.0 ||
    !std::isfinite(acceleration_scaling) || acceleration_scaling <= 0.0 ||
    acceleration_scaling > 1.0 ||
    !std::isfinite(transfer_motion_velocity_scaling) ||
    transfer_motion_velocity_scaling <= 0.0 || transfer_motion_velocity_scaling > 1.0 ||
    !std::isfinite(transfer_motion_acceleration_scaling) ||
    transfer_motion_acceleration_scaling <= 0.0 ||
    transfer_motion_acceleration_scaling > 1.0 ||
    !std::isfinite(receiver_empty_velocity_scaling) ||
    receiver_empty_velocity_scaling <= 0.0 || receiver_empty_velocity_scaling > 1.0 ||
    !std::isfinite(receiver_empty_acceleration_scaling) ||
    receiver_empty_acceleration_scaling <= 0.0 ||
    receiver_empty_acceleration_scaling > 1.0 ||
    !std::isfinite(gripper_scaling) || gripper_scaling <= 0.0 || gripper_scaling > 1.0 ||
    !std::isfinite(object_size_x) || object_size_x <= 0.0 ||
    !std::isfinite(object_size_y) || object_size_y <= 0.0 ||
    !std::isfinite(object_size_z) || object_size_z <= 0.0 ||
    !std::isfinite(right_grasp_y_offset) || !std::isfinite(left_grasp_y_offset) ||
    !std::isfinite(right_gripper_outward_yaw_degrees) ||
    right_gripper_outward_yaw_degrees < -180.0 || right_gripper_outward_yaw_degrees > 180.0 ||
    !std::isfinite(left_gripper_outward_yaw_degrees) ||
    left_gripper_outward_yaw_degrees < -180.0 || left_gripper_outward_yaw_degrees > 180.0 ||
    right_grasp_y_offset >= -0.005 ||
    ((!(sequential_center_regrasp_test_mode || center_regrasp_abort_recovery_mode) &&
    left_grasp_y_offset <= 0.02) ||
    ((sequential_center_regrasp_test_mode || center_regrasp_abort_recovery_mode) &&
    (left_grasp_y_offset <= 0.0 || left_grasp_y_offset > 0.010))) ||
    std::abs(right_grasp_y_offset) > object_size_y / 2.0 - 0.01 ||
    std::abs(left_grasp_y_offset) > object_size_y / 2.0 - 0.01 ||
    ((!sequential_vision_mode &&
      (mode == "handover" || mode == "dual_contact_test") &&
      left_grasp_y_offset - right_grasp_y_offset < 0.065) ||
      (sequential_vision_handover_mode &&
      left_grasp_y_offset - right_grasp_y_offset < 0.045) ||
      (sequential_center_regrasp_test_mode &&
      left_grasp_y_offset - right_grasp_y_offset < 0.040)) ||
    !std::isfinite(cooperative_grasp_y_offset) || cooperative_grasp_y_offset < 0.025 ||
    cooperative_grasp_y_offset > object_size_y / 2.0 - 0.015 ||
    !std::isfinite(transfer_receiver_y_offset) || transfer_receiver_y_offset < 0.020 ||
    transfer_receiver_y_offset > 0.035 ||
    !std::isfinite(transfer_receiver_z_offset) || transfer_receiver_z_offset < 0.0 ||
    transfer_receiver_z_offset > 0.05 ||
    !std::isfinite(transfer_donor_x_offset) ||
    std::abs(transfer_donor_x_offset) > 0.005 ||
    !std::isfinite(grasp_z_offset) || grasp_z_offset < 0.0 || grasp_z_offset > 0.10 ||
    !std::isfinite(cooperative_grasp_z_offset) || cooperative_grasp_z_offset < 0.0 ||
    cooperative_grasp_z_offset > 0.05 ||
    !std::isfinite(cooperative_inward_tilt_degrees) ||
    cooperative_inward_tilt_degrees < 20.0 || cooperative_inward_tilt_degrees > 75.0 ||
    !std::isfinite(transfer_receiver_tilt_degrees) ||
    transfer_receiver_tilt_degrees < 45.0 || transfer_receiver_tilt_degrees > 90.0 ||
    !std::isfinite(transfer_receiver_staging_tilt_degrees) ||
    transfer_receiver_staging_tilt_degrees < 45.0 ||
    transfer_receiver_staging_tilt_degrees > 90.0 ||
    !std::isfinite(transfer_receiver_staging_z_offset) ||
    transfer_receiver_staging_z_offset < 0.0 ||
    transfer_receiver_staging_z_offset > 0.05 ||
    !std::isfinite(transfer_receiver_slide_start_y) ||
    transfer_receiver_slide_start_y < transfer_receiver_y_offset + 0.02 ||
    transfer_receiver_slide_start_y > 0.10 ||
    !std::isfinite(transfer_receiver_staging_y) ||
    transfer_receiver_staging_y < transfer_receiver_slide_start_y + 0.03 ||
    transfer_receiver_staging_y < object_size_y / 2.0 + 0.015 ||
    transfer_receiver_staging_y > 0.15 ||
    !std::isfinite(transfer_receiver_retreat) || transfer_receiver_retreat < 0.03 ||
    transfer_receiver_retreat > 0.12 ||
    !std::isfinite(serial_receiver_prehandover_y_offset) ||
    serial_receiver_prehandover_y_offset < left_grasp_y_offset + 0.015 ||
    serial_receiver_prehandover_y_offset > 0.120 ||
    !std::isfinite(approach_clearance) || approach_clearance < 0.08 ||
    !std::isfinite(cooperative_donor_approach_clearance) ||
    cooperative_donor_approach_clearance < 0.04 ||
    cooperative_donor_approach_clearance > 0.10 ||
    !std::isfinite(handover_lift) || handover_lift < 0.08 ||
    !std::isfinite(pickup_test_lift) || pickup_test_lift < 0.02 || pickup_test_lift > 0.05 ||
    !std::isfinite(serial_carry_z_compensation) ||
    serial_carry_z_compensation < 0.0 || serial_carry_z_compensation > 0.010 ||
    !std::isfinite(cooperative_test_lift) || cooperative_test_lift < 0.005 ||
    cooperative_test_lift > 0.05 ||
    !std::isfinite(cooperative_motion_duration) || cooperative_motion_duration < 1.0 ||
    !std::isfinite(soft_contact) || !std::isfinite(preload) ||
    soft_contact < -0.019 || preload <= soft_contact || preload > 0.0245 ||
    !std::isfinite(transfer_receiver_preload) || transfer_receiver_preload <= soft_contact ||
    transfer_receiver_preload > 0.010 ||
    !std::isfinite(dual_hold_seconds) || dual_hold_seconds < 0.0 ||
    !std::isfinite(receiver_hold_seconds) || receiver_hold_seconds < 0.5 ||
    !std::isfinite(transfer_gate_observation) || transfer_gate_observation < 0.5 ||
    !std::isfinite(transfer_gate_max_drift) || transfer_gate_max_drift <= 0.0 ||
    !std::isfinite(transfer_gate_max_anchor_displacement) ||
    transfer_gate_max_anchor_displacement < transfer_gate_max_drift ||
    transfer_gate_max_anchor_displacement > 0.010 ||
    !std::isfinite(transfer_gate_air_height_tolerance) ||
    transfer_gate_air_height_tolerance <= 0.0 ||
    transfer_gate_air_height_tolerance > 0.020 ||
    !std::isfinite(transfer_gate_max_angle_degrees) ||
    transfer_gate_max_angle_degrees <= 0.0 || transfer_gate_max_angle_degrees > 10.0 ||
    !std::isfinite(transfer_gate_max_tilt_degrees) ||
    transfer_gate_max_tilt_degrees <= 0.0 || transfer_gate_max_tilt_degrees > 20.0 ||
    !std::isfinite(vision_grasp_x_correction) ||
    std::abs(vision_grasp_x_correction) > 0.02 ||
    !std::isfinite(airborne_vision_x_correction) ||
    std::abs(airborne_vision_x_correction) > 0.03 ||
    !std::isfinite(airborne_vision_y_correction) ||
    std::abs(airborne_vision_y_correction) > 0.03 ||
    !std::isfinite(airborne_vision_z_correction) ||
    std::abs(airborne_vision_z_correction) > 0.03 ||
    !std::isfinite(detection_timeout) || detection_timeout <= 0.0 ||
    !std::isfinite(detection_max_age) || detection_max_age <= 0.0 ||
    !std::isfinite(detection_max_spread) || detection_max_spread <= 0.0 ||
    !std::isfinite(recovery_table_surface_z) || recovery_table_surface_z < 0.7 ||
    recovery_table_surface_z > 0.9 ||
    !std::isfinite(center_regrasp_search_tilt_degrees) ||
    center_regrasp_search_tilt_degrees < 45.0 ||
    center_regrasp_search_tilt_degrees > 85.0 ||
    !std::isfinite(center_regrasp_overpass_height) ||
    center_regrasp_overpass_height < 0.005 ||
    center_regrasp_overpass_height > 0.030 ||
    (center_regrasp_mode && !center_regrasp_preflight_only) ||
    (sequential_center_regrasp_test_mode &&
    serial_donor_open_finger_table_clearance < 0.010) ||
    (cooperative_transfer_mode && !cooperative_support_is_centered))
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid air-handover parameters or mode '%s'.", mode.c_str());
    rclcpp::shutdown();
    return 2;
  }

  if (cooperative_transfer_mode) {
    if (mesh_validated_segmented_transfer_profile) {
      RCLCPP_INFO(
        node->get_logger(),
        "Mesh-backed segmented support gate passed: minimum base clearance %.3f mm, "
        "donor/receiver minimum table clearance %.3f/%.3f mm, pad area %.1f/%.1f mm^2 "
        "(ratio %.3f), pad-height mismatch %.3f mm, support midpoint error %.3f mm.",
        kSegmentedMinimumBaseClearance * 1000.0,
        kSegmentedDonorMinimumTableClearance * 1000.0,
        kSegmentedReceiverMinimumTableClearance * 1000.0,
        kSegmentedDonorPadArea * 1e6, kSegmentedReceiverPadArea * 1e6,
        kSegmentedPadAreaRatio, kSegmentedPadHeightMismatch * 1000.0,
        kSegmentedSupportMidpointError * 1000.0);
    } else {
      RCLCPP_INFO(
        node->get_logger(),
        "Cooperative support geometry: donor local [x %.1f, y %.1f, z %.1f] mm, "
        "receiver [x 0.0, y %.1f, z %.1f] mm; span %.1f mm, support midpoint "
        "[%.1f, %.1f, %.1f] mm, TCP height mismatch %.1f mm; conservative "
        "open-finger table clearance donor/receiver contact/receiver staging "
        "%.1f/%.1f/%.1f mm.",
        transfer_donor_x_offset * 1000.0, -cooperative_grasp_y_offset * 1000.0,
        cooperative_grasp_z_offset * 1000.0,
        transfer_receiver_y_offset * 1000.0, transfer_receiver_z_offset * 1000.0,
        cooperative_support_span * 1000.0,
        cooperative_support_midpoint_x * 1000.0,
        cooperative_support_midpoint_y * 1000.0,
        cooperative_support_midpoint_z * 1000.0,
        generic_cooperative_support_height_mismatch * 1000.0,
        donor_open_finger_table_clearance * 1000.0,
        receiver_open_finger_table_clearance * 1000.0,
        receiver_staging_open_finger_table_clearance * 1000.0);
    }
  }
  if (sequential_center_regrasp_test_mode) {
    RCLCPP_INFO(
      node->get_logger(),
      "Serial donor open-hand table clearance: %.3f mm at grasp Z %.1f mm / "
      "inward tilt %.1f deg (required >= 10.000 mm; %s mesh projection).",
      serial_donor_open_finger_table_clearance * 1000.0,
      grasp_z_offset * 1000.0, cooperative_inward_tilt_degrees,
      serial_donor_uses_exact_60_degree_profile ? "exact 60-degree" : "worst-case");
  }

  GazeboObjectPoseMonitor gazebo_pose_monitor;
  if ((physical_right_pick_mode || cooperative_transfer_mode || receiver_place_recovery_mode ||
    dual_hold_place_recovery_mode || right_holder_place_recovery_mode ||
    center_regrasp_abort_recovery_mode || left_cube_to_incubator_rack_mode) &&
    !gazebo_pose_monitor.start())
  {
    RCLCPP_ERROR(
      node->get_logger(), "Cannot subscribe to Gazebo pose topic '%s'.", kGazeboPoseTopic);
    rclcpp::shutdown();
    return 1;
  }
  GazeboIndependentGrasp gazebo_grasp_constraint;
  auto incubator_joint_monitor = left_cube_to_incubator_rack_mode ?
    std::make_unique<IncubatorJointMonitor>(node) : nullptr;

  auto object_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/cube_pose", 10, object_pose_callback);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  MoveGroupInterface right_arm(node, "right_arm");
  MoveGroupInterface left_arm(node, "left_arm");
  MoveGroupInterface both_arms(node, "both_arms");
  MoveGroupInterface right_gripper(node, "right_gripper");
  MoveGroupInterface left_gripper(node, "left_gripper");
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  auto ik_client = node->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
  auto validity_client = node->create_client<moveit_msgs::srv::GetStateValidity>(
    "/check_state_validity");
  auto clear_octomap_client = node->create_client<std_srvs::srv::Empty>(
    "/clear_octomap");
  auto get_scene_client = node->create_client<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene");
  auto apply_scene_client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>(
    "/apply_planning_scene");

  for (auto * group : {&right_arm, &left_arm}) {
    group->setPlanningPipelineId("ompl");
    group->setPlannerId("RRTConnectkConfigDefault");
    group->setPlanningTime(12.0);
    group->setNumPlanningAttempts(10);
    group->setMaxVelocityScalingFactor(velocity_scaling);
    group->setMaxAccelerationScalingFactor(acceleration_scaling);
    group->setPoseReferenceFrame(kPlanningFrame);
  }
  right_arm.setEndEffectorLink("right_dex1_tcp");
  left_arm.setEndEffectorLink("left_dex1_tcp");
  both_arms.setPlanningPipelineId("ompl");
  both_arms.setPlannerId("RRTConnectkConfigDefault");
  both_arms.setPlanningTime(12.0);
  both_arms.setNumPlanningAttempts(10);
  both_arms.setMaxVelocityScalingFactor(velocity_scaling);
  both_arms.setMaxAccelerationScalingFactor(acceleration_scaling);
  for (auto * group : {&right_gripper, &left_gripper}) {
    group->setPlanningPipelineId("ompl");
    group->setPlannerId("RRTConnectkConfigDefault");
    group->setPlanningTime(10.0);
    group->setNumPlanningAttempts(8);
    // The donor release deliberately uses 1 mm finger increments, while the
    // physical feedback gate is 0.5 mm.  Keep the planner goal tolerance well
    // inside that gate so a non-zero release command cannot be accepted as an
    // already-satisfied, one-point trajectory.
    group->setGoalJointTolerance(0.0001);
    group->setMaxVelocityScalingFactor(gripper_scaling);
    group->setMaxAccelerationScalingFactor(gripper_scaling);
  }

  bool ready = right_arm.getCurrentState(15.0) && left_arm.getCurrentState(15.0) &&
    both_arms.getCurrentState(15.0) &&
    right_gripper.getCurrentState(15.0) && left_gripper.getCurrentState(15.0);
  if (!ready) {
    RCLCPP_ERROR(node->get_logger(), "No complete joint state from ros2_control.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto synchronized_joint_names = right_arm.getActiveJoints();
  const auto left_active_joints = left_arm.getActiveJoints();
  synchronized_joint_names.insert(
    synchronized_joint_names.end(), left_active_joints.begin(), left_active_joints.end());
  const auto synchronized_motion_limits = load_synchronized_joint_limits(
    node, synchronized_joint_names);
  if (!synchronized_motion_limits.has_value()) {
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const std::map<std::string, double> right_park{
    {"right_shoulder_pitch_joint", 0.0},
    {"right_shoulder_roll_joint", -0.25},
    {"right_shoulder_yaw_joint", 0.0},
    {"right_elbow_joint", 0.0},
    {"right_wrist_roll_joint", 0.0},
    {"right_wrist_pitch_joint", 0.0},
    {"right_wrist_yaw_joint", 0.0},
  };
  const std::map<std::string, double> left_park{
    {"left_shoulder_pitch_joint", 0.0},
    {"left_shoulder_roll_joint", 0.25},
    {"left_shoulder_yaw_joint", 0.0},
    {"left_elbow_joint", 0.60},
    {"left_wrist_roll_joint", 0.0},
    {"left_wrist_pitch_joint", 0.0},
    {"left_wrist_yaw_joint", 0.0},
  };
  std::map<std::string, double> both_park = right_park;
  both_park.insert(left_park.begin(), left_park.end());
  const std::map<std::string, double> right_open{
    {"right_dex1_finger_1_joint", -0.019},
    {"right_dex1_finger_2_joint", -0.019},
  };
  const std::map<std::string, double> left_open{
    {"left_dex1_finger_1_joint", -0.019},
    {"left_dex1_finger_2_joint", -0.019},
  };
  const std::map<std::string, double> right_soft{
    {"right_dex1_finger_1_joint", soft_contact},
    {"right_dex1_finger_2_joint", soft_contact},
  };
  const std::map<std::string, double> right_preload{
    {"right_dex1_finger_1_joint", preload},
    {"right_dex1_finger_2_joint", preload},
  };
  const std::map<std::string, double> right_unload{
    {"right_dex1_finger_1_joint", -0.003},
    {"right_dex1_finger_2_joint", -0.003},
  };
  const std::map<std::string, double> left_soft{
    {"left_dex1_finger_1_joint", soft_contact},
    {"left_dex1_finger_2_joint", soft_contact},
  };
  const std::map<std::string, double> left_preload{
    {"left_dex1_finger_1_joint", preload},
    {"left_dex1_finger_2_joint", preload},
  };
  const std::map<std::string, double> left_transfer_preload{
    {"left_dex1_finger_1_joint", transfer_receiver_preload},
    {"left_dex1_finger_2_joint", transfer_receiver_preload},
  };
  const std::map<std::string, double> left_unload{
    {"left_dex1_finger_1_joint", -0.003},
    {"left_dex1_finger_2_joint", -0.003},
  };

  bool success = true;
  bool right_closed = false;
  bool left_closed = false;
  bool right_pick_physical_pass = true;
  bool cooperative_both_pregrasp_reached = false;
  bool cooperative_right_final_entry_attempted = false;
  bool cooperative_right_contact_reached = false;
  std::optional<geometry_msgs::msg::Pose> right_lift_physical_pose;
  enum class Owner {None, World, Base, Right, Left};
  Owner owner = Owner::None;

  if (left_cube_to_incubator_rack_mode) {
    // This branch must return before the generic handover cleanup below: that
    // cleanup is allowed to detach stale handover objects, while the live q60
    // scene deliberately contains a right-attached door and handle which are
    // read-only invariants for this left-only task.
    const auto finish_left_rack = [&](const int code) {
        executor.cancel();
        if (spin_thread.joinable()) {
          spin_thread.join();
        }
        rclcpp::shutdown();
        return code;
      };
    constexpr double kDegrees = 57.29577951308232;
    constexpr double kExpectedCubeX = 0.334000;
    constexpr double kExpectedCubeY = 0.377000;
    constexpr double kExpectedCubeZ = 0.830000;
    constexpr double kRackCenterX = 0.610890542898;
    constexpr double kRackCenterY = -0.029527393333;
    constexpr double kRackCubeCenterZ = 0.995407082351;
    // The parallel-yaw grasp reaches its deterministic IK boundary 1.5 mm
    // above the ribbed tray.  Release from that small, level clearance and
    // let Gazebo settle the block onto the unchanged exact rack target.
    constexpr double kRackReleaseHeight = 0.0015;
    constexpr double kRackSurfaceZ = 0.980407082351;
    constexpr double kLeftEndGraspLocalZ = 0.0185;
    constexpr double kOverDoorCubeCenterZ = 1.270;
    constexpr double kSupportEntry = 0.015;
    // Simulation-only scoped exception accepted by the operator: the parallel
    // candidate-3 cabinet-front carry keeps a measured positive 0.983 mm
    // wrist/torso gap on the generated path.
    // Every actual collision and the 10 mm environment gate remain active.
    constexpr double kRackCarrySelfClearance = 0.0005;
    const double initial_cube_yaw = get_double(
      "left_rack_initial_cube_yaw_degrees", 90.0) / kDegrees;
    const double rack_placement_yaw = get_double(
      "left_rack_placement_yaw_degrees", 90.0) / kDegrees;
    const double left_end_grasp_tilt = get_double(
      "left_rack_end_grasp_tilt_degrees", 32.0) / kDegrees;
    const double left_end_grasp_phase = get_double(
      "left_rack_end_grasp_phase_degrees", 0.0) / kDegrees;
    const double left_end_grasp_local_y = get_double(
      "left_rack_end_grasp_local_y_mm", 75.0) * 0.001;
    const double phase_contact_compression = get_double(
      "left_rack_phase_contact_compression_mm", 0.0) * 0.001;
    const double front_carry_roll = get_double(
      "left_rack_front_carry_roll_degrees", 0.0) / kDegrees;
    const double front_carry_pitch = get_double(
      "left_rack_front_carry_pitch_degrees", 0.0) / kDegrees;
    const double front_carry_yaw_offset = get_double(
      "left_rack_front_carry_yaw_offset_degrees", 0.0) / kDegrees;
    const double left_end_pregrasp_local_y = left_end_grasp_local_y + 0.040;
    const double inside_hover_height = get_double(
      "left_rack_inside_hover", 0.025);
    const double front_outside_x = get_double(
      "left_rack_front_outside_x", 0.340);
    constexpr double kExpectedDoor = 60.0 / kDegrees;
    constexpr double kExpectedHandle = 90.0 / kDegrees;

    const bool cache_mode = left_rack_cache_export_only || left_rack_cache_required;
    const bool cache_contract_matches =
      std::abs(object_size_x - 0.080) <= 1e-12 &&
      std::abs(object_size_y - 0.160) <= 1e-12 &&
      std::abs(object_size_z - 0.030) <= 1e-12 &&
      std::abs(initial_cube_yaw * kDegrees - 90.0) <= 1e-12 &&
      std::abs(rack_placement_yaw * kDegrees - 90.0) <= 1e-12 &&
      std::abs(left_end_grasp_tilt * kDegrees - 32.0) <= 1e-12 &&
      std::abs(left_end_grasp_phase) <= 1e-12 &&
      std::abs(left_end_grasp_local_y - 0.075) <= 1e-12 &&
      std::abs(phase_contact_compression) <= 1e-12 &&
      std::abs(front_carry_roll) <= 1e-12 &&
      std::abs(front_carry_pitch) <= 1e-12 &&
      std::abs(front_carry_yaw_offset * kDegrees + 90.0) <= 1e-12 &&
      std::abs(inside_hover_height - 0.025) <= 1e-12 &&
      std::abs(front_outside_x - 0.340) <= 1e-12 &&
      !left_rack_fixed_tcp_redundancy_bridge;
    if ((left_rack_cache_export_only && left_rack_cache_required) ||
      (left_rack_cache_export_only && left_rack_execute) ||
      (cache_mode && left_rack_reset_failed_trial) ||
      (cache_mode && !cache_contract_matches) ||
      !std::isfinite(left_rack_fixed_tcp_bridge_direction_trial_value) ||
      std::abs(
        left_rack_fixed_tcp_bridge_direction_trial_value -
        static_cast<double>(left_rack_fixed_tcp_bridge_direction_trial)) > 1e-12 ||
      left_rack_fixed_tcp_bridge_direction_trial < 0 ||
      left_rack_fixed_tcp_bridge_direction_trial > 10 ||
      (cache_mode && left_rack_fixed_tcp_bridge_direction_trial != 0) ||
      (cache_mode && (left_rack_cache_path.empty() || left_rack_cache_path.front() != '/')) ||
      (left_rack_fast_timing &&
      (!std::isfinite(left_rack_fast_minimum_knot_period) ||
      left_rack_fast_minimum_knot_period < 0.010 ||
      left_rack_fast_minimum_knot_period > 0.100 ||
      !std::isfinite(left_rack_fast_maximum_arm_seconds) ||
      left_rack_fast_maximum_arm_seconds <= 0.0 ||
      left_rack_fast_maximum_arm_seconds > 35.0)))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_RACK_CACHE_CONFIGURATION=FAIL: export/load are exclusive, export "
        "requires no execution, reset cannot be combined with cache, the fixed "
        "cube/rack/front-route contract must match, and fast timing must use a "
        "10..100 ms knot floor with a 0..35 s arm ceiling.");
      return finish_left_rack(2);
    }

    if (!use_gazebo_grasp_constraint) {
      RCLCPP_ERROR(
        node->get_logger(),
        "left_cube_to_incubator_rack requires the independent left Gazebo support; "
        "refusing an unsupported physical carry.");
      return finish_left_rack(2);
    }
    if (!incubator_joint_monitor) {
      return finish_left_rack(1);
    }
    std::optional<IncubatorJointSample> incubator_sample;
    std::optional<GazeboPoseSample> cube_sample;
    const auto telemetry_deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < telemetry_deadline &&
      (!incubator_sample.has_value() ||
      (!left_rack_reset_failed_trial && !cube_sample.has_value())))
    {
      incubator_sample = incubator_joint_monitor->latest();
      cube_sample = gazebo_pose_monitor.latest();
      if (!incubator_sample.has_value() ||
        (!left_rack_reset_failed_trial && !cube_sample.has_value()))
      {
        std::this_thread::sleep_for(50ms);
      }
    }
    if (!incubator_sample.has_value() ||
      (!left_rack_reset_failed_trial && !cube_sample.has_value()) ||
      std::abs(incubator_sample->door - kExpectedDoor) > 2.0 / kDegrees ||
      std::abs(incubator_sample->handle - kExpectedHandle) > 5.0 / kDegrees ||
      !std::isfinite(incubator_sample->door_velocity) ||
      !std::isfinite(incubator_sample->handle_velocity) ||
      std::abs(incubator_sample->door_velocity) > 0.002 ||
      std::abs(incubator_sample->handle_velocity) > 0.002)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_RACK_Q60_GATE=FAIL: fresh stationary 60/90 deg door/handle telemetry "
        "is required; no controller command was sent.");
      return finish_left_rack(1);
    }
    const Eigen::Vector3d expected_cube(kExpectedCubeX, kExpectedCubeY, kExpectedCubeZ);
    const Eigen::Vector3d measured_cube = cube_sample.has_value() ?
      Eigen::Vector3d(
        cube_sample->pose.position.x, cube_sample->pose.position.y,
        cube_sample->pose.position.z) :
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    const double initial_cube_error = (measured_cube - expected_cube).norm();
    auto expected_cube_pose = make_object_pose(
      kExpectedCubeX, kExpectedCubeY, kExpectedCubeZ);
    expected_cube_pose.orientation.z = std::sin(0.5 * initial_cube_yaw);
    expected_cube_pose.orientation.w = std::cos(0.5 * initial_cube_yaw);
    const double initial_cube_orientation_error = cube_sample.has_value() ?
      pose_rotation_distance(expected_cube_pose, cube_sample->pose) :
      std::numeric_limits<double>::infinity();
    if (!left_rack_reset_failed_trial && (initial_cube_error > 0.003 ||
      initial_cube_orientation_error > 2.0 / kDegrees)
    ) {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_RACK_CUBE_GATE=FAIL: live cube differs from persistent left spawn by "
        "%.3f mm / %.3f deg; exact configured initial yaw %.3f deg is required for the common "
        "pickup/rack IK branch. No controller command was sent.",
        initial_cube_error * 1000.0, initial_cube_orientation_error * kDegrees,
        initial_cube_yaw * kDegrees);
      return finish_left_rack(1);
    }
    if (left_rack_reset_failed_trial) {
      const bool finite_live_cube = !cube_sample.has_value() || (
        std::isfinite(measured_cube.x()) && std::isfinite(measured_cube.y()) &&
        std::isfinite(measured_cube.z()) &&
        std::isfinite(cube_sample->pose.orientation.x) &&
        std::isfinite(cube_sample->pose.orientation.y) &&
        std::isfinite(cube_sample->pose.orientation.z) &&
        std::isfinite(cube_sample->pose.orientation.w));
      const bool expected_recovery_region = !cube_sample.has_value() || (
        measured_cube.x() >= 0.30 && measured_cube.x() <= 0.85 &&
        measured_cube.y() >= 0.15 && measured_cube.y() <= 0.55 &&
        measured_cube.z() >= 1.00 && measured_cube.z() <= 1.45);
      if (!left_rack_execute || !finite_live_cube || !expected_recovery_region ||
        (cube_sample.has_value() && !verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, cube_sample->pose, 0.004, 3.0 / kDegrees,
          "left-rack stopped attached-cube recovery start")))
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "LEFT_RACK_FAILED_TRIAL_RESET_GATE=FAIL: explicit execution authority, "
          "stationary live cube and the stopped side-carry "
          "region are required; support remains engaged.");
        return finish_left_rack(1);
      }
    }

    const auto body_objects = planning_scene.getObjects({kIncubatorBodyId});
    const auto attached_mechanism = planning_scene.getAttachedObjects(
      {kIncubatorAttachedDoorId, kIncubatorAttachedHandleId});
    const auto body = body_objects.find(kIncubatorBodyId);
    const auto door = attached_mechanism.find(kIncubatorAttachedDoorId);
    const auto handle = attached_mechanism.find(kIncubatorAttachedHandleId);
    const bool exact_scene_ok =
      body != body_objects.end() && body->second.meshes.size() == 2U &&
      body->second.primitives.empty() &&
      door != attached_mechanism.end() && door->second.object.meshes.size() == 1U &&
      door->second.object.primitives.empty() && door->second.link_name.rfind("right_", 0) == 0U &&
      handle != attached_mechanism.end() && handle->second.object.meshes.size() == 1U &&
      handle->second.object.primitives.empty() &&
      handle->second.link_name.rfind("right_", 0) == 0U;
    if (!exact_scene_ok) {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_RACK_EXACT_SCENE_GATE=FAIL: expected original body+tray meshes and "
        "right-attached original door/handle meshes are not authoritative; no motion.");
      return finish_left_rack(1);
    }
    const auto existing_left_cube = planning_scene.getAttachedObjects({kObjectId});
    const auto existing_world_cube = planning_scene.getObjects({kObjectId});
    if (left_rack_reset_failed_trial) {
      const auto attached = existing_left_cube.find(kObjectId);
      const auto expected_touch_links = hand_touch_links("left");
      const bool already_cleared = existing_left_cube.empty() &&
        existing_world_cube.empty() && !cube_sample.has_value();
      bool attachment_ok = already_cleared || (attached != existing_left_cube.end() &&
        existing_left_cube.size() == 1U && existing_world_cube.empty() &&
        attached->second.link_name == "left_dex1_tcp" &&
        attached->second.object.primitives.size() == 1U &&
        attached->second.object.meshes.empty() &&
        attached->second.object.primitives.front().type ==
        shape_msgs::msg::SolidPrimitive::BOX &&
        attached->second.object.primitives.front().dimensions.size() == 3U);
      if (attachment_ok && !already_cleared) {
        const auto & dimensions = attached->second.object.primitives.front().dimensions;
        attachment_ok = std::abs(dimensions[0] - object_size_x) <= 1e-6 &&
          std::abs(dimensions[1] - object_size_y) <= 1e-6 &&
          std::abs(dimensions[2] - object_size_z) <= 1e-6;
      }
      if (attachment_ok && !already_cleared) {
        auto actual_touch_links = attached->second.touch_links;
        auto required_touch_links = expected_touch_links;
        std::sort(actual_touch_links.begin(), actual_touch_links.end());
        std::sort(required_touch_links.begin(), required_touch_links.end());
        attachment_ok = actual_touch_links == required_touch_links;
      }
      if (!attachment_ok) {
        RCLCPP_ERROR(
          node->get_logger(),
          "LEFT_RACK_FAILED_TRIAL_RESET_OWNER_GATE=FAIL: the exact 80x160x30 mm cube "
          "must be uniquely attached to left_dex1_tcp with the certified touch "
          "links and no world duplicate; support remains engaged.");
        return finish_left_rack(1);
      }
    } else if (!existing_left_cube.empty()) {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_RACK_CUBE_OWNER_GATE=FAIL: the planning cube is already attached; "
        "this mode will not detach an unknown owner.");
      return finish_left_rack(1);
    }
    if (!left_rack_reset_failed_trial && !existing_world_cube.empty()) {
      moveit_msgs::msg::CollisionObject removal;
      removal.header.frame_id = kPlanningFrame;
      removal.id = kObjectId;
      removal.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      if (!planning_scene.applyCollisionObject(removal) ||
        !wait_for_object_absent(planning_scene, kObjectId))
      {
        RCLCPP_ERROR(
          node->get_logger(), "Could not synchronously clear the mode-owned stale cube guard.");
        return finish_left_rack(1);
      }
    }

    auto right_state = right_arm.getCurrentState(10.0);
    auto right_gripper_state = right_gripper.getCurrentState(10.0);
    auto initial_left_state = left_arm.getCurrentState(10.0);
    auto initial_left_gripper_state = left_gripper.getCurrentState(10.0);
    if (!right_state || !right_gripper_state || !initial_left_state ||
      !initial_left_gripper_state)
    {
      return finish_left_rack(1);
    }
    right_state->update();
    right_gripper_state->update();
    initial_left_state->update();
    initial_left_gripper_state->update();
    const auto left_rack_joint_names = left_arm.getActiveJoints();
    std::vector<double> left_rack_live_start;
    left_rack_live_start.reserve(left_rack_joint_names.size());
    for (const auto & joint : left_rack_joint_names) {
      left_rack_live_start.push_back(initial_left_state->getVariablePosition(joint));
    }
    if (left_rack_joint_names.size() != 7U ||
      !std::all_of(
        left_rack_live_start.begin(), left_rack_live_start.end(),
        [](const double value) {return std::isfinite(value);}))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_RACK_START_STATE=FAIL: expected seven finite left-arm joints.");
      return finish_left_rack(1);
    }
    if (left_rack_cache_export_only) {
      const std::map<std::string, double> fresh_left_clear{
        {"left_shoulder_pitch_joint", 0.0},
        {"left_shoulder_roll_joint", 0.30},
        {"left_shoulder_yaw_joint", 0.0},
        {"left_elbow_joint", 0.60},
        {"left_wrist_roll_joint", 0.0},
        {"left_wrist_pitch_joint", 0.0},
        {"left_wrist_yaw_joint", 0.0},
      };
      bool fresh_left_clear_ok = true;
      for (const auto & [joint, expected] : fresh_left_clear) {
        fresh_left_clear_ok = fresh_left_clear_ok &&
          std::abs(initial_left_state->getVariablePosition(joint) - expected) <= 0.003;
      }
      fresh_left_clear_ok = fresh_left_clear_ok &&
        std::abs(initial_left_gripper_state->getVariablePosition(
          "left_dex1_finger_1_joint") + 0.019) <= 0.001 &&
        std::abs(initial_left_gripper_state->getVariablePosition(
          "left_dex1_finger_2_joint") + 0.019) <= 0.001;
      if (!fresh_left_clear_ok) {
        RCLCPP_ERROR(
          node->get_logger(),
          "LEFT_RACK_CACHE_EXPORT_FRESH_GATE=FAIL: export-only requires the "
          "fresh post-door q60 left-clear arm and fully open left fingers; no "
          "controller command was sent.");
        return finish_left_rack(1);
      }
      RCLCPP_INFO(
        node->get_logger(),
        "LEFT_RACK_CACHE_EXPORT_FRESH_GATE=PASS: q60 telemetry, persistent cube, "
        "left-clear arm and open fingers are fresh.");
    }
    std::map<std::string, double> frozen_right_arm;
    std::map<std::string, double> frozen_right_gripper;
    for (const auto & joint : right_arm.getActiveJoints()) {
      frozen_right_arm.emplace(joint, right_state->getVariablePosition(joint));
    }
    for (const auto & joint : right_gripper.getActiveJoints()) {
      frozen_right_gripper.emplace(joint, right_gripper_state->getVariablePosition(joint));
    }
    const auto right_support_unchanged = [&]() {
        const bool arm_ok = joint_target_is_reached(
          right_arm, frozen_right_arm, 0.002, "left-rack immutable right arm");
        const bool gripper_ok = joint_target_is_reached(
          right_gripper, frozen_right_gripper, 0.001,
          "left-rack immutable right gripper");
        const auto sample = incubator_joint_monitor->latest();
        const bool mechanism_ok = sample.has_value() &&
          std::abs(sample->door - kExpectedDoor) <= 2.0 / kDegrees &&
          std::abs(sample->handle - kExpectedHandle) <= 5.0 / kDegrees &&
          std::abs(sample->door_velocity) <= 0.002 &&
          std::abs(sample->handle_velocity) <= 0.002;
        if (!arm_ok || !gripper_ok || !mechanism_ok) {
          RCLCPP_ERROR(
            node->get_logger(),
            "LEFT_RACK_RIGHT_SUPPORT_GATE=FAIL: right support or q60 mechanism changed.");
        }
        return arm_ok && gripper_ok && mechanism_ok;
      };

    const auto planning_cube_pose = cube_sample.has_value() ?
      transform_pose_to_planning_frame(cube_sample->pose, tf_buffer) :
      std::optional<geometry_msgs::msg::Pose>{};
    const auto current_tcp = link_pose_in_frame(
      *initial_left_state, "left_dex1_tcp", kPlanningFrame, tf_buffer);
    if ((!left_rack_reset_failed_trial && !planning_cube_pose.has_value()) ||
      !current_tcp.has_value())
    {
      return finish_left_rack(1);
    }
    if (left_rack_reset_failed_trial) {
      // The user explicitly requested a clean replan rather than a continuation
      // from the cabinet-side collision.  Reset only this left-hand trial:
      // release/delete its simulation cube, clear the matching planning object,
      // retreat the now-empty hand above and in front of the cabinet, then park.
      // The right arm, right gripper, original door and original handle remain
      // frozen throughout.
      bool reset_ok = right_support_unchanged();
      const bool had_attached_cube = !existing_left_cube.empty();
      const bool had_gazebo_cube = cube_sample.has_value();
      if (reset_ok && had_attached_cube) {
        reset_ok = gazebo_grasp_constraint.detach_left_from_cube(true);
      }
      if (reset_ok && had_gazebo_cube) {
        reset_ok = gazebo_grasp_constraint.remove_cube_model_for_reset();
      }
      if (reset_ok && had_attached_cube) {
        reset_ok = remove_left_attached_box(planning_scene);
      }
      if (reset_ok && !planning_scene.getObjects({kObjectId}).empty()) {
        moveit_msgs::msg::CollisionObject removal;
        removal.header.frame_id = kPlanningFrame;
        removal.id = kObjectId;
        removal.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        reset_ok = planning_scene.applyCollisionObject(removal) &&
          wait_for_object_absent(planning_scene, kObjectId);
      }
      if (reset_ok) {
        bool opened = false;
        for (int attempt = 0; attempt < 3 && !opened; ++attempt) {
          const std::string stage =
            "failed left-rack trial reset: open empty left hand attempt " +
            std::to_string(attempt + 1);
          const bool command_ok = execute_prevalidated_joint_target(
            left_gripper, left_open, 0.8, stage);
          opened = gripper_open_target_is_reached(
            left_gripper, left_open, 0.001, stage + " feedback");
          if (!command_ok && !opened) {
            RCLCPP_WARN(
              node->get_logger(),
              "Empty left gripper reset command attempt %d did not reach full open; retrying.",
            attempt + 1);
          }
        }
        if (!opened) {
          const auto gripper_state = left_gripper.getCurrentState(3.0);
          const double finger_1 = gripper_state ? gripper_state->getVariablePosition(
            "left_dex1_finger_1_joint") : 0.0;
          const double finger_2 = gripper_state ? gripper_state->getVariablePosition(
            "left_dex1_finger_2_joint") : 0.0;
          // The cabinet-side contact can temporarily stop one empty finger at
          // about -9 mm.  Both fingers are already separated from the deleted
          // cube, so permit only this outward state for the separating retreat;
          // require full -19 mm feedback again after the hand is parked.
          opened = gripper_state && std::isfinite(finger_1) &&
            std::isfinite(finger_2) && finger_1 <= -0.008 && finger_2 <= -0.008;
          if (!opened && !had_gazebo_cube && !had_attached_cube) {
            // On a resumed reset the task cube is already absent.  A finger
            // can therefore remain cabinet-contact limited even near zero;
            // do not keep driving it into the cabinet.  Move the empty hand
            // out first and require the exact full-open target at the final
            // parked state below.
            opened = gripper_state && std::isfinite(finger_1) &&
              std::isfinite(finger_2);
          }
          if (opened) {
            RCLCPP_WARN(
              node->get_logger(),
              "One empty finger is cabinet-contact limited at %.3f/%.3f mm; "
              "continuing only the separating retreat and retrying full open at park.",
              finger_1 * 1000.0, finger_2 * 1000.0);
          }
        }
        reset_ok = opened;
      }

      auto empty_start = reset_ok ? left_arm.getCurrentState(10.0) : nullptr;
      std::optional<geometry_msgs::msg::Pose> empty_tcp;
      if (empty_start) {
        empty_start->update();
        empty_tcp = link_pose_in_frame(
          *empty_start, "left_dex1_tcp", kPlanningFrame, tf_buffer);
      }
      reset_ok = reset_ok && empty_start != nullptr && empty_tcp.has_value();

      moveit_msgs::msg::RobotTrajectory escape_trajectory;
      moveit::core::RobotState escape_endpoint(
        empty_start ? *empty_start : *initial_left_state);
      if (reset_ok) {
        auto raised = *empty_tcp;
        raised.position.z = std::max(raised.position.z, 0.2225);
        auto outside_front = raised;
        // Read-only sequential IK probing of this exact stopped wrist branch
        // certified the separating x=0.570 / z=0.2225 waypoint with a maximum
        // 3.94 degree joint step.  Park only after reaching that known branch.
        outside_front.position.x = 0.570;
        outside_front.position.y = std::max(raised.position.y, 0.300);
        reset_ok = compute_complete_left_cartesian_path(
          left_arm, *empty_start, {raised, outside_front},
          "failed left-rack trial: separating up-and-front empty-hand retreat",
          escape_trajectory, escape_endpoint, false, true);
      }
      if (reset_ok) {
        reset_ok = left_arm.execute(escape_trajectory) ==
          moveit::core::MoveItErrorCode::SUCCESS;
      }
      if (reset_ok) {
        std::map<std::string, double> escape_target;
        const auto & path = escape_trajectory.joint_trajectory;
        for (size_t index = 0; index < path.joint_names.size(); ++index) {
          escape_target.emplace(
            path.joint_names[index], path.points.back().positions[index]);
        }
        reset_ok = joint_target_is_reached(
          left_arm, escape_target, 0.008,
          "failed left-rack trial: empty-hand front clearance");
      }
      if (reset_ok) {
        reset_ok = right_support_unchanged();
      }
      if (reset_ok) {
        const bool collision_aware_park = plan_and_execute_joint(
          left_arm, left_park,
          "failed left-rack trial: park empty left arm for clean replan");
        if (!collision_aware_park) {
          // This branch is explicitly a simulation reset after the task cube
          // has been deleted and the hand has completed a separating Cartesian
          // retreat.  A stale start-contact flag can still make MoveIt reject
          // the park request; the user authorized a task reset, so fall back to
          // a bounded controller interpolation for this empty left arm only.
          RCLCPP_WARN(
            node->get_logger(),
            "Collision-aware empty-arm park was rejected after the separating "
            "retreat; using the explicit simulation-reset joint interpolation.");
          reset_ok = execute_prevalidated_joint_target(
            left_arm, left_park, 4.0,
            "failed left-rack trial: explicit empty-arm reset to park");
        }
      }
      if (reset_ok) {
        bool parked = joint_target_is_reached(
          left_arm, left_park, 0.010,
          "failed left-rack trial: parked left arm");
        if (!parked) {
          // If the straight park meets the cabinet with shoulder pitch still
          // forward, first fold and move the empty arm laterally to +Y, away
          // from the cabinet, then retry the normal park.
          const std::map<std::string, double> lateral_clear{
            {"left_shoulder_pitch_joint", -0.60},
            {"left_shoulder_roll_joint", 0.75},
            {"left_shoulder_yaw_joint", 0.0},
            {"left_elbow_joint", 1.20},
            {"left_wrist_roll_joint", 0.0},
            {"left_wrist_pitch_joint", 0.0},
            {"left_wrist_yaw_joint", 0.0},
          };
          RCLCPP_WARN(
            node->get_logger(),
            "The first empty-arm reset met the cabinet; folding laterally to +Y "
            "before the final park retry.");
          parked = execute_prevalidated_joint_target(
            left_arm, lateral_clear, 4.0,
            "failed left-rack trial: empty-arm lateral clear") &&
            joint_target_is_reached(
            left_arm, lateral_clear, 0.015,
            "failed left-rack trial: lateral-clear feedback") &&
            execute_prevalidated_joint_target(
            left_arm, left_park, 4.0,
            "failed left-rack trial: final park after lateral clear") &&
            joint_target_is_reached(
            left_arm, left_park, 0.010,
            "failed left-rack trial: final parked feedback");
        }
        reset_ok = parked;
      }
      if (reset_ok) {
        reset_ok = execute_prevalidated_joint_target(
          left_gripper, left_open, 0.8,
          "failed left-rack trial: final full-open at park") &&
          gripper_open_target_is_reached(
          left_gripper, left_open, 0.001,
          "failed left-rack trial: final full-open feedback") &&
          right_support_unchanged();
      }
      if (!reset_ok) {
        RCLCPP_ERROR(
          node->get_logger(),
          "LEFT_RACK_FAILED_TRIAL_RESET=FAIL: stopped after the last confirmed "
          "reset stage; no right-side command was issued.");
        return finish_left_rack(1);
      }
      RCLCPP_INFO(
        node->get_logger(),
        "LEFT_RACK_FAILED_TRIAL_RESET=PASS: failed cube removed, empty left "
        "hand cleared and parked; right q60 support unchanged. Respawn the "
        "cube at the persistent left start before running the new front route.");
      return finish_left_rack(0);
    }
    // MoveIt RobotState collision transforms are expressed in the RobotModel
    // frame, whereas task poses in this project are expressed in pelvis.  The
    // exact FCL Dex1/cube certificate therefore needs this explicit transform;
    // Cartesian planning below intentionally continues to use pelvis poses.
    const geometry_msgs::msg::Pose exact_cube_global_pose = eigen_to_pose(
      initial_left_state->getGlobalLinkTransform(kPlanningFrame) *
      pose_to_eigen(*planning_cube_pose));
    const auto phased_inward_tcp_pose = [&](const double local_y) {
        const Eigen::Isometry3d object = pose_to_eigen(*planning_cube_pose);
        Eigen::Isometry3d relative = object.inverse() * pose_to_eigen(
          make_inward_tcp_pose(
            *planning_cube_pose, local_y, kLeftEndGraspLocalZ,
            left_end_grasp_tilt));
        relative.linear() =
          Eigen::AngleAxisd(
          left_end_grasp_phase, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
          relative.linear();
        return eigen_to_pose(object * relative);
      };
    geometry_msgs::msg::Pose contact_tcp = phased_inward_tcp_pose(
      left_end_grasp_local_y);
    geometry_msgs::msg::Pose pregrasp_tcp = phased_inward_tcp_pose(
      left_end_pregrasp_local_y);
    const double end_edge_margin =
      object_size_y * 0.5 - std::abs(left_end_grasp_local_y);
    if (std::abs(object_size_x - 0.080) > 1e-6 ||
      std::abs(object_size_y - 0.160) > 1e-6 ||
      end_edge_margin + 1e-9 < 0.005)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_RACK_END_GRASP_GEOMETRY=FAIL: exact 80x160 mm cube and at least "
        "5.000 mm local-Y edge margin are required (measured %.3f mm).",
        end_edge_margin * 1000.0);
      return finish_left_rack(1);
    }
    RCLCPP_INFO(
      node->get_logger(),
      "LEFT_RACK_COMMON_GRASP: cube yaw=%.3f deg, local Y/Z=%.1f/+18.5 mm, "
      "inward tilt/phase=%.1f/%.1f deg, TCP qxyzw=[%.6f, %.6f, %.6f, %.6f], "
      "long-edge margin %.3f mm.", initial_cube_yaw * kDegrees,
      left_end_grasp_local_y * 1000.0,
      left_end_grasp_tilt * kDegrees,
      left_end_grasp_phase * kDegrees,
      contact_tcp.orientation.x,
      contact_tcp.orientation.y, contact_tcp.orientation.z,
      contact_tcp.orientation.w, end_edge_margin * 1000.0);

    std_msgs::msg::ColorRGBA cube_color;
    cube_color.r = 0.10F;
    cube_color.g = 0.95F;
    cube_color.b = 0.15F;
    cube_color.a = 1.0F;
    const auto apply_world_cube = [&](const geometry_msgs::msg::Pose & pose) {
        return planning_scene.applyCollisionObject(
          make_collision_object(pose, object_size_x, object_size_y, object_size_z),
          cube_color) && wait_for_world_object(planning_scene, kObjectId);
      };
    const auto clear_mode_cube = [&]() {
        moveit_msgs::msg::CollisionObject removal;
        removal.header.frame_id = kPlanningFrame;
        removal.id = kObjectId;
        removal.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        return planning_scene.applyCollisionObject(removal) &&
               wait_for_object_absent(planning_scene, kObjectId);
      };
    if (!apply_world_cube(*planning_cube_pose)) {
      return finish_left_rack(1);
    }

    std::optional<LeftRackPositionAsset> left_rack_cached_asset;
    if (left_rack_cache_required) {
      std::string cache_error;
      left_rack_cached_asset = read_left_rack_position_asset(
        left_rack_cache_path, left_rack_joint_names, left_rack_live_start,
        cache_error);
      if (!left_rack_cached_asset.has_value()) {
        RCLCPP_ERROR(
          node->get_logger(),
          "LEFT_RACK_CACHE_REQUIRED=FAIL: %s; IK fallback is forbidden and no "
          "controller command was sent.", cache_error.c_str());
        clear_mode_cube();
        return finish_left_rack(1);
      }
      RCLCPP_INFO(
        node->get_logger(),
        "LEFT_RACK_CACHE_REQUIRED=PASS: strict position asset loaded from '%s'; "
        "all spatial and timing audits will be replayed.",
        left_rack_cache_path.c_str());
    }

    moveit_msgs::msg::RobotTrajectory approach_trajectory;
    moveit::core::RobotState contact_state(*initial_left_state);
    std::vector<size_t> approach_boundaries;
    bool preflight_ok = false;
    if (left_rack_cached_asset.has_value()) {
      approach_trajectory.joint_trajectory = left_rack_cached_asset->approach;
      approach_boundaries = left_rack_cached_asset->approach_boundaries;
      preflight_ok = set_left_rack_endpoint_state(
        approach_trajectory.joint_trajectory, *initial_left_state, contact_state);
    } else {
      preflight_ok = compute_complete_left_cartesian_segments(
        left_arm, *initial_left_state, {pregrasp_tcp, contact_tcp},
        "open-hand configured-yaw end-pregrasp and 40 mm contact entry",
        approach_trajectory, contact_state, false, true, &approach_boundaries);
    }
    auto approach_scene = preflight_ok ? read_frozen_planning_scene(
      get_scene_client, left_arm.getRobotModel(), "left-rack approach",
      &*planning_cube_pose, object_size_x, object_size_y, object_size_z) : nullptr;
    const auto approach_forward = preflight_ok ? audit_left_rack_path(
      approach_trajectory.joint_trajectory, *initial_left_state, approach_scene,
      -0.019, "approach forward", 0.0, false, true) : LeftRackPathAudit{};
    const auto approach_reverse_path = reverse_left_joint_path(
      approach_trajectory.joint_trajectory);
    const auto approach_reverse = approach_forward.passed ? audit_left_rack_path(
      approach_reverse_path, contact_state, approach_scene, -0.019,
      "approach exact reverse", 0.0, false, true) : LeftRackPathAudit{};
    preflight_ok = preflight_ok && approach_forward.passed && approach_reverse.passed;

    // A non-zero in-plane grasp phase changes the exact width presented to
    // the two opposing Dex1 pads even though the carried cube's world yaw is
    // still fixed independently.  Use the rectangular projection here so
    // read-only phase trials do not create artificial non-pad penetration by
    // continuing to command the phase-zero 80 mm gap.
    const double phase_magnitude = std::abs(left_end_grasp_phase);
    const double modeled_grasp_width =
      object_size_x * std::cos(phase_magnitude) +
      object_size_y * std::sin(phase_magnitude);
    const double modeled_contact_position =
      0.5 * (kDex1PadGapAtZero - modeled_grasp_width) +
      phase_contact_compression;
    // Fast simulation trial tolerance authorized for this placement pass.
    // Exact mesh penetration remains forbidden; no additional positive
    // non-pad stand-off is required for the tightly fitting original Dex1.
    constexpr double kLeftRackNonpadClearance = 0.0;
    if (preflight_ok && (modeled_contact_position < -0.019 ||
      modeled_contact_position > 0.0245))
    {
      preflight_ok = false;
    }
    ExactBoxGraspReport contact_report;
    if (preflight_ok) {
      for (int step = 0; step <= 20 && preflight_ok; ++step) {
        const double alpha = static_cast<double>(step) / 20.0;
        const double finger = -0.019 + alpha * (modeled_contact_position + 0.019);
        auto closing_state = contact_state;
        closing_state.setVariablePosition("left_dex1_finger_1_joint", finger);
        closing_state.setVariablePosition("left_dex1_finger_2_joint", finger);
        closing_state.update();
        const auto report = exact_left_box_grasp_report(
          closing_state, exact_cube_global_pose,
          object_size_x, object_size_y, object_size_z);
        if (!report.has_value() || report->nonpad_collision ||
          report->nonpad_distance < kLeftRackNonpadClearance)
        {
          if (report.has_value()) {
            RCLCPP_ERROR(
              node->get_logger(),
              "LEFT_RACK_EXACT_BOX_GRASP=FAIL at finger %.3f mm: non-pad '%s' "
              "clearance %.3f mm.", finger * 1000.0,
              report->nearest_nonpad_link.c_str(), report->nonpad_distance * 1000.0);
          }
          preflight_ok = false;
          break;
        }
        if (step == 20) {
          contact_report = *report;
        }
      }
    }
    if (preflight_ok) {
      const bool dual_pad_contact =
        (contact_report.pad1_collision || contact_report.pad1_distance <= 0.00075) &&
        (contact_report.pad2_collision || contact_report.pad2_distance <= 0.00075);
      preflight_ok = dual_pad_contact;
      RCLCPP_INFO(
        node->get_logger(),
        "LEFT_RACK_EXACT_BOX_GRASP: pad1/pad2 %.3f/%.3f mm, non-pad %.3f mm "
        "at modeled %.3f mm projected contact q=%.3f mm (compression %.3f mm) -> %s.",
        contact_report.pad1_distance * 1000.0, contact_report.pad2_distance * 1000.0,
        contact_report.nonpad_distance * 1000.0, modeled_grasp_width * 1000.0,
        modeled_contact_position * 1000.0, phase_contact_compression * 1000.0,
        preflight_ok ? "PASS" : "FAIL");
    }
    if (preflight_ok) {
      auto patch_state = contact_state;
      constexpr double kContactPatchPenetrationPerPad = 0.00025;
      patch_state.setVariablePosition(
        "left_dex1_finger_1_joint",
        modeled_contact_position + kContactPatchPenetrationPerPad);
      patch_state.setVariablePosition(
        "left_dex1_finger_2_joint",
        modeled_contact_position + kContactPatchPenetrationPerPad);
      patch_state.update();
      const auto patch_report = exact_left_box_grasp_report(
        patch_state, exact_cube_global_pose,
        object_size_x, object_size_y, object_size_z);
      double pad1_mean_x = 0.0;
      double pad2_mean_x = 0.0;
      double pad1_centroid_edge = -std::numeric_limits<double>::infinity();
      double pad2_centroid_edge = -std::numeric_limits<double>::infinity();
      if (patch_report.has_value() && patch_report->pad1_contacts > 0U &&
        patch_report->pad2_contacts > 0U)
      {
        pad1_mean_x = patch_report->pad1_contact_x_sum /
          static_cast<double>(patch_report->pad1_contacts);
        pad2_mean_x = patch_report->pad2_contact_x_sum /
          static_cast<double>(patch_report->pad2_contacts);
        const double pad1_mean_y = patch_report->pad1_contact_y_sum /
          static_cast<double>(patch_report->pad1_contacts);
        const double pad2_mean_y = patch_report->pad2_contact_y_sum /
          static_cast<double>(patch_report->pad2_contacts);
        pad1_centroid_edge = object_size_y * 0.5 - std::abs(pad1_mean_y);
        pad2_centroid_edge = object_size_y * 0.5 - std::abs(pad2_mean_y);
      }
      const bool original_pad_patch_ok = patch_report.has_value() &&
        patch_report->pad1_contacts >= 1U && patch_report->pad2_contacts >= 1U &&
        pad1_mean_x * pad2_mean_x < 0.0 &&
        std::abs(pad1_mean_x) >= object_size_x * 0.5 - 0.002 &&
        std::abs(pad2_mean_x) >= object_size_x * 0.5 - 0.002 &&
        pad1_centroid_edge >= 0.004 && pad2_centroid_edge >= 0.004;
      RCLCPP_INFO(
        node->get_logger(),
        "LEFT_RACK_ORIGINAL_PAD_PATCH_CERTIFICATE: contacts=%zu/%zu, mean local "
        "X=%.3f/%.3f mm, centroid long-edge margin=%.3f/%.3f mm, non-pad "
        "clearance=%.3f mm at 0.25 mm/pad diagnostic preload -> %s.",
        patch_report ? patch_report->pad1_contacts : 0U,
        patch_report ? patch_report->pad2_contacts : 0U,
        pad1_mean_x * 1000.0, pad2_mean_x * 1000.0,
        pad1_centroid_edge * 1000.0, pad2_centroid_edge * 1000.0,
        patch_report ? patch_report->nonpad_distance * 1000.0 : 0.0,
        original_pad_patch_ok ? "PASS" : "FAIL");
      preflight_ok = original_pad_patch_ok;
    }
    contact_state.setVariablePosition(
      "left_dex1_finger_1_joint", modeled_contact_position);
    contact_state.setVariablePosition(
      "left_dex1_finger_2_joint", modeled_contact_position);
    contact_state.update();

    const Eigen::Isometry3d contact_tcp_transform = pose_to_eigen(contact_tcp);
    const geometry_msgs::msg::Pose object_in_tcp = eigen_to_pose(
      contact_tcp_transform.inverse() * pose_to_eigen(*planning_cube_pose));
    if (preflight_ok) {
      preflight_ok = planning_scene.applyAttachedCollisionObject(
        make_left_attached_box(
          object_in_tcp, object_size_x, object_size_y, object_size_z)) &&
        wait_for_attached_object_with_touch_links(
          planning_scene, kObjectId, "left_dex1_tcp", hand_touch_links("left"));
    }
    auto carry_scene = preflight_ok ? read_frozen_planning_scene(
      get_scene_client, left_arm.getRobotModel(), "left-rack attached carry",
      &*planning_cube_pose, object_size_x, object_size_y, object_size_z) : nullptr;
    moveit::core::RobotState attached_contact_state(contact_state);
    if (carry_scene) {
      attached_contact_state = carry_scene->getCurrentState();
      attached_contact_state.setVariablePositions(contact_state.getVariablePositions());
      attached_contact_state.setVariablePosition(
        "left_dex1_finger_1_joint", modeled_contact_position);
      attached_contact_state.setVariablePosition(
        "left_dex1_finger_2_joint", modeled_contact_position);
      attached_contact_state.update();
    } else {
      preflight_ok = false;
    }

    geometry_msgs::msg::Pose target_object_world = make_object_pose(
      kRackCenterX, kRackCenterY, kRackCubeCenterZ);
    target_object_world.orientation.z = std::sin(0.5 * rack_placement_yaw);
    target_object_world.orientation.w = std::cos(0.5 * rack_placement_yaw);
    const auto target_object_planning = transform_pose_to_planning_frame(
      target_object_world, tf_buffer);
    geometry_msgs::msg::Pose target_tcp;
    if (target_object_planning.has_value()) {
      target_tcp = eigen_to_pose(pose_to_eigen(*target_object_planning) *
        pose_to_eigen(object_in_tcp).inverse());
    } else {
      preflight_ok = false;
    }
    const auto tcp_for_object_pose_rpy_world = [&] (
      const double x, const double y, const double z,
      const double roll, const double pitch, const double yaw)
      {
        geometry_msgs::msg::Pose world_object = make_object_pose(x, y, z);
        const Eigen::Quaterniond world_orientation{
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())};
        world_object.orientation.x = world_orientation.x();
        world_object.orientation.y = world_orientation.y();
        world_object.orientation.z = world_orientation.z();
        world_object.orientation.w = world_orientation.w();
        const auto planning_object = transform_pose_to_planning_frame(
          world_object, tf_buffer);
        geometry_msgs::msg::Pose tcp;
        if (planning_object.has_value()) {
          tcp = eigen_to_pose(
            pose_to_eigen(*planning_object) * pose_to_eigen(object_in_tcp).inverse());
        }
        return std::pair<bool, geometry_msgs::msg::Pose>{
          planning_object.has_value(), tcp};
      };
    const auto tcp_for_object_pose_world = [&] (
      const double x, const double y, const double z, const double yaw)
      {
        return tcp_for_object_pose_rpy_world(x, y, z, 0.0, 0.0, yaw);
      };
    const auto tcp_for_object_center_world = [&] (
      const double x, const double y, const double z)
      {
        return tcp_for_object_pose_world(x, y, z, rack_placement_yaw);
      };
    geometry_msgs::msg::Pose inside_hover = target_tcp;
    inside_hover.position.z += inside_hover_height;
    geometry_msgs::msg::Pose support_entry = target_tcp;
    support_entry.position.z += kSupportEntry;
    geometry_msgs::msg::Pose support_release_tcp = target_tcp;
    support_release_tcp.position.z += kRackReleaseHeight;

    moveit_msgs::msg::RobotTrajectory carry_trajectory;
    moveit::core::RobotState support_entry_state(attached_contact_state);
    LeftRackPathAudit carry_forward;
    LeftRackPathAudit carry_reverse;
    std::vector<size_t> carry_boundaries;
    std::vector<geometry_msgs::msg::Pose> selected_corridor_waypoints;
    double selected_corridor_height = std::numeric_limits<double>::quiet_NaN();
    size_t selected_corridor_profile = 0U;
    bool selected_corridor_uses_fixed_tcp_bridge = false;
    if (preflight_ok && left_rack_cached_asset.has_value()) {
      carry_trajectory.joint_trajectory = left_rack_cached_asset->carry;
      carry_boundaries = left_rack_cached_asset->carry_boundaries;
      preflight_ok = set_left_rack_endpoint_state(
        carry_trajectory.joint_trajectory, attached_contact_state,
        support_entry_state);
      if (preflight_ok) {
        trajectory_msgs::msg::JointTrajectory endpoint_only;
        endpoint_only.joint_names = carry_trajectory.joint_trajectory.joint_names;
        endpoint_only.points.push_back(
          carry_trajectory.joint_trajectory.points.back());
        const auto endpoint_audit = audit_left_rack_path(
          endpoint_only, support_entry_state, carry_scene,
          modeled_contact_position, "cached carry endpoint", 0.010,
          false, true, false);
        carry_forward = endpoint_audit.passed ? audit_left_rack_path(
          carry_trajectory.joint_trajectory, attached_contact_state,
          carry_scene, modeled_contact_position, "cached carry forward", 0.010,
          false, true, false, kRackCarrySelfClearance) : LeftRackPathAudit{};
        const auto cached_reverse = reverse_left_joint_path(
          carry_trajectory.joint_trajectory);
        carry_reverse = carry_forward.passed ? audit_left_rack_path(
          cached_reverse, support_entry_state, carry_scene,
          modeled_contact_position, "cached carry exact reverse", 0.010,
          false, true, false, kRackCarrySelfClearance) : LeftRackPathAudit{};
        preflight_ok = endpoint_audit.passed && carry_forward.passed &&
          carry_reverse.passed;
      }
    } else if (preflight_ok) {
      const auto state_point = [](
        const moveit::core::RobotState & state,
        const std::vector<std::string> & joint_names)
        {
          trajectory_msgs::msg::JointTrajectoryPoint point;
          point.positions.reserve(joint_names.size());
          for (const auto & joint : joint_names) {
            point.positions.push_back(state.getVariablePosition(joint));
          }
          return point;
        };
      const auto append_position_path = [](
        trajectory_msgs::msg::JointTrajectory & destination,
        const trajectory_msgs::msg::JointTrajectory & source,
        const bool skip_first)
        {
          if (source.points.empty()) {
            return false;
          }
          if (destination.joint_names.empty()) {
            destination.joint_names = source.joint_names;
            destination.header = source.header;
          } else if (destination.joint_names != source.joint_names) {
            return false;
          }
          const size_t begin = skip_first ? 1U : 0U;
          if (source.points.size() <= begin) {
            return false;
          }
          destination.points.insert(
            destination.points.end(), source.points.begin() + begin, source.points.end());
          return true;
        };
      const auto fixed_tcp_bridge_pose_ok = [&] (
        const trajectory_msgs::msg::JointTrajectory & bridge,
        const moveit::core::RobotState & explicit_start,
        const geometry_msgs::msg::Pose & fixed_tcp,
        const std::string & stage)
        {
          constexpr double kBridgeDenseStep = 0.5 * M_PI / 180.0;
          constexpr double kBridgeMaximumJointStep = 4.9 * M_PI / 180.0;
          constexpr double kBridgeTcpTranslationTolerance = 0.0005;
          constexpr double kBridgeTcpRotationTolerance = 0.1 * M_PI / 180.0;
          if (bridge.points.empty() || bridge.joint_names.empty()) {
            return false;
          }
          moveit::core::RobotState state(explicit_start);
          size_t checked = 0U;
          double maximum_translation = 0.0;
          double maximum_rotation = 0.0;
          for (const auto & point : bridge.points) {
            if (point.positions.size() != bridge.joint_names.size()) {
              return false;
            }
            moveit::core::RobotState next(state);
            next.setVariablePositions(bridge.joint_names, point.positions);
            next.setVariablePosition(
              "left_dex1_finger_1_joint", modeled_contact_position);
            next.setVariablePosition(
              "left_dex1_finger_2_joint", modeled_contact_position);
            next.update();
            double maximum_joint_step = 0.0;
            for (const auto & joint : bridge.joint_names) {
              maximum_joint_step = std::max(
                maximum_joint_step,
                std::abs(
                  next.getVariablePosition(joint) - state.getVariablePosition(joint)));
            }
            if (maximum_joint_step > kBridgeMaximumJointStep + 1e-12) {
              return false;
            }
            const size_t subdivisions = std::max<size_t>(
              1U, static_cast<size_t>(std::ceil(maximum_joint_step / kBridgeDenseStep)));
            for (size_t subdivision = 1U; subdivision <= subdivisions; ++subdivision) {
              moveit::core::RobotState sample(state);
              state.interpolate(
                next, static_cast<double>(subdivision) /
                static_cast<double>(subdivisions), sample);
              sample.setVariablePosition(
                "left_dex1_finger_1_joint", modeled_contact_position);
              sample.setVariablePosition(
                "left_dex1_finger_2_joint", modeled_contact_position);
              sample.update();
              const auto actual_tcp = link_pose_in_frame(
                sample, "left_dex1_tcp", kPlanningFrame, tf_buffer);
              if (!actual_tcp.has_value()) {
                return false;
              }
              const Eigen::Isometry3d expected = pose_to_eigen(fixed_tcp);
              const Eigen::Isometry3d actual = pose_to_eigen(*actual_tcp);
              const double translation =
                (actual.translation() - expected.translation()).norm();
              const double rotation = Eigen::AngleAxisd(
                expected.linear().transpose() * actual.linear()).angle();
              maximum_translation = std::max(maximum_translation, translation);
              maximum_rotation = std::max(maximum_rotation, rotation);
              ++checked;
              if (translation > kBridgeTcpTranslationTolerance + 1e-12 ||
                rotation > kBridgeTcpRotationTolerance + 1e-12)
              {
                return false;
              }
            }
            state = next;
          }
          RCLCPP_INFO(
            node->get_logger(),
            "LEFT_RACK_FIXED_TCP_BRIDGE_POSE '%s': %zu dense samples, maximum "
            "TCP drift %.3f mm / %.3f deg (limits 0.500 mm / 0.100 deg) -> PASS.",
            stage.c_str(), checked, maximum_translation * 1000.0,
            maximum_rotation * kDegrees);
          return true;
        };
      const auto build_fixed_tcp_redundancy_bridge = [&] (
        const moveit::core::RobotState & start,
        const geometry_msgs::msg::Pose & fixed_tcp,
        const std::string & stage,
        moveit_msgs::msg::RobotTrajectory & bridge,
        moveit::core::RobotState & endpoint,
        const double endpoint_self_buffer)
        {
          constexpr double kSeedStep = 0.035;
          constexpr size_t kMaximumSeedSteps = 10U;
          constexpr size_t kMinimumSeedStepsBeforeAccept = 1U;
          constexpr double kMaximumGeneratedStep = 4.0 * M_PI / 180.0;
          constexpr double kMinimumUsefulMotion = 1e-5;
          if (!std::isfinite(endpoint_self_buffer) ||
            endpoint_self_buffer < kRackCarrySelfClearance)
          {
            return false;
          }
          const std::vector<std::string> joint_names = left_arm.getActiveJoints();
          const std::vector<std::pair<std::string, double>> directions{
            {"left_shoulder_roll_joint", -kSeedStep},
            {"left_shoulder_roll_joint", +kSeedStep},
            {"left_shoulder_yaw_joint", +kSeedStep},
            {"left_shoulder_yaw_joint", -kSeedStep},
            {"left_elbow_joint", +kSeedStep},
            {"left_elbow_joint", -kSeedStep},
            {"left_wrist_roll_joint", +kSeedStep},
            {"left_wrist_roll_joint", -kSeedStep},
            {"left_wrist_pitch_joint", +0.5 * kSeedStep},
            {"left_wrist_pitch_joint", -0.5 * kSeedStep},
          };
          for (size_t direction_index = 0U;
            direction_index < directions.size(); ++direction_index)
          {
            if (left_rack_fixed_tcp_bridge_direction_trial != 0 &&
              static_cast<int>(direction_index + 1U) !=
              left_rack_fixed_tcp_bridge_direction_trial)
            {
              continue;
            }
            moveit::core::RobotState current(start);
            trajectory_msgs::msg::JointTrajectory trial;
            trial.joint_names = joint_names;
            trial.points.push_back(state_point(current, joint_names));
            for (size_t seed_step = 1U; seed_step <= kMaximumSeedSteps; ++seed_step) {
              std::map<std::string, double> seed;
              for (const auto & joint : joint_names) {
                seed.emplace(joint, current.getVariablePosition(joint));
              }
              seed.at(directions[direction_index].first) +=
                directions[direction_index].second;
              const std::string attempt = stage + " direction " +
                std::to_string(direction_index + 1U) + "/" +
                std::to_string(directions.size()) + " step " +
                std::to_string(seed_step) + "/" +
                std::to_string(kMaximumSeedSteps);
              const auto solution = compute_seeded_ik_joint_target(
                node, left_arm, ik_client, tf_buffer, fixed_tcp, "left_dex1_tcp",
                seed, attempt, {}, false);
              if (!solution.has_value()) {
                break;
              }
              moveit::core::RobotState next(current);
              double maximum_step = 0.0;
              for (const auto & joint : joint_names) {
                const auto found = solution->find(joint);
                if (found == solution->end()) {
                  maximum_step = std::numeric_limits<double>::infinity();
                  break;
                }
                maximum_step = std::max(
                  maximum_step,
                  std::abs(found->second - current.getVariablePosition(joint)));
                next.setVariablePosition(joint, found->second);
              }
              next.setVariablePosition(
                "left_dex1_finger_1_joint", modeled_contact_position);
              next.setVariablePosition(
                "left_dex1_finger_2_joint", modeled_contact_position);
              next.update();
              if (!std::isfinite(maximum_step) ||
                maximum_step > kMaximumGeneratedStep + 1e-12 ||
                maximum_step < kMinimumUsefulMotion || !next.satisfiesBounds(1e-6))
              {
                RCLCPP_INFO(
                  node->get_logger(),
                  "LEFT_RACK_FIXED_TCP_BRIDGE_REJECT '%s': maximum joint step "
                  "%.3f deg (required %.6f..%.3f deg), bounds=%s.",
                  attempt.c_str(), maximum_step * kDegrees,
                  kMinimumUsefulMotion * kDegrees,
                  kMaximumGeneratedStep * kDegrees,
                  next.satisfiesBounds(1e-6) ? "PASS" : "FAIL");
                break;
              }
              trial.points.push_back(state_point(next, joint_names));
              trajectory_msgs::msg::JointTrajectory endpoint_only;
              endpoint_only.joint_names = joint_names;
              endpoint_only.points.push_back(trial.points.back());
              const auto endpoint_audit = audit_left_rack_path(
                endpoint_only, next, carry_scene, modeled_contact_position,
                attempt + " buffered endpoint", 0.010, false, true, false,
                kRackCarrySelfClearance);
              current = next;
              if (seed_step < kMinimumSeedStepsBeforeAccept ||
                !endpoint_audit.passed ||
                endpoint_audit.minimum_self + 1e-12 < endpoint_self_buffer)
              {
                continue;
              }
              const auto bridge_forward = audit_left_rack_path(
                trial, start, carry_scene, modeled_contact_position,
                attempt + " bridge forward", 0.010, false, true, false,
                kRackCarrySelfClearance);
              const auto reverse = reverse_left_joint_path(trial);
              const auto bridge_reverse = bridge_forward.passed ? audit_left_rack_path(
                reverse, current, carry_scene, modeled_contact_position,
                attempt + " bridge exact reverse", 0.010, false, true, false,
                kRackCarrySelfClearance) :
                LeftRackPathAudit{};
              if (!bridge_forward.passed || !bridge_reverse.passed ||
                !fixed_tcp_bridge_pose_ok(trial, start, fixed_tcp, attempt))
              {
                continue;
              }
              bridge.joint_trajectory = std::move(trial);
              endpoint = current;
              RCLCPP_INFO(
                node->get_logger(),
                "LEFT_RACK_FIXED_TCP_REDUNDANCY_BRIDGE=PASS: direction %zu/%zu, "
                "%zu generated states, endpoint self clearance %.3f mm; no "
                "controller command was sent.",
                direction_index + 1U, directions.size(),
                bridge.joint_trajectory.points.size(),
                endpoint_audit.minimum_self * 1000.0);
              return true;
            }
          }
          RCLCPP_ERROR(
            node->get_logger(),
            "LEFT_RACK_FIXED_TCP_REDUNDANCY_BRIDGE=FAIL: no collision-free "
            "same-TCP branch reached the requested %.3f mm endpoint buffer; "
            "no partial path or controller command was sent.",
            endpoint_self_buffer * 1000.0);
          return false;
        };
      const auto compute_seeded_left_cartesian_segments = [&] (
        const moveit::core::RobotState & start,
        const std::vector<geometry_msgs::msg::Pose> & waypoints,
        const std::string & stage,
        moveit_msgs::msg::RobotTrajectory & trajectory,
        moveit::core::RobotState & endpoint,
        std::vector<size_t> & boundaries)
        {
          // GetCartesianPath may reselect the redundant wrist branch at a
          // segment seam even when its explicit start state is the certified
          // fixed-TCP bridge endpoint.  Sample the identical Cartesian line
          // deterministically and seed every IK request with the complete
          // seven-joint solution accepted for the immediately preceding
          // sample.  The ordinary full-path audit below remains authoritative.
          constexpr double kSeededTranslationStep = 0.001;
          constexpr double kSeededRotationStep = 0.5 * M_PI / 180.0;
          constexpr double kSeededMaximumJointStep = 4.0 * M_PI / 180.0;
          constexpr double kDeterministicRebridgeTrigger = 0.001;
          constexpr double kRebridgeEndpointBuffer = 0.0015;
          constexpr size_t kMaximumFailedStepRebridges = 64U;
          constexpr size_t kMaximumLowClearanceRebridges = 64U;
          if (waypoints.empty()) {
            return false;
          }
          const std::vector<std::string> joint_names = left_arm.getActiveJoints();
          if (joint_names.size() != 7U) {
            return false;
          }
          trajectory = moveit_msgs::msg::RobotTrajectory{};
          auto & path = trajectory.joint_trajectory;
          path.joint_names = joint_names;
          boundaries.clear();
          boundaries.push_back(0U);
          moveit::core::RobotState current(start);
          path.points.push_back(state_point(current, joint_names));
          auto current_tcp = link_pose_in_frame(
            current, "left_dex1_tcp", kPlanningFrame, tf_buffer);
          if (!current_tcp.has_value()) {
            return false;
          }
          size_t total_ik_samples = 0U;
          size_t fixed_tcp_rebridges = 0U;
          size_t failed_step_rebridges = 0U;
          size_t low_clearance_rebridges = 0U;
          const auto rebridge_current = [&] (
            const std::string & reason, const double endpoint_buffer) {
              const auto fixed_pose = link_pose_in_frame(
                current, "left_dex1_tcp", kPlanningFrame, tf_buffer);
              moveit_msgs::msg::RobotTrajectory rebridge;
              moveit::core::RobotState rebridge_endpoint(current);
              if (!fixed_pose.has_value() ||
                !build_fixed_tcp_redundancy_bridge(
                  current, *fixed_pose, reason, rebridge, rebridge_endpoint,
                  endpoint_buffer) ||
                !append_position_path(path, rebridge.joint_trajectory, true))
              {
                return false;
              }
              current = rebridge_endpoint;
              ++fixed_tcp_rebridges;
              return true;
            };
          for (size_t waypoint_index = 0U;
            waypoint_index < waypoints.size(); ++waypoint_index)
          {
            const auto & target = waypoints[waypoint_index];
            const Eigen::Vector3d start_translation(
              current_tcp->position.x, current_tcp->position.y,
              current_tcp->position.z);
            const Eigen::Vector3d target_translation(
              target.position.x, target.position.y, target.position.z);
            Eigen::Quaterniond start_orientation(
              current_tcp->orientation.w, current_tcp->orientation.x,
              current_tcp->orientation.y, current_tcp->orientation.z);
            Eigen::Quaterniond target_orientation(
              target.orientation.w, target.orientation.x,
              target.orientation.y, target.orientation.z);
            start_orientation.normalize();
            target_orientation.normalize();
            if (start_orientation.dot(target_orientation) < 0.0) {
              target_orientation.coeffs() *= -1.0;
            }
            const double translation_distance =
              (target_translation - start_translation).norm();
            const double rotation_distance =
              start_orientation.angularDistance(target_orientation);
            const size_t sample_count = std::max<size_t>(
              1U, static_cast<size_t>(std::ceil(std::max(
                translation_distance / kSeededTranslationStep,
                rotation_distance / kSeededRotationStep))));
            for (size_t sample_index = 1U;
              sample_index <= sample_count; ++sample_index)
            {
              const double ratio = static_cast<double>(sample_index) /
                static_cast<double>(sample_count);
              const Eigen::Vector3d sample_translation =
                start_translation + ratio * (target_translation - start_translation);
              const Eigen::Quaterniond sample_orientation =
                start_orientation.slerp(ratio, target_orientation).normalized();
              geometry_msgs::msg::Pose sample_pose;
              sample_pose.position.x = sample_translation.x();
              sample_pose.position.y = sample_translation.y();
              sample_pose.position.z = sample_translation.z();
              sample_pose.orientation.x = sample_orientation.x();
              sample_pose.orientation.y = sample_orientation.y();
              sample_pose.orientation.z = sample_orientation.z();
              sample_pose.orientation.w = sample_orientation.w();
              std::map<std::string, double> seed;
              for (const auto & joint : joint_names) {
                seed.emplace(joint, current.getVariablePosition(joint));
              }
              const auto continuity_gate = [&] (
                const std::map<std::string, double> & candidate)
                {
                  double maximum_step = 0.0;
                  for (const auto & joint : joint_names) {
                    const auto found = candidate.find(joint);
                    if (found == candidate.end()) {
                      return false;
                    }
                    maximum_step = std::max(
                      maximum_step, std::abs(
                        found->second - current.getVariablePosition(joint)));
                  }
                  return maximum_step <= kSeededMaximumJointStep + 1e-12;
                };
              const std::string sample_stage = stage + " waypoint " +
                std::to_string(waypoint_index + 1U) + "/" +
                std::to_string(waypoints.size()) + " sample " +
                std::to_string(sample_index) + "/" +
                std::to_string(sample_count);
              const auto solution = compute_seeded_ik_joint_target(
                node, left_arm, ik_client, tf_buffer, sample_pose,
                "left_dex1_tcp", seed, sample_stage, continuity_gate, false);
              if (!solution.has_value()) {
                if (failed_step_rebridges >= kMaximumFailedStepRebridges ||
                  !rebridge_current(
                    sample_stage + " deterministic IK-failure rebridge",
                    kRebridgeEndpointBuffer))
                {
                  RCLCPP_ERROR(
                    node->get_logger(),
                    "LEFT_RACK_SEEDED_SUFFIX=FAIL at %s; the bounded direction-2 "
                    "fixed-TCP retry was exhausted and partial paths are forbidden.",
                    sample_stage.c_str());
                  return false;
                }
                ++failed_step_rebridges;
                --sample_index;
                continue;
              }
              moveit::core::RobotState next(current);
              for (const auto & joint : joint_names) {
                next.setVariablePosition(joint, solution->at(joint));
              }
              next.setVariablePosition(
                "left_dex1_finger_1_joint", modeled_contact_position);
              next.setVariablePosition(
                "left_dex1_finger_2_joint", modeled_contact_position);
              next.update();
              trajectory_msgs::msg::JointTrajectory step_path;
              step_path.joint_names = joint_names;
              step_path.points.push_back(state_point(next, joint_names));
              const auto step_audit = audit_left_rack_path(
                step_path, current, carry_scene, modeled_contact_position,
                sample_stage + " deterministic step", 0.010,
                false, true, false, kRackCarrySelfClearance);
              if (!step_audit.passed) {
                if (failed_step_rebridges >= kMaximumFailedStepRebridges ||
                  !rebridge_current(
                    sample_stage + " deterministic rejected-step rebridge",
                    kRebridgeEndpointBuffer))
                {
                  RCLCPP_ERROR(
                    node->get_logger(),
                    "LEFT_RACK_SEEDED_SUFFIX=FAIL: deterministic branch failed "
                    "the unchanged audit at %s and the bounded direction-2 "
                    "fixed-TCP retry was exhausted.", sample_stage.c_str());
                  return false;
                }
                ++failed_step_rebridges;
                --sample_index;
                continue;
              }
              if (step_audit.minimum_self + 1e-12 <
                kDeterministicRebridgeTrigger)
              {
                // Do not commit a marginal Cartesian sample and only then
                // attempt to escape at its already-constrained TCP.  Rebridge
                // from the preceding accepted TCP, then solve this identical
                // sample again from the certified branch.
                if (low_clearance_rebridges >= kMaximumLowClearanceRebridges ||
                  !rebridge_current(
                    sample_stage + " low-clearance rebridge",
                    kRebridgeEndpointBuffer))
                {
                  RCLCPP_ERROR(
                    node->get_logger(),
                    "LEFT_RACK_SEEDED_SUFFIX=FAIL: self clearance %.3f mm "
                    "entered the deterministic %.3f mm rebridge gate at %s, "
                    "but direction 2 could not leave it at fixed TCP.",
                    step_audit.minimum_self * 1000.0,
                    kDeterministicRebridgeTrigger * 1000.0,
                    sample_stage.c_str());
                  return false;
                }
                ++low_clearance_rebridges;
                --sample_index;
                continue;
              }
              path.points.push_back(state_point(next, joint_names));
              current = next;
              ++total_ik_samples;
            }
            boundaries.push_back(path.points.size() - 1U);
            current_tcp = target;
          }
          constexpr double kDirectKnotPeriod = 0.10;
          for (size_t index = 0U; index < path.points.size(); ++index) {
            path.points[index].time_from_start =
              static_cast<builtin_interfaces::msg::Duration>(
              rclcpp::Duration::from_seconds(
                kDirectKnotPeriod * static_cast<double>(index)));
          }
          endpoint = current;
          RCLCPP_INFO(
            node->get_logger(),
            "LEFT_RACK_SEEDED_SUFFIX=PASS: %zu Cartesian waypoint(s), %zu "
            "deterministic full-seed IK sample(s), %zu fixed-TCP low-clearance "
            "rebridge(s), no random branch fallback.",
            waypoints.size(), total_ik_samples, fixed_tcp_rebridges);
          return path.points.size() >= 2U && endpoint.satisfiesBounds(1e-6);
        };
      // Follow the operator-marked route: lift only above the table, traverse
      // laterally along the cabinet front while the whole cube stays outside,
      // rise on the opening centerline, then insert along +X.  Never move to
      // rack depth while Y is still on the cabinet side.
      // Candidate 3 is authoritative for this workflow.  Do not spend minutes
      // searching the rejected alternative heights/profiles.
      const std::vector<double> corridor_heights{1.020};
      // The original cabinet STL starts at X=0.4419 m.  At the required
      // 90-degree yaw, the 160 mm local-Y edge contributes an 80 mm world-X
      // half extent, so the default 0.340 m center leaves about 21.9 mm before
      // the cabinet while moving the fixed-orientation wrist away from
      // its observed Cartesian reach boundary.  Expose it only as a
      // simulation-search parameter for read-only trials.
      const double kFrontOutsideX = front_outside_x;
      bool complete_corridor = false;
      for (const double height : corridor_heights) {
        const auto lift = tcp_for_object_pose_world(
          kExpectedCubeX, kExpectedCubeY, height, initial_cube_yaw);
        const auto rotate_for_front = tcp_for_object_center_world(
          kExpectedCubeX, kExpectedCubeY, height);
        const auto front_same_y_initial_yaw = tcp_for_object_pose_world(
          kFrontOutsideX, kExpectedCubeY, height, initial_cube_yaw);
        const auto front_same_y = tcp_for_object_center_world(
          kFrontOutsideX, kExpectedCubeY, height);
        const auto front_outer_y = tcp_for_object_center_world(
          kFrontOutsideX, 0.335, height);
        const auto front_mid_y = tcp_for_object_center_world(
          kFrontOutsideX, 0.160, height);
        const auto front_mid_y_tilted = tcp_for_object_pose_rpy_world(
          kFrontOutsideX, 0.160, height,
          front_carry_roll, front_carry_pitch,
          rack_placement_yaw + front_carry_yaw_offset);
        const auto front_near_y = tcp_for_object_center_world(
          kFrontOutsideX, 0.065, height);
        const auto front_near_y_tilted = tcp_for_object_pose_rpy_world(
          kFrontOutsideX, 0.065, height,
          front_carry_roll, front_carry_pitch,
          rack_placement_yaw + front_carry_yaw_offset);
        const auto front_aligned_high = tcp_for_object_center_world(
          kFrontOutsideX, kRackCenterY, height);
        const auto front_aligned_high_tilted = tcp_for_object_pose_rpy_world(
          kFrontOutsideX, kRackCenterY, height,
          front_carry_roll, front_carry_pitch,
          rack_placement_yaw + front_carry_yaw_offset);
        const auto front_aligned_rise = tcp_for_object_center_world(
          kFrontOutsideX, kRackCenterY, kRackCubeCenterZ + 0.025);
        const auto front_aligned_rise_tilted = tcp_for_object_pose_rpy_world(
          kFrontOutsideX, kRackCenterY, kRackCubeCenterZ + 0.025,
          front_carry_roll, front_carry_pitch,
          rack_placement_yaw + front_carry_yaw_offset);
        const auto front_aligned_hover = tcp_for_object_center_world(
          0.380, kRackCenterY, kRackCubeCenterZ + inside_hover_height);
        const auto front_aligned_hover_tilted = tcp_for_object_pose_rpy_world(
          0.380, kRackCenterY, kRackCubeCenterZ + inside_hover_height,
          front_carry_roll, front_carry_pitch,
          rack_placement_yaw + front_carry_yaw_offset);
        const auto front_entry_0 = tcp_for_object_center_world(
          0.400, kRackCenterY, kRackCubeCenterZ + inside_hover_height);
        const auto front_entry_0_tilted = tcp_for_object_pose_rpy_world(
          0.400, kRackCenterY, kRackCubeCenterZ + inside_hover_height,
          front_carry_roll, front_carry_pitch,
          rack_placement_yaw + front_carry_yaw_offset);
        const auto front_entry_1 = tcp_for_object_center_world(
          0.450, kRackCenterY, kRackCubeCenterZ + inside_hover_height);
        const auto front_entry_2 = tcp_for_object_center_world(
          0.500, kRackCenterY, kRackCubeCenterZ + inside_hover_height);
        const auto front_entry_3 = tcp_for_object_center_world(
          0.555, kRackCenterY, kRackCubeCenterZ + inside_hover_height);
        const auto front_entry_4 = tcp_for_object_center_world(
          0.590, kRackCenterY, kRackCubeCenterZ + inside_hover_height);
        if (!lift.first || !rotate_for_front.first ||
          !front_same_y_initial_yaw.first || !front_same_y.first ||
          !front_outer_y.first || !front_mid_y.first ||
          !front_mid_y_tilted.first || !front_near_y.first ||
          !front_near_y_tilted.first ||
          !front_aligned_high.first || !front_aligned_high_tilted.first ||
          !front_aligned_rise.first || !front_aligned_rise_tilted.first ||
          !front_aligned_hover.first || !front_aligned_hover_tilted.first ||
          !front_entry_0.first || !front_entry_0_tilted.first ||
          !front_entry_1.first ||
          !front_entry_2.first || !front_entry_3.first ||
          !front_entry_4.first)
        {
          continue;
        }
        const std::vector<std::vector<geometry_msgs::msg::Pose>> profiles{
          {
            lift.second, front_same_y_initial_yaw.second, front_same_y.second,
            front_outer_y.second, front_mid_y.second, front_mid_y_tilted.second,
            front_near_y_tilted.second, front_aligned_high_tilted.second,
            front_aligned_rise_tilted.second, front_aligned_hover_tilted.second,
            front_entry_0.second, front_entry_1.second, front_entry_2.second,
            front_entry_3.second, front_entry_4.second,
            inside_hover, support_entry,
          },
        };
        for (size_t profile_index = 0U;
          profile_index < profiles.size() && !complete_corridor; ++profile_index)
        {
          moveit_msgs::msg::RobotTrajectory candidate_trajectory;
          moveit::core::RobotState candidate_endpoint(attached_contact_state);
          std::vector<size_t> candidate_boundaries;
          const auto & candidate_waypoints = profiles[profile_index];
          const std::string candidate_name =
            "attached cube cabinet-front +X insertion z=" + std::to_string(height) +
            (profile_index == 0U ? " move-then-yaw" : " yaw-then-move") +
            std::to_string(rack_placement_yaw * kDegrees);
          std::vector<geometry_msgs::msg::Pose> audited_waypoints = candidate_waypoints;
          bool candidate_complete = false;
          const bool candidate_uses_late_fixed_tcp_bridge =
            left_rack_fixed_tcp_redundancy_bridge &&
            std::abs(front_carry_roll) <= 1e-12 &&
            std::abs(front_carry_pitch) <= 1e-12 &&
            std::abs(front_carry_yaw_offset + 0.5 * M_PI) <= 1e-12;
          const bool candidate_uses_fixed_tcp_bridge =
            left_rack_fixed_tcp_redundancy_bridge &&
            std::abs(front_carry_roll) <= 1e-12 &&
            std::abs(front_carry_pitch) <= 1e-12 &&
            (std::abs(front_carry_yaw_offset) <= 1e-12 ||
            candidate_uses_late_fixed_tcp_bridge);
          if (candidate_uses_fixed_tcp_bridge)
          {
            const std::vector<geometry_msgs::msg::Pose> bridge_prefix =
              candidate_uses_late_fixed_tcp_bridge ?
              std::vector<geometry_msgs::msg::Pose>{
                lift.second, front_same_y_initial_yaw.second,
                front_same_y.second, front_outer_y.second, front_mid_y.second,
                front_mid_y_tilted.second, front_near_y_tilted.second,
                front_aligned_high_tilted.second,
                front_aligned_rise_tilted.second,
                front_aligned_hover_tilted.second,
                front_entry_0_tilted.second,
              } :
              std::vector<geometry_msgs::msg::Pose>{
                lift.second,
                profile_index == 0U ?
                front_same_y_initial_yaw.second : rotate_for_front.second,
                front_same_y.second, front_outer_y.second, front_mid_y.second,
              };
            const std::vector<geometry_msgs::msg::Pose> bridge_suffix =
              candidate_uses_late_fixed_tcp_bridge ?
              std::vector<geometry_msgs::msg::Pose>{
                front_entry_0.second, front_entry_1.second,
                front_entry_2.second, front_entry_3.second,
                front_entry_4.second, inside_hover, support_entry,
              } :
              std::vector<geometry_msgs::msg::Pose>{
                front_near_y.second, front_aligned_high.second,
                front_aligned_rise.second, front_aligned_hover.second,
                front_entry_0.second, front_entry_1.second,
                front_entry_2.second, front_entry_3.second,
                front_entry_4.second, inside_hover, support_entry,
              };
            moveit_msgs::msg::RobotTrajectory prefix_trajectory;
            moveit::core::RobotState prefix_endpoint(attached_contact_state);
            std::vector<size_t> prefix_boundaries;
            moveit_msgs::msg::RobotTrajectory bridge_trajectory;
            moveit::core::RobotState bridge_endpoint(attached_contact_state);
            moveit_msgs::msg::RobotTrajectory suffix_trajectory;
            std::vector<size_t> suffix_boundaries;
            candidate_complete = compute_complete_left_cartesian_segments(
              left_arm, attached_contact_state, bridge_prefix,
              candidate_name + " pre-bridge", prefix_trajectory, prefix_endpoint,
              false, true, &prefix_boundaries);
            if (candidate_complete) {
              candidate_complete = build_fixed_tcp_redundancy_bridge(
                prefix_endpoint, bridge_prefix.back(),
                candidate_name +
                (candidate_uses_late_fixed_tcp_bridge ?
                " fixed centered pre-parallel TCP" : " fixed front-mid TCP"),
                bridge_trajectory,
                bridge_endpoint, 0.004);
            }
            if (candidate_complete) {
              candidate_complete = compute_seeded_left_cartesian_segments(
                bridge_endpoint, bridge_suffix,
                candidate_name + " deterministic post-bridge",
                suffix_trajectory, candidate_endpoint, suffix_boundaries);
            }
            if (candidate_complete) {
              auto & combined = candidate_trajectory.joint_trajectory;
              candidate_complete = append_position_path(
                combined, prefix_trajectory.joint_trajectory, false) &&
                append_position_path(
                combined, bridge_trajectory.joint_trajectory, true) &&
                append_position_path(
                combined, suffix_trajectory.joint_trajectory, true);
              if (candidate_complete) {
                candidate_boundaries = prefix_boundaries;
                const size_t prefix_size = prefix_trajectory.joint_trajectory.points.size();
                const size_t bridge_size = bridge_trajectory.joint_trajectory.points.size();
                const size_t bridge_finish = prefix_size + bridge_size - 2U;
                if (candidate_boundaries.empty() ||
                  candidate_boundaries.back() != bridge_finish)
                {
                  candidate_boundaries.push_back(bridge_finish);
                }
                for (size_t index = 1U; index < suffix_boundaries.size(); ++index) {
                  candidate_boundaries.push_back(
                    bridge_finish + suffix_boundaries[index]);
                }
                constexpr double kDirectKnotPeriod = 0.10;
                for (size_t index = 0U; index < combined.points.size(); ++index) {
                  combined.points[index].time_from_start =
                    static_cast<builtin_interfaces::msg::Duration>(
                    rclcpp::Duration::from_seconds(
                      kDirectKnotPeriod * static_cast<double>(index)));
                  combined.points[index].velocities.clear();
                  combined.points[index].accelerations.clear();
                  combined.points[index].effort.clear();
                }
                audited_waypoints = bridge_prefix;
                audited_waypoints.insert(
                  audited_waypoints.end(), bridge_suffix.begin(), bridge_suffix.end());
              }
            }
          } else {
            candidate_complete = compute_complete_left_cartesian_segments(
              left_arm, attached_contact_state, candidate_waypoints,
              candidate_name, candidate_trajectory, candidate_endpoint, false, true,
              &candidate_boundaries);
          }
          if (!candidate_complete) {
            continue;
          }
          trajectory_msgs::msg::JointTrajectory endpoint_only;
          endpoint_only.joint_names = candidate_trajectory.joint_trajectory.joint_names;
          endpoint_only.points.push_back(
            candidate_trajectory.joint_trajectory.points.back());
          const auto endpoint_audit = audit_left_rack_path(
            endpoint_only, candidate_endpoint, carry_scene,
            modeled_contact_position, candidate_name + " endpoint", 0.010,
            false, true, false);
          if (!endpoint_audit.passed) {
            RCLCPP_ERROR(
              node->get_logger(),
              "LEFT_RACK_FINAL_SUPPORT_ENTRY_COLLISION=FAIL; corridor search stopped.");
            break;
          }
          const auto candidate_forward = audit_left_rack_path(
            candidate_trajectory.joint_trajectory, attached_contact_state,
            carry_scene, modeled_contact_position,
            candidate_name + " forward", 0.010, false, true, false,
            kRackCarrySelfClearance);
          const auto candidate_reverse_path = reverse_left_joint_path(
            candidate_trajectory.joint_trajectory);
          const auto candidate_reverse = candidate_forward.passed ?
            audit_left_rack_path(
              candidate_reverse_path, candidate_endpoint, carry_scene,
              modeled_contact_position, candidate_name + " exact reverse", 0.010,
              false, true, false, kRackCarrySelfClearance) :
            LeftRackPathAudit{};
          if (!candidate_forward.passed || !candidate_reverse.passed) {
            continue;
          }
          carry_trajectory = std::move(candidate_trajectory);
          support_entry_state = candidate_endpoint;
          carry_forward = candidate_forward;
          carry_reverse = candidate_reverse;
          carry_boundaries = std::move(candidate_boundaries);
          selected_corridor_waypoints = audited_waypoints;
          selected_corridor_height = height;
          selected_corridor_profile = profile_index + 1U;
          selected_corridor_uses_fixed_tcp_bridge =
            candidate_uses_fixed_tcp_bridge;
          complete_corridor = true;
          RCLCPP_INFO(
            node->get_logger(),
            "LEFT_RACK_FRONT_CORRIDOR_SELECTED z=%.3f m, profile=%zu.",
            height, profile_index + 1U);
        }
      }
      preflight_ok = complete_corridor;
    }
    preflight_ok = preflight_ok && carry_forward.passed && carry_reverse.passed;

    moveit_msgs::msg::RobotTrajectory support_trajectory;
    moveit::core::RobotState support_state(support_entry_state);
    std::vector<size_t> support_boundaries;
    if (preflight_ok && left_rack_cached_asset.has_value()) {
      support_trajectory.joint_trajectory = left_rack_cached_asset->support;
      support_boundaries = left_rack_cached_asset->support_boundaries;
      preflight_ok = set_left_rack_endpoint_state(
        support_trajectory.joint_trajectory, support_entry_state, support_state);
    } else if (preflight_ok) {
      // The one-shot Cartesian service can reselect the redundant arm branch
      // on this final short descent and report an incomplete fraction.  Keep
      // the exact vertical TCP line to the level 1.5 mm release clearance,
      // use a fixed 0.5 mm step, and use
      // a deterministic secant/seed sequence at the near-singular final few
      // millimetres.  Every accepted solution is still measured from the
      // immediately preceding complete seven-joint state and the complete
      // forward/reverse collision audits below remain mandatory.
      constexpr size_t kSupportEntrySamples = 27U;
      constexpr double kSupportMaximumJointStep = 4.0 * M_PI / 180.0;
      constexpr double kSupportSeedHalfDegree = 0.5 * M_PI / 180.0;
      const std::vector<std::string> joint_names = left_arm.getActiveJoints();
      const auto support_state_point = [](
        const moveit::core::RobotState & state,
        const std::vector<std::string> & names)
        {
          trajectory_msgs::msg::JointTrajectoryPoint point;
          point.positions.reserve(names.size());
          for (const auto & joint : names) {
            point.positions.push_back(state.getVariablePosition(joint));
          }
          return point;
        };
      auto & support_path = support_trajectory.joint_trajectory;
      support_path.joint_names = joint_names;
      moveit::core::RobotState current(support_entry_state);
      support_path.points.push_back(support_state_point(current, joint_names));
      const geometry_msgs::msg::Pose support_start_pose = support_entry;
      std::optional<std::map<std::string, double>> previous_positions;
      for (size_t sample_index = 1U;
        preflight_ok && sample_index <= kSupportEntrySamples; ++sample_index)
      {
        const double ratio = static_cast<double>(sample_index) /
          static_cast<double>(kSupportEntrySamples);
        geometry_msgs::msg::Pose sample_pose = support_start_pose;
        sample_pose.position.x +=
          ratio * (support_release_tcp.position.x - support_start_pose.position.x);
        sample_pose.position.y +=
          ratio * (support_release_tcp.position.y - support_start_pose.position.y);
        sample_pose.position.z +=
          ratio * (support_release_tcp.position.z - support_start_pose.position.z);
        std::map<std::string, double> seed;
        for (const auto & joint : joint_names) {
          seed.emplace(joint, current.getVariablePosition(joint));
        }
        const auto continuity_gate = [&] (
          const std::map<std::string, double> & candidate)
          {
            double maximum_step = 0.0;
            for (const auto & joint : joint_names) {
              const auto found = candidate.find(joint);
              if (found == candidate.end()) {
                return false;
              }
              maximum_step = std::max(
                maximum_step, std::abs(
                  found->second - current.getVariablePosition(joint)));
            }
            return maximum_step <= kSupportMaximumJointStep + 1e-12;
          };
        std::map<std::string, double> predictor = seed;
        if (previous_positions.has_value()) {
          for (const auto & joint : joint_names) {
            predictor.at(joint) += seed.at(joint) - previous_positions->at(joint);
          }
        }
        struct SupportSeedTrial
        {
          const char * label;
          bool use_predictor;
          const char * perturbed_joint;
          double offset;
        };
        const std::array<SupportSeedTrial, 14U> seed_trials{{
          {"secant", true, "", 0.0},
          {"previous", false, "", 0.0},
          {"secant shoulder-roll +0.5deg", true,
            "left_shoulder_roll_joint", kSupportSeedHalfDegree},
          {"secant shoulder-roll -0.5deg", true,
            "left_shoulder_roll_joint", -kSupportSeedHalfDegree},
          {"secant shoulder-roll +1deg", true,
            "left_shoulder_roll_joint", 2.0 * kSupportSeedHalfDegree},
          {"secant shoulder-roll -1deg", true,
            "left_shoulder_roll_joint", -2.0 * kSupportSeedHalfDegree},
          {"previous shoulder-roll +0.5deg", false,
            "left_shoulder_roll_joint", kSupportSeedHalfDegree},
          {"previous shoulder-roll -0.5deg", false,
            "left_shoulder_roll_joint", -kSupportSeedHalfDegree},
          {"secant shoulder-yaw +0.5deg", true,
            "left_shoulder_yaw_joint", kSupportSeedHalfDegree},
          {"secant shoulder-yaw -0.5deg", true,
            "left_shoulder_yaw_joint", -kSupportSeedHalfDegree},
          {"secant elbow +0.5deg", true,
            "left_elbow_joint", kSupportSeedHalfDegree},
          {"secant elbow -0.5deg", true,
            "left_elbow_joint", -kSupportSeedHalfDegree},
          {"secant wrist-pitch +0.5deg", true,
            "left_wrist_pitch_joint", kSupportSeedHalfDegree},
          {"secant wrist-pitch -0.5deg", true,
            "left_wrist_pitch_joint", -kSupportSeedHalfDegree},
        }};
        std::optional<std::map<std::string, double>> solution;
        size_t accepted_trial = 0U;
        for (size_t trial_index = 0U;
          !solution.has_value() && trial_index < seed_trials.size(); ++trial_index)
        {
          const auto & trial = seed_trials[trial_index];
          if (trial.use_predictor && !previous_positions.has_value()) {
            continue;
          }
          std::map<std::string, double> trial_seed =
            trial.use_predictor ? predictor : seed;
          if (trial.perturbed_joint[0] != '\0') {
            trial_seed.at(trial.perturbed_joint) += trial.offset;
          }
          bool seed_in_bounds = true;
          for (const auto & [joint, value] : trial_seed) {
            const auto & bounds = current.getRobotModel()->getVariableBounds(joint);
            if (!std::isfinite(value) ||
              (bounds.position_bounded_ &&
              (value < bounds.min_position_ || value > bounds.max_position_)))
            {
              seed_in_bounds = false;
              break;
            }
          }
          if (!seed_in_bounds) {
            continue;
          }
          solution = compute_seeded_ik_joint_target(
            node, left_arm, ik_client, tf_buffer, sample_pose, "left_dex1_tcp",
            trial_seed, "final 15 mm vertical rack support entry sample " +
            std::to_string(sample_index) + "/" +
            std::to_string(kSupportEntrySamples) + " seed " + trial.label,
            continuity_gate, false);
          if (solution.has_value()) {
            accepted_trial = trial_index;
          }
        }
        if (!solution.has_value()) {
          preflight_ok = false;
          break;
        }
        if (accepted_trial > 1U) {
          RCLCPP_INFO(
            node->get_logger(),
            "LEFT_RACK_SUPPORT_ENTRY_SEED_RECOVERY: sample %zu/%zu accepted '%s'.",
            sample_index, kSupportEntrySamples, seed_trials[accepted_trial].label);
        }
        previous_positions = seed;
        for (const auto & joint : joint_names) {
          current.setVariablePosition(joint, solution->at(joint));
        }
        current.setVariablePosition(
          "left_dex1_finger_1_joint", modeled_contact_position);
        current.setVariablePosition(
          "left_dex1_finger_2_joint", modeled_contact_position);
        current.update();
        support_path.points.push_back(support_state_point(current, joint_names));
      }
      if (preflight_ok && support_path.points.size() == kSupportEntrySamples + 1U) {
        for (size_t index = 0U; index < support_path.points.size(); ++index) {
          support_path.points[index].time_from_start =
            static_cast<builtin_interfaces::msg::Duration>(
            rclcpp::Duration::from_seconds(0.10 * static_cast<double>(index)));
        }
        support_state = current;
        support_boundaries = {0U, support_path.points.size() - 1U};
        RCLCPP_INFO(
          node->get_logger(),
          "LEFT_RACK_SUPPORT_ENTRY_SEEDED=PASS: 27 complete 0.5 mm TCP samples "
          "to a level 1.5 mm release clearance; no partial path accepted and "
          "no controller command was sent.");
      } else {
        preflight_ok = false;
      }
    }
    const auto support_forward = preflight_ok ? audit_left_rack_path(
      support_trajectory.joint_trajectory, support_entry_state, carry_scene,
      modeled_contact_position, "support entry forward", 0.0, true, true, false) :
      LeftRackPathAudit{};
    const auto support_reverse_path = reverse_left_joint_path(
      support_trajectory.joint_trajectory);
    const auto support_reverse = support_forward.passed ? audit_left_rack_path(
      support_reverse_path, support_state, carry_scene,
      modeled_contact_position, "support entry exact reverse", 0.0, true, true, false) :
      LeftRackPathAudit{};
    preflight_ok = preflight_ok && support_forward.passed && support_reverse.passed;

    if (!remove_left_attached_box(planning_scene)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to restore the preflight cube from left attachment.");
      return finish_left_rack(1);
    }
    if (preflight_ok && !apply_world_cube(*target_object_planning)) {
      preflight_ok = false;
    }
    auto retreat_scene = preflight_ok ? read_frozen_planning_scene(
      get_scene_client, left_arm.getRobotModel(), "left-rack released retreat",
      &*planning_cube_pose, object_size_x, object_size_y, object_size_z) : nullptr;
    moveit::core::RobotState released_start(support_state);
    if (retreat_scene) {
      released_start = retreat_scene->getCurrentState();
      released_start.setVariablePositions(support_state.getVariablePositions());
      released_start.setVariablePosition("left_dex1_finger_1_joint", -0.019);
      released_start.setVariablePosition("left_dex1_finger_2_joint", -0.019);
      released_start.update();
    } else {
      preflight_ok = false;
    }
    geometry_msgs::msg::Pose empty_inside_clear = target_tcp;
    empty_inside_clear.position.z += 0.060;
    const auto empty_outside = tcp_for_object_center_world(
      0.410, kRackCenterY, kRackCubeCenterZ + 0.060);
    moveit_msgs::msg::RobotTrajectory retreat_trajectory;
    moveit::core::RobotState retreat_endpoint(released_start);
    std::vector<size_t> retreat_boundaries;
    if (preflight_ok && left_rack_cached_asset.has_value()) {
      retreat_trajectory.joint_trajectory = left_rack_cached_asset->retreat;
      retreat_boundaries = left_rack_cached_asset->retreat_boundaries;
      preflight_ok = set_left_rack_endpoint_state(
        retreat_trajectory.joint_trajectory, released_start, retreat_endpoint);
    } else if (preflight_ok && selected_corridor_uses_fixed_tcp_bridge) {
      const auto reverse_boundaries = [](
        const std::vector<size_t> & forward, const size_t point_count)
        {
          std::vector<size_t> reversed;
          if (forward.size() < 2U || point_count < 2U ||
            forward.front() != 0U || forward.back() != point_count - 1U)
          {
            return reversed;
          }
          reversed.push_back(0U);
          for (size_t index = forward.size() - 1U; index > 0U; --index) {
            reversed.push_back(point_count - 1U - forward[index - 1U]);
          }
          return reversed;
        };
      const auto append_reversed_stage = [&] (
        const trajectory_msgs::msg::JointTrajectory & forward,
        const std::vector<size_t> & forward_boundaries)
        {
          const auto reversed = reverse_left_joint_path(forward);
          const auto boundaries = reverse_boundaries(
            forward_boundaries, forward.points.size());
          auto & destination = retreat_trajectory.joint_trajectory;
          if (boundaries.empty() || reversed.points.empty()) {
            return false;
          }
          if (destination.joint_names.empty()) {
            destination.joint_names = reversed.joint_names;
            destination.header = reversed.header;
          } else if (destination.joint_names != reversed.joint_names) {
            return false;
          }
          const bool skip_first = !destination.points.empty();
          const size_t base = skip_first ? destination.points.size() - 1U : 0U;
          const size_t begin = skip_first ? 1U : 0U;
          if (reversed.points.size() <= begin) {
            return false;
          }
          destination.points.insert(
            destination.points.end(), reversed.points.begin() + begin,
            reversed.points.end());
          if (retreat_boundaries.empty()) {
            retreat_boundaries.push_back(0U);
          }
          for (size_t index = 1U; index < boundaries.size(); ++index) {
            retreat_boundaries.push_back(base + boundaries[index]);
          }
          return true;
        };
      preflight_ok = append_reversed_stage(
        support_trajectory.joint_trajectory, support_boundaries) &&
        append_reversed_stage(carry_trajectory.joint_trajectory, carry_boundaries) &&
        append_reversed_stage(approach_trajectory.joint_trajectory, approach_boundaries);
      if (preflight_ok) {
        preflight_ok = set_left_rack_endpoint_state(
          retreat_trajectory.joint_trajectory, released_start, retreat_endpoint);
      }
      if (preflight_ok) {
        constexpr double kDirectKnotPeriod = 0.10;
        auto & points = retreat_trajectory.joint_trajectory.points;
        for (size_t index = 0U; index < points.size(); ++index) {
          points[index].time_from_start =
            static_cast<builtin_interfaces::msg::Duration>(
            rclcpp::Duration::from_seconds(
              kDirectKnotPeriod * static_cast<double>(index)));
          points[index].velocities.clear();
          points[index].accelerations.clear();
          points[index].effort.clear();
        }
        RCLCPP_INFO(
          node->get_logger(),
          "LEFT_RACK_RELEASED_RETREAT_FIXED_BRIDGE_INVERSE=PASS: exact support, "
          "carry/redundancy-bridge and approach position paths were reversed; "
          "partial reconstruction is forbidden.");
      }
    } else if (preflight_ok) {
      std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
      retreat_waypoints.reserve(selected_corridor_waypoints.size() + 3U);
      for (auto pose = selected_corridor_waypoints.rbegin();
        pose != selected_corridor_waypoints.rend(); ++pose)
      {
        retreat_waypoints.push_back(*pose);
      }
      retreat_waypoints.push_back(contact_tcp);
      retreat_waypoints.push_back(pregrasp_tcp);
      retreat_waypoints.push_back(*current_tcp);
      preflight_ok = !selected_corridor_waypoints.empty() &&
        compute_complete_left_cartesian_segments(
          left_arm, released_start, retreat_waypoints,
          "open left hand exact-reverse exits selected corridor and returns",
          retreat_trajectory, retreat_endpoint, false, true,
          &retreat_boundaries);
      if (preflight_ok) {
        // The Cartesian return reaches the live neutral TCP but may choose a
        // different redundant 7-DOF posture.  Fold the small joint-space
        // neutralization into the audited retreat so the closed-loop wrapper
        // no longer needs a separate bridge action or its inter-stage wait.
        constexpr double kNeutralBridgeMaximumStep = 2.0 * M_PI / 180.0;
        auto & retreat_path = retreat_trajectory.joint_trajectory;
        const auto start_positions = retreat_path.points.back().positions;
        if (retreat_path.joint_names != left_rack_joint_names ||
          start_positions.size() != left_rack_live_start.size())
        {
          preflight_ok = false;
        } else {
          double maximum_difference = 0.0;
          for (size_t joint_index = 0U;
            joint_index < start_positions.size(); ++joint_index)
          {
            maximum_difference = std::max(
              maximum_difference,
              std::abs(
                left_rack_live_start[joint_index] - start_positions[joint_index]));
          }
          const size_t bridge_steps = std::max<size_t>(
            1U, static_cast<size_t>(std::ceil(
              maximum_difference / kNeutralBridgeMaximumStep)));
          for (size_t step = 1U; step <= bridge_steps; ++step) {
            const double alpha = static_cast<double>(step) /
              static_cast<double>(bridge_steps);
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions.reserve(start_positions.size());
            for (size_t joint_index = 0U;
              joint_index < start_positions.size(); ++joint_index)
            {
              point.positions.push_back(
                start_positions[joint_index] + alpha *
                (left_rack_live_start[joint_index] - start_positions[joint_index]));
            }
            retreat_path.points.push_back(std::move(point));
          }
          retreat_boundaries.push_back(retreat_path.points.size() - 1U);
          for (size_t joint_index = 0U;
            joint_index < left_rack_joint_names.size(); ++joint_index)
          {
            retreat_endpoint.setVariablePosition(
              left_rack_joint_names[joint_index], left_rack_live_start[joint_index]);
          }
          retreat_endpoint.setVariablePosition("left_dex1_finger_1_joint", -0.019);
          retreat_endpoint.setVariablePosition("left_dex1_finger_2_joint", -0.019);
          retreat_endpoint.update();
          preflight_ok = retreat_endpoint.satisfiesBounds(1e-6);
          RCLCPP_INFO(
            node->get_logger(),
            "LEFT_RACK_RELEASED_RETREAT_NEUTRAL_BRIDGE: %zu complete joint-space "
            "samples appended with %.3f deg maximum source difference; the "
            "combined path remains subject to full forward/reverse audits.",
            bridge_steps, maximum_difference * kDegrees);
        }
      }
    }
    const auto retreat_forward = preflight_ok ? audit_left_rack_path(
      retreat_trajectory.joint_trajectory, released_start, retreat_scene, -0.019,
      "released retreat forward", 0.010, false, true, false,
      kRackCarrySelfClearance) : LeftRackPathAudit{};
    const auto retreat_reverse_path = reverse_left_joint_path(
      retreat_trajectory.joint_trajectory);
    const auto retreat_reverse = retreat_forward.passed ? audit_left_rack_path(
      retreat_reverse_path, retreat_endpoint, retreat_scene, -0.019,
      "released retreat exact reverse", 0.010, false, true, false,
      kRackCarrySelfClearance) : LeftRackPathAudit{};
    preflight_ok = preflight_ok && retreat_forward.passed && retreat_reverse.passed;

    double left_rack_arm_seconds = 0.0;
    if (preflight_ok) {
      const std::array<std::pair<const char *, moveit_msgs::msg::RobotTrajectory *>, 4U>
      arm_stages{{
          {"approach", &approach_trajectory},
          {"carry", &carry_trajectory},
          {"support", &support_trajectory},
          {"retreat", &retreat_trajectory},
        }};
      for (const auto & [stage, trajectory] : arm_stages) {
        if (left_rack_fast_timing) {
          const auto duration = retime_left_rack_path_locally(
            trajectory->joint_trajectory, *synchronized_motion_limits,
            left_rack_fast_minimum_knot_period, stage);
          if (!duration.has_value()) {
            preflight_ok = false;
            break;
          }
          left_rack_arm_seconds += *duration;
        } else {
          assign_left_rack_uniform_timing(trajectory->joint_trajectory, 0.10);
          left_rack_arm_seconds += 0.10 * static_cast<double>(
            trajectory->joint_trajectory.points.size() - 1U);
        }
      }
      if (preflight_ok && left_rack_fast_timing &&
        left_rack_arm_seconds > left_rack_fast_maximum_arm_seconds + 1e-9)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "LEFT_RACK_FAST_TIMING_TOTAL=FAIL: %.3f s exceeds the %.3f s hard "
          "cube-arm ceiling; no controller command was sent.",
          left_rack_arm_seconds, left_rack_fast_maximum_arm_seconds);
        preflight_ok = false;
      } else if (preflight_ok) {
        RCLCPP_INFO(
          node->get_logger(),
          "LEFT_RACK_ARM_TIMING_AUDIT=PASS: four position-identical arm stages "
          "total %.3f simulation seconds%s.", left_rack_arm_seconds,
          left_rack_fast_timing ? " (fast hard ceiling <=35.000 s)" : "");
      }
    }

    if (!clear_mode_cube() || !apply_world_cube(*planning_cube_pose)) {
      RCLCPP_ERROR(
        node->get_logger(),
        "CRITICAL: could not restore the preflight cube guard to its live table pose.");
      return finish_left_rack(1);
    }
    if (!right_support_unchanged()) {
      return finish_left_rack(1);
    }
    if (!preflight_ok) {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_CUBE_TO_INCUBATOR_RACK_PREFLIGHT=FAIL; complete forward/reverse "
        "collision, clearance, exact-box/Dex1 contact or IK gate failed; no trajectory executed.");
      return finish_left_rack(1);
    }
    RCLCPP_INFO(
      node->get_logger(),
      "LEFT_CUBE_TO_INCUBATOR_RACK_PREFLIGHT=PASS target world "
      "[%.12f, %.12f, %.12f], original tray surface %.12f; all paths complete, "
      "forward/reverse audited, partial forbidden.",
      kRackCenterX, kRackCenterY, kRackCubeCenterZ, kRackSurfaceZ);
    if (left_rack_cache_export_only) {
      if (!std::isfinite(selected_corridor_height) ||
        std::abs(selected_corridor_height - 1.020) > 1e-12 ||
        selected_corridor_profile != 1U)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "LEFT_RACK_CACHE_EXPORT_ONLY=FAIL: the fixed cache contract requires "
          "front corridor z=1.020 m/profile=1; refusing to mislabel another "
          "valid route. No controller command was sent.");
        return finish_left_rack(1);
      }
      LeftRackPositionAsset asset;
      asset.joint_names = left_rack_joint_names;
      asset.fresh_start = left_rack_live_start;
      asset.approach = approach_trajectory.joint_trajectory;
      asset.carry = carry_trajectory.joint_trajectory;
      asset.support = support_trajectory.joint_trajectory;
      asset.retreat = retreat_trajectory.joint_trajectory;
      asset.approach_boundaries = approach_boundaries;
      asset.carry_boundaries = carry_boundaries;
      asset.support_boundaries = support_boundaries;
      asset.retreat_boundaries = retreat_boundaries;
      std::string export_error;
      if (!write_new_left_rack_position_asset(
          left_rack_cache_path, asset, export_error))
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "LEFT_RACK_CACHE_EXPORT_ONLY=FAIL: %s; existing or partial files are "
          "never overwritten and no controller command was sent.",
          export_error.c_str());
        return finish_left_rack(1);
      }
      RCLCPP_INFO(
        node->get_logger(),
        "LEFT_RACK_CACHE_EXPORT_ONLY=PASS: sealed position asset '%s' with "
        "approach/carry/support/retreat points %zu/%zu/%zu/%zu and all segment "
        "boundaries; no controller command, gripper command or Gazebo support "
        "operation was sent.", left_rack_cache_path.c_str(),
        asset.approach.points.size(), asset.carry.points.size(),
        asset.support.points.size(), asset.retreat.points.size());
      return finish_left_rack(0);
    }
    if (!left_rack_execute) {
      RCLCPP_INFO(
        node->get_logger(),
        "LEFT_CUBE_TO_INCUBATOR_RACK_NO_EXECUTION=PASS: left_rack_execute is false; "
        "no controller trajectory, gripper command, Gazebo attach/detach, reset or door action.");
      return finish_left_rack(0);
    }

    const auto endpoint_target = [](const trajectory_msgs::msg::JointTrajectory & path) {
        std::map<std::string, double> target;
        if (path.points.empty() ||
          path.joint_names.size() != path.points.back().positions.size())
        {
          return target;
        }
        for (size_t index = 0; index < path.joint_names.size(); ++index) {
          target.emplace(path.joint_names[index], path.points.back().positions[index]);
        }
        return target;
    };
    bool physical_ok = right_support_unchanged();
    if (physical_ok) {
      const auto approach_target = endpoint_target(
        approach_trajectory.joint_trajectory);
      const bool already_at_contact = !approach_target.empty() &&
        joint_target_is_reached(
          left_arm, approach_target, 0.005,
          "left-rack already-at-contact fast resume");
      if (already_at_contact) {
        RCLCPP_INFO(
          node->get_logger(),
          "LEFT_RACK_CONTACT_RESUME=PASS: left arm is already at the certified "
          "cube-contact endpoint; skipping duplicate approach execution.");
      } else {
        physical_ok = left_arm.execute(approach_trajectory) ==
          moveit::core::MoveItErrorCode::SUCCESS &&
          joint_target_is_reached(
          left_arm, approach_target, 0.005,
          "left-rack physical cube contact");
      }
    }
    const auto live_before_close = gazebo_pose_monitor.latest();
    if (physical_ok) {
      physical_ok = live_before_close.has_value() &&
        verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, cube_sample->pose, 0.003, 2.0 / kDegrees,
        "left-rack pre-close cube stationary");
    }
    if (physical_ok) {
      const bool close_execution = execute_prevalidated_joint_target(
        left_gripper, left_preload, 1.0, "left-rack close onto 80 mm cube");
      const bool close_feedback = verify_gripper_contact_feedback(
        left_gripper, "left", preload, object_size_x, true,
        "left-rack physical dual-pad cube contact", 0.003);
      physical_ok = close_feedback;
      left_closed = close_feedback;
      if (!close_execution && close_feedback) {
        RCLCPP_WARN(
          node->get_logger(),
          "LEFT_RACK_EXPECTED_CONTACT_ABORT=PASS: controller stopped on the cube, "
          "and dual-finger feedback certified the physical contact.");
      }
    }
    if (physical_ok) {
      physical_ok = gazebo_grasp_constraint.attach_left_to_cube();
    }
    if (physical_ok) {
      physical_ok = planning_scene.applyAttachedCollisionObject(
        make_left_attached_box(
          object_in_tcp, object_size_x, object_size_y, object_size_z)) &&
        wait_for_attached_object_with_touch_links(
        planning_scene, kObjectId, "left_dex1_tcp", hand_touch_links("left"));
    }
    if (physical_ok) {
      physical_ok = right_support_unchanged() &&
        left_arm.execute(carry_trajectory) == moveit::core::MoveItErrorCode::SUCCESS &&
        joint_target_is_reached(
        left_arm, endpoint_target(carry_trajectory.joint_trajectory), 0.005,
        "left-rack physical support-entry pose");
    }
    std::optional<moveit_msgs::msg::AllowedCollisionMatrix> original_acm;
    if (physical_ok) {
      original_acm = read_allowed_collision_matrix(get_scene_client);
      auto supported_acm = original_acm.value_or(moveit_msgs::msg::AllowedCollisionMatrix{});
      physical_ok = original_acm.has_value() &&
        set_allowed_collision_pair(supported_acm, kObjectId, kIncubatorBodyId, true) &&
        apply_allowed_collision_matrix(supported_acm, apply_scene_client);
    }
    if (physical_ok) {
      physical_ok = left_arm.execute(support_trajectory) ==
        moveit::core::MoveItErrorCode::SUCCESS &&
        joint_target_is_reached(
        left_arm, endpoint_target(support_trajectory.joint_trajectory), 0.012,
        "left-rack physical tray center");
    }
    if (original_acm.has_value() &&
      !apply_allowed_collision_matrix(*original_acm, apply_scene_client))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "CRITICAL: failed to restore ACM after supported rack entry; retaining left hold.");
      physical_ok = false;
    }
    std::optional<ObjectStabilityReport> supported_report;
    if (physical_ok) {
      supported_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, kRackSurfaceZ,
        object_size_x, object_size_y, object_size_z,
        0.20, 0.40, -0.002, 0.003, 0.002, 2.0 / kDegrees,
        2.0 / kDegrees, "left-rack supported cube at exact tray center");
      if (supported_report.has_value()) {
        const Eigen::Vector3d actual(
          supported_report->final_sample.pose.position.x,
          supported_report->final_sample.pose.position.y,
          supported_report->final_sample.pose.position.z);
        physical_ok = (actual - Eigen::Vector3d(
          kRackCenterX, kRackCenterY, kRackCubeCenterZ)).norm() <= 0.003;
      } else {
        physical_ok = false;
      }
    }
    if (!physical_ok) {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_CUBE_TO_INCUBATOR_RACK_PHYSICAL=STOP_AND_HOLD: left fingers and "
        "left Gazebo support remain engaged; no detach, opening, reset or right command.");
      return finish_left_rack(1);
    }

    // Release authority exists only after the original tray contact and center
    // stability gates above.  From this point the rack, rather than the hand,
    // is the verified support.
    physical_ok = gazebo_grasp_constraint.detach_left_from_cube(true);
    if (!physical_ok) {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_RACK_RELEASE=FAIL: Gazebo support did not detach; left fingers remain closed.");
      return finish_left_rack(1);
    }
    physical_ok = remove_left_attached_box(planning_scene) &&
      apply_world_cube(*target_object_planning);
    if (!physical_ok) {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_RACK_SCENE_RELEASE=FAIL after tray support; refusing to move the hand.");
      return finish_left_rack(1);
    }
    physical_ok = execute_prevalidated_joint_target(
      left_gripper, left_open, 0.8, "left-rack release cube on original tray") &&
      gripper_open_target_is_reached(
      left_gripper, left_open, 0.0005, "left-rack full-open feedback");
    if (physical_ok) {
      physical_ok = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, kRackSurfaceZ,
        object_size_x, object_size_y, object_size_z,
        0.20, 0.60, -0.002, 0.003, 0.002, 2.0 / kDegrees,
        2.0 / kDegrees, "left-rack released cube stability").has_value();
    }
    if (physical_ok) {
      physical_ok = left_arm.execute(retreat_trajectory) ==
        moveit::core::MoveItErrorCode::SUCCESS &&
        joint_target_is_reached(
        left_arm, endpoint_target(retreat_trajectory.joint_trajectory), 0.005,
        "left-rack empty-hand retreat");
    }
    physical_ok = right_support_unchanged() && physical_ok;
    if (!physical_ok) {
      RCLCPP_ERROR(
        node->get_logger(),
        "LEFT_CUBE_TO_INCUBATOR_RACK_PHYSICAL=FAIL after verified tray release; "
        "the cube remains supported on the original rack and no right command was issued.");
      return finish_left_rack(1);
    }
    RCLCPP_INFO(
      node->get_logger(),
      "LEFT_CUBE_TO_INCUBATOR_RACK_PHYSICAL=PASS: left hand placed green_cube "
      "at original middle-tray center [%.12f, %.12f, %.12f] world; right arm, "
      "right gripper, door and handle remained unchanged.",
      kRackCenterX, kRackCenterY, kRackCubeCenterZ);
    return finish_left_rack(0);
  }

  // A previous interrupted run can leave the planning-scene object behind
  // even though the wrapper recreated the Gazebo body.  Resolve the exact
  // owner and wait for every asynchronous scene transition before proceeding.
  const auto stale_attached = planning_scene.getAttachedObjects({kObjectId});
  if (!stale_attached.empty()) {
    const auto & link_name = stale_attached.begin()->second.link_name;
    RCLCPP_WARN(
      node->get_logger(), "Removing stale attached object from link '%s'.", link_name.c_str());
    if (link_name.find("right_") == 0) {
      success = right_arm.detachObject(kObjectId);
    } else if (link_name.find("left_") == 0) {
      success = left_arm.detachObject(kObjectId);
    } else if (link_name == "pelvis") {
      success = both_arms.detachObject(kObjectId);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Unknown stale attachment owner '%s'.", link_name.c_str());
      success = false;
    }
    if (success) {
      success = wait_for_world_object(planning_scene, kObjectId);
    }
  }
  if (success && !planning_scene.getObjects({kObjectId}).empty()) {
    planning_scene.removeCollisionObjects({kObjectId});
    success = wait_for_object_absent(planning_scene, kObjectId);
  }

  if (success && sequential_center_regrasp_test_mode && center_regrasp_preflight_only) {
    // Build the complete right-holder/left-receiver geometry in a synthetic
    // airborne state.  The object is attached to the fixed pelvis frame so it
    // stays at the measured world pose while the two IK branches are sampled;
    // no controller trajectory is sent from this branch.
    std::optional<GazeboPoseSample> gazebo_sample;
    const auto pose_deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < pose_deadline && !gazebo_sample.has_value()) {
      gazebo_sample = gazebo_pose_monitor.latest(0.50);
      if (!gazebo_sample.has_value()) {
        std::this_thread::sleep_for(50ms);
      }
    }
    const auto table_planning_object = gazebo_sample.has_value() ?
      transform_pose_to_planning_frame(gazebo_sample->pose, tf_buffer) :
      std::optional<geometry_msgs::msg::Pose>{};
    success = gazebo_sample.has_value() && table_planning_object.has_value();
    if (!success) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Sequential center preflight has no fresh Gazebo pose in the planning frame.");
    }

    geometry_msgs::msg::Pose airborne_object;
    double table_surface_z = recovery_table_surface_z;
    if (success) {
      airborne_object = *table_planning_object;
      airborne_object.position.x = handover_x;
      airborne_object.position.y = handover_y;
      airborne_object.position.z += pickup_test_lift + serial_carry_z_compensation;
      table_surface_z = object_lowest_corner_z(
        gazebo_sample->pose, object_size_x, object_size_y, object_size_z);
    }
    const auto right_holder_pose = make_inward_tcp_pose(
      airborne_object, right_grasp_y_offset, grasp_z_offset,
      cooperative_inward_tilt_degrees * 0.017453292519943295);
    const auto left_staging_pose = make_inward_tcp_pose(
      airborne_object, serial_receiver_prehandover_y_offset,
      transfer_receiver_z_offset,
      center_regrasp_search_tilt_degrees * 0.017453292519943295);
    const auto left_contact_pose = make_inward_tcp_pose(
      airborne_object, left_grasp_y_offset, transfer_receiver_z_offset,
      center_regrasp_search_tilt_degrees * 0.017453292519943295);

    std::optional<std::map<std::string, double>> right_holder_target;
    std::optional<std::map<std::string, double>> left_staging_target;
    if (success) {
      right_holder_target = compute_nearest_seeded_ik_joint_target(
        node, right_arm, ik_client, tf_buffer, right_holder_pose, "right_dex1_tcp",
        right_park,
        "long-block preflight: right holder at Y=" +
        std::to_string(right_grasp_y_offset * 1000.0) + " mm", 32);
      success = right_holder_target.has_value();
    }
    if (success) {
      left_staging_target = compute_nearest_seeded_ik_joint_target(
        node, left_arm, ik_client, tf_buffer, left_staging_pose, "left_dex1_tcp",
        left_park, "long-block preflight: left staging at Y=+115 mm", 32);
      success = left_staging_target.has_value();
    }

    auto combined_state = both_arms.getCurrentState(10.0);
    success = success && combined_state != nullptr;
    if (success) {
      for (const auto & [joint_name, position] : *right_holder_target) {
        combined_state->setVariablePosition(joint_name, position);
      }
      for (const auto & [joint_name, position] : *left_staging_target) {
        combined_state->setVariablePosition(joint_name, position);
      }
      combined_state->setVariablePosition("right_dex1_finger_1_joint", preload);
      combined_state->setVariablePosition("right_dex1_finger_2_joint", preload);
      combined_state->setVariablePosition("left_dex1_finger_1_joint", -0.019);
      combined_state->setVariablePosition("left_dex1_finger_2_joint", -0.019);
      combined_state->update();
      success = combined_state->satisfiesBounds(1e-6);
    }

    bool airborne_attachment_commanded = false;
    if (success) {
      success = planning_scene.applyCollisionObject(make_collision_object(
          airborne_object, object_size_x, object_size_y, object_size_z)) &&
        wait_for_world_object(planning_scene, kObjectId);
    }
    if (success) {
      const auto touch_links = both_hand_touch_links();
      airborne_attachment_commanded = both_arms.attachObject(
        kObjectId, "pelvis", touch_links);
      success = airborne_attachment_commanded &&
        wait_for_attached_object_with_touch_links(
        planning_scene, kObjectId, "pelvis", touch_links);
      owner = success ? Owner::Base : Owner::World;
    }
    if (success) {
      success = full_robot_state_is_valid(
        validity_client, *combined_state,
        "long-block preflight combined right-holder/left-staging start");
    }

    std::optional<CenterPathValidation> path_validation;
    if (success) {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(make_inward_tcp_pose(
          airborne_object, serial_receiver_prehandover_y_offset,
          transfer_receiver_z_offset + center_regrasp_overpass_height,
          center_regrasp_search_tilt_degrees * 0.017453292519943295));
      waypoints.push_back(make_inward_tcp_pose(
          airborne_object, left_grasp_y_offset,
          transfer_receiver_z_offset + center_regrasp_overpass_height,
          center_regrasp_search_tilt_degrees * 0.017453292519943295));
      waypoints.push_back(left_contact_pose);
      left_arm.setStartState(*combined_state);
      moveit_msgs::msg::RobotTrajectory trajectory;
      moveit_msgs::msg::MoveItErrorCodes error_code;
      const double fraction = left_arm.computeCartesianPath(
        waypoints, kCartesianEefStep, trajectory, true, &error_code);
      left_arm.setStartStateToCurrentState();
      RCLCPP_INFO(
        node->get_logger(),
        "Long-block sequential center preflight Cartesian path: %.1f%%, %zu points "
        "(error=%d).",
        fraction * 100.0, trajectory.joint_trajectory.points.size(), error_code.val);
      if (fraction >= kMinCartesianFraction &&
        trajectory.joint_trajectory.points.size() >= 2)
      {
        ::planning_scene::PlanningScene geometric_scene(both_arms.getRobotModel());
        path_validation = validate_center_regrasp_path(
          trajectory.joint_trajectory, *combined_state, geometric_scene,
          validity_client, table_surface_z,
          "long-block sequential center regrasp offline path");
      }
      success = path_validation.has_value();

      if (success) {
        auto closing_state = *combined_state;
        for (const auto & [joint_name, position] : path_validation->endpoint) {
          closing_state.setVariablePosition(joint_name, position);
        }
        ::planning_scene::PlanningScene geometric_scene(both_arms.getRobotModel());
        for (int finger_mm = -19; finger_mm <= 3 && success; ++finger_mm) {
          const double finger_position = static_cast<double>(finger_mm) / 1000.0;
          closing_state.setVariablePosition("left_dex1_finger_1_joint", finger_position);
          closing_state.setVariablePosition("left_dex1_finger_2_joint", finger_position);
          closing_state.update();
          const auto clearance = center_regrasp_clearance(
            geometric_scene, closing_state, table_surface_z);
          success = closing_state.satisfiesBounds(1e-6) &&
            clearance.has_value() && clearance->hand_to_hand >= 0.005 &&
            clearance->hand_to_table >= 0.010 &&
            full_robot_state_is_valid(
            validity_client, closing_state,
            "long-block preflight left closing " + std::to_string(finger_mm) + " mm");
        }

        if (success && donor_release_test_mode) {
          closing_state.setVariablePosition("left_dex1_finger_1_joint", transfer_receiver_preload);
          closing_state.setVariablePosition("left_dex1_finger_2_joint", transfer_receiver_preload);
          closing_state.setVariablePosition("right_dex1_finger_1_joint", -0.002);
          closing_state.setVariablePosition("right_dex1_finger_2_joint", -0.002);
          closing_state.update();
          const Eigen::Matrix3d object_rotation = pose_to_eigen(airborne_object).linear();
          const double donor_tilt =
            cooperative_inward_tilt_degrees * 0.017453292519943295;
          auto right_retreat_15 = right_holder_pose;
          const Eigen::Vector3d first_retreat = object_rotation * Eigen::Vector3d(
            0.0, -std::cos(donor_tilt) * 0.015,
            std::sin(donor_tilt) * 0.015);
          right_retreat_15.position.x += first_retreat.x();
          right_retreat_15.position.y += first_retreat.y();
          right_retreat_15.position.z += first_retreat.z();
          ::planning_scene::PlanningScene geometric_scene(both_arms.getRobotModel());
          std::optional<CenterPathValidation> donor_retreat_validation;
          const std::vector<std::pair<double, std::string>> second_segment_candidates{
            {donor_tilt, "continue along 60 deg grasp axis"},
            {0.0, "switch second segment to object-local horizontal"},
          };
          for (const auto & [second_tilt, candidate_name] : second_segment_candidates) {
            auto right_retreat_50 = right_retreat_15;
            const Eigen::Vector3d second_retreat = object_rotation * Eigen::Vector3d(
              0.0, -std::cos(second_tilt) * 0.035,
              std::sin(second_tilt) * 0.035);
            right_retreat_50.position.x += second_retreat.x();
            right_retreat_50.position.y += second_retreat.y();
            right_retreat_50.position.z += second_retreat.z();

            right_arm.setStartState(closing_state);
            moveit_msgs::msg::RobotTrajectory donor_retreat_trajectory;
            moveit_msgs::msg::MoveItErrorCodes donor_retreat_error;
            const double donor_retreat_fraction = right_arm.computeCartesianPath(
              {right_retreat_15, right_retreat_50}, kCartesianEefStep,
              donor_retreat_trajectory, true, &donor_retreat_error);
            right_arm.setStartStateToCurrentState();
            RCLCPP_INFO(
              node->get_logger(),
              "Long-block donor-release preflight retreat candidate '%s': %.1f%%, %zu "
              "points (error=%d).",
              candidate_name.c_str(), donor_retreat_fraction * 100.0,
              donor_retreat_trajectory.joint_trajectory.points.size(),
              donor_retreat_error.val);
            if (donor_retreat_fraction >= kMinCartesianFraction) {
              donor_retreat_validation = validate_center_regrasp_path(
                donor_retreat_trajectory.joint_trajectory, closing_state,
                geometric_scene, validity_client, table_surface_z,
                "long-block donor-release offline 50 mm retreat: " + candidate_name);
            }
            if (donor_retreat_validation.has_value()) {
              RCLCPP_INFO(
                node->get_logger(),
                "Selected donor-release second segment: %s.", candidate_name.c_str());
              break;
            }
          }
          success = donor_retreat_validation.has_value();
          if (success) {
            for (const auto & [joint_name, position] : donor_retreat_validation->endpoint) {
              closing_state.setVariablePosition(joint_name, position);
            }
            closing_state.update();
          }

          auto left_table_pose = left_contact_pose;
          left_table_pose.position.z -= pickup_test_lift;
          moveit_msgs::msg::RobotTrajectory left_place_trajectory;
          moveit_msgs::msg::MoveItErrorCodes left_place_error;
          double left_place_fraction = 0.0;
          if (success) {
            left_arm.setStartState(closing_state);
            left_place_fraction = left_arm.computeCartesianPath(
              {left_table_pose}, kCartesianEefStep,
              left_place_trajectory, true, &left_place_error);
            left_arm.setStartStateToCurrentState();
            RCLCPP_INFO(
              node->get_logger(),
              "Long-block donor-release preflight left vertical place: %.1f%%, %zu points "
              "(error=%d).",
              left_place_fraction * 100.0,
              left_place_trajectory.joint_trajectory.points.size(),
              left_place_error.val);
            const auto left_place_validation =
              left_place_fraction >= kMinCartesianFraction ?
              validate_center_regrasp_path(
                left_place_trajectory.joint_trajectory, closing_state,
                geometric_scene, validity_client, table_surface_z,
                "long-block donor-release offline left vertical place") :
              std::optional<CenterPathValidation>{};
            success = left_place_validation.has_value();
          }
        }
      }
    }

    bool scene_cleaned = true;
    if (airborne_attachment_commanded) {
      scene_cleaned = both_arms.detachObject(kObjectId) &&
        wait_for_world_object(planning_scene, kObjectId);
      owner = scene_cleaned ? Owner::World : Owner::Base;
    }
    if (scene_cleaned && !planning_scene.getObjects({kObjectId}).empty()) {
      planning_scene.removeCollisionObjects({kObjectId});
      scene_cleaned = wait_for_object_absent(planning_scene, kObjectId);
      owner = scene_cleaned ? Owner::None : Owner::World;
    }
    success = success && scene_cleaned;
    if (success) {
      RCLCPP_INFO(
        node->get_logger(),
        "LONG_BLOCK_CENTER_PREFLIGHT_PASS: size %.0fx%.0fx%.0f mm, donor/receiver "
        "Y=%.1f/%.1f mm, staging Y=%.1f mm, lift %.1f mm, carry Z compensation "
        "%.1f mm; no trajectory executed.",
        object_size_x * 1000.0, object_size_y * 1000.0, object_size_z * 1000.0,
        right_grasp_y_offset * 1000.0, left_grasp_y_offset * 1000.0,
        serial_receiver_prehandover_y_offset * 1000.0, pickup_test_lift * 1000.0,
        serial_carry_z_compensation * 1000.0);
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "Long-block sequential center preflight failed closed; no trajectory executed.");
    }
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return success ? 0 : 1;
  }

  if (success && center_regrasp_abort_recovery_mode) {
    // Resume only from the failed center-contact state: the Gazebo fixed grasp
    // keeps the right wrist and block stationary while the empty left gripper
    // opens and reverses the exact 15 mm overpass path.  This recovery never
    // publishes either grasp-constraint command.
    right_arm.setMaxVelocityScalingFactor(transfer_motion_velocity_scaling);
    right_arm.setMaxAccelerationScalingFactor(transfer_motion_acceleration_scaling);
    left_arm.setMaxVelocityScalingFactor(transfer_motion_velocity_scaling);
    left_arm.setMaxAccelerationScalingFactor(transfer_motion_acceleration_scaling);

    const auto recovery_report = verify_gazebo_object_stability(
      node, gazebo_pose_monitor, recovery_table_surface_z,
      object_size_x, object_size_y, object_size_z,
      0.20, transfer_gate_observation,
      pickup_test_lift - transfer_gate_air_height_tolerance,
      pickup_test_lift + transfer_gate_air_height_tolerance,
      transfer_gate_max_drift,
      transfer_gate_max_angle_degrees * 0.017453292519943295,
      transfer_gate_max_tilt_degrees * 0.017453292519943295,
      "center-regrasp abort right-held baseline");
    success = recovery_report.has_value();

    std::optional<geometry_msgs::msg::Pose> planning_object_pose;
    if (success) {
      planning_object_pose = transform_pose_to_planning_frame(
        recovery_report->final_sample.pose, tf_buffer);
      success = planning_object_pose.has_value();
    }
    bool recovery_attachment_commanded = false;
    if (success) {
      success = planning_scene.applyCollisionObject(make_collision_object(
          *planning_object_pose, object_size_x, object_size_y, object_size_z)) &&
        wait_for_world_object(planning_scene, kObjectId);
    }
    if (success) {
      const auto touch_links = both_hand_touch_links();
      recovery_attachment_commanded = right_arm.attachObject(
        kObjectId, "right_dex1_tcp", touch_links);
      success = recovery_attachment_commanded &&
        wait_for_attached_object_with_touch_links(
        planning_scene, kObjectId, "right_dex1_tcp", touch_links);
      owner = success ? Owner::Right : Owner::World;
    }

    if (success) {
      success = plan_and_execute_joint(
        left_gripper, left_open,
        "center-regrasp abort: open non-supporting left gripper");
      left_closed = !success;
    }
    if (success) {
      success = wait_for_sim_duration(node, 0.20, "center-regrasp abort left-open settle");
    }
    if (success) {
      success = gripper_open_target_is_reached(
        left_gripper, left_open, 0.0005,
        "center-regrasp abort left-open feedback");
      left_closed = !success;
    }
    if (success) {
      success = verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, recovery_report->final_sample.pose,
        transfer_gate_max_anchor_displacement,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        "center-regrasp abort after left opens");
    }

    const auto reverse_center_path_gate = [&](
        const trajectory_msgs::msg::JointTrajectory & trajectory) -> bool {
        auto common_start = both_arms.getCurrentState(10.0);
        if (!common_start) {
          return false;
        }
        common_start->update();
        ::planning_scene::PlanningScene geometric_scene(both_arms.getRobotModel());
        return validate_center_regrasp_path(
          trajectory, *common_start, geometric_scene, validity_client,
          recovery_table_surface_z,
          "center-regrasp abort reverse path").has_value();
      };
    if (success) {
      const double receiver_tilt =
        center_regrasp_search_tilt_degrees * 0.017453292519943295;
      const auto center_overpass = make_inward_tcp_pose(
        *planning_object_pose, left_grasp_y_offset,
        transfer_receiver_z_offset + center_regrasp_overpass_height,
        receiver_tilt);
      const auto outer_overpass = make_inward_tcp_pose(
        *planning_object_pose, serial_receiver_prehandover_y_offset,
        transfer_receiver_z_offset + center_regrasp_overpass_height,
        receiver_tilt);
      const auto outer_staging = make_inward_tcp_pose(
        *planning_object_pose, serial_receiver_prehandover_y_offset,
        transfer_receiver_z_offset, receiver_tilt);
      success = plan_and_execute_cartesian(
        left_arm, outer_staging,
        "center-regrasp abort: left reverses 15 mm overpass to outer staging",
        false, reverse_center_path_gate, {center_overpass, outer_overpass});
    }
    if (success) {
      success = verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, recovery_report->final_sample.pose,
        transfer_gate_max_anchor_displacement,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        "center-regrasp abort after left retreat");
    }

    bool scene_cleaned = true;
    if (recovery_attachment_commanded) {
      scene_cleaned = right_arm.detachObject(kObjectId) &&
        wait_for_world_object(planning_scene, kObjectId);
      owner = scene_cleaned ? Owner::World : Owner::Right;
    }
    if (scene_cleaned && !planning_scene.getObjects({kObjectId}).empty()) {
      planning_scene.removeCollisionObjects({kObjectId});
      scene_cleaned = wait_for_object_absent(planning_scene, kObjectId);
      owner = scene_cleaned ? Owner::None : Owner::World;
    }
    success = success && scene_cleaned;

    if (success) {
      RCLCPP_INFO(
        node->get_logger(),
        "Center-regrasp abort recovery completed: the open left hand returned to local "
        "Y=%.1f mm while the right grasp constraint remained untouched.",
        serial_receiver_prehandover_y_offset * 1000.0);
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "Center-regrasp abort recovery stopped; the right grasp constraint was not changed.");
    }
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return success ? 0 : 1;
  }

  if (success && dual_hold_place_recovery_mode) {
    // Resume only from an interrupted state in which both grippers still
    // support the block.  Descend on one shared time axis and do not park near
    // the released table object; reset_only will park later in an empty scene.
    const bool recovery_right_already_open = gripper_open_target_is_reached(
      right_gripper, right_open, 0.0005,
      "dual-hold recovery initial right-open feedback");
    const bool recovery_left_already_open = gripper_open_target_is_reached(
      left_gripper, left_open, 0.0005,
      "dual-hold recovery initial left-open feedback");
    const bool recovery_grippers_already_open =
      recovery_right_already_open && recovery_left_already_open;
    right_closed = !recovery_right_already_open;
    left_closed = !recovery_left_already_open;
    if (recovery_grippers_already_open) {
      RCLCPP_INFO(
        node->get_logger(),
        "Both recovery grippers are already physically open; skipping all unload/open "
        "commands and proceeding directly to table verification and outward retreat.");
    }
    right_arm.setMaxVelocityScalingFactor(transfer_motion_velocity_scaling);
    right_arm.setMaxAccelerationScalingFactor(transfer_motion_acceleration_scaling);
    left_arm.setMaxVelocityScalingFactor(transfer_motion_velocity_scaling);
    left_arm.setMaxAccelerationScalingFactor(transfer_motion_acceleration_scaling);

    auto recovery_report = verify_gazebo_object_stability(
      node, gazebo_pose_monitor, recovery_table_surface_z,
      object_size_x, object_size_y, object_size_z,
      0.20, transfer_gate_observation,
      -0.002, cooperative_test_lift + transfer_gate_air_height_tolerance,
      transfer_gate_max_drift,
      transfer_gate_max_angle_degrees * 0.017453292519943295,
      transfer_gate_max_tilt_degrees * 0.017453292519943295,
      "dual-hold recovery supported baseline");
    success = recovery_report.has_value();

    if (success) {
      const auto planning_object_pose = transform_pose_to_planning_frame(
        recovery_report->final_sample.pose, tf_buffer);
      success = planning_object_pose.has_value();
      if (success) {
        success = planning_scene.applyCollisionObject(make_collision_object(
            *planning_object_pose, object_size_x, object_size_y, object_size_z));
      }
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
      // A center regrasp can place the long block against either Dex1 palm/base
      // as well as the finger pads.  Those are intentional grasp contacts; keep
      // every other robot and world collision active during the recovery.
      const auto touch_links = both_hand_touch_links();
      if (success) {
        success = right_arm.attachObject(kObjectId, "right_dex1_tcp", touch_links);
        owner = success ? Owner::Right : Owner::World;
      }
      if (success) {
        success = wait_for_attached_object_with_touch_links(
          planning_scene, kObjectId, "right_dex1_tcp", touch_links);
      }
    }

    std::optional<geometry_msgs::msg::Pose> expected_table_pose;
    if (success) {
      const double lowering_distance = std::max(
        0.0, recovery_report->bottom_clearance - 0.0005);
      expected_table_pose = recovery_report->final_sample.pose;
      if (recovery_report->bottom_clearance <= 0.003) {
        RCLCPP_INFO(
          node->get_logger(),
          "Dual-hold recovery detected the block already on the table (clearance %.2f mm); "
          "skipping vertical lowering and proceeding to verified unload.",
          recovery_report->bottom_clearance * 1000.0);
      } else if (lowering_distance < 0.001 || lowering_distance > 0.060) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Dual-hold recovery lowering distance %.2f mm is outside the safe airborne range.",
          lowering_distance * 1000.0);
        success = false;
      } else {
        expected_table_pose->position.z -= lowering_distance;
        moveit_msgs::msg::RobotTrajectory synchronized_lower;
        success = build_synchronized_cartesian_trajectory(
          right_arm, left_arm, tf_buffer, validity_client,
          *synchronized_motion_limits,
          -lowering_distance, cooperative_motion_duration,
          transfer_motion_velocity_scaling,
          transfer_motion_acceleration_scaling,
          "dual-hold recovery vertical table descent", synchronized_lower);
        if (success) {
          success = execute_synchronized_trajectory(
            both_arms, synchronized_lower,
            "dual-hold recovery vertical table descent");
        }
      }
    }

    const auto recovery_table_gate = [&](const std::string & stage_name) -> bool {
        const auto report = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, recovery_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.30, transfer_gate_observation,
          -0.002, 0.003,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          transfer_gate_max_tilt_degrees * 0.017453292519943295,
          stage_name);
        if (!report.has_value()) {
          return false;
        }
        recovery_report = report;
        return true;
      };

    if (success) {
      success = recovery_table_gate("dual-hold recovery placement at table");
    }
    if (success && expected_table_pose.has_value()) {
      success = verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, *expected_table_pose,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        "dual-hold recovery table placement versus commanded lowering");
    }
    if (success && !recovery_grippers_already_open) {
      success = plan_and_execute_joint(
        right_gripper, right_soft, "dual-hold recovery: unload right on table");
    }
    if (success && !recovery_grippers_already_open) {
      success = recovery_table_gate("dual-hold recovery after right unload");
    }
    if (success && !recovery_grippers_already_open) {
      success = plan_and_execute_joint(
        left_gripper, left_soft, "dual-hold recovery: unload left on table");
    }
    if (success && !recovery_grippers_already_open) {
      success = recovery_table_gate("dual-hold recovery after left unload");
    }
    if (success && !recovery_grippers_already_open) {
      success = plan_and_execute_joint(
        right_gripper, right_open, "dual-hold recovery: open right on table");
      if (success) {
        success = gripper_open_target_is_reached(
          right_gripper, right_open, 0.0005,
          "dual-hold recovery right-open feedback");
      }
      if (!success) {
        RCLCPP_WARN(
          node->get_logger(),
          "Dual-hold recovery right-open feedback missed the 0.5 mm gate; "
          "reasserting the same table-supported target once.");
        success = plan_and_execute_joint(
          right_gripper, right_open,
          "dual-hold recovery: reassert open right on table");
        if (success) {
          success = gripper_open_target_is_reached(
            right_gripper, right_open, 0.0005,
            "dual-hold recovery right-open feedback retry");
        }
      }
      right_closed = !success;
    }
    if (success && !recovery_grippers_already_open) {
      success = recovery_table_gate("dual-hold recovery after right release");
    }
    if (success && !recovery_grippers_already_open) {
      success = plan_and_execute_joint(
        left_gripper, left_open, "dual-hold recovery: open left on table");
      if (success) {
        success = gripper_open_target_is_reached(
          left_gripper, left_open, 0.0005,
          "dual-hold recovery left-open feedback");
      }
      if (!success) {
        RCLCPP_WARN(
          node->get_logger(),
          "Dual-hold recovery left-open feedback missed the 0.5 mm gate; "
          "reasserting the same table-supported target once.");
        success = plan_and_execute_joint(
          left_gripper, left_open,
          "dual-hold recovery: reassert open left on table");
        if (success) {
          success = gripper_open_target_is_reached(
            left_gripper, left_open, 0.0005,
            "dual-hold recovery left-open feedback retry");
        }
      }
      left_closed = !success;
    }
    if (success) {
      success = recovery_table_gate("dual-hold recovery released block on table");
    }
    if (success) {
      const bool world_restored = right_arm.detachObject(kObjectId) &&
        wait_for_world_object(planning_scene, kObjectId);
      success = world_restored;
      owner = world_restored ? Owner::World : Owner::Right;
    }
    if (success) {
      const auto released_object = gazebo_pose_monitor.latest();
      success = released_object.has_value();
      const auto planning_object_pose = success ?
        transform_pose_to_planning_frame(released_object->pose, tf_buffer) :
        std::optional<geometry_msgs::msg::Pose>{};
      success = planning_object_pose.has_value();
      if (success) {
        const double donor_tilt =
          cooperative_inward_tilt_degrees * 0.017453292519943295;
        const double inward_component =
          std::cos(donor_tilt) * cooperative_donor_approach_clearance;
        const double upward_component =
          std::sin(donor_tilt) * cooperative_donor_approach_clearance;
        auto right_retreat = make_inward_tcp_pose(
          *planning_object_pose, -cooperative_grasp_y_offset,
          cooperative_grasp_z_offset, donor_tilt);
        right_retreat.position.x += transfer_donor_x_offset;
        translate_pose_in_object_planar_frame(
          right_retreat, *planning_object_pose, 0.0,
          -inward_component, upward_component);
        success = plan_and_execute_cartesian(
          right_arm, right_retreat,
          "dual-hold recovery: right reverses original inward approach");
      }
    }
    if (success) {
      success = recovery_table_gate("dual-hold recovery after right retreat");
    }
    if (success) {
      const auto released_object = gazebo_pose_monitor.latest();
      success = released_object.has_value();
      const auto planning_object_pose = success ?
        transform_pose_to_planning_frame(released_object->pose, tf_buffer) :
        std::optional<geometry_msgs::msg::Pose>{};
      success = planning_object_pose.has_value();
      if (success) {
        const double receiver_tilt =
          transfer_receiver_tilt_degrees * 0.017453292519943295;
        const double receiver_staging_tilt =
          transfer_receiver_staging_tilt_degrees * 0.017453292519943295;
        const auto left_rotated_entry = make_inward_tcp_pose(
          *planning_object_pose, transfer_receiver_slide_start_y,
          transfer_receiver_z_offset, receiver_tilt);
        const auto left_pregrasp = make_inward_tcp_pose(
          *planning_object_pose, transfer_receiver_slide_start_y,
          transfer_receiver_staging_z_offset, receiver_staging_tilt);
        const auto left_outer = make_inward_tcp_pose(
          *planning_object_pose, transfer_receiver_staging_y,
          transfer_receiver_staging_z_offset, receiver_staging_tilt);
        const auto receiver_recovery_margin_gate = [&left_arm](
            const trajectory_msgs::msg::JointTrajectory & trajectory)
          {
            return verify_trajectory_joint_bound_margin(
              trajectory, left_arm.getRobotModel(), 0.05,
              "dual-hold receiver reverse path");
          };
        success = plan_and_execute_cartesian(
          left_arm, left_pregrasp,
          "dual-hold recovery: left reverses straight entry and outer rotation",
          false, receiver_recovery_margin_gate, {left_rotated_entry}, false);
        if (success) {
          success = plan_and_execute_cartesian(
            left_arm, left_outer,
            "dual-hold recovery: left returns to outer staging",
            false, receiver_recovery_margin_gate);
        }
      }
    }
    if (success) {
      success = recovery_table_gate("dual-hold recovery after left retreat");
    }
    if (success) {
      planning_scene.removeCollisionObjects({kObjectId});
      success = wait_for_object_absent(planning_scene, kObjectId);
      owner = success ? Owner::None : Owner::World;
    }

    if (success) {
      RCLCPP_INFO(
        node->get_logger(),
        "Dual-hold recovery completed: block returned to the table and both open hands "
        "retreated along their verified outward paths; use reset_only to park in an empty scene.");
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "Dual-hold recovery stopped; any gripper not yet table-verified remains closed.");
    }
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return success ? 0 : 1;
  }

  if (success && right_holder_place_recovery_mode) {
    // Resume only from an interrupted serial stage in which the right hand is
    // the sole physical support.  Return the real body to the table instead
    // of deleting it, and never open the gripper before the table gate passes.
    right_closed = true;
    right_arm.setMaxVelocityScalingFactor(transfer_motion_velocity_scaling);
    right_arm.setMaxAccelerationScalingFactor(transfer_motion_acceleration_scaling);

    // Recovery must be able to lower a slipping object instead of demanding
    // that it remain stable in mid-air first.  Require a fresh, finite pose
    // and a bounded table clearance, but make drift and the normal 10 degree
    // operating tilt diagnostic-only for this one pre-lowering sample.  All
    // post-lowering table, displacement and penetration gates remain strict.
    std::optional<ObjectStabilityReport> recovery_report;
    const auto recovery_sample = gazebo_pose_monitor.latest(0.50);
    if (!recovery_sample.has_value()) {
      RCLCPP_ERROR(
        node->get_logger(),
        "No fresh Gazebo pose is available for right-holder physical recovery.");
      success = false;
    } else {
      ObjectStabilityReport initial_report;
      initial_report.final_sample = *recovery_sample;
      initial_report.bottom_clearance = object_lowest_corner_z(
        recovery_sample->pose, object_size_x, object_size_y, object_size_z) -
        recovery_table_surface_z;
      initial_report.upright_tilt = object_upright_tilt(recovery_sample->pose);
      constexpr double kRecoveryInitialMaximumTilt =
        30.0 * 0.017453292519943295;
      constexpr double kRadiansToDegrees = 57.29577951308232;
      RCLCPP_WARN(
        node->get_logger(),
        "Right-holder recovery immediate baseline: clearance %.2f mm, tilt %.2f deg. "
        "Proceeding directly to vertical lowering without an airborne stability wait.",
        initial_report.bottom_clearance * 1000.0,
        initial_report.upright_tilt * kRadiansToDegrees);
      if (!std::isfinite(initial_report.bottom_clearance) ||
        !std::isfinite(initial_report.upright_tilt) ||
        initial_report.bottom_clearance < -0.002 ||
        initial_report.bottom_clearance >
        pickup_test_lift + transfer_gate_air_height_tolerance ||
        initial_report.upright_tilt > kRecoveryInitialMaximumTilt)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Right-holder recovery pose is outside the immediate-lowering envelope; "
          "refusing to infer a vertical displacement.");
        success = false;
      } else {
        recovery_report = initial_report;
      }
    }

    const bool already_released_on_table = success &&
      recovery_report->bottom_clearance <= 0.003 &&
      recovery_report->upright_tilt <=
      transfer_gate_max_tilt_degrees * 0.017453292519943295 &&
      gripper_open_target_is_reached(
        right_gripper, right_open, 0.0005,
        "right-holder recovery already-open feedback");
    if (already_released_on_table) {
      right_closed = false;
      RCLCPP_INFO(
        node->get_logger(),
        "Right-holder recovery detected an already released, table-flat block; "
        "skipping attachment and all gripper commands.");
      const auto current_right = current_link_pose_in_frame(
        right_arm, "right_dex1_tcp", kPlanningFrame, tf_buffer);
      success = current_right.has_value();
      if (success) {
        auto retreat = *current_right;
        retreat.position.z += transfer_receiver_retreat;
        success = plan_and_execute_cartesian(
          right_arm, retreat,
          "right-holder recovery: retreat from already released block");
      }
      if (success) {
        success = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, recovery_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.30, transfer_gate_observation,
          -0.002, 0.003,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          transfer_gate_max_tilt_degrees * 0.017453292519943295,
          "right-holder recovery released block after retreat").has_value();
      }
      if (success) {
        RCLCPP_INFO(
          node->get_logger(),
          "Right-holder recovery completed from the already released state; "
          "no Gazebo reset or repeated grasp was used.");
      } else {
        RCLCPP_ERROR(
          node->get_logger(),
          "Right-holder recovery could not finish the open-hand vertical retreat.");
      }
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return success ? 0 : 1;
    }

    if (success) {
      const auto planning_object_pose = transform_pose_to_planning_frame(
        recovery_report->final_sample.pose, tf_buffer);
      success = planning_object_pose.has_value();
      if (success) {
        success = planning_scene.applyCollisionObject(make_collision_object(
            *planning_object_pose, object_size_x, object_size_y, object_size_z));
      }
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
      const auto touch_links = finger_touch_links("right");
      if (success) {
        success = right_arm.attachObject(kObjectId, "right_dex1_tcp", touch_links);
        owner = success ? Owner::Right : Owner::World;
      }
      if (success) {
        success = wait_for_attached_object_with_touch_links(
          planning_scene, kObjectId, "right_dex1_tcp", touch_links);
      }
    }

    std::optional<geometry_msgs::msg::Pose> expected_table_pose;
    if (success) {
      const double lowering_distance = std::max(
        0.0, recovery_report->bottom_clearance - 0.0005);
      if (lowering_distance > 0.060) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Right-holder recovery lowering distance %.2f mm is outside the safe range.",
          lowering_distance * 1000.0);
        success = false;
      } else if (lowering_distance < 0.001) {
        expected_table_pose = recovery_report->final_sample.pose;
        RCLCPP_INFO(
          node->get_logger(),
          "Right-holder recovery block is already table-supported (clearance %.2f mm); "
          "skipping a redundant vertical lowering command.",
          recovery_report->bottom_clearance * 1000.0);
      } else {
        const auto current_right = current_link_pose_in_frame(
          right_arm, "right_dex1_tcp", kPlanningFrame, tf_buffer);
        success = current_right.has_value();
        if (success) {
          expected_table_pose = recovery_report->final_sample.pose;
          expected_table_pose->position.z -= lowering_distance;
          auto right_place = *current_right;
          right_place.position.z -= lowering_distance;
          constexpr double kRecoverySupportedEntry = 0.015;
          const double free_lowering = std::max(
            0.0, lowering_distance - kRecoverySupportedEntry);
          if (free_lowering > 0.001) {
            auto right_support_entry = *current_right;
            right_support_entry.position.z -= free_lowering;
            success = plan_and_execute_cartesian(
              right_arm, right_support_entry,
              "right-holder recovery: lower outside table safety shell");
          }
          if (success) {
            success = plan_and_execute_supported_object_vertical_segment(
              right_arm, right_place, false, tf_buffer, validity_client,
              get_scene_client, apply_scene_client,
              "right-holder recovery: enter final 15 mm table support");
          }
        }
      }
    }

    const auto recovery_table_gate = [&] (
        const std::string & stage_name,
        const double maximum_tilt_radians = -1.0) -> bool {
        const double active_maximum_tilt = maximum_tilt_radians > 0.0 ?
          maximum_tilt_radians :
          transfer_gate_max_tilt_degrees * 0.017453292519943295;
        const auto report = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, recovery_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.30, transfer_gate_observation,
          -0.002, 0.003,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          active_maximum_tilt,
          stage_name);
        if (!report.has_value()) {
          return false;
        }
        recovery_report = report;
        return true;
    };

    if (success) {
      success = recovery_table_gate(
        "right-holder recovery placement at table",
        30.0 * 0.017453292519943295);
    }
    if (success && expected_table_pose.has_value()) {
      success = verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, *expected_table_pose,
        transfer_gate_max_drift,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "right-holder recovery placement versus commanded lowering");
    }
    if (success && use_gazebo_grasp_constraint) {
      success = gazebo_grasp_constraint.detach_right_from_cube(true);
    }
    if (success) {
      success = plan_and_execute_joint(
        right_gripper, right_soft, "right-holder recovery: unload on table");
    }
    if (success) {
      success = recovery_table_gate(
        "right-holder recovery after preload unload",
        30.0 * 0.017453292519943295);
    }
    if (success) {
      success = plan_and_execute_joint(
        right_gripper, right_open, "right-holder recovery: release on table");
    }
    if (success) {
      success = gripper_open_target_is_reached(
        right_gripper, right_open, 0.0005,
        "right-holder recovery open feedback");
      if (!success) {
        RCLCPP_WARN(
          node->get_logger(),
          "Right-holder recovery open feedback missed the 0.5 mm gate; reasserting the "
          "same table-supported open target once.");
        success = plan_and_execute_joint(
          right_gripper, right_open,
          "right-holder recovery: reassert release on table");
        if (success) {
          success = gripper_open_target_is_reached(
            right_gripper, right_open, 0.0005,
            "right-holder recovery open feedback retry");
        }
      }
      right_closed = !success;
    }
    if (success) {
      success = recovery_table_gate("right-holder recovery released block on table");
    }
    if (success) {
      success = right_arm.detachObject(kObjectId);
      owner = success ? Owner::World : Owner::Right;
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
    }
    if (success) {
      // The physical table gate has already proved release.  Remove the world
      // collision box before separating the formerly touching fingers, or the
      // Cartesian solver rejects its contact start state at fraction 0%.
      planning_scene.removeCollisionObjects({kObjectId});
      success = wait_for_object_absent(planning_scene, kObjectId);
      owner = success ? Owner::None : Owner::World;
    }
    if (success) {
      const auto current_right = current_link_pose_in_frame(
        right_arm, "right_dex1_tcp", kPlanningFrame, tf_buffer);
      success = current_right.has_value();
      if (success) {
        auto retreat = *current_right;
        retreat.position.z += transfer_receiver_retreat;
        success = plan_and_execute_cartesian(
          right_arm, retreat, "right-holder recovery: retreat vertically");
      }
    }
    if (success) {
      success = recovery_table_gate("right-holder recovery after retreat");
    }
    if (success) {
      RCLCPP_INFO(
        node->get_logger(),
        "Right-holder recovery completed: the existing block is on the table and the open "
        "right hand retreated vertically; no Gazebo reset was used.");
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "Right-holder recovery stopped; the right gripper remains closed unless table release "
        "was already verified.");
    }
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return success ? 0 : 1;
  }

  if (success && receiver_place_recovery_mode) {
    // Resume only from the receiver-supported airborne state.  The left hand
    // must remain closed until Gazebo proves that the block is back on the table.
    left_closed = true;
    right_arm.setMaxVelocityScalingFactor(transfer_motion_velocity_scaling);
    right_arm.setMaxAccelerationScalingFactor(transfer_motion_acceleration_scaling);
    left_arm.setMaxVelocityScalingFactor(transfer_motion_velocity_scaling);
    left_arm.setMaxAccelerationScalingFactor(transfer_motion_acceleration_scaling);

    const bool recovery_donor_already_open = gripper_open_target_is_reached(
      right_gripper, right_open, 0.0005,
      "recovery initial donor-open feedback");
    const bool recovery_donor_already_parked = joint_target_is_reached(
      right_arm, right_park, 0.01, "recovery initial donor-park feedback");

    auto receiver_report = verify_gazebo_object_stability(
      node, gazebo_pose_monitor, recovery_table_surface_z,
      object_size_x, object_size_y, object_size_z,
      0.20, transfer_gate_observation,
      0.0005, cooperative_test_lift + transfer_gate_air_height_tolerance,
      transfer_gate_max_drift,
      transfer_gate_max_angle_degrees * 0.017453292519943295,
      transfer_gate_max_tilt_degrees * 0.017453292519943295,
      "recovery receiver-only airborne baseline");
    success = receiver_report.has_value();

    if (success) {
      const auto planning_object_pose = transform_pose_to_planning_frame(
        receiver_report->final_sample.pose, tf_buffer);
      success = planning_object_pose.has_value();
      if (success) {
        success = planning_scene.applyCollisionObject(make_collision_object(
            *planning_object_pose, object_size_x, object_size_y, object_size_z));
      }
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
      const auto touch_links = hand_touch_links("left");
      if (success) {
        success = left_arm.attachObject(kObjectId, "left_dex1_tcp", touch_links);
        owner = success ? Owner::Left : Owner::World;
      }
      if (success) {
        success = wait_for_attached_object_with_touch_links(
          planning_scene, kObjectId, "left_dex1_tcp", touch_links);
      }
    }

    const auto recovery_air_gate = [&](const std::string & stage_name) -> bool {
        const auto report = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, recovery_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.20, transfer_gate_observation,
          0.0005, cooperative_test_lift + transfer_gate_air_height_tolerance,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          transfer_gate_max_tilt_degrees * 0.017453292519943295,
          stage_name);
        if (!report.has_value() || !verify_gazebo_pose_near_reference(
            node, gazebo_pose_monitor, receiver_report->final_sample.pose,
            transfer_gate_max_anchor_displacement,
            transfer_gate_max_angle_degrees * 0.017453292519943295,
            stage_name + " versus recovery anchor"))
        {
          return false;
        }
        receiver_report = report;
        return true;
      };

    // The interrupted donor still has essentially zero pad clearance.  Open
    // it at the current pose before moving the arm, otherwise the pads drag
    // the long block through the receiver grasp during the retreat.
    if (success && !recovery_donor_already_open) {
      success = plan_and_execute_joint(
        right_gripper, right_open, "recovery: clear donor opens completely");
    }
    if (success) {
      success = joint_target_is_reached(
        right_gripper, right_open, 0.0005, "recovery clear donor open");
      if (!success) {
        RCLCPP_WARN(
          node->get_logger(),
          "Recovery donor-open feedback missed the unchanged 0.5 mm gate; "
          "reasserting the identical out-of-contact target once.");
        success = plan_and_execute_joint(
          right_gripper, right_open, "recovery: reassert clear donor open");
        if (success) {
          success = joint_target_is_reached(
            right_gripper, right_open, 0.0005, "recovery clear donor open retry");
        }
      }
      right_closed = !success;
    }
    if (success) {
      success = recovery_air_gate("recovery support after clear donor opens");
    }
    if (success && !recovery_donor_already_parked) {
      const auto current_right = current_link_pose_in_frame(
        right_arm, "right_dex1_tcp", kPlanningFrame, tf_buffer);
      success = current_right.has_value();
      if (success) {
        auto full_retreat = *current_right;
        const Eigen::Matrix3d object_rotation =
          pose_to_eigen(receiver_report->final_sample.pose).linear();
        const Eigen::Vector3d displacement =
          object_rotation * Eigen::Vector3d(0.0, -0.035, 0.0);
        full_retreat.position.x += displacement.x();
        full_retreat.position.y += displacement.y();
        full_retreat.position.z += displacement.z();
        success = plan_and_execute_cartesian(
          right_arm, full_retreat,
          "recovery: open donor completes 35 mm horizontal retreat");
      }
    }
    if (success && !recovery_donor_already_parked) {
      success = plan_and_execute_joint(
        right_arm, right_park, "recovery: park released donor arm");
    }
    if (success) {
      success = joint_target_is_reached(
        right_arm, right_park, 0.01, "recovery park released donor arm");
    }
    if (success) {
      success = recovery_air_gate("recovery receiver support after donor park");
    }

    std::optional<geometry_msgs::msg::Pose> expected_table_pose;
    if (success) {
      const auto current_left = current_link_pose_in_frame(
        left_arm, "left_dex1_tcp", kPlanningFrame, tf_buffer);
      success = current_left.has_value();
      if (success) {
        const double lowering_distance = std::max(
          0.0, receiver_report->bottom_clearance - 0.0005);
        if (lowering_distance < 0.001) {
          success = false;
        } else {
          expected_table_pose = receiver_report->final_sample.pose;
          expected_table_pose->position.z -= lowering_distance;
          const double final_supported_distance = std::min(0.015, lowering_distance);
          const double free_distance = lowering_distance - final_supported_distance;
          auto left_support_entry = *current_left;
          left_support_entry.position.z -= free_distance;
          if (free_distance >= 0.001) {
            success = plan_and_execute_cartesian(
              left_arm, left_support_entry,
              "recovery: receiver lowers outside table safety shell");
          }
          if (success) {
            const auto live_support_entry = current_link_pose_in_frame(
              left_arm, "left_dex1_tcp", kPlanningFrame, tf_buffer);
            success = live_support_entry.has_value();
            auto left_place = live_support_entry.value_or(left_support_entry);
            left_place.position.z -= final_supported_distance;
            if (success) {
            success = plan_and_execute_supported_object_vertical_segment(
              left_arm, left_place, false, tf_buffer, validity_client,
              get_scene_client, apply_scene_client,
              "recovery: receiver enters final table support segment",
              final_supported_distance);
            }
          }
        }
      }
    }
    if (success) {
      receiver_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, recovery_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.30, transfer_gate_observation,
        -0.002, 0.003,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "recovery receiver placement at table");
      success = receiver_report.has_value() && expected_table_pose.has_value() &&
        verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, *expected_table_pose,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        "recovery table placement versus commanded lowering");
    }
    if (success && use_gazebo_grasp_constraint) {
      success = gazebo_grasp_constraint.detach_left_from_cube(true);
    }
    if (success) {
      success = wait_for_sim_duration(
        node, 0.20, "recovery table support after left support detach");
    }
    if (success) {
      receiver_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, recovery_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.20, transfer_gate_observation,
        -0.002, 0.003,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "recovery table support after left compliant support detach");
      success = receiver_report.has_value();
    }
    if (success) {
      success = plan_and_execute_joint(
        left_gripper, left_soft, "recovery: receiver unloads on table");
    }
    if (success) {
      receiver_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, recovery_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.30, transfer_gate_observation,
        -0.002, 0.003,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "recovery table support after receiver unload");
      success = receiver_report.has_value();
    }
    if (success) {
      success = plan_and_execute_joint(
        left_gripper, left_open, "recovery: receiver releases block");
      left_closed = !success;
    }
    if (success) {
      receiver_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, recovery_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.30, transfer_gate_observation,
        -0.002, 0.003,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "recovery released block resting on table");
      success = receiver_report.has_value();
    }
    if (success) {
      success = left_arm.detachObject(kObjectId);
      owner = success ? Owner::World : Owner::Left;
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
    }
    if (success) {
      const auto current_left = current_link_pose_in_frame(
        left_arm, "left_dex1_tcp", kPlanningFrame, tf_buffer);
      success = current_left.has_value();
      if (success) {
        auto retreat = *current_left;
        retreat.position.z += transfer_receiver_retreat;
        success = plan_and_execute_cartesian(
          left_arm, retreat, "recovery: receiver retreats vertically");
      }
    }
    // Do not sweep the released receiver through a long joint-space park while
    // the block is still on the table.  The vertical retreat above is the safe
    // terminal pose; reset_only parks both arms after removing the block.
    if (success) {
      planning_scene.removeCollisionObjects({kObjectId});
      success = wait_for_object_absent(planning_scene, kObjectId);
      owner = success ? Owner::None : Owner::World;
    }

    if (success) {
      RCLCPP_INFO(
        node->get_logger(),
        "Receiver-only recovery completed: donor cleared and parked, block returned to the "
        "table, and the open receiver retreated vertically.");
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "Receiver-only recovery stopped; the left gripper remains closed unless table release "
        "was already verified.");
    }
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return success ? 0 : 1;
  }

  // Start from a repeatable, camera-clear configuration.  Move the right arm
  // out first: after an interrupted pose test it can still occupy the lower
  // shared-grasp point and block a direct left-arm park trajectory.
  bool left_parked_early = false;
  bool left_safe_receiver_branch_start = false;
  bool left_table_margin_recovery_performed = false;
  if (success) {
    success = plan_and_execute_joint(right_gripper, right_open, "open right gripper");
  }
  if (success) {
    // An interrupted run can leave an empty finger a few millimetres inside the
    // newly modeled 10 mm table safety shell.  Exit that already-existing
    // margin violation before commanding any finger motion; the helper permits
    // only the reported finger/table pairs and restores the full ACM before it
    // returns.
    success = recover_left_from_table_safety_margin_if_needed(
      left_arm, tf_buffer, validity_client, get_scene_client,
      apply_scene_client, left_table_margin_recovery_performed);
  }
  if (success) {
    success = plan_and_execute_joint(left_gripper, left_open, "open left gripper");
  }
  if (success) {
    success = plan_and_execute_joint(right_arm, right_park, "park right arm");
    if (success) {
      success = joint_target_is_reached(
        right_arm, right_park, 0.01, "park right arm");
    }
    if (!success) {
      RCLCPP_WARN(
        node->get_logger(),
        "Direct right-arm park was blocked after an interrupted run; trying a vertical retreat.");
      const auto current_pose = current_link_pose_in_frame(
        right_arm, "right_dex1_tcp", kPlanningFrame, tf_buffer);
      success = current_pose.has_value();
      if (success) {
        auto recovery_retreat = *current_pose;
        recovery_retreat.position.z += approach_clearance;
        success = plan_and_execute_cartesian(
          right_arm, recovery_retreat, "recover right arm above interrupted grasp");
      }
      if (success) {
        success = plan_and_execute_joint(right_arm, right_park, "park recovered right arm");
      }
      if (success) {
        success = joint_target_is_reached(
          right_arm, right_park, 0.01, "park recovered right arm");
      }
      if (!success) {
        // In serial handover failures the left receiver can be directly above
        // the donor.  Moving the donor farther upward is then the wrong
        // recovery direction.  Clear the empty receiver first and retry the
        // donor park in the now-open workspace.
        RCLCPP_WARN(
          node->get_logger(),
          "Right upward recovery is blocked; parking the empty left receiver first.");
        success = plan_and_execute_joint(
          left_arm, left_park, "recovery: clear left receiver before right park");
        if (success) {
          success = joint_target_is_reached(
            left_arm, left_park, 0.01,
            "recovery: clear left receiver before right park");
          left_parked_early = success;
        }
        if (success) {
          success = plan_and_execute_joint(
            right_arm, right_park, "recovery: park right after clearing left");
        }
        if (success) {
          success = joint_target_is_reached(
            right_arm, right_park, 0.01,
            "recovery: park right after clearing left");
        }
      }
    }
  }
  if (success && !left_parked_early) {
    const auto left_start_state = left_arm.getCurrentState(10.0);
    success = left_start_state != nullptr;
    double left_park_start_error = 0.0;
    if (success) {
      for (const auto & [joint_name, expected] : left_park) {
        left_park_start_error = std::max(
          left_park_start_error,
          std::abs(left_start_state->getVariablePosition(joint_name) - expected));
      }
    }
    if (success && cooperative_transfer_mode) {
      const double shoulder_pitch = left_start_state->getVariablePosition(
        "left_shoulder_pitch_joint");
      const double shoulder_yaw = left_start_state->getVariablePosition(
        "left_shoulder_yaw_joint");
      const double elbow = left_start_state->getVariablePosition("left_elbow_joint");
      const double wrist_roll = left_start_state->getVariablePosition(
        "left_wrist_roll_joint");
      const double wrist_pitch = left_start_state->getVariablePosition(
        "left_wrist_pitch_joint");
      left_safe_receiver_branch_start =
        shoulder_pitch >= -1.10 && shoulder_pitch <= -0.70 &&
        shoulder_yaw >= 0.50 && shoulder_yaw <= 1.10 &&
        elbow >= 2.15 && elbow <= 2.75 &&
        wrist_roll >= 1.45 && wrist_roll <= 2.20 &&
        wrist_pitch >= -0.05 && wrist_pitch <= 0.35;
      if (left_safe_receiver_branch_start) {
        left_safe_receiver_branch_start = gripper_open_target_is_reached(
          left_gripper, left_open, 0.0005,
          "cooperative startup retained receiver-open feedback");
      }
    }
    if (success && sequential_center_regrasp_test_mode) {
      const double shoulder_pitch = left_start_state->getVariablePosition(
        "left_shoulder_pitch_joint");
      const double shoulder_roll = left_start_state->getVariablePosition(
        "left_shoulder_roll_joint");
      const double shoulder_yaw = left_start_state->getVariablePosition(
        "left_shoulder_yaw_joint");
      const double elbow = left_start_state->getVariablePosition("left_elbow_joint");
      const double wrist_roll = left_start_state->getVariablePosition(
        "left_wrist_roll_joint");
      const double wrist_pitch = left_start_state->getVariablePosition(
        "left_wrist_pitch_joint");
      const double wrist_yaw = left_start_state->getVariablePosition(
        "left_wrist_yaw_joint");
      left_safe_receiver_branch_start =
        shoulder_pitch >= -0.60 && shoulder_pitch <= -0.20 &&
        shoulder_roll >= -0.10 && shoulder_roll <= 0.40 &&
        shoulder_yaw >= -0.90 && shoulder_yaw <= -0.20 &&
        elbow >= 0.45 && elbow <= 0.80 &&
        wrist_roll >= 0.10 && wrist_roll <= 0.80 &&
        wrist_pitch >= 0.00 && wrist_pitch <= 0.35 &&
        wrist_yaw >= -1.20 && wrist_yaw <= -0.80;
      if (left_safe_receiver_branch_start) {
        left_safe_receiver_branch_start = gripper_open_target_is_reached(
          left_gripper, left_open, 0.0005,
          "serial-center startup retained receiver-open feedback");
      }
    }
    if (success && left_safe_receiver_branch_start) {
      RCLCPP_INFO(
        node->get_logger(),
        "Left arm already occupies the verified open receiver outer/pregrasp branch; "
        "retaining it and skipping the unnecessary low-plane joint-space park.");
    } else if (success && left_park_start_error <= 0.01) {
      RCLCPP_INFO(
        node->get_logger(),
        "Left arm already satisfies the parked target; skipping startup recovery motion.");
    } else if (success) {
      // The receiver outer-staging TCP can be close to the physical Gazebo
      // table.  Clear the low plane first using the independently measured
      // 120 mm recovery lift; do not reuse a grasp approach distance.  A state
      // that started inside the modeled safety shell was already lifted by the
      // one-way recovery above and must not be lifted a second time.
      RCLCPP_WARN(
        node->get_logger(),
        "Left arm is not parked (maximum start error %.3f rad); preparing safe "
        "joint-space parking after vertical clearance.",
        left_park_start_error);
      if (mode == "reset_only") {
        // The wrapper removes the physical object before reset_only.  If the
        // current complete state is already collision-free, let MoveIt's
        // environment-aware joint planner take the empty receiver directly to
        // park.  This avoids demanding a fixed 120 mm Cartesian lift from a
        // high outer-staging posture whose vertical IK branch can end early.
        success = full_robot_state_is_valid(
          validity_client, *left_start_state,
          "empty-scene direct left park start");
        if (success) {
          RCLCPP_INFO(
            node->get_logger(),
            "Empty-scene reset start is fully valid; planning the left park directly "
            "with robot/environment collision checking.");
          success = plan_and_execute_joint(
            left_arm, left_park, "empty-scene direct left park");
        }
      } else if (!left_table_margin_recovery_performed) {
        const auto current_pose = current_link_pose_in_frame(
          left_arm, "left_dex1_tcp", kPlanningFrame, tf_buffer);
        success = current_pose.has_value();
        if (success) {
          auto clearance_pose = *current_pose;
          clearance_pose.position.z += kLeftStartupRecoveryClearance;
          success = plan_and_execute_cartesian(
            left_arm, clearance_pose,
            "startup clearance: raise empty left receiver before park");
        }
      } else {
        RCLCPP_INFO(
          node->get_logger(),
          "The empty left receiver already completed the 120 mm safety-shell exit lift.");
      }
      if (success && mode != "reset_only") {
        success = plan_and_execute_joint(
          left_arm, left_park, "park left arm after vertical clearance");
      }
      if (success) {
        success = joint_target_is_reached(
          left_arm, left_park, 0.01, "park left arm after vertical clearance");
      }
    }
  }
  if (success) {
    // Re-assert both arm targets on one controller time axis.  With Gazebo's
    // low position gain, the first arm can otherwise sag while the second arm
    // completes a long park trajectory and violate MoveIt's 0.02 rad start
    // tolerance on the very next plan.
    if (left_safe_receiver_branch_start) {
      RCLCPP_INFO(
        node->get_logger(),
        "Right arm is feedback-parked and the open left receiver is retained on its "
        "verified outward branch; skipping redundant both-arm reset.");
    } else if (joint_target_is_reached(
        both_arms, both_park, 0.01, "both arms already parked"))
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Both arms already satisfy the parked target; skipping redundant joint planning.");
    } else {
      success = plan_and_execute_joint(
        both_arms, both_park, "synchronize both parked arms");
      if (success) {
        success = joint_target_is_reached(
          both_arms, both_park, 0.01, "synchronize both parked arms");
      }
    }
  }
  if (success) {
    success = wait_for_sim_duration(node, 0.20, "synchronized park settle");
  }

  if (success && mode == "pose_test") {
    const auto handover_object = make_object_pose(
      handover_x, handover_y, tabletop_center_z + pickup_test_lift);
    const auto right_handover = make_downward_tcp_pose(
      handover_object, right_grasp_y_offset, grasp_z_offset,
      0.0, right_gripper_outward_yaw);
    const auto left_handover = make_downward_tcp_pose(
      handover_object, left_grasp_y_offset, grasp_z_offset,
      left_gripper_outward_yaw);
    auto right_pre_handover = right_handover;
    right_pre_handover.position.z += approach_clearance;
    auto left_pre_handover = left_handover;
    left_pre_handover.position.z += approach_clearance;

    const auto pose_test_direct_gate = [](
        const trajectory_msgs::msg::JointTrajectory & trajectory) {
        return verify_direct_joint_path(trajectory, "outward pose-test empty-hand path");
      };
    const auto right_pose_test_posture_gate = [](const std::map<std::string, double> & target) {
        return std::abs(target.at("right_shoulder_yaw_joint")) <= 1.2 &&
               target.at("right_elbow_joint") >= 0.8;
      };
    const auto right_pose_test_target = compute_nearest_seeded_ik_joint_target(
      node, right_arm, ik_client, tf_buffer, right_pre_handover, "right_dex1_tcp",
      right_park, "pose test: nearest natural right prehandover", 32, 7.5, 0.05,
      right_pose_test_posture_gate);
    success = right_pose_test_target.has_value();
    if (success) {
      success = plan_and_execute_seeded_joint_with_cartesian_probe(
        right_arm, *right_pose_test_target, {right_handover},
        "pose test: right takes direct outward-linkage grasp",
        {}, pose_test_direct_gate);
    }
    if (success) {
      success = plan_and_execute_cartesian(
        right_arm, right_handover, "pose test: right descends directly");
    }
    const auto left_pose_test_posture_gate = [](const std::map<std::string, double> & target) {
        return std::abs(target.at("left_shoulder_yaw_joint")) <= 1.2 &&
               target.at("left_elbow_joint") >= 0.8;
      };
    std::optional<std::map<std::string, double>> left_pose_test_target;
    if (success) {
      left_pose_test_target = compute_nearest_seeded_ik_joint_target(
        node, left_arm, ik_client, tf_buffer, left_pre_handover, "left_dex1_tcp",
        left_park, "pose test: nearest natural mirrored left prehandover", 8,
        7.5, 0.05, left_pose_test_posture_gate);
      success = left_pose_test_target.has_value();
    }
    if (success) {
      success = plan_and_execute_seeded_joint_with_cartesian_probe(
        left_arm, *left_pose_test_target, {left_handover},
        "pose test: left takes direct mirrored outward-linkage grasp",
        {}, pose_test_direct_gate);
    }
    if (success) {
      success = plan_and_execute_cartesian(
        left_arm, left_handover, "pose test: left descends directly");
    }
    if (success) {
      RCLCPP_INFO(
        node->get_logger(), "Pose test reached both grasp points; holding for %.1f seconds.",
        dual_hold_seconds);
      std::this_thread::sleep_for(std::chrono::duration<double>(dual_hold_seconds));
      // Exercise the real donor-release escape while both hands are still at
      // the shared grasp.  A vertical retreat drives the right wrist-pitch
      // joint into its limit; pulling toward -Y instead clears the block end
      // and increases the distance between the two hand bases.
      auto right_retreat = right_handover;
      right_retreat.position.y -= approach_clearance;
      success = plan_and_execute_cartesian(
        right_arm, right_retreat, "pose test: right exits outward");
    }
    if (success) {
      success = plan_and_execute_joint(right_arm, right_park, "pose test: park right");
    }
    if (success) {
      success = plan_and_execute_cartesian(
        left_arm, left_pre_handover, "pose test: left retreat");
    }
    if (success) {
      success = plan_and_execute_joint(left_arm, left_park, "pose test: park left");
    }
  }

  geometry_msgs::msg::Pose pickup_object;
  geometry_msgs::msg::Pose right_grasp;
  geometry_msgs::msg::Pose right_pregrasp;
  geometry_msgs::msg::Pose right_lift;
  geometry_msgs::msg::Pose left_grasp;
  geometry_msgs::msg::Pose left_receiver_rotated_entry;
  geometry_msgs::msg::Pose left_pregrasp;
  geometry_msgs::msg::Pose left_outer_staging;
  std::optional<geometry_msgs::msg::Pose> serial_left_staging_pose;
  std::optional<geometry_msgs::msg::Pose> serial_left_nominal_contact_pose;
  bool serial_left_prepositioned =
    sequential_center_regrasp_test_mode && left_safe_receiver_branch_start;
  std::optional<double> gazebo_table_surface_z;
  std::optional<geometry_msgs::msg::Pose> gazebo_initial_object_pose;
  if (success && mode != "reset_only" && mode != "pose_test") {
    RCLCPP_INFO(node->get_logger(), "Waiting for fresh real-camera block detection...");
    const auto detection = wait_for_stable_object_pose(
      node, tf_buffer, detection_timeout, detection_max_age, detection_max_spread, 3, true);
    if (!detection.has_value()) {
      RCLCPP_ERROR(node->get_logger(), "No stable pickup pose received before timeout.");
      success = false;
    } else {
      pickup_object = *detection;
      pickup_object.position.x += vision_grasp_x_correction;
      RCLCPP_INFO(
        node->get_logger(),
        "Applied pelvis-frame visual grasp correction dx=%.1f mm; corrected center x=%.3f m.",
        vision_grasp_x_correction * 1000.0, pickup_object.position.x);
      if (sequential_center_regrasp_test_mode) {
        const auto gazebo_pose = gazebo_pose_monitor.latest();
        const auto gazebo_planning_pose = gazebo_pose.has_value() ?
          transform_pose_to_planning_frame(gazebo_pose->pose, tf_buffer) :
          std::optional<geometry_msgs::msg::Pose>{};
        if (!gazebo_pose.has_value() || !gazebo_planning_pose.has_value()) {
          RCLCPP_ERROR(
            node->get_logger(),
            "Center-regrasp startup lacks a Gazebo orientation cross-check.");
          success = false;
        } else {
          const double orientation_error = pose_rotation_distance(
            pickup_object, *gazebo_planning_pose);
          if (orientation_error >
            transfer_gate_max_angle_degrees * 0.017453292519943295)
          {
            RCLCPP_WARN(
              node->get_logger(),
              "Tabletop HSV yaw is occluded by the staged receiver (%.2f deg from Gazebo); "
              "retaining visual XYZ but using the physically verified Gazebo orientation.",
              orientation_error * 57.29577951308232);
            pickup_object.orientation = gazebo_planning_pose->orientation;
          }
        }
      }
      if (success && cooperative_mode) {
        const double tilt = cooperative_inward_tilt_degrees * 0.017453292519943295;
        const double donor_approach_clearance = cooperative_transfer_mode ?
          cooperative_donor_approach_clearance : approach_clearance;
        const double inward_component = std::cos(tilt) * donor_approach_clearance;
        const double upward_component = std::sin(tilt) * donor_approach_clearance;
        right_grasp = make_inward_tcp_pose(
          pickup_object, -cooperative_grasp_y_offset, cooperative_grasp_z_offset, tilt);
        if (cooperative_transfer_mode) {
          right_grasp.position.x += transfer_donor_x_offset;
          RCLCPP_INFO(
            node->get_logger(),
            "Applied donor-only contact alignment dx=%.1f mm; receiver remains centered on vision.",
            transfer_donor_x_offset * 1000.0);
        }
        right_pregrasp = right_grasp;
        translate_pose_in_object_planar_frame(
          right_pregrasp, pickup_object, 0.0, -inward_component, upward_component);
        if (cooperative_transfer_mode) {
          const double receiver_tilt =
            transfer_receiver_tilt_degrees * 0.017453292519943295;
          left_grasp = make_inward_tcp_pose(
            pickup_object, transfer_receiver_y_offset,
            transfer_receiver_z_offset, receiver_tilt);
          const double receiver_staging_tilt =
            transfer_receiver_staging_tilt_degrees * 0.017453292519943295;
          // Keep the proven high-clearance wrist orientation at the empty
          // outer/pregrasp stations.  After the donor reaches contact, rotate
          // the receiver in place at the still-clear slide-start station, then
          // enter along a straight fixed-orientation segment.  Separating the
          // rotation from the inward slide keeps the combined support centered
          // and avoids the two Dex1 bases sweeping through each other.
          left_receiver_rotated_entry = make_inward_tcp_pose(
            pickup_object, transfer_receiver_slide_start_y,
            transfer_receiver_z_offset, receiver_tilt);
          left_pregrasp = make_inward_tcp_pose(
            pickup_object, transfer_receiver_slide_start_y,
            transfer_receiver_staging_z_offset, receiver_staging_tilt);
          left_outer_staging = make_inward_tcp_pose(
            pickup_object, transfer_receiver_staging_y,
            transfer_receiver_staging_z_offset, receiver_staging_tilt);
          const auto gazebo_pose = gazebo_pose_monitor.latest();
          if (!gazebo_pose.has_value()) {
            RCLCPP_ERROR(
              node->get_logger(),
              "No fresh Gazebo object pose is available before cooperative transfer.");
            success = false;
          } else {
            gazebo_initial_object_pose = gazebo_pose->pose;
            gazebo_table_surface_z = object_lowest_corner_z(
              gazebo_pose->pose, object_size_x, object_size_y, object_size_z);
            RCLCPP_INFO(
              node->get_logger(),
              "Transfer physical baseline: table surface z=%.6f m; receiver TCP y=%.1f mm, "
              "contact/staging z offset=%.1f/%.1f mm, pregrasp/outer y=%.1f/%.1f mm, "
              "staging/contact tilt=%.1f/%.1f deg.",
              *gazebo_table_surface_z, transfer_receiver_y_offset * 1000.0,
              transfer_receiver_z_offset * 1000.0,
              transfer_receiver_staging_z_offset * 1000.0,
              transfer_receiver_slide_start_y * 1000.0,
              transfer_receiver_staging_y * 1000.0,
              transfer_receiver_staging_tilt_degrees,
              transfer_receiver_tilt_degrees);
          }
        } else {
          left_grasp = make_inward_tcp_pose(
            pickup_object, cooperative_grasp_y_offset, cooperative_grasp_z_offset, tilt);
          left_pregrasp = left_grasp;
          translate_pose_in_object_planar_frame(
            left_pregrasp, pickup_object, 0.0, inward_component, upward_component);
        }
      } else if (success) {
        if (sequential_center_regrasp_test_mode) {
          const double donor_tilt =
            cooperative_inward_tilt_degrees * 0.017453292519943295;
          right_grasp = make_inward_tcp_pose(
            pickup_object, right_grasp_y_offset, grasp_z_offset, donor_tilt);
          right_pregrasp = right_grasp;
          const double inward_component =
            std::cos(donor_tilt) * cooperative_donor_approach_clearance;
          const double upward_component =
            std::sin(donor_tilt) * cooperative_donor_approach_clearance;
          translate_pose_in_object_planar_frame(
            right_pregrasp, pickup_object, 0.0, -inward_component, upward_component);
        } else {
          right_grasp = make_object_aligned_downward_tcp_pose(
            pickup_object, right_grasp_y_offset, grasp_z_offset,
            0.0, right_gripper_outward_yaw);
          right_pregrasp = right_grasp;
          right_pregrasp.position.z += approach_clearance;
        }
        const Eigen::Isometry3d pickup_transform = pose_to_eigen(pickup_object);
        const double pickup_yaw = std::atan2(
          pickup_transform.linear()(1, 0), pickup_transform.linear()(0, 0));
        RCLCPP_INFO(
          node->get_logger(),
          "Object-aligned right grasp: local Y offset %.1f mm, tabletop yaw %.2f deg.",
          right_grasp_y_offset * 1000.0, pickup_yaw * 57.29577951308232);
        right_lift = right_grasp;
        right_lift.position.z += physical_right_pick_mode ? pickup_test_lift : handover_lift;
        if (sequential_vision_mode) {
          auto nominal_handover_object = make_object_pose(
            handover_x, handover_y,
            pickup_object.position.z + pickup_test_lift + serial_carry_z_compensation);
          nominal_handover_object.orientation = pickup_object.orientation;
          if (sequential_center_regrasp_test_mode) {
            const double receiver_tilt =
              center_regrasp_search_tilt_degrees * 0.017453292519943295;
            serial_left_nominal_contact_pose = make_inward_tcp_pose(
              nominal_handover_object, left_grasp_y_offset,
              transfer_receiver_z_offset, receiver_tilt);
            serial_left_staging_pose = make_inward_tcp_pose(
              nominal_handover_object, serial_receiver_prehandover_y_offset,
              transfer_receiver_z_offset, receiver_tilt);
          } else {
            serial_left_nominal_contact_pose = make_object_aligned_downward_tcp_pose(
              nominal_handover_object, left_grasp_y_offset, grasp_z_offset,
              left_gripper_outward_yaw);
            serial_left_staging_pose = make_object_aligned_downward_tcp_pose(
              nominal_handover_object, serial_receiver_prehandover_y_offset,
              grasp_z_offset, left_gripper_outward_yaw);
          }
        }
        if (physical_right_pick_mode) {
          const auto gazebo_pose = gazebo_pose_monitor.latest();
          if (!gazebo_pose.has_value()) {
            RCLCPP_ERROR(
              node->get_logger(),
              "No fresh Gazebo object pose is available before the right pickup test.");
            success = false;
          } else {
            gazebo_initial_object_pose = gazebo_pose->pose;
            gazebo_table_surface_z = object_lowest_corner_z(
              gazebo_pose->pose, object_size_x, object_size_y, object_size_z);
            RCLCPP_INFO(
              node->get_logger(),
              "Right-pick physical baseline: table surface z=%.6f m, object center "
              "[%.3f, %.3f, %.3f].",
              *gazebo_table_surface_z, gazebo_pose->pose.position.x,
              gazebo_pose->pose.position.y, gazebo_pose->pose.position.z);
          }
        }
      }
    }
  }

  if (success && cooperative_transfer_mode) {
    const auto planning_object_pose = transform_pose_to_planning_frame(
      *gazebo_initial_object_pose, tf_buffer);
    success = planning_object_pose.has_value();
    if (success) {
      std_msgs::msg::ColorRGBA color;
      color.r = 0.10F;
      color.g = 0.95F;
      color.b = 0.15F;
      color.a = 1.0F;
      success = planning_scene.applyCollisionObject(
        make_collision_object(
          *planning_object_pose, object_size_x, object_size_y, object_size_z),
        color);
      owner = success ? Owner::World : Owner::None;
    }
    if (success) {
      success = wait_for_world_object(planning_scene, kObjectId);
    }
  }

  std::function<bool(const trajectory_msgs::msg::JointTrajectory &)> donor_branch_gate;
  std::optional<std::map<std::string, double>> donor_lift_preflight_target;
  std::optional<std::map<std::string, double>> receiver_lift_preflight_target;
  if (success && cooperative_mode) {
    // The central table pose lets both grippers establish support before the
    // lift.  The lift test is symmetric; the transfer test places the receiver
    // near the center of mass so it can safely become the only support.
    if (cooperative_transfer_mode) {
      const std::map<std::string, std::pair<double, double>> donor_joint_bounds{
        {"right_shoulder_pitch_joint", {-3.14, 1.57}},
        {"right_shoulder_roll_joint", {-3.40, 0.38}},
        {"right_shoulder_yaw_joint", {-3.01, 2.66}},
        {"right_elbow_joint", {-0.95, 3.18}},
        {"right_wrist_roll_joint", {-2.75, 3.01}},
        {"right_wrist_pitch_joint", {-0.4625, 0.4625}},
        {"right_wrist_yaw_joint", {-1.27, 1.27}},
      };
      const auto donor_solution_gate = [node, donor_joint_bounds](
        const std::map<std::string, double> & positions)
        {
          double minimum_margin = std::numeric_limits<double>::infinity();
          for (const auto & [joint_name, bounds] : donor_joint_bounds) {
            const auto found = positions.find(joint_name);
            if (found == positions.end() || !std::isfinite(found->second)) {
              return false;
            }
            minimum_margin = std::min(
              minimum_margin,
              std::min(found->second - bounds.first, bounds.second - found->second));
          }
          const double shoulder_yaw = positions.at("right_shoulder_yaw_joint");
          const double elbow = positions.at("right_elbow_joint");
          const double wrist_roll = positions.at("right_wrist_roll_joint");
          const double wrist_yaw = positions.at("right_wrist_yaw_joint");
          const bool accepted =
            shoulder_yaw > -0.10 && shoulder_yaw < 1.40 &&
            elbow > -0.10 && elbow < 1.50 &&
            wrist_roll > -1.20 && wrist_roll < 0.80 &&
            wrist_yaw > 0.40 && wrist_yaw < 1.25 &&
            minimum_margin >= 0.005;
          RCLCPP_INFO(
            node->get_logger(),
            "Donor IK signature: shoulder yaw %.3f, elbow %.3f, wrist roll %.3f, "
            "wrist yaw %.3f, minimum bound margin %.1f mrad (%s).",
            shoulder_yaw, elbow, wrist_roll, wrist_yaw,
            minimum_margin * 1000.0, accepted ? "accepted" : "rejected");
          return accepted;
        };
      donor_branch_gate = [node, donor_joint_bounds](
        const trajectory_msgs::msg::JointTrajectory & trajectory)
        {
          if (trajectory.points.size() < 2) {
            return false;
          }
          std::map<std::string, size_t> indices;
          for (const auto & [joint_name, unused] : donor_joint_bounds) {
            static_cast<void>(unused);
            const auto found = std::find(
              trajectory.joint_names.begin(), trajectory.joint_names.end(), joint_name);
            if (found == trajectory.joint_names.end()) {
              return false;
            }
            indices.emplace(
              joint_name,
              static_cast<size_t>(std::distance(trajectory.joint_names.begin(), found)));
          }
          double minimum_margin = std::numeric_limits<double>::infinity();
          double maximum_joint_step = 0.0;
          for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
            const auto & point = trajectory.points[point_index];
            if (point.positions.size() != trajectory.joint_names.size()) {
              return false;
            }
            for (const auto & [joint_name, bounds] : donor_joint_bounds) {
              const size_t joint_index = indices.at(joint_name);
              const double position = point.positions[joint_index];
              if (!std::isfinite(position)) {
                return false;
              }
              minimum_margin = std::min(
                minimum_margin,
                std::min(position - bounds.first, bounds.second - position));
              if (point_index > 0) {
                maximum_joint_step = std::max(
                  maximum_joint_step,
                  std::abs(position - trajectory.points[point_index - 1].positions[joint_index]));
              }
            }
            const double shoulder_yaw =
              point.positions[indices.at("right_shoulder_yaw_joint")];
            const double elbow = point.positions[indices.at("right_elbow_joint")];
            const double wrist_roll =
              point.positions[indices.at("right_wrist_roll_joint")];
            const double wrist_yaw = point.positions[indices.at("right_wrist_yaw_joint")];
            if (shoulder_yaw <= -0.10 || shoulder_yaw >= 1.40 ||
              elbow <= -0.10 || elbow >= 1.50 ||
              wrist_roll <= -1.20 || wrist_roll >= 0.80 ||
              wrist_yaw <= 0.40 || wrist_yaw >= 1.25)
            {
              return false;
            }
          }
          const auto & endpoint = trajectory.points.back();
          const bool accepted = minimum_margin >= 0.005 && maximum_joint_step <= 0.35;
          RCLCPP_INFO(
            node->get_logger(),
            "Donor branch path: %zu points, endpoint [yaw %.3f, elbow %.3f, "
            "wrist roll %.3f, wrist yaw %.3f], minimum bound margin %.1f mrad, "
            "maximum joint step %.3f rad (%s).",
            trajectory.points.size(),
            endpoint.positions[indices.at("right_shoulder_yaw_joint")],
            endpoint.positions[indices.at("right_elbow_joint")],
            endpoint.positions[indices.at("right_wrist_roll_joint")],
            endpoint.positions[indices.at("right_wrist_yaw_joint")],
            minimum_margin * 1000.0, maximum_joint_step,
            accepted ? "accepted" : "rejected");
          return accepted;
        };
      const std::map<std::string, double> donor_known_good_seed{
        {"right_shoulder_pitch_joint", -0.300},
        {"right_shoulder_roll_joint", -0.210},
        {"right_shoulder_yaw_joint", 0.519},
        {"right_elbow_joint", 0.473},
        {"right_wrist_roll_joint", -0.352},
        {"right_wrist_pitch_joint", 0.073},
        {"right_wrist_yaw_joint", 1.052},
      };
      auto right_lift_probe = right_grasp;
      right_lift_probe.position.z += cooperative_test_lift;
      std::optional<std::map<std::string, double>> right_pregrasp_joint_target;
      // Solving the distant pregrasp repeatedly with one identical seed always
      // selected the same branch, which stopped part-way through contact.  The
      // load-bearing contact and vertical lift are the constraints that matter,
      // so solve that centered contact first and trace the exact branch outward
      // to obtain its pregrasp endpoint.  Neither Cartesian trace is executed.
      // Temporarily attach the stationary planning object to the fixed pelvis
      // with only the donor finger links marked as touch links.  This preserves
      // self/body collision checking while distinguishing intended pad contact
      // from an actual arm or wrist collision.  Restore the object to World
      // before planning or executing any empty-hand motion.
      const auto donor_contact_touch_links = finger_touch_links("right");
      const bool donor_contact_attachment_commanded = both_arms.attachObject(
        kObjectId, "pelvis", donor_contact_touch_links);
      owner = donor_contact_attachment_commanded ? Owner::Base : Owner::World;
      const bool donor_contact_scene_ready = donor_contact_attachment_commanded &&
        wait_for_attached_object_with_touch_links(
        planning_scene, kObjectId, "pelvis", donor_contact_touch_links);
      std::optional<std::map<std::string, double>> donor_contact_target;
      if (donor_contact_scene_ready) {
        std::vector<std::map<std::string, double>> donor_contact_seeds;
        const auto add_seed = [&donor_contact_seeds, &donor_known_good_seed](
            const double shoulder_pitch_delta,
            const double shoulder_roll_delta,
            const double shoulder_yaw_delta,
            const double elbow_delta,
            const double wrist_roll_delta,
            const double wrist_pitch_delta,
            const double wrist_yaw_delta)
          {
            auto seed = donor_known_good_seed;
            seed.at("right_shoulder_pitch_joint") += shoulder_pitch_delta;
            seed.at("right_shoulder_roll_joint") += shoulder_roll_delta;
            seed.at("right_shoulder_yaw_joint") += shoulder_yaw_delta;
            seed.at("right_elbow_joint") += elbow_delta;
            seed.at("right_wrist_roll_joint") += wrist_roll_delta;
            seed.at("right_wrist_pitch_joint") += wrist_pitch_delta;
            seed.at("right_wrist_yaw_joint") += wrist_yaw_delta;
            donor_contact_seeds.push_back(std::move(seed));
          };
        add_seed(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        add_seed(0.0, 0.0, 0.16, 0.0, 0.0, 0.0, 0.0);
        add_seed(0.0, 0.0, -0.16, 0.0, 0.0, 0.0, 0.0);
        add_seed(0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0);
        add_seed(0.0, 0.0, 0.0, -0.20, 0.0, 0.0, 0.0);
        add_seed(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15);
        add_seed(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.20);
        add_seed(0.0, 0.0, 0.08, 0.25, 0.0, 0.0, 0.08);
        add_seed(0.0, 0.0, -0.08, 0.25, 0.0, 0.0, -0.08);
        add_seed(-0.15, 0.10, 0.08, 0.25, 0.0, 0.0, 0.08);
        add_seed(0.15, -0.10, -0.08, 0.25, 0.0, 0.0, -0.08);
        add_seed(0.0, 0.0, 0.08, 0.20, 0.20, 0.15, 0.08);
        add_seed(0.0, 0.0, -0.08, 0.20, -0.20, -0.15, -0.08);
        auto pregrasp_signature_seed = donor_known_good_seed;
        pregrasp_signature_seed.at("right_shoulder_yaw_joint") = 0.543;
        pregrasp_signature_seed.at("right_elbow_joint") = 0.810;
        pregrasp_signature_seed.at("right_wrist_roll_joint") = -0.287;
        pregrasp_signature_seed.at("right_wrist_yaw_joint") = 1.130;
        donor_contact_seeds.push_back(std::move(pregrasp_signature_seed));

        const size_t donor_contact_seed_count = donor_preflight_only ?
          1 : donor_contact_seeds.size();
        for (size_t index = 0;
          index < donor_contact_seed_count && !donor_contact_target.has_value(); ++index)
        {
          donor_contact_target = compute_seeded_ik_joint_target(
            node, right_arm, ik_client, tf_buffer, right_grasp, "right_dex1_tcp",
            donor_contact_seeds[index],
            "cooperative transfer: donor allowed-contact IK seed " +
            std::to_string(index + 1) + "/" + std::to_string(donor_contact_seed_count),
            donor_solution_gate);
        }

        if (!donor_contact_target.has_value() && !donor_preflight_only) {
          bool kinematic_only_solution_found = false;
          for (size_t index = 0;
            index < donor_contact_seeds.size() && !kinematic_only_solution_found; ++index)
          {
            const auto diagnostic_target = compute_seeded_ik_joint_target(
              node, right_arm, ik_client, tf_buffer, right_grasp, "right_dex1_tcp",
              donor_contact_seeds[index],
              "diagnostic-only donor IK without collision checking seed " +
              std::to_string(index + 1) + "/" + std::to_string(donor_contact_seeds.size()),
              {}, false);
            kinematic_only_solution_found = diagnostic_target.has_value();
          }
          RCLCPP_ERROR(
            node->get_logger(),
            "Donor contact seed scan found %s; no diagnostic solution will be executed.",
            kinematic_only_solution_found ?
            "a kinematic-only solution blocked by collision geometry" :
            "no IK solution even with collision checking disabled");
        }
      } else {
        RCLCPP_ERROR(
          node->get_logger(),
          "Could not establish the temporary donor finger-contact planning scene.");
      }
      if (donor_contact_target.has_value()) {
        moveit_msgs::msg::RobotState contact_start;
        contact_start.is_diff = true;
        for (const auto & [joint_name, position] : *donor_contact_target) {
          contact_start.joint_state.name.push_back(joint_name);
          contact_start.joint_state.position.push_back(position);
        }
        right_arm.setStartState(contact_start);
        moveit_msgs::msg::RobotTrajectory outward_trace;
        moveit_msgs::msg::MoveItErrorCodes outward_trace_error;
        const double outward_fraction = right_arm.computeCartesianPath(
          {right_pregrasp}, kCartesianEefStep,
          outward_trace, true, &outward_trace_error);
        const auto & outward_joints = outward_trace.joint_trajectory;
        RCLCPP_INFO(
          node->get_logger(),
          "Donor contact-to-pregrasp reverse trace: %.1f%% (%zu points, error=%d).",
          outward_fraction * 100.0, outward_joints.points.size(), outward_trace_error.val);

        if (outward_fraction >= kMinCartesianFraction && outward_joints.points.size() >= 2 &&
          donor_branch_gate(outward_joints) &&
          outward_joints.joint_names.size() == outward_joints.points.back().positions.size())
        {
          std::map<std::string, double> traced_pregrasp_target;
          bool finite_endpoint = true;
          for (size_t index = 0; index < outward_joints.joint_names.size(); ++index) {
            const double position = outward_joints.points.back().positions[index];
            finite_endpoint = finite_endpoint && std::isfinite(position);
            traced_pregrasp_target.emplace(outward_joints.joint_names[index], position);
          }
          if (finite_endpoint) {
            moveit_msgs::msg::RobotState traced_pregrasp_start;
            traced_pregrasp_start.is_diff = true;
            for (const auto & [joint_name, position] : traced_pregrasp_target) {
              traced_pregrasp_start.joint_state.name.push_back(joint_name);
              traced_pregrasp_start.joint_state.position.push_back(position);
            }
            right_arm.setStartState(traced_pregrasp_start);
            moveit_msgs::msg::RobotTrajectory forward_probe;
            moveit_msgs::msg::MoveItErrorCodes forward_probe_error;
            const double forward_fraction = right_arm.computeCartesianPath(
              {right_grasp, right_lift_probe}, kCartesianEefStep,
              forward_probe, true, &forward_probe_error);
            RCLCPP_INFO(
              node->get_logger(),
              "Donor traced pregrasp contact-and-5 cm lift preview: %.1f%% "
              "(%zu points, error=%d).",
              forward_fraction * 100.0,
              forward_probe.joint_trajectory.points.size(), forward_probe_error.val);
            if (forward_fraction >= kMinCartesianFraction &&
              forward_probe.joint_trajectory.points.size() >= 2 &&
              donor_branch_gate(forward_probe.joint_trajectory))
            {
              const auto & probe_joints = forward_probe.joint_trajectory;
              const auto & probe_endpoint = probe_joints.points.back();
              std::map<std::string, double> lift_target;
              if (probe_joints.joint_names.size() == probe_endpoint.positions.size()) {
                for (size_t index = 0; index < probe_joints.joint_names.size(); ++index) {
                  lift_target.emplace(
                    probe_joints.joint_names[index], probe_endpoint.positions[index]);
                }
                donor_lift_preflight_target = std::move(lift_target);
              }
              right_pregrasp_joint_target = std::move(traced_pregrasp_target);
            }
          }
        }
      }
      bool donor_contact_scene_restored = false;
      if (donor_contact_attachment_commanded) {
        donor_contact_scene_restored = both_arms.detachObject(kObjectId);
        owner = donor_contact_scene_restored ? Owner::World : Owner::Base;
        if (donor_contact_scene_restored) {
          donor_contact_scene_restored = wait_for_world_object(planning_scene, kObjectId);
        }
      }
      if (!donor_contact_scene_restored) {
        RCLCPP_ERROR(
          node->get_logger(),
          "The temporary donor contact scene was not restored to World; refusing to move.");
        right_pregrasp_joint_target.reset();
      }
      right_arm.setStartStateToCurrentState();
      if (!right_pregrasp_joint_target.has_value()) {
        RCLCPP_ERROR(
          node->get_logger(),
          "The donor contact branch could not trace through pregrasp, centered contact "
          "and the complete 5 cm lift preview.");
      }
      success = right_pregrasp_joint_target.has_value();
      if (success && donor_preflight_only) {
        RCLCPP_INFO(
          node->get_logger(),
          "Donor preflight passed without executing a robot trajectory; the planning object "
          "has been restored to World.");
        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();
        return 0;
      }
      if (success && !transfer_preflight_only) {
        success = plan_and_execute_seeded_joint_with_cartesian_probe(
          right_arm, *right_pregrasp_joint_target,
          {right_grasp, right_lift_probe},
          "cooperative transfer: seeded donor pregrasp",
          donor_branch_gate);
      } else if (success) {
        RCLCPP_INFO(
          node->get_logger(),
          "Receiver preflight requested; donor branch passed but the right arm remains parked.");
      }
    } else {
      success = plan_and_execute_pose(
        right_arm, right_pregrasp, "cooperative test: right pregrasp");
    }
    if (success && cooperative_transfer_mode && !transfer_preflight_only) {
      success = verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, *gazebo_initial_object_pose,
        0.005, 3.0 * 0.017453292519943295,
        "right donor reached collision-aware pregrasp");
    }
    if (success && cooperative_transfer_mode) {
      for (auto * group : {&right_arm, &left_arm, &both_arms}) {
        group->setMaxVelocityScalingFactor(transfer_motion_velocity_scaling);
        group->setMaxAccelerationScalingFactor(transfer_motion_acceleration_scaling);
      }
      RCLCPP_INFO(
        node->get_logger(),
        "Entering receiver-pregrasp and load-bearing window at velocity=%.2f, "
        "acceleration=%.2f; empty-space scaling remains %.2f/%.2f.",
        transfer_motion_velocity_scaling, transfer_motion_acceleration_scaling,
        velocity_scaling, acceleration_scaling);
    }
    if (success && cooperative_transfer_mode) {
      // The same TCP pose has both low- and high-elbow IK branches.  Repeated
      // unconstrained pose plans deterministically return the low branch on
      // this model, so solve the high branch explicitly from a tested seed,
      // then plan to that exact joint target.  Validate every continuation
      // before moving and never relax the collision or Cartesian gates.
      auto left_lift_probe = left_grasp;
      left_lift_probe.position.z += cooperative_test_lift;
      const std::map<std::string, std::pair<double, double>> receiver_joint_bounds{
        {"left_shoulder_pitch_joint", {-3.14, 1.57}},
        {"left_shoulder_roll_joint", {-0.38, 3.40}},
        {"left_shoulder_yaw_joint", {-2.66, 3.01}},
        {"left_elbow_joint", {-0.95, 3.18}},
        {"left_wrist_roll_joint", {-3.01, 2.75}},
        {"left_wrist_pitch_joint", {-0.4625, 0.4625}},
        {"left_wrist_yaw_joint", {-1.27, 1.27}},
      };
      const auto receiver_solution_gate = [
        &node, &receiver_joint_bounds, mesh_validated_segmented_transfer_profile](
        const std::map<std::string, double> & positions)
        {
          double minimum_margin = std::numeric_limits<double>::infinity();
          for (const auto & [joint_name, bounds] : receiver_joint_bounds) {
            const auto found = positions.find(joint_name);
            if (found == positions.end() || !std::isfinite(found->second)) {
              return false;
            }
            minimum_margin = std::min(
              minimum_margin,
              std::min(found->second - bounds.first, bounds.second - found->second));
          }
          const double shoulder_pitch = positions.at("left_shoulder_pitch_joint");
          const double shoulder_roll = positions.at("left_shoulder_roll_joint");
          const double shoulder_yaw = positions.at("left_shoulder_yaw_joint");
          const double elbow = positions.at("left_elbow_joint");
          const double wrist_roll = positions.at("left_wrist_roll_joint");
          const double wrist_pitch = positions.at("left_wrist_pitch_joint");
          const double wrist_yaw = positions.at("left_wrist_yaw_joint");
          const bool required_outer_wrist_branch =
            !mesh_validated_segmented_transfer_profile ||
            (wrist_pitch > 0.10 && wrist_pitch < 0.30);
          const bool accepted =
            shoulder_pitch > -1.45 && shoulder_pitch < -0.75 &&
            shoulder_roll > -0.10 && shoulder_roll < 1.20 &&
            shoulder_yaw > -0.15 && shoulder_yaw < 0.90 &&
            elbow > 1.90 && elbow < 3.00 &&
            wrist_yaw > -0.75 && wrist_yaw < 0.50 &&
            required_outer_wrist_branch && minimum_margin >= 0.05;
          RCLCPP_INFO(
            node->get_logger(),
            "Receiver IK signature: shoulder [pitch %.3f, roll %.3f, yaw %.3f], "
            "elbow %.3f, wrist [roll %.3f, pitch %.3f, yaw %.3f], "
            "minimum bound margin %.1f mrad (%s).",
            shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
            wrist_roll, wrist_pitch, wrist_yaw, minimum_margin * 1000.0,
            accepted ? "accepted" : "rejected");
          return accepted;
        };
      const auto receiver_branch_gate = [
        &node, &receiver_joint_bounds, mesh_validated_segmented_transfer_profile](
        const trajectory_msgs::msg::JointTrajectory & trajectory)
        {
          if (trajectory.points.size() < 2) {
            return false;
          }
          std::map<std::string, size_t> indices;
          for (const auto & [joint_name, unused] : receiver_joint_bounds) {
            static_cast<void>(unused);
            const auto found = std::find(
              trajectory.joint_names.begin(), trajectory.joint_names.end(), joint_name);
            if (found == trajectory.joint_names.end()) {
              return false;
            }
            indices.emplace(
              joint_name,
              static_cast<size_t>(std::distance(trajectory.joint_names.begin(), found)));
          }
          double minimum_margin = std::numeric_limits<double>::infinity();
          std::string closest_joint;
          size_t closest_point_index = 0;
          double closest_joint_position = 0.0;
          double closest_lower_bound = 0.0;
          double closest_upper_bound = 0.0;
          double maximum_joint_step = 0.0;
          double maximum_shoulder_pitch = -std::numeric_limits<double>::infinity();
          double minimum_shoulder_roll = std::numeric_limits<double>::infinity();
          double maximum_shoulder_roll = -std::numeric_limits<double>::infinity();
          double minimum_shoulder_yaw = std::numeric_limits<double>::infinity();
          double maximum_shoulder_yaw = -std::numeric_limits<double>::infinity();
          double minimum_elbow = std::numeric_limits<double>::infinity();
          double minimum_wrist_yaw = std::numeric_limits<double>::infinity();
          double maximum_wrist_yaw = -std::numeric_limits<double>::infinity();
          const double maximum_allowed_shoulder_pitch =
            mesh_validated_segmented_transfer_profile ? -0.70 : -0.75;
          const double minimum_allowed_elbow =
            mesh_validated_segmented_transfer_profile ? 2.0 : 1.8;
          const double required_joint_bound_margin =
            mesh_validated_segmented_transfer_profile ? 0.050 : 0.005;
          bool high_elbow_signature_preserved = true;
          for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
            const auto & point = trajectory.points[point_index];
            if (point.positions.size() != trajectory.joint_names.size()) {
              return false;
            }
            for (const auto & [joint_name, bounds] : receiver_joint_bounds) {
              const size_t joint_index = indices.at(joint_name);
              const double position = point.positions[joint_index];
              if (!std::isfinite(position)) {
                return false;
              }
              const double margin = std::min(
                position - bounds.first, bounds.second - position);
              if (margin < minimum_margin) {
                minimum_margin = margin;
                closest_joint = joint_name;
                closest_point_index = point_index;
                closest_joint_position = position;
                closest_lower_bound = bounds.first;
                closest_upper_bound = bounds.second;
              }
              if (point_index > 0) {
                maximum_joint_step = std::max(
                  maximum_joint_step,
                  std::abs(position - trajectory.points[point_index - 1].positions[joint_index]));
              }
            }
            const double shoulder_pitch =
              point.positions[indices.at("left_shoulder_pitch_joint")];
            const double shoulder_roll =
              point.positions[indices.at("left_shoulder_roll_joint")];
            const double shoulder_yaw =
              point.positions[indices.at("left_shoulder_yaw_joint")];
            const double elbow = point.positions[indices.at("left_elbow_joint")];
            const double wrist_yaw = point.positions[indices.at("left_wrist_yaw_joint")];
            maximum_shoulder_pitch = std::max(maximum_shoulder_pitch, shoulder_pitch);
            minimum_shoulder_roll = std::min(minimum_shoulder_roll, shoulder_roll);
            maximum_shoulder_roll = std::max(maximum_shoulder_roll, shoulder_roll);
            minimum_shoulder_yaw = std::min(minimum_shoulder_yaw, shoulder_yaw);
            maximum_shoulder_yaw = std::max(maximum_shoulder_yaw, shoulder_yaw);
            minimum_elbow = std::min(minimum_elbow, elbow);
            minimum_wrist_yaw = std::min(minimum_wrist_yaw, wrist_yaw);
            maximum_wrist_yaw = std::max(maximum_wrist_yaw, wrist_yaw);
            high_elbow_signature_preserved = high_elbow_signature_preserved &&
              shoulder_pitch < maximum_allowed_shoulder_pitch &&
              shoulder_roll > -0.30 && shoulder_roll < 1.25 &&
              shoulder_yaw > -0.35 && shoulder_yaw < 1.00 &&
              elbow > minimum_allowed_elbow &&
              wrist_yaw > -0.75 && wrist_yaw < 0.50;
          }
          const auto & endpoint = trajectory.points.back();
          const double shoulder_pitch =
            endpoint.positions[indices.at("left_shoulder_pitch_joint")];
          const double elbow = endpoint.positions[indices.at("left_elbow_joint")];
          const double wrist_yaw = endpoint.positions[indices.at("left_wrist_yaw_joint")];
          const bool accepted = high_elbow_signature_preserved &&
            minimum_margin >= required_joint_bound_margin && maximum_joint_step <= 0.35;
          const double closest_progress = trajectory.points.size() > 1 ?
            static_cast<double>(closest_point_index) /
            static_cast<double>(trajectory.points.size() - 1) : 0.0;
          RCLCPP_INFO(
            node->get_logger(),
            "Receiver branch path: %zu points, endpoint [%.3f, %.3f, %.3f], "
            "ranges [shoulder pitch <= %.3f, roll %.3f..%.3f, yaw %.3f..%.3f, "
            "elbow >= %.3f, wrist yaw %.3f..%.3f], "
            "minimum bound margin %.1f mrad at %s point %zu/%zu (%.1f%%) "
            "(q=%.6f, bounds %.6f..%.6f), maximum joint step %.3f rad "
            "(gates: shoulder pitch < %.3f, elbow > %.3f, margin >= %.1f mrad; %s).",
            trajectory.points.size(), shoulder_pitch, elbow, wrist_yaw,
            maximum_shoulder_pitch, minimum_shoulder_roll, maximum_shoulder_roll,
            minimum_shoulder_yaw, maximum_shoulder_yaw, minimum_elbow,
            minimum_wrist_yaw, maximum_wrist_yaw,
            minimum_margin * 1000.0, closest_joint.c_str(), closest_point_index + 1,
            trajectory.points.size(), closest_progress * 100.0, closest_joint_position,
            closest_lower_bound, closest_upper_bound, maximum_joint_step,
            maximum_allowed_shoulder_pitch, minimum_allowed_elbow,
            required_joint_bound_margin * 1000.0,
            accepted ? "accepted" : "rejected");
          return accepted;
        };
      const auto receiver_joint_plan_gate = [&node](
        const trajectory_msgs::msg::JointTrajectory & trajectory)
        {
          if (trajectory.points.size() < 2 || trajectory.joint_names.empty()) {
            return false;
          }
          const auto roll_found = std::find(
            trajectory.joint_names.begin(), trajectory.joint_names.end(),
            "left_shoulder_roll_joint");
          const auto yaw_found = std::find(
            trajectory.joint_names.begin(), trajectory.joint_names.end(),
            "left_shoulder_yaw_joint");
          if (roll_found == trajectory.joint_names.end() ||
            yaw_found == trajectory.joint_names.end())
          {
            return false;
          }
          const size_t roll_index = static_cast<size_t>(
            std::distance(trajectory.joint_names.begin(), roll_found));
          const size_t yaw_index = static_cast<size_t>(
            std::distance(trajectory.joint_names.begin(), yaw_found));
          double minimum_roll = std::numeric_limits<double>::infinity();
          double maximum_roll = -std::numeric_limits<double>::infinity();
          double minimum_yaw = std::numeric_limits<double>::infinity();
          double maximum_yaw = -std::numeric_limits<double>::infinity();
          double maximum_joint_step = 0.0;
          bool outward_path = true;
          for (size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
            const auto & point = trajectory.points[point_index];
            if (point.positions.size() != trajectory.joint_names.size()) {
              return false;
            }
            const double roll = point.positions[roll_index];
            const double yaw = point.positions[yaw_index];
            if (!std::isfinite(roll) || !std::isfinite(yaw)) {
              return false;
            }
            minimum_roll = std::min(minimum_roll, roll);
            maximum_roll = std::max(maximum_roll, roll);
            minimum_yaw = std::min(minimum_yaw, yaw);
            maximum_yaw = std::max(maximum_yaw, yaw);
            outward_path = outward_path && roll > -0.10 && roll < 1.25 &&
              yaw > -0.15 && yaw < 1.00;
            if (point_index > 0) {
              for (size_t joint_index = 0;
                joint_index < trajectory.joint_names.size(); ++joint_index)
              {
                maximum_joint_step = std::max(
                  maximum_joint_step,
                  std::abs(
                    point.positions[joint_index] -
                    trajectory.points[point_index - 1].positions[joint_index]));
              }
            }
          }
          const bool direct_path = verify_direct_joint_path(
            trajectory, "receiver outward empty-hand joint plan", 0.35, 0.70);
          const bool accepted = outward_path && maximum_joint_step <= 0.10 && direct_path;
          RCLCPP_INFO(
            node->get_logger(),
            "Receiver empty-hand shoulder path: roll %.3f..%.3f, yaw %.3f..%.3f, "
            "maximum point step %.3f rad (%s).",
            minimum_roll, maximum_roll, minimum_yaw, maximum_yaw,
            maximum_joint_step, accepted ? "accepted" : "rejected");
          return accepted;
        };
      const std::vector<std::map<std::string, double>> receiver_outward_seeds{
        {
          // Positive-wrist redundant branch: offline FK/IK tracing keeps the
          // exact centered contact geometry while avoiding the negative wrist
          // pitch limit during the 80->50 degree outer rotation.
          {"left_shoulder_pitch_joint", -0.9829},
          {"left_shoulder_roll_joint", 0.2819},
          {"left_shoulder_yaw_joint", 0.7815},
          {"left_elbow_joint", 2.6991},
          {"left_wrist_roll_joint", 1.8226},
          {"left_wrist_pitch_joint", 0.2387},
          {"left_wrist_yaw_joint", 0.0564},
        },
        {
          {"left_shoulder_pitch_joint", -0.95972},
          {"left_shoulder_roll_joint", 0.30395},
          {"left_shoulder_yaw_joint", 0.85197},
          {"left_elbow_joint", 2.70055},
          {"left_wrist_roll_joint", 1.88482},
          {"left_wrist_pitch_joint", 0.28653},
          {"left_wrist_yaw_joint", 0.04330},
        },
        {
          {"left_shoulder_pitch_joint", -1.02285},
          {"left_shoulder_roll_joint", 0.23500},
          {"left_shoulder_yaw_joint", 0.64661},
          {"left_elbow_joint", 2.69708},
          {"left_wrist_roll_joint", 1.70571},
          {"left_wrist_pitch_joint", 0.14485},
          {"left_wrist_yaw_joint", 0.07353},
        },
        {
          {"left_shoulder_pitch_joint", -1.05},
          {"left_shoulder_roll_joint", 0.25},
          {"left_shoulder_yaw_joint", 0.00},
          {"left_elbow_joint", 2.35},
          {"left_wrist_roll_joint", 0.70},
          {"left_wrist_pitch_joint", -0.20},
          {"left_wrist_yaw_joint", 0.00},
        },
        {
          {"left_shoulder_pitch_joint", -1.10},
          {"left_shoulder_roll_joint", 0.45},
          {"left_shoulder_yaw_joint", 0.25},
          {"left_elbow_joint", 2.45},
          {"left_wrist_roll_joint", 0.60},
          {"left_wrist_pitch_joint", -0.25},
          {"left_wrist_yaw_joint", -0.10},
        },
        {
          {"left_shoulder_pitch_joint", -1.20},
          {"left_shoulder_roll_joint", 0.65},
          {"left_shoulder_yaw_joint", 0.45},
          {"left_elbow_joint", 2.55},
          {"left_wrist_roll_joint", 0.40},
          {"left_wrist_pitch_joint", -0.20},
          {"left_wrist_yaw_joint", 0.10},
        },
        {
          {"left_shoulder_pitch_joint", -1.05},
          {"left_shoulder_roll_joint", 0.35},
          {"left_shoulder_yaw_joint", -0.10},
          {"left_elbow_joint", 2.40},
          {"left_wrist_roll_joint", 1.00},
          {"left_wrist_pitch_joint", -0.20},
          {"left_wrist_yaw_joint", -0.30},
        },
      };
      std::optional<std::map<std::string, double>> left_outer_joint_target;
      bool receiver_probe_attachment_commanded = false;
      if (success) {
        const auto touch_links = both_finger_touch_links();
        receiver_probe_attachment_commanded =
          both_arms.attachObject(kObjectId, "pelvis", touch_links);
        success = receiver_probe_attachment_commanded;
        owner = success ? Owner::Base : Owner::World;
        if (success) {
          success = wait_for_attached_object_with_touch_links(
            planning_scene, kObjectId, "pelvis", touch_links);
        }
      }
      if (success) {
        // Do not accept the first outer IK pose in isolation.  Trace every
        // candidate through the complete collision-checked rotate/contact/lift
        // continuation and select only a branch that preserves all gates.
        for (size_t seed_index = 0;
          seed_index < receiver_outward_seeds.size() &&
          !left_outer_joint_target.has_value(); ++seed_index)
        {
          auto candidate = compute_seeded_ik_joint_target(
            node, left_arm, ik_client, tf_buffer, left_outer_staging, "left_dex1_tcp",
            receiver_outward_seeds[seed_index],
            "cooperative transfer: outward receiver outer-staging IK seed " +
            std::to_string(seed_index + 1) + "/" +
            std::to_string(receiver_outward_seeds.size()),
            receiver_solution_gate);
          if (!candidate.has_value()) {
            continue;
          }
          std::map<std::string, double> candidate_lift_target;
          const bool complete_branch_passed =
            plan_and_execute_joint_with_cartesian_continuation(
            left_arm, *candidate, left_pregrasp,
            "cooperative transfer: receiver full-branch preflight seed " +
            std::to_string(seed_index + 1),
            {left_receiver_rotated_entry, left_grasp, left_lift_probe},
            receiver_branch_gate, receiver_joint_plan_gate, false, true,
            &candidate_lift_target);
          if (complete_branch_passed) {
            left_outer_joint_target = std::move(candidate);
            receiver_lift_preflight_target = std::move(candidate_lift_target);
          }
        }
        success = left_outer_joint_target.has_value();
      }
      if (success && !transfer_preflight_only) {
        success = plan_and_execute_joint_with_cartesian_continuation(
          left_arm, *left_outer_joint_target, left_pregrasp,
          "cooperative transfer: seeded outward receiver staging",
          {left_receiver_rotated_entry, left_grasp, left_lift_probe}, receiver_branch_gate,
          receiver_joint_plan_gate, true, true);
      }
      bool receiver_probe_scene_restored = !receiver_probe_attachment_commanded;
      if (receiver_probe_attachment_commanded) {
        receiver_probe_scene_restored = both_arms.detachObject(kObjectId);
        owner = receiver_probe_scene_restored ? Owner::World : Owner::Base;
        if (receiver_probe_scene_restored) {
          receiver_probe_scene_restored = wait_for_world_object(planning_scene, kObjectId);
        }
      }
      if (!receiver_probe_scene_restored) {
        RCLCPP_ERROR(
          node->get_logger(),
          "The receiver probe planning object was not restored to World; refusing to move.");
        success = false;
      }
      if (success && center_regrasp_preflight_only) {
        success = donor_lift_preflight_target.has_value() &&
          receiver_lift_preflight_target.has_value() &&
          gazebo_initial_object_pose.has_value() && gazebo_table_surface_z.has_value();
        if (!success) {
          RCLCPP_ERROR(
            node->get_logger(),
            "Center-regrasp preflight lacks a complete donor/receiver 50 mm lift endpoint.");
        }
        const auto initial_planning_object = success ?
          transform_pose_to_planning_frame(*gazebo_initial_object_pose, tf_buffer) :
          std::optional<geometry_msgs::msg::Pose>{};
        success = success && initial_planning_object.has_value();
        auto airborne_planning_object = initial_planning_object.value_or(
          geometry_msgs::msg::Pose{});
        airborne_planning_object.position.z += cooperative_test_lift;
        auto airborne_detected_object = pickup_object;
        airborne_detected_object.position.z += cooperative_test_lift;

        bool airborne_attachment_commanded = false;
        if (success) {
          success = planning_scene.applyCollisionObject(make_collision_object(
              airborne_planning_object, object_size_x, object_size_y, object_size_z));
        }
        if (success) {
          success = wait_for_world_object(planning_scene, kObjectId);
        }
        if (success) {
          const auto touch_links = both_finger_touch_links();
          airborne_attachment_commanded = both_arms.attachObject(
            kObjectId, "pelvis", touch_links);
          success = airborne_attachment_commanded &&
            wait_for_attached_object_with_touch_links(
            planning_scene, kObjectId, "pelvis", touch_links);
          owner = success ? Owner::Base : Owner::World;
        }

        std::optional<CenterCandidateResult> selected_center_candidate;
        if (success) {
          auto combined_lift_state = both_arms.getCurrentState(10.0);
          success = combined_lift_state != nullptr;
          if (success) {
            for (const auto & [joint_name, position] : *donor_lift_preflight_target) {
              combined_lift_state->setVariablePosition(joint_name, position);
            }
            for (const auto & [joint_name, position] : *receiver_lift_preflight_target) {
              combined_lift_state->setVariablePosition(joint_name, position);
            }
            combined_lift_state->setVariablePosition(
              "right_dex1_finger_1_joint", preload);
            combined_lift_state->setVariablePosition(
              "right_dex1_finger_2_joint", preload);
            combined_lift_state->setVariablePosition(
              "left_dex1_finger_1_joint", -0.019);
            combined_lift_state->setVariablePosition(
              "left_dex1_finger_2_joint", -0.019);
            combined_lift_state->update();
            success = combined_lift_state->satisfiesBounds(1e-6) &&
              full_robot_state_is_valid(
              validity_client, *combined_lift_state,
              "center-regrasp combined dual-lift start");
          }
          if (success) {
            ::planning_scene::PlanningScene geometric_scene(
              both_arms.getRobotModel());
            selected_center_candidate = search_center_regrasp_candidates(
              left_arm, *combined_lift_state, geometric_scene,
              validity_client, airborne_detected_object,
              *gazebo_table_surface_z, center_regrasp_z_sweep_only,
              center_regrasp_search_tilt_degrees * 0.017453292519943295);
            success = selected_center_candidate.has_value();
          }
        }

        bool table_scene_restored = !airborne_attachment_commanded;
        if (airborne_attachment_commanded) {
          table_scene_restored = both_arms.detachObject(kObjectId);
          owner = table_scene_restored ? Owner::World : Owner::Base;
          if (table_scene_restored) {
            table_scene_restored = wait_for_world_object(planning_scene, kObjectId);
          }
        }
        if (table_scene_restored && initial_planning_object.has_value()) {
          table_scene_restored = planning_scene.applyCollisionObject(make_collision_object(
              *initial_planning_object, object_size_x, object_size_y, object_size_z)) &&
            wait_for_world_object(planning_scene, kObjectId);
        }
        success = success && table_scene_restored;
        if (success) {
          RCLCPP_INFO(
            node->get_logger(),
            "Center-regrasp offline preflight passed; the Gazebo block and robot were not "
            "moved, and the planning object was restored to its measured table pose.");
        } else {
          RCLCPP_ERROR(
            node->get_logger(),
            "Center-regrasp offline preflight failed closed; no robot trajectory was executed.");
        }
        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();
        return success ? 0 : 1;
      }
      if (success && receiver_preflight_only) {
        RCLCPP_INFO(
          node->get_logger(),
          "Receiver preflight passed from parked state through high-clearance staging, "
          "centered contact and the complete 5 cm lift preview; no arm trajectory was executed.");
        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();
        return 0;
      }
    } else if (success) {
      success = plan_and_execute_pose(
        left_arm, left_pregrasp, "cooperative test: left pregrasp");
    }
    if (success && cooperative_transfer_mode) {
      success = verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, *gazebo_initial_object_pose,
        0.005, 3.0 * 0.017453292519943295,
        "both arms reached collision-aware pregrasp poses");
      cooperative_both_pregrasp_reached = success;
    }
    if (success && cooperative_transfer_mode) {
      // Pelvis is fixed in this simulation, so this attachment keeps the
      // planning object stationary while enabling only intentional finger
      // contact during the two straight final approaches.
      const auto touch_links = both_finger_touch_links();
      success = both_arms.attachObject(kObjectId, "pelvis", touch_links);
      owner = success ? Owner::Base : Owner::World;
      if (success) {
        success = wait_for_attached_object_with_touch_links(
          planning_scene, kObjectId, "pelvis", touch_links);
      }
    }
    if (success) {
      cooperative_right_final_entry_attempted = cooperative_transfer_mode;
      success = plan_and_execute_cartesian(
        right_arm, right_grasp, "cooperative test: right enters from outside",
        false, donor_branch_gate);
      cooperative_right_contact_reached = cooperative_transfer_mode && success;
    }
    if (!success && cooperative_right_final_entry_attempted) {
      diagnose_cartesian_collision(
        right_arm, right_grasp, validity_client,
        "cooperative donor final entry with receiver at pregrasp");
    }
    if (success && cooperative_transfer_mode) {
      success = verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, *gazebo_initial_object_pose,
        0.005, 3.0 * 0.017453292519943295,
        "right donor reached contact pose");
    }
    const double required_receiver_entry_margin =
      mesh_validated_segmented_transfer_profile ? 0.050 : 0.005;
    const auto receiver_entry_margin_gate = [
      &left_arm, required_receiver_entry_margin](
        const trajectory_msgs::msg::JointTrajectory & trajectory)
      {
        return verify_trajectory_joint_bound_margin(
          trajectory, left_arm.getRobotModel(), required_receiver_entry_margin,
          "receiver segmented final entry");
      };
    if (success && cooperative_transfer_mode) {
      success = plan_and_execute_cartesian(
        left_arm, left_grasp,
        "cooperative test: left rotates outside then enters straight", false,
        receiver_entry_margin_gate, {left_receiver_rotated_entry});
    } else if (success) {
      success = plan_and_execute_cartesian(
        left_arm, left_grasp, "cooperative test: left enters from outside");
    }
    if (success && cooperative_transfer_mode) {
      success = verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, *gazebo_initial_object_pose,
        0.005, 3.0 * 0.017453292519943295,
        "left receiver reached contact pose");
    }
    if (success && cooperative_transfer_mode) {
      success = both_arms.detachObject(kObjectId);
      owner = success ? Owner::World : Owner::Base;
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
    }
    if (success) {
      success = wait_for_sim_duration(node, 0.20, "cooperative table contact settle");
    }
    if (success) {
      auto scene_object_pose = pickup_object;
      if (cooperative_transfer_mode) {
        const auto gazebo_pose = gazebo_pose_monitor.latest();
        success = gazebo_pose.has_value();
        if (success) {
          const auto transformed = transform_pose_to_planning_frame(
            gazebo_pose->pose, tf_buffer);
          success = transformed.has_value();
          if (success) {
            scene_object_pose = *transformed;
          }
        }
      }
      auto object = make_collision_object(
        scene_object_pose, object_size_x, object_size_y, object_size_z);
      std_msgs::msg::ColorRGBA color;
      color.r = 0.10F;
      color.g = 0.95F;
      color.b = 0.15F;
      color.a = 1.0F;
      success = planning_scene.applyCollisionObject(object, color);
      owner = success ? Owner::World : Owner::None;
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
    }
    if (success) {
      const auto touch_links = cooperative_transfer_mode ?
        both_finger_touch_links() : both_hand_touch_links();
      success = right_arm.attachObject(kObjectId, "right_dex1_tcp", touch_links);
      owner = success ? Owner::Right : Owner::World;
      if (success) {
        success = cooperative_transfer_mode ?
          wait_for_attached_object_with_touch_links(
            planning_scene, kObjectId, "right_dex1_tcp", touch_links) :
          wait_for_attached_object(
            planning_scene, kObjectId, "right_dex1_tcp");
      }
    }
    if (success) {
      success = plan_and_execute_joint(
        right_gripper, right_soft, "cooperative test: right soft contact");
    }
    if (success) {
      success = plan_and_execute_joint(
        left_gripper, left_soft, "cooperative test: left soft contact");
    }
    if (success) {
      success = wait_for_sim_duration(node, 0.20, "cooperative soft contact settle");
    }
    if (success) {
      const bool right_feedback_ok = verify_gripper_contact_feedback(
        right_gripper, "right", soft_contact, object_size_x, false,
        "cooperative right soft contact");
      const bool left_feedback_ok = verify_gripper_contact_feedback(
        left_gripper, "left", soft_contact, object_size_x, false,
        "cooperative left soft contact");
      success = right_feedback_ok && left_feedback_ok;
    }
    if (success) {
      success = plan_and_execute_joint(
        right_gripper, right_preload, "cooperative test: right preload");
      right_closed = success;
    }
    if (success) {
      success = plan_and_execute_joint(
        left_gripper,
        cooperative_transfer_mode ? left_transfer_preload : left_preload,
        cooperative_transfer_mode ?
        "cooperative transfer: central receiver preload" :
        "cooperative test: left preload");
      left_closed = success;
    }
    if (success) {
      success = wait_for_sim_duration(node, 0.50, "cooperative preload settle");
    }
    if (success) {
      const bool right_contact_ok = verify_gripper_contact_feedback(
        right_gripper, "right", preload, object_size_x, true,
        "cooperative right preload");
      const bool left_contact_ok = verify_gripper_contact_feedback(
        left_gripper, "left",
        cooperative_transfer_mode ? transfer_receiver_preload : preload,
        object_size_x, true, "cooperative left preload");
      success = right_contact_ok && left_contact_ok;
    }

    moveit_msgs::msg::RobotTrajectory synchronized_lift;
    if (success) {
      // Derive both measured TCP poses from the same state inside the builder,
      // then offset them in pelvis/world-aligned Z.  This preserves their
      // actual relative transform after the pads have settled on the block.
      success = build_synchronized_cartesian_trajectory(
        right_arm, left_arm, tf_buffer, validity_client,
        *synchronized_motion_limits,
        cooperative_test_lift, cooperative_motion_duration,
        cooperative_transfer_mode ? transfer_motion_velocity_scaling : velocity_scaling,
        cooperative_transfer_mode ? transfer_motion_acceleration_scaling : acceleration_scaling,
        "cooperative table lift", synchronized_lift);
    }
    if (success) {
      success = execute_synchronized_trajectory(
        both_arms, synchronized_lift, "cooperative table lift");
    }
    if (success) {
      RCLCPP_INFO(
        node->get_logger(),
        "Both grippers lifted the central block %.1f mm; holding for %.1f seconds.",
        cooperative_test_lift * 1000.0, receiver_hold_seconds);
      success = wait_for_sim_duration(
        node, receiver_hold_seconds, "cooperative airborne hold");
    }
    if (success && cooperative_lift_mode) {
      const auto synchronized_lower = reverse_synchronized_trajectory(
        synchronized_lift);
      success = execute_synchronized_trajectory(
        both_arms, synchronized_lower, "cooperative return to table");
    }
    if (success && cooperative_lift_mode) {
      success = wait_for_sim_duration(node, 0.40, "cooperative table settle");
    }
    if (success && cooperative_lift_mode) {
      success = plan_and_execute_joint(
        right_gripper, right_soft, "cooperative test: right unloads");
    }
    if (success && cooperative_lift_mode) {
      success = plan_and_execute_joint(
        left_gripper, left_soft, "cooperative test: left unloads");
    }
    if (success && cooperative_lift_mode) {
      success = plan_and_execute_joint(
        right_gripper, right_open, "cooperative test: right opens");
      right_closed = !success;
    }
    if (success && cooperative_lift_mode) {
      success = plan_and_execute_joint(
        left_gripper, left_open, "cooperative test: left opens");
      left_closed = !success;
    }
    if (success && cooperative_lift_mode) {
      success = wait_for_sim_duration(node, 0.20, "cooperative release settle");
    }
    if (success && cooperative_lift_mode) {
      success = right_arm.detachObject(kObjectId);
      owner = success ? Owner::World : Owner::Right;
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
    }
    if (success && cooperative_lift_mode) {
      success = plan_and_execute_cartesian(
        right_arm, right_pregrasp, "cooperative test: right retreats outward");
    }
    if (success && cooperative_lift_mode) {
      success = plan_and_execute_cartesian(
        left_arm, left_pregrasp, "cooperative test: left retreats outward");
    }
    if (success && cooperative_lift_mode) {
      success = plan_and_execute_joint(right_arm, right_park, "cooperative test: park right");
    }
    if (success && cooperative_lift_mode) {
      success = plan_and_execute_joint(left_arm, left_park, "cooperative test: park left");
    }
    if (success && cooperative_lift_mode) {
      planning_scene.removeCollisionObjects({kObjectId});
      success = wait_for_object_absent(planning_scene, kObjectId);
      owner = success ? Owner::None : Owner::World;
    }

    std::optional<ObjectStabilityReport> receiver_report;
    std::optional<geometry_msgs::msg::Pose> airborne_anchor_pose;
    std::optional<geometry_msgs::msg::Pose> expected_table_pose;
    if (success && cooperative_transfer_mode) {
      const double minimum_air_clearance = std::max(
        0.0005, cooperative_test_lift - transfer_gate_air_height_tolerance);
      const double maximum_air_clearance =
        cooperative_test_lift + transfer_gate_air_height_tolerance;
      receiver_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.20, transfer_gate_observation,
        minimum_air_clearance, maximum_air_clearance,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        5.0 * 0.017453292519943295,
        "dual-hand asymmetric lift");
      airborne_anchor_pose = *gazebo_initial_object_pose;
      airborne_anchor_pose->position.z += cooperative_test_lift;
      const bool lift_matches_command = receiver_report.has_value() &&
        verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, *airborne_anchor_pose,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "dual-hand lift versus commanded world pose");
      success = lift_matches_command;
    }

    if (success && cooperative_transfer_mode && cooperative_transfer_lift_only) {
      RCLCPP_INFO(
        node->get_logger(),
        "Centered dual-hand lift-only mode: returning the verified block to the table "
        "before any donor unloading or single-hand off-center support.");
      const auto synchronized_lower = reverse_synchronized_trajectory(synchronized_lift);
      success = execute_synchronized_trajectory(
        both_arms, synchronized_lower, "centered transfer demo: synchronized return to table");
      if (success) {
        success = wait_for_sim_duration(node, 0.40, "centered transfer demo table settle");
      }
      if (success) {
        const auto table_return_report = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, *gazebo_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.10, transfer_gate_observation,
          -0.002, 0.003,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          5.0 * 0.017453292519943295,
          "centered transfer demo returned load to table before unloading");
        success = table_return_report.has_value();
      }
      if (success) {
        success = verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, *gazebo_initial_object_pose,
          0.010, 5.0 * 0.017453292519943295,
          "centered transfer demo table return versus initial pose");
      }
      if (success) {
        success = plan_and_execute_joint(
          right_gripper, right_soft, "centered transfer demo: right unloads");
      }
      if (success) {
        success = plan_and_execute_joint(
          left_gripper, left_soft, "centered transfer demo: left unloads");
      }
      if (success) {
        const auto unloaded_table_report = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, *gazebo_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.10, transfer_gate_observation,
          -0.002, 0.003,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          5.0 * 0.017453292519943295,
          "centered transfer demo unloaded block remains on table");
        success = unloaded_table_report.has_value();
      }
      if (success) {
        success = plan_and_execute_joint(
          right_gripper, right_open, "centered transfer demo: right opens");
        if (success) {
          success = gripper_open_target_is_reached(
            right_gripper, right_open, 0.0005,
            "centered transfer demo right-open feedback");
        }
        if (!success) {
          RCLCPP_WARN(
            node->get_logger(),
            "Right open feedback missed the 0.5 mm gate; reasserting the same "
            "table-supported open target once.");
          success = plan_and_execute_joint(
            right_gripper, right_open,
            "centered transfer demo: reassert right open");
          if (success) {
            success = gripper_open_target_is_reached(
              right_gripper, right_open, 0.0005,
              "centered transfer demo right-open feedback retry");
          }
        }
        right_closed = !success;
      }
      if (success) {
        success = plan_and_execute_joint(
          left_gripper, left_open, "centered transfer demo: left opens");
        if (success) {
          success = gripper_open_target_is_reached(
            left_gripper, left_open, 0.0005,
            "centered transfer demo left-open feedback");
        }
        if (!success) {
          RCLCPP_WARN(
            node->get_logger(),
            "Left open feedback missed the 0.5 mm gate; reasserting the same "
            "table-supported open target once.");
          success = plan_and_execute_joint(
            left_gripper, left_open,
            "centered transfer demo: reassert left open");
          if (success) {
            success = gripper_open_target_is_reached(
              left_gripper, left_open, 0.0005,
              "centered transfer demo left-open feedback retry");
          }
        }
        left_closed = !success;
      }
      if (success) {
        success = wait_for_sim_duration(node, 0.20, "centered transfer demo release settle");
      }
      if (success) {
        const auto released_table_report = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, *gazebo_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.10, transfer_gate_observation,
          -0.002, 0.003,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          5.0 * 0.017453292519943295,
          "centered transfer demo physically released block on table");
        success = released_table_report.has_value();
      }
      if (success) {
        bool world_restored = right_arm.detachObject(kObjectId) &&
          wait_for_world_object(planning_scene, kObjectId);
        if (!world_restored) {
          RCLCPP_WARN(
            node->get_logger(),
            "Both grippers are confirmed open, but the first planning-scene detach was not "
            "observable; retrying that scene-only operation once without moving either arm.");
          world_restored = right_arm.detachObject(kObjectId) &&
            wait_for_world_object(planning_scene, kObjectId);
        }
        success = world_restored;
        owner = world_restored ? Owner::World : Owner::Right;
      }
      if (success) {
        success = verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, *gazebo_initial_object_pose,
          0.010, 5.0 * 0.017453292519943295,
          "centered transfer demo returned block to table");
      }
      if (success) {
        success = plan_and_execute_cartesian(
          right_arm, right_pregrasp,
          "centered transfer demo: right retreats from released block", false,
          donor_branch_gate);
      }
      if (success) {
        success = plan_and_execute_cartesian(
          left_arm, left_pregrasp,
          "centered transfer demo: left reverses straight entry and outer rotation", false,
          receiver_entry_margin_gate, {left_receiver_rotated_entry}, false);
      }
      if (success) {
        success = plan_and_execute_cartesian(
          left_arm, left_outer_staging,
          "centered transfer demo: left moves to outer staging", false,
          receiver_entry_margin_gate);
      }
      if (success) {
        success = plan_and_execute_joint(
          right_arm, right_park, "centered transfer demo: park right");
      }
      if (success) {
        success = joint_target_is_reached(
          right_arm, right_park, 0.01,
          "centered transfer demo parked right feedback");
      }
      if (success) {
        planning_scene.removeCollisionObjects({kObjectId});
        success = wait_for_object_absent(planning_scene, kObjectId);
        owner = success ? Owner::None : Owner::World;
      }
      if (success) {
        RCLCPP_INFO(
          node->get_logger(),
          "Centered dual-arm 50 mm lift, airborne hold and synchronized return completed; "
          "the donor-release phase was intentionally skipped and the empty receiver remains "
          "at its verified outer staging pose.");
      } else {
        RCLCPP_ERROR(
          node->get_logger(),
          "Centered lift-only cleanup stopped at its first failed physical gate; "
          "single-hand transfer was not attempted.");
      }
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return success ? 0 : 1;
    }

    // Synchronize MoveIt's collision object to the measured Gazebo pose before
    // changing the kinematic owner.  The object remains physically held by
    // both grippers throughout this scene-only Right -> World -> Left swap.
    if (success && cooperative_transfer_mode) {
      const auto planning_object_pose = transform_pose_to_planning_frame(
        receiver_report->final_sample.pose, tf_buffer);
      success = planning_object_pose.has_value();
      if (success) {
        success = right_arm.detachObject(kObjectId);
        owner = success ? Owner::World : Owner::Right;
      }
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
      if (success) {
        success = planning_scene.applyCollisionObject(make_collision_object(
            *planning_object_pose, object_size_x, object_size_y, object_size_z));
      }
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
      if (success) {
        const auto touch_links = both_finger_touch_links();
        success = left_arm.attachObject(
          kObjectId, "left_dex1_tcp", touch_links);
        owner = success ? Owner::Left : Owner::World;
        if (success) {
          success = wait_for_attached_object_with_touch_links(
            planning_scene, kObjectId, "left_dex1_tcp", touch_links);
        }
      }
    }
    if (success && cooperative_transfer_mode) {
      success = plan_and_execute_joint(
        right_gripper, right_soft, "cooperative transfer: donor unloads to soft contact");
    }
    if (success && cooperative_transfer_mode) {
      success = wait_for_sim_duration(node, 0.30, "donor soft-contact settle");
    }
    // Separate receiver load capacity from the final donor-pad disengagement.
    // If the block has already rolled while the donor remains at soft contact,
    // increasing the opening clearance cannot fix it; restore donor preload and
    // stop before introducing the disengagement transient.
    if (success && cooperative_transfer_mode) {
      const double minimum_air_clearance = std::max(
        0.0005, cooperative_test_lift - transfer_gate_air_height_tolerance);
      const auto soft_contact_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.0, transfer_gate_observation,
        minimum_air_clearance, cooperative_test_lift + transfer_gate_air_height_tolerance,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "receiver support with donor at soft contact");
      const bool soft_contact_matches_lift = soft_contact_report.has_value() &&
        verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, receiver_report->final_sample.pose,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "donor soft-contact versus dual-hand lift") &&
        verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, *airborne_anchor_pose,
          transfer_gate_max_anchor_displacement,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "donor soft-contact versus airborne anchor");
      if (!soft_contact_matches_lift) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Receiver support became unstable at donor soft contact; restoring donor preload "
          "and stopping before additional finger opening.");
        plan_and_execute_joint(
          right_gripper, right_preload,
          "cooperative transfer recovery: restore donor preload after soft-contact gate");
        success = false;
      } else {
        receiver_report = soft_contact_report;
      }
    }
    if (success && cooperative_transfer_mode) {
      // Remove only the residual preload at the contact pose.  Opening to
      // 1 mm while the loaded donor pad is still geometrically trapped can
      // inject a roll impulse, so the arm peels away by 2 mm before the first
      // positive finger clearance is commanded below.
      for (int clearance_mm = 0; clearance_mm <= 0; ++clearance_mm) {
        const double finger_position = -static_cast<double>(clearance_mm) / 1000.0;
        const std::string step_name =
          "donor opens to " + std::to_string(clearance_mm) + " mm clearance";
        const std::map<std::string, double> finger_target{
          {"right_dex1_finger_1_joint", finger_position},
          {"right_dex1_finger_2_joint", finger_position},
        };
        success = plan_and_execute_joint(
          right_gripper, finger_target, "cooperative transfer: " + step_name);
        if (!success) {
          break;
        }
        const double minimum_air_clearance = std::max(
          0.0005, cooperative_test_lift - transfer_gate_air_height_tolerance);
        const auto step_report = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, *gazebo_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.30, transfer_gate_observation,
          minimum_air_clearance, cooperative_test_lift + transfer_gate_air_height_tolerance,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          transfer_gate_max_tilt_degrees * 0.017453292519943295,
          "receiver support after " + step_name);
        const bool step_matches_previous = step_report.has_value() &&
          verify_gazebo_pose_near_reference(
            node, gazebo_pose_monitor, receiver_report->final_sample.pose,
            transfer_gate_max_drift,
            transfer_gate_max_angle_degrees * 0.017453292519943295,
            step_name + " versus previous support") &&
          verify_gazebo_pose_near_reference(
            node, gazebo_pose_monitor, *airborne_anchor_pose,
            transfer_gate_max_anchor_displacement,
            transfer_gate_max_angle_degrees * 0.017453292519943295,
            step_name + " versus airborne anchor");
        if (!step_matches_previous) {
          RCLCPP_ERROR(
            node->get_logger(),
            "Receiver support failed during staged donor opening; restoring donor preload.");
          plan_and_execute_joint(
            right_gripper, right_preload,
            "cooperative transfer recovery: restore donor preload after staged opening");
          success = false;
          break;
        }
        receiver_report = step_report;
      }
    }

    if (success && cooperative_transfer_mode) {
      // First peel the loaded pad away with a 2 mm arm withdrawal at zero
      // clearance, then alternate 1 mm-per-finger opening with 2 mm arm
      // increments.  Each normal and tangential release
      // step gets its own physical gate so a slip cannot be hidden between
      // commands.  Once the fingers reach 8 mm clearance, finish the 30 mm
      // short retreat with the original millimetre-scale 2/3/5 mm increments.
      struct DonorDisengagementStep
      {
        double retreat_increment;
        std::optional<double> finger_target_before_retreat;
      };
      const std::vector<DonorDisengagementStep> donor_disengagement_steps{
        {0.002, std::nullopt},
        {0.002, -0.001},
        {0.002, -0.002},
        {0.002, -0.003},
        {0.002, -0.004},
        {0.002, -0.005},
        {0.002, -0.006},
        {0.002, -0.007},
        {0.002, -0.008},
        {0.002, std::nullopt},
        {0.002, std::nullopt},
        {0.003, std::nullopt},
        {0.005, std::nullopt},
      };
      const double tilt = cooperative_inward_tilt_degrees * 0.017453292519943295;
      double cumulative_retreat = 0.0;
      int finger_clearance_mm = 0;

      const auto verify_disengagement_state =
        [&](const std::string & stage_name, const double settle_seconds) -> bool
        {
          const double minimum_air_clearance = std::max(
            0.0005, cooperative_test_lift - transfer_gate_air_height_tolerance);
          const auto step_report = verify_gazebo_object_stability(
            node, gazebo_pose_monitor, *gazebo_table_surface_z,
            object_size_x, object_size_y, object_size_z,
            settle_seconds, transfer_gate_observation,
            minimum_air_clearance, cooperative_test_lift + transfer_gate_air_height_tolerance,
            transfer_gate_max_drift,
            transfer_gate_max_angle_degrees * 0.017453292519943295,
            transfer_gate_max_tilt_degrees * 0.017453292519943295,
            "receiver support after " + stage_name);
          const bool matches_previous = step_report.has_value() &&
            verify_gazebo_pose_near_reference(
              node, gazebo_pose_monitor, receiver_report->final_sample.pose,
              transfer_gate_max_drift,
              transfer_gate_max_angle_degrees * 0.017453292519943295,
              stage_name + " versus previous support") &&
            verify_gazebo_pose_near_reference(
              node, gazebo_pose_monitor, *airborne_anchor_pose,
              transfer_gate_max_anchor_displacement,
              transfer_gate_max_angle_degrees * 0.017453292519943295,
              stage_name + " versus airborne anchor");
          if (matches_previous) {
            receiver_report = step_report;
          }
          return matches_previous;
        };

      for (const auto & step : donor_disengagement_steps) {
        if (step.finger_target_before_retreat.has_value()) {
          finger_clearance_mm = static_cast<int>(std::lround(
              -*step.finger_target_before_retreat * 1000.0));
          const std::string opening_name =
            "donor opens to " + std::to_string(finger_clearance_mm) +
            " mm clearance after " +
            std::to_string(static_cast<int>(std::lround(cumulative_retreat * 1000.0))) +
            " mm outward retreat";
          const std::map<std::string, double> finger_target{
            {"right_dex1_finger_1_joint", *step.finger_target_before_retreat},
            {"right_dex1_finger_2_joint", *step.finger_target_before_retreat},
          };
          const bool opening_executed = plan_and_execute_joint(
            right_gripper, finger_target, "cooperative transfer: " + opening_name);
          success = opening_executed;
          if (success) {
            success = joint_target_is_reached(
              right_gripper, finger_target, 0.0005, opening_name);
          }
          for (int retry = 1; opening_executed && !success && retry <= 3; ++retry) {
            RCLCPP_WARN(
              node->get_logger(),
              "Donor-opening feedback missed the unchanged 0.5 mm gate at '%s'; "
              "reasserting the identical target (%d/3).",
              opening_name.c_str(), retry);
            const bool reasserted = plan_and_execute_joint(
              right_gripper, finger_target,
              "cooperative transfer: reassert " + opening_name + " attempt " +
              std::to_string(retry));
            if (!reasserted) {
              success = false;
              break;
            }
            success = joint_target_is_reached(
              right_gripper, finger_target, 0.0005,
              opening_name + " retry " + std::to_string(retry));
          }
          if (success) {
            success = verify_disengagement_state(opening_name, 0.30);
          }
          if (!success) {
            RCLCPP_ERROR(
              node->get_logger(),
              "Receiver support failed during alternating donor opening; "
              "stopping without re-closing the displaced donor.");
            break;
          }
        }

        const auto current_right = current_link_pose_in_frame(
          right_arm, "right_dex1_tcp", kPlanningFrame, tf_buffer);
        if (!current_right.has_value()) {
          success = false;
          break;
        }
        auto retreat_target = *current_right;
        retreat_target.position.y -= std::cos(tilt) * step.retreat_increment;
        retreat_target.position.z += std::sin(tilt) * step.retreat_increment;
        cumulative_retreat += step.retreat_increment;
        const int retreat_mm =
          static_cast<int>(std::lround(cumulative_retreat * 1000.0));
        const std::string step_name =
          "donor retreats to " + std::to_string(retreat_mm) +
          " mm outward at " + std::to_string(finger_clearance_mm) +
          " mm finger clearance";
        success = plan_and_execute_cartesian(
          right_arm, retreat_target, "cooperative transfer: " + step_name,
          false, donor_branch_gate);
        if (!success) {
          break;
        }
        if (!verify_disengagement_state(step_name, 0.20)) {
          RCLCPP_ERROR(
            node->get_logger(),
            "Receiver support failed during staged donor retreat; stopping without "
            "closing the displaced donor onto the block.");
          success = false;
          break;
        }
      }
    }
    if (success && cooperative_transfer_mode) {
      // The donor pads are now 30 mm away from the block.  Complete the finger
      // opening out of contact so it cannot inject another unloading impulse.
      success = plan_and_execute_joint(
        right_gripper, right_open,
        "cooperative transfer: donor opens completely after physical retreat");
      if (success) {
        right_closed = false;
      }
    }
    if (success && cooperative_transfer_mode) {
      const double minimum_air_clearance = std::max(
        0.0005, cooperative_test_lift - transfer_gate_air_height_tolerance);
      const auto fully_open_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.20, transfer_gate_observation,
        minimum_air_clearance, cooperative_test_lift + transfer_gate_air_height_tolerance,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "receiver support after clear donor opens completely");
      const bool open_matches_retreat = fully_open_report.has_value() &&
        verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, receiver_report->final_sample.pose,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "clear donor opening versus verified retreat") &&
        verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, *airborne_anchor_pose,
          transfer_gate_max_anchor_displacement,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "clear donor opening versus airborne anchor");
      success = open_matches_retreat;
      if (success) {
        receiver_report = fully_open_report;
      }
    }
    if (success && cooperative_transfer_mode) {
      // The donor is physically clear after the verified short retreat.  From
      // this point onward it must collide normally with the receiver-held
      // object; only the left fingers retain intentional touch permission.
      // Re-attaching an already attached object does not replace touch_links
      // in MoveIt.  Perform an explicit scene-only Left -> World -> Left swap
      // at the measured Gazebo pose while the physical left gripper remains
      // closed and continues supporting the airborne block.
      const auto touch_links = finger_touch_links("left");
      const auto planning_object_pose = transform_pose_to_planning_frame(
        receiver_report->final_sample.pose, tf_buffer);
      success = planning_object_pose.has_value();
      if (success) {
        success = left_arm.detachObject(kObjectId);
        owner = success ? Owner::World : Owner::Left;
      }
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
      if (success) {
        success = planning_scene.applyCollisionObject(make_collision_object(
            *planning_object_pose, object_size_x, object_size_y, object_size_z));
      }
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
      if (success) {
        success = left_arm.attachObject(kObjectId, "left_dex1_tcp", touch_links);
        owner = success ? Owner::Left : Owner::World;
      }
      if (success) {
        success = wait_for_attached_object_with_touch_links(
          planning_scene, kObjectId, "left_dex1_tcp", touch_links);
      }
      if (success) {
        RCLCPP_INFO(
          node->get_logger(),
          "Planning-scene touch permission narrowed to the left receiver fingers only.");
      } else {
        RCLCPP_ERROR(
          node->get_logger(),
          "Failed to replace the receiver attachment with left-only touch permission.");
      }
    }
    if (success && cooperative_transfer_mode) {
      right_arm.setMaxVelocityScalingFactor(velocity_scaling);
      right_arm.setMaxAccelerationScalingFactor(acceleration_scaling);
      RCLCPP_INFO(
        node->get_logger(),
        "Donor is physically clear; restoring its empty-space scaling to %.2f/%.2f.",
        velocity_scaling, acceleration_scaling);
      const auto current_right = current_link_pose_in_frame(
        right_arm, "right_dex1_tcp", kPlanningFrame, tf_buffer);
      success = current_right.has_value();
      if (success) {
        const double tilt = cooperative_inward_tilt_degrees * 0.017453292519943295;
        const double remaining_retreat = approach_clearance - 0.030;
        auto right_full_retreat = *current_right;
        right_full_retreat.position.y -= std::cos(tilt) * remaining_retreat;
        right_full_retreat.position.z += std::sin(tilt) * remaining_retreat;
        success = plan_and_execute_cartesian(
          right_arm, right_full_retreat,
          "cooperative transfer: donor completes outward retreat");
      }
    }
    if (success && cooperative_transfer_mode) {
      success = plan_and_execute_joint(
        right_arm, right_park, "cooperative transfer: park released donor arm");
    }
    if (success && cooperative_transfer_mode) {
      const double minimum_air_clearance = std::max(
        0.0005, cooperative_test_lift - transfer_gate_air_height_tolerance);
      const auto parked_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.20, receiver_hold_seconds,
        minimum_air_clearance, cooperative_test_lift + transfer_gate_air_height_tolerance,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "central receiver-only support after donor retreat");
      const bool parked_matches_short_retreat = parked_report.has_value() &&
        verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, receiver_report->final_sample.pose,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "full donor retreat versus verified short retreat") &&
        verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, *airborne_anchor_pose,
          transfer_gate_max_anchor_displacement,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "full donor retreat versus airborne anchor");
      success = parked_matches_short_retreat;
      if (success) {
        receiver_report = parked_report;
      }
    }
    if (success && cooperative_transfer_mode) {
      const auto current_left = current_link_pose_in_frame(
        left_arm, "left_dex1_tcp", kPlanningFrame, tf_buffer);
      success = current_left.has_value();
      if (success) {
        const double lowering_distance = std::max(
          0.0, receiver_report->bottom_clearance - 0.0005);
        if (lowering_distance < 0.001) {
          RCLCPP_ERROR(
            node->get_logger(),
            "Receiver block is already too close to the table for a controlled lowering.");
          success = false;
        } else {
          expected_table_pose = receiver_report->final_sample.pose;
          expected_table_pose->position.z -= lowering_distance;
          auto left_place = *current_left;
          left_place.position.z -= lowering_distance;
          success = plan_and_execute_cartesian(
            left_arm, left_place,
            "cooperative transfer: receiver lowers block vertically to table");
        }
      }
    }
    if (success && cooperative_transfer_mode) {
      receiver_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.30, transfer_gate_observation,
        -0.002, 0.003,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "receiver placement at table");
      const bool placement_matches_command = receiver_report.has_value() &&
        expected_table_pose.has_value() &&
        verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, *expected_table_pose,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "table placement versus commanded vertical lowering");
      success = placement_matches_command;
    }
    if (success && cooperative_transfer_mode) {
      success = plan_and_execute_joint(
        left_gripper, left_soft, "cooperative transfer: receiver unloads on table");
    }
    if (success && cooperative_transfer_mode) {
      const auto table_unload_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.30, transfer_gate_observation,
        -0.002, 0.003,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "table support after receiver unload");
      const bool table_unload_matches_placement = table_unload_report.has_value() &&
        verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, receiver_report->final_sample.pose,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "table unload versus controlled placement");
      success = table_unload_matches_placement;
      if (success) {
        receiver_report = table_unload_report;
      }
    }
    if (success && cooperative_transfer_mode) {
      success = plan_and_execute_joint(
        left_gripper, left_open, "cooperative transfer: receiver releases block");
      left_closed = !success;
    }
    if (success && cooperative_transfer_mode) {
      const auto release_report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.30, transfer_gate_observation,
        -0.002, 0.003,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "released block resting on table");
      const bool release_matches_unload = release_report.has_value() &&
        verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, receiver_report->final_sample.pose,
          0.005, 5.0 * 0.017453292519943295,
          "released block versus table-unloaded pose");
      success = release_matches_unload;
      if (success) {
        receiver_report = release_report;
      }
    }
    if (success && cooperative_transfer_mode) {
      success = left_arm.detachObject(kObjectId);
      owner = success ? Owner::World : Owner::Left;
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
    }
    if (success && cooperative_transfer_mode) {
      left_arm.setMaxVelocityScalingFactor(velocity_scaling);
      left_arm.setMaxAccelerationScalingFactor(acceleration_scaling);
      RCLCPP_INFO(
        node->get_logger(),
        "Block is released on the table; restoring receiver empty-space scaling to %.2f/%.2f.",
        velocity_scaling, acceleration_scaling);
      const auto current_left = current_link_pose_in_frame(
        left_arm, "left_dex1_tcp", kPlanningFrame, tf_buffer);
      success = current_left.has_value();
      if (success) {
        auto left_retreat = *current_left;
        left_retreat.position.z += transfer_receiver_retreat;
        success = plan_and_execute_cartesian(
          left_arm, left_retreat,
          "cooperative transfer: receiver retreats vertically");
      }
    }
    if (success && cooperative_transfer_mode) {
      success = plan_and_execute_joint(
        left_arm, left_park, "cooperative transfer: park receiver arm");
    }
    if (success && cooperative_transfer_mode) {
      planning_scene.removeCollisionObjects({kObjectId});
      success = wait_for_object_absent(planning_scene, kObjectId);
      owner = success ? Owner::None : Owner::World;
    }
  }

  if (success && sequential_vision_mode && !serial_left_prepositioned) {
    if (!serial_left_staging_pose.has_value() ||
      !serial_left_nominal_contact_pose.has_value() ||
      !gazebo_initial_object_pose.has_value())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Serial receiver prepositioning is missing its visual or Gazebo baseline.");
      success = false;
    } else {
      left_arm.setMaxVelocityScalingFactor(receiver_empty_velocity_scaling);
      left_arm.setMaxAccelerationScalingFactor(receiver_empty_acceleration_scaling);
      RCLCPP_INFO(
        node->get_logger(),
        "Prepositioning the empty left receiver before donor pickup at %.2f/%.2f scaling.",
        receiver_empty_velocity_scaling, receiver_empty_acceleration_scaling);
      const auto left_staging_direct_gate = [](
          const trajectory_msgs::msg::JointTrajectory & trajectory) {
          return verify_direct_joint_path(
            trajectory, "left empty-hand early staging", 0.55, 0.90);
        };
      const auto left_staging_posture_gate = [
        &node, sequential_center_regrasp_test_mode](
          const std::map<std::string, double> & target) {
          const double shoulder_yaw = target.at("left_shoulder_yaw_joint");
          const double elbow = target.at("left_elbow_joint");
          const double minimum_elbow =
            sequential_center_regrasp_test_mode ? 0.50 : 0.80;
          const bool accepted =
            std::abs(shoulder_yaw) <= 1.2 && elbow >= minimum_elbow;
          RCLCPP_INFO(
            node->get_logger(),
            "Left early-staging candidate: shoulder yaw %.3f rad, elbow %.3f rad "
            "(required >= %.3f; %s).",
            shoulder_yaw, elbow, minimum_elbow,
            accepted ? "accepted" : "rejected");
          return accepted;
        };
      const auto left_margin_state = left_arm.getCurrentState(10.0);
      success = left_margin_state != nullptr;
      if (success) {
        const auto left_robot_model = left_margin_state->getRobotModel();
        const auto left_branch_margin_gate = [left_robot_model](
            const trajectory_msgs::msg::JointTrajectory & trajectory) {
            // The selected empty-hand target itself retains a 0.05 rad
            // minimum margin.  Allow the collision-checked transit to pass
            // the shoulder-roll boundary at 0.02 rad instead of rejecting
            // the complete 100% Cartesian continuation seen in simulation.
            return verify_trajectory_joint_bound_margin(
              trajectory, left_robot_model, 0.02,
              "left early-staging final-approach branch");
          };
        const auto left_staging_joint_target = compute_nearest_seeded_ik_joint_target(
          node, left_arm, ik_client, tf_buffer, *serial_left_staging_pose,
          "left_dex1_tcp", left_park, "serial left: early outer staging IK", 32,
          std::numeric_limits<double>::infinity(), 0.05, left_staging_posture_gate);
        success = left_staging_joint_target.has_value();
        if (success) {
          success = plan_and_execute_seeded_joint_with_cartesian_probe(
            left_arm, *left_staging_joint_target,
            {*serial_left_nominal_contact_pose},
            "serial left prepositions before donor pickup",
            left_branch_margin_gate, left_staging_direct_gate);
        }
      }
      if (success) {
        success = verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, *gazebo_initial_object_pose,
          transfer_gate_max_anchor_displacement,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "serial early left staging keeps table block fixed");
      }
      serial_left_prepositioned = success;
    }
  } else if (success && sequential_center_regrasp_test_mode &&
    serial_left_prepositioned)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Reusing the feedback-verified open left receiver at the %.1f mm center staging pose.",
      serial_receiver_prehandover_y_offset * 1000.0);
  }

  if (success && mode != "reset_only" && mode != "pose_test" &&
    !cooperative_mode)
  {
    if (physical_right_pick_mode) {
      const auto direct_approach_gate = [](
          const trajectory_msgs::msg::JointTrajectory & trajectory) {
          return verify_direct_joint_path(trajectory, "right empty-hand pregrasp");
        };
      const auto natural_right_posture_gate = [
        &node, sequential_center_regrasp_test_mode](
          const std::map<std::string, double> & target) {
          const double shoulder_yaw = target.at("right_shoulder_yaw_joint");
          const double elbow = target.at("right_elbow_joint");
          const double minimum_elbow =
            sequential_center_regrasp_test_mode ? 0.50 : 0.80;
          const bool accepted =
            std::abs(shoulder_yaw) <= 1.2 && elbow >= minimum_elbow;
          RCLCPP_INFO(
            node->get_logger(),
            "Right natural-posture candidate: shoulder yaw %.3f rad, elbow %.3f rad "
            "(required >= %.3f; %s).",
            shoulder_yaw, elbow, minimum_elbow,
            accepted ? "accepted" : "rejected");
          return accepted;
        };
      const auto right_margin_state = right_arm.getCurrentState(10.0);
      success = right_margin_state != nullptr;
      if (success) {
        const auto right_robot_model = right_margin_state->getRobotModel();
        const auto right_branch_margin_gate = [right_robot_model](
            const trajectory_msgs::msg::JointTrajectory & trajectory) {
            return verify_trajectory_joint_bound_margin(
              trajectory, right_robot_model, 0.05,
              "right contact-and-lift branch");
          };
        const auto right_pregrasp_joint_target = compute_nearest_seeded_ik_joint_target(
          node, right_arm, ik_client, tf_buffer, right_pregrasp, "right_dex1_tcp",
          right_park, "right physical pick: natural pregrasp IK", 32, 7.0, 0.05,
          natural_right_posture_gate);
        success = right_pregrasp_joint_target.has_value();
        if (success) {
          success = plan_and_execute_seeded_joint_with_cartesian_probe(
            right_arm, *right_pregrasp_joint_target, {right_grasp, right_lift},
            "right moves through the nearest natural pregrasp", right_branch_margin_gate,
            direct_approach_gate);
        }
      }
    } else {
      // Reach the known handover IK branch while the hand is still empty, then
      // follow that branch outward to the pickup.  Returning along this line
      // with the block keeps the TCP orientation fixed and avoids the wrist
      // branch change produced by unconstrained OMPL.
      const auto branch_object = make_object_pose(
        handover_x, handover_y, pickup_object.position.z + handover_lift);
      const auto branch_handover = make_downward_tcp_pose(
        branch_object, right_grasp_y_offset, grasp_z_offset,
        0.0, right_gripper_outward_yaw);
      success = plan_and_execute_pose_with_cartesian_continuation(
        right_arm, branch_handover, right_lift,
        "right selects complete carry IK branch");
    }
    if (success) {
      success = plan_and_execute_cartesian(right_arm, right_grasp, "right descend to end grasp");
      if (!success) {
        diagnose_cartesian_collision(
          right_arm, right_grasp, validity_client,
          "right descend to end grasp (not executed)");
      }
    }
    if (success) {
      success = sequential_center_regrasp_test_mode ?
        refresh_dynamic_octomap_at_stationary_pose(
          node, both_arms, clear_octomap_client, validity_client, 3.0,
          "right open-hand contact") :
        wait_for_sim_duration(node, 0.20, "right contact settle");
    }
    if (success) {
      auto object = make_collision_object(
        pickup_object, object_size_x, object_size_y, object_size_z);
      std_msgs::msg::ColorRGBA color;
      color.r = 0.10F;
      color.g = 0.95F;
      color.b = 0.15F;
      color.a = 1.0F;
      success = planning_scene.applyCollisionObject(object, color);
      owner = success ? Owner::World : Owner::None;
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
    }
    // Keep the table-supported block in the world while the fingers establish
    // real contact.  Attaching it before closure makes MoveIt treat the block as
    // rigidly carried even though Gazebo still has it resting on the table, so
    // gripper planning can be rejected by the table safety shell.  Permit only
    // the intended right-finger/object pairs during closure, then restore the
    // authoritative matrix before assigning kinematic ownership.
    std::optional<moveit_msgs::msg::AllowedCollisionMatrix> right_contact_original_matrix;
    bool right_contact_allowance_active = false;
    if (success) {
      right_contact_original_matrix = read_allowed_collision_matrix(get_scene_client);
      success = right_contact_original_matrix.has_value();
      if (!success) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Could not read the collision matrix for right-finger contact formation.");
      }
    }
    if (success) {
      auto contact_matrix = *right_contact_original_matrix;
      for (const auto & link_name : finger_touch_links("right")) {
        success = set_allowed_collision_pair(
          contact_matrix, kObjectId, link_name, true) && success;
      }
      if (success) {
        success = apply_allowed_collision_matrix(contact_matrix, apply_scene_client);
      }
      right_contact_allowance_active = success;
      if (!success) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Could not apply the temporary right-finger/object contact allowance.");
      }
    }
    if (success) {
      constexpr double kRightPickupMaximumPinchCenterOffset = 0.0002;
      constexpr int kRightPickupMaximumContactAttempts = 3;
      bool symmetric_contact_formed = false;
      for (int attempt = 1;
        attempt <= kRightPickupMaximumContactAttempts && success &&
        !symmetric_contact_formed; ++attempt)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "Right pickup contact attempt %d/%d; required pinch-center offset <= %.3f mm.",
          attempt, kRightPickupMaximumContactAttempts,
          kRightPickupMaximumPinchCenterOffset * 1000.0);
        success = plan_and_execute_joint(
          right_gripper, right_soft,
          "right soft contact attempt " + std::to_string(attempt));
        if (success) {
          success = wait_for_sim_duration(node, 0.15, "right soft contact settle");
        }
        if (success) {
          success = verify_gripper_contact_feedback(
            right_gripper, "right", soft_contact, object_size_x, false,
            "right soft contact attempt " + std::to_string(attempt));
        }
        if (success) {
          success = plan_and_execute_joint(
            right_gripper, right_preload,
            "right grasp preload attempt " + std::to_string(attempt));
          right_closed = success;
        }
        if (success) {
          success = wait_for_sim_duration(node, 0.50, "right preload settle");
        }
        if (success) {
          symmetric_contact_formed = verify_gripper_contact_feedback(
            right_gripper, "right", preload, object_size_x, true,
            "right grasp preload attempt " + std::to_string(attempt),
            kRightPickupMaximumPinchCenterOffset);
        }
        if (success && !symmetric_contact_formed) {
          RCLCPP_WARN(
            node->get_logger(),
            "Right pickup contact attempt %d was asymmetric; reopening on the table before "
            "retrying without moving either arm.", attempt);
          success = plan_and_execute_joint(
            right_gripper, right_open,
            "right reopens after asymmetric contact attempt " + std::to_string(attempt));
          right_closed = !success;
          if (success) {
            success = wait_for_sim_duration(node, 0.20, "right contact retry open settle");
          }
          if (success) {
            success = gripper_open_target_is_reached(
              right_gripper, right_open, 0.0005,
              "right contact retry open feedback");
            right_closed = !success;
          }
        }
      }
      if (success && !symmetric_contact_formed) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Right pickup could not form <= %.3f mm symmetric contact in %d attempts; "
          "the block remains on the table and no lift will be executed.",
          kRightPickupMaximumPinchCenterOffset * 1000.0,
          kRightPickupMaximumContactAttempts);
        success = false;
        right_closed = false;
      }
    }
    if (success && sequential_center_regrasp_test_mode && use_gazebo_grasp_constraint) {
      success = gazebo_grasp_constraint.attach_right_to_cube();
      if (!success) {
        // The physical body is still table-supported and the temporary
        // finger/object collision allowance is active.  Best-effort detach any
        // partially created joint, then reopen before restoring the matrix.
        gazebo_grasp_constraint.detach_right_from_cube(false);
        const bool reopened = plan_and_execute_joint(
          right_gripper, right_open,
          "right releases after unconfirmed Gazebo grasp constraint");
        right_closed = !reopened;
        if (reopened) {
          gripper_open_target_is_reached(
            right_gripper, right_open, 0.0005,
            "right open after unconfirmed Gazebo grasp constraint");
        }
      }
    }
    if (right_contact_allowance_active) {
      const bool restored = apply_allowed_collision_matrix(
        *right_contact_original_matrix, apply_scene_client);
      if (!restored) {
        RCLCPP_ERROR(
          node->get_logger(),
          "CRITICAL: failed to restore the collision matrix after right contact formation.");
        success = false;
      } else {
        RCLCPP_INFO(
          node->get_logger(),
          "Restored normal object collision checking after physical right contact formed.");
      }
    }
    if (success) {
      const auto touch_links = sequential_vision_mode ?
        both_hand_touch_links() : finger_touch_links("right");
      success = right_arm.attachObject(kObjectId, "right_dex1_tcp", touch_links);
      owner = success ? Owner::Right : Owner::World;
      if (success) {
        success = wait_for_attached_object_with_touch_links(
          planning_scene, kObjectId, "right_dex1_tcp", touch_links);
      }
    }
    if (success) {
      if (physical_right_pick_mode) {
        auto right_support_exit = right_grasp;
        right_support_exit.position.z += 0.015;
        success = plan_and_execute_supported_object_vertical_segment(
          right_arm, right_support_exit, true, tf_buffer, validity_client,
          get_scene_client, apply_scene_client,
          "right lift exits known table support by 15 mm");
        if (success) {
          success = plan_and_execute_cartesian(
            right_arm, right_lift,
            "right completes remaining 35 mm collision-aware lift");
        }
      } else {
        success = plan_and_execute_cartesian(
          right_arm, right_lift, "right lifts block clear of table");
      }
    }
    if (success && physical_right_pick_mode) {
      const auto report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.20, transfer_gate_observation,
        pickup_test_lift - transfer_gate_air_height_tolerance,
        pickup_test_lift + transfer_gate_air_height_tolerance,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "right single-hand 5 cm lift");
      right_pick_physical_pass = report.has_value();
      if (report.has_value()) {
        right_lift_physical_pose = report->final_sample.pose;
        const double dx = report->final_sample.pose.position.x -
          gazebo_initial_object_pose->position.x;
        const double dy = report->final_sample.pose.position.y -
          gazebo_initial_object_pose->position.y;
        const double horizontal_displacement = std::hypot(dx, dy);
        const double rotation = pose_rotation_distance(
          *gazebo_initial_object_pose, report->final_sample.pose);
        constexpr double kRadiansToDegrees = 57.29577951308232;
        RCLCPP_INFO(
          node->get_logger(),
          "Right-pick anchor gate: horizontal displacement %.2f mm, orientation change %.2f deg.",
          horizontal_displacement * 1000.0, rotation * kRadiansToDegrees);
        if (horizontal_displacement > transfer_gate_max_anchor_displacement ||
          rotation > transfer_gate_max_angle_degrees * 0.017453292519943295)
        {
          RCLCPP_ERROR(
            node->get_logger(),
            "Right-pick anchor gate rejected the airborne pose; lowering the block safely.");
          right_pick_physical_pass = false;
        }
      }
    }
  }

  if (success && physical_right_pick_mode &&
    (!sequential_vision_mode || !right_pick_physical_pass))
  {
    if (mode == "right_pick_test" && right_pick_physical_pass) {
      RCLCPP_INFO(
        node->get_logger(), "Right pickup test is airborne; holding for %.1f seconds.",
        receiver_hold_seconds);
      success = wait_for_sim_duration(node, receiver_hold_seconds, "right airborne test hold");
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "Right pickup physical gate failed; skipping the hold and lowering the block safely.");
    }
    if (success) {
      auto right_support_entry = right_grasp;
      right_support_entry.position.z += 0.015;
      success = plan_and_execute_cartesian(
        right_arm, right_support_entry,
        "right test lowers 35 mm outside table safety shell");
      if (success) {
        success = plan_and_execute_supported_object_vertical_segment(
          right_arm, right_grasp, false, tf_buffer, validity_client,
          get_scene_client, apply_scene_client,
          "right test enters known table support by 15 mm");
      }
    }
    if (success) {
      success = wait_for_sim_duration(node, 0.30, "right test table settle");
    }
    if (success) {
      success = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.30, transfer_gate_observation,
        -0.002, 0.003,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "right test table support before release").has_value();
    }
    if (success && sequential_center_regrasp_test_mode && use_gazebo_grasp_constraint) {
      success = gazebo_grasp_constraint.detach_right_from_cube(true);
    }
    if (success) {
      success = plan_and_execute_joint(right_gripper, right_open, "right test releases block");
      right_closed = !success;
    }
    if (success) {
      success = wait_for_sim_duration(node, 0.20, "right test release settle");
    }
    if (success) {
      success = right_arm.detachObject(kObjectId);
      owner = success ? Owner::World : Owner::Right;
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
    }
    if (success) {
      success = plan_and_execute_cartesian(right_arm, right_pregrasp, "right test retreat");
    }
    if (success) {
      success = plan_and_execute_joint(right_arm, right_park, "right test park");
    }
    if (success) {
      planning_scene.removeCollisionObjects({kObjectId});
      success = wait_for_object_absent(planning_scene, kObjectId);
      owner = success ? Owner::None : Owner::World;
    }
    if (success && !right_pick_physical_pass) {
      success = false;
    }
  }

  if (success &&
    (mode == "dual_contact_test" || mode == "handover" || donor_release_test_mode))
  {
    const double active_handover_lift =
      sequential_vision_mode ? pickup_test_lift : handover_lift;
    auto handover_object = make_object_pose(
      handover_x, handover_y,
      pickup_object.position.z + active_handover_lift +
      (sequential_center_regrasp_test_mode ? serial_carry_z_compensation : 0.0));
    if (sequential_vision_mode) {
      handover_object.orientation = pickup_object.orientation;
    }
    geometry_msgs::msg::Pose right_handover;
    if (sequential_center_regrasp_test_mode) {
      right_handover = make_inward_tcp_pose(
        handover_object, right_grasp_y_offset, grasp_z_offset,
        cooperative_inward_tilt_degrees * 0.017453292519943295);
    } else if (sequential_vision_mode) {
      right_handover = make_object_aligned_downward_tcp_pose(
        handover_object, right_grasp_y_offset, grasp_z_offset,
        0.0, right_gripper_outward_yaw);
    } else {
      right_handover = make_downward_tcp_pose(
        handover_object, right_grasp_y_offset, grasp_z_offset,
        0.0, right_gripper_outward_yaw);
    }
    auto receiver_object = handover_object;
    std::optional<geometry_msgs::msg::Pose> receiver_reference_pose;

    // Never use unconstrained OMPL while physically carrying the block: it can
    // rotate the TCP through large intermediate angles even when the final
    // pose is correct.  A partial Cartesian solution stops before execution.
    success = plan_and_execute_cartesian(
      right_arm, right_handover, "right carries block to airborne handover");
    if (success && sequential_vision_mode) {
      const auto report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.20, transfer_gate_observation,
        pickup_test_lift - transfer_gate_air_height_tolerance,
        pickup_test_lift + transfer_gate_air_height_tolerance,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "serial right-hand central carry");
      success = report.has_value();
      if (success) {
        receiver_reference_pose = report->final_sample.pose;
        const double central_error = std::hypot(
          report->final_sample.pose.position.x - handover_x,
          report->final_sample.pose.position.y - handover_y);
        const double stage_rotation = pose_rotation_distance(
          *right_lift_physical_pose, report->final_sample.pose);
        constexpr double kRadiansToDegrees = 57.29577951308232;
        RCLCPP_INFO(
          node->get_logger(),
          "Serial carry target gate: central error %.2f mm, stage orientation change %.2f deg.",
          central_error * 1000.0, stage_rotation * kRadiansToDegrees);
        if (central_error > transfer_gate_max_anchor_displacement ||
          stage_rotation > transfer_gate_max_angle_degrees * 0.017453292519943295)
        {
          RCLCPP_ERROR(
            node->get_logger(),
            "Serial carry target gate rejected the held block; refusing the left approach.");
          success = false;
        }
      }
    }
    if (success && sequential_vision_mode) {
      RCLCPP_INFO(
        node->get_logger(),
        "Right hand is stationary at 5 cm; waiting for a fresh real D435 airborne detection...");
      const auto airborne_detection = wait_for_stable_object_pose(
        node, tf_buffer, detection_timeout, detection_max_age, detection_max_spread, 3, false);
      if (!airborne_detection.has_value()) {
        RCLCPP_ERROR(
          node->get_logger(), "No stable real-camera pose for the right-held airborne block.");
        success = false;
      } else {
        receiver_object = *airborne_detection;
        receiver_object.position.x += airborne_vision_x_correction;
        receiver_object.position.y += airborne_vision_y_correction;
        receiver_object.position.z += airborne_vision_z_correction;
        // The donor has already physically verified the carried orientation.
        // Keep that tabletop-derived yaw and use the hand-occluded D435 result
        // only for the receiver's current XYZ.
        receiver_object.orientation = handover_object.orientation;
        RCLCPP_INFO(
          node->get_logger(),
          "Applied airborne hand-occlusion correction [%.1f, %.1f, %.1f] mm.",
          airborne_vision_x_correction * 1000.0,
          airborne_vision_y_correction * 1000.0,
          airborne_vision_z_correction * 1000.0);
        const auto gazebo_planning_pose = transform_pose_to_planning_frame(
          *receiver_reference_pose, tf_buffer);
        success = gazebo_planning_pose.has_value();
        if (success) {
          const Eigen::Vector3d vision_position(
            receiver_object.position.x, receiver_object.position.y,
            receiver_object.position.z);
          const Eigen::Vector3d gazebo_position(
            gazebo_planning_pose->position.x, gazebo_planning_pose->position.y,
            gazebo_planning_pose->position.z);
          const double vision_error = (vision_position - gazebo_position).norm();
          const double vision_orientation_error = pose_rotation_distance(
            receiver_object, *gazebo_planning_pose);
          RCLCPP_INFO(
            node->get_logger(),
            "Fresh airborne D435 pose [%.3f, %.3f, %.3f], Gazebo cross-check error "
            "%.2f mm / %.2f deg.",
            receiver_object.position.x, receiver_object.position.y,
            receiver_object.position.z, vision_error * 1000.0,
            vision_orientation_error * 57.29577951308232);
          if (vision_error > transfer_gate_max_anchor_displacement) {
            RCLCPP_ERROR(
              node->get_logger(),
              "Airborne D435/Gazebo 10 mm position gate failed; refusing the left approach.");
            success = false;
          } else if (sequential_center_regrasp_test_mode && vision_error > 0.005) {
            RCLCPP_WARN(
              node->get_logger(),
              "Airborne D435 error %.2f mm is inside the 10 mm validity gate but outside "
              "the 5 mm precision window; using the verified Gazebo load-bearing XYZ for "
              "the center-contact geometry.",
              vision_error * 1000.0);
            receiver_object.position = gazebo_planning_pose->position;
          } else if (vision_orientation_error >
            transfer_gate_max_angle_degrees * 0.017453292519943295)
          {
            RCLCPP_WARN(
              node->get_logger(),
              "Airborne HSV orientation differs by %.2f deg under gripper occlusion; keeping it "
              "diagnostic-only and using the verified fixed downward handover orientation.",
              vision_orientation_error * 57.29577951308232);
          }
        }
      }
    }

    // The right donor has already passed the physical carry-orientation gate.
    // Preserve that known fixed downward grasp frame for the receiver instead
    // of rotating the left TCP with a hand-occluded HSV contour angle.
    geometry_msgs::msg::Pose left_handover;
    if (sequential_center_regrasp_test_mode) {
      left_handover = make_inward_tcp_pose(
        receiver_object, left_grasp_y_offset, transfer_receiver_z_offset,
        center_regrasp_search_tilt_degrees * 0.017453292519943295);
    } else if (sequential_vision_mode) {
      left_handover = make_object_aligned_downward_tcp_pose(
        receiver_object, left_grasp_y_offset, grasp_z_offset,
        left_gripper_outward_yaw);
    } else {
      left_handover = make_downward_tcp_pose(
        receiver_object, left_grasp_y_offset, grasp_z_offset,
        left_gripper_outward_yaw);
    }
    auto left_pre_handover = left_handover;
    if (sequential_center_regrasp_test_mode) {
      left_pre_handover = make_inward_tcp_pose(
        receiver_object, serial_receiver_prehandover_y_offset,
        transfer_receiver_z_offset,
        center_regrasp_search_tilt_degrees * 0.017453292519943295);
      RCLCPP_INFO(
        node->get_logger(),
        "Center-regrasp receiver stages at object-local Y=%.1f mm, then slides to "
        "COM offset Y=%.1f mm at Z=%.3f mm / tilt %.1f deg.",
        serial_receiver_prehandover_y_offset * 1000.0,
        left_grasp_y_offset * 1000.0,
        transfer_receiver_z_offset * 1000.0,
        center_regrasp_search_tilt_degrees);
    } else if (sequential_vision_mode) {
      left_pre_handover = make_object_aligned_downward_tcp_pose(
        receiver_object, serial_receiver_prehandover_y_offset,
        grasp_z_offset, left_gripper_outward_yaw);
      RCLCPP_INFO(
        node->get_logger(),
        "Serial left same-height outer staging uses object-local Y=%.1f mm before the "
        "horizontal approach to Y=%.1f mm.",
        serial_receiver_prehandover_y_offset * 1000.0,
        left_grasp_y_offset * 1000.0);
    } else {
      left_pre_handover.position.z += approach_clearance;
    }
    if (success && sequential_vision_mode && serial_left_prepositioned) {
      if (sequential_center_regrasp_test_mode) {
        success = refresh_dynamic_octomap_at_stationary_pose(
          node, both_arms, clear_octomap_client, validity_client, 1.0,
          "right-held central transfer");
      }
    }
    if (success && sequential_vision_mode && serial_left_prepositioned) {
      const auto current_left = current_link_pose_in_frame(
        left_arm, "left_dex1_tcp", kPlanningFrame, tf_buffer);
      success = current_left.has_value() && serial_left_staging_pose.has_value();
      if (success) {
        const double dx =
          current_left->position.x - serial_left_staging_pose->position.x;
        const double dy =
          current_left->position.y - serial_left_staging_pose->position.y;
        const double dz =
          current_left->position.z - serial_left_staging_pose->position.z;
        const double translation_error = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double rotation_error = pose_rotation_distance(
          *serial_left_staging_pose, *current_left);
        constexpr double kRadiansToDegrees = 57.29577951308232;
        RCLCPP_INFO(
          node->get_logger(),
          "Using the receiver staged before donor pickup: TCP error %.2f mm / %.2f deg.",
          translation_error * 1000.0, rotation_error * kRadiansToDegrees);
        if (translation_error > transfer_gate_max_anchor_displacement ||
          rotation_error > transfer_gate_max_angle_degrees * 0.017453292519943295)
        {
          RCLCPP_ERROR(
            node->get_logger(),
            "The early receiver staging moved outside the 10 mm / 5 deg gate; refusing "
            "contact instead of replanning while the donor is loaded.");
          success = false;
        }
      }
    }
    if (success && sequential_vision_mode && !serial_left_prepositioned) {
      left_arm.setMaxVelocityScalingFactor(receiver_empty_velocity_scaling);
      left_arm.setMaxAccelerationScalingFactor(receiver_empty_acceleration_scaling);
      RCLCPP_INFO(
        node->get_logger(),
        "Moving the empty left receiver to its 100 mm-clear pregrasp at %.2f/%.2f scaling.",
        receiver_empty_velocity_scaling, receiver_empty_acceleration_scaling);
      const auto left_direct_approach_gate = [](
          const trajectory_msgs::msg::JointTrajectory & trajectory) {
          // This arm is empty and remains 100 mm above the held object.  Allow
          // a bounded OMPL detour around the torso while collision checking,
          // Cartesian contact probing and joint-margin gates stay mandatory.
          return verify_direct_joint_path(
            trajectory, "left empty-hand prehandover", 0.55, 0.90);
        };
      const auto natural_left_posture_gate = [&node](
          const std::map<std::string, double> & target) {
          const double shoulder_yaw = target.at("left_shoulder_yaw_joint");
          const double elbow = target.at("left_elbow_joint");
          const bool accepted = std::abs(shoulder_yaw) <= 1.2 && elbow >= 0.8;
          RCLCPP_INFO(
            node->get_logger(),
            "Left natural-posture candidate: shoulder yaw %.3f rad, elbow %.3f rad (%s).",
            shoulder_yaw, elbow, accepted ? "accepted" : "rejected");
          return accepted;
        };
      const auto left_margin_state = left_arm.getCurrentState(10.0);
      success = left_margin_state != nullptr;
      if (success) {
        const auto left_robot_model = left_margin_state->getRobotModel();
        const auto left_branch_margin_gate = [left_robot_model](
            const trajectory_msgs::msg::JointTrajectory & trajectory) {
            return verify_trajectory_joint_bound_margin(
              trajectory, left_robot_model, 0.05,
              "left final approach branch");
          };
        const auto left_pre_handover_joint_target = compute_nearest_seeded_ik_joint_target(
          node, left_arm, ik_client, tf_buffer, left_pre_handover, "left_dex1_tcp",
          left_park, "serial left: natural prehandover IK", 32,
          std::numeric_limits<double>::infinity(), 0.05, natural_left_posture_gate);
        success = left_pre_handover_joint_target.has_value();
        if (success) {
          success = plan_and_execute_seeded_joint_with_cartesian_probe(
            left_arm, *left_pre_handover_joint_target, {left_handover},
            "serial left uses the nearest natural prehandover", left_branch_margin_gate,
            left_direct_approach_gate);
        }
      }
    } else if (success && !sequential_vision_mode) {
      success = plan_and_execute_pose(left_arm, left_pre_handover, "left approaches held block");
    }
    if (success && sequential_vision_mode) {
      success = verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, *receiver_reference_pose,
        transfer_gate_max_anchor_displacement,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        "serial left pregrasp approach");
    }
    if (success && sequential_vision_mode) {
      left_arm.setMaxVelocityScalingFactor(transfer_motion_velocity_scaling);
      left_arm.setMaxAccelerationScalingFactor(transfer_motion_acceleration_scaling);
      RCLCPP_INFO(
        node->get_logger(),
        "Entering the receiver contact window at the original %.2f/%.2f scaling.",
        transfer_motion_velocity_scaling, transfer_motion_acceleration_scaling);
    }
    const auto center_contact_path_gate = [&](
        const trajectory_msgs::msg::JointTrajectory & trajectory) -> bool {
        if (!sequential_center_regrasp_test_mode || !gazebo_table_surface_z.has_value()) {
          return !sequential_center_regrasp_test_mode;
        }
        auto common_start = both_arms.getCurrentState(10.0);
        if (!common_start) {
          return false;
        }
        common_start->update();
        ::planning_scene::PlanningScene geometric_scene(both_arms.getRobotModel());
        return validate_center_regrasp_path(
          trajectory, *common_start, geometric_scene, validity_client,
          *gazebo_table_surface_z, "serial center-regrasp live contact path").has_value();
      };
    std::vector<geometry_msgs::msg::Pose> center_contact_intermediate_waypoints;
    if (success && sequential_center_regrasp_test_mode) {
      center_contact_intermediate_waypoints.push_back(make_inward_tcp_pose(
          receiver_object, serial_receiver_prehandover_y_offset,
          transfer_receiver_z_offset + center_regrasp_overpass_height,
          center_regrasp_search_tilt_degrees * 0.017453292519943295));
      center_contact_intermediate_waypoints.push_back(make_inward_tcp_pose(
          receiver_object, left_grasp_y_offset,
          transfer_receiver_z_offset + center_regrasp_overpass_height,
          center_regrasp_search_tilt_degrees * 0.017453292519943295));
      RCLCPP_INFO(
        node->get_logger(),
        "Center-regrasp overpass path: raise the open receiver %.1f mm at local Y=%.1f mm, "
        "translate above the donor base to Y=%.1f mm, then lower vertically to contact.",
        center_regrasp_overpass_height * 1000.0,
        serial_receiver_prehandover_y_offset * 1000.0,
        left_grasp_y_offset * 1000.0);
    }
    if (success) {
      success = plan_and_execute_cartesian(
        left_arm, left_handover, "left enters COM-centered grasp", false,
        center_contact_path_gate, center_contact_intermediate_waypoints);
      if (!success && sequential_center_regrasp_test_mode) {
        diagnose_cartesian_collision(
          left_arm, left_handover, validity_client,
          "left enters COM-centered grasp (not executed)");
      }
    }
    if (success && sequential_vision_mode) {
      success = verify_gazebo_pose_near_reference(
        node, gazebo_pose_monitor, *receiver_reference_pose,
        transfer_gate_max_anchor_displacement,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        "serial left open-finger contact");
    }
    const auto serial_center_current_clearance_gate = [&] (
        const std::string & stage_name) -> bool {
        if (!sequential_center_regrasp_test_mode || !gazebo_table_surface_z.has_value()) {
          return !sequential_center_regrasp_test_mode;
        }
        auto state = both_arms.getCurrentState(10.0);
        if (!state) {
          return false;
        }
        state->update();
        ::planning_scene::PlanningScene geometric_scene(both_arms.getRobotModel());
        const auto clearance = center_regrasp_clearance(
          geometric_scene, *state, *gazebo_table_surface_z);
        if (!clearance.has_value()) {
          return false;
        }
        RCLCPP_INFO(
          node->get_logger(),
          "Center-regrasp clearance gate '%s': hand/hand %.3f mm, hand/table %.3f mm.",
          stage_name.c_str(), clearance->hand_to_hand * 1000.0,
          clearance->hand_to_table * 1000.0);
        return clearance->hand_to_hand >= 0.005 &&
          clearance->hand_to_table >= 0.010 &&
          full_robot_state_is_valid(validity_client, *state, stage_name);
      };
    if (success) {
      success = plan_and_execute_joint(left_gripper, left_soft, "left soft contact");
    }
    if (success) {
      success = serial_center_current_clearance_gate("left soft-contact endpoint");
    }
    if (success) {
      success = wait_for_sim_duration(node, 0.15, "left soft contact settle");
    }
    if (success) {
      success = verify_gripper_contact_feedback(
        left_gripper, "left", soft_contact, object_size_x, false,
        "left soft contact");
    }
    if (success) {
      success = plan_and_execute_joint(
        left_gripper,
        sequential_vision_mode ? left_transfer_preload : left_preload,
        "left grasp preload");
      left_closed = success;
    }
    if (success) {
      success = wait_for_sim_duration(node, 0.50, "left preload settle");
    }
    if (success) {
      success = verify_gripper_contact_feedback(
        left_gripper, "left",
        sequential_vision_mode ? transfer_receiver_preload : preload,
        object_size_x, true, "left grasp preload");
    }
    if (success) {
      success = serial_center_current_clearance_gate("left COM-preload endpoint");
    }
    if (success) {
      RCLCPP_INFO(
        node->get_logger(),
        "Both grippers are closed on opposite ends; cooperative hold for %.1f seconds.",
        dual_hold_seconds);
      success = wait_for_sim_duration(node, dual_hold_seconds, "cooperative dual contact hold");
    }
    if (success && sequential_vision_mode) {
      const auto report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.20, transfer_gate_observation,
        pickup_test_lift - transfer_gate_air_height_tolerance,
        pickup_test_lift + transfer_gate_air_height_tolerance,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "serial dual-contact hold");
      success = report.has_value();
      if (success) {
        const Eigen::Vector3d reference_position(
          receiver_reference_pose->position.x, receiver_reference_pose->position.y,
          receiver_reference_pose->position.z);
        const Eigen::Vector3d current_position(
          report->final_sample.pose.position.x, report->final_sample.pose.position.y,
          report->final_sample.pose.position.z);
        const double displacement = (current_position - reference_position).norm();
        const double rotation = pose_rotation_distance(
          *receiver_reference_pose, report->final_sample.pose);
        constexpr double kRadiansToDegrees = 57.29577951308232;
        RCLCPP_INFO(
          node->get_logger(),
          "Serial dual-contact anchor gate: displacement %.2f mm, rotation %.2f deg.",
          displacement * 1000.0, rotation * kRadiansToDegrees);
        if (displacement > transfer_gate_max_anchor_displacement ||
          rotation > transfer_gate_max_angle_degrees * 0.017453292519943295)
        {
          RCLCPP_ERROR(
            node->get_logger(),
            "Serial dual-contact anchor gate failed; refusing donor release.");
          success = false;
        }
      }
    }

    if (success && mode == "dual_contact_test") {
      success = plan_and_execute_joint(left_gripper, left_soft, "dual test: left unloads preload");
      if (success) {
        success = wait_for_sim_duration(node, 0.30, "dual test left preload unload");
      }
      if (success && sequential_center_regrasp_test_mode) {
        success = verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, *receiver_reference_pose,
          transfer_gate_max_anchor_displacement,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "center test left preload unload");
      }
      if (success) {
        success = plan_and_execute_joint(left_gripper, left_unload, "dual test: left clears block");
      }
      if (success) {
        success = wait_for_sim_duration(node, 0.30, "dual test left clearance settle");
      }
      if (success && sequential_center_regrasp_test_mode) {
        success = verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, *receiver_reference_pose,
          transfer_gate_max_anchor_displacement,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "center test left clears while right supports");
      }
      if (success) {
        success = plan_and_execute_joint(left_gripper, left_open, "dual test: left opens fully");
      }
      left_closed = !success;
      if (success) {
        success = wait_for_sim_duration(node, 0.20, "dual test left release settle");
      }
      if (success && sequential_center_regrasp_test_mode) {
        success = verify_gazebo_pose_near_reference(
          node, gazebo_pose_monitor, *receiver_reference_pose,
          transfer_gate_max_anchor_displacement,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          "center test left fully open while right supports");
      }
      if (success) {
        success = plan_and_execute_cartesian(
          left_arm, left_pre_handover, "dual test: left retreats");
      }
      if (success) {
        success = plan_and_execute_cartesian(
          right_arm, right_lift, "dual test: right returns above pickup");
      }
      if (success && sequential_center_regrasp_test_mode) {
        success = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, *gazebo_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.20, transfer_gate_observation,
          pickup_test_lift - transfer_gate_air_height_tolerance,
          pickup_test_lift + transfer_gate_air_height_tolerance,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          transfer_gate_max_tilt_degrees * 0.017453292519943295,
          "center test right-only return carry").has_value();
      }
      if (success && sequential_center_regrasp_test_mode) {
        auto right_support_entry = right_grasp;
        right_support_entry.position.z += 0.015;
        success = plan_and_execute_cartesian(
          right_arm, right_support_entry,
          "center test lowers 35 mm outside table safety shell");
        if (success) {
          success = plan_and_execute_supported_object_vertical_segment(
            right_arm, right_grasp, false, tf_buffer, validity_client,
            get_scene_client, apply_scene_client,
            "center test enters known table support by 15 mm");
        }
      } else if (success) {
        success = plan_and_execute_cartesian(
          right_arm, right_grasp, "dual test: right lowers block");
      }
      if (success) {
        success = wait_for_sim_duration(node, 0.30, "dual test table settle");
      }
      if (success && sequential_center_regrasp_test_mode) {
        success = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, *gazebo_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.30, transfer_gate_observation,
          -0.002, 0.003,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          transfer_gate_max_tilt_degrees * 0.017453292519943295,
          "center test table support before right release").has_value();
      }
      if (success && sequential_center_regrasp_test_mode && use_gazebo_grasp_constraint) {
        success = gazebo_grasp_constraint.detach_right_from_cube(true);
      }
      if (success) {
        success = plan_and_execute_joint(right_gripper, right_open, "dual test: right releases");
        right_closed = !success;
      }
      if (success) {
        success = wait_for_sim_duration(node, 0.20, "dual test release settle");
      }
      if (success) {
        success = right_arm.detachObject(kObjectId);
        owner = success ? Owner::World : Owner::Right;
        if (success) {
          success = wait_for_world_object(planning_scene, kObjectId);
        }
      }
      if (success) {
        success = plan_and_execute_cartesian(
          right_arm, right_pregrasp, "dual test: right retreat");
      }
      if (success) {
        success = plan_and_execute_joint(right_arm, right_park, "dual test: park right");
      }
      if (success) {
        success = plan_and_execute_joint(left_arm, left_park, "dual test: park left");
      }
      if (success) {
        planning_scene.removeCollisionObjects({kObjectId});
        success = wait_for_object_absent(planning_scene, kObjectId);
        owner = success ? Owner::None : Owner::World;
      }
    }

    const auto serial_receiver_support_gate = [&](const std::string & stage_name) -> bool {
        if (!sequential_vision_mode) {
          return true;
        }
        if (!receiver_reference_pose.has_value()) {
          RCLCPP_ERROR(
            node->get_logger(),
            "Serial receiver gate '%s' has no verified airborne anchor.",
            stage_name.c_str());
          return false;
        }
        const auto report = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, *gazebo_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.20, transfer_gate_observation,
          pickup_test_lift - transfer_gate_air_height_tolerance,
          pickup_test_lift + transfer_gate_air_height_tolerance,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          transfer_gate_max_tilt_degrees * 0.017453292519943295,
          stage_name);
        if (!report.has_value()) {
          return false;
        }
        const Eigen::Vector3d anchor_position(
          receiver_reference_pose->position.x,
          receiver_reference_pose->position.y,
          receiver_reference_pose->position.z);
        const Eigen::Vector3d current_position(
          report->final_sample.pose.position.x,
          report->final_sample.pose.position.y,
          report->final_sample.pose.position.z);
        const double displacement = (current_position - anchor_position).norm();
        const double rotation = pose_rotation_distance(
          *receiver_reference_pose, report->final_sample.pose);
        constexpr double kRadiansToDegrees = 57.29577951308232;
        RCLCPP_INFO(
          node->get_logger(),
          "Serial receiver anchor gate '%s': displacement %.2f mm / %.2f deg.",
          stage_name.c_str(), displacement * 1000.0,
          rotation * kRadiansToDegrees);
        if (displacement > transfer_gate_max_anchor_displacement ||
          rotation > transfer_gate_max_angle_degrees * 0.017453292519943295)
        {
          RCLCPP_ERROR(
            node->get_logger(),
            "Serial receiver anchor gate '%s' failed; preserving the current grasp state.",
            stage_name.c_str());
          return false;
        }
        return true;
      };

    if (success && donor_release_test_mode) {
      // Synchronize MoveIt's attached object to the last measured Gazebo pose,
      // then give the receiver ownership while both real grippers stay closed.
      const auto planning_object_pose = transform_pose_to_planning_frame(
        *receiver_reference_pose, tf_buffer);
      success = planning_object_pose.has_value();
      if (success) {
        success = right_arm.detachObject(kObjectId);
        owner = success ? Owner::World : Owner::Right;
      }
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
      if (success) {
        success = planning_scene.applyCollisionObject(make_collision_object(
            *planning_object_pose, object_size_x, object_size_y, object_size_z));
      }
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
      if (success) {
        success = left_arm.attachObject(
          kObjectId, "left_dex1_tcp", both_hand_touch_links());
        owner = success ? Owner::Left : Owner::World;
      }
      if (success) {
        success = wait_for_attached_object(
          planning_scene, kObjectId, "left_dex1_tcp");
      }

      auto previous_support_pose = *receiver_reference_pose;
      const auto donor_receiver_gate = [&] (const std::string & stage_name) -> bool {
          const auto report = verify_gazebo_object_stability(
            node, gazebo_pose_monitor, *gazebo_table_surface_z,
            object_size_x, object_size_y, object_size_z,
            0.20, transfer_gate_observation,
            pickup_test_lift - transfer_gate_air_height_tolerance,
            pickup_test_lift + transfer_gate_air_height_tolerance,
            transfer_gate_max_drift,
            transfer_gate_max_angle_degrees * 0.017453292519943295,
            transfer_gate_max_tilt_degrees * 0.017453292519943295,
            stage_name);
          if (!report.has_value()) {
            return false;
          }
          const Eigen::Vector3d anchor_position(
            receiver_reference_pose->position.x,
            receiver_reference_pose->position.y,
            receiver_reference_pose->position.z);
          const Eigen::Vector3d previous_position(
            previous_support_pose.position.x,
            previous_support_pose.position.y,
            previous_support_pose.position.z);
          const Eigen::Vector3d current_position(
            report->final_sample.pose.position.x,
            report->final_sample.pose.position.y,
            report->final_sample.pose.position.z);
          const double step_displacement = (current_position - previous_position).norm();
          const double cumulative_displacement = (current_position - anchor_position).norm();
          const double step_rotation = pose_rotation_distance(
            previous_support_pose, report->final_sample.pose);
          const double cumulative_rotation = pose_rotation_distance(
            *receiver_reference_pose, report->final_sample.pose);
          constexpr double kRadiansToDegrees = 57.29577951308232;
          RCLCPP_INFO(
            node->get_logger(),
            "Donor-release gate '%s': step %.2f mm / %.2f deg, cumulative %.2f mm / "
            "%.2f deg.",
            stage_name.c_str(), step_displacement * 1000.0,
            step_rotation * kRadiansToDegrees,
            cumulative_displacement * 1000.0,
            cumulative_rotation * kRadiansToDegrees);
          if (step_displacement > transfer_gate_max_drift ||
            cumulative_displacement > transfer_gate_max_anchor_displacement ||
            step_rotation > transfer_gate_max_angle_degrees * 0.017453292519943295 ||
            cumulative_rotation > transfer_gate_max_angle_degrees * 0.017453292519943295)
          {
            RCLCPP_ERROR(
              node->get_logger(),
              "Donor-release physical gate failed at '%s'; preserving the current gripper "
              "state.", stage_name.c_str());
            return false;
          }
          previous_support_pose = report->final_sample.pose;
          return true;
        };

      // Preserve the block as an independent Gazebo collision body, but move
      // the compliant grasp support from the verified donor grasp to the
      // verified receiver grasp before unloading.  Both hands remain closed
      // during the overlap, so there is no unsupported transfer instant.
      if (success && use_gazebo_grasp_constraint) {
        success = gazebo_grasp_constraint.attach_left_to_cube();
      }
      if (success) {
        success = gazebo_grasp_constraint.detach_right_from_cube(true);
      }
      if (success) {
        success = wait_for_sim_duration(
          node, 0.50, "left support after right simulation-support detach");
      }
      if (success) {
        success = donor_receiver_gate("left-supported hold after right support detach");
        if (!success) {
          gazebo_grasp_constraint.reattach_right_to_cube();
        }
      }

      const std::vector<double> donor_unload_targets{0.002, 0.001, 0.0, -0.001, -0.002};
      for (const double target : donor_unload_targets) {
        if (!success) {
          break;
        }
        const int target_mm = static_cast<int>(std::lround(target * 1000.0));
        const std::string stage_name =
          "right donor unload target " + std::to_string(target_mm) + " mm";
        const std::map<std::string, double> finger_target{
          {"right_dex1_finger_1_joint", target},
          {"right_dex1_finger_2_joint", target},
        };
        success = plan_and_execute_joint(right_gripper, finger_target, stage_name);
        if (success) {
          success = wait_for_sim_duration(node, 0.30, stage_name + " settle");
        }
        if (success) {
          success = verify_gripper_contact_feedback(
            right_gripper, "right", target, object_size_x, false,
            stage_name + " feedback");
        }
        if (success) {
          success = donor_receiver_gate(stage_name);
        }
      }

      const auto donor_retreat_path_gate = [&] (
          const trajectory_msgs::msg::JointTrajectory & trajectory) -> bool {
          auto common_start = both_arms.getCurrentState(10.0);
          if (!common_start) {
            return false;
          }
          common_start->update();
          ::planning_scene::PlanningScene geometric_scene(both_arms.getRobotModel());
          return validate_center_regrasp_path(
            trajectory, *common_start, geometric_scene, validity_client,
            *gazebo_table_surface_z, "donor-release retreat path").has_value();
        };
      const Eigen::Matrix3d object_rotation = pose_to_eigen(handover_object).linear();
      const double donor_tilt = cooperative_inward_tilt_degrees * 0.017453292519943295;
      const auto retreat_donor_by = [&] (
          const double distance, const double retreat_tilt,
          const std::string & stage_name) -> bool {
          const auto current_right = current_link_pose_in_frame(
            right_arm, "right_dex1_tcp", kPlanningFrame, tf_buffer);
          if (!current_right.has_value()) {
            return false;
          }
          auto target = *current_right;
          const Eigen::Vector3d displacement = object_rotation * Eigen::Vector3d(
            0.0, -std::cos(retreat_tilt) * distance,
            std::sin(retreat_tilt) * distance);
          target.position.x += displacement.x();
          target.position.y += displacement.y();
          target.position.z += displacement.z();
          return plan_and_execute_cartesian(
            right_arm, target, stage_name, false, donor_retreat_path_gate);
        };

      if (success) {
        success = plan_and_execute_joint(
          right_gripper, right_open,
          "right donor opens fully at unloaded contact before retreat");
      }
      if (success) {
        success = wait_for_sim_duration(node, 0.20, "clear donor full-open settle");
      }
      if (success) {
        success = gripper_open_target_is_reached(
          right_gripper, right_open, 0.0005, "clear donor full-open feedback");
        right_closed = !success;
      }
      if (success) {
        success = donor_receiver_gate("clear donor fully open");
      }
      if (success) {
        success = retreat_donor_by(
          0.015, donor_tilt, "open right donor peels outward 15 mm");
      }
      if (success) {
        success = donor_receiver_gate("open right donor 15 mm peel retreat");
      }
      if (success) {
        success = left_arm.attachObject(
          kObjectId, "left_dex1_tcp", hand_touch_links("left"));
        if (success) {
          success = wait_for_attached_object(
            planning_scene, kObjectId, "left_dex1_tcp");
        }
      }
      if (success) {
        success = retreat_donor_by(
          0.035, 0.0,
          "right donor completes 50 mm cumulative retreat with horizontal second segment");
      }
      if (success) {
        success = donor_receiver_gate("right donor cumulative 50 mm retreat");
      }
      if (success) {
        success = plan_and_execute_joint(right_arm, right_park, "right parks after donor release");
      }
      if (success) {
        success = verify_gripper_contact_feedback(
          left_gripper, "left", transfer_receiver_preload, object_size_x, true,
          "left-only hold gripper feedback");
      }
      if (success) {
        success = wait_for_sim_duration(node, receiver_hold_seconds, "left-only airborne hold");
      }
      if (success) {
        success = donor_receiver_gate("left-only airborne hold");
      }

      const auto left_air_pose = success ? current_link_pose_in_frame(
        left_arm, "left_dex1_tcp", kPlanningFrame, tf_buffer) : std::nullopt;
      success = success && left_air_pose.has_value();
      geometry_msgs::msg::Pose left_support_entry;
      geometry_msgs::msg::Pose left_table_pose;
      if (success) {
        left_support_entry = *left_air_pose;
        left_support_entry.position.z -= 0.035;
        left_table_pose = *left_air_pose;
        left_table_pose.position.z -= 0.050;
        success = plan_and_execute_cartesian(
          left_arm, left_support_entry, "left-only lowers 35 mm outside table shell");
      }
      if (success) {
        success = plan_and_execute_supported_object_vertical_segment(
          left_arm, left_table_pose, false, tf_buffer, validity_client,
          get_scene_client, apply_scene_client,
          "left-only enters table support by final 15 mm");
      }
      if (success) {
        success = wait_for_sim_duration(node, 0.30, "left-only table settle");
      }
      if (success) {
        success = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, *gazebo_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.30, transfer_gate_observation,
          -0.002, 0.003,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          transfer_gate_max_tilt_degrees * 0.017453292519943295,
          "left-only table support before release").has_value();
      }
      if (success && use_gazebo_grasp_constraint) {
        success = gazebo_grasp_constraint.detach_left_from_cube(true);
      }
      if (success) {
        success = wait_for_sim_duration(node, 0.20, "table support after left support detach");
      }
      if (success) {
        success = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, *gazebo_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.20, transfer_gate_observation,
          -0.002, 0.003,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          transfer_gate_max_tilt_degrees * 0.017453292519943295,
          "table support after left compliant support detach").has_value();
      }
      if (success) {
        success = plan_and_execute_joint(
          left_gripper, left_open, "left releases after verified table support");
      }
      if (success) {
        success = wait_for_sim_duration(node, 0.20, "left table release settle");
      }
      if (success) {
        success = gripper_open_target_is_reached(
          left_gripper, left_open, 0.0005, "left final open feedback");
        for (int retry = 1; !success && retry <= 3; ++retry) {
          const bool reasserted = plan_and_execute_joint(
            left_gripper, left_open,
            "left reasserts final open target attempt " + std::to_string(retry));
          success = reasserted && gripper_open_target_is_reached(
            left_gripper, left_open, 0.0005,
            "left final open feedback retry " + std::to_string(retry));
        }
        left_closed = !success;
      }
      if (success) {
        success = verify_gazebo_object_stability(
          node, gazebo_pose_monitor, *gazebo_table_surface_z,
          object_size_x, object_size_y, object_size_z,
          0.20, transfer_gate_observation,
          -0.002, 0.003,
          transfer_gate_max_drift,
          transfer_gate_max_angle_degrees * 0.017453292519943295,
          transfer_gate_max_tilt_degrees * 0.017453292519943295,
          "final table placement after left release").has_value();
      }
      if (success) {
        success = left_arm.detachObject(kObjectId);
        owner = success ? Owner::World : Owner::Left;
        if (success) {
          success = wait_for_world_object(planning_scene, kObjectId);
        }
      }
      if (success) {
        success = plan_and_execute_cartesian(
          left_arm, *left_air_pose, "left retreats vertically after table release");
      }
      if (success) {
        success = plan_and_execute_joint(left_arm, left_park, "left parks after placement");
      }
      if (success) {
        planning_scene.removeCollisionObjects({kObjectId});
        success = wait_for_object_absent(planning_scene, kObjectId);
        owner = success ? Owner::None : Owner::World;
      }
    }

    // MoveIt's collision object has one kinematic owner at a time.  Physical
    // Gazebo contact remains on both grippers during this short ownership swap.
    if (success && mode == "handover") {
      success = right_arm.detachObject(kObjectId);
      owner = success ? Owner::World : Owner::Right;
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
    }
    if (success && mode == "handover") {
      success = left_arm.attachObject(
        kObjectId, "left_dex1_tcp", both_hand_touch_links());
      owner = success ? Owner::Left : Owner::World;
      if (success) {
        success = wait_for_attached_object(
          planning_scene, kObjectId, "left_dex1_tcp");
      }
    }
    if (success && mode == "handover") {
      success = plan_and_execute_joint(right_gripper, right_soft, "right unloads donor preload");
    }
    if (success && mode == "handover") {
      success = wait_for_sim_duration(node, 0.30, "right donor preload unload");
    }
    if (success && mode == "handover" && sequential_vision_handover_mode) {
      success = serial_receiver_support_gate("serial donor preload unload");
    }
    if (success && mode == "handover") {
      success = plan_and_execute_joint(right_gripper, right_unload, "right clears transferred block");
    }
    if (success && mode == "handover") {
      success = wait_for_sim_duration(node, 0.30, "right donor clearance settle");
    }
    if (success && mode == "handover" && sequential_vision_handover_mode) {
      success = serial_receiver_support_gate("serial donor clears receiver-held block");
    }
    if (success && mode == "handover") {
      success = plan_and_execute_joint(right_gripper, right_open, "right opens after transfer");
      right_closed = !success;
    }
    if (success && mode == "handover") {
      success = wait_for_sim_duration(node, 0.20, "right donor release settle");
    }
    if (success && mode == "handover" && sequential_vision_handover_mode) {
      success = serial_receiver_support_gate("serial donor fully open");
    }
    if (success && mode == "handover") {
      auto right_retreat = right_handover;
      if (sequential_vision_handover_mode) {
        const Eigen::Vector3d world_retreat =
          pose_to_eigen(handover_object).linear() *
          Eigen::Vector3d(0.0, -approach_clearance, 0.0);
        right_retreat.position.x += world_retreat.x();
        right_retreat.position.y += world_retreat.y();
        right_retreat.position.z += world_retreat.z();
      } else {
        right_retreat.position.y -= approach_clearance;
      }
      success = plan_and_execute_cartesian(
        right_arm, right_retreat, "right exits outward while left supports block");
    }
    if (success && mode == "handover" && sequential_vision_handover_mode) {
      success = serial_receiver_support_gate("serial donor outward retreat");
    }
    if (success && mode == "handover") {
      success = plan_and_execute_joint(right_arm, right_park, "right parks after transfer");
    }
    if (success && mode == "handover") {
      // The donor is now clear, so restore normal collision checking between
      // the carried block and the right hand for all subsequent motion.
      success = left_arm.attachObject(
        kObjectId, "left_dex1_tcp", hand_touch_links("left"));
      if (success) {
        success = wait_for_attached_object(
          planning_scene, kObjectId, "left_dex1_tcp");
      }
    }
    if (success && mode == "handover") {
      RCLCPP_INFO(
        node->get_logger(), "Left gripper alone supports the block; holding for %.1f seconds.",
        receiver_hold_seconds);
      success = wait_for_sim_duration(node, receiver_hold_seconds, "left receiver hold");
    }
    if (success && sequential_vision_handover_mode) {
      const auto report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.20, transfer_gate_observation,
        pickup_test_lift - transfer_gate_air_height_tolerance,
        pickup_test_lift + transfer_gate_air_height_tolerance,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "serial left receiver-only hold");
      success = report.has_value();
      if (success) {
        receiver_reference_pose = report->final_sample.pose;
      }
    }

    auto place_object_air = make_object_pose(
      place_x, place_y, pickup_object.position.z + active_handover_lift);
    auto place_object_table = make_object_pose(
      place_x, place_y, pickup_object.position.z);
    if (sequential_vision_handover_mode) {
      place_object_air.orientation = handover_object.orientation;
      place_object_table.orientation = handover_object.orientation;
    }
    const auto left_pre_place = sequential_vision_handover_mode ?
      make_object_aligned_downward_tcp_pose(
        place_object_air, left_grasp_y_offset, grasp_z_offset,
        left_gripper_outward_yaw) :
      make_downward_tcp_pose(
        place_object_air, left_grasp_y_offset, grasp_z_offset,
        left_gripper_outward_yaw);
    const auto left_place = sequential_vision_handover_mode ?
      make_object_aligned_downward_tcp_pose(
        place_object_table, left_grasp_y_offset, grasp_z_offset,
        left_gripper_outward_yaw) :
      make_downward_tcp_pose(
        place_object_table, left_grasp_y_offset, grasp_z_offset,
        left_gripper_outward_yaw);
    if (success && mode == "handover") {
      success = plan_and_execute_cartesian(
        left_arm, left_pre_place, "left carries transferred block");
    }
    if (success && sequential_vision_handover_mode) {
      const auto report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.20, transfer_gate_observation,
        pickup_test_lift - transfer_gate_air_height_tolerance,
        pickup_test_lift + transfer_gate_air_height_tolerance,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "serial left carry to place");
      success = report.has_value();
      if (success) {
        const double place_error = std::hypot(
          report->final_sample.pose.position.x - place_x,
          report->final_sample.pose.position.y - place_y);
        RCLCPP_INFO(
          node->get_logger(), "Serial place target horizontal error %.2f mm.",
          place_error * 1000.0);
        if (place_error > transfer_gate_max_anchor_displacement) {
          RCLCPP_ERROR(
            node->get_logger(),
            "Serial place target exceeded the 10 mm distance gate; refusing to descend.");
          success = false;
        }
      }
    }
    if (success && mode == "handover") {
      success = plan_and_execute_cartesian(left_arm, left_place, "left lowers block to table");
    }
    if (success && mode == "handover") {
      success = wait_for_sim_duration(node, 0.30, "left table settle");
    }
    if (success && mode == "handover") {
      success = plan_and_execute_joint(left_gripper, left_open, "left releases block");
      left_closed = !success;
    }
    if (success && mode == "handover") {
      success = wait_for_sim_duration(node, 0.20, "left release settle");
    }
    if (success && sequential_vision_handover_mode) {
      const auto report = verify_gazebo_object_stability(
        node, gazebo_pose_monitor, *gazebo_table_surface_z,
        object_size_x, object_size_y, object_size_z,
        0.20, transfer_gate_observation,
        -0.003, 0.005,
        transfer_gate_max_drift,
        transfer_gate_max_angle_degrees * 0.017453292519943295,
        transfer_gate_max_tilt_degrees * 0.017453292519943295,
        "serial final table placement");
      success = report.has_value();
    }
    if (success && mode == "handover") {
      success = left_arm.detachObject(kObjectId);
      owner = success ? Owner::World : Owner::Left;
      if (success) {
        success = wait_for_world_object(planning_scene, kObjectId);
      }
    }
    if (success && mode == "handover") {
      success = plan_and_execute_cartesian(left_arm, left_pre_place, "left final retreat");
    }
    if (success && mode == "handover") {
      success = plan_and_execute_joint(left_arm, left_park, "left final park");
    }
    if (success && mode == "handover") {
      planning_scene.removeCollisionObjects({kObjectId});
      success = wait_for_object_absent(planning_scene, kObjectId);
      owner = success ? Owner::None : Owner::World;
    }
  }

  if (
    !success && cooperative_transfer_mode &&
    cooperative_both_pregrasp_reached && cooperative_right_final_entry_attempted &&
    !cooperative_right_contact_reached && !right_closed && !left_closed)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Cooperative contact entry failed before either gripper closed.  Retreating the empty "
      "receiver along its verified Cartesian slide to the 145 mm outer staging pose before "
      "leaving the failed run; no joint-space reset will be attempted near the block.");
    bool recovery_scene_ready = false;
    if (owner == Owner::Base) {
      recovery_scene_ready = both_arms.detachObject(kObjectId);
      if (recovery_scene_ready) {
        recovery_scene_ready = wait_for_world_object(planning_scene, kObjectId);
      }
      if (recovery_scene_ready) {
        owner = Owner::World;
      } else {
        RCLCPP_ERROR(
          node->get_logger(),
          "Could not restore the block as a world collision object; refusing the automatic "
          "empty-hand retreat because its collision scene is not trustworthy.");
      }
    } else if (owner == Owner::World) {
      recovery_scene_ready = wait_for_world_object(planning_scene, kObjectId);
      if (!recovery_scene_ready) {
        RCLCPP_ERROR(
          node->get_logger(),
          "The block owner says world, but the collision object is not observable; refusing the "
          "automatic empty-hand retreat.");
      }
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "Unexpected block ownership during contact-entry recovery; refusing automatic motion.");
    }
    const bool empty_receiver_retreat_ok = recovery_scene_ready &&
      plan_and_execute_cartesian(
      left_arm, left_outer_staging,
      "failure recovery: empty receiver retreats to outer staging");
    if (empty_receiver_retreat_ok) {
      cooperative_both_pregrasp_reached = false;
      RCLCPP_WARN(
        node->get_logger(),
        "Empty receiver reached the outer staging pose.  No fallback joint-space reset will run "
        "during this failed attempt; inspect the donor pose before starting another run.");
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "Empty receiver Cartesian retreat did not complete.  No fallback reset will be attempted; "
        "inspect the actual arm poses in Gazebo before starting another run.");
    }
  }

  if (!success) {
    if (right_closed || left_closed) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Air handover stopped while a gripper may support the block.  Grippers remain closed "
        "to avoid dropping it; inspect Gazebo before any reset.");
    } else {
      if (owner == Owner::Right) {
        if (right_arm.detachObject(kObjectId)) {
          wait_for_world_object(planning_scene, kObjectId);
        }
      } else if (owner == Owner::Left) {
        if (left_arm.detachObject(kObjectId)) {
          wait_for_world_object(planning_scene, kObjectId);
        }
      } else if (owner == Owner::Base) {
        if (both_arms.detachObject(kObjectId)) {
          wait_for_world_object(planning_scene, kObjectId);
        }
      }
      planning_scene.removeCollisionObjects({kObjectId});
      if (wait_for_object_absent(planning_scene, kObjectId)) {
        owner = Owner::None;
      }
    }
  }

  if (success) {
    RCLCPP_INFO(
      node->get_logger(),
      mode == "reset_only" ?
      "Interrupted-state reset completed; both arms and grippers are parked." :
      mode == "pose_test" ?
      "Bimanual airborne pose test completed successfully." :
      mode == "right_pick_test" ?
      "Right offset-pick physical test completed successfully." :
      mode == "cooperative_lift_test" ?
      "Synchronized cooperative table-lift test completed successfully." :
      mode == "cooperative_transfer_test" ?
      "Airborne cooperative transfer completed: the central receiver retained the block while "
      "the donor opened and retreated." :
      donor_release_test_mode ?
      "Serial donor-release test completed: the left COM grasp retained the block through "
      "millimetre donor unloading, right retreat, left-only hold and vertical table placement." :
      sequential_center_regrasp_test_mode ?
      "Serial center-regrasp test completed: right picked and carried centrally, left formed "
      "a COM grasp, and the right donor retained ownership throughout." :
      sequential_vision_mode ?
      "Serial vision handover completed: right lifted, D435 re-detected, left received and placed." :
      mode == "dual_contact_test" ?
      "Dual-contact test completed; the right gripper retained ownership throughout." :
      "Airborne right-to-left handover completed successfully without an intermediate place.");
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
