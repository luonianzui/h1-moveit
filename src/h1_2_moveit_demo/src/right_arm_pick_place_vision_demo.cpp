#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <optional>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

constexpr char kPlanningFrame[] = "pelvis";
constexpr double kDefaultObjectSizeX = 0.08;
constexpr double kDefaultObjectSizeY = 0.16;
constexpr double kDefaultObjectSizeZ = 0.03;
constexpr double kCartesianEefStep = 0.005;
constexpr double kMinCartesianFraction = 0.995;
constexpr double kDefaultVelocityScaling = 0.50;
constexpr double kDefaultAccelerationScaling = 0.50;
constexpr auto kDetectionPollPeriod = 100ms;

// 全局变量存储检测到的方块位置
std::optional<geometry_msgs::msg::PoseStamped> g_detected_cube_pose;
std::mutex g_cube_pose_mutex;

void cube_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(g_cube_pose_mutex);
  g_detected_cube_pose = *msg;
}

bool set_bounded_start_state(
  MoveGroupInterface & move_group,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_pick_place_vision");
  auto start_state = move_group.getCurrentState(10.0);
  if (!start_state) {
    RCLCPP_ERROR(logger, "No current state is available for stage '%s'.", stage_name.c_str());
    return false;
  }

  // A trajectory can finish numerically a few ulps outside a URDF joint
  // limit.  Clamp the planning copy just inside every active joint bound so a
  // later invocation can safely start from the previous retreat pose.
  constexpr double kJointLimitMargin = 1e-6;
  for (const auto & joint : move_group.getActiveJoints()) {
    const auto & bounds = start_state->getRobotModel()->getVariableBounds(joint);
    if (!bounds.position_bounded_) {
      continue;
    }
    const double span = bounds.max_position_ - bounds.min_position_;
    const double margin = std::min(kJointLimitMargin, span * 0.25);
    const double bounded_position = std::clamp(
      start_state->getVariablePosition(joint),
      bounds.min_position_ + margin,
      bounds.max_position_ - margin);
    start_state->setVariablePosition(joint, bounded_position);
  }
  start_state->update();

  moveit_msgs::msg::RobotState start_state_msg;
  start_state_msg.is_diff = true;
  for (const auto & joint : move_group.getActiveJoints()) {
    start_state_msg.joint_state.name.push_back(joint);
    start_state_msg.joint_state.position.push_back(
      start_state->getVariablePosition(joint));
  }
  move_group.setStartState(start_state_msg);
  return true;
}

bool plan_and_execute_pose(
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_pick_place_vision");
  if (!set_bounded_start_state(move_group, stage_name)) {
    return false;
  }
  move_group.setPoseTarget(target);

  // Keep planning and execution separate: the combined move() action ignores
  // an explicitly supplied start state, which makes a previous trajectory's
  // sub-microradian limit overshoot impossible to recover from.
  MoveGroupInterface::Plan plan;
  RCLCPP_INFO(logger, "Planning stage: %s", stage_name.c_str());
  if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning failed at stage '%s'.", stage_name.c_str());
    return false;
  }
  if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed at stage '%s'.", stage_name.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "Stage completed: %s", stage_name.c_str());
  return true;
}

bool plan_and_execute_cartesian(
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target,
  const std::string & stage_name,
  const bool fallback_to_pose_planning = false)
{
  const auto logger = rclcpp::get_logger("h1_2_pick_place_vision");
  if (!set_bounded_start_state(move_group, stage_name)) {
    return false;
  }
  move_group.clearPoseTargets();

  // Interpolate the TCP directly from its current pose to the target.  Unlike
  // an OMPL pose goal, every intermediate Cartesian sample is solved by IK and
  // collision checked, so vertical approach / lift / retreat remain straight.
  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  const std::vector<geometry_msgs::msg::Pose> waypoints{target};

  RCLCPP_INFO(
    logger, "Planning Cartesian stage: %s (eef_step=%.3f m)",
    stage_name.c_str(), kCartesianEefStep);
  const double fraction = move_group.computeCartesianPath(
    waypoints, kCartesianEefStep, trajectory, true, &error_code);

  const auto point_count = trajectory.joint_trajectory.points.size();
  if (fraction < kMinCartesianFraction || point_count == 0) {
    if (fallback_to_pose_planning) {
      RCLCPP_WARN(
        logger,
        "Cartesian stage '%s' solved only %.1f%% (%zu points, error=%d); "
        "falling back to collision-aware OMPL before executing any partial path.",
        stage_name.c_str(), fraction * 100.0, point_count, error_code.val);
      return plan_and_execute_pose(move_group, target, stage_name + " (OMPL fallback)");
    }
    RCLCPP_ERROR(
      logger,
      "Cartesian planning failed at stage '%s': fraction=%.1f%%, points=%zu, error=%d.",
      stage_name.c_str(), fraction * 100.0, point_count, error_code.val);
    return false;
  }

  RCLCPP_INFO(
    logger,
    "Cartesian stage '%s' solved %.1f%% with %zu trajectory points; executing...",
    stage_name.c_str(), fraction * 100.0, point_count);
  if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed at Cartesian stage '%s'.", stage_name.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Cartesian stage completed: %s", stage_name.c_str());
  return true;
}

bool plan_and_execute(
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & target,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_pick_place_vision");
  auto start_state = move_group.getCurrentState(10.0);
  if (!start_state) {
    RCLCPP_ERROR(logger, "No current state is available for stage '%s'.", stage_name.c_str());
    return false;
  }

  // Gazebo contact can push an open finger a few picometres past the exact
  // -0.02 URDF boundary.  Keep the planning copy just inside the limit; the
  // resulting sub-micrometre correction is far below execution tolerance.
  constexpr double kFingerLowerBoundMargin = -0.019999;
  for (const auto & joint : move_group.getActiveJoints()) {
    if (joint.find("_dex1_finger_") == std::string::npos) {
      continue;
    }
    if (start_state->getVariablePosition(joint) < kFingerLowerBoundMargin) {
      start_state->setVariablePosition(joint, kFingerLowerBoundMargin);
    }
  }
  start_state->enforceBounds();
  start_state->update();

  // Send a differential start state for this planning group so joints from
  // the other arm / gripper are not serialized into the request.
  moveit_msgs::msg::RobotState start_state_msg;
  start_state_msg.is_diff = true;
  for (const auto & joint : move_group.getActiveJoints()) {
    start_state_msg.joint_state.name.push_back(joint);
    start_state_msg.joint_state.position.push_back(
      start_state->getVariablePosition(joint));
  }
  move_group.setStartState(start_state_msg);
  if (!move_group.setJointValueTarget(target)) {
    RCLCPP_ERROR(logger, "Stage '%s' violates a joint limit.", stage_name.c_str());
    return false;
  }

  MoveGroupInterface::Plan plan;
  RCLCPP_INFO(logger, "Planning stage: %s", stage_name.c_str());
  if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning failed at stage '%s'.", stage_name.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger, "Stage '%s' planned with %zu trajectory points; executing...",
    stage_name.c_str(), plan.trajectory.joint_trajectory.points.size());

  auto error_code = move_group.execute(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed at stage '%s' with error code: %d",
                 stage_name.c_str(), error_code.val);
    return false;
  }

  RCLCPP_INFO(logger, "Stage completed: %s", stage_name.c_str());
  return true;
}

moveit_msgs::msg::CollisionObject make_grasp_object(
  const geometry_msgs::msg::Pose & pose,
  const std::string & object_id,
  const double size_x,
  const double size_y,
  const double size_z)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = kPlanningFrame;
  object.id = object_id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size_x, size_y, size_z};
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  options.append_parameter_override("use_sim_time", true);
  auto node = rclcpp::Node::make_shared("h1_2_arm_pick_place_vision_demo", options);

  std::string arm_side = "right";
  if (node->has_parameter("arm_side")) {
    node->get_parameter("arm_side", arm_side);
  } else {
    arm_side = node->declare_parameter<std::string>("arm_side", "right");
  }
  if (arm_side != "right" && arm_side != "left") {
    RCLCPP_ERROR(node->get_logger(), "arm_side must be 'right' or 'left', got '%s'.",
      arm_side.c_str());
    rclcpp::shutdown();
    return 2;
  }

  const auto get_double_parameter = [&node](
    const std::string & name, const double default_value)
    {
      double value = default_value;
      if (node->has_parameter(name)) {
        const auto parameter = node->get_parameter(name);
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          value = parameter.as_double();
        } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          value = static_cast<double>(parameter.as_int());
        } else {
          value = std::numeric_limits<double>::quiet_NaN();
        }
      } else {
        value = node->declare_parameter<double>(name, default_value);
      }
      return value;
    };
  const double velocity_scaling = get_double_parameter(
    "velocity_scaling", kDefaultVelocityScaling);
  const double acceleration_scaling = get_double_parameter(
    "acceleration_scaling", kDefaultAccelerationScaling);
  if (!std::isfinite(velocity_scaling) || velocity_scaling <= 0.0 || velocity_scaling > 1.0 ||
    !std::isfinite(acceleration_scaling) || acceleration_scaling <= 0.0 ||
    acceleration_scaling > 1.0)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "velocity_scaling and acceleration_scaling must be in (0, 1]; got %.3f and %.3f.",
      velocity_scaling, acceleration_scaling);
    rclcpp::shutdown();
    return 2;
  }

  const bool is_left = arm_side == "left";
  const double place_x = get_double_parameter("place_x", 0.25);
  const double place_y = get_double_parameter("place_y", is_left ? 0.35 : -0.35);
  const double pick_z_min = get_double_parameter("pick_z_min", -0.16);
  const double pick_z_max = get_double_parameter("pick_z_max", -0.06);
  const double detection_max_spread = get_double_parameter("detection_max_spread", 0.008);
  const double vertical_clearance = get_double_parameter("vertical_clearance", 0.10);
  const double detection_max_age = get_double_parameter("detection_max_age", 0.50);
  const double detection_timeout = get_double_parameter("detection_timeout", 30.0);
  const double pick_x_min = get_double_parameter("pick_x_min", 0.20);
  const double pick_x_max = get_double_parameter("pick_x_max", 0.40);
  const double pick_y_min = get_double_parameter("pick_y_min", is_left ? -0.05 : -0.40);
  const double pick_y_max = get_double_parameter("pick_y_max", is_left ? 0.40 : 0.05);
  const double gripper_close_velocity_scaling = get_double_parameter(
    "gripper_close_velocity_scaling", 0.25);
  const double gripper_close_acceleration_scaling = get_double_parameter(
    "gripper_close_acceleration_scaling", 0.25);
  const double object_size_x = get_double_parameter("object_size_x", kDefaultObjectSizeX);
  const double object_size_y = get_double_parameter("object_size_y", kDefaultObjectSizeY);
  const double object_size_z = get_double_parameter("object_size_z", kDefaultObjectSizeZ);
  const double gripper_soft_contact_position = get_double_parameter(
    "gripper_soft_contact_position", 0.001);
  const double gripper_preload_position = get_double_parameter(
    "gripper_preload_position", 0.003);
  const double grasp_z_offset = get_double_parameter("grasp_z_offset", 0.020);
  int64_t detection_sample_count = 3;
  if (node->has_parameter("detection_sample_count")) {
    const auto parameter = node->get_parameter("detection_sample_count");
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
      RCLCPP_ERROR(node->get_logger(), "detection_sample_count must be an integer.");
      rclcpp::shutdown();
      return 2;
    }
    detection_sample_count = parameter.as_int();
  } else {
    detection_sample_count = node->declare_parameter<int64_t>("detection_sample_count", 3);
  }
  bool park_after_retreat = false;
  if (node->has_parameter("park_after_retreat")) {
    const auto parameter = node->get_parameter("park_after_retreat");
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
      RCLCPP_ERROR(node->get_logger(), "park_after_retreat must be a boolean.");
      rclcpp::shutdown();
      return 2;
    }
    park_after_retreat = parameter.as_bool();
  } else {
    park_after_retreat = node->declare_parameter<bool>("park_after_retreat", false);
  }
  bool park_only = false;
  if (node->has_parameter("park_only")) {
    const auto parameter = node->get_parameter("park_only");
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
      RCLCPP_ERROR(node->get_logger(), "park_only must be a boolean.");
      rclcpp::shutdown();
      return 2;
    }
    park_only = parameter.as_bool();
  } else {
    park_only = node->declare_parameter<bool>("park_only", false);
  }
  if (!std::isfinite(place_x) || !std::isfinite(place_y) ||
    !std::isfinite(pick_z_min) || !std::isfinite(pick_z_max) ||
    !std::isfinite(detection_max_spread) || detection_max_spread <= 0.0 ||
    !std::isfinite(vertical_clearance) || vertical_clearance < 0.08 ||
    vertical_clearance > 0.30 ||
    !std::isfinite(detection_max_age) || detection_max_age <= 0.0 ||
    !std::isfinite(detection_timeout) || detection_timeout <= 0.0 ||
    !std::isfinite(pick_x_min) || !std::isfinite(pick_x_max) || pick_x_min >= pick_x_max ||
    !std::isfinite(pick_y_min) || !std::isfinite(pick_y_max) || pick_y_min >= pick_y_max ||
    !std::isfinite(gripper_close_velocity_scaling) ||
    gripper_close_velocity_scaling <= 0.0 || gripper_close_velocity_scaling > 1.0 ||
    !std::isfinite(gripper_close_acceleration_scaling) ||
    gripper_close_acceleration_scaling <= 0.0 || gripper_close_acceleration_scaling > 1.0 ||
    !std::isfinite(object_size_x) || object_size_x <= 0.0 || object_size_x > 0.50 ||
    !std::isfinite(object_size_y) || object_size_y <= 0.0 || object_size_y > 0.50 ||
    !std::isfinite(object_size_z) || object_size_z <= 0.0 || object_size_z > 0.50 ||
    !std::isfinite(gripper_soft_contact_position) ||
    gripper_soft_contact_position < -0.019 || gripper_soft_contact_position > 0.0245 ||
    !std::isfinite(gripper_preload_position) ||
    gripper_preload_position <= gripper_soft_contact_position ||
    gripper_preload_position > 0.0245 ||
    !std::isfinite(grasp_z_offset) || grasp_z_offset < -0.10 || grasp_z_offset > 0.10 ||
    pick_z_min >= pick_z_max || detection_sample_count < 1 || detection_sample_count > 10)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid placement/detection parameters: place=[%.3f, %.3f], z=[%.3f, %.3f], "
      "samples=%ld, max_spread=%.4f, clearance=%.3f, object=[%.3f, %.3f, %.3f].",
      place_x, place_y, pick_z_min, pick_z_max,
      static_cast<long>(detection_sample_count), detection_max_spread, vertical_clearance,
      object_size_x, object_size_y, object_size_z);
    rclcpp::shutdown();
    return 2;
  }
  const std::string arm_group = arm_side + "_arm";
  const std::string gripper_group = arm_side + "_gripper";
  const std::string tcp_link = arm_side + "_dex1_tcp";
  const std::string cube_id = "green_grasp_cube_" + arm_side;
  const std::string finger_1_joint = arm_side + "_dex1_finger_1_joint";
  const std::string finger_2_joint = arm_side + "_dex1_finger_2_joint";
  const std::map<std::string, double> parked_arm{
    {arm_side + "_shoulder_pitch_joint", 0.0},
    {arm_side + "_shoulder_roll_joint", is_left ? 0.25 : -0.25},
    {arm_side + "_shoulder_yaw_joint", 0.0},
    // A flexed 0.60 rad elbow leaves the hand beside the old 60 mm cube, but
    // overlaps the 200 mm handover block and can slowly tip it.  Let the arm
    // hang straight at its side so both the camera view and Gazebo body are
    // clear of the enlarged object.
    {arm_side + "_elbow_joint", 0.0},
    {arm_side + "_wrist_roll_joint", 0.0},
    {arm_side + "_wrist_pitch_joint", 0.0},
    {arm_side + "_wrist_yaw_joint", 0.0},
  };

  // 订阅视觉检测话题
  auto cube_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/cube_pose", 10, cube_pose_callback);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  // TF2 buffer for coordinate transformation
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  MoveGroupInterface arm(node, arm_group);
  MoveGroupInterface gripper(node, gripper_group);
  moveit::planning_interface::PlanningSceneInterface planning_scene;

  for (auto * group : {&arm, &gripper}) {
    group->setPlanningPipelineId("ompl");
    group->setPlannerId("RRTConnectkConfigDefault");
    group->setPlanningTime(10.0);
    group->setNumPlanningAttempts(8);
    group->setMaxVelocityScalingFactor(velocity_scaling);
    group->setMaxAccelerationScalingFactor(acceleration_scaling);
  }
  arm.setPoseReferenceFrame(kPlanningFrame);
  arm.setEndEffectorLink(tcp_link);

  if (!arm.getCurrentState(15.0) || !gripper.getCurrentState(15.0)) {
    RCLCPP_ERROR(node->get_logger(), "No joint state received from ros2_control.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  if (park_only) {
    const bool parked = plan_and_execute(arm, parked_arm, "park arm at side");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return parked ? 0 : 1;
  }

  const std::map<std::string, double> open_gripper{
    {finger_1_joint, -0.019},
    {finger_2_joint, -0.019},
  };
  const std::map<std::string, double> closed_gripper{
    // The 50 mm grasp thickness contacts near 0.000. Command farther inward
    // so Gazebo's position controller maintains physical preload.
    {finger_1_joint, gripper_preload_position},
    {finger_2_joint, gripper_preload_position},
  };
  const std::map<std::string, double> soft_contact_gripper{
    {finger_1_joint, gripper_soft_contact_position},
    {finger_2_joint, gripper_soft_contact_position},
  };

  // 等待视觉检测
  RCLCPP_INFO(node->get_logger(), "等待视觉系统检测方块位置...");
  geometry_msgs::msg::Pose cube_pose_world;
  bool cube_detected = false;
  std::optional<int64_t> last_detection_stamp;
  std::vector<geometry_msgs::msg::Pose> detection_samples;

  const auto detection_deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(detection_timeout));
  while (std::chrono::steady_clock::now() < detection_deadline && !cube_detected) {
    std::this_thread::sleep_for(kDetectionPollPeriod);

    std::optional<geometry_msgs::msg::PoseStamped> detected_cube_pose;
    {
      std::lock_guard<std::mutex> lock(g_cube_pose_mutex);
      detected_cube_pose = g_detected_cube_pose;
    }
    if (detected_cube_pose.has_value()) {
      const int64_t detection_stamp =
        rclcpp::Time(detected_cube_pose->header.stamp).nanoseconds();
      if (last_detection_stamp.has_value() && detection_stamp == *last_detection_stamp) {
        continue;
      }
      last_detection_stamp = detection_stamp;

      const double sample_age =
        (node->now() - rclcpp::Time(detected_cube_pose->header.stamp)).seconds();
      if (sample_age < 0.0 || sample_age > detection_max_age) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "Ignoring stale visual sample aged %.3fs; maximum is %.3fs.",
          sample_age, detection_max_age);
        continue;
      }

      // Gazebo world is 1 m below the fixed-base MoveIt model. Transform the
      // visual target into the shared pelvis frame before planning.
      try {
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf_buffer->transform(*detected_cube_pose, transformed_pose, kPlanningFrame,
                            tf2::durationFromSec(0.1));
        const double candidate_z = transformed_pose.pose.position.z;
        if (candidate_z < pick_z_min || candidate_z > pick_z_max) {
          RCLCPP_WARN_THROTTLE(
            node->get_logger(), *node->get_clock(), 2000,
            "Ignoring non-tabletop visual sample at z=%.3f; expected [%.3f, %.3f].",
            candidate_z, pick_z_min, pick_z_max);
          continue;
        }
        const double candidate_x = transformed_pose.pose.position.x;
        const double candidate_y = transformed_pose.pose.position.y;
        if (candidate_x < pick_x_min || candidate_x > pick_x_max ||
          candidate_y < pick_y_min || candidate_y > pick_y_max)
        {
          RCLCPP_WARN_THROTTLE(
            node->get_logger(), *node->get_clock(), 2000,
            "Ignoring visual sample outside %s-hand workspace: [%.3f, %.3f], "
            "expected x=[%.3f, %.3f], y=[%.3f, %.3f].",
            is_left ? "left" : "right", candidate_x, candidate_y,
            pick_x_min, pick_x_max, pick_y_min, pick_y_max);
          continue;
        }
        detection_samples.push_back(transformed_pose.pose);
        if (detection_samples.size() > static_cast<size_t>(detection_sample_count)) {
          detection_samples.erase(detection_samples.begin());
        }

        if (detection_samples.size() == static_cast<size_t>(detection_sample_count)) {
          std::vector<double> xs;
          std::vector<double> ys;
          std::vector<double> zs;
          xs.reserve(detection_samples.size());
          ys.reserve(detection_samples.size());
          zs.reserve(detection_samples.size());
          for (const auto & sample : detection_samples) {
            xs.push_back(sample.position.x);
            ys.push_back(sample.position.y);
            zs.push_back(sample.position.z);
          }
          const auto [x_min, x_max] = std::minmax_element(xs.begin(), xs.end());
          const auto [y_min, y_max] = std::minmax_element(ys.begin(), ys.end());
          const auto [z_min, z_max] = std::minmax_element(zs.begin(), zs.end());
          const double spread = std::hypot(
            std::hypot(*x_max - *x_min, *y_max - *y_min), *z_max - *z_min);

          if (spread <= detection_max_spread) {
            const auto median = [](std::vector<double> values) {
                const auto middle = values.begin() + values.size() / 2;
                std::nth_element(values.begin(), middle, values.end());
                return *middle;
              };
            cube_pose_world = transformed_pose.pose;
            cube_pose_world.position.x = median(xs);
            cube_pose_world.position.y = median(ys);
            cube_pose_world.position.z = median(zs);
            // The detector estimates only the object center; its identity
            // quaternion is expressed in the optical frame and therefore
            // becomes the camera mounting rotation after TF.  The Gazebo
            // handover block is spawned axis-aligned in the planning frame,
            // so keep the collision box axis-aligned instead of treating the
            // camera-frame orientation as an object orientation estimate.
            cube_pose_world.orientation.x = 0.0;
            cube_pose_world.orientation.y = 0.0;
            cube_pose_world.orientation.z = 0.0;
            cube_pose_world.orientation.w = 1.0;
            cube_detected = true;

            RCLCPP_INFO(
              node->get_logger(),
              "方块稳定检测成功（%ld帧，散差 %.1fmm）：[%.3f, %.3f, %.3f]",
              static_cast<long>(detection_sample_count), spread * 1000.0,
              cube_pose_world.position.x,
              cube_pose_world.position.y,
              cube_pose_world.position.z);
          } else {
            RCLCPP_WARN_THROTTLE(
              node->get_logger(), *node->get_clock(), 2000,
              "Visual samples are not stable yet: spread %.1fmm exceeds %.1fmm.",
              spread * 1000.0, detection_max_spread * 1000.0);
          }
        }
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node->get_logger(), "TF 转换失败: %s", ex.what());
      }
    }
  }

  if (!cube_detected) {
    RCLCPP_ERROR(
      node->get_logger(), "%.1f秒内未获得新鲜、稳定且位于工作区内的方块位置，演示中止。",
      detection_timeout);
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  // The gripper approach axis is local +Y; rotating -90 degrees about X
  // points that axis down at the table.  Keep the TCP above the geometric
  // center so the pads contact near the upper part of the tall block.
  geometry_msgs::msg::Pose grasp_pose = cube_pose_world;
  // Keep the contact band 30 mm below the top face, matching the proven
  // 60 mm cube geometry while the planning collision object remains centered.
  grasp_pose.position.z += grasp_z_offset;
  grasp_pose.orientation.x = -0.7071067811865476;
  grasp_pose.orientation.y = 0.0;
  grasp_pose.orientation.z = 0.0;
  grasp_pose.orientation.w = 0.7071067811865476;

  // 预抓取姿态（抓取上方 10cm）
  geometry_msgs::msg::Pose pregrasp_pose = grasp_pose;
  pregrasp_pose.position.z += vertical_clearance;

  // 放置到同一桌面上的另一个已验证可达位置。
  geometry_msgs::msg::Pose object_place_pose;
  object_place_pose.position.x = place_x;
  object_place_pose.position.y = place_y;
  object_place_pose.position.z = cube_pose_world.position.z;
  object_place_pose.orientation.w = 1.0;
  geometry_msgs::msg::Pose place_pose = object_place_pose;
  place_pose.position.z += grasp_z_offset;
  place_pose.orientation = grasp_pose.orientation;

  RCLCPP_INFO(
    node->get_logger(),
    "开始执行%s手抓取任务（速度 %.0f%%，加速度 %.0f%%）...",
    is_left ? "左" : "右", velocity_scaling * 100.0, acceleration_scaling * 100.0);
  const auto task_start_time = std::chrono::steady_clock::now();

  bool success = true;
  bool collision_object_added = false;
  bool collision_object_attached = false;

  // 1. 张开夹爪
  if (success) {
    success = plan_and_execute(gripper, open_gripper, "open gripper");
  }

  // 2. 移动到预抓取位置
  if (success) {
    success = plan_and_execute_pose(arm, pregrasp_pose, "move to pregrasp");
    if (success) {
      RCLCPP_INFO(node->get_logger(), "移动到预抓取位置完成");
    } else {
      RCLCPP_ERROR(node->get_logger(), "移动到预抓取位置失败");
    }
  }

  // 3. 下降到抓取位置
  if (success) {
    success = plan_and_execute_cartesian(arm, grasp_pose, "descend to grasp");
    if (success) {
      RCLCPP_INFO(node->get_logger(), "下降到抓取位置完成");
    } else {
      RCLCPP_ERROR(node->get_logger(), "下降到抓取位置失败");
    }
    // Give Gazebo contact a short moment to settle before closing the fingers.
    std::this_thread::sleep_for(400ms);
  }

  // 4. 闭合夹爪并附加方块
  if (success) {
    const auto cube = make_grasp_object(
      cube_pose_world, cube_id, object_size_x, object_size_y, object_size_z);
    std_msgs::msg::ColorRGBA cube_color;
    cube_color.r = 0.10F;
    cube_color.g = 0.95F;
    cube_color.b = 0.15F;
    cube_color.a = 1.0F;
    success = planning_scene.applyCollisionObject(cube, cube_color);
    collision_object_added = success;

    if (success) {
      success = arm.attachObject(
        cube_id, tcp_link,
      {
        arm_side + "_wrist_yaw_link",
        arm_side + "_dex1_adapter_link",
        arm_side + "_dex1_base_link",
        arm_side + "_dex1_finger_1_slider_link",
        arm_side + "_dex1_finger_1_link",
        arm_side + "_dex1_finger_1_pad_link",
        arm_side + "_dex1_finger_2_slider_link",
        arm_side + "_dex1_finger_2_link",
        arm_side + "_dex1_finger_2_pad_link",
        tcp_link,
        });
      collision_object_attached = success;
    }

    if (success) {
      gripper.setMaxVelocityScalingFactor(gripper_close_velocity_scaling);
      gripper.setMaxAccelerationScalingFactor(gripper_close_acceleration_scaling);
      success = plan_and_execute(gripper, soft_contact_gripper, "close to soft contact");
      if (success) {
        std::this_thread::sleep_for(300ms);
        success = plan_and_execute(gripper, closed_gripper, "apply grasp preload");
      }
      // Preserve physical preload settling; this wait is contact-critical.
      if (success) {
        std::this_thread::sleep_for(700ms);
      }
      gripper.setMaxVelocityScalingFactor(velocity_scaling);
      gripper.setMaxAccelerationScalingFactor(acceleration_scaling);
    }
  }

  // 5. 抬起方块
  if (success) {
    geometry_msgs::msg::Pose lift_pose = grasp_pose;
    lift_pose.position.z += vertical_clearance;
    success = plan_and_execute_cartesian(arm, lift_pose, "lift cube");
    if (success) {
      RCLCPP_INFO(node->get_logger(), "抬起方块完成");
    }
  }

  // 6. 移动到放置位置上方
  if (success) {
    geometry_msgs::msg::Pose pre_place_pose = place_pose;
    pre_place_pose.position.z += vertical_clearance;
    success = plan_and_execute_cartesian(
      arm, pre_place_pose, "straight transfer above place", true);
    if (success) {
      RCLCPP_INFO(node->get_logger(), "移动到放置位置上方完成");
    }
  }

  // 7. 下降到放置位置
  if (success) {
    success = plan_and_execute_cartesian(arm, place_pose, "descend to place");
    if (success) {
      RCLCPP_INFO(node->get_logger(), "下降到放置位置完成");
    }
    // Let the cube settle on the table before opening the gripper.
    std::this_thread::sleep_for(500ms);
  }

  // 8. 张开夹爪释放
  if (success) {
    success = plan_and_execute(gripper, open_gripper, "open gripper to release");
    if (success) {
      success = arm.detachObject(cube_id);
      if (success) {
        collision_object_attached = false;
      }
      std::this_thread::sleep_for(300ms);
    }
    if (success) {
      planning_scene.removeCollisionObjects({cube_id});
      collision_object_added = false;
    }
  }

  // 9. 撤离
  if (success) {
    geometry_msgs::msg::Pose retreat_pose = place_pose;
    retreat_pose.position.z += vertical_clearance;
    success = plan_and_execute_cartesian(arm, retreat_pose, "retreat");
    if (success) {
      RCLCPP_INFO(node->get_logger(), "撤离完成");
    }
  }

  // Once the gripper is vertically clear, restore the released cube as a world
  // collision object so OMPL can choose a short park path without sweeping the
  // physical handoff object.  Keeping it during release would make the contact
  // start state invalid, so it is re-added only after the straight retreat.
  if (success && park_after_retreat) {
    const auto handoff_cube = make_grasp_object(
      object_place_pose, cube_id, object_size_x, object_size_y, object_size_z);
    std_msgs::msg::ColorRGBA cube_color;
    cube_color.r = 0.10F;
    cube_color.g = 0.95F;
    cube_color.b = 0.15F;
    cube_color.a = 1.0F;
    success = planning_scene.applyCollisionObject(handoff_cube, cube_color);
    collision_object_added = success;
  }
  if (success && park_after_retreat) {
    success = plan_and_execute(arm, parked_arm, "collision-aware park after handoff");
    if (success) {
      RCLCPP_INFO(node->get_logger(), "已绕开交接方块停靠，相机视线已让出");
    }
  }
  if (success && park_after_retreat && collision_object_added) {
    planning_scene.removeCollisionObjects({cube_id});
    collision_object_added = false;
  }

  // A failed close / lift / transfer must not poison the next invocation with
  // a stale attached object or a world collision object.  Keep cleanup on the
  // common exit path and perform it while move_group services are still alive.
  if (collision_object_attached) {
    if (!arm.detachObject(cube_id)) {
      RCLCPP_WARN(
        node->get_logger(),
        "Failed to detach stale planning-scene object '%s' during cleanup.",
        cube_id.c_str());
    }
  }
  if (collision_object_added) {
    planning_scene.removeCollisionObjects({cube_id});
  }

  if (success) {
    const double elapsed_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - task_start_time).count();
    RCLCPP_INFO(
      node->get_logger(),
      "%s手视觉引导抓取演示成功完成！墙钟耗时 %.1f 秒。",
      is_left ? "左" : "右", elapsed_seconds);
  } else {
    RCLCPP_ERROR(node->get_logger(), "视觉引导抓取演示未完成。");
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
