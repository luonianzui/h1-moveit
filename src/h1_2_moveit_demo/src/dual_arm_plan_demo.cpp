#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

bool plan_and_execute(
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & target,
  const std::string & motion_name)
{
  const auto logger = rclcpp::get_logger("h1_2_dual_arm_demo");

  move_group.setStartStateToCurrentState();
  if (!move_group.setJointValueTarget(target)) {
    RCLCPP_ERROR(logger, "Target '%s' violates a joint limit.", motion_name.c_str());
    return false;
  }

  MoveGroupInterface::Plan plan;
  RCLCPP_INFO(logger, "Planning '%s' for both H1-2 arms...", motion_name.c_str());
  const bool planned =
    move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  if (!planned) {
    RCLCPP_ERROR(logger, "Planning failed for '%s'.", motion_name.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger, "Plan '%s' succeeded with %zu trajectory points; executing...",
    motion_name.c_str(), plan.trajectory.joint_trajectory.points.size());
  const bool executed =
    move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  if (!executed) {
    RCLCPP_ERROR(logger, "Execution failed for '%s'.", motion_name.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Motion '%s' completed.", motion_name.c_str());
  return true;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("h1_2_dual_arm_demo", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  MoveGroupInterface both_arms(node, "both_arms");
  both_arms.setPlanningPipelineId("ompl");
  both_arms.setPlannerId("RRTConnectkConfigDefault");
  both_arms.setPlanningTime(10.0);
  both_arms.setNumPlanningAttempts(5);
  both_arms.setMaxVelocityScalingFactor(0.25);
  both_arms.setMaxAccelerationScalingFactor(0.25);

  if (!both_arms.getCurrentState(15.0)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No joint state received from ros2_control within 15 seconds.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const std::map<std::string, double> reach_forward{
    {"left_shoulder_pitch_joint", -0.65},
    {"left_shoulder_roll_joint", 0.45},
    {"left_shoulder_yaw_joint", -0.15},
    {"left_elbow_joint", 1.05},
    {"left_wrist_roll_joint", 0.20},
    {"left_wrist_pitch_joint", 0.20},
    {"left_wrist_yaw_joint", 0.0},
    {"right_shoulder_pitch_joint", -0.65},
    {"right_shoulder_roll_joint", -0.45},
    {"right_shoulder_yaw_joint", 0.15},
    {"right_elbow_joint", 1.05},
    {"right_wrist_roll_joint", -0.20},
    {"right_wrist_pitch_joint", 0.20},
    {"right_wrist_yaw_joint", 0.0},
  };

  const std::map<std::string, double> neutral{
    {"left_shoulder_pitch_joint", 0.0},
    {"left_shoulder_roll_joint", 0.25},
    {"left_shoulder_yaw_joint", 0.0},
    {"left_elbow_joint", 0.60},
    {"left_wrist_roll_joint", 0.0},
    {"left_wrist_pitch_joint", 0.0},
    {"left_wrist_yaw_joint", 0.0},
    {"right_shoulder_pitch_joint", 0.0},
    {"right_shoulder_roll_joint", -0.25},
    {"right_shoulder_yaw_joint", 0.0},
    {"right_elbow_joint", 0.60},
    {"right_wrist_roll_joint", 0.0},
    {"right_wrist_pitch_joint", 0.0},
    {"right_wrist_yaw_joint", 0.0},
  };

  bool success = plan_and_execute(both_arms, reach_forward, "symmetric forward reach");
  if (success) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    success = plan_and_execute(both_arms, neutral, "return to neutral");
  }

  if (success) {
    RCLCPP_INFO(
      node->get_logger(),
      "H1-2 dual-arm MoveIt planning and mock execution demo finished successfully.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "H1-2 dual-arm demo did not complete.");
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
