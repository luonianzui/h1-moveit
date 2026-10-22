#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

constexpr char kWorldFrame[] = "world";
constexpr char kTcpLink[] = "right_dex1_tcp";
constexpr char kCubeId[] = "green_grasp_cube";
constexpr double kCubeSize = 0.06;

bool plan_and_execute(
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & target,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_return");
  move_group.setStartStateToCurrentState();
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

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto error_code = move_group.execute(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed at stage '%s' with error code: %d",
                 stage_name.c_str(), error_code.val);
    return false;
  }

  RCLCPP_INFO(logger, "Stage completed: %s", stage_name.c_str());
  return true;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("h1_2_right_arm_return_demo", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  MoveGroupInterface right_arm(node, "right_arm");
  MoveGroupInterface right_gripper(node, "right_gripper");

  for (auto * group : {&right_arm, &right_gripper}) {
    group->setPlanningPipelineId("ompl");
    group->setPlannerId("RRTConnectkConfigDefault");
    group->setPlanningTime(10.0);
    group->setNumPlanningAttempts(8);
    group->setMaxVelocityScalingFactor(0.20);
    group->setMaxAccelerationScalingFactor(0.20);
  }
  right_arm.setPoseReferenceFrame(kWorldFrame);
  right_arm.setEndEffectorLink(kTcpLink);

  if (!right_arm.getCurrentState(15.0) || !right_gripper.getCurrentState(15.0)) {
    RCLCPP_ERROR(node->get_logger(), "No joint state received from ros2_control.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const std::map<std::string, double> open_gripper{
    {"right_dex1_finger_1_joint", -0.02},
    {"right_dex1_finger_2_joint", -0.02},
  };
  const std::map<std::string, double> closed_gripper{
    {"right_dex1_finger_1_joint", -0.005},
    {"right_dex1_finger_2_joint", -0.005},
  };

  // 从放置位置的姿态
  const std::map<std::string, double> at_place_pose{
    {"right_shoulder_pitch_joint", -0.30},
    {"right_shoulder_roll_joint", -0.18},
    {"right_shoulder_yaw_joint", -0.75},
    {"right_elbow_joint", 1.15},
    {"right_wrist_roll_joint", -0.05},
    {"right_wrist_pitch_joint", 0.15},
    {"right_wrist_yaw_joint", 0.30},
  };

  // 接近放置位置
  const std::map<std::string, double> approach_place{
    {"right_shoulder_pitch_joint", -0.30},
    {"right_shoulder_roll_joint", -0.24},
    {"right_shoulder_yaw_joint", -0.55},
    {"right_elbow_joint", 1.05},
    {"right_wrist_roll_joint", -0.05},
    {"right_wrist_pitch_joint", 0.12},
    {"right_wrist_yaw_joint", 0.25},
  };

  // 抬起（返回路径）
  const std::map<std::string, double> lift_return{
    {"right_shoulder_pitch_joint", -0.35},
    {"right_shoulder_roll_joint", -0.24},
    {"right_shoulder_yaw_joint", -0.55},
    {"right_elbow_joint", 1.05},
    {"right_wrist_roll_joint", -0.05},
    {"right_wrist_pitch_joint", 0.12},
    {"right_wrist_yaw_joint", 0.25},
  };

  // 转移回原位
  const std::map<std::string, double> transfer_back{
    {"right_shoulder_pitch_joint", -0.45},
    {"right_shoulder_roll_joint", -0.43},
    {"right_shoulder_yaw_joint", 0.10},
    {"right_elbow_joint", 0.90},
    {"right_wrist_roll_joint", -0.10},
    {"right_wrist_pitch_joint", 0.10},
    {"right_wrist_yaw_joint", 0.0},
  };

  // 原始抓取位置
  const std::map<std::string, double> original_grasp{
    {"right_shoulder_pitch_joint", -0.70},
    {"right_shoulder_roll_joint", -0.50},
    {"right_shoulder_yaw_joint", 0.18},
    {"right_elbow_joint", 1.15},
    {"right_wrist_roll_joint", -0.15},
    {"right_wrist_pitch_joint", 0.15},
    {"right_wrist_yaw_joint", 0.0},
  };

  // 撤离到预抓取
  const std::map<std::string, double> retreat_to_pregrasp{
    {"right_shoulder_pitch_joint", -0.50},
    {"right_shoulder_roll_joint", -0.48},
    {"right_shoulder_yaw_joint", 0.28},
    {"right_elbow_joint", 0.95},
    {"right_wrist_roll_joint", -0.18},
    {"right_wrist_pitch_joint", 0.10},
    {"right_wrist_yaw_joint", 0.0},
  };

  // 中立位置
  const std::map<std::string, double> neutral{
    {"right_shoulder_pitch_joint", 0.0},
    {"right_shoulder_roll_joint", 0.0},
    {"right_shoulder_yaw_joint", 0.0},
    {"right_elbow_joint", 0.0},
    {"right_wrist_roll_joint", 0.0},
    {"right_wrist_pitch_joint", 0.0},
    {"right_wrist_yaw_joint", 0.0},
  };

  RCLCPP_INFO(node->get_logger(), "开始将立方体放回原位...");
  bool success = true;

  // 1. 张开夹爪
  success = plan_and_execute(right_gripper, open_gripper, "open gripper");
  std::this_thread::sleep_for(500ms);

  // 2. 移动到接近放置位置
  if (success) {
    success = plan_and_execute(right_arm, approach_place, "approach place location");
    std::this_thread::sleep_for(700ms);
  }

  // 3. 下降到放置位置
  if (success) {
    success = plan_and_execute(right_arm, at_place_pose, "descend to place pose");
    std::this_thread::sleep_for(700ms);
  }

  // 4. 闭合夹爪抓取
  if (success) {
    success = plan_and_execute(right_gripper, closed_gripper, "close gripper to grasp");
    std::this_thread::sleep_for(1000ms);
  }

  // 5. 抬起立方体
  if (success) {
    success = plan_and_execute(right_arm, lift_return, "lift cube from place");
    std::this_thread::sleep_for(900ms);
  }

  // 6. 转移回原位
  if (success) {
    success = plan_and_execute(right_arm, transfer_back, "transfer back");
    std::this_thread::sleep_for(900ms);
  }

  // 7. 下降到原始位置
  if (success) {
    success = plan_and_execute(right_arm, original_grasp, "lower to original position");
    std::this_thread::sleep_for(1700ms);
  }

  // 8. 张开夹爪释放
  if (success) {
    success = plan_and_execute(right_gripper, open_gripper, "open gripper to release");
    std::this_thread::sleep_for(700ms);
  }

  // 9. 撤离
  if (success) {
    success = plan_and_execute(right_arm, retreat_to_pregrasp, "retreat from original position");
    std::this_thread::sleep_for(700ms);
  }

  // 10. 回到中立姿态
  if (success) {
    success = plan_and_execute(right_arm, neutral, "return to neutral");
  }

  if (success) {
    RCLCPP_INFO(node->get_logger(), "立方体已成功放回原位！");
  } else {
    RCLCPP_ERROR(node->get_logger(), "放回演示未完成");
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
