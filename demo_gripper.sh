#!/bin/bash
# Dex1-1 夹爪慢速开合演示脚本
# 此脚本演示 Dex1-1 双指夹爪从完全张开（-0.020 m）到抓取位置（-0.005 m）的运动

set -e

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 设置 ROS 环境
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "=========================================="
echo "  Dex1-1 双指夹爪开合演示"
echo "=========================================="
echo ""
echo "此演示将展示右手 Dex1-1 夹爪的开合动作："
echo "  - 张开位置: -0.020 m"
echo "  - 闭合位置: -0.005 m（抓取 60mm 立方体）"
echo "  - 每个动作持续 3 秒"
echo ""
echo "请观察 RViz 窗口中右手夹爪的运动"
echo "=========================================="
echo ""

sleep 2

echo "[1/5] 初始化：将夹爪张开到 -0.020 m"
ros2 action send_goal /right_gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [right_dex1_finger_1_joint, right_dex1_finger_2_joint]
  points:
  - positions: [-0.020, -0.020]
    time_from_start:
      sec: 3
      nanosec: 0
" > /dev/null 2>&1

echo "   ✓ 夹爪已张开"
sleep 2

echo ""
echo "[2/5] 闭合夹爪到 -0.005 m"
ros2 action send_goal /right_gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [right_dex1_finger_1_joint, right_dex1_finger_2_joint]
  points:
  - positions: [-0.005, -0.005]
    time_from_start:
      sec: 3
      nanosec: 0
" > /dev/null 2>&1

echo "   ✓ 夹爪已闭合（模拟抓取）"
sleep 2

echo ""
echo "[3/5] 重新张开到 -0.020 m"
ros2 action send_goal /right_gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [right_dex1_finger_1_joint, right_dex1_finger_2_joint]
  points:
  - positions: [-0.020, -0.020]
    time_from_start:
      sec: 3
      nanosec: 0
" > /dev/null 2>&1

echo "   ✓ 夹爪已张开（模拟释放）"
sleep 2

echo ""
echo "[4/5] 再次闭合到 -0.005 m"
ros2 action send_goal /right_gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [right_dex1_finger_1_joint, right_dex1_finger_2_joint]
  points:
  - positions: [-0.005, -0.005]
    time_from_start:
      sec: 3
      nanosec: 0
" > /dev/null 2>&1

echo "   ✓ 夹爪已闭合"
sleep 2

echo ""
echo "[5/5] 恢复到初始张开位置"
ros2 action send_goal /right_gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [right_dex1_finger_1_joint, right_dex1_finger_2_joint]
  points:
  - positions: [-0.020, -0.020]
    time_from_start:
      sec: 3
      nanosec: 0
" > /dev/null 2>&1

echo "   ✓ 夹爪已恢复到张开状态"
echo ""
echo "=========================================="
echo "  演示完成！"
echo "=========================================="
echo ""
echo "夹爪已完成 2 次完整的开合循环"
echo "运动范围: 0.015 m (从 -0.020 到 -0.005)"
echo ""
