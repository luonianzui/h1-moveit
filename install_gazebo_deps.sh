#!/bin/bash
# Gazebo 和 RealSense 依赖包安装脚本

set -e

echo "=========================================="
echo "  安装 Gazebo 和 RealSense 依赖包"
echo "=========================================="
echo ""

echo "更新软件包列表..."
sudo apt-get update

echo ""
echo "安装依赖包..."
sudo apt-get install -y \
  ros-jazzy-gz-sim-vendor \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-description \
  python3-opencv \
  python3-numpy

echo ""
echo "=========================================="
echo "  安装完成！"
echo "=========================================="
echo ""
echo "现在可以编译工作空间了："
echo "  cd ~/h1_2_ros2_ws"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  colcon build --symlink-install"
