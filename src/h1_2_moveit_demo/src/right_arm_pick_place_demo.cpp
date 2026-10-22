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

geometry_msgs::msg::Pose pose_from_transform(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.rotation());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

geometry_msgs::msg::Pose tcp_pose_for_target(
  MoveGroupInterface & arm,
  const std::map<std::string, double> & target)
{
  auto state = arm.getCurrentState(10.0);
  if (!state) {
    throw std::runtime_error("No current robot state is available.");
  }

  state->setVariablePositions(target);
  state->update();
  return pose_from_transform(state->getGlobalLinkTransform(kTcpLink));
}

bool plan_and_execute(
  MoveGroupInterface & move_group,
  const std::map<std::string, double> & target,
  const std::string & stage_name)
{
  const auto logger = rclcpp::get_logger("h1_2_pick_place");
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

  // Add a small delay to allow action server to be ready
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

visualization_msgs::msg::Marker make_box_marker(
  const rclcpp::Time & stamp,
  int id,
  const std::string & name_space,
  const geometry_msgs::msg::Pose & pose,
  double size_x,
  double size_y,
  double size_z,
  float red,
  float green,
  float blue)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = kWorldFrame;
  marker.header.stamp = stamp;
  marker.ns = name_space;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = size_x;
  marker.scale.y = size_y;
  marker.scale.z = size_z;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0F;
  return marker;
}

class StageVisualizer
{
public:
  StageVisualizer(
    const rclcpp::Node::SharedPtr & node,
    geometry_msgs::msg::Pose pick_pose,
    geometry_msgs::msg::Pose place_pose)
  : node_(node), pick_pose_(std::move(pick_pose)), place_pose_(std::move(place_pose))
  {
    publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/rviz_visual_tools", rclcpp::QoS(10).reliable());
  }

  void show_scene()
  {
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = node_->now();

    markers.markers.push_back(
      make_box_marker(
        stamp, 0, "pick_place_cube", pick_pose_,
        kCubeSize, kCubeSize, kCubeSize, 0.10F, 0.85F, 0.15F));

    auto pick_table_pose = pick_pose_;
    pick_table_pose.orientation.x = 0.0;
    pick_table_pose.orientation.y = 0.0;
    pick_table_pose.orientation.z = 0.0;
    pick_table_pose.orientation.w = 1.0;
    pick_table_pose.position.z -= (kCubeSize / 2.0) + 0.025;
    markers.markers.push_back(
      make_box_marker(
        stamp, 1, "pick_place_tables", pick_table_pose,
        0.18, 0.18, 0.05, 0.42F, 0.22F, 0.08F));

    auto place_table_pose = place_pose_;
    place_table_pose.orientation.x = 0.0;
    place_table_pose.orientation.y = 0.0;
    place_table_pose.orientation.z = 0.0;
    place_table_pose.orientation.w = 1.0;
    place_table_pose.position.z -= (kCubeSize / 2.0) + 0.025;
    markers.markers.push_back(
      make_box_marker(
        stamp, 2, "pick_place_tables", place_table_pose,
        0.18, 0.18, 0.05, 0.18F, 0.32F, 0.55F));

    markers.markers.push_back(make_text_marker(stamp, "READY: PICK GREEN CUBE"));
    publisher_->publish(markers);
  }

  void set_stage(const std::string & text, bool show_pick_cube)
  {
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = node_->now();
    markers.markers.push_back(make_text_marker(stamp, text));

    visualization_msgs::msg::Marker cube_marker;
    cube_marker.header.frame_id = kWorldFrame;
    cube_marker.header.stamp = stamp;
    cube_marker.ns = "pick_place_cube";
    cube_marker.id = 0;
    cube_marker.action = show_pick_cube ?
      visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    if (show_pick_cube) {
      cube_marker.type = visualization_msgs::msg::Marker::CUBE;
      cube_marker.pose = pick_pose_;
      cube_marker.scale.x = kCubeSize;
      cube_marker.scale.y = kCubeSize;
      cube_marker.scale.z = kCubeSize;
      cube_marker.color.r = 0.10F;
      cube_marker.color.g = 0.85F;
      cube_marker.color.b = 0.15F;
      cube_marker.color.a = 1.0F;
    }
    markers.markers.push_back(cube_marker);
    publisher_->publish(markers);
  }

  void show_placed_cube(const std::string & text)
  {
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = node_->now();
    markers.markers.push_back(make_text_marker(stamp, text));

    visualization_msgs::msg::Marker attached_cube;
    attached_cube.header.frame_id = kTcpLink;
    attached_cube.header.stamp = stamp;
    attached_cube.ns = "attached_cube";
    attached_cube.id = 0;
    attached_cube.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(attached_cube);

    markers.markers.push_back(
      make_box_marker(
        stamp, 0, "placed_cube", place_pose_,
        kCubeSize, kCubeSize, kCubeSize, 0.10F, 0.85F, 0.15F));
    publisher_->publish(markers);
  }

  void show_attached_cube(const std::string & text)
  {
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = node_->now();
    markers.markers.push_back(make_text_marker(stamp, text));

    visualization_msgs::msg::Marker cube;
    cube.header.frame_id = kTcpLink;
    cube.header.stamp = stamp;
    cube.ns = "attached_cube";
    cube.id = 0;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.action = visualization_msgs::msg::Marker::ADD;
    cube.pose.orientation.w = 1.0;
    cube.scale.x = kCubeSize;
    cube.scale.y = kCubeSize;
    cube.scale.z = kCubeSize;
    cube.color.r = 0.10F;
    cube.color.g = 0.95F;
    cube.color.b = 0.15F;
    cube.color.a = 1.0F;
    markers.markers.push_back(cube);
    publisher_->publish(markers);
  }

private:
  visualization_msgs::msg::Marker make_text_marker(
    const rclcpp::Time & stamp,
    const std::string & text) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = kWorldFrame;
    marker.header.stamp = stamp;
    marker.ns = "pick_place_stage";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pick_pose_;
    marker.pose.position.z += 0.22;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.065;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 0.2F;
    marker.color.a = 1.0F;
    marker.text = text;
    return marker;
  }

  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::Pose pick_pose_;
  geometry_msgs::msg::Pose place_pose_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

moveit_msgs::msg::CollisionObject make_cube_object(
  const geometry_msgs::msg::Pose & pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = kWorldFrame;
  object.id = kCubeId;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {kCubeSize, kCubeSize, kCubeSize};
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("h1_2_right_arm_pick_place_demo", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  MoveGroupInterface right_arm(node, "right_arm");
  MoveGroupInterface right_gripper(node, "right_gripper");
  moveit::planning_interface::PlanningSceneInterface planning_scene;

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
    // Slightly inside the URDF lower limit (-0.02) to avoid floating-point
    // boundary rejection by MoveIt's CheckStartStateBounds adapter.
    {"right_dex1_finger_1_joint", -0.019},
    {"right_dex1_finger_2_joint", -0.019},
  };
  const std::map<std::string, double> closed_gripper{
    // At -5 mm the Dex1-1 contact pads leave roughly 60 mm for the cube.
    {"right_dex1_finger_1_joint", -0.005},
    {"right_dex1_finger_2_joint", -0.005},
  };

  const std::map<std::string, double> pregrasp{
    {"right_shoulder_pitch_joint", -0.50},
    {"right_shoulder_roll_joint", -0.48},
    {"right_shoulder_yaw_joint", 0.28},
    {"right_elbow_joint", 0.95},
    {"right_wrist_roll_joint", -0.18},
    {"right_wrist_pitch_joint", 0.10},
    {"right_wrist_yaw_joint", 0.0},
  };
  const std::map<std::string, double> grasp{
    {"right_shoulder_pitch_joint", -0.70},
    {"right_shoulder_roll_joint", -0.50},
    {"right_shoulder_yaw_joint", 0.18},
    {"right_elbow_joint", 1.15},
    {"right_wrist_roll_joint", -0.15},
    {"right_wrist_pitch_joint", 0.15},
    {"right_wrist_yaw_joint", 0.0},
  };
  const std::map<std::string, double> lift{
    {"right_shoulder_pitch_joint", -0.45},
    {"right_shoulder_roll_joint", -0.43},
    {"right_shoulder_yaw_joint", 0.10},
    {"right_elbow_joint", 0.90},
    {"right_wrist_roll_joint", -0.10},
    {"right_wrist_pitch_joint", 0.10},
    {"right_wrist_yaw_joint", 0.0},
  };
  const std::map<std::string, double> transfer{
    {"right_shoulder_pitch_joint", -0.35},
    {"right_shoulder_roll_joint", -0.24},
    {"right_shoulder_yaw_joint", -0.55},
    {"right_elbow_joint", 1.05},
    {"right_wrist_roll_joint", -0.05},
    {"right_wrist_pitch_joint", 0.12},
    {"right_wrist_yaw_joint", 0.25},
  };
  const std::map<std::string, double> place{
    {"right_shoulder_pitch_joint", -0.55},
    {"right_shoulder_roll_joint", -0.28},
    {"right_shoulder_yaw_joint", -0.65},
    {"right_elbow_joint", 1.20},
    {"right_wrist_roll_joint", -0.05},
    {"right_wrist_pitch_joint", 0.18},
    {"right_wrist_yaw_joint", 0.25},
  };
  const std::map<std::string, double> retreat{
    {"right_shoulder_pitch_joint", -0.25},
    {"right_shoulder_roll_joint", -0.22},
    {"right_shoulder_yaw_joint", -0.45},
    {"right_elbow_joint", 0.82},
    {"right_wrist_roll_joint", 0.0},
    {"right_wrist_pitch_joint", 0.08},
    {"right_wrist_yaw_joint", 0.15},
  };
  const std::map<std::string, double> neutral{
    {"right_shoulder_pitch_joint", 0.0},
    {"right_shoulder_roll_joint", -0.25},
    {"right_shoulder_yaw_joint", 0.0},
    {"right_elbow_joint", 0.60},
    {"right_wrist_roll_joint", 0.0},
    {"right_wrist_pitch_joint", 0.0},
    {"right_wrist_yaw_joint", 0.0},
  };

  bool success = false;
  try {
    const auto pick_pose = tcp_pose_for_target(right_arm, grasp);
    const auto place_pose = tcp_pose_for_target(right_arm, place);
    RCLCPP_INFO(
      node->get_logger(), "Pick position: [%.3f, %.3f, %.3f]",
      pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);
    RCLCPP_INFO(
      node->get_logger(), "Place position: [%.3f, %.3f, %.3f]",
      place_pose.position.x, place_pose.position.y, place_pose.position.z);

    StageVisualizer visualizer(node, pick_pose, place_pose);
    planning_scene.removeCollisionObjects({kCubeId});
    visualizer.show_scene();
    std::this_thread::sleep_for(1500ms);

    visualizer.set_stage("1/8 OPEN GRIPPER", true);
    success = plan_and_execute(right_gripper, open_gripper, "open gripper");

    if (success) {
      visualizer.set_stage("2/8 MOVE TO PREGRASP", true);
      success = plan_and_execute(right_arm, pregrasp, "move to pregrasp");
      std::this_thread::sleep_for(700ms);
    }
    if (success) {
      visualizer.set_stage("3/8 APPROACH CUBE", true);
      success = plan_and_execute(right_arm, grasp, "approach cube");
      std::this_thread::sleep_for(700ms);
    }
    if (success) {
      visualizer.set_stage("4/8 CLOSE AND ATTACH", false);
      const auto cube = make_cube_object(pick_pose);
      std_msgs::msg::ColorRGBA cube_color;
      cube_color.r = 0.10F;
      cube_color.g = 0.95F;
      cube_color.b = 0.15F;
      cube_color.a = 1.0F;
      success = planning_scene.applyCollisionObject(cube, cube_color);
      if (success) {
        std::this_thread::sleep_for(500ms);
        success = right_arm.attachObject(
          kCubeId, kTcpLink,
          {
            "right_wrist_yaw_link",
            "right_dex1_adapter_link",
            "right_dex1_base_link",
            "right_dex1_finger_1_slider_link",
            "right_dex1_finger_1_link",
            "right_dex1_finger_1_pad_link",
            "right_dex1_finger_2_slider_link",
            "right_dex1_finger_2_link",
            "right_dex1_finger_2_pad_link",
            kTcpLink,
          });
      }
      if (success) {
        std::this_thread::sleep_for(500ms);
        visualizer.show_attached_cube("4/8 CLOSE AND ATTACH");
        success = plan_and_execute(right_gripper, closed_gripper, "close gripper");
      }
    }
    if (success) {
      std::this_thread::sleep_for(1000ms);
      visualizer.show_attached_cube("5/8 LIFT CUBE");
      success = plan_and_execute(right_arm, lift, "lift cube");
      std::this_thread::sleep_for(900ms);
    }
    if (success) {
      visualizer.show_attached_cube("6/8 TRANSFER CUBE");
      success = plan_and_execute(right_arm, transfer, "transfer cube");
      std::this_thread::sleep_for(900ms);
    }
    if (success) {
      visualizer.show_attached_cube("7/8 LOWER TO PLACE");
      success = plan_and_execute(right_arm, place, "move to place pose");
      std::this_thread::sleep_for(900ms);
    }
    if (success) {
      visualizer.show_attached_cube("8/8 RELEASE AND RETREAT");
      success = plan_and_execute(right_gripper, open_gripper, "open gripper to release");
      if (success) {
        success = right_arm.detachObject(kCubeId);
        std::this_thread::sleep_for(700ms);
      }
      if (success) {
        planning_scene.removeCollisionObjects({kCubeId});
        visualizer.show_placed_cube("CUBE PLACED - RETREATING");
        std::this_thread::sleep_for(500ms);
        success = plan_and_execute(right_arm, retreat, "retreat from placed cube");
      }
      if (success) {
        success = plan_and_execute(right_arm, neutral, "return right arm to neutral");
      }
    }

    visualizer.set_stage(
      success ? "PICK AND PLACE COMPLETE" : "PICK FAILED - CHECK LOG",
      false);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Pick-and-place exception: %s", error.what());
    success = false;
  }

  if (success) {
    RCLCPP_INFO(
      node->get_logger(),
      "H1-2 right-arm cube pick-and-place demo finished successfully.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "H1-2 cube pick-and-place demo did not complete.");
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
