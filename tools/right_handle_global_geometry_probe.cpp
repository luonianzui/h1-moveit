// Read-only batch probe for exact right-Dex1/original-handle geometry.
//
// This file is intentionally not part of the workspace CMake project.  It does
// not plan, execute, publish a trajectory, modify the planning scene, or call a
// Gazebo service.  It reads RobotModel/current state through MoveGroupInterface,
// reads incubator_handle_exact from /get_planning_scene, caches every FCL mesh,
// and evaluates stdin candidates in one process.
//
// Input (whitespace separated, one candidate per line; '#' starts a comment):
//   id x y z qx qy qz qw finger1_mm finger2_mm
// TCP poses are in pelvis.  The output is CSV.  The strict_pass column covers
// only the exact local gripper/handle gate; it is not an IK, robot self/world,
// OctoMap, approach-path, interpolation, or controller-safety result.  Optional
// ROS parameter ik_seeds (>0) adds independent right-arm KDL reachability
// columns; its default 0 preserves the original output schema.

#include <Eigen/Geometry>

#include <fcl/fcl.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using FclGeometry = std::shared_ptr<fcl::CollisionGeometryd>;

constexpr char kPlanningFrame[] = "pelvis";
constexpr char kHandleId[] = "incubator_handle_exact";
constexpr char kTcpLink[] = "right_dex1_tcp";
constexpr char kFinger1Joint[] = "right_dex1_finger_1_joint";
constexpr char kFinger2Joint[] = "right_dex1_finger_2_joint";
constexpr char kPad1Link[] = "right_dex1_finger_1_pad_link";
constexpr char kPad2Link[] = "right_dex1_finger_2_pad_link";
constexpr char kFinger1Link[] = "right_dex1_finger_1_link";
constexpr char kFinger2Link[] = "right_dex1_finger_2_link";
constexpr char kSlider1Link[] = "right_dex1_finger_1_slider_link";
constexpr char kSlider2Link[] = "right_dex1_finger_2_slider_link";
constexpr char kBaseLink[] = "right_dex1_base_link";

constexpr std::size_t kMaximumContacts = 4096U;
constexpr double kMinimumPatchSpan = 0.00010;
constexpr double kMinimumRadius = 0.035;
constexpr double kEndTolerance = 0.001;

struct Thresholds
{
  double minimum_station = 0.015;
  double maximum_station = 1000.0;
  double maximum_station_separation = 0.026;
  double minimum_body_clearance = 0.0006;
};

struct CachedShape
{
  FclGeometry geometry;
  Eigen::Isometry3d collision_origin = Eigen::Isometry3d::Identity();
};

struct CachedLink
{
  const moveit::core::LinkModel * model = nullptr;
  std::vector<CachedShape> shapes;
};

struct WorldShape
{
  FclGeometry geometry;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

struct HandleGeometry
{
  FclGeometry geometry;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d pivot = Eigen::Vector3d::Zero();
  Eigen::Vector3d free_axis = Eigen::Vector3d::UnitZ();
  double minimum_local_x = std::numeric_limits<double>::infinity();
  double maximum_local_x = -std::numeric_limits<double>::infinity();
};

struct PadPatch
{
  bool geometry_found = false;
  bool saturated = false;
  std::size_t contacts = 0U;
  double minimum_station = std::numeric_limits<double>::infinity();
  double maximum_station = -std::numeric_limits<double>::infinity();
  Eigen::Vector3d minimum_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d maximum_position = Eigen::Vector3d::Zero();
  double maximum_penetration = 0.0;
  double separated_distance = std::numeric_limits<double>::infinity();

  double span() const
  {
    return contacts > 0U ? maximum_station - minimum_station : 0.0;
  }

  double center_station() const
  {
    return contacts > 0U ? 0.5 * (minimum_station + maximum_station) :
           std::numeric_limits<double>::quiet_NaN();
  }

  Eigen::Vector3d center_position() const
  {
    return 0.5 * (minimum_position + maximum_position);
  }

  // This is a ranking aid, not a mathematically exact signed distance.
  double heuristic_signed_clearance() const
  {
    return contacts > 0U ? -maximum_penetration : separated_distance;
  }
};

struct Clearance
{
  bool geometry_found = false;
  bool collision = false;
  double minimum_distance = std::numeric_limits<double>::infinity();
  double maximum_penetration = 0.0;

  // Collision uses the deepest returned FCL contact; separation uses distance.
  double heuristic_signed_clearance() const
  {
    return collision ? -maximum_penetration : minimum_distance;
  }
};

struct Candidate
{
  std::string id;
  Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
  double q1 = 0.0;
  double q2 = 0.0;
};

struct LocalIk
{
  std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> loader;
  std::shared_ptr<kinematics::KinematicsBase> solver;
  std::string base_frame;
};

struct IkAudit
{
  std::size_t seeds = 0U;
  std::size_t hits = 0U;
  double best_minimum_joint_margin =
    -std::numeric_limits<double>::infinity();
  std::string best_limiting_joint = "none";
};

struct Evaluation
{
  bool pads_gate = false;
  bool station_gate = false;
  bool ends_gate = false;
  bool nonpad_gate = false;
  bool strict_pass = false;
  double score_mm = std::numeric_limits<double>::infinity();
  PadPatch pad1;
  PadPatch pad2;
  Clearance finger1;
  Clearance finger2;
  Clearance slider1;
  Clearance slider2;
  Clearance base;
  double pad1_local_x = std::numeric_limits<double>::quiet_NaN();
  double pad2_local_x = std::numeric_limits<double>::quiet_NaN();
  double pad1_radius = std::numeric_limits<double>::quiet_NaN();
  double pad2_radius = std::numeric_limits<double>::quiet_NaN();
  double station_separation = std::numeric_limits<double>::infinity();
};

std::optional<Eigen::Isometry3d> checked_pose(
  const geometry_msgs::msg::Pose & pose)
{
  const std::array<double, 7> values{
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z,
    pose.orientation.w};
  if (!std::all_of(values.begin(), values.end(), [](double v) {
      return std::isfinite(v);
    }))
  {
    return std::nullopt;
  }
  Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);
  if (q.norm() < 1e-12) {
    return std::nullopt;
  }
  q.normalize();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  result.linear() = q.toRotationMatrix();
  return result;
}

geometry_msgs::msg::Pose eigen_to_pose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  Eigen::Quaterniond quaternion(transform.linear());
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

std::optional<LocalIk> make_local_ik(
  const rclcpp::Node::SharedPtr & node,
  const moveit::core::RobotModelConstPtr & robot_model)
{
  if (!robot_model) {
    return std::nullopt;
  }
  const auto * group = robot_model->getJointModelGroup("right_arm");
  if (!group) {
    return std::nullopt;
  }
  const auto & roots = group->getJointRoots();
  const auto * root_parent = roots.size() == 1U ?
    roots.front()->getParentLinkModel() : nullptr;
  if (!root_parent) {
    return std::nullopt;
  }
  try {
    LocalIk local;
    local.base_frame = root_parent->getName();
    local.loader = std::make_shared<
      pluginlib::ClassLoader<kinematics::KinematicsBase>>(
      "moveit_core", "kinematics::KinematicsBase");
    local.solver = local.loader->createSharedInstance(
      "kdl_kinematics_plugin/KDLKinematicsPlugin");
    if (!local.solver || !local.solver->initialize(
        node, *robot_model, "right_arm", local.base_frame, {kTcpLink}, 0.005))
    {
      return std::nullopt;
    }
    return local;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node->get_logger(), "Could not initialize read-only right KDL IK: %s",
      error.what());
    return std::nullopt;
  }
}

class IkProbe
{
public:
  IkProbe(
    moveit::core::RobotModelConstPtr model,
    moveit::core::RobotStatePtr state,
    LocalIk local)
  : model_(std::move(model)), template_state_(std::move(state)),
    local_(std::move(local))
  {
    group_ = model_->getJointModelGroup("right_arm");
    planning_link_ = model_->getLinkModel(kPlanningFrame);
    tcp_link_ = model_->getLinkModel(kTcpLink);
    base_link_ = model_->getLinkModel(local_.base_frame);
    if (!group_ || !planning_link_ || !tcp_link_ || !base_link_ ||
      !local_.solver ||
      local_.solver->getJointNames().size() != group_->getVariableCount())
    {
      throw std::runtime_error("incomplete right-arm KDL IK model");
    }
    const auto & all_variables = model_->getVariableNames();
    for (const auto & name : local_.solver->getJointNames()) {
      if (std::find(all_variables.begin(), all_variables.end(), name) ==
        all_variables.end())
      {
        throw std::runtime_error("KDL IK returned an unknown joint: " + name);
      }
    }
  }

  IkAudit evaluate(
    const Eigen::Isometry3d & target_tcp, const std::size_t seed_count) const
  {
    IkAudit audit;
    audit.seeds = seed_count;
    const Eigen::Isometry3d target_in_solver_base =
      template_state_->getGlobalLinkTransform(base_link_).inverse() *
      template_state_->getGlobalLinkTransform(planning_link_) * target_tcp;
    const auto & solver_names = local_.solver->getJointNames();
    for (std::size_t index = 0U; index < seed_count; ++index) {
      (void)index;
      moveit::core::RobotState random_seed(*template_state_);
      random_seed.setToRandomPositions(group_);
      random_seed.update();
      std::vector<double> seed;
      seed.reserve(solver_names.size());
      for (const auto & name : solver_names) {
        seed.push_back(random_seed.getVariablePosition(name));
      }
      std::vector<double> solution;
      moveit_msgs::msg::MoveItErrorCodes code;
      if (!local_.solver->getPositionIK(
          eigen_to_pose(target_in_solver_base), seed, solution, code) ||
        code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
        solution.size() != solver_names.size())
      {
        continue;
      }
      moveit::core::RobotState solved(random_seed);
      solved.setVariablePositions(solver_names, solution);
      solved.update();
      const Eigen::Isometry3d solved_tcp =
        solved.getGlobalLinkTransform(planning_link_).inverse() *
        solved.getGlobalLinkTransform(tcp_link_);
      const double position_error =
        (solved_tcp.translation() - target_tcp.translation()).norm();
      const double orientation_error = Eigen::AngleAxisd(
        target_tcp.linear().transpose() * solved_tcp.linear()).angle();
      if (!solved.satisfiesBounds(group_) || position_error > 0.0005 ||
        orientation_error > 0.5 * std::acos(-1.0) / 180.0)
      {
        continue;
      }

      ++audit.hits;
      double minimum_margin = std::numeric_limits<double>::infinity();
      std::string limiting_joint = "unbounded";
      for (const auto & name : solver_names) {
        const auto & bounds = model_->getVariableBounds(name);
        if (!bounds.position_bounded_) {
          continue;
        }
        const double value = solved.getVariablePosition(name);
        const double margin = std::min(
          value - bounds.min_position_, bounds.max_position_ - value);
        if (margin < minimum_margin) {
          minimum_margin = margin;
          limiting_joint = name;
        }
      }
      if (minimum_margin > audit.best_minimum_joint_margin) {
        audit.best_minimum_joint_margin = minimum_margin;
        audit.best_limiting_joint = limiting_joint;
      }
    }
    return audit;
  }

private:
  moveit::core::RobotModelConstPtr model_;
  moveit::core::RobotStatePtr template_state_;
  LocalIk local_;
  const moveit::core::JointModelGroup * group_ = nullptr;
  const moveit::core::LinkModel * planning_link_ = nullptr;
  const moveit::core::LinkModel * tcp_link_ = nullptr;
  const moveit::core::LinkModel * base_link_ = nullptr;
};

FclGeometry fcl_geometry_for_robot_shape(const shapes::ShapeConstPtr & shape)
{
  if (!shape) {
    return nullptr;
  }
  switch (shape->type) {
    case shapes::SPHERE:
      return std::make_shared<fcl::Sphered>(
        static_cast<const shapes::Sphere *>(shape.get())->radius);
    case shapes::CYLINDER:
    {
      const auto * value = static_cast<const shapes::Cylinder *>(shape.get());
      return std::make_shared<fcl::Cylinderd>(value->radius, value->length);
    }
    case shapes::CONE:
    {
      const auto * value = static_cast<const shapes::Cone *>(shape.get());
      return std::make_shared<fcl::Coned>(value->radius, value->length);
    }
    case shapes::BOX:
    {
      const auto * value = static_cast<const shapes::Box *>(shape.get());
      return std::make_shared<fcl::Boxd>(
        value->size[0], value->size[1], value->size[2]);
    }
    case shapes::MESH:
    {
      const auto * value = static_cast<const shapes::Mesh *>(shape.get());
      if (value->vertex_count == 0U || value->triangle_count == 0U ||
        value->vertices == nullptr || value->triangles == nullptr)
      {
        return nullptr;
      }
      std::vector<fcl::Vector3d> vertices;
      vertices.reserve(value->vertex_count);
      for (unsigned int i = 0U; i < value->vertex_count; ++i) {
        vertices.emplace_back(
          value->vertices[3U * i], value->vertices[3U * i + 1U],
          value->vertices[3U * i + 2U]);
      }
      std::vector<fcl::Triangle> triangles;
      triangles.reserve(value->triangle_count);
      for (unsigned int i = 0U; i < value->triangle_count; ++i) {
        triangles.emplace_back(
          value->triangles[3U * i], value->triangles[3U * i + 1U],
          value->triangles[3U * i + 2U]);
      }
      auto mesh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
      if (mesh->beginModel(
          static_cast<int>(triangles.size()),
          static_cast<int>(vertices.size())) != fcl::BVH_OK ||
        mesh->addSubModel(vertices, triangles) != fcl::BVH_OK ||
        mesh->endModel() != fcl::BVH_OK)
      {
        return nullptr;
      }
      return mesh;
    }
    default:
      return nullptr;
  }
}

std::optional<HandleGeometry> make_handle_geometry(
  const shape_msgs::msg::Mesh & mesh, const Eigen::Isometry3d & pose)
{
  if (mesh.vertices.empty() || mesh.triangles.empty()) {
    return std::nullopt;
  }
  HandleGeometry result;
  result.pose = pose;
  std::vector<fcl::Vector3d> vertices;
  vertices.reserve(mesh.vertices.size());
  for (const auto & vertex : mesh.vertices) {
    if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) ||
      !std::isfinite(vertex.z))
    {
      return std::nullopt;
    }
    vertices.emplace_back(vertex.x, vertex.y, vertex.z);
    result.minimum_local_x = std::min(result.minimum_local_x, vertex.x);
    result.maximum_local_x = std::max(result.maximum_local_x, vertex.x);
  }
  std::vector<fcl::Triangle> triangles;
  triangles.reserve(mesh.triangles.size());
  for (const auto & triangle : mesh.triangles) {
    if (triangle.vertex_indices[0] >= mesh.vertices.size() ||
      triangle.vertex_indices[1] >= mesh.vertices.size() ||
      triangle.vertex_indices[2] >= mesh.vertices.size())
    {
      return std::nullopt;
    }
    triangles.emplace_back(
      triangle.vertex_indices[0], triangle.vertex_indices[1],
      triangle.vertex_indices[2]);
  }
  auto geometry = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
  if (geometry->beginModel(
      static_cast<int>(triangles.size()),
      static_cast<int>(vertices.size())) != fcl::BVH_OK ||
    geometry->addSubModel(vertices, triangles) != fcl::BVH_OK ||
    geometry->endModel() != fcl::BVH_OK)
  {
    return std::nullopt;
  }
  result.geometry = std::move(geometry);

  // Exact rigid relation exported from incubator_assembly.SLDASM.  Applying it
  // to the current mesh pose remains valid after latch and door rotations.
  Eigen::Isometry3d closed_mesh = Eigen::Isometry3d::Identity();
  closed_mesh.translation() =
    Eigen::Vector3d(0.431247170396, 0.119573973523, -0.094003898168);
  closed_mesh.linear() = Eigen::AngleAxisd(
    1.613424341497, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d closed_pivot(
    0.430848293340, 0.129565720889, 0.024999998514);
  const Eigen::Vector3d mesh_local_pivot = closed_mesh.inverse() * closed_pivot;
  result.pivot = result.pose * mesh_local_pivot;
  result.free_axis = -result.pose.linear().col(2).normalized();
  return result;
}

class Probe
{
public:
  Probe(
    moveit::core::RobotModelConstPtr model,
    moveit::core::RobotStatePtr state,
    HandleGeometry handle,
    Thresholds thresholds)
  : model_(std::move(model)), template_state_(std::move(state)),
    handle_(std::move(handle)), thresholds_(thresholds)
  {
    if (!model_ || !template_state_) {
      throw std::runtime_error("RobotModel/current state is missing");
    }
    tcp_model_ = model_->getLinkModel(kTcpLink);
    finger1_joint_model_ = model_->getJointModel(kFinger1Joint);
    finger2_joint_model_ = model_->getJointModel(kFinger2Joint);
    if (!tcp_model_ || !finger1_joint_model_ || !finger2_joint_model_) {
      throw std::runtime_error("right TCP or finger joint is missing in RobotModel");
    }
    for (const char * name : std::array<const char *, 7>{
        kPad1Link, kPad2Link, kFinger1Link, kFinger2Link,
        kSlider1Link, kSlider2Link, kBaseLink})
    {
      cache_link(name);
    }
    if (handle_.maximum_local_x - handle_.minimum_local_x <
      2.0 * kEndTolerance)
    {
      throw std::runtime_error("handle local-X extent is too small");
    }
  }

  std::optional<Evaluation> evaluate(const Candidate & candidate) const
  {
    moveit::core::RobotState state(*template_state_);
    state.setVariablePosition(kFinger1Joint, candidate.q1);
    state.setVariablePosition(kFinger2Joint, candidate.q2);
    state.update();
    if (!state.satisfiesBounds(finger1_joint_model_) ||
      !state.satisfiesBounds(finger2_joint_model_))
    {
      return std::nullopt;
    }

    const Eigen::Isometry3d model_tcp =
      state.getGlobalLinkTransform(tcp_model_);
    const Eigen::Isometry3d tcp_model = model_tcp.inverse();
    const auto world_shapes = [&](const char * name) {
        std::vector<WorldShape> result;
        const auto & cached = links_.at(name);
        const Eigen::Isometry3d model_link =
          state.getGlobalLinkTransform(cached.model);
        result.reserve(cached.shapes.size());
        for (const auto & shape : cached.shapes) {
          result.push_back(WorldShape{
            shape.geometry,
            candidate.tcp * tcp_model * model_link * shape.collision_origin});
        }
        return result;
      };

    const auto pad1_shapes = world_shapes(kPad1Link);
    const auto pad2_shapes = world_shapes(kPad2Link);
    const auto finger1_shapes = world_shapes(kFinger1Link);
    const auto finger2_shapes = world_shapes(kFinger2Link);
    const auto slider1_shapes = world_shapes(kSlider1Link);
    const auto slider2_shapes = world_shapes(kSlider2Link);
    const auto base_shapes = world_shapes(kBaseLink);

    Evaluation result;
    result.pad1 = pad_patch(pad1_shapes);
    result.pad2 = pad_patch(pad2_shapes);
    result.finger1 = clearance(finger1_shapes);
    result.finger2 = clearance(finger2_shapes);
    result.slider1 = clearance(slider1_shapes);
    result.slider2 = clearance(slider2_shapes);
    result.base = clearance(base_shapes);

    if (result.pad1.contacts > 0U) {
      result.pad1_local_x =
        (handle_.pose.inverse() * result.pad1.center_position()).x();
      result.pad1_radius =
        (result.pad1.center_position() - handle_.pivot).norm();
    }
    if (result.pad2.contacts > 0U) {
      result.pad2_local_x =
        (handle_.pose.inverse() * result.pad2.center_position()).x();
      result.pad2_radius =
        (result.pad2.center_position() - handle_.pivot).norm();
    }
    if (result.pad1.contacts > 0U && result.pad2.contacts > 0U) {
      result.station_separation = std::abs(
        result.pad1.center_station() - result.pad2.center_station());
    }

    const bool pad1_end_low = result.pad1.contacts > 0U &&
      result.pad1_local_x <= handle_.minimum_local_x + kEndTolerance;
    const bool pad1_end_high = result.pad1.contacts > 0U &&
      result.pad1_local_x >= handle_.maximum_local_x - kEndTolerance;
    const bool pad2_end_low = result.pad2.contacts > 0U &&
      result.pad2_local_x <= handle_.minimum_local_x + kEndTolerance;
    const bool pad2_end_high = result.pad2.contacts > 0U &&
      result.pad2_local_x >= handle_.maximum_local_x - kEndTolerance;
    result.ends_gate =
      (pad1_end_low && pad2_end_high) || (pad2_end_low && pad1_end_high);
    result.pads_gate =
      result.pad1.geometry_found && result.pad2.geometry_found &&
      result.pad1.contacts > 0U && result.pad2.contacts > 0U &&
      !result.pad1.saturated && !result.pad2.saturated &&
      result.pad1.span() >= kMinimumPatchSpan &&
      result.pad2.span() >= kMinimumPatchSpan &&
      result.pad1_radius >= kMinimumRadius &&
      result.pad2_radius >= kMinimumRadius;
    result.station_gate =
      result.pad1.contacts > 0U && result.pad2.contacts > 0U &&
      result.pad1.center_station() >= thresholds_.minimum_station &&
      result.pad2.center_station() >= thresholds_.minimum_station &&
      result.pad1.center_station() <= thresholds_.maximum_station &&
      result.pad2.center_station() <= thresholds_.maximum_station &&
      result.station_separation <= thresholds_.maximum_station_separation;
    const auto clear = [](const Clearance & value, double minimum) {
        return value.geometry_found && !value.collision &&
               value.minimum_distance >= minimum;
      };
    result.nonpad_gate =
      clear(result.finger1, thresholds_.minimum_body_clearance) &&
      clear(result.finger2, thresholds_.minimum_body_clearance) &&
      clear(result.slider1, 0.0) && clear(result.slider2, 0.0) &&
      clear(result.base, 0.0);
    result.strict_pass = result.pads_gate && result.station_gate &&
      result.ends_gate && result.nonpad_gate;
    result.score_mm = continuous_score(result);
    return result;
  }

  double minimum_handle_x() const {return handle_.minimum_local_x;}
  double maximum_handle_x() const {return handle_.maximum_local_x;}

private:
  void cache_link(const std::string & name)
  {
    const auto * link = model_->getLinkModel(name);
    if (!link) {
      throw std::runtime_error("RobotModel link is missing: " + name);
    }
    const auto & shapes = link->getShapes();
    const auto & origins = link->getCollisionOriginTransforms();
    if (shapes.empty() || shapes.size() != origins.size()) {
      throw std::runtime_error("invalid collision geometry on link: " + name);
    }
    CachedLink cached;
    cached.model = link;
    cached.shapes.reserve(shapes.size());
    for (std::size_t i = 0U; i < shapes.size(); ++i) {
      const auto found = geometry_cache_.find(shapes[i].get());
      FclGeometry geometry;
      if (found != geometry_cache_.end()) {
        geometry = found->second;
      } else {
        geometry = fcl_geometry_for_robot_shape(shapes[i]);
        if (!geometry) {
          throw std::runtime_error("unsupported collision shape on link: " + name);
        }
        geometry_cache_.emplace(shapes[i].get(), geometry);
      }
      cached.shapes.push_back(CachedShape{geometry, origins[i]});
    }
    links_.emplace(name, std::move(cached));
  }

  PadPatch pad_patch(const std::vector<WorldShape> & shapes) const
  {
    PadPatch patch;
    fcl::CollisionObjectd handle(handle_.geometry, fcl::Transform3d(handle_.pose));
    for (const auto & shape : shapes) {
      patch.geometry_found = true;
      fcl::CollisionObjectd pad(shape.geometry, fcl::Transform3d(shape.pose));
      const fcl::CollisionRequestd collision_request(kMaximumContacts, true);
      fcl::CollisionResultd collision_result;
      fcl::collide(&pad, &handle, collision_request, collision_result);
      if (collision_result.numContacts() >= kMaximumContacts) {
        patch.saturated = true;
      }
      std::vector<fcl::Contactd> contacts;
      collision_result.getContacts(contacts);
      patch.contacts += contacts.size();
      for (const auto & contact : contacts) {
        if (!contact.pos.allFinite() || !std::isfinite(contact.penetration_depth)) {
          patch.saturated = true;
          continue;
        }
        patch.maximum_penetration = std::max(
          patch.maximum_penetration, contact.penetration_depth);
        const double station =
          handle_.free_axis.dot(contact.pos - handle_.pivot);
        if (station < patch.minimum_station) {
          patch.minimum_station = station;
          patch.minimum_position = contact.pos;
        }
        if (station > patch.maximum_station) {
          patch.maximum_station = station;
          patch.maximum_position = contact.pos;
        }
      }
      if (contacts.empty()) {
        const fcl::DistanceRequestd request(false, true, 0.0, 0.0, 1e-7);
        fcl::DistanceResultd result;
        const double distance = fcl::distance(&pad, &handle, request, result);
        if (std::isfinite(distance)) {
          patch.separated_distance = std::min(patch.separated_distance, distance);
        }
      }
    }
    return patch;
  }

  Clearance clearance(const std::vector<WorldShape> & shapes) const
  {
    Clearance result;
    fcl::CollisionObjectd handle(handle_.geometry, fcl::Transform3d(handle_.pose));
    for (const auto & shape : shapes) {
      result.geometry_found = true;
      fcl::CollisionObjectd object(shape.geometry, fcl::Transform3d(shape.pose));
      const fcl::CollisionRequestd collision_request(256U, true);
      fcl::CollisionResultd collision_result;
      fcl::collide(&object, &handle, collision_request, collision_result);
      result.collision = result.collision || collision_result.isCollision();
      std::vector<fcl::Contactd> contacts;
      collision_result.getContacts(contacts);
      for (const auto & contact : contacts) {
        if (std::isfinite(contact.penetration_depth)) {
          result.maximum_penetration = std::max(
            result.maximum_penetration, contact.penetration_depth);
        }
      }
      const fcl::DistanceRequestd request(false, true, 0.0, 0.0, 1e-7);
      fcl::DistanceResultd distance_result;
      const double distance = fcl::distance(
        &object, &handle, request, distance_result);
      if (std::isfinite(distance)) {
        result.minimum_distance = std::min(result.minimum_distance, distance);
      }
    }
    return result;
  }

  double continuous_score(const Evaluation & value) const
  {
    // Sum of physical deficits in metres, returned in millimetres.  Missing
    // contact adds 0.1 mm plus positive pad distance.  This score is solely for
    // ordering failed candidates; strict_pass remains the authoritative gate.
    double deficit = 0.0;
    const auto add_pad = [&](const PadPatch & pad, double radius) {
        if (pad.contacts == 0U) {
          deficit += 0.0001 + std::max(0.0, pad.heuristic_signed_clearance());
          return;
        }
        if (pad.saturated) {
          deficit += 0.001;
        }
        deficit += std::max(0.0, kMinimumPatchSpan - pad.span());
        deficit += std::max(0.0, kMinimumRadius - radius);
        deficit += std::max(0.0, thresholds_.minimum_station - pad.center_station());
        deficit += std::max(0.0, pad.center_station() - thresholds_.maximum_station);
      };
    add_pad(value.pad1, value.pad1_radius);
    add_pad(value.pad2, value.pad2_radius);
    if (value.pad1.contacts > 0U && value.pad2.contacts > 0U) {
      deficit += std::max(
        0.0, value.station_separation - thresholds_.maximum_station_separation);
      const double assignment_12 =
        std::max(0.0, value.pad1_local_x -
          (handle_.minimum_local_x + kEndTolerance)) +
        std::max(0.0, (handle_.maximum_local_x - kEndTolerance) -
          value.pad2_local_x);
      const double assignment_21 =
        std::max(0.0, value.pad2_local_x -
          (handle_.minimum_local_x + kEndTolerance)) +
        std::max(0.0, (handle_.maximum_local_x - kEndTolerance) -
          value.pad1_local_x);
      deficit += std::min(assignment_12, assignment_21);
    }
    const auto clearance_deficit = [](const Clearance & clearance, double minimum) {
        const double signed_value = clearance.heuristic_signed_clearance();
        return std::isfinite(signed_value) ?
               std::max(0.0, minimum - signed_value) : 1.0;
      };
    deficit += clearance_deficit(
      value.finger1, thresholds_.minimum_body_clearance);
    deficit += clearance_deficit(
      value.finger2, thresholds_.minimum_body_clearance);
    deficit += clearance_deficit(value.slider1, 0.0);
    deficit += clearance_deficit(value.slider2, 0.0);
    deficit += clearance_deficit(value.base, 0.0);
    return 1000.0 * deficit;
  }

  moveit::core::RobotModelConstPtr model_;
  moveit::core::RobotStatePtr template_state_;
  HandleGeometry handle_;
  Thresholds thresholds_;
  const moveit::core::LinkModel * tcp_model_ = nullptr;
  const moveit::core::JointModel * finger1_joint_model_ = nullptr;
  const moveit::core::JointModel * finger2_joint_model_ = nullptr;
  std::map<const shapes::Shape *, FclGeometry> geometry_cache_;
  std::map<std::string, CachedLink> links_;
};

std::optional<Candidate> parse_candidate(
  const std::string & line, std::string & reason)
{
  std::istringstream input(line);
  Candidate candidate;
  double x, y, z, qx, qy, qz, qw, q1_mm, q2_mm;
  if (!(input >> candidate.id >> x >> y >> z >> qx >> qy >> qz >> qw >>
    q1_mm >> q2_mm))
  {
    reason = "expected_10_fields";
    return std::nullopt;
  }
  std::string extra;
  if (input >> extra) {
    reason = "extra_field";
    return std::nullopt;
  }
  const std::array<double, 9> values{x, y, z, qx, qy, qz, qw, q1_mm, q2_mm};
  if (!std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    reason = "nonfinite";
    return std::nullopt;
  }
  Eigen::Quaterniond rotation(qw, qx, qy, qz);
  if (rotation.norm() < 1e-12) {
    reason = "zero_quaternion";
    return std::nullopt;
  }
  rotation.normalize();
  candidate.tcp.translation() = Eigen::Vector3d(x, y, z);
  candidate.tcp.linear() = rotation.toRotationMatrix();
  candidate.q1 = q1_mm * 0.001;
  candidate.q2 = q2_mm * 0.001;
  return candidate;
}

void print_header(const bool include_ik)
{
  std::cout <<
    "id,valid_input,strict_pass,score_mm,pads_gate,station_gate,ends_gate,"
    "nonpad_gate,pad1_contacts,pad2_contacts,pad1_span_mm,pad2_span_mm,"
    "pad1_station_mm,pad2_station_mm,station_separation_mm,"
    "pad1_local_x_mm,pad2_local_x_mm,pad1_radius_mm,pad2_radius_mm,"
    "pad1_signed_mm,pad2_signed_mm,finger1_signed_mm,finger2_signed_mm,"
    "slider1_signed_mm,slider2_signed_mm,base_signed_mm,reason";
  if (include_ik) {
    std::cout <<
      ",ik_seeds,ik_hits,best_minimum_joint_margin_deg,best_limiting_joint";
  }
  std::cout << '\n';
}

void print_invalid(
  const std::string & id, const std::string & reason,
  const std::size_t ik_seeds)
{
  std::cout << id << ",0,0,inf,0,0,0,0,0,0,nan,nan,nan,nan,inf,"
            << "nan,nan,nan,nan,inf,inf,inf,inf,inf,inf,inf," << reason
            ;
  if (ik_seeds > 0U) {
    std::cout << ',' << ik_seeds << ",0,-inf,none";
  }
  std::cout << '\n';
}

void print_evaluation(
  const Candidate & candidate, const Evaluation & value,
  const IkAudit * ik)
{
  const auto mm = [](double v) {return 1000.0 * v;};
  std::cout << candidate.id << ",1," << value.strict_pass << ','
            << value.score_mm << ',' << value.pads_gate << ','
            << value.station_gate << ',' << value.ends_gate << ','
            << value.nonpad_gate << ',' << value.pad1.contacts << ','
            << value.pad2.contacts << ',' << mm(value.pad1.span()) << ','
            << mm(value.pad2.span()) << ','
            << mm(value.pad1.center_station()) << ','
            << mm(value.pad2.center_station()) << ','
            << mm(value.station_separation) << ','
            << mm(value.pad1_local_x) << ',' << mm(value.pad2_local_x) << ','
            << mm(value.pad1_radius) << ',' << mm(value.pad2_radius) << ','
            << mm(value.pad1.heuristic_signed_clearance()) << ','
            << mm(value.pad2.heuristic_signed_clearance()) << ','
            << mm(value.finger1.heuristic_signed_clearance()) << ','
            << mm(value.finger2.heuristic_signed_clearance()) << ','
            << mm(value.slider1.heuristic_signed_clearance()) << ','
            << mm(value.slider2.heuristic_signed_clearance()) << ','
            << mm(value.base.heuristic_signed_clearance()) << ",ok";
  if (ik) {
    const double margin_degrees = ik->best_minimum_joint_margin *
      180.0 / std::acos(-1.0);
    std::cout << ',' << ik->seeds << ',' << ik->hits << ','
              << margin_degrees << ',' << ik->best_limiting_joint;
  }
  std::cout << '\n';
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "right_handle_global_geometry_probe",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});
  const auto finish = [&]() {
      executor.cancel();
      if (spinner.joinable()) {
        spinner.join();
      }
      rclcpp::shutdown();
    };

  int exit_code = 1;
  try {
    Thresholds thresholds;
    // Overrides are automatically declared by NodeOptions.  get_parameter_or()
    // therefore supports both ROS overrides and the no-override defaults without
    // risking a second declare_parameter() call.
    double minimum_station_mm = 15.0;
    double maximum_station_mm = 1000000.0;
    double maximum_station_separation_mm = 26.0;
    double minimum_body_clearance_mm = 0.6;
    int64_t ik_seeds_parameter = 0;
    node->get_parameter_or(
      "minimum_station_mm", minimum_station_mm, minimum_station_mm);
    node->get_parameter_or(
      "maximum_station_mm", maximum_station_mm, maximum_station_mm);
    node->get_parameter_or(
      "maximum_station_separation_mm", maximum_station_separation_mm,
      maximum_station_separation_mm);
    node->get_parameter_or(
      "minimum_body_clearance_mm", minimum_body_clearance_mm,
      minimum_body_clearance_mm);
    node->get_parameter_or(
      "ik_seeds", ik_seeds_parameter, ik_seeds_parameter);
    thresholds.minimum_station = minimum_station_mm * 0.001;
    thresholds.maximum_station = maximum_station_mm * 0.001;
    thresholds.maximum_station_separation =
      maximum_station_separation_mm * 0.001;
    thresholds.minimum_body_clearance = minimum_body_clearance_mm * 0.001;
    if (!std::isfinite(thresholds.minimum_station) ||
      !std::isfinite(thresholds.maximum_station) ||
      thresholds.minimum_station > thresholds.maximum_station ||
      !std::isfinite(thresholds.maximum_station_separation) ||
      thresholds.maximum_station_separation < 0.0 ||
      !std::isfinite(thresholds.minimum_body_clearance) ||
      thresholds.minimum_body_clearance < 0.0 || ik_seeds_parameter < 0 ||
      ik_seeds_parameter > 1000000)
    {
      throw std::runtime_error("invalid gate parameters");
    }

    MoveGroupInterface right_arm(node, "right_arm");
    const auto model = right_arm.getRobotModel();
    const auto state = right_arm.getCurrentState(10.0);
    if (!model || !state) {
      throw std::runtime_error("failed to obtain MoveIt RobotModel/current state");
    }
    const std::size_t ik_seeds =
      static_cast<std::size_t>(ik_seeds_parameter);
    std::unique_ptr<IkProbe> ik_probe;
    if (ik_seeds > 0U) {
      const auto local_ik = make_local_ik(node, model);
      if (!local_ik) {
        throw std::runtime_error("failed to initialize right-arm KDL IK");
      }
      ik_probe = std::make_unique<IkProbe>(model, state, *local_ik);
    }

    const auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene");
    if (!client->wait_for_service(5s)) {
      throw std::runtime_error("/get_planning_scene is unavailable");
    }
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
    auto future = client->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {
      throw std::runtime_error("timed out reading planning scene");
    }
    const auto response = future.get();
    const auto found = std::find_if(
      response->scene.world.collision_objects.begin(),
      response->scene.world.collision_objects.end(),
      [](const auto & object) {return object.id == kHandleId;});
    if (found == response->scene.world.collision_objects.end()) {
      throw std::runtime_error("incubator_handle_exact is absent from world");
    }
    const std::string frame = found->header.frame_id;
    if (frame != kPlanningFrame && frame != std::string("/") + kPlanningFrame) {
      throw std::runtime_error("handle object is not expressed in pelvis");
    }
    if (found->meshes.size() != 1U || found->mesh_poses.size() != 1U ||
      !found->primitives.empty() || !found->primitive_poses.empty())
    {
      throw std::runtime_error(
              "handle must contain exactly 1 mesh and 0 primitives");
    }
    const auto object_pose = checked_pose(found->pose);
    const auto mesh_pose = checked_pose(found->mesh_poses.front());
    if (!object_pose || !mesh_pose) {
      throw std::runtime_error("handle object/mesh pose is invalid");
    }
    const auto handle = make_handle_geometry(
      found->meshes.front(), *object_pose * *mesh_pose);
    if (!handle) {
      throw std::runtime_error("failed to build exact handle FCL mesh");
    }

    const Eigen::Isometry3d planning_to_handle = *object_pose * *mesh_pose;
    const Eigen::Quaterniond handle_quaternion(planning_to_handle.linear());
    RCLCPP_INFO(
      node->get_logger(),
      "Exact live handle mesh pose in pelvis: p=(%.12f, %.12f, %.12f), "
      "q_xyzw=(%.15f, %.15f, %.15f, %.15f).",
      planning_to_handle.translation().x(),
      planning_to_handle.translation().y(),
      planning_to_handle.translation().z(),
      handle_quaternion.x(), handle_quaternion.y(), handle_quaternion.z(),
      handle_quaternion.w());

    Probe probe(model, state, *handle, thresholds);
    RCLCPP_INFO(
      node->get_logger(),
      "Read-only probe ready: cached RobotModel/right Dex1/original handle; "
      "stdin format is id x y z qx qy qz qw q1_mm q2_mm. No trajectory or "
      "scene write will be issued.");
    std::cout << std::setprecision(12) << std::boolalpha;
    print_header(ik_probe != nullptr);
    std::string line;
    while (std::getline(std::cin, line)) {
      const auto first = line.find_first_not_of(" \t\r\n");
      if (first == std::string::npos || line[first] == '#') {
        continue;
      }
      std::string reason;
      const auto candidate = parse_candidate(line, reason);
      if (!candidate) {
        std::istringstream id_input(line);
        std::string id = "invalid";
        id_input >> id;
        print_invalid(id, reason, ik_seeds);
        continue;
      }
      const auto evaluation = probe.evaluate(*candidate);
      if (!evaluation) {
        print_invalid(candidate->id, "finger_joint_out_of_bounds", ik_seeds);
        continue;
      }
      const std::optional<IkAudit> ik = ik_probe ?
        std::optional<IkAudit>(ik_probe->evaluate(candidate->tcp, ik_seeds)) :
        std::nullopt;
      print_evaluation(*candidate, *evaluation, ik ? &*ik : nullptr);
    }
    exit_code = 0;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
  }
  finish();
  return exit_code;
}
