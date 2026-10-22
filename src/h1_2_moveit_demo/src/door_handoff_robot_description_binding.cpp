#include "h1_2_moveit_demo/door_handoff_robot_description_binding.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace h1_2_moveit_demo::door_handoff_path_certificate
{
namespace
{

constexpr char kLowElbowMesh[] =
  "package://h1_2_description/meshes_low/right_elbow_link.STL";
constexpr char kLowTorsoMesh[] =
  "package://h1_2_description/meshes_low/torso_link.STL";
constexpr char kHighElbowMesh[] =
  "package://h1_2_description/meshes/right_elbow_link.STL";
constexpr char kHighTorsoMesh[] =
  "package://h1_2_description/meshes/torso_link.STL";

bool valid_description_bytes(const std::string & xml)
{
  return !xml.empty() && xml.size() <= kMaximumRobotDescriptionBytes &&
         xml.find('\0') == std::string::npos;
}

bool contains(const std::string & value, const char * required)
{
  return value.find(required) != std::string::npos;
}

std::vector<std::uint8_t> bytes(const std::string & value)
{
  return std::vector<std::uint8_t>(value.begin(), value.end());
}

RobotDescriptionBindingHashResult failure(const std::string & message)
{
  RobotDescriptionBindingHashResult result;
  result.message = message;
  return result;
}

}  // namespace

RobotDescriptionBindingHashResult hash_robot_description_bindings(
  const RobotDescriptionBindingSnapshot & snapshot)
{
  if (!valid_description_bytes(snapshot.move_group_low_urdf) ||
    !valid_description_bytes(snapshot.robot_state_publisher_high_urdf) ||
    !valid_description_bytes(snapshot.move_group_srdf))
  {
    return failure(
      "robot description bytes are empty, oversized, or contain NUL");
  }
  if (snapshot.move_group_low_urdf ==
    snapshot.robot_state_publisher_high_urdf)
  {
    return failure("low- and high-poly URDF snapshots are identical");
  }
  const bool low_mesh_contract =
    contains(snapshot.move_group_low_urdf, kLowElbowMesh) &&
    contains(snapshot.move_group_low_urdf, kLowTorsoMesh) &&
    !contains(snapshot.move_group_low_urdf, kHighElbowMesh) &&
    !contains(snapshot.move_group_low_urdf, kHighTorsoMesh);
  const bool high_mesh_contract =
    contains(snapshot.robot_state_publisher_high_urdf, kHighElbowMesh) &&
    contains(snapshot.robot_state_publisher_high_urdf, kHighTorsoMesh) &&
    !contains(snapshot.robot_state_publisher_high_urdf, kLowElbowMesh) &&
    !contains(snapshot.robot_state_publisher_high_urdf, kLowTorsoMesh);
  if (!low_mesh_contract || !high_mesh_contract) {
    return failure(
      "low/high URDF source-role URI markers are invalid");
  }

  RobotDescriptionBindingHashResult result;
  result.urdf = sha256_binding_bundle(
    "urdf_sha256",
    {
      BindingHashEntry{
        "move_group/robot_description", bytes(snapshot.move_group_low_urdf)},
      BindingHashEntry{
        "robot_state_publisher/robot_description",
        bytes(snapshot.robot_state_publisher_high_urdf)},
    });
  if (!result.urdf.passed) {
    result.message = "URDF binding failed: " + result.urdf.message;
    return result;
  }
  result.srdf = sha256_binding_bundle(
    "srdf_sha256",
    {BindingHashEntry{
        "move_group/robot_description_semantic",
        bytes(snapshot.move_group_srdf)}});
  if (!result.srdf.passed) {
    result.message = "SRDF binding failed: " + result.srdf.message;
    return result;
  }
  result.passed = true;
  result.message =
    "owned low/high URDF and SRDF snapshots passed typed binding validation";
  return result;
}

}  // namespace h1_2_moveit_demo::door_handoff_path_certificate
