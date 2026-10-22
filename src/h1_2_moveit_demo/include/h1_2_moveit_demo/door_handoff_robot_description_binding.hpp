#ifndef H1_2_MOVEIT_DEMO__DOOR_HANDOFF_ROBOT_DESCRIPTION_BINDING_HPP_
#define H1_2_MOVEIT_DEMO__DOOR_HANDOFF_ROBOT_DESCRIPTION_BINDING_HPP_

#include "h1_2_moveit_demo/door_handoff_binding_hash.hpp"

#include <cstddef>
#include <string>

namespace h1_2_moveit_demo::door_handoff_path_certificate
{

constexpr std::size_t kMaximumRobotDescriptionBytes = 16U * 1024U * 1024U;

struct RobotDescriptionBindingSnapshot
{
  // These owned strings must also be the source used to construct the low and
  // high RobotModels.  They are hashed byte-for-byte without XML normalization.
  std::string move_group_low_urdf;
  std::string robot_state_publisher_high_urdf;
  std::string move_group_srdf;
};

struct RobotDescriptionBindingHashResult
{
  bool passed{false};
  std::string message;
  BindingBundleHashResult urdf;
  BindingBundleHashResult srdf;
};

RobotDescriptionBindingHashResult hash_robot_description_bindings(
  const RobotDescriptionBindingSnapshot & snapshot);

}  // namespace h1_2_moveit_demo::door_handoff_path_certificate

#endif  // H1_2_MOVEIT_DEMO__DOOR_HANDOFF_ROBOT_DESCRIPTION_BINDING_HPP_
