#include "h1_2_moveit_demo/door_handoff_robot_description_binding.hpp"

#include <gtest/gtest.h>

#include <string>
#include <utility>

namespace binding =
  h1_2_moveit_demo::door_handoff_path_certificate;

namespace
{

binding::RobotDescriptionBindingSnapshot valid_snapshot()
{
  binding::RobotDescriptionBindingSnapshot snapshot;
  snapshot.move_group_low_urdf =
    "<robot name=\"low\">"
    "package://h1_2_description/meshes_low/right_elbow_link.STL "
    "package://h1_2_description/meshes_low/torso_link.STL"
    "</robot>\n";
  snapshot.robot_state_publisher_high_urdf =
    "<robot name=\"high\">"
    "package://h1_2_description/meshes/right_elbow_link.STL "
    "package://h1_2_description/meshes/torso_link.STL"
    "</robot>\n";
  snapshot.move_group_srdf =
    "<robot name=\"semantic\"><group name=\"right_arm\"/></robot>\n";
  return snapshot;
}

TEST(DoorHandoffRobotDescriptionBinding, ValidSnapshotsAreDeterministic)
{
  const auto first = binding::hash_robot_description_bindings(valid_snapshot());
  const auto second = binding::hash_robot_description_bindings(valid_snapshot());
  ASSERT_TRUE(first.passed) << first.message;
  ASSERT_TRUE(second.passed) << second.message;
  EXPECT_EQ(first.urdf.entry_count, 2U);
  EXPECT_EQ(first.srdf.entry_count, 1U);
  EXPECT_EQ(first.urdf.raw_size_bytes, 269U);
  EXPECT_EQ(first.urdf.envelope_size_bytes, 403U);
  EXPECT_EQ(first.srdf.raw_size_bytes, 57U);
  EXPECT_EQ(first.srdf.envelope_size_bytes, 145U);
  EXPECT_EQ(
    first.urdf.sha256,
    "f3fdb8e3ff3003fe7dae7c2fc54c690c441be70a77c5667c2dbe5f0c3c09369a");
  EXPECT_EQ(
    first.srdf.sha256,
    "e8b19e8d52963535fb2548e52103ba2619301750a81e50c8d1da80a7d45a1f61");
  EXPECT_EQ(first.urdf.sha256, second.urdf.sha256);
  EXPECT_EQ(first.srdf.sha256, second.srdf.sha256);
}

TEST(DoorHandoffRobotDescriptionBinding, ExactXmlBytesAffectDigest)
{
  const auto baseline = binding::hash_robot_description_bindings(valid_snapshot());
  auto changed_urdf = valid_snapshot();
  changed_urdf.move_group_low_urdf += "\n";
  auto changed_srdf = valid_snapshot();
  changed_srdf.move_group_srdf += "\r\n";
  const auto urdf = binding::hash_robot_description_bindings(changed_urdf);
  const auto srdf = binding::hash_robot_description_bindings(changed_srdf);
  ASSERT_TRUE(baseline.passed);
  ASSERT_TRUE(urdf.passed);
  ASSERT_TRUE(srdf.passed);
  EXPECT_NE(baseline.urdf.sha256, urdf.urdf.sha256);
  EXPECT_EQ(baseline.srdf.sha256, urdf.srdf.sha256);
  EXPECT_EQ(baseline.urdf.sha256, srdf.urdf.sha256);
  EXPECT_NE(baseline.srdf.sha256, srdf.srdf.sha256);
}

TEST(DoorHandoffRobotDescriptionBinding, InvalidXmlAndModelRolesFailClosed)
{
  auto empty = valid_snapshot();
  empty.move_group_srdf.clear();
  EXPECT_FALSE(binding::hash_robot_description_bindings(empty).passed);

  auto nul = valid_snapshot();
  nul.move_group_srdf.insert(4U, 1U, '\0');
  EXPECT_FALSE(binding::hash_robot_description_bindings(nul).passed);

  auto low_nul = valid_snapshot();
  low_nul.move_group_low_urdf.insert(4U, 1U, '\0');
  EXPECT_FALSE(binding::hash_robot_description_bindings(low_nul).passed);

  auto high_nul = valid_snapshot();
  high_nul.robot_state_publisher_high_urdf.insert(4U, 1U, '\0');
  EXPECT_FALSE(binding::hash_robot_description_bindings(high_nul).passed);

  auto identical = valid_snapshot();
  identical.robot_state_publisher_high_urdf = identical.move_group_low_urdf;
  EXPECT_FALSE(binding::hash_robot_description_bindings(identical).passed);

  auto swapped = valid_snapshot();
  std::swap(
    swapped.move_group_low_urdf,
    swapped.robot_state_publisher_high_urdf);
  EXPECT_FALSE(binding::hash_robot_description_bindings(swapped).passed);
}

TEST(DoorHandoffRobotDescriptionBinding, SourceRoleMarkersMustBeExclusive)
{
  auto missing_low_elbow = valid_snapshot();
  const auto low_elbow = missing_low_elbow.move_group_low_urdf.find(
    "meshes_low/right_elbow_link.STL");
  ASSERT_NE(low_elbow, std::string::npos);
  missing_low_elbow.move_group_low_urdf.replace(
    low_elbow, std::string("meshes_low/right_elbow_link.STL").size(),
    "meshes_low/missing_elbow_link.STL");
  EXPECT_FALSE(
    binding::hash_robot_description_bindings(missing_low_elbow).passed);

  auto low_mixed = valid_snapshot();
  low_mixed.move_group_low_urdf +=
    "package://h1_2_description/meshes/right_elbow_link.STL";
  EXPECT_FALSE(binding::hash_robot_description_bindings(low_mixed).passed);

  auto high_mixed = valid_snapshot();
  high_mixed.robot_state_publisher_high_urdf +=
    "package://h1_2_description/meshes_low/right_elbow_link.STL";
  EXPECT_FALSE(binding::hash_robot_description_bindings(high_mixed).passed);
}

TEST(DoorHandoffRobotDescriptionBinding, FieldSizeLimitFailsClosed)
{
  auto boundary = valid_snapshot();
  const std::string prefix = "<robot>";
  const std::string suffix = "</robot>";
  boundary.move_group_srdf = prefix +
    std::string(
    binding::kMaximumRobotDescriptionBytes - prefix.size() - suffix.size(),
    'x') + suffix;
  ASSERT_EQ(
    boundary.move_group_srdf.size(),
    binding::kMaximumRobotDescriptionBytes);
  EXPECT_TRUE(binding::hash_robot_description_bindings(boundary).passed);

  boundary.move_group_srdf.push_back('x');
  EXPECT_GT(
    boundary.move_group_srdf.size(),
    binding::kMaximumRobotDescriptionBytes);
  EXPECT_FALSE(binding::hash_robot_description_bindings(boundary).passed);
}

}  // namespace
