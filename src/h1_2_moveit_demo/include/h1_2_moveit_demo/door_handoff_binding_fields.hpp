#ifndef H1_2_MOVEIT_DEMO__DOOR_HANDOFF_BINDING_FIELDS_HPP_
#define H1_2_MOVEIT_DEMO__DOOR_HANDOFF_BINDING_FIELDS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace h1_2_moveit_demo::door_handoff_path_certificate
{

enum class BindingFieldKind : std::uint8_t
{
  kDirectFile,
  kCompositeBundle,
};

struct BindingFieldSpec
{
  std::string_view name;
  BindingFieldKind kind;
};

inline constexpr std::array<BindingFieldSpec, 18U> kBindingFieldRegistry{
  BindingFieldSpec{"source_sha256", BindingFieldKind::kDirectFile},
  BindingFieldSpec{"executable_sha256", BindingFieldKind::kDirectFile},
  BindingFieldSpec{"urdf_sha256", BindingFieldKind::kCompositeBundle},
  BindingFieldSpec{"srdf_sha256", BindingFieldKind::kCompositeBundle},
  BindingFieldSpec{"kinematics_sha256", BindingFieldKind::kCompositeBundle},
  BindingFieldSpec{"joint_limits_sha256", BindingFieldKind::kCompositeBundle},
  BindingFieldSpec{"tcp_definition_sha256", BindingFieldKind::kCompositeBundle},
  BindingFieldSpec{
    "collision_mesh_bundle_sha256", BindingFieldKind::kCompositeBundle},
  BindingFieldSpec{"original_handle_stl_sha256", BindingFieldKind::kDirectFile},
  BindingFieldSpec{"original_door_stl_sha256", BindingFieldKind::kDirectFile},
  BindingFieldSpec{"original_body_stl_sha256", BindingFieldKind::kDirectFile},
  BindingFieldSpec{"original_tray_stl_sha256", BindingFieldKind::kDirectFile},
  BindingFieldSpec{"static_scene_sha256", BindingFieldKind::kCompositeBundle},
  BindingFieldSpec{
    "scene_pose_and_hinge_sha256", BindingFieldKind::kCompositeBundle},
  BindingFieldSpec{"parameter_set_sha256", BindingFieldKind::kCompositeBundle},
  BindingFieldSpec{
    "start_full_joint_state_sha256", BindingFieldKind::kCompositeBundle},
  BindingFieldSpec{
    "runtime_versions_sha256", BindingFieldKind::kCompositeBundle},
  BindingFieldSpec{"timing_policy_sha256", BindingFieldKind::kCompositeBundle},
};

constexpr std::size_t binding_field_count(const BindingFieldKind kind)
{
  std::size_t count = 0U;
  for (const auto & field : kBindingFieldRegistry) {
    if (field.kind == kind) {
      ++count;
    }
  }
  return count;
}

constexpr bool binding_field_has_kind(
  const std::string_view name,
  const BindingFieldKind kind)
{
  for (const auto & field : kBindingFieldRegistry) {
    if (field.name == name) {
      return field.kind == kind;
    }
  }
  return false;
}

constexpr bool binding_field_names_are_unique()
{
  for (std::size_t left = 0U; left < kBindingFieldRegistry.size(); ++left) {
    for (std::size_t right = left + 1U;
      right < kBindingFieldRegistry.size(); ++right)
    {
      if (kBindingFieldRegistry[left].name == kBindingFieldRegistry[right].name) {
        return false;
      }
    }
  }
  return true;
}

static_assert(kBindingFieldRegistry.size() == 18U);
static_assert(binding_field_count(BindingFieldKind::kCompositeBundle) == 12U);
static_assert(binding_field_count(BindingFieldKind::kDirectFile) == 6U);
static_assert(binding_field_names_are_unique());

}  // namespace h1_2_moveit_demo::door_handoff_path_certificate

#endif  // H1_2_MOVEIT_DEMO__DOOR_HANDOFF_BINDING_FIELDS_HPP_
