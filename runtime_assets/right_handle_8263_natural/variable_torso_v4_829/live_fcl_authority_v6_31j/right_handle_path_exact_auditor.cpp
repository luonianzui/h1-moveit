#define main candidate3_middle_exact_auditor_disabled_main
#include "candidate3_middle_exact_auditor.cpp"
#undef main

namespace
{
constexpr char kPathAuditSchema[] = "h1_right_handle_path_exact_audit/v2";
constexpr char kPathFailureSchema[] = "h1_right_handle_path_exact_audit_failure/v2";
constexpr char kPathAuditorVersion[] = "h1_right_handle_path_exact_auditor/829-a1-torso-margin";
constexpr double kDenseAngularStep = 0.1 * kPi / 180.0;
constexpr double kDensePrismaticStep = 0.0001;
constexpr double kTorsoLimit = 45.0 * kPi / 180.0;
constexpr double kArmMargin = 2.0 * kPi / 180.0;
constexpr double kAdjacentLimit = 4.9 * kPi / 180.0;
constexpr double kNonpadHandleClearance = 0.0006;
constexpr double kPathLeftHandleClearance = 0.010;
constexpr double kPatchSpan = 0.0001;
constexpr double kPatchStation = 0.015;
constexpr double kPatchRadius = 0.035;
constexpr std::size_t kPatchMaxContacts = 4096U;
constexpr double kOpenFingerPosition = -0.019;

const std::array<std::string, 7> kRightArmVariables{
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint",
  "right_wrist_roll_joint", "right_wrist_pitch_joint",
  "right_wrist_yaw_joint"};
const std::array<std::string, 7> kLeftArmVariables{
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint", "left_elbow_joint",
  "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"};
const std::array<std::string, 2> kRightFingerVariables{
  "right_dex1_finger_1_joint", "right_dex1_finger_2_joint"};

struct PathArguments
{
  std::filesystem::path scene;
  std::filesystem::path path;
  std::filesystem::path output;
  std::filesystem::path urdf;
  std::filesystem::path srdf;
  std::filesystem::path handle_stl;
  double park_left_roll{0.0};
  std::array<double, 7> lift_arm{};
  std::array<double, 7> stow_arm{};
  std::array<double, 7> q0_outer_arm{};
  std::array<double, 7> q90_outer_arm{};
  std::filesystem::path q0_ingress;
};

struct PathRow
{
  std::size_t index{0U};
  double handle_angle{0.0};
  std::size_t contact_index{0U};
  std::string action;
  std::string geometry_source;
  double phase_deg{0.0};
  double z_m{0.0};
  double palm_tilt_deg{0.0};
  double side_tilt_deg{0.0};
  double outward_m{0.0};
  double local_x_m{0.0};
  double q1_m{0.0};
  double q2_m{0.0};
  double torso{0.0};
  std::array<double, 7> arm{};
};

struct PathAsset
{
  std::string sha256;
  std::string asset_id;
  bool variable_torso{false};
  double torso_abs_limit{0.0};
  std::vector<PathRow> rows;
};

struct IngressAsset
{
  std::string sha256;
  std::string scene_cdr_sha256;
  double park_left_roll{0.0};
  std::vector<std::array<double, 8>> waypoints;
};

struct ContactPatch
{
  std::size_t contacts{0U};
  bool saturated{false};
  double minimum_station{std::numeric_limits<double>::infinity()};
  double maximum_station{-std::numeric_limits<double>::infinity()};
  Eigen::Vector3d minimum_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d maximum_point{Eigen::Vector3d::Zero()};

  double span() const
  {
    return contacts == 0U ? 0.0 : maximum_station - minimum_station;
  }
  double station() const
  {
    return contacts == 0U ? 0.0 : 0.5 * (minimum_station + maximum_station);
  }
  Eigen::Vector3d center() const
  {
    return 0.5 * (minimum_point + maximum_point);
  }
};

std::string read_text_file(const std::filesystem::path & path)
{
  const auto bytes = read_bytes(path);
  return std::string(bytes.begin(), bytes.end());
}

struct PatchPair
{
  ContactPatch pad1;
  ContactPatch pad2;
};

struct HandleGeometry
{
  shapes::ShapeConstPtr shape;
  Eigen::Isometry3d q0_pose{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d pivot{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rotation_axis{Eigen::Vector3d::UnitY()};
};

struct DirectionMetric
{
  std::string name;
  std::size_t samples{0U};
  std::map<std::string, std::size_t> stage_samples;
};

struct PathAuditMetric
{
  std::size_t samples{0U};
  std::size_t contact_samples{0U};
  std::size_t pad1_patch_samples{0U};
  std::size_t pad2_patch_samples{0U};
  double minimum_arm_margin{std::numeric_limits<double>::infinity()};
  double maximum_abs_torso{0.0};
  double maximum_dense_revolute_step{0.0};
  double maximum_dense_prismatic_step{0.0};
  double maximum_dense_handle_step{0.0};
  double minimum_nonpad_handle_clearance{std::numeric_limits<double>::infinity()};
  double minimum_left_handle_clearance{std::numeric_limits<double>::infinity()};
  double minimum_patch_span{std::numeric_limits<double>::infinity()};
  double minimum_patch_station{std::numeric_limits<double>::infinity()};
  double minimum_patch_radius{std::numeric_limits<double>::infinity()};
  double launch_left_roll{0.0};
  double park_left_roll{0.0};
  std::array<double, 7> launch_left_arm{};
  std::array<double, 7> park_left_arm{};
  std::array<double, 7> lift_arm{};
  std::array<double, 7> stow_arm{};
  std::array<double, 7> q0_outer_arm{};
  std::array<double, 7> q90_outer_arm{};
  std::string q0_ingress_sha256;
  std::size_t q0_ingress_waypoints{0U};
  std::vector<DirectionMetric> directions;
};

struct SampleContext
{
  std::string direction;
  std::string stage;
  std::size_t dense_index{0U};
  std::size_t source_first{0U};
  std::size_t source_second{0U};
};

PathArguments path_arguments(const int argc, char ** argv)
{
  const auto required = [&](const std::string & name) {
      const auto value = option(argc, argv, name);
      if (!value || value->empty()) {
        throw std::runtime_error("ARGUMENT_MISSING:" + name);
      }
      return std::filesystem::path(*value);
    };
  const auto vector7 = [&](const std::string & name) {
      const auto value = option(argc, argv, name);
      if (!value || value->empty()) {
        throw std::runtime_error("ARGUMENT_MISSING:" + name);
      }
      std::array<double, 7> output{};
      std::istringstream stream(*value);
      std::string field;
      std::size_t index = 0U;
      while (std::getline(stream, field, ',')) {
        if (index >= output.size()) {
          throw std::runtime_error("ARGUMENT_VECTOR_WIDTH:" + name);
        }
        output[index] = std::stod(field);
        if (!std::isfinite(output[index])) {
          throw std::runtime_error("ARGUMENT_VECTOR_NONFINITE:" + name);
        }
        ++index;
      }
      if (index != output.size()) {
        throw std::runtime_error("ARGUMENT_VECTOR_WIDTH:" + name);
      }
      return output;
    };
  const auto park_text = option(argc, argv, "--park-left-roll");
  if (!park_text || park_text->empty()) {
    throw std::runtime_error("ARGUMENT_MISSING:--park-left-roll");
  }
  const double park = std::stod(*park_text);
  if (!std::isfinite(park)) {
    throw std::runtime_error("ARGUMENT_NONFINITE:--park-left-roll");
  }
  const auto ingress_text = option(argc, argv, "--q0-ingress");
  return PathArguments{
    required("--scene"), required("--path"), required("--output"),
    required("--urdf"), required("--srdf"), required("--handle-stl"), park,
    vector7("--lift-arm"), vector7("--stow-arm"),
    vector7("--q0-outer-arm"), vector7("--q90-outer-arm"),
    ingress_text ? std::filesystem::path(*ingress_text) : std::filesystem::path{}};
}

double finite_scalar(const YAML::Node & node, const std::string & status)
{
  if (!node || !node.IsScalar()) {
    throw std::runtime_error(status);
  }
  const double value = node.as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error(status);
  }
  return value;
}

PathAsset load_path_asset(const std::filesystem::path & path)
{
  if (std::filesystem::is_symlink(path)) {
    throw std::runtime_error("PATH_ASSET_SYMLINK");
  }
  PathAsset output;
  const auto bytes = read_bytes(path);
  output.sha256 = sha256_hex(bytes);
  const auto root = YAML::Load(std::string(bytes.begin(), bytes.end()));
  const auto schema = scalar_string(root["$schema"], "PATH_SCHEMA");
  const int schema_version = root["schema_version"].as<int>();
  const bool supported_v1 =
    schema == "h1_moveit/right_handle_q0_q90_fixed_torso_path/v1" &&
    schema_version == 1;
  const bool supported_natural_v2 =
    schema == "h1_moveit/right_handle_q0_q90_fixed_torso_natural_v2_path/v2" &&
    schema_version == 2;
  const bool supported_variable_v3 =
    schema == "h1_moveit/right_handle_variable_torso_path/v3" &&
    schema_version == 3;
  const bool supported_variable_v4 =
    schema == "h1_moveit/right_handle_variable_torso_path/v4" &&
    schema_version == 4;
  const bool supported_variable = supported_variable_v3 || supported_variable_v4;
  if (!root.IsMap() ||
    (!supported_v1 && !supported_natural_v2 && !supported_variable))
  {
    throw std::runtime_error("PATH_CONTRACT");
  }
  output.variable_torso = supported_variable;
  output.asset_id = scalar_string(root["asset_id"], "PATH_ASSET_ID");
  const auto status = root["status"];
  const auto scope = root["scope"];
  const auto contract = root["contract"];
  if (!status.IsMap() || !scope.IsMap() || !contract.IsMap()) {
    throw std::runtime_error("PATH_FAIL_CLOSED_CONTRACT");
  }
  const bool legacy_status_valid = !supported_variable_v4 &&
    status["offline_kinematic_path_valid"].as<bool>() &&
    status["forward_discrete_path_valid"].as<bool>() &&
    status["reverse_discrete_path_valid"].as<bool>();
  // A v4 retarget is deliberately a non-self-authorizing candidate.  This
  // auditor is the authority that proves its dense, bidirectional topology;
  // requiring those flags to be pre-set would create a circular certificate.
  const bool candidate_status_valid = supported_variable_v4 &&
    status["ik_retarget_complete"].as<bool>() &&
    !status["offline_kinematic_path_valid"].as<bool>() &&
    !status["forward_discrete_path_valid"].as<bool>() &&
    !status["reverse_discrete_path_valid"].as<bool>() &&
    !status["forward_dense_exact_complete"].as<bool>() &&
    !status["reverse_dense_exact_complete"].as<bool>() &&
    !status["full_execution_topology_exact_complete"].as<bool>() &&
    !status["execution_ready"].as<bool>() &&
    !status["gazebo_slow_contact_dynamics_complete"].as<bool>();
  const double contract_torso_limit_deg =
    finite_scalar(contract["torso_abs_limit_deg"], "PATH_TORSO_LIMIT");
  const double maximum_contract_torso_deg = supported_variable_v4 ? 44.5 : 45.0;
  if ((!legacy_status_valid && !candidate_status_valid) ||
    !status["exact_q90_endpoint_present"].as<bool>() ||
    !(supported_variable ? status["torso_variable"].as<bool>() :
    status["torso_fixed"].as<bool>()) || !status["fail_closed"].as<bool>() ||
    scope["handle_arm"].as<std::string>() != "right" ||
    scope["left_hand_handle_contact_allowed"].as<bool>() ||
    scope["controller_commands_issued_by_this_generation"].as<int>() != 0 ||
    std::abs(finite_scalar(contract["handle_start_deg"], "PATH_START") - 0.0) > 1e-12 ||
    std::abs(finite_scalar(contract["handle_end_deg"], "PATH_END") - 90.0) > 1e-12 ||
    contract_torso_limit_deg <= 0.0 ||
    contract_torso_limit_deg > maximum_contract_torso_deg + 1e-12 ||
    finite_scalar(contract["minimum_arm_joint_limit_margin_deg"], "PATH_MARGIN") < 2.0 - 1e-12 ||
    finite_scalar(contract["maximum_adjacent_joint_step_deg"], "PATH_STEP") > 4.9 + 1e-12 ||
    !contract["all_nonpad_handle_contact_forbidden"].as<bool>() ||
    !contract["partial_path_forbidden"].as<bool>())
  {
    throw std::runtime_error("PATH_FAIL_CLOSED_CONTRACT");
  }
  output.torso_abs_limit = contract_torso_limit_deg * kPi / 180.0;
  const auto rows = root["rows"];
  if (!rows.IsSequence() || rows.size() < 2U) {
    throw std::runtime_error("PATH_ROWS");
  }
  output.rows.reserve(rows.size());
  double previous_angle = -1.0;
  double fixed_torso = 0.0;
  for (std::size_t index = 0U; index < rows.size(); ++index) {
    const auto node = rows[index];
    PathRow row;
    row.index = node[supported_variable ? "index" : "sample_index"].as<std::size_t>();
    row.handle_angle = finite_scalar(node["handle_angle_deg"], "PATH_HANDLE") * kPi / 180.0;
    row.contact_index = node["contact_index"].as<std::size_t>();
    row.action = scalar_string(node["action"], "PATH_ACTION");
    row.geometry_source = scalar_string(node["geometry_source"], "PATH_GEOMETRY_SOURCE");
    row.phase_deg = finite_scalar(node["phase_deg"], "PATH_PHASE");
    row.z_m = finite_scalar(node["z_mm"], "PATH_Z") * 0.001;
    row.palm_tilt_deg = finite_scalar(node["palm_down_tilt_deg"], "PATH_PALM_TILT");
    row.side_tilt_deg = finite_scalar(node["side_tilt_deg"], "PATH_SIDE_TILT");
    row.outward_m = finite_scalar(node["outward_offset_mm"], "PATH_OUTWARD") * 0.001;
    row.local_x_m = finite_scalar(node["local_x_offset_mm"], "PATH_LOCAL_X") * 0.001;
    row.q1_m = finite_scalar(node["right_pad1_slider_mm"], "PATH_Q1") * 0.001;
    row.q2_m = finite_scalar(node["right_pad2_slider_mm"], "PATH_Q2") * 0.001;
    row.torso = supported_variable ?
      finite_scalar(node["torso_yaw_deg"], "PATH_TORSO") * kPi / 180.0 :
      finite_scalar(node["torso_yaw_rad"], "PATH_TORSO");
    const auto arm = node["right_arm_rad"];
    if (row.index != index || !arm.IsSequence() || arm.size() != row.arm.size() ||
      row.handle_angle + 1e-12 < previous_angle || row.handle_angle < -1e-12 ||
      row.handle_angle > kPi / 2.0 + 1e-12 ||
      std::abs(row.torso) > output.torso_abs_limit + 1e-12)
    {
      throw std::runtime_error("PATH_ROW_SHAPE:" + std::to_string(index));
    }
    for (std::size_t joint = 0U; joint < row.arm.size(); ++joint) {
      row.arm[joint] = finite_scalar(arm[joint], "PATH_ARM");
    }
    if (index == 0U) {
      fixed_torso = row.torso;
    } else if (!supported_variable && std::abs(row.torso - fixed_torso) > 2e-12) {
      throw std::runtime_error("PATH_TORSO_NOT_FIXED");
    }
    const auto gate = node["original_stl_gate"];
    if (!gate.IsMap() || !gate["pass_"].as<bool>() ||
      !gate["pad1_contact_authorized"].as<bool>() ||
      !gate["nonpad_contact_forbidden"].as<bool>())
    {
      throw std::runtime_error("PATH_DISCRETE_GATE:" + std::to_string(index));
    }
    previous_angle = row.handle_angle;
    output.rows.push_back(std::move(row));
  }
  if (std::abs(output.rows.front().handle_angle) > 1e-12 ||
    std::abs(output.rows.back().handle_angle - kPi / 2.0) > 1e-12)
  {
    throw std::runtime_error("PATH_ENDPOINTS");
  }
  return output;
}

IngressAsset load_ingress_asset(const std::filesystem::path & path)
{
  if (path.empty() || std::filesystem::is_symlink(path)) {
    throw std::runtime_error("INGRESS_ASSET_PATH");
  }
  const auto bytes = read_bytes(path);
  const auto root = YAML::Load(std::string(bytes.begin(), bytes.end()));
  if (!root.IsMap() ||
    scalar_string(root["schema"], "INGRESS_SCHEMA") !=
    "h1_right_handle_splice_search/v1" ||
    scalar_string(root["status"], "INGRESS_STATUS") != "PASS" ||
    !root["valid"].as<bool>() || root["controller_commands_issued"].as<int>() != 0 ||
    !root["offline_only"].as<bool>() ||
    std::abs(finite_scalar(root["handle_angle_deg"], "INGRESS_HANDLE")) > 1e-12)
  {
    throw std::runtime_error("INGRESS_CONTRACT");
  }
  IngressAsset output;
  output.sha256 = sha256_hex(bytes);
  output.scene_cdr_sha256 = scalar_string(root["scene_cdr_sha256"], "INGRESS_SCENE_SHA");
  output.park_left_roll = finite_scalar(
    root["park_left_shoulder_roll_rad"], "INGRESS_PARK_LEFT");
  const auto order = root["configuration_order"];
  const std::array<std::string, 8> expected_order{
    "torso_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint"};
  if (!order.IsSequence() || order.size() != expected_order.size()) {
    throw std::runtime_error("INGRESS_ORDER");
  }
  for (std::size_t index = 0U; index < expected_order.size(); ++index) {
    if (order[index].as<std::string>() != expected_order[index]) {
      throw std::runtime_error("INGRESS_ORDER");
    }
  }
  const auto fingers = root["right_fingers_rad_or_m"];
  if (!fingers.IsMap() ||
    std::abs(finite_scalar(fingers[kRightFingerVariables[0]], "INGRESS_Q1") -
    kOpenFingerPosition) > 1e-12 ||
    std::abs(finite_scalar(fingers[kRightFingerVariables[1]], "INGRESS_Q2") -
    kOpenFingerPosition) > 1e-12)
  {
    throw std::runtime_error("INGRESS_FINGERS");
  }
  const auto waypoints = root["waypoints"];
  if (!waypoints.IsSequence() || waypoints.size() < 2U ||
    waypoints.size() != root["waypoint_count"].as<std::size_t>())
  {
    throw std::runtime_error("INGRESS_WAYPOINTS");
  }
  output.waypoints.reserve(waypoints.size());
  for (const auto & node : waypoints) {
    if (!node.IsSequence() || node.size() != 8U) {
      throw std::runtime_error("INGRESS_WAYPOINT_SHAPE");
    }
    std::array<double, 8> waypoint{};
    for (std::size_t index = 0U; index < waypoint.size(); ++index) {
      waypoint[index] = finite_scalar(node[index], "INGRESS_WAYPOINT");
    }
    output.waypoints.push_back(waypoint);
  }
  return output;
}

moveit::core::RobotState row_state(
  const moveit::core::RobotState & seam, const PathRow & row,
  const bool open_fingers)
{
  moveit::core::RobotState output(seam);
  output.setVariablePosition("torso_joint", row.torso);
  for (std::size_t index = 0U; index < row.arm.size(); ++index) {
    output.setVariablePosition(kRightArmVariables[index], row.arm[index]);
  }
  output.setVariablePosition(
    kRightFingerVariables[0], open_fingers ? kOpenFingerPosition : row.q1_m);
  output.setVariablePosition(
    kRightFingerVariables[1], open_fingers ? kOpenFingerPosition : row.q2_m);
  output.updateCollisionBodyTransforms();
  return output;
}

moveit::core::RobotState ingress_state(
  const moveit::core::RobotState & park, const std::array<double, 8> & waypoint)
{
  moveit::core::RobotState output(park);
  output.setVariablePosition("torso_joint", waypoint[0]);
  for (std::size_t index = 0U; index < kRightArmVariables.size(); ++index) {
    output.setVariablePosition(kRightArmVariables[index], waypoint[index + 1U]);
  }
  for (const auto & finger : kRightFingerVariables) {
    output.setVariablePosition(finger, kOpenFingerPosition);
  }
  output.updateCollisionBodyTransforms();
  return output;
}

void verify_seam(const moveit::core::RobotState & seam)
{
  const std::array<double, 7> nominal{0.0, -0.25, 0.0, 0.60, 0.0, 0.0, 0.0};
  if (std::abs(seam.getVariablePosition("torso_joint")) > 0.003) {
    throw std::runtime_error("SEAM_TORSO_NOT_NEUTRAL");
  }
  for (std::size_t index = 0U; index < nominal.size(); ++index) {
    if (std::abs(seam.getVariablePosition(kRightArmVariables[index]) - nominal[index]) > 0.004) {
      throw std::runtime_error("SEAM_RIGHT_ARM:" + kRightArmVariables[index]);
    }
  }
  for (const auto & finger : kRightFingerVariables) {
    if (std::abs(seam.getVariablePosition(finger) - kOpenFingerPosition) > 0.001) {
      throw std::runtime_error("SEAM_RIGHT_FINGER:" + finger);
    }
  }
}

HandleGeometry prepare_handle_geometry(planning_scene::PlanningScene & scene)
{
  const auto handle = scene.getWorld()->getObject(kHandleId);
  if (!handle || handle->shapes_.size() != 1U || handle->global_shape_poses_.size() != 1U) {
    throw std::runtime_error("HANDLE_WORLD_GEOMETRY");
  }
  HandleGeometry output;
  output.shape = handle->shapes_.front();
  output.q0_pose = handle->global_shape_poses_.front();
  const Eigen::Vector3d pivot_local(
    0.009999668689, -0.000027284668, 0.119003896682);
  output.pivot = output.q0_pose * pivot_local;
  output.rotation_axis = (output.q0_pose.linear() * Eigen::Vector3d::UnitY()).normalized();
  return output;
}

void verify_collision_geometry_fail_closed(
  const planning_scene::PlanningScene & scene)
{
  const std::array<std::string, 27U> required_links{
    "torso_link",
    "right_shoulder_pitch_link", "right_shoulder_roll_link",
    "right_shoulder_yaw_link", "right_elbow_link",
    "right_wrist_roll_link", "right_wrist_pitch_link",
    "right_dex1_adapter_link", "right_dex1_base_link",
    "right_dex1_finger_1_slider_link", "right_dex1_finger_1_link",
    "right_dex1_finger_1_pad_link", "right_dex1_finger_2_slider_link",
    "right_dex1_finger_2_link", "right_dex1_finger_2_pad_link",
    "left_shoulder_pitch_link", "left_shoulder_roll_link",
    "left_shoulder_yaw_link", "left_elbow_link",
    "left_wrist_roll_link", "left_wrist_pitch_link",
    "left_dex1_adapter_link", "left_dex1_base_link",
    "left_dex1_finger_1_slider_link", "left_dex1_finger_1_link",
    "left_dex1_finger_1_pad_link", "left_dex1_finger_2_pad_link"};
  const auto valid_shape = [](const shapes::ShapeConstPtr & shape) {
      if (!shape) {
        return false;
      }
      const auto * mesh = dynamic_cast<const shapes::Mesh *>(shape.get());
      return mesh == nullptr || (mesh->vertex_count > 0U && mesh->triangle_count > 0U);
    };
  for (const auto & name : required_links) {
    const auto * link = scene.getRobotModel()->getLinkModel(name);
    if (!link || link->getShapes().empty() ||
      !std::all_of(link->getShapes().begin(), link->getShapes().end(), valid_shape))
    {
      throw std::runtime_error("MESH_GEOMETRY_MISSING:ROBOT_LINK:" + name);
    }
  }
  const auto object_ids = scene.getWorld()->getObjectIds();
  if (object_ids.empty()) {
    throw std::runtime_error("MESH_GEOMETRY_MISSING:WORLD_EMPTY");
  }
  for (const auto & id : object_ids) {
    const auto object = scene.getWorld()->getObject(id);
    if (!object || object->shapes_.empty() ||
      object->shapes_.size() != object->global_shape_poses_.size() ||
      !std::all_of(object->shapes_.begin(), object->shapes_.end(), valid_shape))
    {
      throw std::runtime_error("MESH_GEOMETRY_MISSING:WORLD_OBJECT:" + id);
    }
  }
  const auto handle = scene.getWorld()->getObject(kHandleId);
  const auto door = scene.getWorld()->getObject("incubator_door_exact");
  if (!handle || !door) {
    throw std::runtime_error("MESH_GEOMETRY_MISSING:EXACT_MECHANISM");
  }
}

Eigen::Isometry3d handle_pose(const HandleGeometry & geometry, const double angle)
{
  Eigen::Isometry3d rotation = Eigen::Isometry3d::Identity();
  rotation.linear() = Eigen::AngleAxisd(angle, geometry.rotation_axis).toRotationMatrix();
  rotation.translation() = geometry.pivot - rotation.linear() * geometry.pivot;
  return rotation * geometry.q0_pose;
}

ContactPatch pad_patch(
  FclGeometryCache & cache, const moveit::core::RobotState & state,
  const std::string & pad_link, const HandleGeometry & handle,
  const Eigen::Isometry3d & handle_global)
{
  const auto * link = state.getRobotModel()->getLinkModel(pad_link);
  if (!link || link->getShapes().empty()) {
    throw std::runtime_error("PAD_COLLISION_GEOMETRY:" + pad_link);
  }
  fcl::CollisionObjectd handle_object(cache.get(handle.shape), fcl::Transform3d(handle_global));
  ContactPatch output;
  const Eigen::Vector3d pivot = handle_global * Eigen::Vector3d(
    0.009999668689, -0.000027284668, 0.119003896682);
  const Eigen::Vector3d station_axis =
    (handle_global.linear() * -Eigen::Vector3d::UnitZ()).normalized();
  for (std::size_t shape_index = 0U; shape_index < link->getShapes().size(); ++shape_index) {
    fcl::CollisionObjectd pad_object(
      cache.get(link->getShapes()[shape_index]),
      fcl::Transform3d(state.getCollisionBodyTransform(link, shape_index)));
    fcl::CollisionRequestd request(kPatchMaxContacts, true);
    fcl::CollisionResultd result;
    fcl::collide(&pad_object, &handle_object, request, result);
    output.saturated = output.saturated || result.numContacts() >= kPatchMaxContacts;
    std::vector<fcl::Contactd> contacts;
    result.getContacts(contacts);
    output.contacts += contacts.size();
    for (const auto & contact : contacts) {
      const double station = station_axis.dot(contact.pos - pivot);
      if (station < output.minimum_station) {
        output.minimum_station = station;
        output.minimum_point = contact.pos;
      }
      if (station > output.maximum_station) {
        output.maximum_station = station;
        output.maximum_point = contact.pos;
      }
    }
  }
  return output;
}

bool valid_patch(const ContactPatch & patch, double & radius)
{
  radius = patch.contacts == 0U ? 0.0 : patch.center().norm();
  return patch.contacts != 0U && !patch.saturated &&
         patch.span() + 1e-12 >= kPatchSpan &&
         patch.station() + 1e-12 >= kPatchStation;
}

std::pair<double, std::string> handle_distance(
  planning_scene::PlanningScene & scene, const moveit::core::RobotState & state,
  const collision_detection::AllowedCollisionMatrix & base_acm,
  const std::vector<std::string> & selected_links, const double threshold)
{
  auto acm = base_acm;
  const auto collision_links = scene.getRobotModel()->getLinkModelNamesWithCollisionGeometry();
  for (std::size_t first = 0U; first < collision_links.size(); ++first) {
    for (std::size_t second = first; second < collision_links.size(); ++second) {
      acm.setEntry(collision_links[first], collision_links[second], true);
    }
  }
  for (const auto & object : scene.getWorld()->getObjectIds()) {
    for (const auto & link : collision_links) {
      acm.setEntry(object, link, true);
    }
  }
  for (const auto & link : selected_links) {
    acm.setEntry(kHandleId, link, false);
  }
  collision_detection::DistanceRequest request;
  request.type = collision_detection::DistanceRequestTypes::ALL;
  request.enable_nearest_points = true;
  request.enable_signed_distance = true;
  request.max_contacts_per_body = 1U;
  request.distance_threshold = threshold;
  request.acm = &acm;
  collision_detection::DistanceResult result;
  scene.getCollisionEnvUnpadded()->distanceRobot(request, result, state);
  double minimum = threshold;
  std::string pair;
  for (const auto & entries : result.distances) {
    for (const auto & value : entries.second) {
      if ((value.link_names[0] == kHandleId || value.link_names[1] == kHandleId) &&
        std::isfinite(value.distance) && value.distance < minimum)
      {
        minimum = value.distance;
        pair = value.link_names[0] + ":" + value.link_names[1];
      }
    }
  }
  return {minimum, pair};
}

std::string collision_pair(const collision_detection::CollisionResult & result)
{
  if (result.contacts.empty()) {
    return "unknown";
  }
  return result.contacts.begin()->first.first + ":" + result.contacts.begin()->first.second;
}

double arm_margin(const moveit::core::RobotState & state)
{
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto & variable : kRightArmVariables) {
    const auto & bounds = state.getRobotModel()->getVariableBounds(variable);
    if (!bounds.position_bounded_) {
      throw std::runtime_error("ARM_UNBOUNDED:" + variable);
    }
    const double position = state.getVariablePosition(variable);
    minimum = std::min(
      minimum, std::min(position - bounds.min_position_, bounds.max_position_ - position));
  }
  return minimum;
}

std::size_t edge_subdivisions(
  const moveit::core::RobotState & first, const moveit::core::RobotState & second,
  const double first_handle, const double second_handle,
  const PathRow * first_row, const PathRow * second_row)
{
  std::size_t subdivisions = std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(
    std::abs(second_handle - first_handle) / kDenseAngularStep)));
  for (const auto & variable : kRightArmVariables) {
    subdivisions = std::max<std::size_t>(subdivisions, static_cast<std::size_t>(std::ceil(
      std::abs(second.getVariablePosition(variable) - first.getVariablePosition(variable)) /
      kDenseAngularStep)));
  }
  subdivisions = std::max<std::size_t>(subdivisions, static_cast<std::size_t>(std::ceil(
    std::abs(second.getVariablePosition("torso_joint") -
    first.getVariablePosition("torso_joint")) / kDenseAngularStep)));
  subdivisions = std::max<std::size_t>(subdivisions, static_cast<std::size_t>(std::ceil(
    std::abs(second.getVariablePosition("left_shoulder_roll_joint") -
    first.getVariablePosition("left_shoulder_roll_joint")) / kDenseAngularStep)));
  for (const auto & variable : kRightFingerVariables) {
    subdivisions = std::max<std::size_t>(subdivisions, static_cast<std::size_t>(std::ceil(
      std::abs(second.getVariablePosition(variable) - first.getVariablePosition(variable)) /
      kDensePrismaticStep)));
  }
  if (first_row != nullptr && second_row != nullptr) {
    for (const double angular_delta : {
      std::abs(second_row->phase_deg - first_row->phase_deg),
      std::abs(second_row->palm_tilt_deg - first_row->palm_tilt_deg),
      std::abs(second_row->side_tilt_deg - first_row->side_tilt_deg)})
    {
      subdivisions = std::max<std::size_t>(subdivisions,
        static_cast<std::size_t>(std::ceil(angular_delta / 0.1)));
    }
    for (const double linear_delta : {
      std::abs(second_row->z_m - first_row->z_m),
      std::abs(second_row->outward_m - first_row->outward_m),
      std::abs(second_row->local_x_m - first_row->local_x_m)})
    {
      subdivisions = std::max<std::size_t>(subdivisions,
        static_cast<std::size_t>(std::ceil(linear_delta / kDensePrismaticStep)));
    }
  }
  return subdivisions;
}

class ExactPathAuditor
{
public:
  ExactPathAuditor(
    planning_scene::PlanningScene & scene, const PathAsset & asset,
    const IngressAsset & ingress,
    const double park_left_roll, const std::array<double, 7> & lift_arm,
    const std::array<double, 7> & stow_arm,
    const std::array<double, 7> & q0_outer_arm,
    const std::array<double, 7> & q90_outer_arm)
  : scene_(scene), asset_(asset), ingress_(ingress), handle_(prepare_handle_geometry(scene)),
    launch_(scene.getCurrentState()), park_(launch_), base_acm_(scene.getAllowedCollisionMatrix()),
    lift_arm_(lift_arm), stow_arm_(stow_arm), q0_outer_arm_(q0_outer_arm),
    q90_outer_arm_(q90_outer_arm)
  {
    verify_seam(launch_);
    park_.setVariablePosition("left_shoulder_roll_joint", park_left_roll);
    park_.updateCollisionBodyTransforms();
    if (!park_.satisfiesBounds(1e-9)) {
      throw std::runtime_error("PARK_LEFT_ROLL_BOUNDS");
    }
    metric_.launch_left_roll = launch_.getVariablePosition("left_shoulder_roll_joint");
    metric_.park_left_roll = park_left_roll;
    for (std::size_t index = 0U; index < kLeftArmVariables.size(); ++index) {
      metric_.launch_left_arm[index] = launch_.getVariablePosition(kLeftArmVariables[index]);
      metric_.park_left_arm[index] = park_.getVariablePosition(kLeftArmVariables[index]);
    }
    metric_.lift_arm = lift_arm_;
    metric_.stow_arm = stow_arm_;
    metric_.q0_outer_arm = q0_outer_arm_;
    metric_.q90_outer_arm = q90_outer_arm_;
    metric_.q0_ingress_sha256 = ingress_.sha256;
    metric_.q0_ingress_waypoints = ingress_.waypoints.size();
    if (asset_.variable_torso) {
      if (ingress_.waypoints.empty() ||
        std::abs(ingress_.park_left_roll - park_left_roll) > 1e-12)
      {
        throw std::runtime_error("VARIABLE_PATH_INGRESS_CONTRACT");
      }
      const auto & goal = ingress_.waypoints.back();
      if (std::abs(goal[0] - asset_.rows.front().torso) > 2e-12) {
        throw std::runtime_error("VARIABLE_PATH_INGRESS_Q0_TORSO");
      }
      for (std::size_t index = 0U; index < asset_.rows.front().arm.size(); ++index) {
        if (std::abs(goal[index + 1U] - asset_.rows.front().arm[index]) > 2e-12) {
          throw std::runtime_error("VARIABLE_PATH_INGRESS_Q0_ARM");
        }
      }
    }
    collision_links_ = scene_.getRobotModel()->getLinkModelNamesWithCollisionGeometry();
    for (const auto & link : collision_links_) {
      const bool is_pad = link == kRightHandlePads[0] || link == kRightHandlePads[1];
      if (link.rfind("left_", 0U) == 0U) {
        left_links_.push_back(link);
      } else if (!is_pad) {
        nonpad_nonleft_links_.push_back(link);
      }
    }
    if (left_links_.empty() || nonpad_nonleft_links_.empty()) {
      throw std::runtime_error("HANDLE_DISTANCE_LINK_SETS");
    }
    for (const auto & object : scene_.getWorld()->getObjectIds()) {
      for (const auto & link : collision_links_) {
        base_acm_.setEntry(object, link, false);
      }
    }
    base_acm_.setEntry("right_elbow_link", "torso_link", false);
    collision_request_.contacts = true;
    collision_request_.max_contacts = 32U;
    collision_request_.max_contacts_per_pair = 4U;
    collision_request_.pad_self_collisions = false;
    world_request_ = collision_request_;
    world_request_.pad_environment_collisions = false;
  }

  PathAuditMetric run()
  {
    if (asset_.variable_torso) {
      run_variable_topology();
      if (metric_.directions.size() != 2U || metric_.contact_samples == 0U) {
        throw std::runtime_error("AUDIT_SAMPLE_CONTRACT");
      }
      return metric_;
    }
    run_direction(false);
    run_direction(true);
    if (metric_.directions.size() != 2U || metric_.contact_samples == 0U) {
      throw std::runtime_error("AUDIT_SAMPLE_CONTRACT");
    }
    return metric_;
  }

private:
  void fail(const SampleContext & context, const std::string & status) const
  {
    throw std::runtime_error(
            status + ":direction=" + context.direction + ":stage=" + context.stage +
            ":dense=" + std::to_string(context.dense_index) +
            ":source=" + std::to_string(context.source_first) + "-" +
            std::to_string(context.source_second));
  }

  void audit_sample(
    const moveit::core::RobotState & sample, const double angle,
    const bool pads_authorized, const bool contact_required,
    DirectionMetric & direction, const SampleContext & context,
    const moveit::core::RobotState * previous, const double previous_angle)
  {
    if (!sample.satisfiesBounds(1e-9)) {
      fail(context, "JOINT_BOUNDS");
    }
    const double torso = std::abs(sample.getVariablePosition("torso_joint"));
    metric_.maximum_abs_torso = std::max(metric_.maximum_abs_torso, torso);
    if (torso > asset_.torso_abs_limit + 1e-12) {
      fail(context, "TORSO_LIMIT");
    }
    const double margin = arm_margin(sample);
    metric_.minimum_arm_margin = std::min(metric_.minimum_arm_margin, margin);
    if (margin + 1e-12 < kArmMargin) {
      fail(context, "ARM_MARGIN");
    }
    if (previous != nullptr) {
      for (const auto & variable : kRightArmVariables) {
        metric_.maximum_dense_revolute_step = std::max(
          metric_.maximum_dense_revolute_step,
          std::abs(sample.getVariablePosition(variable) - previous->getVariablePosition(variable)));
      }
      metric_.maximum_dense_revolute_step = std::max(
        metric_.maximum_dense_revolute_step,
        std::abs(sample.getVariablePosition("torso_joint") -
        previous->getVariablePosition("torso_joint")));
      metric_.maximum_dense_revolute_step = std::max(
        metric_.maximum_dense_revolute_step,
        std::abs(sample.getVariablePosition("left_shoulder_roll_joint") -
        previous->getVariablePosition("left_shoulder_roll_joint")));
      for (const auto & variable : kRightFingerVariables) {
        metric_.maximum_dense_prismatic_step = std::max(
          metric_.maximum_dense_prismatic_step,
          std::abs(sample.getVariablePosition(variable) - previous->getVariablePosition(variable)));
      }
      metric_.maximum_dense_handle_step = std::max(
        metric_.maximum_dense_handle_step, std::abs(angle - previous_angle));
      if (metric_.maximum_dense_revolute_step > kDenseAngularStep + 1e-10 ||
        metric_.maximum_dense_prismatic_step > kDensePrismaticStep + 1e-10 ||
        metric_.maximum_dense_handle_step > kDenseAngularStep + 1e-10)
      {
        fail(context, "DENSE_STEP");
      }
    }

    const Eigen::Isometry3d global_handle = handle_pose(handle_, angle);
    set_single_shape_pose(scene_, kHandleId, handle_.shape, global_handle);
    auto collision_acm = base_acm_;
    if (pads_authorized) {
      for (const auto & pad : kRightHandlePads) {
        collision_acm.setEntry(kHandleId, pad, true);
      }
    }
    collision_detection::CollisionResult self_result;
    scene_.checkSelfCollision(collision_request_, self_result, sample, collision_acm);
    if (self_result.collision) {
      fail(context, "COLLISION_SELF:" + collision_pair(self_result));
    }
    collision_detection::CollisionResult world_result;
    scene_.getCollisionEnvUnpadded()->checkRobotCollision(
      world_request_, world_result, sample, collision_acm);
    if (world_result.collision) {
      fail(context, "COLLISION_WORLD:" + collision_pair(world_result));
    }

    const auto nonpad = handle_distance(
      scene_, sample, base_acm_, nonpad_nonleft_links_, 0.020);
    metric_.minimum_nonpad_handle_clearance = std::min(
      metric_.minimum_nonpad_handle_clearance, nonpad.first);
    if (nonpad.first + 1e-12 < kNonpadHandleClearance) {
      fail(context, "NONPAD_HANDLE_CLEARANCE:" + nonpad.second);
    }
    const auto left = handle_distance(scene_, sample, base_acm_, left_links_, 0.050);
    metric_.minimum_left_handle_clearance = std::min(
      metric_.minimum_left_handle_clearance, left.first);
    if (left.first + 1e-12 < kPathLeftHandleClearance) {
      fail(context, "LEFT_HANDLE_CLEARANCE:" + left.second);
    }

    if (contact_required) {
      ++metric_.contact_samples;
      const auto pad1 = pad_patch(
        fcl_cache_, sample, kRightHandlePads[0], handle_, global_handle);
      const auto pad2 = pad_patch(
        fcl_cache_, sample, kRightHandlePads[1], handle_, global_handle);
      double radius1 = 0.0;
      double radius2 = 0.0;
      bool valid1 = valid_patch(pad1, radius1);
      bool valid2 = valid_patch(pad2, radius2);
      if (valid1) {
        radius1 = (pad1.center() -
          global_handle * Eigen::Vector3d(0.009999668689, -0.000027284668, 0.119003896682)).norm();
        valid1 = radius1 + 1e-12 >= kPatchRadius;
      }
      if (valid2) {
        radius2 = (pad2.center() -
          global_handle * Eigen::Vector3d(0.009999668689, -0.000027284668, 0.119003896682)).norm();
        valid2 = radius2 + 1e-12 >= kPatchRadius;
      }
      if (!valid1 && !valid2) {
        fail(context, "RIGHT_PAD_PATCH");
      }
      const auto & accepted = valid1 ? pad1 : pad2;
      const double radius = valid1 ? radius1 : radius2;
      metric_.minimum_patch_span = std::min(metric_.minimum_patch_span, accepted.span());
      metric_.minimum_patch_station = std::min(metric_.minimum_patch_station, accepted.station());
      metric_.minimum_patch_radius = std::min(metric_.minimum_patch_radius, radius);
      metric_.pad1_patch_samples += valid1 ? 1U : 0U;
      metric_.pad2_patch_samples += valid2 ? 1U : 0U;
    }
    ++metric_.samples;
    ++direction.samples;
    ++direction.stage_samples[context.stage];
  }

  void audit_edge(
    const moveit::core::RobotState & first, const moveit::core::RobotState & second,
    const double first_angle, const double second_angle,
    const bool pads_authorized, const bool contact_required,
    DirectionMetric & direction, const std::string & stage,
    const std::size_t source_first, const std::size_t source_second,
    const PathRow * first_row = nullptr, const PathRow * second_row = nullptr)
  {
    const std::size_t subdivisions = edge_subdivisions(
      first, second, first_angle, second_angle, first_row, second_row);
    moveit::core::RobotState previous(first);
    double previous_angle = first_angle;
    for (std::size_t subdivision = 1U; subdivision <= subdivisions; ++subdivision) {
      const double fraction = static_cast<double>(subdivision) /
        static_cast<double>(subdivisions);
      moveit::core::RobotState sample(first);
      first.interpolate(second, fraction, sample);
      sample.updateCollisionBodyTransforms();
      const double angle = first_angle + (second_angle - first_angle) * fraction;
      SampleContext context{
        direction.name, stage, direction.samples, source_first, source_second};
      audit_sample(
        sample, angle, pads_authorized, contact_required,
        direction, context, &previous, previous_angle);
      previous = sample;
      previous_angle = angle;
    }
  }

  void run_variable_topology()
  {
    metric_.directions.push_back(DirectionMetric{"prefix"});
    auto & prefix = metric_.directions.back();
    const auto & q0_row = asset_.rows.front();
    const auto & q90_row = asset_.rows.back();
    moveit::core::RobotState current(launch_);
    SampleContext seam_context{"prefix", "seam_start", 0U, 0U, 0U};
    audit_sample(current, 0.0, false, false, prefix, seam_context, nullptr, 0.0);
    audit_edge(
      current, park_, 0.0, 0.0, false, false, prefix,
      "prepare_left_park", 0U, 0U);
    current = park_;
    for (std::size_t index = 0U; index < ingress_.waypoints.size(); ++index) {
      const auto next = ingress_state(park_, ingress_.waypoints[index]);
      audit_edge(
        current, next, 0.0, 0.0, false, false, prefix,
        "ingress_open_to_q0", 0U, 0U);
      current = next;
    }
    const auto q0_open = row_state(park_, q0_row, true);
    for (const auto & variable : kRightArmVariables) {
      if (std::abs(current.getVariablePosition(variable) -
        q0_open.getVariablePosition(variable)) > 2e-12)
      {
        throw std::runtime_error("VARIABLE_PATH_INGRESS_OPEN_SEAM");
      }
    }
    const auto q0_closed = row_state(park_, q0_row, false);
    audit_edge(
      current, q0_closed, 0.0, 0.0, true, false, prefix,
      "close_q0", 0U, 0U);
    SampleContext q0_contact_context{
      "prefix", "contact_q0_to_q90_hold", prefix.samples, 0U, 0U};
    audit_sample(
      q0_closed, 0.0, true, true, prefix, q0_contact_context, nullptr, 0.0);
    current = q0_closed;
    for (std::size_t index = 1U; index < asset_.rows.size(); ++index) {
      const auto next = row_state(park_, asset_.rows[index], false);
      audit_edge(
        current, next, asset_.rows[index - 1U].handle_angle,
        asset_.rows[index].handle_angle, true, true, prefix,
        "contact_q0_to_q90_hold", index - 1U, index,
        &asset_.rows[index - 1U], &asset_.rows[index]);
      current = next;
    }

    metric_.directions.push_back(DirectionMetric{"suffix"});
    auto & suffix = metric_.directions.back();
    SampleContext q90_seam_context{
      "suffix", "q90_continuous_grasp_seam", 0U,
      asset_.rows.size() - 1U, asset_.rows.size() - 1U};
    audit_sample(
      current, q90_row.handle_angle, true, true, suffix,
      q90_seam_context, nullptr, q90_row.handle_angle);
    for (std::size_t index = asset_.rows.size() - 1U; index > 0U; --index) {
      const auto next = row_state(park_, asset_.rows[index - 1U], false);
      audit_edge(
        current, next, asset_.rows[index].handle_angle,
        asset_.rows[index - 1U].handle_angle, true, true, suffix,
        "contact_reverse_q90_to_q0", index, index - 1U,
        &asset_.rows[index], &asset_.rows[index - 1U]);
      current = next;
    }
    const auto q0_released = row_state(park_, q0_row, true);
    audit_edge(
      current, q0_released, 0.0, 0.0, true, false, suffix,
      "release_q0", 0U, 0U);
    current = q0_released;
    for (std::size_t index = ingress_.waypoints.size(); index > 0U; --index) {
      const auto next = ingress_state(park_, ingress_.waypoints[index - 1U]);
      audit_edge(
        current, next, 0.0, 0.0, false, false, suffix,
        "egress_open_from_q0", 0U, 0U);
      current = next;
    }
    audit_edge(
      current, park_, 0.0, 0.0, false, false, suffix,
      "egress_open_from_q0", 0U, 0U);
    audit_edge(
      park_, launch_, 0.0, 0.0, false, false, suffix,
      "return_left", 0U, 0U);
  }

  void run_direction(const bool reverse)
  {
    metric_.directions.push_back(DirectionMetric{reverse ? "reverse" : "forward"});
    auto & direction = metric_.directions.back();
    const std::size_t first_index = reverse ? asset_.rows.size() - 1U : 0U;
    const std::size_t last_index = reverse ? 0U : asset_.rows.size() - 1U;
    const auto & first_row = asset_.rows[first_index];
    const auto & last_row = asset_.rows[last_index];
    const double start_angle = first_row.handle_angle;
    moveit::core::RobotState current(launch_);
    SampleContext seam_context{direction.name, "seam_start", 0U, first_index, first_index};
    audit_sample(
      current, start_angle, false, false, direction, seam_context, nullptr, start_angle);

    audit_edge(
      current, park_, start_angle, start_angle, false, false, direction,
      "prepare_left_park", first_index, first_index);
    current = park_;
    moveit::core::RobotState lifted(park_);
    for (std::size_t index = 0U; index < lift_arm_.size(); ++index) {
      lifted.setVariablePosition(kRightArmVariables[index], lift_arm_[index]);
    }
    lifted.updateCollisionBodyTransforms();
    audit_edge(
      current, lifted, start_angle, start_angle, false, false, direction,
      "neutral_to_lift", first_index, first_index);
    current = lifted;
    moveit::core::RobotState stowed(park_);
    for (std::size_t index = 0U; index < stow_arm_.size(); ++index) {
      stowed.setVariablePosition(kRightArmVariables[index], stow_arm_[index]);
    }
    stowed.updateCollisionBodyTransforms();
    audit_edge(
      current, stowed, start_angle, start_angle, false, false, direction,
      "lift_to_stow", first_index, first_index);
    current = stowed;
    moveit::core::RobotState stowed_torso(stowed);
    stowed_torso.setVariablePosition("torso_joint", first_row.torso);
    stowed_torso.updateCollisionBodyTransforms();
    audit_edge(
      current, stowed_torso, start_angle, start_angle, false, false, direction,
      "rotate_torso_at_stow", first_index, first_index);
    current = stowed_torso;
    moveit::core::RobotState outer_pregrasp(park_);
    outer_pregrasp.setVariablePosition("torso_joint", first_row.torso);
    const auto & outer_arm = reverse ? q90_outer_arm_ : q0_outer_arm_;
    for (std::size_t index = 0U; index < outer_arm.size(); ++index) {
      outer_pregrasp.setVariablePosition(kRightArmVariables[index], outer_arm[index]);
    }
    outer_pregrasp.updateCollisionBodyTransforms();
    audit_edge(
      current, outer_pregrasp, start_angle, start_angle, false, false, direction,
      "neutral_to_outer_pregrasp", first_index, first_index);
    current = outer_pregrasp;
    const auto approach = row_state(park_, first_row, true);
    audit_edge(
      current, approach, start_angle, start_angle, false, false, direction,
      "approach_right_arm_open", first_index, first_index);
    const auto closed = row_state(park_, first_row, false);
    audit_edge(
      approach, closed, start_angle, start_angle, true, false, direction,
      "close_right_hand", first_index, first_index);
    SampleContext contact_start{
      direction.name, "contact_arc", direction.samples, first_index, first_index};
    audit_sample(
      closed, start_angle, true, true, direction, contact_start, nullptr, start_angle);
    current = closed;

    if (!reverse) {
      for (std::size_t index = 1U; index < asset_.rows.size(); ++index) {
        const auto next = row_state(park_, asset_.rows[index], false);
        audit_edge(
          current, next, asset_.rows[index - 1U].handle_angle,
          asset_.rows[index].handle_angle, true, true, direction,
          "contact_arc", index - 1U, index,
          &asset_.rows[index - 1U], &asset_.rows[index]);
        current = next;
      }
    } else {
      for (std::size_t index = asset_.rows.size() - 1U; index > 0U; --index) {
        const auto next = row_state(park_, asset_.rows[index - 1U], false);
        audit_edge(
          current, next, asset_.rows[index].handle_angle,
          asset_.rows[index - 1U].handle_angle, true, true, direction,
          "contact_arc", index, index - 1U,
          &asset_.rows[index], &asset_.rows[index - 1U]);
        current = next;
      }
    }

    const auto released = row_state(park_, last_row, true);
    audit_edge(
      current, released, last_row.handle_angle, last_row.handle_angle,
      true, false, direction, "release_right_hand", last_index, last_index);
    moveit::core::RobotState outer_retreat(park_);
    outer_retreat.setVariablePosition("torso_joint", last_row.torso);
    const auto & retreat_arm = reverse ? q0_outer_arm_ : q90_outer_arm_;
    for (std::size_t index = 0U; index < retreat_arm.size(); ++index) {
      outer_retreat.setVariablePosition(kRightArmVariables[index], retreat_arm[index]);
    }
    outer_retreat.updateCollisionBodyTransforms();
    audit_edge(
      released, outer_retreat, last_row.handle_angle, last_row.handle_angle,
      false, false, direction, "contact_to_outer_retreat", last_index, last_index);
    moveit::core::RobotState stowed_retreat(park_);
    for (std::size_t index = 0U; index < stow_arm_.size(); ++index) {
      stowed_retreat.setVariablePosition(kRightArmVariables[index], stow_arm_[index]);
    }
    stowed_retreat.updateCollisionBodyTransforms();
    moveit::core::RobotState stowed_retreat_torso(stowed_retreat);
    stowed_retreat_torso.setVariablePosition("torso_joint", last_row.torso);
    stowed_retreat_torso.updateCollisionBodyTransforms();
    audit_edge(
      outer_retreat, stowed_retreat_torso, last_row.handle_angle, last_row.handle_angle,
      false, false, direction, "outer_retreat_to_stow", last_index, last_index);
    audit_edge(
      stowed_retreat_torso, stowed_retreat, last_row.handle_angle, last_row.handle_angle,
      false, false, direction, "rotate_torso_from_stow", last_index, last_index);
    moveit::core::RobotState lifted_retreat(park_);
    for (std::size_t index = 0U; index < lift_arm_.size(); ++index) {
      lifted_retreat.setVariablePosition(kRightArmVariables[index], lift_arm_[index]);
    }
    lifted_retreat.updateCollisionBodyTransforms();
    audit_edge(
      stowed_retreat, lifted_retreat, last_row.handle_angle, last_row.handle_angle,
      false, false, direction, "stow_to_lift", last_index, last_index);
    audit_edge(
      lifted_retreat, park_, last_row.handle_angle, last_row.handle_angle,
      false, false, direction, "lift_to_neutral", last_index, last_index);
    audit_edge(
      park_, launch_, last_row.handle_angle, last_row.handle_angle,
      false, false, direction, "return_launch_seam", last_index, last_index);
  }

  planning_scene::PlanningScene & scene_;
  const PathAsset & asset_;
  const IngressAsset & ingress_;
  HandleGeometry handle_;
  moveit::core::RobotState launch_;
  moveit::core::RobotState park_;
  collision_detection::AllowedCollisionMatrix base_acm_;
  collision_detection::CollisionRequest collision_request_;
  collision_detection::CollisionRequest world_request_;
  std::vector<std::string> collision_links_;
  std::vector<std::string> left_links_;
  std::vector<std::string> nonpad_nonleft_links_;
  FclGeometryCache fcl_cache_;
  PathAuditMetric metric_;
  std::array<double, 7> lift_arm_{};
  std::array<double, 7> stow_arm_{};
  std::array<double, 7> q0_outer_arm_{};
  std::array<double, 7> q90_outer_arm_{};
};

std::string map_json(const std::map<std::string, std::size_t> & values)
{
  std::ostringstream output;
  output << '{';
  std::size_t index = 0U;
  for (const auto & value : values) {
    if (index++ != 0U) {
      output << ',';
    }
    output << '"' << json_escape(value.first) << "\":" << value.second;
  }
  output << '}';
  return output.str();
}

std::string vector7_json(const std::array<double, 7> & values)
{
  std::ostringstream output;
  output << std::setprecision(17) << '[';
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << values[index];
  }
  output << ']';
  return output.str();
}

std::string path_evidence_json(
  const PathAsset & asset, const Projection & projection,
  const PathAuditMetric & metric)
{
  std::ostringstream output;
  output << std::boolalpha << std::setprecision(17) <<
    "{\n"
    "  \"schema\": \"" << kPathAuditSchema << "\",\n"
    "  \"auditor_version\": \"" << kPathAuditorVersion << "\",\n"
    "  \"audit_method\": \"OFFLINE_FCL_UNPADDED_DYNAMIC_ORIGINAL_STL\",\n"
    "  \"path_asset_id\": \"" << json_escape(asset.asset_id) << "\",\n"
    "  \"path_sha256\": \"" << asset.sha256 << "\",\n"
    "  \"raw_scene_cdr_sha256\": \"" << projection.raw_cdr_sha256 << "\",\n"
    "  \"physical_scene_cdr_sha256\": \"" << projection.physical_cdr_sha256 << "\",\n"
    "  \"controller_commands_issued\": 0,\n"
    "  \"offline_only\": true,\n"
    "  \"execution_topology\": \"right_q0_q90_hold_middle_hold_q90_q0_release/v1\",\n"
    "  \"forward_passed\": true,\n"
    "  \"reverse_passed\": true,\n"
    "  \"prefix_passed\": true,\n"
    "  \"suffix_passed\": true,\n"
    "  \"q90_continuous_grasp_seam_passed\": true,\n"
    "  \"q90_release_commands\": 0,\n"
    "  \"q90_regrasp_commands\": 0,\n"
    "  \"variable_torso_path\": " << asset.variable_torso << ",\n"
    "  \"execution_splices_included\": true,\n"
    "  \"samples\": " << metric.samples << ",\n"
    "  \"contact_samples\": " << metric.contact_samples << ",\n"
    "  \"pad1_patch_samples\": " << metric.pad1_patch_samples << ",\n"
    "  \"pad2_patch_samples\": " << metric.pad2_patch_samples << ",\n"
    "  \"minimum_arm_joint_margin_rad\": " << metric.minimum_arm_margin << ",\n"
    "  \"maximum_abs_torso_rad\": " << metric.maximum_abs_torso << ",\n"
    "  \"torso_abs_contract_rad\": " << asset.torso_abs_limit << ",\n"
    "  \"maximum_dense_revolute_step_rad\": " << metric.maximum_dense_revolute_step << ",\n"
    "  \"maximum_dense_prismatic_step_m\": " << metric.maximum_dense_prismatic_step << ",\n"
    "  \"maximum_dense_handle_step_rad\": " << metric.maximum_dense_handle_step << ",\n"
    "  \"launch_left_shoulder_roll_rad\": " << metric.launch_left_roll << ",\n"
    "  \"park_left_shoulder_roll_rad\": " << metric.park_left_roll << ",\n"
    "  \"launch_left_arm_rad\": " << vector7_json(metric.launch_left_arm) << ",\n"
    "  \"park_left_arm_rad\": " << vector7_json(metric.park_left_arm) << ",\n";
  if (asset.variable_torso) {
    output <<
      "  \"q0_ingress_asset_sha256\": \"" << metric.q0_ingress_sha256 << "\",\n"
      "  \"q0_ingress_waypoint_count\": " << metric.q0_ingress_waypoints << ",\n"
      "  \"q0_ingress_forward_passed\": true,\n"
      "  \"q0_ingress_exact_reverse_egress_passed\": true,\n"
      "  \"execution_contract\": {\n"
      "    \"prefix_stages\": [\"prepare_left_park\",\"ingress_open_to_q0\","
      "\"close_q0\",\"contact_q0_to_q90_hold\"],\n"
      "    \"suffix_stages\": [\"q90_continuous_grasp_seam\","
      "\"contact_reverse_q90_to_q0\",\"release_q0\","
      "\"egress_open_from_q0\",\"return_left\"],\n"
      "    \"q90_release_forbidden\": true,\n"
      "    \"q90_regrasp_forbidden\": true,\n"
      "    \"q0_ingress_fingers_open_m\": " << kOpenFingerPosition << "\n"
      "  },\n";
  } else {
    output <<
      "  \"execution_contract\": {\n"
      "    \"right_lift_arm_rad\": " << vector7_json(metric.lift_arm) << ",\n"
      "    \"right_stow_arm_rad\": " << vector7_json(metric.stow_arm) << ",\n"
      "    \"q0_outer_pregrasp_arm_rad\": " << vector7_json(metric.q0_outer_arm) << ",\n"
      "    \"q90_outer_pregrasp_arm_rad\": " << vector7_json(metric.q90_outer_arm) << ",\n"
      "    \"prefix_terminal\": \"park_then_contact_arc\",\n"
      "    \"suffix_terminal\": \"launch_seam\"\n"
      "  },\n";
  }
  output <<
    "  \"minimum_nonpad_handle_clearance_m\": " <<
    metric.minimum_nonpad_handle_clearance << ",\n"
    "  \"minimum_left_handle_clearance_m\": " << metric.minimum_left_handle_clearance << ",\n"
    "  \"minimum_accepted_patch_span_m\": " << metric.minimum_patch_span << ",\n"
    "  \"minimum_accepted_patch_station_m\": " << metric.minimum_patch_station << ",\n"
    "  \"minimum_accepted_patch_radius_m\": " << metric.minimum_patch_radius << ",\n"
    "  \"dense_limits\": {\"angular_rad\":" << kDenseAngularStep <<
    ",\"prismatic_m\":" << kDensePrismaticStep << "},\n"
    "  \"authorized_handle_contact_pairs\": [[\"incubator_handle_exact\","
    "\"right_dex1_finger_1_pad_link\"],[\"incubator_handle_exact\","
    "\"right_dex1_finger_2_pad_link\"]],\n"
    "  \"left_handle_allowed_contact_pairs\": [],\n"
    "  \"directions\": [\n";
  for (std::size_t index = 0U; index < metric.directions.size(); ++index) {
    const auto & direction = metric.directions[index];
    output << "    {\"name\":\"" << direction.name << "\",\"samples\":" <<
      direction.samples << ",\"stage_samples\":" << map_json(direction.stage_samples) << "}" <<
      (index + 1U == metric.directions.size() ? "\n" : ",\n");
  }
  output <<
    "  ],\n"
    "  \"collision_free\": true,\n"
    "  \"self_collision_checked\": true,\n"
    "  \"full_world_and_table_guard_checked\": true,\n"
    "  \"original_handle_stl_sha256\": \"" << kExpectedHandleSha << "\",\n"
    "  \"valid\": true,\n"
    "  \"status\": \"PASS\"\n"
    "}\n";
  return output.str();
}

std::string path_failure_json(
  const std::string & status, const std::string & path_hash,
  const Projection * projection)
{
  std::ostringstream output;
  output << "{\n"
    "  \"schema\": \"" << kPathFailureSchema << "\",\n"
    "  \"auditor_version\": \"" << kPathAuditorVersion << "\",\n"
    "  \"status\": \"ERROR:" << json_escape(status) << "\",\n"
    "  \"valid\": false,\n"
    "  \"controller_commands_issued\": 0,\n"
    "  \"path_sha256\": \"" << json_escape(path_hash) << "\"";
  if (projection != nullptr) {
    output << ",\n  \"raw_scene_cdr_sha256\": \"" << projection->raw_cdr_sha256 << "\"";
  }
  output << "\n}\n";
  return output.str();
}
}  // namespace

#ifndef RIGHT_HANDLE_PATH_EXACT_AUDITOR_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  std::filesystem::path output_path;
  std::optional<Projection> projection;
  std::string path_hash;
  try {
    const auto args = path_arguments(argc, argv);
    output_path = args.output;
    const auto cdr = read_bytes(args.scene);
    const auto raw_scene = deserialize_scene(cdr);
    projection = project_physical_scene(raw_scene, cdr);
    if (!projection->unknown.empty() || !projection->handle_exact_retained) {
      throw std::runtime_error("SCENE_PROJECTION");
    }
    const auto asset = load_path_asset(args.path);
    path_hash = asset.sha256;
    IngressAsset ingress;
    if (asset.variable_torso) {
      ingress = load_ingress_asset(args.q0_ingress);
      if (ingress.scene_cdr_sha256 != projection->raw_cdr_sha256) {
        throw std::runtime_error("INGRESS_SCENE_SHA_MISMATCH");
      }
    }
    verify_handle_mesh(projection->physical, read_bytes(args.handle_stl));
    auto robot_model = load_robot_model(
      read_text_file(args.urdf), read_text_file(args.srdf));
    planning_scene::PlanningScene physical_scene(robot_model);
    if (!physical_scene.usePlanningSceneMsg(projection->physical)) {
      throw std::runtime_error("PLANNING_SCENE_LOAD");
    }
    verify_collision_geometry_fail_closed(physical_scene);
    ExactPathAuditor auditor(
      physical_scene, asset, ingress, args.park_left_roll, args.lift_arm, args.stow_arm,
      args.q0_outer_arm, args.q90_outer_arm);
    const auto metric = auditor.run();
    atomic_write(args.output, path_evidence_json(asset, *projection, metric));
    std::cout << "RIGHT_HANDLE_PATH_EXACT_AUDIT=PASS samples=" << metric.samples <<
      " contact_samples=" << metric.contact_samples <<
      " minimum_nonpad_handle_clearance_m=" << std::setprecision(17) <<
      metric.minimum_nonpad_handle_clearance <<
      " minimum_left_handle_clearance_m=" << metric.minimum_left_handle_clearance << std::endl;
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "RIGHT_HANDLE_PATH_EXACT_AUDIT=ERROR:" << error.what() << std::endl;
    if (!output_path.empty()) {
      try {
        atomic_write(output_path, path_failure_json(error.what(), path_hash,
          projection ? &*projection : nullptr));
      } catch (const std::exception &) {
      }
    }
    rclcpp::shutdown();
    return 3;
  }
}
#endif
