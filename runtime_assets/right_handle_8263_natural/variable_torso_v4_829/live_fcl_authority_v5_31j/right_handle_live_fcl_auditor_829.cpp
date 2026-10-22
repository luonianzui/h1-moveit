// File-only replay auditor for the 829 live right-handle telemetry transcript.
//
// This executable deliberately has no ROS graph, controller, or Gazebo surface.
// It replays already-recorded joint and mechanism telemetry against the exact
// MoveIt/FCL geometry.  Its contact result is geometric only: it never claims
// that a force or wrench was measured.
#define RIGHT_HANDLE_PATH_EXACT_AUDITOR_NO_MAIN
#include "right_handle_path_exact_auditor.cpp"

namespace
{
constexpr char kLiveTsvHeader[] =
  "id\tcontact_required\tdoor_angle_deg\thandle_angle_deg\ttorso_rad\t"
  "leg_rad\tleft_arm_rad\tright_arm_rad\tleft_gripper_m\tright_gripper_m";
constexpr char kLiveRowPrefix[] = "RIGHT_HANDLE_LIVE_FCL_ROW_829";
constexpr char kLiveSummaryPrefix[] = "RIGHT_HANDLE_LIVE_FCL_SUMMARY_829";
constexpr double kLiveTorsoLimit = 44.5 * kPi / 180.0;
constexpr double kLiveDoorLimit = 60.0 * kPi / 180.0;
constexpr double kLiveHandleLimit = 90.0 * kPi / 180.0;
// Gazebo's hard-stop solver can report a tiny, deterministic penetration at
// either door endpoint.  Admit only that measured physics envelope; keep the
// generic numeric tolerance strict for every other geometric contract.
constexpr double kLiveDoorPhysicsTolerance = 5e-5;
constexpr double kLiveNumericTolerance = 1e-12;
// JointState values come from Gazebo's iterative constraint solver.  Keep the
// runtime admission envelope six orders of magnitude below one radian while
// avoiding false failures at an otherwise legal hard-stop command.
constexpr double kLiveJointBoundsTolerance = 1e-6;
constexpr double kLiveLeftHandleClearance = 0.050;
// FCL penetration patches are not stable evidence for a rigid-body contact
// solver: a correctly loaded pad can settle a few micrometres outside the
// triangle surface.  Require at least one of the two explicitly authorized
// right-hand pads to remain inside this geometric engagement band, without
// claiming force or wrench measurement.  The user contract selects the right
// hand, not a particular pad within that hand.
constexpr double kLiveRightPadGripProximity = 0.0003;

const std::array<std::string, 2> kLiveAdjacentRightFingerBodies{
  "right_dex1_finger_1_link", "right_dex1_finger_2_link"};

const std::array<std::string, 2> kLiveLeftFingerVariables{
  "left_dex1_finger_1_joint", "left_dex1_finger_2_joint"};
const std::array<std::string, 12> kLiveLegVariables{
  "left_hip_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_yaw_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};

struct LiveArguments
{
  std::filesystem::path scene;
  std::filesystem::path path;
  std::filesystem::path telemetry_tsv;
  std::filesystem::path urdf;
  std::filesystem::path srdf;
  std::filesystem::path handle_stl;
};

struct LiveTelemetryRow
{
  std::string id;
  bool contact_required{false};
  double door_angle{0.0};
  double handle_angle{0.0};
  double torso{0.0};
  std::array<double, 12> legs{};
  std::array<double, 7> left_arm{};
  std::array<double, 7> right_arm{};
  std::array<double, 2> left_gripper{};
  std::array<double, 2> right_gripper{};
};

struct LiveDynamicGeometry
{
  shapes::ShapeConstPtr door_shape;
  shapes::ShapeConstPtr handle_shape;
  Eigen::Isometry3d initial_door_frame{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d initial_handle_pose{Eigen::Isometry3d::Identity()};
};

struct LiveRowMetric
{
  bool pad1_valid{false};
  bool pad2_valid{false};
  bool right_pad_grip_engaged{false};
  std::size_t pad1_contacts{0U};
  std::size_t pad2_contacts{0U};
  double pad1_distance{std::numeric_limits<double>::infinity()};
  double pad2_distance{std::numeric_limits<double>::infinity()};
  double nonpad_clearance{std::numeric_limits<double>::infinity()};
  double adjacent_finger_clearance{std::numeric_limits<double>::infinity()};
  double left_clearance{std::numeric_limits<double>::infinity()};
  double patch_span{0.0};
  double patch_station{0.0};
  double patch_radius{0.0};
};

struct LiveSummary
{
  std::size_t evaluated{0U};
  std::size_t passed{0U};
  std::size_t contact_required{0U};
  std::size_t contact_required_passed{0U};
  std::size_t noncontact{0U};
  std::size_t noncontact_passed{0U};
  std::size_t pad1_patch_valid{0U};
  std::size_t pad2_patch_valid{0U};
  std::size_t right_pad_grip_engaged{0U};
  double maximum_abs_torso{0.0};
  double maximum_contact_minimum_right_pad_distance{
    -std::numeric_limits<double>::infinity()};
  double minimum_nonpad_clearance{std::numeric_limits<double>::infinity()};
  double minimum_adjacent_finger_clearance{std::numeric_limits<double>::infinity()};
  double minimum_left_clearance{std::numeric_limits<double>::infinity()};
  double minimum_patch_span{std::numeric_limits<double>::infinity()};
  double minimum_patch_station{std::numeric_limits<double>::infinity()};
  double minimum_patch_radius{std::numeric_limits<double>::infinity()};
};

std::string live_token_escape(const std::string & value)
{
  std::ostringstream output;
  output << std::hex << std::uppercase << std::setfill('0');
  for (const unsigned char character : value) {
    const bool unreserved =
      (character >= 'a' && character <= 'z') ||
      (character >= 'A' && character <= 'Z') ||
      (character >= '0' && character <= '9') ||
      character == '_' || character == '-' || character == '.' || character == ':';
    if (unreserved) {
      output << character;
    } else {
      output << '%' << std::setw(2) << static_cast<unsigned int>(character);
    }
  }
  return output.str();
}

std::string live_number_token(const double value)
{
  std::ostringstream output;
  output << std::setprecision(17) << value;
  return output.str();
}

LiveArguments live_arguments(const int argc, char ** argv)
{
  const std::set<std::string> allowed{
    "--scene", "--path", "--telemetry-tsv", "--urdf", "--srdf", "--handle-stl"};
  std::map<std::string, std::string> values;
  if (argc != 13) {
    throw std::runtime_error("ARGUMENT_COUNT");
  }
  for (int index = 1; index < argc; index += 2) {
    const std::string name(argv[index]);
    if (allowed.count(name) != 1U) {
      throw std::runtime_error("ARGUMENT_UNKNOWN:" + name);
    }
    if (index + 1 >= argc || std::string(argv[index + 1]).empty()) {
      throw std::runtime_error("ARGUMENT_MISSING:" + name);
    }
    if (!values.emplace(name, argv[index + 1]).second) {
      throw std::runtime_error("ARGUMENT_DUPLICATE:" + name);
    }
  }
  for (const auto & name : allowed) {
    if (values.count(name) != 1U) {
      throw std::runtime_error("ARGUMENT_MISSING:" + name);
    }
  }
  return LiveArguments{
    values.at("--scene"), values.at("--path"), values.at("--telemetry-tsv"),
    values.at("--urdf"), values.at("--srdf"), values.at("--handle-stl")};
}

std::vector<std::uint8_t> live_read_regular_single_link(
  const std::filesystem::path & path)
{
  struct stat before {};
  struct stat after {};
  if (::lstat(path.c_str(), &before) != 0 || !S_ISREG(before.st_mode) ||
    S_ISLNK(before.st_mode) || before.st_nlink != 1)
  {
    throw std::runtime_error("INPUT_NOT_REGULAR_SINGLE_LINK:" + path.string());
  }
  const auto bytes = read_bytes(path);
  if (::lstat(path.c_str(), &after) != 0 ||
    before.st_dev != after.st_dev || before.st_ino != after.st_ino ||
    before.st_size != after.st_size ||
    before.st_mtim.tv_sec != after.st_mtim.tv_sec ||
    before.st_mtim.tv_nsec != after.st_mtim.tv_nsec)
  {
    throw std::runtime_error("INPUT_TOCTOU:" + path.string());
  }
  return bytes;
}

std::vector<std::string> live_split_exact(
  const std::string & text, const char delimiter)
{
  std::vector<std::string> fields;
  std::size_t begin = 0U;
  while (true) {
    const auto end = text.find(delimiter, begin);
    fields.push_back(text.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  return fields;
}

double live_finite_number(const std::string & text, const std::string & status)
{
  if (text.empty() ||
    std::isspace(static_cast<unsigned char>(text.front())) ||
    std::isspace(static_cast<unsigned char>(text.back())))
  {
    throw std::runtime_error(status);
  }
  std::size_t consumed = 0U;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error(status);
  }
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::runtime_error(status);
  }
  return value;
}

template<std::size_t Width>
std::array<double, Width> live_csv_vector(
  const std::string & text, const std::string & status)
{
  const auto fields = live_split_exact(text, ',');
  if (fields.size() != Width) {
    throw std::runtime_error(status + "_WIDTH");
  }
  std::array<double, Width> values{};
  for (std::size_t index = 0U; index < Width; ++index) {
    values[index] = live_finite_number(fields[index], status + "_FINITE");
  }
  return values;
}

std::vector<LiveTelemetryRow> load_live_telemetry(
  const std::vector<std::uint8_t> & bytes)
{
  const std::string text(bytes.begin(), bytes.end());
  if (text.find('\0') != std::string::npos || text.find('\r') != std::string::npos) {
    throw std::runtime_error("TSV_ENCODING");
  }
  std::istringstream stream(text);
  std::string line;
  if (!std::getline(stream, line) || line != kLiveTsvHeader) {
    throw std::runtime_error("TSV_HEADER");
  }
  std::set<std::string> ids;
  std::vector<LiveTelemetryRow> rows;
  std::size_t line_number = 1U;
  while (std::getline(stream, line)) {
    ++line_number;
    if (line.empty()) {
      throw std::runtime_error("TSV_EMPTY_ROW:" + std::to_string(line_number));
    }
    const auto fields = live_split_exact(line, '\t');
    if (fields.size() != 10U) {
      throw std::runtime_error("TSV_FIELD_COUNT:" + std::to_string(line_number));
    }
    LiveTelemetryRow row;
    row.id = fields[0];
    if (row.id.empty() || row.id.size() > 128U ||
      !std::all_of(row.id.begin(), row.id.end(), [](const unsigned char character) {
        return std::isalnum(character) || character == '_' || character == '-' ||
               character == '.' || character == ':';
      }) || !ids.insert(row.id).second)
    {
      throw std::runtime_error("TSV_ID:" + std::to_string(line_number));
    }
    if (fields[1] != "0" && fields[1] != "1") {
      throw std::runtime_error("TSV_CONTACT_REQUIRED:" + std::to_string(line_number));
    }
    row.contact_required = fields[1] == "1";
    row.door_angle = live_finite_number(
      fields[2], "TSV_DOOR_ANGLE:" + std::to_string(line_number)) * kPi / 180.0;
    row.handle_angle = live_finite_number(
      fields[3], "TSV_HANDLE_ANGLE:" + std::to_string(line_number)) * kPi / 180.0;
    row.torso = live_finite_number(
      fields[4], "TSV_TORSO:" + std::to_string(line_number));
    row.legs = live_csv_vector<12>(
      fields[5], "TSV_LEGS:" + std::to_string(line_number));
    row.left_arm = live_csv_vector<7>(
      fields[6], "TSV_LEFT_ARM:" + std::to_string(line_number));
    row.right_arm = live_csv_vector<7>(
      fields[7], "TSV_RIGHT_ARM:" + std::to_string(line_number));
    row.left_gripper = live_csv_vector<2>(
      fields[8], "TSV_LEFT_GRIPPER:" + std::to_string(line_number));
    row.right_gripper = live_csv_vector<2>(
      fields[9], "TSV_RIGHT_GRIPPER:" + std::to_string(line_number));
    if (row.door_angle < -kLiveDoorPhysicsTolerance ||
      row.door_angle > kLiveDoorLimit + kLiveDoorPhysicsTolerance)
    {
      throw std::runtime_error("TSV_DOOR_RANGE:" + std::to_string(line_number));
    }
    if (row.handle_angle < -kLiveNumericTolerance ||
      row.handle_angle > kLiveHandleLimit + kLiveNumericTolerance)
    {
      throw std::runtime_error("TSV_HANDLE_RANGE:" + std::to_string(line_number));
    }
    rows.push_back(std::move(row));
  }
  if (rows.empty()) {
    throw std::runtime_error("TSV_NO_ROWS");
  }
  return rows;
}

void verify_live_path_v4_contract(
  const std::vector<std::uint8_t> & bytes, const PathAsset & asset)
{
  const auto root = YAML::Load(std::string(bytes.begin(), bytes.end()));
  if (!root.IsMap() ||
    scalar_string(root["$schema"], "LIVE_PATH_SCHEMA") !=
    "h1_moveit/right_handle_variable_torso_path/v4" ||
    root["schema_version"].as<int>() != 4 || !asset.variable_torso ||
    std::abs(asset.torso_abs_limit - kLiveTorsoLimit) > kLiveNumericTolerance)
  {
    throw std::runtime_error("LIVE_PATH_V4_CONTRACT");
  }
  const auto contract = root["contract"];
  if (!contract.IsMap() ||
    std::abs(finite_scalar(contract["torso_abs_limit_deg"], "LIVE_PATH_TORSO") - 44.5) >
    kLiveNumericTolerance ||
    std::abs(finite_scalar(contract["handle_start_deg"], "LIVE_PATH_START")) >
    kLiveNumericTolerance ||
    std::abs(finite_scalar(contract["handle_end_deg"], "LIVE_PATH_END") - 90.0) >
    kLiveNumericTolerance ||
    !contract["all_nonpad_handle_contact_forbidden"].as<bool>() ||
    !contract["left_hand_handle_contact_forbidden"].as<bool>())
  {
    throw std::runtime_error("LIVE_PATH_V4_CONTRACT");
  }
}

Eigen::Isometry3d live_desired_door_pose(
  const LiveDynamicGeometry & geometry, const double door_angle)
{
  const auto door_visual = transform_rpy(
    kPi / 2.0, 0.0, -1.528168312093,
    Eigen::Vector3d(0.005592142864, 0.408609505443, 0.0));
  return geometry.initial_door_frame *
         transform_rpy(0.0, 0.0, door_angle, Eigen::Vector3d::Zero()) *
         door_visual;
}

Eigen::Isometry3d live_desired_handle_pose(
  const LiveDynamicGeometry & geometry, const double door_angle,
  const double handle_angle)
{
  const auto door_handle = transform_rpy(
    0.0, 0.0, 1.613424341497,
    Eigen::Vector3d(-0.019105784513, 0.377528763146, 0.189999998514));
  const auto handle_rotation = transform_rpy(
    0.0, handle_angle, 0.0, Eigen::Vector3d::Zero());
  const auto handle_visual = transform_rpy(
    0.0, 0.0, 0.0,
    Eigen::Vector3d(-0.009999668689, 0.000027284668, -0.119003896682));
  return geometry.initial_door_frame *
         transform_rpy(0.0, 0.0, door_angle, Eigen::Vector3d::Zero()) *
         door_handle * handle_rotation * handle_visual;
}

LiveDynamicGeometry prepare_live_dynamic_geometry(
  planning_scene::PlanningScene & scene)
{
  const auto door = scene.getWorld()->getObject("incubator_door_exact");
  const auto handle = scene.getWorld()->getObject(kHandleId);
  if (!door || door->shapes_.size() != 1U || door->global_shape_poses_.size() != 1U ||
    !handle || handle->shapes_.size() != 1U || handle->global_shape_poses_.size() != 1U)
  {
    throw std::runtime_error("LIVE_DYNAMIC_GEOMETRY");
  }
  LiveDynamicGeometry output;
  output.door_shape = door->shapes_.front();
  output.handle_shape = handle->shapes_.front();
  const auto door_visual = transform_rpy(
    kPi / 2.0, 0.0, -1.528168312093,
    Eigen::Vector3d(0.005592142864, 0.408609505443, 0.0));
  output.initial_door_frame = door->global_shape_poses_.front() * door_visual.inverse();
  output.initial_handle_pose = handle->global_shape_poses_.front();
  const auto expected_q0 = live_desired_handle_pose(output, 0.0, 0.0);
  if ((expected_q0.translation() - output.initial_handle_pose.translation()).norm() > 1e-6 ||
    rotation_distance(expected_q0, output.initial_handle_pose) > 1e-6)
  {
    throw std::runtime_error("LIVE_HANDLE_Q0_SCENE_POSE");
  }
  return output;
}

void exclude_live_dynamic_cube_domain(planning_scene::PlanningScene & scene)
{
  // The bag does not carry a collision-object pose stream for the supported
  // cube.  Replaying its q0 scene pose while the left hand carries it would be
  // physically false.  Cube contact and motion are therefore owned by the
  // separately bound candidate3 middle exact certificate.  This auditor keeps
  // the actual left-arm telemetry and still checks self collision, every other
  // world object, and the left-hand-to-handle distance on every retained row.
  const std::array<std::string, 2> cube_objects{
    "green_cube_dynamic_exact", "green_cube_10mm_guard"};
  for (const auto & id : cube_objects) {
    const auto object = scene.getWorld()->getObject(id);
    if (!object || object->shapes_.size() != 1U ||
      object->global_shape_poses_.size() != 1U)
    {
      throw std::runtime_error("LIVE_CUBE_DOMAIN_OBJECT:" + id);
    }
  }
  for (const auto & id : cube_objects) {
    scene.getWorldNonConst()->removeObject(id);
    if (scene.getWorld()->getObject(id)) {
      throw std::runtime_error("LIVE_CUBE_DOMAIN_REMOVE:" + id);
    }
  }
}

moveit::core::RobotState live_robot_state(
  const moveit::core::RobotState & base, const LiveTelemetryRow & row)
{
  moveit::core::RobotState state(base);
  state.setVariablePosition("torso_joint", row.torso);
  for (std::size_t index = 0U; index < row.legs.size(); ++index) {
    state.setVariablePosition(kLiveLegVariables[index], row.legs[index]);
  }
  for (std::size_t index = 0U; index < row.left_arm.size(); ++index) {
    state.setVariablePosition(kLeftArmVariables[index], row.left_arm[index]);
    state.setVariablePosition(kRightArmVariables[index], row.right_arm[index]);
  }
  for (std::size_t index = 0U; index < row.left_gripper.size(); ++index) {
    state.setVariablePosition(kLiveLeftFingerVariables[index], row.left_gripper[index]);
    state.setVariablePosition(kRightFingerVariables[index], row.right_gripper[index]);
  }
  state.updateCollisionBodyTransforms();
  return state;
}

bool live_valid_patch(
  const ContactPatch & patch, const Eigen::Isometry3d & handle_pose,
  double & radius)
{
  if (!valid_patch(patch, radius)) {
    radius = 0.0;
    return false;
  }
  const Eigen::Vector3d pivot = handle_pose * Eigen::Vector3d(
    0.009999668689, -0.000027284668, 0.119003896682);
  radius = (patch.center() - pivot).norm();
  return radius + kLiveNumericTolerance >= kPatchRadius;
}

class LiveFclAuditor
{
public:
  explicit LiveFclAuditor(planning_scene::PlanningScene & scene)
  : scene_(scene), geometry_(prepare_live_dynamic_geometry(scene)),
    base_state_(scene.getCurrentState()), base_acm_(scene.getAllowedCollisionMatrix())
  {
    collision_links_ = scene_.getRobotModel()->getLinkModelNamesWithCollisionGeometry();
    for (const auto & link : collision_links_) {
      const bool right_pad = link == kRightHandlePads[0] || link == kRightHandlePads[1];
      const bool adjacent_right_finger = std::find(
        kLiveAdjacentRightFingerBodies.begin(),
        kLiveAdjacentRightFingerBodies.end(), link) !=
        kLiveAdjacentRightFingerBodies.end();
      if (link.rfind("left_", 0U) == 0U) {
        left_links_.push_back(link);
      } else if (adjacent_right_finger) {
        adjacent_finger_links_.push_back(link);
      } else if (!right_pad) {
        nonpad_nonleft_links_.push_back(link);
      }
    }
    if (left_links_.empty() || adjacent_finger_links_.size() !=
      kLiveAdjacentRightFingerBodies.size() || nonpad_nonleft_links_.empty())
    {
      throw std::runtime_error("LIVE_HANDLE_DISTANCE_LINK_SETS");
    }
    for (const auto & object : scene_.getWorld()->getObjectIds()) {
      for (const auto & link : collision_links_) {
        base_acm_.setEntry(object, link, false);
      }
    }
    base_acm_.setEntry("right_elbow_link", "torso_link", false);
    collision_request_.contacts = true;
    collision_request_.max_contacts = 64U;
    collision_request_.max_contacts_per_pair = 8U;
    collision_request_.pad_self_collisions = false;
    world_request_ = collision_request_;
    world_request_.pad_environment_collisions = false;
  }

  LiveRowMetric audit(const LiveTelemetryRow & row)
  {
    if (std::abs(row.torso) > kLiveTorsoLimit + kLiveNumericTolerance) {
      throw std::runtime_error("TORSO_44_5_DEGREE_LIMIT");
    }
    auto state = live_robot_state(base_state_, row);
    if (!state.satisfiesBounds(kLiveJointBoundsTolerance)) {
      throw std::runtime_error("ROBOT_BOUNDS");
    }

    const auto door_pose = live_desired_door_pose(geometry_, row.door_angle);
    const auto handle_pose = live_desired_handle_pose(
      geometry_, row.door_angle, row.handle_angle);
    set_single_shape_pose(scene_, "incubator_door_exact", geometry_.door_shape, door_pose);
    set_single_shape_pose(scene_, kHandleId, geometry_.handle_shape, handle_pose);

    auto collision_acm = base_acm_;
    if (row.contact_required) {
      for (const auto & pad : kRightHandlePads) {
        collision_acm.setEntry(kHandleId, pad, true);
      }
    }
    collision_detection::CollisionResult self_result;
    scene_.checkSelfCollision(collision_request_, self_result, state, collision_acm);
    if (self_result.collision) {
      throw std::runtime_error("COLLISION_SELF:" + collision_pair(self_result));
    }
    collision_detection::CollisionResult world_result;
    scene_.getCollisionEnvUnpadded()->checkRobotCollision(
      world_request_, world_result, state, collision_acm);
    if (world_result.collision) {
      throw std::runtime_error("COLLISION_WORLD:" + collision_pair(world_result));
    }

    LiveRowMetric metric;
    HandleGeometry patch_geometry;
    patch_geometry.shape = geometry_.handle_shape;
    const auto pad1 = pad_patch(
      fcl_cache_, state, kRightHandlePads[0], patch_geometry, handle_pose);
    const auto pad2 = pad_patch(
      fcl_cache_, state, kRightHandlePads[1], patch_geometry, handle_pose);
    metric.pad1_contacts = pad1.contacts;
    metric.pad2_contacts = pad2.contacts;
    double radius1 = 0.0;
    double radius2 = 0.0;
    metric.pad1_valid = live_valid_patch(pad1, handle_pose, radius1);
    metric.pad2_valid = live_valid_patch(pad2, handle_pose, radius2);
    const std::vector<std::string> pad1_link{kRightHandlePads[0]};
    const std::vector<std::string> pad2_link{kRightHandlePads[1]};
    const auto pad1_distance = handle_distance(
      scene_, state, base_acm_, pad1_link, 0.020);
    const auto pad2_distance = handle_distance(
      scene_, state, base_acm_, pad2_link, 0.020);
    metric.pad1_distance = pad1_distance.first;
    metric.pad2_distance = pad2_distance.first;
    const double minimum_right_pad_distance =
      std::min(metric.pad1_distance, metric.pad2_distance);
    metric.right_pad_grip_engaged = minimum_right_pad_distance <=
      kLiveRightPadGripProximity + kLiveNumericTolerance;

    const auto nonpad = handle_distance(
      scene_, state, base_acm_, nonpad_nonleft_links_, 0.020);
    metric.nonpad_clearance = nonpad.first;
    if (metric.nonpad_clearance + kLiveNumericTolerance < kNonpadHandleClearance) {
      throw std::runtime_error(
              "NONPAD_HANDLE_CLEARANCE:" + nonpad.second +
              ":distance_m=" + live_number_token(metric.nonpad_clearance));
    }
    const auto adjacent_finger = handle_distance(
      scene_, state, base_acm_, adjacent_finger_links_, 0.020);
    metric.adjacent_finger_clearance = adjacent_finger.first;
    if (metric.adjacent_finger_clearance < -kLiveNumericTolerance) {
      throw std::runtime_error(
              "RIGHT_FINGER_BODY_HANDLE_PENETRATION:" +
              adjacent_finger.second + ":distance_m=" +
              live_number_token(metric.adjacent_finger_clearance));
    }
    const auto left = handle_distance(
      scene_, state, base_acm_, left_links_, kLiveLeftHandleClearance);
    metric.left_clearance = left.first;
    if (metric.left_clearance + kLiveNumericTolerance < kLiveLeftHandleClearance) {
      throw std::runtime_error("LEFT_HANDLE_50MM_CLEARANCE:" + left.second);
    }

    if (row.contact_required) {
      if (!metric.right_pad_grip_engaged) {
        throw std::runtime_error(
                "RIGHT_PAD_GRIP_PROXIMITY_REQUIRED:pad1_distance_m=" +
                live_number_token(metric.pad1_distance) +
                ":pad2_distance_m=" + live_number_token(metric.pad2_distance) +
                ":pad1_contacts=" + std::to_string(metric.pad1_contacts) +
                ":pad2_contacts=" + std::to_string(metric.pad2_contacts) +
                ":pad1_span_m=" + live_number_token(pad1.span()) +
                ":pad2_span_m=" + live_number_token(pad2.span()) +
                ":pad1_station_m=" + live_number_token(pad1.station()) +
                ":pad2_station_m=" + live_number_token(pad2.station()) +
                ":pad1_radius_m=" + live_number_token(radius1) +
                ":pad2_radius_m=" + live_number_token(radius2));
      }
      const bool pad1_is_closer = metric.pad1_distance <= metric.pad2_distance;
      metric.patch_span = pad1_is_closer ? pad1.span() : pad2.span();
      metric.patch_station = pad1_is_closer ? pad1.station() : pad2.station();
      metric.patch_radius = pad1_is_closer ? radius1 : radius2;
    } else if (metric.pad1_distance <= kLiveRightPadGripProximity +
      kLiveNumericTolerance || metric.pad2_distance <= kLiveRightPadGripProximity +
      kLiveNumericTolerance)
    {
      throw std::runtime_error("RIGHT_PAD_GRIP_PROXIMITY_FORBIDDEN");
    }
    return metric;
  }

private:
  planning_scene::PlanningScene & scene_;
  LiveDynamicGeometry geometry_;
  moveit::core::RobotState base_state_;
  collision_detection::AllowedCollisionMatrix base_acm_;
  collision_detection::CollisionRequest collision_request_;
  collision_detection::CollisionRequest world_request_;
  std::vector<std::string> collision_links_;
  std::vector<std::string> left_links_;
  std::vector<std::string> adjacent_finger_links_;
  std::vector<std::string> nonpad_nonleft_links_;
  FclGeometryCache fcl_cache_;
};

void print_live_row_pass(const LiveTelemetryRow & row, const LiveRowMetric & metric)
{
  std::cout << std::boolalpha << std::setprecision(17) << kLiveRowPrefix <<
    " id=" << row.id <<
    " status=PASS valid=true contact_required=" << row.contact_required <<
    " door_angle_deg=" << row.door_angle * 180.0 / kPi <<
    " handle_angle_deg=" << row.handle_angle * 180.0 / kPi <<
    " torso_rad=" << row.torso <<
    " collision_free=true pad1_patch_valid=" << metric.pad1_valid <<
    " pad2_patch_valid=" << metric.pad2_valid <<
    " right_pad_grip_engaged=" << metric.right_pad_grip_engaged <<
    " pad1_contacts=" << metric.pad1_contacts <<
    " pad2_contacts=" << metric.pad2_contacts <<
    " pad1_handle_distance_m=" << metric.pad1_distance <<
    " pad2_handle_distance_m=" << metric.pad2_distance <<
    " nonpad_handle_clearance_m=" << metric.nonpad_clearance <<
    " adjacent_finger_handle_clearance_m=" << metric.adjacent_finger_clearance <<
    " left_handle_clearance_m=" << metric.left_clearance <<
    " patch_span_m=" << metric.patch_span <<
    " patch_station_m=" << metric.patch_station <<
    " patch_radius_m=" << metric.patch_radius <<
    " geometric_only=true force_or_wrench_claimed=false" << std::endl;
}

void print_live_row_error(const LiveTelemetryRow & row, const std::string & reason)
{
  std::cout << std::boolalpha << std::setprecision(17) << kLiveRowPrefix <<
    " id=" << row.id <<
    " status=ERROR valid=false contact_required=" << row.contact_required <<
    " door_angle_deg=" << row.door_angle * 180.0 / kPi <<
    " handle_angle_deg=" << row.handle_angle * 180.0 / kPi <<
    " torso_rad=" << row.torso <<
    " reason=" << live_token_escape(reason) <<
    " geometric_only=true force_or_wrench_claimed=false" << std::endl;
}

void print_live_summary(
  const LiveSummary & summary, const bool valid, const std::string & reason,
  const std::string & scene_sha, const std::string & path_sha,
  const std::string & telemetry_sha, const std::string & urdf_sha,
  const std::string & srdf_sha, const std::string & handle_sha)
{
  const auto finite_or_zero = [](const double value) {
      return std::isfinite(value) ? value : 0.0;
    };
  std::cout << std::boolalpha << std::setprecision(17) << kLiveSummaryPrefix <<
    " status=" << (valid ? "PASS" : "ERROR") <<
    " valid=" << valid <<
    " evaluated=" << summary.evaluated <<
    " passed=" << summary.passed <<
    " contact_required=" << summary.contact_required <<
    " contact_required_passed=" << summary.contact_required_passed <<
    " noncontact=" << summary.noncontact <<
    " noncontact_passed=" << summary.noncontact_passed <<
    " pad1_patch_valid=" << summary.pad1_patch_valid <<
    " pad2_patch_valid=" << summary.pad2_patch_valid <<
    " right_pad_grip_engaged=" << summary.right_pad_grip_engaged <<
    " max_torso_rad=" << summary.maximum_abs_torso <<
    " max_contact_minimum_right_pad_distance_m=" <<
    summary.maximum_contact_minimum_right_pad_distance <<
    " min_nonpad_handle_clearance_m=" <<
    finite_or_zero(summary.minimum_nonpad_clearance) <<
    " min_adjacent_finger_handle_clearance_m=" <<
    finite_or_zero(summary.minimum_adjacent_finger_clearance) <<
    " min_left_handle_clearance_m=" << finite_or_zero(summary.minimum_left_clearance) <<
    " min_patch_span_m=" << finite_or_zero(summary.minimum_patch_span) <<
    " min_patch_station_m=" << finite_or_zero(summary.minimum_patch_station) <<
    " min_patch_radius_m=" << finite_or_zero(summary.minimum_patch_radius) <<
    " collision_free=" << valid <<
    " scene_sha256=" << scene_sha <<
    " path_sha256=" << path_sha <<
    " telemetry_sha256=" << telemetry_sha <<
    " urdf_sha256=" << urdf_sha <<
    " srdf_sha256=" << srdf_sha <<
    " handle_stl_sha256=" << handle_sha <<
    " reason=" << live_token_escape(reason.empty() ? "NONE" : reason) <<
    " geometric_only=true force_or_wrench_claimed=false" << std::endl;
}
}  // namespace

int main(int argc, char ** argv)
{
  LiveSummary summary;
  std::string scene_sha;
  std::string path_sha;
  std::string telemetry_sha;
  std::string urdf_sha;
  std::string srdf_sha;
  std::string handle_sha;
  try {
    const auto args = live_arguments(argc, argv);
    const auto scene_bytes = live_read_regular_single_link(args.scene);
    const auto path_bytes = live_read_regular_single_link(args.path);
    const auto telemetry_bytes = live_read_regular_single_link(args.telemetry_tsv);
    const auto urdf_bytes = live_read_regular_single_link(args.urdf);
    const auto srdf_bytes = live_read_regular_single_link(args.srdf);
    const auto handle_bytes = live_read_regular_single_link(args.handle_stl);
    scene_sha = sha256_hex(scene_bytes);
    path_sha = sha256_hex(path_bytes);
    telemetry_sha = sha256_hex(telemetry_bytes);
    urdf_sha = sha256_hex(urdf_bytes);
    srdf_sha = sha256_hex(srdf_bytes);
    handle_sha = sha256_hex(handle_bytes);

    const auto raw_scene = deserialize_scene(scene_bytes);
    const auto projection = project_physical_scene(raw_scene, scene_bytes);
    if (!projection.unknown.empty() || !projection.handle_exact_retained) {
      throw std::runtime_error("SCENE_PROJECTION");
    }
    const auto path_asset = load_path_asset(args.path);
    if (path_asset.sha256 != path_sha) {
      throw std::runtime_error("PATH_TOCTOU");
    }
    verify_live_path_v4_contract(path_bytes, path_asset);
    verify_handle_mesh(projection.physical, handle_bytes);
    const std::string urdf_xml(urdf_bytes.begin(), urdf_bytes.end());
    const std::string srdf_xml(srdf_bytes.begin(), srdf_bytes.end());
    auto robot_model = load_robot_model(urdf_xml, srdf_xml);
    planning_scene::PlanningScene physical_scene(robot_model);
    if (!physical_scene.usePlanningSceneMsg(projection.physical)) {
      throw std::runtime_error("PLANNING_SCENE_LOAD");
    }
    verify_collision_geometry_fail_closed(physical_scene);
    exclude_live_dynamic_cube_domain(physical_scene);
    const auto rows = load_live_telemetry(telemetry_bytes);
    LiveFclAuditor auditor(physical_scene);

    for (const auto & row : rows) {
      ++summary.evaluated;
      summary.maximum_abs_torso = std::max(summary.maximum_abs_torso, std::abs(row.torso));
      if (row.contact_required) {
        ++summary.contact_required;
      } else {
        ++summary.noncontact;
      }
      try {
        const auto metric = auditor.audit(row);
        ++summary.passed;
        summary.contact_required_passed += row.contact_required ? 1U : 0U;
        summary.noncontact_passed += row.contact_required ? 0U : 1U;
        summary.pad1_patch_valid += metric.pad1_valid ? 1U : 0U;
        summary.pad2_patch_valid += metric.pad2_valid ? 1U : 0U;
        summary.right_pad_grip_engaged +=
          metric.right_pad_grip_engaged ? 1U : 0U;
        if (row.contact_required) {
          summary.maximum_contact_minimum_right_pad_distance = std::max(
            summary.maximum_contact_minimum_right_pad_distance,
            std::min(metric.pad1_distance, metric.pad2_distance));
        }
        summary.minimum_nonpad_clearance = std::min(
          summary.minimum_nonpad_clearance, metric.nonpad_clearance);
        summary.minimum_adjacent_finger_clearance = std::min(
          summary.minimum_adjacent_finger_clearance,
          metric.adjacent_finger_clearance);
        summary.minimum_left_clearance = std::min(
          summary.minimum_left_clearance, metric.left_clearance);
        if (row.contact_required) {
          summary.minimum_patch_span = std::min(
            summary.minimum_patch_span, metric.patch_span);
          summary.minimum_patch_station = std::min(
            summary.minimum_patch_station, metric.patch_station);
          summary.minimum_patch_radius = std::min(
            summary.minimum_patch_radius, metric.patch_radius);
        }
        print_live_row_pass(row, metric);
      } catch (const std::exception & error) {
        print_live_row_error(row, error.what());
      }
    }

    std::string final_reason;
    const bool complete_categories =
      summary.contact_required != 0U && summary.noncontact != 0U;
    if (!complete_categories) {
      final_reason = "CONTACT_AND_NONCONTACT_ROWS_REQUIRED";
    } else if (summary.passed != summary.evaluated ||
      summary.contact_required_passed != summary.contact_required ||
      summary.noncontact_passed != summary.noncontact)
    {
      final_reason = "ROW_FAILURE";
    }
    const bool valid = final_reason.empty();
    print_live_summary(
      summary, valid, final_reason, scene_sha, path_sha, telemetry_sha,
      urdf_sha, srdf_sha, handle_sha);
    return valid ? 0 : 3;
  } catch (const std::exception & error) {
    print_live_summary(
      summary, false, error.what(), scene_sha, path_sha, telemetry_sha,
      urdf_sha, srdf_sha, handle_sha);
    return 3;
  }
}
