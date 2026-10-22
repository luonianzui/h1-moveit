#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <fcl/fcl.h>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/attached_body.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcutils/sha256.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>

namespace
{
constexpr char kProjectionSchema[] = "h1_candidate3_middle_scene_projection/v1";
constexpr char kFailureSchema[] = "h1_candidate3_middle_exact_audit_failure/v1";
constexpr char kAuditorVersion[] =
  "h1_candidate3_middle_e0_exact_auditor/828-e0-raw-v1";
constexpr char kHandleId[] = "incubator_handle_exact";
constexpr char kHandleMaskId[] = "incubator_handle_visual_mask";
constexpr char kLeftFingerMaskId[] = "left_dex1_finger_2_visual_mask";
constexpr char kExpectedHandleSha[] =
  "ad6670185fa5756ba5dabdee7e542db2df3346973fea4eb887288f54a6b6f59e";
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kHandleAngle = kPi / 2.0;
constexpr double kDoorOpenAngle = kPi / 3.0;
constexpr double kAngularStep = 0.1 * kPi / 180.0;
constexpr double kPrismaticStep = 0.0001;
constexpr double kRequiredClearance = 0.010;
constexpr double kParkStaticClearance = 0.0105;
constexpr double kRightHandleNonpadClearance = 0.0006;
constexpr double kLeftHandleClearance = 0.05;
constexpr double kDistanceBand = kRequiredClearance + 1e-6;

const std::array<std::string, 2> kRightHandlePads{
  "right_dex1_finger_1_pad_link", "right_dex1_finger_2_pad_link"};
const std::array<std::string, 7> kRightArmJoints{
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint",
  "right_wrist_roll_joint", "right_wrist_pitch_joint",
  "right_wrist_yaw_joint"};
const std::array<std::string, 3> kPhysicalLeftFinger2Links{
  "left_dex1_finger_2_slider_link", "left_dex1_finger_2_link",
  "left_dex1_finger_2_pad_link"};
const std::array<std::string, 6> kLeftCubeTouchLinks{
  "left_dex1_finger_1_slider_link", "left_dex1_finger_1_link",
  "left_dex1_finger_1_pad_link", "left_dex1_finger_2_slider_link",
  "left_dex1_finger_2_link", "left_dex1_finger_2_pad_link"};
const std::array<std::string, 6> kLeftCubeReleaseRetreatLinks{
  "left_dex1_finger_1_slider_link", "left_dex1_finger_1_link",
  "left_dex1_finger_1_pad_link", "left_dex1_finger_2_slider_link",
  "left_dex1_finger_2_link", "left_dex1_finger_2_pad_link"};

const std::set<std::string> kPerceptionOnlyMaskIds{
  "incubator_handle_visual_mask",
  "incubator_handle_visual_mask_attached",
  "left_wrist_yaw_visual_mask",
  "right_wrist_yaw_visual_mask",
  "left_ankle_pitch_visual_mask",
  "right_ankle_pitch_visual_mask",
  "left_ankle_roll_visual_mask",
  "right_ankle_roll_visual_mask",
  "left_knee_visual_mask",
  "right_knee_visual_mask",
  "left_dex1_finger_2_slider_visual_mask",
  "left_dex1_finger_2_visual_mask",
  "left_dex1_finger_1_visual_mask",
};

struct Arguments
{
  bool inspect_projection{false};
  bool static_only{false};
  bool tail_scan{false};
  bool cube_only{false};
  bool diagnose_cube_door{false};
  double park_roll{0.0};
  std::filesystem::path scene;
  std::filesystem::path output;
  std::filesystem::path plan;
  std::filesystem::path urdf;
  std::filesystem::path srdf;
  std::filesystem::path door_open;
  std::filesystem::path seam_to_cube;
  std::filesystem::path cube;
  std::filesystem::path cube_reference;
  std::filesystem::path seam_to_hinge;
  std::filesystem::path door_close;
  std::filesystem::path handle_stl;
  std::filesystem::path robot_model_inventory;
};

struct SceneBinding
{
  std::uint64_t epoch{0U};
  std::string raw_scene_hash;
};

struct Projection
{
  moveit_msgs::msg::PlanningScene physical;
  std::vector<std::string> removed;
  std::vector<std::string> unknown;
  std::string raw_cdr_sha256;
  std::string physical_cdr_sha256;
  SceneBinding binding;
  bool handle_exact_retained{false};
  bool mask_conflict_pair_present{false};
};

enum class AssetKind
{
  DoorPre,
  DoorOpen,
  Door60SeamToCube,
  LeftCubeReady,
  Cube,
  LeftCubePark,
  Door60SeamToHinge,
  DoorClose,
  CubeReference,
};

struct PhaseSpec
{
  const char * name;
  AssetKind asset;
  std::size_t start;
  std::size_t end;
  bool door_supported;
};

const std::array<PhaseSpec, 11> kPhases{{
  {"door_pre_support", AssetKind::DoorPre, 0U, 1U, false},
  {"door_open_supported", AssetKind::DoorOpen, 0U, 1U, true},
  {"door60_seam_to_cube", AssetKind::Door60SeamToCube, 0U, 1U, true},
  {"left_cube_ready", AssetKind::LeftCubeReady, 0U, 1U, true},
  {"cube_contact", AssetKind::Cube, 0U, 2U, true},
  {"cube_carry_place", AssetKind::Cube, 2U, 4U, true},
  {"cube_release", AssetKind::Cube, 4U, 5U, true},
  {"cube_retreat", AssetKind::Cube, 5U, 6U, true},
  {"left_cube_park", AssetKind::LeftCubePark, 0U, 1U, true},
  {"door60_seam_to_hinge", AssetKind::Door60SeamToHinge, 0U, 1U, true},
  {"door_close_supported", AssetKind::DoorClose, 0U, 1U, true},
}};

struct AssetMessage
{
  std::size_t sequence{0U};
  std::vector<std::string> joint_names;
  std::vector<std::vector<double>> positions;
  std::vector<std::int64_t> time_from_start_ns;
};

struct AssetData
{
  std::vector<AssetMessage> messages;
};

struct PhaseMetric
{
  std::string name;
  std::size_t samples{0U};
  std::size_t subdivisions{0U};
  std::size_t right_handle_authorized_samples{0U};
  double minimum_environment_clearance{kDistanceBand};
  double minimum_table_clearance{kDistanceBand};
  double minimum_door_clearance{kDistanceBand};
  double minimum_handle_clearance{kDistanceBand};
  double minimum_cube_clearance{kDistanceBand};
  double minimum_self_clearance{kDistanceBand};
  double minimum_right_handle_nonpad_clearance{
    std::numeric_limits<double>::infinity()};
  double minimum_left_handle_clearance{
    std::numeric_limits<double>::infinity()};
  double minimum_torso{std::numeric_limits<double>::infinity()};
  double maximum_torso{-std::numeric_limits<double>::infinity()};
};

struct NeighborhoodSample
{
  int offset{0};
  std::string link;
  double distance{std::numeric_limits<double>::infinity()};
};

struct ExactAuditResult
{
  std::size_t samples{0U};
  std::size_t subdivisions{0U};
  double minimum_environment_clearance{kDistanceBand};
  double minimum_table_clearance{kDistanceBand};
  double minimum_door_clearance{kDistanceBand};
  double minimum_handle_clearance{kDistanceBand};
  double minimum_cube_clearance{kDistanceBand};
  double minimum_self_clearance{kDistanceBand};
  double minimum_right_handle_nonpad_clearance{
    std::numeric_limits<double>::infinity()};
  std::size_t cube_guard_exact_refinement_samples{0U};
  double minimum_cube_exact_noncontact_clearance{
    std::numeric_limits<double>::infinity()};
  NeighborhoodSample initial_park_door_clearance;
  NeighborhoodSample final_park_door_clearance;
  double minimum_left_handle_clearance{std::numeric_limits<double>::infinity()};
  std::size_t left_handle_contact_count{0U};
  double maximum_abs_torso{0.0};
  double minimum_torso{std::numeric_limits<double>::infinity()};
  double maximum_torso{-std::numeric_limits<double>::infinity()};
  double initial_left_shoulder_roll{0.0};
  std::optional<double> observed_visual_distance;
  std::size_t cube_release_retreat_window_samples{0U};
  std::size_t cube_release_retreat_gate_point{0U};
  std::size_t cube_release_retreat_gate_subdivision{0U};
  std::size_t cube_release_retreat_gate_ordinal{0U};
  std::size_t cube_release_retreat_last_below_gate_ordinal{0U};
  bool cube_release_retreat_gate_reached{false};
  double cube_release_retreat_minimum_exact_distance{
    std::numeric_limits<double>::infinity()};
  double cube_release_retreat_maximum_exact_distance{
    -std::numeric_limits<double>::infinity()};
  double cube_release_retreat_final_exact_distance{
    std::numeric_limits<double>::quiet_NaN()};
  double cube_release_retreat_maximum_exact_penetration{0.0};
  double cube_release_retreat_minimum_projection_step{
    std::numeric_limits<double>::infinity()};
  double cube_release_retreat_final_projection{0.0};
  double cube_release_retreat_post_gate_minimum_exact_distance{
    std::numeric_limits<double>::infinity()};
  double cube_release_retreat_post_gate_minimum_guard_physical_lower_bound{
    std::numeric_limits<double>::infinity()};
  std::set<std::string> cube_release_retreat_observed_exact_links;
  std::set<std::string> cube_release_retreat_observed_guard_links;
  std::set<std::pair<std::string, std::string>>
    cube_release_retreat_observed_exact_pairs;
  std::set<std::pair<std::string, std::string>>
    cube_release_retreat_observed_guard_pairs;
  std::size_t right_handle_window_samples{0U};
  double right_handle_maximum_observed_penetration{0.0};
  std::set<std::pair<std::string, std::string>> right_handle_observed_pairs;
  std::size_t cube_grasp_carry_window_samples{0U};
  double cube_grasp_carry_maximum_observed_penetration{0.0};
  std::set<std::pair<std::string, std::string>>
    cube_grasp_carry_observed_exact_pairs;
  std::set<std::pair<std::string, std::string>>
    cube_grasp_carry_observed_guard_pairs;
  double cube_grasp_carry_minimum_guard_physical_lower_bound{
    std::numeric_limits<double>::infinity()};
  double cube_bump_maximum_tcp_translation{0.0};
  double cube_bump_maximum_tcp_rotation{0.0};
  double final_target_translation_error{
    std::numeric_limits<double>::infinity()};
  double final_target_rotation_error{
    std::numeric_limits<double>::infinity()};
  std::int64_t cube_release_retreat_minimum_time_step_ns{
    std::numeric_limits<std::int64_t>::max()};
  double cube_release_retreat_minimum_path_parameter_step{
    std::numeric_limits<double>::infinity()};
  std::array<NeighborhoodSample, 3> neighborhood{{
    {-1, "", std::numeric_limits<double>::infinity()},
    {0, "", std::numeric_limits<double>::infinity()},
    {1, "", std::numeric_limits<double>::infinity()},
  }};
  std::vector<PhaseMetric> phases;
};

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<unsigned int>(character) << std::dec;
        } else {
          output << character;
        }
    }
  }
  return output.str();
}

std::string json_strings(const std::vector<std::string> & values)
{
  std::ostringstream output;
  output << '[';
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << '"' << json_escape(values[index]) << '"';
  }
  output << ']';
  return output.str();
}

std::string json_pairs(
  const std::set<std::pair<std::string, std::string>> & values)
{
  std::ostringstream output;
  output << '[';
  std::size_t index = 0U;
  for (const auto & [first, second] : values) {
    if (index++ != 0U) {
      output << ',';
    }
    output << "[\"" << json_escape(first) << "\",\"" <<
      json_escape(second) << "\"]";
  }
  output << ']';
  return output.str();
}

std::string sha256_hex(const std::vector<std::uint8_t> & bytes);

std::vector<std::uint8_t> read_bytes(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) {
    throw std::runtime_error("INPUT_OPEN:" + path.string());
  }
  const auto size = stream.tellg();
  if (size <= 0) {
    throw std::runtime_error("INPUT_EMPTY:" + path.string());
  }
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
  stream.seekg(0);
  stream.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (!stream) {
    throw std::runtime_error("INPUT_SHORT_READ:" + path.string());
  }
  return bytes;
}

std::vector<std::uint8_t> read_sealed_bytes(
  const std::filesystem::path & path, const std::string & expected_sha256,
  const mode_t expected_mode)
{
  struct stat before {};
  struct stat after {};
  if (::lstat(path.c_str(), &before) != 0 || !S_ISREG(before.st_mode) ||
    S_ISLNK(before.st_mode) || before.st_nlink != 1 ||
    (before.st_mode & 0777) != expected_mode)
  {
    throw std::runtime_error("SEALED_FILE_METADATA:" + path.string());
  }
  const auto bytes = read_bytes(path);
  if (::lstat(path.c_str(), &after) != 0 || before.st_dev != after.st_dev ||
    before.st_ino != after.st_ino || before.st_size != after.st_size ||
    before.st_mtim.tv_sec != after.st_mtim.tv_sec ||
    before.st_mtim.tv_nsec != after.st_mtim.tv_nsec ||
    before.st_ctim.tv_sec != after.st_ctim.tv_sec ||
    before.st_ctim.tv_nsec != after.st_ctim.tv_nsec)
  {
    throw std::runtime_error("SEALED_FILE_TOCTOU:" + path.string());
  }
  if (sha256_hex(bytes) != expected_sha256) {
    throw std::runtime_error("SEALED_FILE_SHA256:" + path.string());
  }
  return bytes;
}

moveit::core::RobotModelPtr load_robot_model(
  const std::string & urdf_xml, const std::string & srdf_xml)
{
  const auto urdf_model = urdf::parseURDF(urdf_xml);
  if (!urdf_model) {
    throw std::runtime_error("URDF_PARSE");
  }
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initString(*urdf_model, srdf_xml)) {
    throw std::runtime_error("SRDF_PARSE");
  }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  if (model->getName() != "h1_2") {
    throw std::runtime_error("ROBOT_MODEL_NAME");
  }
  return model;
}

std::string sha256_hex(const std::uint8_t * data, const std::size_t size)
{
  rcutils_sha256_ctx_t context;
  rcutils_sha256_init(&context);
  if (size != 0U) {
    rcutils_sha256_update(&context, data, size);
  }
  std::array<std::uint8_t, RCUTILS_SHA256_BLOCK_SIZE> digest{};
  rcutils_sha256_final(&context, digest.data());
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const auto value : digest) {
    output << std::setw(2) << static_cast<unsigned int>(value);
  }
  return output.str();
}

std::string sha256_hex(const std::vector<std::uint8_t> & bytes)
{
  return sha256_hex(bytes.data(), bytes.size());
}

std::optional<std::string> option(
  const int argc, char ** argv, const std::string & name)
{
  for (int index = 1; index + 1 < argc; ++index) {
    if (argv[index] == name) {
      return std::string(argv[index + 1]);
    }
  }
  return std::nullopt;
}

bool flag(const int argc, char ** argv, const std::string & name)
{
  for (int index = 1; index < argc; ++index) {
    if (argv[index] == name) {
      return true;
    }
  }
  return false;
}

Arguments arguments(const int argc, char ** argv)
{
  const auto required = [&](const std::string & name) {
      const auto value = option(argc, argv, name);
      if (!value || value->empty()) {
        throw std::runtime_error("ARGUMENT_MISSING:" + name);
      }
      return std::filesystem::path(*value);
    };
  Arguments result;
  result.inspect_projection = flag(argc, argv, "--inspect-projection");
  result.static_only = flag(argc, argv, "--static-only");
  result.tail_scan = flag(argc, argv, "--tail-scan");
  result.cube_only = flag(argc, argv, "--cube-only");
  result.diagnose_cube_door = flag(argc, argv, "--diagnose-cube-door");
  result.scene = required("--scene");
  result.output = required("--output");
  if (!result.inspect_projection) {
    if (result.static_only || result.tail_scan || result.cube_only ||
      result.diagnose_cube_door)
    {
      throw std::runtime_error("ARGUMENT_DIAGNOSTIC_FLAG_FORBIDDEN");
    }
    const auto park_roll = option(argc, argv, "--park-roll");
    if (!park_roll) {
      throw std::runtime_error("ARGUMENT_MISSING:--park-roll");
    }
    result.park_roll = std::stod(*park_roll);
    if (!std::isfinite(result.park_roll) ||
      (std::abs(result.park_roll - 0.312) > 1e-12 &&
      std::abs(result.park_roll - 0.316) > 1e-12 &&
      std::abs(result.park_roll - 0.320) > 1e-12))
    {
      throw std::runtime_error("ARGUMENT_PARK_ROLL_CANDIDATE");
    }
    result.plan = required("--plan");
    result.urdf = required("--urdf");
    result.srdf = required("--srdf");
    result.door_open = required("--door-open");
    result.seam_to_cube = required("--seam-to-cube");
    result.cube = required("--cube");
    result.cube_reference = required("--cube-reference");
    result.seam_to_hinge = required("--seam-to-hinge");
    result.door_close = required("--door-close");
    result.handle_stl = required("--handle-stl");
    result.robot_model_inventory = required("--robot-model-inventory");
  }
  return result;
}

moveit_msgs::msg::PlanningScene deserialize_scene(
  const std::vector<std::uint8_t> & bytes)
{
  rclcpp::SerializedMessage serialized(bytes.size());
  auto & raw = serialized.get_rcl_serialized_message();
  if (raw.buffer_capacity < bytes.size()) {
    throw std::runtime_error("CDR_BUFFER_CAPACITY");
  }
  std::memcpy(raw.buffer, bytes.data(), bytes.size());
  raw.buffer_length = bytes.size();
  moveit_msgs::msg::PlanningScene message;
  rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serialization;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

std::vector<std::uint8_t> serialize_scene(
  const moveit_msgs::msg::PlanningScene & message)
{
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serialization;
  serialization.serialize_message(&message, &serialized);
  const auto & raw = serialized.get_rcl_serialized_message();
  return std::vector<std::uint8_t>(raw.buffer, raw.buffer + raw.buffer_length);
}

bool lowercase_hex_64(const std::string & value)
{
  return value.size() == 64U && std::all_of(
    value.begin(), value.end(), [](const char character) {
      return (character >= '0' && character <= '9') ||
             (character >= 'a' && character <= 'f');
    });
}

SceneBinding parse_scene_name(const std::string & name)
{
  constexpr char prefix[] = "h1_scene_epoch:";
  if (name.rfind(prefix, 0U) != 0U) {
    throw std::runtime_error("SCENE_NAME_PREFIX");
  }
  const std::size_t epoch_begin = std::strlen(prefix);
  const std::size_t separator = name.find(':', epoch_begin);
  if (separator == std::string::npos) {
    throw std::runtime_error("SCENE_NAME_FORMAT");
  }
  const std::string epoch_text = name.substr(epoch_begin, separator - epoch_begin);
  const std::string hash = name.substr(separator + 1U);
  if (epoch_text.empty() || !std::all_of(
      epoch_text.begin(), epoch_text.end(), [](const char value) {
        return value >= '0' && value <= '9';
      }) || !lowercase_hex_64(hash))
  {
    throw std::runtime_error("SCENE_NAME_FORMAT");
  }
  return SceneBinding{std::stoull(epoch_text), hash};
}

bool is_visual_mask_id(const std::string & id)
{
  return id.find("visual_mask") != std::string::npos;
}

void classify_id(
  const std::string & id, std::set<std::string> & removed,
  std::set<std::string> & unknown)
{
  if (!is_visual_mask_id(id)) {
    return;
  }
  if (kPerceptionOnlyMaskIds.count(id) != 0U) {
    removed.insert(id);
  } else {
    unknown.insert(id);
  }
}

Projection project_physical_scene(
  const moveit_msgs::msg::PlanningScene & raw_scene,
  const std::vector<std::uint8_t> & raw_cdr)
{
  if (raw_scene.is_diff) {
    throw std::runtime_error("SCENE_IS_DIFF");
  }
  Projection result;
  result.binding = parse_scene_name(raw_scene.name);
  result.raw_cdr_sha256 = sha256_hex(raw_cdr);
  result.physical = raw_scene;
  std::set<std::string> removed;
  std::set<std::string> unknown;

  for (const auto & object : raw_scene.world.collision_objects) {
    classify_id(object.id, removed, unknown);
  }
  for (const auto & attached : raw_scene.robot_state.attached_collision_objects) {
    classify_id(attached.object.id, removed, unknown);
  }

  auto & world = result.physical.world.collision_objects;
  world.erase(std::remove_if(
      world.begin(), world.end(), [](const auto & object) {
        return kPerceptionOnlyMaskIds.count(object.id) != 0U;
      }), world.end());
  auto & attached = result.physical.robot_state.attached_collision_objects;
  attached.erase(std::remove_if(
      attached.begin(), attached.end(), [](const auto & value) {
        return kPerceptionOnlyMaskIds.count(value.object.id) != 0U;
      }), attached.end());

  result.removed.assign(removed.begin(), removed.end());
  result.unknown.assign(unknown.begin(), unknown.end());
  result.mask_conflict_pair_present =
    removed.count(kHandleMaskId) != 0U && removed.count(kLeftFingerMaskId) != 0U;
  result.handle_exact_retained = std::any_of(
    world.begin(), world.end(), [](const auto & object) {
      return object.id == kHandleId;
    });
  result.physical_cdr_sha256 = sha256_hex(serialize_scene(result.physical));
  return result;
}

void atomic_write(const std::filesystem::path & path, const std::string & payload)
{
  const auto absolute = std::filesystem::absolute(path);
  if (!absolute.parent_path().empty()) {
    std::filesystem::create_directories(absolute.parent_path());
  }
  const auto temporary = absolute.parent_path() /
    ("." + absolute.filename().string() + ".tmp");
  {
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream) {
      throw std::runtime_error("OUTPUT_CREATE");
    }
    stream << payload;
    stream.flush();
    if (!stream) {
      throw std::runtime_error("OUTPUT_WRITE");
    }
  }
  std::filesystem::rename(temporary, absolute);
}

std::string projection_json(const Projection & projection)
{
  std::ostringstream output;
  output << std::boolalpha <<
    "{\n"
    "  \"schema\": \"" << kProjectionSchema << "\",\n"
    "  \"auditor_version\": \"" << kAuditorVersion << "\",\n"
    "  \"mode\": \"READ_ONLY_PROJECTION_INSPECTION\",\n"
    "  \"scene_epoch_id\": " << projection.binding.epoch << ",\n"
    "  \"raw_scene_hash\": \"" << projection.binding.raw_scene_hash << "\",\n"
    "  \"raw_scene_cdr_sha256\": \"" << projection.raw_cdr_sha256 << "\",\n"
    "  \"physical_scene_hash\": \"" << projection.physical_cdr_sha256 << "\",\n"
    "  \"removed_visual_mask_ids\": " << json_strings(projection.removed) << ",\n"
    "  \"unknown_visual_mask_ids\": " << json_strings(projection.unknown) << ",\n"
    "  \"classification\": \"" <<
    (projection.removed.empty() ? "RAW_SCENE_ALREADY_PHYSICAL" :
    (projection.mask_conflict_pair_present ? "ALLOWLIST_MASKS_PROJECTED" :
    "INCOMPLETE_MASK_PROJECTION")) << "\",\n"
    "  \"handle_exact_retained\": " << projection.handle_exact_retained << ",\n"
    "  \"controller_commands_issued\": 0,\n"
    "  \"valid_execution_certificate\": false\n"
    "}\n";
  return output.str();
}

std::uint32_t little_u32(const std::uint8_t * bytes)
{
  return static_cast<std::uint32_t>(bytes[0]) |
         (static_cast<std::uint32_t>(bytes[1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

float little_float(const std::uint8_t * bytes)
{
  const std::uint32_t bits = little_u32(bytes);
  float result = 0.0F;
  static_assert(sizeof(result) == sizeof(bits));
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}

const moveit_msgs::msg::CollisionObject * world_object(
  const moveit_msgs::msg::PlanningScene & scene, const std::string & id)
{
  const auto found = std::find_if(
    scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
    [&](const auto & object) {return object.id == id;});
  if (found == scene.world.collision_objects.end()) {
    throw std::runtime_error("WORLD_OBJECT_MISSING:" + id);
  }
  if (std::count_if(
      scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
      [&](const auto & object) {return object.id == id;}) != 1)
  {
    throw std::runtime_error("WORLD_OBJECT_DUPLICATE:" + id);
  }
  return &*found;
}

void verify_handle_mesh(
  const moveit_msgs::msg::PlanningScene & physical_scene,
  const std::vector<std::uint8_t> & bytes)
{
  if (sha256_hex(bytes) != kExpectedHandleSha) {
    throw std::runtime_error("HANDLE_STL_SHA256");
  }
  if (bytes.size() < 84U) {
    throw std::runtime_error("HANDLE_STL_FORMAT");
  }
  const std::uint32_t count = little_u32(bytes.data() + 80U);
  if (bytes.size() != 84U + static_cast<std::size_t>(count) * 50U) {
    throw std::runtime_error("HANDLE_STL_FORMAT");
  }
  const auto * handle = world_object(physical_scene, kHandleId);
  if (handle->meshes.size() != 1U || handle->mesh_poses.size() != 1U ||
    !handle->primitives.empty() || !handle->primitive_poses.empty())
  {
    throw std::runtime_error("HANDLE_GEOMETRY_TYPE");
  }
  const auto & mesh = handle->meshes.front();
  if (mesh.triangles.size() != count || mesh.vertices.size() != count * 3U) {
    throw std::runtime_error("HANDLE_MESH_CARDINALITY");
  }
  constexpr double tolerance = 2e-9;
  for (std::size_t triangle_index = 0U; triangle_index < count; ++triangle_index) {
    const auto & triangle = mesh.triangles[triangle_index];
    const std::size_t base = 84U + triangle_index * 50U;
    for (std::size_t corner = 0U; corner < 3U; ++corner) {
      if (triangle.vertex_indices[corner] != triangle_index * 3U + corner) {
        throw std::runtime_error("HANDLE_MESH_TRIANGLE_ORDER");
      }
      const auto & vertex = mesh.vertices[triangle.vertex_indices[corner]];
      const std::array<double, 3> observed{vertex.x, vertex.y, vertex.z};
      for (std::size_t axis = 0U; axis < 3U; ++axis) {
        const double expected = static_cast<double>(little_float(
          bytes.data() + base + 12U + corner * 12U + axis * 4U)) * 0.001;
        if (!std::isfinite(observed[axis]) || std::abs(observed[axis] - expected) > tolerance) {
          throw std::runtime_error("HANDLE_MESH_VERTEX");
        }
      }
    }
  }
}

std::string scalar_string(const YAML::Node & node, const std::string & status)
{
  if (!node || !node.IsScalar()) {
    throw std::runtime_error(status);
  }
  return node.as<std::string>();
}

void require_raw_manifest(const YAML::Node & root)
{
  if (!root.IsMap() || root.size() != 5U ||
    scalar_string(root["schema"], "RAW_MANIFEST_SCHEMA") !=
      "h1_candidate3_middle_raw_asset_bindings/828-e0-v1" ||
    !root["offline_only"].as<bool>() ||
    root["controller_commands_issued"].as<int>() != 0)
  {
    throw std::runtime_error("RAW_MANIFEST_ROOT");
  }
  const std::map<std::string, std::array<std::string, 3>> expected_assets{
    {"door_open", {
      "31c17f5499d95bff4493ee49ea4803fb33f82b73f045c578598eee9421e309a7",
      "dde9709953cc2001b70c0a74ad4fa93a5151968facc530673aca1a1e2650142e",
      "door_open.trajectory.json"}},
    {"door60_seam_to_cube", {
      "5398ca08d40a3c3f926a65337eb0c2559c6e074179dad8a5a510742450095cfe",
      "6336085cb5d16bfadb1f5ae826a0db920cbf523e170a3eef6b151477b67c3245",
      "door60_seam_to_cube.trajectory.json"}},
    {"cube", {
      "e4049007ab80b724541cb8db5d19813ae9c1e743f57f7de2e994c86a43801ab4",
      "198678a7179e3ec84f8fd101dc2517e5aec356f6cb69d9a0e895761751f1fb72",
      "cube.trajectory.json"}},
    {"door60_seam_to_hinge", {
      "86f59c168629a558b801d3b4ab99537d96bf1e1871992999c8d41ce02eede75a",
      "7097e8c74330a6c9bda82df55b13e7639e5004bcb218eb4542b28b1c160cf821",
      "door60_seam_to_hinge.trajectory.json"}},
    {"door_close", {
      "fdba173ecfe9f2fef2b85f033dd3f1707dabded31d82a66934ba92504d7d835b",
      "ef01cb56754968726864b67fbe70f88b06f573788bf51ea9fd32b8f3a1bd38e7",
      "door_close.trajectory.json"}},
  };
  const auto assets = root["assets"];
  if (!assets.IsMap() || assets.size() != expected_assets.size()) {
    throw std::runtime_error("RAW_MANIFEST_ASSETS");
  }
  for (const auto & expected : expected_assets) {
    const auto binding = assets[expected.first];
    if (!binding.IsMap() || binding.size() != 4U ||
      scalar_string(binding["file_sha256"], "RAW_ASSET_FILE") !=
      expected.second[0] ||
      scalar_string(binding["payload_sha256"], "RAW_ASSET_PAYLOAD") !=
      expected.second[1] ||
      scalar_string(binding["path"], "RAW_ASSET_PATH") != expected.second[2] ||
      scalar_string(binding["schema"], "RAW_ASSET_SCHEMA") !=
      "h1_candidate3_middle_trajectory_asset/828-e0-v1")
    {
      throw std::runtime_error("RAW_MANIFEST_ASSET:" + expected.first);
    }
  }
  const std::array<std::tuple<const char *, std::size_t, std::size_t>, 11> ranges{{
    {"door_pre_support", 0U, 0U}, {"door_open_supported", 0U, 2U},
    {"door60_seam_to_cube", 0U, 2U}, {"left_cube_ready", 0U, 1U},
    {"cube_contact", 1U, 3U}, {"cube_carry_place", 3U, 5U},
    {"cube_release", 5U, 6U}, {"cube_retreat", 6U, 7U},
    {"left_cube_park", 7U, 8U}, {"door60_seam_to_hinge", 0U, 2U},
    {"door_close_supported", 0U, 2U},
  }};
  const auto phase_ranges = root["phase_message_ranges"];
  if (!phase_ranges.IsMap() || phase_ranges.size() != ranges.size()) {
    throw std::runtime_error("RAW_MANIFEST_PHASE_RANGES");
  }
  for (const auto & [name, start, end] : ranges) {
    const auto range = phase_ranges[name];
    if (!range.IsSequence() || range.size() != 2U ||
      range[0].as<std::size_t>() != start || range[1].as<std::size_t>() != end)
    {
      throw std::runtime_error(std::string("RAW_MANIFEST_PHASE_RANGE:") + name);
    }
  }
}

struct RobotModelSnapshot
{
  std::string inventory_sha256;
  std::string urdf_sha256;
  std::string srdf_sha256;
  std::string urdf_xml;
  std::string srdf_xml;
  std::vector<std::pair<std::filesystem::path, std::string>> meshes;
};

RobotModelSnapshot require_robot_model_snapshot(const Arguments & args)
{
  constexpr char inventory_sha[] =
    "320942fbd0a1607c585e247fbc43ce9cc876a616afe0565dd3c48af1aaaaa8fe";
  constexpr char urdf_sha[] =
    "5a2f06ab759564c451882ec52e3ef23655aa95a32e62e479c9f63333ae06d576";
  constexpr char srdf_sha[] =
    "2f268a57406fcea301cd1a58c3912b91aa9a3a676e2ae42b9f9c741fac30d79d";
  const auto inventory_bytes = read_sealed_bytes(
    args.robot_model_inventory, inventory_sha, 0400);
  const std::string inventory_text(
    inventory_bytes.begin(), inventory_bytes.end());
  const auto root = YAML::Load(inventory_text);
  if (!root.IsMap() || root.size() != 8U ||
    scalar_string(root["schema"], "ROBOT_MODEL_INVENTORY_SCHEMA") !=
      "h1_candidate3_middle_robot_model_snapshot/828-e0-v1" ||
    scalar_string(root["source_urdf_sha256"], "ROBOT_MODEL_SOURCE_URDF") !=
      "3aad4bdc038ff6cde3bef221bb15b8134bb45f7a87ef5dfa0fadc2126bd44929" ||
    scalar_string(root["source_srdf_sha256"], "ROBOT_MODEL_SOURCE_SRDF") !=
      srdf_sha ||
    scalar_string(root["derived_urdf_sha256"], "ROBOT_MODEL_URDF_SHA") !=
      urdf_sha ||
    scalar_string(root["srdf_sha256"], "ROBOT_MODEL_SRDF_SHA") != srdf_sha)
  {
    throw std::runtime_error("ROBOT_MODEL_INVENTORY_ROOT");
  }
  auto workspace = std::filesystem::canonical(args.robot_model_inventory);
  for (std::size_t index = 0U; index < 4U; ++index) {
    workspace = workspace.parent_path();
  }
  const auto bound_path = [&](const YAML::Node & value, const char * status) {
      const auto relative = std::filesystem::path(
        scalar_string(value, status));
      if (relative.is_absolute()) {
        throw std::runtime_error(std::string(status) + "_ABSOLUTE");
      }
      const auto resolved = std::filesystem::canonical(workspace / relative);
      if (resolved.string().rfind(workspace.string() + "/", 0U) != 0U) {
        throw std::runtime_error(std::string(status) + "_ESCAPE");
      }
      return resolved;
    };
  const auto urdf_path = bound_path(
    root["derived_urdf_path"], "ROBOT_MODEL_URDF_PATH");
  const auto srdf_path = bound_path(root["srdf_path"], "ROBOT_MODEL_SRDF_PATH");
  if (std::filesystem::canonical(args.urdf) != urdf_path ||
    std::filesystem::canonical(args.srdf) != srdf_path)
  {
    throw std::runtime_error("ROBOT_MODEL_ARGUMENT_PATH_BINDING");
  }
  const auto urdf_bytes = read_sealed_bytes(urdf_path, urdf_sha, 0400);
  const auto srdf_bytes = read_sealed_bytes(srdf_path, srdf_sha, 0400);
  RobotModelSnapshot result;
  result.inventory_sha256 = inventory_sha;
  result.urdf_sha256 = urdf_sha;
  result.srdf_sha256 = srdf_sha;
  result.urdf_xml.assign(urdf_bytes.begin(), urdf_bytes.end());
  result.srdf_xml.assign(srdf_bytes.begin(), srdf_bytes.end());
  if (result.urdf_xml.find("package://") != std::string::npos) {
    throw std::runtime_error("ROBOT_MODEL_UNBOUND_PACKAGE_URI");
  }
  const auto meshes = root["mesh_files"];
  if (!meshes.IsSequence() || meshes.size() != 35U) {
    throw std::runtime_error("ROBOT_MODEL_MESH_COUNT");
  }
  std::set<std::filesystem::path> unique_paths;
  std::set<std::string> unique_uris;
  for (const auto & mesh : meshes) {
    if (!mesh.IsMap() || mesh.size() != 6U ||
      !mesh["roles"].IsSequence() || mesh["roles"].size() == 0U)
    {
      throw std::runtime_error("ROBOT_MODEL_MESH_BINDING");
    }
    const auto path = bound_path(
      mesh["snapshot_path"], "ROBOT_MODEL_MESH_PATH");
    const auto sha = scalar_string(mesh["file_sha256"], "ROBOT_MODEL_MESH_SHA");
    const auto uri = scalar_string(
      mesh["resolved_file_uri"], "ROBOT_MODEL_MESH_URI");
    if (uri != "file://" + path.string() ||
      result.urdf_xml.find(uri) == std::string::npos ||
      !unique_paths.insert(path).second || !unique_uris.insert(uri).second)
    {
      throw std::runtime_error("ROBOT_MODEL_MESH_IDENTITY");
    }
    const auto bytes = read_sealed_bytes(path, sha, 0400);
    if (mesh["size_bytes"].as<std::size_t>() != bytes.size()) {
      throw std::runtime_error("ROBOT_MODEL_MESH_SIZE");
    }
    result.meshes.emplace_back(path, sha);
  }
  return result;
}

void reverify_robot_model_snapshot(const RobotModelSnapshot & snapshot)
{
  for (const auto & [path, sha] : snapshot.meshes) {
    (void)read_sealed_bytes(path, sha, 0400);
  }
}

AssetData require_asset(
  const std::filesystem::path & path, const std::string & file_sha,
  const std::string & payload_sha, const std::string & asset_id,
  const std::vector<std::string> & expected_topics,
  const bool synchronized)
{
  const auto file_bytes = read_sealed_bytes(path, file_sha, 0400);
  const std::string canonical_wrapper(file_bytes.begin(), file_bytes.end());
  const std::string prefix = "{\"payload\":";
  const std::string marker = ",\"sha256\":\"";
  const auto marker_position = canonical_wrapper.rfind(marker);
  const std::string expected_suffix = marker + payload_sha + "\"}\n";
  if (canonical_wrapper.rfind(prefix, 0U) != 0U ||
    marker_position == std::string::npos ||
    canonical_wrapper.substr(marker_position) != expected_suffix)
  {
    throw std::runtime_error("ASSET_CANONICAL_WRAPPER:" + path.string());
  }
  const auto payload_begin = prefix.size();
  const auto payload_size = marker_position - payload_begin;
  const auto * payload_bytes = reinterpret_cast<const std::uint8_t *>(
    canonical_wrapper.data() + payload_begin);
  if (sha256_hex(payload_bytes, payload_size) != payload_sha) {
    throw std::runtime_error("ASSET_CANONICAL_PAYLOAD_HASH:" + path.string());
  }
  const auto root = YAML::Load(canonical_wrapper);
  if (!root.IsMap() || root.size() != 2U ||
    scalar_string(root["sha256"], "ASSET_PAYLOAD_HASH") != payload_sha)
  {
    throw std::runtime_error("ASSET_WRAPPER:" + path.string());
  }
  const auto payload = root["payload"];
  if (!payload.IsMap() || payload.size() != 4U ||
    scalar_string(payload["schema"], "ASSET_SCHEMA") !=
      "h1_candidate3_middle_trajectory_asset/828-e0-v1" ||
    scalar_string(payload["asset_id"], "ASSET_ID") != asset_id ||
    !payload["messages"].IsSequence() ||
    payload["messages"].size() != expected_topics.size() ||
    !payload["synchronization_groups"].IsSequence())
  {
    throw std::runtime_error("ASSET_CONTRACT:" + path.string());
  }
  AssetData result;
  result.messages.reserve(expected_topics.size());
  for (std::size_t message_index = 0U;
    message_index < expected_topics.size(); ++message_index)
  {
    const auto message_node = payload["messages"][message_index];
    if (!message_node.IsMap() || message_node.size() != 3U ||
      scalar_string(message_node["topic"], "ASSET_TOPIC") !=
      expected_topics[message_index])
    {
      throw std::runtime_error("ASSET_MESSAGE_IDENTITY:" + path.string());
    }
    const auto names = message_node["joint_names"];
    const auto points = message_node["points"];
    if (!names.IsSequence() ||
      (names.size() != 1U && names.size() != 2U && names.size() != 7U) ||
      !points.IsSequence() || points.size() == 0U)
    {
      throw std::runtime_error("ASSET_MESSAGE_SHAPE:" + path.string());
    }
    AssetMessage message;
    message.sequence = message_index;
    for (const auto & name : names) {
      const std::string value = name.as<std::string>();
      const bool torso = expected_topics[message_index] ==
        "/torso_controller/joint_trajectory";
      const bool left = expected_topics[message_index].rfind("/left_", 0U) == 0U;
      const bool right = expected_topics[message_index].rfind("/right_", 0U) == 0U;
      if ((torso && value != "torso_joint") ||
        (left && value.rfind("left_", 0U) != 0U) ||
        (right && value.rfind("right_", 0U) != 0U))
      {
        throw std::runtime_error("ASSET_JOINT_AUTHORITY:" + path.string());
      }
      message.joint_names.push_back(value);
    }
    std::int64_t previous_time = -1;
    for (const auto & point : points) {
      if (!point.IsMap() || point.size() != 2U) {
        throw std::runtime_error("ASSET_POINT_KEYS:" + path.string());
      }
      const auto positions = point["positions"];
      if (!positions.IsSequence() || positions.size() != message.joint_names.size()) {
        throw std::runtime_error("ASSET_POINT_WIDTH:" + path.string());
      }
      std::vector<double> values;
      values.reserve(positions.size());
      for (const auto & position : positions) {
        const double value = position.as<double>();
        if (!std::isfinite(value)) {
          throw std::runtime_error("ASSET_POINT_NONFINITE:" + path.string());
        }
        values.push_back(value);
      }
      const auto time = point["time_from_start_ns"].as<std::int64_t>();
      if (time < 0 || time <= previous_time) {
        throw std::runtime_error("ASSET_POINT_TIME:" + path.string());
      }
      previous_time = time;
      message.positions.push_back(std::move(values));
      message.time_from_start_ns.push_back(time);
    }
    result.messages.push_back(std::move(message));
  }
  const auto groups = payload["synchronization_groups"];
  if (synchronized) {
    if (groups.size() != 1U || !groups[0].IsMap() || groups[0].size() != 2U ||
      !groups[0]["message_indices"].IsSequence() ||
      groups[0]["message_indices"].size() != 2U ||
      groups[0]["message_indices"][0].as<std::size_t>() != 0U ||
      groups[0]["message_indices"][1].as<std::size_t>() != 1U ||
      !groups[0]["shared_time_from_start_ns"].IsSequence() ||
      groups[0]["shared_time_from_start_ns"].size() !=
      result.messages[0].time_from_start_ns.size() ||
      result.messages[0].time_from_start_ns !=
      result.messages[1].time_from_start_ns)
    {
      throw std::runtime_error("ASSET_SYNC_GROUP:" + path.string());
    }
    for (std::size_t index = 0U;
      index < result.messages[0].time_from_start_ns.size(); ++index)
    {
      if (groups[0]["shared_time_from_start_ns"][index].as<std::int64_t>() !=
        result.messages[0].time_from_start_ns[index])
      {
        throw std::runtime_error("ASSET_SYNC_GRID:" + path.string());
      }
    }
  } else if (groups.size() != 0U) {
    throw std::runtime_error("ASSET_UNEXPECTED_SYNC_GROUP:" + path.string());
  }
  return result;
}

AssetData combine_synchronized(AssetData input)
{
  if (input.messages.size() != 2U ||
    input.messages[0].positions.size() != input.messages[1].positions.size() ||
    input.messages[0].time_from_start_ns != input.messages[1].time_from_start_ns)
  {
    throw std::runtime_error("ASSET_COMBINE_SYNC");
  }
  AssetMessage combined;
  combined.sequence = 0U;
  combined.joint_names = input.messages[0].joint_names;
  combined.joint_names.insert(
    combined.joint_names.end(), input.messages[1].joint_names.begin(),
    input.messages[1].joint_names.end());
  combined.time_from_start_ns = input.messages[0].time_from_start_ns;
  for (std::size_t index = 0U;
    index < input.messages[0].positions.size(); ++index)
  {
    auto positions = input.messages[0].positions[index];
    positions.insert(
      positions.end(), input.messages[1].positions[index].begin(),
      input.messages[1].positions[index].end());
    combined.positions.push_back(std::move(positions));
  }
  AssetData result;
  result.messages.push_back(std::move(combined));
  return result;
}

AssetData require_cube_reference(const std::filesystem::path & path)
{
  constexpr char file_sha[] =
    "09919a92a65b5ca90b3a11420bc9209ceead1e90112f94da2c4c18711af8e134";
  constexpr char payload_sha[] =
    "acaec97384f16a736f6b246a3fa1a0897b7aad93e72a6204701610ebf4f7e705";
  const auto bytes = read_sealed_bytes(path, file_sha, 0400);
  const std::string text(bytes.begin(), bytes.end());
  const std::string prefix = "{\"payload\":";
  const std::string marker = ",\"sha256\":\"";
  const auto marker_position = text.rfind(marker);
  if (text.rfind(prefix, 0U) != 0U || marker_position == std::string::npos ||
    text.substr(marker_position) != marker + payload_sha + "\"}\n")
  {
    throw std::runtime_error("CUBE_REFERENCE_CANONICAL_WRAPPER");
  }
  const auto * payload_bytes = reinterpret_cast<const std::uint8_t *>(
    text.data() + prefix.size());
  if (sha256_hex(
      payload_bytes, marker_position - prefix.size()) != payload_sha)
  {
    throw std::runtime_error("CUBE_REFERENCE_CANONICAL_PAYLOAD");
  }
  const auto root = YAML::Load(text);
  const auto payload = root["payload"];
  const std::vector<std::string> topics{
    "/left_arm_controller/joint_trajectory",
    "/left_gripper_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory",
    "/left_gripper_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory"};
  const std::vector<std::string> arm_joints{
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint"};
  const std::vector<std::string> gripper_joints{
    "left_dex1_finger_1_joint", "left_dex1_finger_2_joint"};
  const std::array<std::size_t, 6> point_counts{{32U, 11U, 360U, 28U, 11U, 413U}};
  if (!root.IsMap() || root.size() != 2U ||
    scalar_string(root["sha256"], "CUBE_REFERENCE_SHA") != payload_sha ||
    !payload.IsMap() || payload.size() != 6U ||
    scalar_string(payload["schema"], "CUBE_REFERENCE_SCHEMA") !=
      "h1_joint_trajectory_asset/v1" ||
    scalar_string(payload["stage"], "CUBE_REFERENCE_STAGE") != "cube" ||
    !payload["messages"].IsSequence() ||
    payload["messages"].size() != topics.size())
  {
    throw std::runtime_error("CUBE_REFERENCE_ROOT");
  }
  AssetData result;
  for (std::size_t message_index = 0U;
    message_index < topics.size(); ++message_index)
  {
    const auto node = payload["messages"][message_index];
    const std::size_t expected_node_size = message_index == 1U ? 6U : 5U;
    if (!node.IsMap() || node.size() != expected_node_size ||
      node["sequence"].as<std::size_t>() != message_index ||
      scalar_string(node["topic"], "CUBE_REFERENCE_TOPIC") !=
      topics[message_index] || !node["joint_names"].IsSequence() ||
      !node["points"].IsSequence() ||
      node["points"].size() != point_counts[message_index] ||
      node["wall_offset_ns"].as<std::int64_t>() < 0)
    {
      throw std::runtime_error("CUBE_REFERENCE_MESSAGE");
    }
    if (message_index == 1U) {
      const auto endpoint = node["expected_actual_endpoint"];
      if (!endpoint.IsMap() || endpoint.size() != 3U ||
        scalar_string(endpoint["kind"], "CUBE_REFERENCE_ENDPOINT_KIND") !=
        "cube_gripper_contact_close" ||
        std::abs(endpoint["desired_error_limit_rad"].as<double>() - 0.02) >
        1e-15 || !endpoint["positions"].IsSequence() ||
        endpoint["positions"].size() != 2U ||
        std::abs(endpoint["positions"][0].as<double>() + 0.014942) > 1e-12 ||
        std::abs(endpoint["positions"][1].as<double>() + 0.014942) > 1e-12)
      {
        throw std::runtime_error("CUBE_REFERENCE_ENDPOINT");
      }
    }
    AssetMessage message;
    message.sequence = message_index;
    for (const auto & name : node["joint_names"]) {
      message.joint_names.push_back(name.as<std::string>());
    }
    const auto & expected_joints =
      topics[message_index] == "/left_arm_controller/joint_trajectory" ?
      arm_joints : gripper_joints;
    if (message.joint_names != expected_joints) {
      throw std::runtime_error("CUBE_REFERENCE_JOINT_NAMES");
    }
    std::int64_t previous = -1;
    for (const auto & point : node["points"]) {
      const auto positions = point["positions"];
      const auto stamp = point["time_from_start"];
      if (!point.IsMap() || point.size() != 5U ||
        !point["accelerations"].IsSequence() ||
        point["accelerations"].size() != 0U ||
        !point["effort"].IsSequence() || point["effort"].size() != 0U ||
        !point["velocities"].IsSequence() ||
        point["velocities"].size() != 0U || !positions.IsSequence() ||
        positions.size() != message.joint_names.size() ||
        !stamp.IsMap() || stamp.size() != 2U)
      {
        throw std::runtime_error("CUBE_REFERENCE_POINT");
      }
      std::vector<double> values;
      for (const auto & position : positions) {
        const double value = position.as<double>();
        if (!std::isfinite(value)) {
          throw std::runtime_error("CUBE_REFERENCE_NONFINITE");
        }
        values.push_back(value);
      }
      const auto seconds = stamp["sec"].as<std::int64_t>();
      const auto nanoseconds = stamp["nanosec"].as<std::int64_t>();
      if (seconds < 0 || nanoseconds < 0 || nanoseconds >= 1000000000LL) {
        throw std::runtime_error("CUBE_REFERENCE_STAMP");
      }
      const auto time = seconds * 1000000000LL + nanoseconds;
      if (time < 0 || time <= previous) {
        throw std::runtime_error("CUBE_REFERENCE_TIME");
      }
      previous = time;
      message.positions.push_back(std::move(values));
      message.time_from_start_ns.push_back(time);
    }
    result.messages.push_back(std::move(message));
  }
  return result;
}

std::map<AssetKind, AssetData> require_assets(const Arguments & args)
{
  const std::vector<std::string> pair_topics{
    "/torso_controller/joint_trajectory",
    "/right_arm_controller/joint_trajectory"};
  const std::vector<std::string> cube_topics{
    "/left_arm_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory",
    "/left_gripper_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory",
    "/left_gripper_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory"};
  auto door_open = combine_synchronized(require_asset(
    args.door_open,
    "31c17f5499d95bff4493ee49ea4803fb33f82b73f045c578598eee9421e309a7",
    "dde9709953cc2001b70c0a74ad4fa93a5151968facc530673aca1a1e2650142e",
    "door_open", pair_topics, true));
  auto seam_to_cube = combine_synchronized(require_asset(
    args.seam_to_cube,
    "5398ca08d40a3c3f926a65337eb0c2559c6e074179dad8a5a510742450095cfe",
    "6336085cb5d16bfadb1f5ae826a0db920cbf523e170a3eef6b151477b67c3245",
    "door60_seam_to_cube", pair_topics, true));
  auto cube = require_asset(
    args.cube,
    "e4049007ab80b724541cb8db5d19813ae9c1e743f57f7de2e994c86a43801ab4",
    "198678a7179e3ec84f8fd101dc2517e5aec356f6cb69d9a0e895761751f1fb72",
    "cube", cube_topics, false);
  auto cube_reference = require_cube_reference(args.cube_reference);
  auto seam_to_hinge = combine_synchronized(require_asset(
    args.seam_to_hinge,
    "86f59c168629a558b801d3b4ab99537d96bf1e1871992999c8d41ce02eede75a",
    "7097e8c74330a6c9bda82df55b13e7639e5004bcb218eb4542b28b1c160cf821",
    "door60_seam_to_hinge", pair_topics, true));
  auto door_close = combine_synchronized(require_asset(
    args.door_close,
    "fdba173ecfe9f2fef2b85f033dd3f1707dabded31d82a66934ba92504d7d835b",
    "ef01cb56754968726864b67fbe70f88b06f573788bf51ea9fd32b8f3a1bd38e7",
    "door_close", pair_topics, true));

  const auto reverse_equal = [](const AssetData & first, const AssetData & second,
      const std::string & status) {
      const auto & forward = first.messages.at(0U).positions;
      const auto & reverse = second.messages.at(0U).positions;
      if (forward.size() != reverse.size()) {
        throw std::runtime_error(status + "_COUNT");
      }
      for (std::size_t index = 0U; index < forward.size(); ++index) {
        const auto & left = forward[index];
        const auto & right = reverse[reverse.size() - 1U - index];
        if (left.size() != right.size()) {
          throw std::runtime_error(status + "_WIDTH");
        }
        for (std::size_t joint = 0U; joint < left.size(); ++joint) {
          if (std::abs(left[joint] - right[joint]) > 1e-12) {
            throw std::runtime_error(status + "_VALUE");
          }
        }
      }
    };
  reverse_equal(door_open, door_close, "DOOR_REVERSE_TRAJECTORY");
  reverse_equal(seam_to_cube, seam_to_hinge, "SEAM_REVERSE_TRAJECTORY");

  AssetData door_pre;
  AssetMessage door_pre_message;
  door_pre_message.sequence = 0U;
  door_pre_message.joint_names = door_open.messages.at(0U).joint_names;
  door_pre_message.positions = {door_open.messages.at(0U).positions.at(0U)};
  door_pre_message.time_from_start_ns = {0};
  door_pre.messages.push_back(std::move(door_pre_message));

  AssetData ready;
  ready.messages.push_back(cube.messages.at(0U));
  AssetData inner_cube;
  for (std::size_t index = 1U; index <= 6U; ++index) {
    auto message = cube.messages.at(index);
    message.sequence = index - 1U;
    inner_cube.messages.push_back(std::move(message));
  }
  AssetData park;
  park.messages.push_back(cube.messages.at(7U));
  return {
    {AssetKind::DoorPre, std::move(door_pre)},
    {AssetKind::DoorOpen, std::move(door_open)},
    {AssetKind::Door60SeamToCube, std::move(seam_to_cube)},
    {AssetKind::LeftCubeReady, std::move(ready)},
    {AssetKind::Cube, std::move(inner_cube)},
    {AssetKind::LeftCubePark, std::move(park)},
    {AssetKind::Door60SeamToHinge, std::move(seam_to_hinge)},
    {AssetKind::DoorClose, std::move(door_close)},
    {AssetKind::CubeReference, std::move(cube_reference)},
  };
}

Eigen::Isometry3d transform_rpy(
  const double roll, const double pitch, const double yaw,
  const Eigen::Vector3d & translation)
{
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.linear() = (
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  output.translation() = translation;
  return output;
}

double rotation_distance(
  const Eigen::Isometry3d & first, const Eigen::Isometry3d & second)
{
  return Eigen::AngleAxisd(first.linear().transpose() * second.linear()).angle();
}

class FclGeometryCache
{
public:
  std::shared_ptr<fcl::CollisionGeometryd> get(const shapes::ShapeConstPtr & shape)
  {
    if (!shape) {
      throw std::runtime_error("FCL_NULL_SHAPE");
    }
    const auto found = cache_.find(shape.get());
    if (found != cache_.end()) {
      return found->second;
    }
    std::shared_ptr<fcl::CollisionGeometryd> geometry;
    switch (shape->type) {
      case shapes::SPHERE: {
          const auto * value = static_cast<const shapes::Sphere *>(shape.get());
          geometry = std::make_shared<fcl::Sphered>(value->radius);
          break;
        }
      case shapes::CYLINDER: {
          const auto * value = static_cast<const shapes::Cylinder *>(shape.get());
          geometry = std::make_shared<fcl::Cylinderd>(value->radius, value->length);
          break;
        }
      case shapes::CONE: {
          const auto * value = static_cast<const shapes::Cone *>(shape.get());
          geometry = std::make_shared<fcl::Coned>(value->radius, value->length);
          break;
        }
      case shapes::BOX: {
          const auto * value = static_cast<const shapes::Box *>(shape.get());
          geometry = std::make_shared<fcl::Boxd>(
            value->size[0], value->size[1], value->size[2]);
          break;
        }
      case shapes::MESH: {
          const auto * value = static_cast<const shapes::Mesh *>(shape.get());
          auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
          std::vector<fcl::Vector3d> vertices;
          std::vector<fcl::Triangle> triangles;
          vertices.reserve(value->vertex_count);
          triangles.reserve(value->triangle_count);
          for (std::size_t index = 0U; index < value->vertex_count; ++index) {
            vertices.emplace_back(
              value->vertices[3U * index], value->vertices[3U * index + 1U],
              value->vertices[3U * index + 2U]);
          }
          for (std::size_t index = 0U; index < value->triangle_count; ++index) {
            triangles.emplace_back(
              value->triangles[3U * index], value->triangles[3U * index + 1U],
              value->triangles[3U * index + 2U]);
          }
          if (model->beginModel(value->triangle_count, value->vertex_count) !=
            fcl::BVH_OK || model->addSubModel(vertices, triangles) != fcl::BVH_OK ||
            model->endModel() != fcl::BVH_OK)
          {
            throw std::runtime_error("FCL_MESH_BUILD");
          }
          geometry = model;
          break;
        }
      default:
        throw std::runtime_error("FCL_UNSUPPORTED_SHAPE");
    }
    cache_.emplace(shape.get(), geometry);
    return geometry;
  }

private:
  std::map<const shapes::Shape *, std::shared_ptr<fcl::CollisionGeometryd>> cache_;
};

double direct_shape_distance(
  FclGeometryCache & cache,
  const shapes::ShapeConstPtr & first_shape,
  const Eigen::Isometry3d & first_pose,
  const shapes::ShapeConstPtr & second_shape,
  const Eigen::Isometry3d & second_pose)
{
  const fcl::Transform3d first_transform = first_pose;
  const fcl::Transform3d second_transform = second_pose;
  fcl::CollisionObjectd first(cache.get(first_shape), first_transform);
  fcl::CollisionObjectd second(cache.get(second_shape), second_transform);
  fcl::DistanceRequestd request(true, true, 0.0, 0.0, 1e-9);
  fcl::DistanceResultd result;
  const double distance = fcl::distance(&first, &second, request, result);
  if (!std::isfinite(distance) || !std::isfinite(result.min_distance)) {
    throw std::runtime_error("FCL_NONFINITE_DISTANCE");
  }
  return result.min_distance;
}

double direct_body_distance(
  FclGeometryCache & cache,
  const std::vector<shapes::ShapeConstPtr> & first_shapes,
  const EigenSTL::vector_Isometry3d & first_poses,
  const std::vector<shapes::ShapeConstPtr> & second_shapes,
  const EigenSTL::vector_Isometry3d & second_poses)
{
  if (first_shapes.empty() || second_shapes.empty() ||
    first_shapes.size() != first_poses.size() || second_shapes.size() != second_poses.size())
  {
    throw std::runtime_error("FCL_BODY_SHAPE_COUNT");
  }
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t first = 0U; first < first_shapes.size(); ++first) {
    for (std::size_t second = 0U; second < second_shapes.size(); ++second) {
      minimum = std::min(
        minimum, direct_shape_distance(
          cache, first_shapes[first], first_poses[first],
          second_shapes[second], second_poses[second]));
    }
  }
  return minimum;
}

struct DynamicGeometry
{
  shapes::ShapeConstPtr door_shape;
  shapes::ShapeConstPtr handle_shape;
  shapes::ShapeConstPtr cube_shape;
  shapes::ShapeConstPtr cube_guard_shape;
  std::vector<shapes::ShapeConstPtr> handle_mask_shapes;
  Eigen::Isometry3d initial_door_frame{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d initial_handle_pose{Eigen::Isometry3d::Identity()};
  EigenSTL::vector_Isometry3d handle_mask_in_handle;
  Eigen::Isometry3d initial_cube_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d cube_guard_in_cube{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d target_cube_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d tcp_to_cube{Eigen::Isometry3d::Identity()};
  bool cube_attachment_captured{false};
};

Eigen::Isometry3d desired_door_pose(
  const DynamicGeometry & geometry, const double door_angle)
{
  const auto door_visual = transform_rpy(
    kPi / 2.0, 0.0, -1.528168312093,
    Eigen::Vector3d(0.005592142864, 0.408609505443, 0.0));
  return geometry.initial_door_frame *
         transform_rpy(0.0, 0.0, door_angle, Eigen::Vector3d::Zero()) *
         door_visual;
}

Eigen::Isometry3d desired_handle_pose(
  const DynamicGeometry & geometry, const double door_angle)
{
  const auto door_handle = transform_rpy(
    0.0, 0.0, 1.613424341497,
    Eigen::Vector3d(-0.019105784513, 0.377528763146, 0.189999998514));
  const auto handle_rotation = transform_rpy(
    0.0, kHandleAngle, 0.0, Eigen::Vector3d::Zero());
  const auto handle_visual = transform_rpy(
    0.0, 0.0, 0.0,
    Eigen::Vector3d(-0.009999668689, 0.000027284668, -0.119003896682));
  return geometry.initial_door_frame *
         transform_rpy(0.0, 0.0, door_angle, Eigen::Vector3d::Zero()) *
         door_handle * handle_rotation * handle_visual;
}

void set_single_shape_pose(
  planning_scene::PlanningScene & scene, const std::string & id,
  const shapes::ShapeConstPtr & shape, const Eigen::Isometry3d & global_pose)
{
  Eigen::Isometry3d local_pose = Eigen::Isometry3d::Identity();
  {
    // Release the old World::Object before MoveIt replaces it.  Keeping the
    // shared_ptr alive across the observer callback leaves FCL's body-name
    // reference dangling and corrupts the next ACM query.
    const auto object = scene.getWorld()->getObject(id);
    if (!object || object->shapes_.size() != 1U || object->shapes_[0] != shape) {
      throw std::runtime_error("DYNAMIC_OBJECT_SHAPE:" + id);
    }
    local_pose = object->pose_.inverse() * global_pose;
  }
  if (!scene.getWorldNonConst()->moveShapeInObject(id, shape, local_pose)) {
    throw std::runtime_error("DYNAMIC_OBJECT_MOVE:" + id);
  }
}

DynamicGeometry prepare_dynamic_geometry(
  planning_scene::PlanningScene & physical_scene,
  planning_scene::PlanningScene & raw_scene,
  const bool mask_conflict_pair_present,
  const bool project_canonical_e0_entry)
{
  DynamicGeometry output;
  const auto door = physical_scene.getWorld()->getObject("incubator_door_exact");
  const auto handle = physical_scene.getWorld()->getObject(kHandleId);
  const auto cube = physical_scene.getWorld()->getObject("green_cube_dynamic_exact");
  const auto guard = physical_scene.getWorld()->getObject("green_cube_10mm_guard");
  const auto mask = raw_scene.getWorld()->getObject(kHandleMaskId);
  if (!door || door->shapes_.size() != 1U ||
    !handle || handle->shapes_.size() != 1U ||
    !cube || cube->shapes_.size() != 1U ||
    !guard || guard->shapes_.size() != 1U ||
    (mask_conflict_pair_present && (!mask || mask->shapes_.empty())) ||
    (!mask_conflict_pair_present && mask))
  {
    throw std::runtime_error("DYNAMIC_GEOMETRY_OBJECTS");
  }
  output.door_shape = door->shapes_.front();
  output.handle_shape = handle->shapes_.front();
  output.cube_shape = cube->shapes_.front();
  output.cube_guard_shape = guard->shapes_.front();
  if (mask_conflict_pair_present) {
    output.handle_mask_shapes = mask->shapes_;
  }
  const auto door_visual = transform_rpy(
    kPi / 2.0, 0.0, -1.528168312093,
    Eigen::Vector3d(0.005592142864, 0.408609505443, 0.0));
  output.initial_door_frame = door->global_shape_poses_.front() * door_visual.inverse();
  output.initial_handle_pose = handle->global_shape_poses_.front();
  const auto expected_handle = desired_handle_pose(output, 0.0);
  if ((expected_handle.translation() - output.initial_handle_pose.translation()).norm() > 1e-6 ||
    rotation_distance(expected_handle, output.initial_handle_pose) > 1e-6)
  {
    if (!project_canonical_e0_entry) {
      throw std::runtime_error("HANDLE_Q90_INITIAL_POSE");
    }
    // The authoritative captured scene is the fresh q0/locked scene.  The
    // prefix owns q0->q90; cube-only replay starts at that explicitly bound
    // E0 boundary and therefore projects only the dynamic handle pose here.
    output.initial_handle_pose = expected_handle;
  }
  if (mask_conflict_pair_present) {
    for (const auto & pose : mask->global_shape_poses_) {
      output.handle_mask_in_handle.push_back(output.initial_handle_pose.inverse() * pose);
    }
  }
  output.initial_cube_pose = cube->global_shape_poses_.front();
  output.cube_guard_in_cube = output.initial_cube_pose.inverse() *
    guard->global_shape_poses_.front();

  const auto spawn_world = transform_rpy(
    0.0, 0.0, kPi / 2.0, Eigen::Vector3d(0.334, 0.377, 0.830));
  const auto target_world = transform_rpy(
    0.0, 0.0, kPi / 2.0,
    Eigen::Vector3d(0.610890542898, -0.029527393333, 0.995407082351));
  const Eigen::Isometry3d planning_from_world =
    output.initial_cube_pose * spawn_world.inverse();
  output.target_cube_pose = planning_from_world * target_world;
  return output;
}

double message_door_angle(
  const AssetKind asset, const std::size_t,
  const std::size_t point_index, const std::size_t point_count)
{
  const double fraction = point_count <= 1U ? 1.0 :
    static_cast<double>(point_index) / static_cast<double>(point_count - 1U);
  if (asset == AssetKind::DoorOpen) {
    return fraction * kDoorOpenAngle;
  }
  if (asset == AssetKind::DoorClose) {
    return (1.0 - fraction) * kDoorOpenAngle;
  }
  if (asset == AssetKind::DoorPre) {
    return 0.0;
  }
  return kDoorOpenAngle;
}

std::size_t subdivision_count(
  const moveit::core::RobotModelConstPtr & model,
  const moveit::core::RobotState & first,
  const moveit::core::RobotState & second,
  const std::vector<std::string> & changed_variables,
  const double first_door_angle, const double second_door_angle)
{
  std::size_t subdivisions = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(
      std::abs(second_door_angle - first_door_angle) / kAngularStep)));
  for (const auto & variable : changed_variables) {
    const auto * joint = model->getJointOfVariable(variable);
    if (!joint) {
      throw std::runtime_error("ASSET_UNKNOWN_JOINT:" + variable);
    }
    const double delta = std::abs(
      second.getVariablePosition(variable) - first.getVariablePosition(variable));
    double step = 0.0;
    if (joint->getType() == moveit::core::JointModel::REVOLUTE) {
      step = kAngularStep;
    } else if (joint->getType() == moveit::core::JointModel::PRISMATIC) {
      step = kPrismaticStep;
    } else if (delta <= 1e-12) {
      continue;
    } else {
      throw std::runtime_error("ASSET_UNSUPPORTED_JOINT_TYPE:" + variable);
    }
    subdivisions = std::max<std::size_t>(
      subdivisions, static_cast<std::size_t>(std::ceil(delta / step)));
  }
  return subdivisions;
}

void audit_cube_bump_against_reference(
  const moveit::core::RobotModelConstPtr & model,
  const moveit::core::RobotState & cube_start, const AssetData & modified,
  const AssetData & reference, ExactAuditResult & output)
{
  constexpr double translation_limit = 0.006;
  constexpr double rotation_limit = 0.013962634015954637;
  if (modified.messages.size() != 6U ||
    reference.messages.size() != modified.messages.size())
  {
    throw std::runtime_error("CUBE_BUMP_MESSAGE_COUNT");
  }
  moveit::core::RobotState modified_state(cube_start);
  moveit::core::RobotState reference_state(cube_start);
  bool roll_delta_observed = false;
  const auto observe = [&](const moveit::core::RobotState & modified_sample,
      const moveit::core::RobotState & reference_sample) {
      const auto & modified_tcp =
        modified_sample.getGlobalLinkTransform("left_dex1_tcp");
      const auto & reference_tcp =
        reference_sample.getGlobalLinkTransform("left_dex1_tcp");
      output.cube_bump_maximum_tcp_translation = std::max(
        output.cube_bump_maximum_tcp_translation,
        (modified_tcp.translation() - reference_tcp.translation()).norm());
      output.cube_bump_maximum_tcp_rotation = std::max(
        output.cube_bump_maximum_tcp_rotation,
        rotation_distance(modified_tcp, reference_tcp));
    };
  observe(modified_state, reference_state);
  for (std::size_t message_index = 0U;
    message_index < modified.messages.size(); ++message_index)
  {
    const auto & modified_message = modified.messages[message_index];
    const auto & reference_message = reference.messages[message_index];
    if (modified_message.sequence != message_index ||
      reference_message.sequence != message_index ||
      modified_message.joint_names != reference_message.joint_names ||
      modified_message.positions.size() != reference_message.positions.size() ||
      modified_message.time_from_start_ns !=
      reference_message.time_from_start_ns)
    {
      throw std::runtime_error("CUBE_BUMP_REFERENCE_STRUCTURE");
    }
    for (std::size_t point_index = 0U;
      point_index < modified_message.positions.size(); ++point_index)
    {
      const auto & modified_positions =
        modified_message.positions[point_index];
      const auto & reference_positions =
        reference_message.positions[point_index];
      if (modified_positions.size() != modified_message.joint_names.size() ||
        reference_positions.size() != modified_positions.size())
      {
        throw std::runtime_error("CUBE_BUMP_REFERENCE_WIDTH");
      }
      for (std::size_t joint_index = 0U;
        joint_index < modified_positions.size(); ++joint_index)
      {
        const double delta = modified_positions[joint_index] -
          reference_positions[joint_index];
        if (modified_message.joint_names[joint_index] ==
          "left_shoulder_roll_joint")
        {
          roll_delta_observed = roll_delta_observed || std::abs(delta) > 1e-12;
        } else if (std::abs(delta) > 1e-12) {
          throw std::runtime_error(
                  "CUBE_BUMP_UNAUTHORIZED_JOINT_DELTA:" +
                  modified_message.joint_names[joint_index]);
        }
      }
      moveit::core::RobotState modified_next(modified_state);
      modified_next.setVariablePositions(
        modified_message.joint_names, modified_positions);
      modified_next.updateCollisionBodyTransforms();
      moveit::core::RobotState reference_next(reference_state);
      reference_next.setVariablePositions(
        reference_message.joint_names, reference_positions);
      reference_next.updateCollisionBodyTransforms();
      const std::size_t subdivisions = std::max(
        subdivision_count(
          model, modified_state, modified_next, modified_message.joint_names,
          kDoorOpenAngle, kDoorOpenAngle),
        subdivision_count(
          model, reference_state, reference_next, reference_message.joint_names,
          kDoorOpenAngle, kDoorOpenAngle));
      for (std::size_t subdivision = 1U;
        subdivision <= subdivisions; ++subdivision)
      {
        const double fraction = static_cast<double>(subdivision) /
          static_cast<double>(subdivisions);
        moveit::core::RobotState modified_sample(modified_state);
        moveit::core::RobotState reference_sample(reference_state);
        modified_state.interpolate(modified_next, fraction, modified_sample);
        reference_state.interpolate(reference_next, fraction, reference_sample);
        modified_sample.updateCollisionBodyTransforms();
        reference_sample.updateCollisionBodyTransforms();
        observe(modified_sample, reference_sample);
      }
      modified_state = modified_next;
      reference_state = reference_next;
    }
  }
  if (!roll_delta_observed ||
    !std::isfinite(output.cube_bump_maximum_tcp_translation) ||
    !std::isfinite(output.cube_bump_maximum_tcp_rotation) ||
    output.cube_bump_maximum_tcp_translation < 0.0 ||
    output.cube_bump_maximum_tcp_rotation < 0.0 ||
    output.cube_bump_maximum_tcp_translation > translation_limit + 1e-12 ||
    output.cube_bump_maximum_tcp_rotation > rotation_limit + 1e-12)
  {
    throw std::runtime_error("CUBE_BUMP_TCP_LIMIT");
  }
}

struct IndexedWitnessState
{
  std::size_t dense_index;
  std::string phase;
  std::size_t source_message_index;
  moveit::core::RobotState state;
};

std::array<IndexedWitnessState, 3> trace_release_witness_grid(
  const moveit::core::RobotState & cube_start, const AssetData & cube)
{
  constexpr double witness_angular_step = 3.0 * kPi / 180.0;
  struct WitnessPhase
  {
    const char * name;
    std::size_t start;
    std::size_t end;
  };
  const std::array<WitnessPhase, 4> ranges{{
    {"cube_contact", 0U, 2U}, {"cube_carry_place", 2U, 4U},
    {"cube_release", 4U, 5U}, {"cube_retreat", 5U, 6U},
  }};
  moveit::core::RobotState current(cube_start);
  std::vector<IndexedWitnessState> selected;
  std::size_t index = 0U;
  const auto capture = [&](const char * phase, const std::size_t message_index,
      const moveit::core::RobotState & state) {
      if (index >= 526U && index <= 528U) {
        selected.push_back(IndexedWitnessState{
          index, phase, message_index, moveit::core::RobotState(state)});
      }
    };
  for (const auto & range : ranges) {
    ++index;
    capture(range.name, range.start, current);  // Explicit phase seam sample.
    for (std::size_t message_index = range.start;
      message_index < range.end; ++message_index)
    {
      const auto & message = cube.messages.at(message_index);
      for (const auto & point : message.positions) {
        moveit::core::RobotState next(current);
        next.setVariablePositions(message.joint_names, point);
        next.updateCollisionBodyTransforms();
        std::size_t subdivisions = 1U;
        for (const auto & variable : message.joint_names) {
          const auto * joint = current.getRobotModel()->getJointOfVariable(variable);
          if (!joint) {
            throw std::runtime_error("WITNESS_UNKNOWN_JOINT:" + variable);
          }
          const double step = joint->getType() == moveit::core::JointModel::PRISMATIC ?
            kPrismaticStep : witness_angular_step;
          subdivisions = std::max<std::size_t>(
            subdivisions, static_cast<std::size_t>(std::ceil(
              std::abs(next.getVariablePosition(variable) -
              current.getVariablePosition(variable)) / step)));
        }
        for (std::size_t subdivision = 1U;
          subdivision <= subdivisions; ++subdivision)
        {
          moveit::core::RobotState sample(current);
          current.interpolate(
            next, static_cast<double>(subdivision) /
            static_cast<double>(subdivisions), sample);
          sample.updateCollisionBodyTransforms();
          ++index;
          capture(range.name, message_index, sample);
        }
        current = next;
      }
    }
  }
  if (selected.size() != 3U ||
    selected[0].dense_index != 526U ||
    selected[1].dense_index != 527U ||
    selected[2].dense_index != 528U ||
    selected[1].phase != "cube_release" ||
    selected[1].source_message_index != 4U)
  {
    throw std::runtime_error("CUBE_RELEASE_GLOBAL_DENSE_INDEX_527");
  }
  return {std::move(selected[0]), std::move(selected[1]), std::move(selected[2])};
}

void verify_seam_state(
  const moveit::core::RobotState & state, const double park_roll)
{
  const std::array<std::string, 19> names{
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
    "left_dex1_finger_1_joint", "left_dex1_finger_2_joint",
    "right_dex1_finger_1_joint", "right_dex1_finger_2_joint", "torso_joint"};
  const std::array<double, 19> nominal{
    -0.000012933885986937658, park_roll, 0.000023774721497885907,
    0.6000753185207717, 0.000023572560034304773,
    -0.00008730492997967213, -0.00001654021997178485,
    -0.41373146194762, 0.23755434038212944, 1.1008080707922079,
    0.8747783439736734, -0.3046852067271319, 0.4094159191441438,
    -0.9999635680199944,
    -0.019, -0.019, 0.0108, 0.007, -0.04773321113053429};
  for (std::size_t index = 0U; index < names.size(); ++index) {
    if (!state.getRobotModel()->getJointOfVariable(names[index])) {
      throw std::runtime_error("SEAM_VARIABLE_MISSING:" + names[index]);
    }
    const double tolerance = 1e-12;
    const double value = state.getVariablePosition(names[index]);
    if (!std::isfinite(value) || std::abs(value - nominal[index]) > tolerance) {
      throw std::runtime_error("SEAM_POSITION:" + names[index]);
    }
  }
}

void move_dynamic_geometry(
  planning_scene::PlanningScene & physical_scene,
  planning_scene::PlanningScene & raw_scene,
  const DynamicGeometry & geometry, const double door_angle,
  const Eigen::Isometry3d & cube_pose)
{
  const auto door_pose = desired_door_pose(geometry, door_angle);
  const auto handle_pose = desired_handle_pose(geometry, door_angle);
  set_single_shape_pose(
    physical_scene, "incubator_door_exact", geometry.door_shape, door_pose);
  set_single_shape_pose(physical_scene, kHandleId, geometry.handle_shape, handle_pose);
  set_single_shape_pose(
    physical_scene, "green_cube_dynamic_exact", geometry.cube_shape, cube_pose);
  set_single_shape_pose(
    physical_scene, "green_cube_10mm_guard", geometry.cube_guard_shape,
    cube_pose * geometry.cube_guard_in_cube);

  if (geometry.handle_mask_shapes.empty()) {
    return;
  }
  for (std::size_t index = 0U;
    index < geometry.handle_mask_shapes.size(); ++index)
  {
    Eigen::Isometry3d local_pose = Eigen::Isometry3d::Identity();
    {
      const auto mask = raw_scene.getWorld()->getObject(kHandleMaskId);
      if (!mask || mask->shapes_.size() != geometry.handle_mask_shapes.size() ||
        mask->shapes_.size() != geometry.handle_mask_in_handle.size())
      {
        throw std::runtime_error("HANDLE_MASK_GEOMETRY_CHANGED");
      }
      local_pose = mask->pose_.inverse() *
        handle_pose * geometry.handle_mask_in_handle[index];
    }
    if (!raw_scene.getWorldNonConst()->moveShapeInObject(
        kHandleMaskId, geometry.handle_mask_shapes[index], local_pose))
    {
      throw std::runtime_error("HANDLE_MASK_MOVE");
    }
  }
}

bool right_handle_contact_authorized(
  const AssetKind, const std::size_t)
{
  // The prefix enters at canonical E0 with q90 already held.  The middle never
  // releases or retreats the right hand; the suffix owns that transition.
  return true;
}

bool right_handle_distance_window(
  const AssetKind, const std::size_t)
{
  return true;
}

void add_authorized_pairs(
  collision_detection::AllowedCollisionMatrix & matrix,
  const AssetKind asset, const std::size_t message_index)
{
  if (right_handle_contact_authorized(asset, message_index)) {
    for (const auto & pad : kRightHandlePads) {
      matrix.setEntry(kHandleId, pad, true);
    }
  }
  const bool cube_hand_contact =
    asset == AssetKind::Cube && message_index <= 5U;
  if (cube_hand_contact) {
    if (message_index <= 4U) {
      for (const auto & link : kLeftCubeTouchLinks) {
        matrix.setEntry("green_cube_dynamic_exact", link, true);
        matrix.setEntry("green_cube_10mm_guard", link, true);
      }
    } else {
      for (const auto & link : kLeftCubeReleaseRetreatLinks) {
        matrix.setEntry("green_cube_dynamic_exact", link, true);
        matrix.setEntry("green_cube_10mm_guard", link, true);
      }
    }
  }
}

void update_minimum_from_distance(
  const collision_detection::DistanceResult & distance_result,
  const std::set<std::string> & required_bodies, double & minimum)
{
  bool observed = false;
  for (const auto & pair : distance_result.distances) {
    for (const auto & value : pair.second) {
      const bool selected = required_bodies.count(value.link_names[0]) != 0U ||
        required_bodies.count(value.link_names[1]) != 0U;
      if (selected && std::isfinite(value.distance)) {
        observed = true;
        minimum = std::min(minimum, value.distance);
      }
    }
  }
  if (!observed) {
    minimum = std::min(minimum, kDistanceBand);
  }
}

double physical_environment_distance(
  const collision_detection::DistanceResultsData & value)
{
  const bool expanded_guard =
    value.link_names[0] == "environment_table_10mm_margin" ||
    value.link_names[1] == "environment_table_10mm_margin" ||
    value.link_names[0] == "green_cube_10mm_guard" ||
    value.link_names[1] == "green_cube_10mm_guard";
  return value.distance + (expanded_guard ? kRequiredClearance : 0.0);
}

void update_environment_minimum(
  const collision_detection::DistanceResult & distance_result,
  const std::set<std::string> & environment_bodies, double & minimum)
{
  for (const auto & pair : distance_result.distances) {
    for (const auto & value : pair.second) {
      const bool selected = environment_bodies.count(value.link_names[0]) != 0U ||
        environment_bodies.count(value.link_names[1]) != 0U;
      if (selected && std::isfinite(value.distance)) {
        minimum = std::min(minimum, physical_environment_distance(value));
      }
    }
  }
  // No near-band pair is an honest >=10.001 mm lower bound.  Expanded table
  // and cube guards are conservative supersets of the uniform 10 mm offset:
  // box axes grow by 2*margin, cylinder height by 2*margin and radius by
  // margin.  Therefore guard non-collision plus distance d proves at least
  // 10 mm+d clearance from the underlying physical surface.
  minimum = std::min(minimum, kDistanceBand);
}

double selected_environment_minimum(
  const collision_detection::DistanceResult & distance_result,
  const std::set<std::string> & bodies)
{
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto & pair : distance_result.distances) {
    for (const auto & value : pair.second) {
      if ((bodies.count(value.link_names[0]) != 0U ||
        bodies.count(value.link_names[1]) != 0U) &&
        std::isfinite(value.distance))
      {
        minimum = std::min(minimum, physical_environment_distance(value));
      }
    }
  }
  return minimum;
}

std::string closest_distance_pair(
  const collision_detection::DistanceResult & distance_result,
  const std::set<std::string> & required_bodies)
{
  double minimum = std::numeric_limits<double>::infinity();
  std::string first;
  std::string second;
  for (const auto & pair : distance_result.distances) {
    for (const auto & value : pair.second) {
      const bool selected = required_bodies.count(value.link_names[0]) != 0U ||
        required_bodies.count(value.link_names[1]) != 0U;
      const double physical_distance = physical_environment_distance(value);
      if (selected && std::isfinite(physical_distance) && physical_distance < minimum) {
        minimum = physical_distance;
        first = value.link_names[0];
        second = value.link_names[1];
      }
    }
  }
  std::ostringstream output;
  output << first << ":" << second << ":" << std::setprecision(17) << minimum;
  return output.str();
}

void allow_all_self_distance_pairs(
  collision_detection::AllowedCollisionMatrix & matrix,
  const std::vector<std::string> & collision_links)
{
  for (std::size_t first = 0U; first < collision_links.size(); ++first) {
    for (std::size_t second = first; second < collision_links.size(); ++second) {
      matrix.setEntry(collision_links[first], collision_links[second], true);
    }
  }
}

Eigen::Isometry3d cube_pose_for_sample(
  const DynamicGeometry & geometry, const moveit::core::RobotState & sample,
  const AssetKind asset, const std::size_t message_index)
{
  if (asset == AssetKind::DoorPre || asset == AssetKind::DoorOpen ||
    asset == AssetKind::Door60SeamToCube ||
    asset == AssetKind::LeftCubeReady ||
    (asset == AssetKind::Cube && message_index <= 1U))
  {
    return geometry.initial_cube_pose;
  }
  if (asset == AssetKind::Cube && message_index <= 3U) {
    if (!geometry.cube_attachment_captured) {
      throw std::runtime_error("CUBE_ATTACHMENT_NOT_CAPTURED");
    }
    return sample.getGlobalLinkTransform("left_dex1_tcp") * geometry.tcp_to_cube;
  }
  return geometry.target_cube_pose;
}

NeighborhoodSample physical_finger2_distance(
  FclGeometryCache & cache, const DynamicGeometry & geometry,
  const moveit::core::RobotState & state, const int offset)
{
  NeighborhoodSample output;
  output.offset = offset;
  const EigenSTL::vector_Isometry3d handle_poses{
    desired_handle_pose(geometry, kDoorOpenAngle)};
  const std::vector<shapes::ShapeConstPtr> handle_shapes{geometry.handle_shape};
  for (const auto & link_name : kPhysicalLeftFinger2Links) {
    const auto * link = state.getRobotModel()->getLinkModel(link_name);
    if (!link || link->getShapes().empty()) {
      throw std::runtime_error("PHYSICAL_FINGER_LINK_GEOMETRY:" + link_name);
    }
    EigenSTL::vector_Isometry3d poses;
    poses.reserve(link->getShapes().size());
    for (std::size_t index = 0U; index < link->getShapes().size(); ++index) {
      poses.push_back(state.getCollisionBodyTransform(link, index));
    }
    const double distance = direct_body_distance(
      cache, handle_shapes, handle_poses, link->getShapes(), poses);
    if (distance < output.distance) {
      output.distance = distance;
      output.link = link_name;
    }
  }
  return output;
}

NeighborhoodSample direct_object_allowed_distance(
  FclGeometryCache & cache, const shapes::ShapeConstPtr & object_shape,
  const Eigen::Isometry3d & object_pose,
  const moveit::core::RobotState & state,
  const std::set<std::string> & allowed_links)
{
  NeighborhoodSample output;
  const std::vector<shapes::ShapeConstPtr> object_shapes{object_shape};
  const EigenSTL::vector_Isometry3d object_poses{object_pose};
  for (const auto & link_name : allowed_links) {
    const auto * link = state.getRobotModel()->getLinkModel(link_name);
    if (!link || link->getShapes().empty()) {
      throw std::runtime_error("CONTACT_LINK_GEOMETRY:" + link_name);
    }
    EigenSTL::vector_Isometry3d link_poses;
    link_poses.reserve(link->getShapes().size());
    for (std::size_t index = 0U; index < link->getShapes().size(); ++index) {
      link_poses.push_back(state.getCollisionBodyTransform(link, index));
    }
    const double distance = direct_body_distance(
      cache, object_shapes, object_poses, link->getShapes(), link_poses);
    if (distance < output.distance) {
      output.distance = distance;
      output.link = link_name;
    }
  }
  if (output.link.empty() || !std::isfinite(output.distance)) {
    throw std::runtime_error("CONTACT_ALLOWED_DISTANCE");
  }
  return output;
}

NeighborhoodSample exact_door_robot_distance(
  FclGeometryCache & cache, const DynamicGeometry & geometry,
  const moveit::core::RobotState & state, const double door_angle)
{
  NeighborhoodSample output;
  const EigenSTL::vector_Isometry3d door_poses{
    desired_door_pose(geometry, door_angle)};
  const std::vector<shapes::ShapeConstPtr> door_shapes{geometry.door_shape};
  for (const auto * link : state.getRobotModel()->getLinkModelsWithCollisionGeometry()) {
    EigenSTL::vector_Isometry3d poses;
    poses.reserve(link->getShapes().size());
    for (std::size_t index = 0U; index < link->getShapes().size(); ++index) {
      poses.push_back(state.getCollisionBodyTransform(link, index));
    }
    const double distance = direct_body_distance(
      cache, door_shapes, door_poses, link->getShapes(), poses);
    if (distance < output.distance) {
      output.distance = distance;
      output.link = link->getName();
    }
  }
  if (output.link.empty() || !std::isfinite(output.distance)) {
    throw std::runtime_error("EXACT_DOOR_ROBOT_GEOMETRY");
  }
  return output;
}

NeighborhoodSample moveit_world_object_robot_distance(
  planning_scene::PlanningScene & scene,
  const moveit::core::RobotState & state,
  const collision_detection::AllowedCollisionMatrix & base_acm,
  const std::vector<std::string> & collision_links,
  const std::string & object_id)
{
  collision_detection::AllowedCollisionMatrix matrix = base_acm;
  allow_all_self_distance_pairs(matrix, collision_links);
  for (const auto & world_id : scene.getWorld()->getObjectIds()) {
    for (const auto & link : collision_links) {
      matrix.setEntry(world_id, link, true);
    }
  }
  for (const auto & link : collision_links) {
    matrix.setEntry(object_id, link, false);
  }
  collision_detection::DistanceRequest request;
  request.type = collision_detection::DistanceRequestTypes::ALL;
  request.enable_nearest_points = true;
  request.enable_signed_distance = true;
  request.max_contacts_per_body = 1000U;
  request.distance_threshold = 2.0;
  request.acm = &matrix;
  collision_detection::DistanceResult result;
  scene.getCollisionEnvUnpadded()->distanceRobot(request, result, state);
  NeighborhoodSample output;
  for (const auto & pair : result.distances) {
    for (const auto & value : pair.second) {
      if ((value.link_names[0] == object_id ||
        value.link_names[1] == object_id) && value.distance < output.distance)
      {
        output.distance = value.distance;
        output.link = value.link_names[0] == object_id ?
          value.link_names[1] : value.link_names[0];
      }
    }
  }
  if (output.link.empty() || !std::isfinite(output.distance)) {
    throw std::runtime_error("WORLD_OBJECT_DISTANCE:" + object_id);
  }
  return output;
}

NeighborhoodSample specialized_object_robot_distance(
  planning_scene::PlanningScene & scene,
  const moveit::core::RobotState & state,
  const collision_detection::AllowedCollisionMatrix & matrix,
  const std::string & object_id)
{
  collision_detection::DistanceRequest request;
  request.type = collision_detection::DistanceRequestTypes::GLOBAL;
  request.enable_nearest_points = true;
  request.enable_signed_distance = true;
  request.max_contacts_per_body = 1U;
  request.distance_threshold = std::numeric_limits<double>::max();
  request.acm = &matrix;
  collision_detection::DistanceResult result;
  scene.getCollisionEnvUnpadded()->distanceRobot(request, result, state);
  NeighborhoodSample output;
  const auto & global = result.minimum_distance;
  if (std::isfinite(global.distance)) {
    output.distance = global.distance;
    output.link = global.link_names[0] + "|" + global.link_names[1];
  }
  for (const auto & pair : result.distances) {
    for (const auto & value : pair.second) {
      if ((value.link_names[0] == object_id ||
        value.link_names[1] == object_id) &&
        std::isfinite(value.distance) && value.distance < output.distance)
      {
        output.distance = value.distance;
        output.link = value.link_names[0] == object_id ?
          value.link_names[1] : value.link_names[0];
      }
    }
  }
  return output;
}

double visual_mask_distance(
  FclGeometryCache & cache, const DynamicGeometry & geometry,
  planning_scene::PlanningScene & raw_scene,
  const moveit::core::RobotState & physical_state)
{
  auto raw_state = raw_scene.getCurrentState();
  raw_state.setVariablePositions(physical_state.getVariablePositions());
  raw_state.updateCollisionBodyTransforms();
  const auto * finger_mask = raw_state.getAttachedBody(kLeftFingerMaskId);
  if (!finger_mask || finger_mask->getShapes().empty()) {
    throw std::runtime_error("LEFT_FINGER_VISUAL_MASK_ATTACHED_BODY");
  }
  EigenSTL::vector_Isometry3d handle_mask_poses;
  const auto handle_pose = desired_handle_pose(geometry, kDoorOpenAngle);
  for (const auto & relative : geometry.handle_mask_in_handle) {
    handle_mask_poses.push_back(handle_pose * relative);
  }
  return direct_body_distance(
    cache, geometry.handle_mask_shapes, handle_mask_poses,
    finger_mask->getShapes(), finger_mask->getGlobalCollisionBodyTransforms());
}

ExactAuditResult audit_dynamic(
  planning_scene::PlanningScene & physical_scene,
  planning_scene::PlanningScene & raw_scene,
  const std::map<AssetKind, AssetData> & assets,
  const bool mask_conflict_pair_present, const double park_roll,
  const bool static_only, const bool tail_scan, const bool diagnose_cube_door)
{
  ExactAuditResult output;
  auto geometry = prepare_dynamic_geometry(
    physical_scene, raw_scene, mask_conflict_pair_present, true);
  auto state = physical_scene.getCurrentState();
  const std::array<std::string, 7> left_arm_joints{
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint"};
  const std::array<double, 7> e0_left_park_arm{
    -0.000012933885986937658, park_roll, 0.000023774721497885907,
    0.6000753185207717, 0.000023572560034304773,
    -0.00008730492997967213, -0.00001654021997178485};
  const std::array<double, 7> e0_right_arm{
    -0.41373146194762, 0.23755434038212944, 1.1008080707922079,
    0.8747783439736734, -0.3046852067271319, 0.4094159191441438,
    -0.9999635680199944};
  for (std::size_t index = 0U; index < e0_left_park_arm.size(); ++index) {
    state.setVariablePosition(left_arm_joints[index], e0_left_park_arm[index]);
    state.setVariablePosition(kRightArmJoints[index], e0_right_arm[index]);
  }
  state.setVariablePosition("torso_joint", -0.04773321113053429);
  state.setVariablePosition("left_dex1_finger_1_joint", -0.019);
  state.setVariablePosition("left_dex1_finger_2_joint", -0.019);
  state.setVariablePosition("right_dex1_finger_1_joint", 0.0108);
  state.setVariablePosition("right_dex1_finger_2_joint", 0.0070);
  state.updateCollisionBodyTransforms();
  move_dynamic_geometry(
    physical_scene, raw_scene, geometry, 0.0, geometry.initial_cube_pose);
  verify_seam_state(state, park_roll);
  output.initial_left_shoulder_roll =
    state.getVariablePosition("left_shoulder_roll_joint");
  if (!state.satisfiesBounds(1e-6)) {
    throw std::runtime_error("INITIAL_JOINT_BOUNDS");
  }

  const auto collision_links =
    physical_scene.getRobotModel()->getLinkModelNamesWithCollisionGeometry();
  const std::set<std::string> collision_link_bodies(
    collision_links.begin(), collision_links.end());
  const auto world_id_vector = physical_scene.getWorld()->getObjectIds();
  std::set<std::string> environment_bodies(
    world_id_vector.begin(), world_id_vector.end());
  environment_bodies.insert("<octomap>");
  const std::set<std::string> table_bodies{"environment_table_10mm_margin"};
  const std::set<std::string> door_bodies{"incubator_door_exact"};
  const std::set<std::string> handle_bodies{kHandleId};
  const std::set<std::string> cube_bodies{
    "green_cube_dynamic_exact", "green_cube_10mm_guard"};
  const std::set<std::string> handle_body{kHandleId};
  collision_detection::AllowedCollisionMatrix base_acm =
    physical_scene.getAllowedCollisionMatrix();
  for (const auto & object : physical_scene.getWorld()->getObjectIds()) {
    for (const auto & link : collision_links) {
      base_acm.setEntry(object, link, false);
    }
  }
  base_acm.setEntry("right_elbow_link", "torso_link", false);
  collision_detection::AllowedCollisionMatrix right_nonpad_distance_acm =
    base_acm;
  allow_all_self_distance_pairs(right_nonpad_distance_acm, collision_links);
  for (const auto & object : physical_scene.getWorld()->getObjectIds()) {
    for (const auto & link : collision_links) {
      right_nonpad_distance_acm.setEntry(object, link, true);
    }
  }
  std::size_t right_nonpad_link_count = 0U;
  for (const auto & link : collision_links) {
    if (link.rfind("right_dex1_", 0U) == 0U &&
      link != "right_dex1_finger_1_pad_link" &&
      link != "right_dex1_finger_2_pad_link")
    {
      right_nonpad_distance_acm.setEntry(kHandleId, link, false);
      ++right_nonpad_link_count;
    }
  }
  if (right_nonpad_link_count == 0U) {
    throw std::runtime_error("RIGHT_HANDLE_NONPAD_LINK_SET");
  }
  const auto right_object_matrix = [&](const std::string & object_id) {
      collision_detection::AllowedCollisionMatrix matrix = base_acm;
      allow_all_self_distance_pairs(matrix, collision_links);
      for (const auto & object : physical_scene.getWorld()->getObjectIds()) {
        for (const auto & link : collision_links) {
          matrix.setEntry(object, link, true);
        }
      }
      std::size_t selected = 0U;
      for (const auto & link : collision_links) {
        if (link.rfind("right_", 0U) == 0U) {
          matrix.setEntry(object_id, link, false);
          ++selected;
        }
      }
      if (selected == 0U) {
        throw std::runtime_error("RIGHT_COLLISION_LINK_SET");
      }
      return matrix;
    };
  const auto cube_exact_right_acm =
    right_object_matrix("green_cube_dynamic_exact");
  const auto cube_guard_right_acm =
    right_object_matrix("green_cube_10mm_guard");

  collision_detection::CollisionRequest self_request;
  self_request.contacts = true;
  self_request.max_contacts = 32U;
  self_request.max_contacts_per_pair = 4U;
  self_request.pad_self_collisions = false;
  collision_detection::CollisionRequest world_request = self_request;
  world_request.pad_environment_collisions = false;

  if (diagnose_cube_door) {
    const auto & retreat = assets.at(AssetKind::Cube).messages.at(5U);
    FclGeometryCache cache;
    move_dynamic_geometry(
      physical_scene, raw_scene, geometry, kDoorOpenAngle,
      geometry.target_cube_pose);
    for (std::size_t point_index = 88U; point_index <= 105U; ++point_index) {
      moveit::core::RobotState start(state);
      if (point_index != 0U) {
        start.setVariablePositions(
          retreat.joint_names, retreat.positions.at(point_index - 1U));
      }
      start.updateCollisionBodyTransforms();
      moveit::core::RobotState next(start);
      next.setVariablePositions(
        retreat.joint_names, retreat.positions.at(point_index));
      next.updateCollisionBodyTransforms();
      const std::size_t subdivisions = subdivision_count(
        physical_scene.getRobotModel(), start, next, retreat.joint_names,
        kDoorOpenAngle, kDoorOpenAngle);
      for (std::size_t subdivision = 1U;
        subdivision <= subdivisions; ++subdivision)
      {
        moveit::core::RobotState sample(start);
        start.interpolate(
          next, static_cast<double>(subdivision) /
          static_cast<double>(subdivisions), sample);
        sample.updateCollisionBodyTransforms();
        const auto baseline = exact_door_robot_distance(
          cache, geometry, sample, kDoorOpenAngle);
        std::cerr << std::setprecision(17) <<
          "RETREAT_DOOR_DERIVATIVE\tpoint=" << point_index <<
          "\tsubdivision=" << subdivision <<
          "\tsubdivisions=" << subdivisions <<
          "\tdelta_deg=0\tdoor_m=" << baseline.distance <<
          "\tlink=" << baseline.link << "\ttcp_delta_m=0\n";
        if (point_index >= 96U && point_index <= 104U) {
          const Eigen::Isometry3d baseline_tcp =
            sample.getGlobalLinkTransform("left_dex1_tcp");
          for (const double delta_deg : {0.60, 0.62, 0.65, 0.67}) {
            moveit::core::RobotState perturbed(sample);
            perturbed.setVariablePosition(
              "left_shoulder_roll_joint",
              perturbed.getVariablePosition("left_shoulder_roll_joint") +
              delta_deg * kPi / 180.0);
            perturbed.updateCollisionBodyTransforms();
            const auto distance = exact_door_robot_distance(
              cache, geometry, perturbed, kDoorOpenAngle);
            const double tcp_delta =
              (perturbed.getGlobalLinkTransform("left_dex1_tcp").translation() -
              baseline_tcp.translation()).norm();
            const double tcp_rotation = rotation_distance(
              baseline_tcp,
              perturbed.getGlobalLinkTransform("left_dex1_tcp"));
            std::cerr << std::setprecision(17) <<
              "RETREAT_DOOR_DERIVATIVE\tpoint=" << point_index <<
              "\tsubdivision=" << subdivision <<
              "\tsubdivisions=" << subdivisions <<
              "\tdelta_deg=" << delta_deg <<
              "\tdoor_m=" << distance.distance <<
              "\tlink=" << distance.link <<
              "\ttcp_delta_m=" << tcp_delta <<
              "\ttcp_rotation_rad=" << tcp_rotation << "\n";
          }
        }
      }
    }
    throw std::runtime_error("RETREAT_DOOR_DERIVATIVE_COMPLETE");
  }

  output.initial_park_door_clearance = moveit_world_object_robot_distance(
    physical_scene, state, base_acm, collision_links, "incubator_door_exact");
  if (output.initial_park_door_clearance.distance + 1e-12 <
    kParkStaticClearance)
  {
    throw std::runtime_error(
            "INITIAL_PARK_DOOR_CLEARANCE:" +
            output.initial_park_door_clearance.link + ":" +
            std::to_string(output.initial_park_door_clearance.distance));
  }
  if (static_only) {
    collision_detection::CollisionResult static_self_collision;
    physical_scene.checkSelfCollision(
      self_request, static_self_collision, state, base_acm);
    collision_detection::CollisionResult static_world_collision;
    physical_scene.getCollisionEnvUnpadded()->checkRobotCollision(
      world_request, static_world_collision, state, base_acm);
    if (static_self_collision.collision || static_world_collision.collision) {
      throw std::runtime_error(
              static_self_collision.collision ? "STATIC_SELF_COLLISION" :
              "STATIC_WORLD_COLLISION");
    }
    collision_detection::AllowedCollisionMatrix static_world_distance_acm =
      base_acm;
    allow_all_self_distance_pairs(static_world_distance_acm, collision_links);
    collision_detection::DistanceRequest static_world_request;
    static_world_request.type = collision_detection::DistanceRequestTypes::ALL;
    static_world_request.enable_nearest_points = true;
    static_world_request.enable_signed_distance = true;
    static_world_request.max_contacts_per_body = 1000U;
    static_world_request.distance_threshold = 0.2;
    static_world_request.acm = &static_world_distance_acm;
    collision_detection::DistanceResult static_world_distance;
    physical_scene.getCollisionEnvUnpadded()->distanceRobot(
      static_world_request, static_world_distance, state);
    collision_detection::DistanceRequest static_self_request = static_world_request;
    static_self_request.acm = &base_acm;
    collision_detection::DistanceResult static_self_distance;
    physical_scene.getCollisionEnvUnpadded()->distanceSelf(
      static_self_request, static_self_distance, state);
    std::ostringstream status;
    status << std::setprecision(17) << "STATIC_ONLY:park_roll=" << park_roll <<
      ":door=" << output.initial_park_door_clearance.distance <<
      ":door_link=" << output.initial_park_door_clearance.link <<
      ":table=" << selected_environment_minimum(
        static_world_distance, table_bodies) <<
      ":handle=" << selected_environment_minimum(
        static_world_distance, handle_bodies) <<
      ":cube=" << selected_environment_minimum(
        static_world_distance, cube_bodies) <<
      ":self=" << selected_environment_minimum(
        static_self_distance, collision_link_bodies) <<
      ":collision_free=true";
    throw std::runtime_error(status.str());
  }
  std::optional<moveit::core::RobotState> cube_witness_start;
  bool cube_bump_audited = false;
  double current_door_angle = 0.0;
  bool tail_sensitivity_done = false;
  FclGeometryCache tail_fcl_cache;
  FclGeometryCache diagnostic_cube_door_cache;
  FclGeometryCache authorized_contact_cache;
  const std::set<std::string> right_handle_pad_links(
    kRightHandlePads.begin(), kRightHandlePads.end());
  const std::set<std::string> left_cube_touch_links(
    kLeftCubeTouchLinks.begin(), kLeftCubeTouchLinks.end());
  const std::set<std::string> release_retreat_links(
    kLeftCubeReleaseRetreatLinks.begin(), kLeftCubeReleaseRetreatLinks.end());
  double release_retreat_previous_projection = 0.0;
  std::optional<Eigen::Vector3d> release_retreat_tcp_origin;
  std::optional<Eigen::Vector3d> release_retreat_direction;
  std::optional<std::size_t> release_retreat_last_below_gate_ordinal;
  std::size_t release_retreat_gate_ordinal = 0U;
  std::size_t release_retreat_current_ordinal = 0U;

  for (const auto & phase : kPhases) {
    if (phase.asset == AssetKind::Cube && phase.start == 0U) {
      if (cube_bump_audited) {
        throw std::runtime_error("CUBE_BUMP_DUPLICATE_AUDIT");
      }
      cube_witness_start = state;
      audit_cube_bump_against_reference(
        physical_scene.getRobotModel(), state, assets.at(AssetKind::Cube),
        assets.at(AssetKind::CubeReference), output);
      cube_bump_audited = true;
    }
    PhaseMetric phase_metric;
    phase_metric.name = phase.name;
    output.phases.push_back(std::move(phase_metric));
    auto & metric = output.phases.back();
    const auto & asset = assets.at(phase.asset);
    for (std::size_t message_index = phase.start;
      message_index < phase.end; ++message_index)
    {
      const auto & message = asset.messages.at(message_index);
      if (phase.asset == AssetKind::Cube && message_index == 2U &&
        !geometry.cube_attachment_captured)
      {
        geometry.tcp_to_cube =
          state.getGlobalLinkTransform("left_dex1_tcp").inverse() *
          geometry.initial_cube_pose;
        geometry.cube_attachment_captured = true;
      }
      if (phase.asset == AssetKind::Cube && message_index == 4U) {
        const auto carried = state.getGlobalLinkTransform("left_dex1_tcp") *
          geometry.tcp_to_cube;
        output.final_target_translation_error =
          (carried.translation() - geometry.target_cube_pose.translation()).norm();
        output.final_target_rotation_error =
          rotation_distance(carried, geometry.target_cube_pose);
        if (output.final_target_translation_error > 0.003 ||
          output.final_target_rotation_error > 2.0 * kPi / 180.0)
        {
          throw std::runtime_error("CUBE_CARRY_TARGET_POSE");
        }
      }
      if (phase.asset == AssetKind::Cube && message_index == 5U) {
        if (message.time_from_start_ns.size() < 2U) {
          throw std::runtime_error("CUBE_RELEASE_RETREAT_TIME_GRID");
        }
        if (message.time_from_start_ns.front() > 0) {
          output.cube_release_retreat_minimum_time_step_ns = std::min(
            output.cube_release_retreat_minimum_time_step_ns,
            message.time_from_start_ns.front());
        }
        for (std::size_t time_index = 1U;
          time_index < message.time_from_start_ns.size(); ++time_index)
        {
          const auto time_step = message.time_from_start_ns[time_index] -
            message.time_from_start_ns[time_index - 1U];
          if (time_step <= 0) {
            throw std::runtime_error("CUBE_RELEASE_RETREAT_TIME_STEP");
          }
          output.cube_release_retreat_minimum_time_step_ns = std::min(
            output.cube_release_retreat_minimum_time_step_ns, time_step);
        }
        const Eigen::Vector3d release_tcp =
          state.getGlobalLinkTransform("left_dex1_tcp").translation();
        moveit::core::RobotState release_end(state);
        release_end.setVariablePositions(
          message.joint_names, message.positions.back());
        release_end.updateCollisionBodyTransforms();
        const Eigen::Vector3d retreat_chord =
          release_end.getGlobalLinkTransform("left_dex1_tcp").translation() -
          release_tcp;
        if (retreat_chord.norm() < 1e-9) {
          throw std::runtime_error("CUBE_RELEASE_RETREAT_ZERO_TCP_CHORD");
        }
        release_retreat_tcp_origin = release_tcp;
        release_retreat_direction = retreat_chord.normalized();
        if (release_retreat_direction->dot(
            release_tcp - geometry.target_cube_pose.translation()) <= 0.0)
        {
          throw std::runtime_error("CUBE_RELEASE_RETREAT_TCP_DIRECTION");
        }
        const auto boundary_exact = moveit_world_object_robot_distance(
          physical_scene, state, base_acm, collision_links,
          "green_cube_dynamic_exact");
        const auto boundary_guard = moveit_world_object_robot_distance(
          physical_scene, state, base_acm, collision_links,
          "green_cube_10mm_guard");
        if (boundary_exact.distance < 0.0) {
          throw std::runtime_error("CUBE_RELEASE_RETREAT_BOUNDARY");
        }
        output.cube_release_retreat_minimum_exact_distance = std::min(
          output.cube_release_retreat_minimum_exact_distance,
          boundary_exact.distance);
        output.cube_release_retreat_maximum_exact_distance = std::max(
          output.cube_release_retreat_maximum_exact_distance,
          boundary_exact.distance);
        output.cube_release_retreat_final_exact_distance = boundary_exact.distance;

        // Pass one locates the final sample that still needs the conservative
        // 10 mm cube-guard exception.  Pass two below grants the exception to
        // exactly that shortest prefix; the immediately following sample and
        // every later sample are checked with the unmodified ACM.
        std::vector<std::pair<std::size_t, std::size_t>> sample_coordinates;
        sample_coordinates.emplace_back(0U, 0U);  // detached release boundary
        const auto below_gate = [](const NeighborhoodSample & exact,
            const NeighborhoodSample & guard) {
            return exact.distance + 1e-12 < kDistanceBand ||
                   guard.distance + kRequiredClearance + 1e-12 < kDistanceBand;
          };
        if (below_gate(boundary_exact, boundary_guard)) {
          if (!release_retreat_links.count(boundary_exact.link) ||
            !release_retreat_links.count(boundary_guard.link))
          {
            throw std::runtime_error("CUBE_RELEASE_RETREAT_BOUNDARY_LINK");
          }
          release_retreat_last_below_gate_ordinal = 0U;
        }
        moveit::core::RobotState scan_state(state);
        double scan_door_angle = current_door_angle;
        std::size_t scan_ordinal = 0U;
        for (std::size_t scan_point = 0U;
          scan_point < message.positions.size(); ++scan_point)
        {
          moveit::core::RobotState scan_next(scan_state);
          scan_next.setVariablePositions(
            message.joint_names, message.positions[scan_point]);
          scan_next.updateCollisionBodyTransforms();
          const double scan_target_door = message_door_angle(
            phase.asset, message_index, scan_point, message.positions.size());
          const std::size_t scan_subdivisions = subdivision_count(
            physical_scene.getRobotModel(), scan_state, scan_next,
            message.joint_names, scan_door_angle, scan_target_door);
          for (std::size_t scan_subdivision = 1U;
            scan_subdivision <= scan_subdivisions; ++scan_subdivision)
          {
            const double scan_fraction =
              static_cast<double>(scan_subdivision) /
              static_cast<double>(scan_subdivisions);
            moveit::core::RobotState scan_sample(scan_state);
            scan_state.interpolate(scan_next, scan_fraction, scan_sample);
            scan_sample.updateCollisionBodyTransforms();
            const double sample_door_angle = scan_door_angle +
              (scan_target_door - scan_door_angle) * scan_fraction;
            move_dynamic_geometry(
              physical_scene, raw_scene, geometry, sample_door_angle,
              geometry.target_cube_pose);
            const auto exact = moveit_world_object_robot_distance(
              physical_scene, scan_sample, base_acm, collision_links,
              "green_cube_dynamic_exact");
            const auto guard = moveit_world_object_robot_distance(
              physical_scene, scan_sample, base_acm, collision_links,
              "green_cube_10mm_guard");
            ++scan_ordinal;
            sample_coordinates.emplace_back(scan_point, scan_subdivision);
            if (exact.distance < 0.0) {
              throw std::runtime_error(
                      "CUBE_RELEASE_RETREAT_PRESCAN_EXACT_PENETRATION");
            }
            if (below_gate(exact, guard)) {
              if (!release_retreat_links.count(exact.link) ||
                !release_retreat_links.count(guard.link))
              {
                throw std::runtime_error(
                        "CUBE_RELEASE_RETREAT_PRESCAN_LINK_WHITELIST:" +
                        exact.link + ":" + guard.link);
              }
              release_retreat_last_below_gate_ordinal = scan_ordinal;
            }
          }
          scan_state = scan_next;
          scan_door_angle = scan_target_door;
        }
        move_dynamic_geometry(
          physical_scene, raw_scene, geometry, current_door_angle,
          geometry.target_cube_pose);
        release_retreat_gate_ordinal =
          release_retreat_last_below_gate_ordinal ?
          *release_retreat_last_below_gate_ordinal + 1U : 0U;
        if (release_retreat_gate_ordinal >= sample_coordinates.size()) {
          throw std::runtime_error("CUBE_RELEASE_RETREAT_NO_STRICT_SUFFIX");
        }
        output.cube_release_retreat_gate_ordinal = release_retreat_gate_ordinal;
        output.cube_release_retreat_last_below_gate_ordinal =
          release_retreat_last_below_gate_ordinal.value_or(0U);
        output.cube_release_retreat_gate_point =
          sample_coordinates[release_retreat_gate_ordinal].first;
        output.cube_release_retreat_gate_subdivision =
          sample_coordinates[release_retreat_gate_ordinal].second;
        if (release_retreat_gate_ordinal == 0U) {
          output.cube_release_retreat_gate_reached = true;
          output.cube_release_retreat_post_gate_minimum_exact_distance =
            boundary_exact.distance;
          output.cube_release_retreat_post_gate_minimum_guard_physical_lower_bound =
            boundary_guard.distance + kRequiredClearance;
          if (below_gate(boundary_exact, boundary_guard)) {
            throw std::runtime_error("CUBE_RELEASE_RETREAT_BOUNDARY_GATE");
          }
        } else {
          if (below_gate(boundary_exact, boundary_guard)) {
            output.cube_release_retreat_observed_exact_links.insert(
              boundary_exact.link);
            output.cube_release_retreat_observed_guard_links.insert(
              boundary_guard.link);
            output.cube_release_retreat_observed_exact_pairs.emplace(
              "green_cube_dynamic_exact", boundary_exact.link);
            output.cube_release_retreat_observed_guard_pairs.emplace(
              "green_cube_10mm_guard", boundary_guard.link);
          }
          ++output.cube_release_retreat_window_samples;
        }
      }
      for (std::size_t point_index = 0U;
        point_index < message.positions.size(); ++point_index)
      {
        moveit::core::RobotState next(state);
        next.setVariablePositions(message.joint_names, message.positions[point_index]);
        next.updateCollisionBodyTransforms();
        if (!next.satisfiesBounds(1e-6)) {
          throw std::runtime_error("JOINT_BOUNDS:" + std::string(phase.name));
        }
        const double target_door_angle = message_door_angle(
          phase.asset, message_index, point_index, message.positions.size());
        const std::size_t subdivisions = subdivision_count(
          physical_scene.getRobotModel(), state, next, message.joint_names,
          current_door_angle, target_door_angle);
        if (phase.asset == AssetKind::Cube && message_index == 5U) {
          output.cube_release_retreat_minimum_path_parameter_step = std::min(
            output.cube_release_retreat_minimum_path_parameter_step,
            1.0 / static_cast<double>(subdivisions));
        }
        for (std::size_t subdivision = 1U;
          subdivision <= subdivisions; ++subdivision)
        {
          const double fraction = static_cast<double>(subdivision) /
            static_cast<double>(subdivisions);
          moveit::core::RobotState sample(state);
          state.interpolate(next, fraction, sample);
          sample.updateCollisionBodyTransforms();
          const double door_angle = current_door_angle +
            (target_door_angle - current_door_angle) * fraction;
          if (door_angle < -1e-12 || door_angle > kDoorOpenAngle + 1e-12) {
            throw std::runtime_error("DOOR_ANGLE_RANGE");
          }
          const auto cube_pose = cube_pose_for_sample(
            geometry, sample, phase.asset, message_index);
          move_dynamic_geometry(
            physical_scene, raw_scene, geometry, door_angle, cube_pose);

          const bool cube_release_retreat_sample =
            phase.asset == AssetKind::Cube && message_index == 5U;
          bool cube_release_retreat_authorized_sample = false;
          if (cube_release_retreat_sample) {
            const auto exact = moveit_world_object_robot_distance(
              physical_scene, sample, base_acm, collision_links,
              "green_cube_dynamic_exact");
            const auto guard = moveit_world_object_robot_distance(
              physical_scene, sample, base_acm, collision_links,
              "green_cube_10mm_guard");
            if (!release_retreat_tcp_origin || !release_retreat_direction) {
              throw std::runtime_error("CUBE_RELEASE_RETREAT_TCP_PROJECTION_STATE");
            }
            const double projection =
              (sample.getGlobalLinkTransform("left_dex1_tcp").translation() -
              *release_retreat_tcp_origin).dot(*release_retreat_direction);
            const double projection_step =
              projection - release_retreat_previous_projection;
            // Projection is diagnostic only: the collision geometry can rotate
            // around the fixed cube during a valid retreat.  Exact penetration
            // remains forbidden at every sample.
            if (exact.distance < 0.0) {
              throw std::runtime_error("CUBE_RELEASE_RETREAT_EXACT_PENETRATION");
            }
            release_retreat_previous_projection = projection;
            output.cube_release_retreat_minimum_exact_distance = std::min(
              output.cube_release_retreat_minimum_exact_distance, exact.distance);
            output.cube_release_retreat_maximum_exact_distance = std::max(
              output.cube_release_retreat_maximum_exact_distance, exact.distance);
            output.cube_release_retreat_final_exact_distance = exact.distance;
            output.cube_release_retreat_maximum_exact_penetration = std::max(
              output.cube_release_retreat_maximum_exact_penetration,
              std::max(0.0, -exact.distance));
            output.cube_release_retreat_minimum_projection_step = std::min(
              output.cube_release_retreat_minimum_projection_step,
              projection_step);
            output.cube_release_retreat_final_projection = projection;
            ++release_retreat_current_ordinal;
            cube_release_retreat_authorized_sample =
              release_retreat_last_below_gate_ordinal &&
              release_retreat_current_ordinal <=
              *release_retreat_last_below_gate_ordinal;
            if (cube_release_retreat_authorized_sample) {
              ++output.cube_release_retreat_window_samples;
              if (exact.distance + 1e-12 < kDistanceBand ||
                guard.distance + kRequiredClearance + 1e-12 < kDistanceBand)
              {
                if (!release_retreat_links.count(exact.link) ||
                  !release_retreat_links.count(guard.link))
                {
                  throw std::runtime_error(
                          "CUBE_RELEASE_RETREAT_LINK_WHITELIST:" + exact.link +
                          ":" + guard.link);
                }
                output.cube_release_retreat_observed_exact_links.insert(
                  exact.link);
                output.cube_release_retreat_observed_guard_links.insert(
                  guard.link);
                output.cube_release_retreat_observed_exact_pairs.emplace(
                  "green_cube_dynamic_exact", exact.link);
                output.cube_release_retreat_observed_guard_pairs.emplace(
                  "green_cube_10mm_guard", guard.link);
              }
            } else {
              if (release_retreat_current_ordinal == release_retreat_gate_ordinal) {
                output.cube_release_retreat_gate_reached = true;
              }
              output.cube_release_retreat_post_gate_minimum_exact_distance =
                std::min(
                output.cube_release_retreat_post_gate_minimum_exact_distance,
                exact.distance);
              output.cube_release_retreat_post_gate_minimum_guard_physical_lower_bound =
                std::min(
                output.cube_release_retreat_post_gate_minimum_guard_physical_lower_bound,
                guard.distance + kRequiredClearance);
              if (exact.distance + 1e-12 < kDistanceBand ||
                guard.distance + kRequiredClearance + 1e-12 < kDistanceBand)
              {
                throw std::runtime_error("CUBE_RELEASE_RETREAT_GATE_REGRESSION");
              }
            }
          }

          if (diagnose_cube_door) {
            if (phase.asset == AssetKind::Cube && message_index == 2U &&
              point_index >= 220U && point_index <= 340U)
            {
              const auto door = exact_door_robot_distance(
                diagnostic_cube_door_cache, geometry, sample, door_angle);
              metric.minimum_door_clearance = std::min(
                metric.minimum_door_clearance, door.distance);
              output.minimum_door_clearance = std::min(
                output.minimum_door_clearance, door.distance);
              ++metric.samples;
              ++output.samples;
              std::cerr << std::setprecision(17) <<
                "CUBE_DOOR_FAST_DIAGNOSTIC\tmessage=" << message_index <<
                "\tpoint=" << point_index <<
                "\tsubdivision=" << subdivision <<
                "\tdoor_m=" << door.distance <<
                "\tlink=" << door.link << '\n';
            }
            continue;
          }

          // Tail diagnostics reconstruct the exact asset state but intentionally
          // avoid spending time auditing the already-known prefix.
          if (tail_scan && phase.asset == AssetKind::DoorOpen &&
            (message_index < 1U ||
            (message_index == 1U && point_index < 760U)))
          {
            continue;
          }

          collision_detection::AllowedCollisionMatrix collision_acm = base_acm;
          add_authorized_pairs(collision_acm, phase.asset, message_index);
          if (cube_release_retreat_sample &&
            !cube_release_retreat_authorized_sample)
          {
            // Close only the cube-specific prefix window.  The independent
            // right-pad/handle authorization must remain active while the
            // right hand continues to support the open door.
            for (const auto & link : kLeftCubeReleaseRetreatLinks) {
              collision_acm.setEntry("green_cube_dynamic_exact", link, false);
              collision_acm.setEntry("green_cube_10mm_guard", link, false);
            }
          }
          const bool right_handle_collision_authorized =
            right_handle_contact_authorized(phase.asset, message_index);
          const bool right_handle_distance_active =
            right_handle_distance_window(phase.asset, message_index);
          const bool intentional_cube_hand_contact =
            phase.asset == AssetKind::Cube &&
            (message_index <= 4U ||
            (message_index == 5U && cube_release_retreat_authorized_sample));
          if (right_handle_collision_authorized) {
            ++metric.right_handle_authorized_samples;
            ++output.right_handle_window_samples;
            const auto handle_contact = direct_object_allowed_distance(
              authorized_contact_cache, geometry.handle_shape,
              desired_handle_pose(geometry, door_angle), sample,
              right_handle_pad_links);
            output.right_handle_maximum_observed_penetration = std::max(
              output.right_handle_maximum_observed_penetration,
              std::max(0.0, -handle_contact.distance));
            output.right_handle_observed_pairs.emplace(
              kHandleId, handle_contact.link);
          }
          if (phase.asset == AssetKind::Cube && message_index <= 4U) {
            ++output.cube_grasp_carry_window_samples;
            const auto exact_contact = direct_object_allowed_distance(
              authorized_contact_cache, geometry.cube_shape, cube_pose,
              sample, left_cube_touch_links);
            const auto guard_contact = direct_object_allowed_distance(
              authorized_contact_cache, geometry.cube_guard_shape,
              cube_pose * geometry.cube_guard_in_cube, sample,
              left_cube_touch_links);
            output.cube_grasp_carry_maximum_observed_penetration = std::max(
              output.cube_grasp_carry_maximum_observed_penetration,
              std::max(0.0, -exact_contact.distance));
            output.cube_grasp_carry_observed_exact_pairs.emplace(
              "green_cube_dynamic_exact", exact_contact.link);
            output.cube_grasp_carry_observed_guard_pairs.emplace(
              "green_cube_10mm_guard", guard_contact.link);
            output.cube_grasp_carry_minimum_guard_physical_lower_bound =
              std::min(
              output.cube_grasp_carry_minimum_guard_physical_lower_bound,
              guard_contact.distance + kRequiredClearance);
          }
          collision_detection::CollisionResult self_result;
          physical_scene.checkSelfCollision(
            self_request, self_result, sample, collision_acm);
          if (self_result.collision) {
            const auto pair = self_result.contacts.empty() ?
              std::make_pair(std::string("unknown"), std::string("unknown")) :
              self_result.contacts.begin()->first;
            throw std::runtime_error(
                    "COLLISION_SELF:" + std::string(phase.name) + ":" +
                    pair.first + ":" + pair.second + ":message=" +
                    std::to_string(message_index) + ":point=" +
                    std::to_string(point_index) + ":subdivision=" +
                    std::to_string(subdivision));
          }
          collision_detection::CollisionResult world_result;
          physical_scene.getCollisionEnvUnpadded()->checkRobotCollision(
            world_request, world_result, sample, collision_acm);
          if (world_result.collision) {
            bool guard_only = !world_result.contacts.empty();
            for (const auto & contact : world_result.contacts) {
              guard_only = guard_only &&
                (contact.first.first == "green_cube_10mm_guard" ||
                contact.first.second == "green_cube_10mm_guard");
            }
            const auto exact_cube = moveit_world_object_robot_distance(
              physical_scene, sample, base_acm, collision_links,
              "green_cube_dynamic_exact");
            if (guard_only && !intentional_cube_hand_contact &&
              exact_cube.distance + 1e-12 >= kRequiredClearance)
            {
              ++output.cube_guard_exact_refinement_samples;
              output.minimum_cube_exact_noncontact_clearance = std::min(
                output.minimum_cube_exact_noncontact_clearance,
                exact_cube.distance);
              world_result.clear();
            }
          }
          if (world_result.collision) {
            const auto pair = world_result.contacts.empty() ?
              std::make_pair(std::string("unknown"), std::string("unknown")) :
              world_result.contacts.begin()->first;
            const double depth = world_result.contacts.empty() ?
              std::numeric_limits<double>::quiet_NaN() :
              world_result.contacts.begin()->second.front().depth;
            const auto cube_exact = moveit_world_object_robot_distance(
              physical_scene, sample, base_acm, collision_links,
              "green_cube_dynamic_exact");
            const auto cube_guard = moveit_world_object_robot_distance(
              physical_scene, sample, base_acm, collision_links,
              "green_cube_10mm_guard");
            std::ostringstream detail;
            detail << std::setprecision(17) <<
              ":depth=" << depth <<
              ":cube_exact_link=" << cube_exact.link <<
              ":cube_exact_m=" << cube_exact.distance <<
              ":cube_guard_link=" << cube_guard.link <<
              ":cube_guard_m=" << cube_guard.distance;
            throw std::runtime_error(
                    "COLLISION_WORLD:" + std::string(phase.name) + ":" +
                    pair.first + ":" + pair.second + ":message=" +
                    std::to_string(message_index) + ":point=" +
                    std::to_string(point_index) + ":subdivision=" +
                    std::to_string(subdivision) + detail.str());
          }

          if (right_handle_distance_active) {
            collision_detection::DistanceRequest nonpad_request;
            nonpad_request.type = collision_detection::DistanceRequestTypes::SINGLE;
            nonpad_request.enable_nearest_points = true;
            nonpad_request.enable_signed_distance = true;
            nonpad_request.max_contacts_per_body = 1U;
            nonpad_request.distance_threshold = 2.0;
            nonpad_request.acm = &right_nonpad_distance_acm;
            collision_detection::DistanceResult nonpad_result;
            physical_scene.getCollisionEnvUnpadded()->distanceRobot(
              nonpad_request, nonpad_result, sample);
            NeighborhoodSample nonpad;
            update_minimum_from_distance(
              nonpad_result, handle_body, nonpad.distance);
            nonpad.link = closest_distance_pair(nonpad_result, handle_body);
            metric.minimum_right_handle_nonpad_clearance = std::min(
              metric.minimum_right_handle_nonpad_clearance, nonpad.distance);
            output.minimum_right_handle_nonpad_clearance = std::min(
              output.minimum_right_handle_nonpad_clearance, nonpad.distance);
            if (nonpad.distance + 1e-12 < kRightHandleNonpadClearance) {
              throw std::runtime_error(
                      "RIGHT_HANDLE_NONPAD_CLEARANCE:" +
                      std::string(phase.name) + ":" + nonpad.link + ":" +
                      std::to_string(nonpad.distance));
            }
          }

          const bool in_tail_scan = tail_scan &&
            phase.asset == AssetKind::DoorOpen && message_index == 1U &&
            point_index >= 760U && point_index <= 805U;
          collision_detection::AllowedCollisionMatrix distance_acm = collision_acm;
          allow_all_self_distance_pairs(distance_acm, collision_links);
          if (right_handle_distance_active) {
            for (const auto & link : collision_links) {
              if (link.rfind("right_dex1_", 0U) == 0U) {
                distance_acm.setEntry(kHandleId, link, true);
              }
            }
          }
          collision_detection::DistanceRequest distance_request;
          distance_request.type = collision_detection::DistanceRequestTypes::ALL;
          distance_request.enable_nearest_points = true;
          distance_request.enable_signed_distance = true;
          distance_request.max_contacts_per_body = 1U;
          distance_request.distance_threshold = kDistanceBand;
          distance_request.acm = &distance_acm;
          collision_detection::DistanceResult distance_result;
          physical_scene.getCollisionEnvUnpadded()->distanceRobot(
            distance_request, distance_result, sample);
          collision_detection::DistanceRequest self_distance_request =
            distance_request;
          self_distance_request.acm = &collision_acm;
          collision_detection::DistanceResult self_distance_result;
          physical_scene.getCollisionEnvUnpadded()->distanceSelf(
            self_distance_request, self_distance_result, sample);
          double sample_environment_clearance = kDistanceBand;
          double sample_table_clearance = kDistanceBand;
          double sample_door_clearance = kDistanceBand;
          double sample_handle_clearance = kDistanceBand;
          double sample_cube_clearance = kDistanceBand;
          double sample_self_clearance = kDistanceBand;
          update_environment_minimum(
            distance_result, environment_bodies,
            sample_environment_clearance);
          update_environment_minimum(
            distance_result, table_bodies, sample_table_clearance);
          update_environment_minimum(
            distance_result, door_bodies, sample_door_clearance);
          update_environment_minimum(
            distance_result, handle_bodies, sample_handle_clearance);
          update_environment_minimum(
            distance_result, cube_bodies, sample_cube_clearance);
          update_minimum_from_distance(
            self_distance_result, collision_link_bodies,
            sample_self_clearance);
          if (sample_environment_clearance + 1e-12 < kRequiredClearance &&
            closest_distance_pair(
              distance_result, environment_bodies).find(
              "green_cube_10mm_guard") != std::string::npos)
          {
            collision_detection::AllowedCollisionMatrix refined_acm =
              distance_acm;
            for (const auto & link : collision_links) {
              refined_acm.setEntry("green_cube_10mm_guard", link, true);
            }
            collision_detection::DistanceRequest refined_request = distance_request;
            refined_request.acm = &refined_acm;
            collision_detection::DistanceResult refined_result;
            physical_scene.getCollisionEnvUnpadded()->distanceRobot(
              refined_request, refined_result, sample);
            sample_environment_clearance = kDistanceBand;
            sample_cube_clearance = kDistanceBand;
            update_environment_minimum(
              refined_result, environment_bodies,
              sample_environment_clearance);
            update_environment_minimum(
              refined_result, cube_bodies, sample_cube_clearance);
            if (!intentional_cube_hand_contact) {
              const auto exact_cube = moveit_world_object_robot_distance(
                physical_scene, sample, base_acm, collision_links,
                "green_cube_dynamic_exact");
              ++output.cube_guard_exact_refinement_samples;
              output.minimum_cube_exact_noncontact_clearance = std::min(
                output.minimum_cube_exact_noncontact_clearance,
                exact_cube.distance);
              sample_cube_clearance = std::min(
                sample_cube_clearance, exact_cube.distance);
              sample_environment_clearance = std::min(
                sample_environment_clearance, exact_cube.distance);
            }
          }
          if (in_tail_scan) {
            const auto exact = specialized_object_robot_distance(
              physical_scene, sample, cube_exact_right_acm,
              "green_cube_dynamic_exact");
            const auto guard = specialized_object_robot_distance(
              physical_scene, sample, cube_guard_right_acm,
              "green_cube_10mm_guard");
            std::cerr << std::setprecision(17) <<
              "TAIL_SCAN\tpoint=" << point_index <<
              "\tsubdivision=" << subdivision <<
              "\tsubdivisions=" << subdivisions <<
              "\texact_link=" << exact.link <<
              "\texact_m=" << exact.distance <<
              "\tguard_link=" << guard.link <<
              "\tguard_raw_m=" << guard.distance <<
              "\tguard_physical_lower_m=" <<
              (guard.distance + kRequiredClearance) <<
              "\tenvironment_m=" << sample_environment_clearance << '\n';

            if (!tail_sensitivity_done && exact.distance + 1e-12 <
              kRequiredClearance)
            {
              tail_sensitivity_done = true;
              for (const auto & joint : kRightArmJoints) {
                for (const double direction : {-1.0, 1.0}) {
                  moveit::core::RobotState perturbed(sample);
                  perturbed.setVariablePosition(
                    joint, perturbed.getVariablePosition(joint) +
                    direction * kAngularStep);
                  perturbed.updateCollisionBodyTransforms();
                  const bool bounds_ok = perturbed.satisfiesBounds(1e-6);
                  collision_detection::CollisionResult perturbed_self;
                  physical_scene.checkSelfCollision(
                    self_request, perturbed_self, perturbed, collision_acm);
                  collision_detection::CollisionResult perturbed_world;
                  physical_scene.getCollisionEnvUnpadded()->checkRobotCollision(
                    world_request, perturbed_world, perturbed, collision_acm);

                  collision_detection::DistanceResult perturbed_distance;
                  physical_scene.getCollisionEnvUnpadded()->distanceRobot(
                    distance_request, perturbed_distance, perturbed);
                  double perturbed_environment = kDistanceBand;
                  update_environment_minimum(
                    perturbed_distance, environment_bodies,
                    perturbed_environment);
                  const auto perturbed_cube = specialized_object_robot_distance(
                    physical_scene, perturbed, cube_exact_right_acm,
                    "green_cube_dynamic_exact");

                  collision_detection::DistanceRequest perturbed_nonpad_request;
                  perturbed_nonpad_request.type =
                    collision_detection::DistanceRequestTypes::SINGLE;
                  perturbed_nonpad_request.enable_nearest_points = true;
                  perturbed_nonpad_request.enable_signed_distance = true;
                  perturbed_nonpad_request.max_contacts_per_body = 1U;
                  perturbed_nonpad_request.distance_threshold = 2.0;
                  perturbed_nonpad_request.acm = &right_nonpad_distance_acm;
                  collision_detection::DistanceResult perturbed_nonpad_result;
                  physical_scene.getCollisionEnvUnpadded()->distanceRobot(
                    perturbed_nonpad_request, perturbed_nonpad_result, perturbed);
                  NeighborhoodSample perturbed_nonpad;
                  update_minimum_from_distance(
                    perturbed_nonpad_result, handle_body,
                    perturbed_nonpad.distance);

                  std::cerr << std::setprecision(17) <<
                    "TAIL_SENSITIVITY\tpoint=" << point_index <<
                    "\tsubdivision=" << subdivision <<
                    "\tjoint=" << joint <<
                    "\tdelta_rad=" << direction * kAngularStep <<
                    "\tbounds_ok=" << (bounds_ok ? "true" : "false") <<
                    "\tself_collision=" <<
                    (perturbed_self.collision ? "true" : "false") <<
                    "\tworld_collision=" <<
                    (perturbed_world.collision ? "true" : "false") <<
                    "\tenvironment_m=" << perturbed_environment <<
                    "\tcube_exact_m=" << perturbed_cube.distance <<
                    "\tcube_link=" << perturbed_cube.link <<
                    "\tright_handle_nonpad_m=" << perturbed_nonpad.distance <<
                    '\n';
                }
              }
            }
          }
          metric.minimum_environment_clearance = std::min(
            metric.minimum_environment_clearance, sample_environment_clearance);
          metric.minimum_table_clearance = std::min(
            metric.minimum_table_clearance, sample_table_clearance);
          metric.minimum_door_clearance = std::min(
            metric.minimum_door_clearance, sample_door_clearance);
          metric.minimum_handle_clearance = std::min(
            metric.minimum_handle_clearance, sample_handle_clearance);
          metric.minimum_cube_clearance = std::min(
            metric.minimum_cube_clearance, sample_cube_clearance);
          metric.minimum_self_clearance = std::min(
            metric.minimum_self_clearance, sample_self_clearance);
          output.minimum_environment_clearance = std::min(
            output.minimum_environment_clearance,
            metric.minimum_environment_clearance);
          output.minimum_table_clearance = std::min(
            output.minimum_table_clearance, metric.minimum_table_clearance);
          output.minimum_door_clearance = std::min(
            output.minimum_door_clearance, metric.minimum_door_clearance);
          output.minimum_handle_clearance = std::min(
            output.minimum_handle_clearance, metric.minimum_handle_clearance);
          output.minimum_cube_clearance = std::min(
            output.minimum_cube_clearance, metric.minimum_cube_clearance);
          output.minimum_self_clearance = std::min(
            output.minimum_self_clearance, metric.minimum_self_clearance);
          if (sample_environment_clearance + 1e-12 < kRequiredClearance &&
            !in_tail_scan)
          {
            const auto environment_pair = closest_distance_pair(
              distance_result, environment_bodies);
            const bool cube_door_diagnostic_sample =
              diagnose_cube_door && phase.asset == AssetKind::Cube &&
              message_index == 2U &&
              environment_pair.find("incubator_door_exact") != std::string::npos;
            if (cube_door_diagnostic_sample) {
              std::cerr << std::setprecision(17) <<
                "CUBE_DOOR_DIAGNOSTIC\tmessage=" << message_index <<
                "\tpoint=" << point_index <<
                "\tsubdivision=" << subdivision <<
                "\tdoor_m=" << sample_door_clearance <<
                "\tpair=" << environment_pair << '\n';
            } else if (phase.asset == AssetKind::Cube && message_index == 2U &&
              environment_pair.find("incubator_door_exact") != std::string::npos)
            {
              const std::array<std::string, 7> left_arm_joints{
                "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
                "left_shoulder_yaw_joint", "left_elbow_joint",
                "left_wrist_roll_joint", "left_wrist_pitch_joint",
                "left_wrist_yaw_joint"};
              const auto reference_tcp =
                sample.getGlobalLinkTransform("left_dex1_tcp");
              FclGeometryCache sensitivity_cache;
              for (const auto & joint : left_arm_joints) {
                for (const double delta : {-kAngularStep, kAngularStep}) {
                  moveit::core::RobotState perturbed(sample);
                  perturbed.setVariablePosition(
                    joint, perturbed.getVariablePosition(joint) + delta);
                  perturbed.updateCollisionBodyTransforms();
                  const auto door = exact_door_robot_distance(
                    sensitivity_cache, geometry, perturbed, door_angle);
                  const auto tcp =
                    perturbed.getGlobalLinkTransform("left_dex1_tcp");
                  std::cerr << std::setprecision(17) <<
                    "CUBE_DOOR_SENSITIVITY\tmessage=" << message_index <<
                    "\tpoint=" << point_index <<
                    "\tsubdivision=" << subdivision <<
                    "\tjoint=" << joint <<
                    "\tdelta_rad=" << delta <<
                    "\tdoor_link=" << door.link <<
                    "\tdoor_m=" << door.distance <<
                    "\ttcp_translation_mm=" <<
                    (tcp.translation() - reference_tcp.translation()).norm() * 1000.0 <<
                    "\ttcp_rotation_deg=" <<
                    rotation_distance(reference_tcp, tcp) * 180.0 / kPi << '\n';
                }
              }
            }
            if (!cube_door_diagnostic_sample) {
              throw std::runtime_error(
                    "ENVIRONMENT_CLEARANCE:" + std::string(phase.name) +
                    ":message=" + std::to_string(message_index) +
                    ":point=" + std::to_string(point_index) +
                    ":subdivision=" + std::to_string(subdivision) + ":" +
                    environment_pair);
            }
          }

          collision_detection::AllowedCollisionMatrix left_handle_acm = base_acm;
          allow_all_self_distance_pairs(left_handle_acm, collision_links);
          for (const auto & object : physical_scene.getWorld()->getObjectIds()) {
            for (const auto & link : collision_links) {
              left_handle_acm.setEntry(object, link, true);
            }
          }
          std::size_t left_link_count = 0U;
          for (const auto & link : collision_links) {
            if (link.rfind("left_", 0U) == 0U) {
              left_handle_acm.setEntry(kHandleId, link, false);
              ++left_link_count;
            }
          }
          if (left_link_count == 0U) {
            throw std::runtime_error("LEFT_COLLISION_LINK_SET");
          }
          collision_detection::CollisionResult left_collision;
          physical_scene.getCollisionEnvUnpadded()->checkRobotCollision(
            world_request, left_collision, sample, left_handle_acm);
          if (left_collision.collision) {
            ++output.left_handle_contact_count;
            throw std::runtime_error("LEFT_HANDLE_CONTACT:" + std::string(phase.name));
          }
          collision_detection::DistanceRequest left_distance_request;
          left_distance_request.type = collision_detection::DistanceRequestTypes::ALL;
          left_distance_request.enable_nearest_points = true;
          left_distance_request.enable_signed_distance = true;
          left_distance_request.max_contacts_per_body = 1U;
          left_distance_request.distance_threshold = 2.0;
          left_distance_request.acm = &left_handle_acm;
          collision_detection::DistanceResult left_distance_result;
          physical_scene.getCollisionEnvUnpadded()->distanceRobot(
            left_distance_request, left_distance_result, sample);
          update_minimum_from_distance(
            left_distance_result, handle_body,
            output.minimum_left_handle_clearance);
          update_minimum_from_distance(
            left_distance_result, handle_body,
            metric.minimum_left_handle_clearance);
          if (output.minimum_left_handle_clearance + 1e-12 <
            kLeftHandleClearance)
          {
            throw std::runtime_error("LEFT_HANDLE_CLEARANCE:" + std::string(phase.name));
          }

          const double torso = sample.getVariablePosition("torso_joint");
          output.maximum_abs_torso = std::max(
            output.maximum_abs_torso, std::abs(torso));
          output.minimum_torso = std::min(output.minimum_torso, torso);
          output.maximum_torso = std::max(output.maximum_torso, torso);
          metric.minimum_torso = std::min(metric.minimum_torso, torso);
          metric.maximum_torso = std::max(metric.maximum_torso, torso);
          if (output.maximum_abs_torso > kPi / 4.0 + 1e-12) {
            throw std::runtime_error("TORSO_45_DEGREE_LIMIT");
          }
          ++output.samples;
          ++output.subdivisions;
          ++metric.samples;
          ++metric.subdivisions;

          if (tail_scan && phase.asset == AssetKind::DoorOpen &&
            message_index == 1U && point_index == 805U &&
            subdivision == subdivisions)
          {
            throw std::runtime_error("TAIL_SCAN_COMPLETE_NONCERTIFICATE");
          }

        }
        state = next;
        current_door_angle = target_door_angle;
      }
    }
  }

  output.final_park_door_clearance = moveit_world_object_robot_distance(
    physical_scene, state, base_acm, collision_links, "incubator_door_exact");
  if (!output.cube_release_retreat_gate_reached) {
    throw std::runtime_error("CUBE_RELEASE_RETREAT_GATE_NOT_REACHED");
  }
  if (std::abs(current_door_angle) > 1e-12) {
    throw std::runtime_error("FINAL_DOOR_ANGLE");
  }
  verify_seam_state(state, park_roll);
  if (output.final_park_door_clearance.distance + 1e-12 <
    kParkStaticClearance)
  {
    throw std::runtime_error(
            "FINAL_PARK_DOOR_CLEARANCE:" +
            output.final_park_door_clearance.link + ":" +
            std::to_string(output.final_park_door_clearance.distance));
  }
  if (!cube_witness_start || !cube_bump_audited) {
    throw std::runtime_error("CUBE_RELEASE_WITNESS_START");
  }
  const auto witness = trace_release_witness_grid(
    *cube_witness_start, assets.at(AssetKind::Cube));
  FclGeometryCache fcl_cache;
  const std::array<const moveit::core::RobotState *, 3> states{
    &witness[0].state, &witness[1].state, &witness[2].state};
  for (std::size_t index = 0U; index < states.size(); ++index) {
    output.neighborhood[index] = physical_finger2_distance(
      fcl_cache, geometry, *states[index], static_cast<int>(index) - 1);
    if (output.neighborhood[index].distance + 1e-12 < kRequiredClearance) {
      throw std::runtime_error("CUBE_RELEASE_PHYSICAL_STL_CLEARANCE");
    }
  }
  if (mask_conflict_pair_present) {
    output.observed_visual_distance = visual_mask_distance(
      fcl_cache, geometry, raw_scene, witness[1].state);
    if (std::abs(*output.observed_visual_distance - 0.002665455) > 1e-9) {
      throw std::runtime_error("CUBE_RELEASE_VISUAL_DISTANCE");
    }
  }
  const std::size_t expected_phase_count = kPhases.size();
  if (output.samples == 0U || output.phases.size() != expected_phase_count) {
    throw std::runtime_error("AUDIT_SAMPLE_COUNT");
  }
  std::size_t phase_sample_sum = 0U;
  std::size_t phase_subdivision_sum = 0U;
  for (const auto & metric : output.phases) {
    if (metric.samples == 0U || metric.subdivisions == 0U) {
      throw std::runtime_error("AUDIT_EMPTY_PHASE:" + metric.name);
    }
    phase_sample_sum += metric.samples;
    phase_subdivision_sum += metric.subdivisions;
  }
  if (phase_sample_sum != output.samples ||
    phase_subdivision_sum != output.subdivisions)
  {
    throw std::runtime_error("AUDIT_PHASE_TOTALS");
  }
  const auto finite = [](const double value) {return std::isfinite(value);};
  const std::array<double, 13> required_finite{{
    output.minimum_environment_clearance, output.minimum_table_clearance,
    output.minimum_door_clearance, output.minimum_handle_clearance,
    output.minimum_cube_clearance, output.minimum_self_clearance,
    output.minimum_right_handle_nonpad_clearance,
    output.minimum_left_handle_clearance, output.minimum_torso,
    output.maximum_torso, output.maximum_abs_torso,
    output.initial_park_door_clearance.distance,
    output.final_park_door_clearance.distance,
  }};
  if (!std::all_of(required_finite.begin(), required_finite.end(), finite) ||
    output.minimum_environment_clearance + 1e-12 < kDistanceBand ||
    output.minimum_table_clearance + 1e-12 < kDistanceBand ||
    output.minimum_door_clearance + 1e-12 < kDistanceBand ||
    output.minimum_right_handle_nonpad_clearance + 1e-12 <
      kRightHandleNonpadClearance ||
    output.minimum_left_handle_clearance + 1e-12 < kLeftHandleClearance ||
    output.maximum_abs_torso > kPi / 4.0 + 1e-12)
  {
    throw std::runtime_error("AUDIT_GLOBAL_METRICS");
  }
  for (const auto & metric : output.phases) {
    const std::array<double, 10> phase_finite{{
      metric.minimum_environment_clearance, metric.minimum_table_clearance,
      metric.minimum_door_clearance, metric.minimum_handle_clearance,
      metric.minimum_cube_clearance, metric.minimum_self_clearance,
      metric.minimum_right_handle_nonpad_clearance,
      metric.minimum_left_handle_clearance, metric.minimum_torso,
      metric.maximum_torso,
    }};
    if (!std::all_of(phase_finite.begin(), phase_finite.end(), finite) ||
      metric.right_handle_authorized_samples != metric.samples ||
      metric.minimum_environment_clearance + 1e-12 < kDistanceBand ||
      metric.minimum_table_clearance + 1e-12 < kDistanceBand ||
      metric.minimum_door_clearance + 1e-12 < kDistanceBand ||
      metric.minimum_right_handle_nonpad_clearance + 1e-12 <
        kRightHandleNonpadClearance ||
      metric.minimum_left_handle_clearance + 1e-12 < kLeftHandleClearance ||
      std::max(std::abs(metric.minimum_torso),
      std::abs(metric.maximum_torso)) > kPi / 4.0 + 1e-12)
    {
      throw std::runtime_error("AUDIT_PHASE_METRICS:" + metric.name);
    }
  }
  const std::array<double, 9> retreat_finite{{
    output.cube_release_retreat_minimum_exact_distance,
    output.cube_release_retreat_maximum_exact_distance,
    output.cube_release_retreat_final_exact_distance,
    output.cube_release_retreat_maximum_exact_penetration,
    output.cube_release_retreat_minimum_path_parameter_step,
    output.cube_release_retreat_minimum_projection_step,
    output.cube_release_retreat_final_projection,
    output.cube_release_retreat_post_gate_minimum_exact_distance,
    output.cube_release_retreat_post_gate_minimum_guard_physical_lower_bound,
  }};
  if (!std::all_of(retreat_finite.begin(), retreat_finite.end(), finite) ||
    !output.cube_release_retreat_gate_reached ||
    output.cube_release_retreat_window_samples == 0U ||
    output.cube_release_retreat_minimum_time_step_ns ==
      std::numeric_limits<std::int64_t>::max() ||
    output.cube_release_retreat_minimum_time_step_ns <= 0 ||
    output.cube_release_retreat_minimum_path_parameter_step <= 0.0 ||
    output.cube_release_retreat_maximum_exact_penetration > 1e-12 ||
    output.cube_release_retreat_post_gate_minimum_exact_distance + 1e-12 <
      kDistanceBand ||
    output.cube_release_retreat_post_gate_minimum_guard_physical_lower_bound +
      1e-12 < kDistanceBand)
  {
    throw std::runtime_error("AUDIT_RELEASE_RETREAT_METRICS");
  }
  if (output.cube_guard_exact_refinement_samples == 0U &&
    finite(output.minimum_cube_exact_noncontact_clearance))
  {
    throw std::runtime_error("AUDIT_CUBE_REFINEMENT_NULLABILITY");
  }
  if (output.cube_guard_exact_refinement_samples != 0U &&
    !finite(output.minimum_cube_exact_noncontact_clearance))
  {
    throw std::runtime_error("AUDIT_CUBE_REFINEMENT_FINITE");
  }
  const std::size_t expected_cube_grasp_carry_samples =
    output.phases.at(4U).samples + output.phases.at(5U).samples +
    output.phases.at(6U).samples;
  if (output.right_handle_window_samples != output.samples ||
    output.cube_grasp_carry_window_samples !=
      expected_cube_grasp_carry_samples ||
    output.right_handle_observed_pairs.empty() ||
    output.cube_grasp_carry_observed_exact_pairs.empty() ||
    output.cube_grasp_carry_observed_guard_pairs.empty() ||
    output.cube_release_retreat_observed_exact_pairs.empty() ||
    output.cube_release_retreat_observed_guard_pairs.empty() ||
    !finite(output.right_handle_maximum_observed_penetration) ||
    !finite(output.cube_grasp_carry_maximum_observed_penetration) ||
    !finite(output.cube_grasp_carry_minimum_guard_physical_lower_bound) ||
    !finite(output.cube_bump_maximum_tcp_translation) ||
    !finite(output.cube_bump_maximum_tcp_rotation) ||
    !finite(output.final_target_translation_error) ||
    !finite(output.final_target_rotation_error) ||
    output.right_handle_maximum_observed_penetration < 0.0 ||
    output.cube_grasp_carry_maximum_observed_penetration < 0.0 ||
    output.cube_bump_maximum_tcp_translation < 0.0 ||
    output.cube_bump_maximum_tcp_translation > 0.006 + 1e-12 ||
    output.cube_bump_maximum_tcp_rotation < 0.0 ||
    output.cube_bump_maximum_tcp_rotation >
      0.013962634015954637 + 1e-12 ||
    output.final_target_translation_error < 0.0 ||
    output.final_target_translation_error > 0.003 + 1e-12 ||
    output.final_target_rotation_error < 0.0 ||
    output.final_target_rotation_error >
      0.03490658503988659 + 1e-12)
  {
    throw std::runtime_error("AUDIT_CONTACT_WINDOWS");
  }
  for (const auto & [object_id, link] : output.right_handle_observed_pairs) {
    if (object_id != kHandleId || !right_handle_pad_links.count(link)) {
      throw std::runtime_error("AUDIT_HANDLE_OBSERVED_PAIR");
    }
  }
  for (const auto & [object_id, link] :
    output.cube_grasp_carry_observed_exact_pairs)
  {
    if (object_id != "green_cube_dynamic_exact" ||
      !left_cube_touch_links.count(link))
    {
      throw std::runtime_error("AUDIT_CUBE_EXACT_OBSERVED_PAIR");
    }
  }
  for (const auto & [object_id, link] :
    output.cube_release_retreat_observed_exact_pairs)
  {
    if (object_id != "green_cube_dynamic_exact" ||
      !left_cube_touch_links.count(link))
    {
      throw std::runtime_error("AUDIT_RETREAT_EXACT_OBSERVED_PAIR");
    }
  }
  for (const auto * pairs : {
      &output.cube_grasp_carry_observed_guard_pairs,
      &output.cube_release_retreat_observed_guard_pairs})
  {
    for (const auto & [object_id, link] : *pairs) {
      if (object_id != "green_cube_10mm_guard" ||
        !left_cube_touch_links.count(link))
      {
        throw std::runtime_error("AUDIT_CUBE_GUARD_OBSERVED_PAIR");
      }
    }
  }
  return output;
}

std::string evidence_json(
  const std::string & raw_manifest_sha256, const Projection & projection,
  const RobotModelSnapshot & robot_model_snapshot,
  const ExactAuditResult & audit)
{
  if (!lowercase_hex_64(raw_manifest_sha256)) {
    throw std::runtime_error("RAW_MANIFEST_SHA256");
  }
  const std::array<std::string, 11> asset_ids{
    "door_open", "door_open", "door60_seam_to_cube", "cube", "cube",
    "cube", "cube", "cube", "cube", "door60_seam_to_hinge", "door_close"};
  const std::array<std::array<std::size_t, 2>, 11> formal_ranges{{
    {{0U, 0U}}, {{0U, 2U}}, {{0U, 2U}}, {{0U, 1U}}, {{1U, 3U}},
    {{3U, 5U}}, {{5U, 6U}}, {{6U, 7U}}, {{7U, 8U}}, {{0U, 2U}},
    {{0U, 2U}},
  }};
  std::ostringstream output;
  output << std::boolalpha << std::setprecision(17) <<
    "{\n"
    "  \"schema\": \"h1_candidate3_middle_e0_raw_combined_exact/828-e0-v1\",\n"
    "  \"auditor_version\": \"" << kAuditorVersion << "\",\n"
    "  \"audit_method\": \"FCL_UNPADDED_DYNAMIC_EXACT_STL_DENSE_INTERPOLATION\",\n"
    "  \"raw_asset_manifest_sha256\": \"" << raw_manifest_sha256 << "\",\n"
    "  \"raw_scene_hash\": \"" << projection.binding.raw_scene_hash << "\",\n"
    "  \"physical_scene_hash\": \"" << projection.physical_cdr_sha256 << "\",\n"
    "  \"raw_scene_cdr_sha256\": \"" << projection.raw_cdr_sha256 << "\",\n"
    "  \"entry_projection\": \"prefix_certified_endpoint_projection_to_canonical_e0_q90_held\",\n"
    "  \"robot_model_snapshot\": {\n"
    "    \"inventory_sha256\": \"" <<
    robot_model_snapshot.inventory_sha256 << "\",\n"
    "    \"urdf_sha256\": \"" << robot_model_snapshot.urdf_sha256 << "\",\n"
    "    \"srdf_sha256\": \"" << robot_model_snapshot.srdf_sha256 << "\",\n"
    "    \"mesh_count\": " << robot_model_snapshot.meshes.size() << "\n"
    "  },\n"
    "  \"controller_commands_issued\": 0,\n"
    "  \"offline_only\": true,\n"
    "  \"sample_count\": " << audit.samples << ",\n"
    "  \"subdivision_count\": " << audit.subdivisions << ",\n"
    "  \"collision_free\": true,\n"
    "  \"cube_bump\": {\n"
    "    \"reference_file_sha256\": \"09919a92a65b5ca90b3a11420bc9209ceead1e90112f94da2c4c18711af8e134\",\n"
    "    \"reference_payload_sha256\": \"acaec97384f16a736f6b246a3fa1a0897b7aad93e72a6204701610ebf4f7e705\",\n"
    "    \"only_modified_joint\": \"left_shoulder_roll_joint\",\n"
    "    \"comparison\": \"same_start_state_dense_interpolation_max_of_modified_and_reference_subdivisions\",\n"
    "    \"maximum_tcp_translation_m\": " <<
    audit.cube_bump_maximum_tcp_translation << ",\n"
    "    \"maximum_tcp_rotation_rad\": " <<
    audit.cube_bump_maximum_tcp_rotation << ",\n"
    "    \"translation_limit_m\": 0.006,\n"
    "    \"rotation_limit_rad\": 0.013962634015954637\n"
    "  },\n"
    "  \"final_target\": {\n"
    "    \"translation_error_m\": " << audit.final_target_translation_error << ",\n"
    "    \"rotation_error_rad\": " << audit.final_target_rotation_error << ",\n"
    "    \"translation_limit_m\": 0.003,\n"
    "    \"rotation_limit_rad\": 0.03490658503988659\n"
    "  },\n"
    "  \"minimum_clearances_m\": {\n"
    "    \"unauthorized_environment\": " << audit.minimum_environment_clearance << ",\n"
    "    \"table\": " << audit.minimum_table_clearance << ",\n"
    "    \"door\": " << audit.minimum_door_clearance << ",\n"
    "    \"handle\": " << audit.minimum_handle_clearance << ",\n"
    "    \"cube\": " << audit.minimum_cube_clearance << ",\n"
    "    \"self\": " << audit.minimum_self_clearance << ",\n"
    "    \"right_handle_nonpad\": " << audit.minimum_right_handle_nonpad_clearance << ",\n"
    "    \"left_handle\": " << audit.minimum_left_handle_clearance << "\n"
    "  },\n"
    "  \"cube_guard_exact_refinement_samples\": " <<
    audit.cube_guard_exact_refinement_samples << ",\n"
    "  \"minimum_cube_exact_noncontact_clearance_m\": ";
  if (std::isfinite(audit.minimum_cube_exact_noncontact_clearance)) {
    output << audit.minimum_cube_exact_noncontact_clearance;
  } else {
    output << "null";
  }
  output << ",\n"
    "  \"minimum_environment_clearance_is_lower_bound\": true,\n"
    "  \"park_roll_rad\": " << audit.initial_left_shoulder_roll << ",\n"
    "  \"environment_clearance_derivation\": {\n"
    "    \"exact_world_objects\": \"direct_unpadded_FCL_distance\",\n"
    "    \"environment_table_10mm_margin\": \"0.010_plus_guard_distance\",\n"
    "    \"green_cube_10mm_guard\": \"0.010_plus_guard_distance\",\n"
    "    \"self_pairs_excluded\": true\n"
    "  },\n"
    "  \"park_static_clearance_required_m\": " << kParkStaticClearance << ",\n"
    "  \"initial_park_exact_door_clearance\": {\"link\":\"" <<
    json_escape(audit.initial_park_door_clearance.link) <<
    "\",\"distance_m\":" << audit.initial_park_door_clearance.distance << "},\n"
    "  \"final_park_exact_door_clearance\": {\"link\":\"" <<
    json_escape(audit.final_park_door_clearance.link) <<
    "\",\"distance_m\":" << audit.final_park_door_clearance.distance << "},\n"
    "  \"right_handle_nonpad_clearance_required_m\": " <<
    kRightHandleNonpadClearance << ",\n"
    "  \"left_handle_clearance_required_m\": " <<
    kLeftHandleClearance << ",\n"
    "  \"dynamic_handle_exact\": {\n"
    "    \"id\": \"incubator_handle_exact\",\n"
    "    \"geometry_type\": \"mesh\",\n"
    "    \"mesh_count\": 1,\n"
    "    \"primitive_count\": 0,\n"
    "    \"geometry_sha256\": \"" << kExpectedHandleSha << "\",\n"
    "    \"retained_every_phase\": true,\n"
    "    \"audited_phases\": [";
  for (std::size_t index = 0U; index < kPhases.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << '"' << kPhases[index].name << '"';
  }
  output << "],\n"
    "    \"angle_rad\": " << kHandleAngle << ",\n"
    "    \"door_angle_min_rad\": 0,\n"
    "    \"door_angle_max_rad\": " << kDoorOpenAngle << "\n"
    "  },\n"
    "  \"right_handle_allowed_contact_pairs\": [\n"
    "    [\"incubator_handle_exact\",\"right_dex1_finger_1_pad_link\"],\n"
    "    [\"incubator_handle_exact\",\"right_dex1_finger_2_pad_link\"]\n"
    "  ],\n"
    "  \"right_door_allowed_contact_pairs\": [],\n"
    "  \"left_cube_allowed_contact_links\": [\n"
    "    \"left_dex1_finger_1_slider_link\",\"left_dex1_finger_1_link\",\n"
    "    \"left_dex1_finger_1_pad_link\",\"left_dex1_finger_2_slider_link\",\n"
    "    \"left_dex1_finger_2_link\",\"left_dex1_finger_2_pad_link\"\n"
    "  ],\n"
    "  \"left_cube_object_ids\": [\"green_cube_dynamic_exact\",\"green_cube_10mm_guard\"],\n"
    "  \"contact_window_measurements\": {\n"
    "    \"right_handle_hold\": {\n"
    "      \"authorized_sample_count\": " <<
    audit.right_handle_window_samples << ",\n"
    "      \"maximum_observed_penetration_m\": " <<
    audit.right_handle_maximum_observed_penetration << ",\n"
    "      \"penetration_geometry\": \"physical_exact_incubator_handle_mesh\",\n"
    "      \"observed_pair_semantics\": \"nearest_signed_distance_observation_not_collision_event\",\n"
    "      \"observed_pairs\": " <<
    json_pairs(audit.right_handle_observed_pairs) << "\n"
    "    },\n"
    "    \"cube_grasp_carry_release\": {\n"
    "      \"authorized_sample_count\": " <<
    audit.cube_grasp_carry_window_samples << ",\n"
    "      \"maximum_observed_penetration_m\": " <<
    audit.cube_grasp_carry_maximum_observed_penetration << ",\n"
    "      \"penetration_geometry\": \"green_cube_dynamic_exact_physical_only\",\n"
    "      \"observed_pair_semantics\": \"nearest_signed_distance_observation_not_collision_event\",\n"
    "      \"observed_pairs\": " <<
    json_pairs(audit.cube_grasp_carry_observed_exact_pairs) << ",\n"
    "      \"guard_observation_geometry\": \"green_cube_10mm_guard_conservative_nonphysical\",\n"
    "      \"guard_nearest_pairs\": " <<
    json_pairs(audit.cube_grasp_carry_observed_guard_pairs) << ",\n"
    "      \"minimum_guard_physical_lower_bound_m\": " <<
    audit.cube_grasp_carry_minimum_guard_physical_lower_bound << "\n"
    "    },\n"
    "    \"cube_release_retreat\": {\n"
    "      \"authorized_sample_count\": " <<
    audit.cube_release_retreat_window_samples << ",\n"
    "      \"maximum_observed_penetration_m\": " <<
    audit.cube_release_retreat_maximum_exact_penetration << ",\n"
    "      \"penetration_geometry\": \"green_cube_dynamic_exact_physical_only\",\n"
    "      \"observed_pair_semantics\": \"nearest_signed_distance_observation_not_collision_event\",\n"
    "      \"observed_pairs\": " <<
    json_pairs(audit.cube_release_retreat_observed_exact_pairs) << ",\n"
    "      \"guard_observation_geometry\": \"green_cube_10mm_guard_conservative_nonphysical\",\n"
    "      \"guard_nearest_pairs\": " <<
    json_pairs(audit.cube_release_retreat_observed_guard_pairs) << "\n"
    "    }\n"
    "  },\n"
    "  \"torso_joint\": {\n"
    "    \"commanded\": true,\n"
    "    \"minimum_rad\": " << audit.minimum_torso << ",\n"
    "    \"maximum_rad\": " << audit.maximum_torso << ",\n"
    "    \"maximum_absolute_rad\": " << audit.maximum_abs_torso << "\n"
    "  },\n"
    "  \"left_handle\": {\n"
    "    \"allowed_contact_pairs\": [],\n"
    "    \"contact_count\": " << audit.left_handle_contact_count << ",\n"
    "    \"minimum_clearance_m\": " << audit.minimum_left_handle_clearance << "\n"
    "  },\n"
    "  \"mask_projection\": {\n"
    "    \"raw_scene_hash\": \"" << projection.binding.raw_scene_hash << "\",\n"
    "    \"physical_scene_hash\": \"" << projection.physical_cdr_sha256 << "\",\n"
    "    \"removed_visual_mask_ids\": " << json_strings(projection.removed) << ",\n"
    "    \"unknown_visual_mask_ids\": " << json_strings(projection.unknown) << ",\n"
    "    \"classification\": \"" <<
    (projection.mask_conflict_pair_present ? "ALLOWLIST_MASKS_PROJECTED" :
    "RAW_SCENE_ALREADY_PHYSICAL") << "\",\n"
    "    \"handle_exact_retained\": true\n"
    "  },\n"
    "  \"mask_conflict_recheck\": {\n"
    "    \"visual_pair\": [\"incubator_handle_visual_mask\","
    "\"left_dex1_finger_2_visual_mask\"],\n"
    "    \"phase\": \"cube_release\",\n"
    "    \"source_wrapper_message_index\": 5,\n"
    "    \"dense_sample_index\": 527,\n"
    "    \"dense_index_scope\": \"cube_asset_global_release_recheck_grid_v1\",\n"
    "    \"dense_grid_max_angular_step_rad\": " << 3.0 * kPi / 180.0 << ",\n"
    "    \"dense_grid_max_prismatic_step_m\": " << kPrismaticStep << ",\n"
    "    \"observed_visual_distance_m\": ";
  if (audit.observed_visual_distance) {
    output << *audit.observed_visual_distance;
  } else {
    output << "null";
  }
  output << ",\n"
    "    \"classification\": \"" <<
    (projection.mask_conflict_pair_present ? "MASK_ONLY_FALSE_POSITIVE" :
    "RAW_SCENE_ALREADY_PHYSICAL") << "\",\n"
    "    \"physical_exact_stl_neighborhood\": [\n";
  for (std::size_t index = 0U; index < audit.neighborhood.size(); ++index) {
    const auto & sample = audit.neighborhood[index];
    output << "      {\"offset\":" << sample.offset <<
      ",\"dense_sample_index\":" << 527 + sample.offset <<
      ",\"exact_stl\":true,\"pair\":[\"incubator_handle_exact\",\"" <<
      json_escape(sample.link) << "\"],\"distance_m\":" << sample.distance << "}";
    output << (index + 1U == audit.neighborhood.size() ? "\n" : ",\n");
  }
  output <<
    "    ]\n"
    "  },\n"
    "  \"cube_release_retreat\": {\n"
    "    \"phase_name\": \"cube_retreat\",\n"
    "    \"message_index\": 6,\n"
    "    \"start_point_index\": 0,\n"
    "    \"start_subdivision\": 0,\n"
    "    \"window_sample_count\": " <<
    audit.cube_release_retreat_window_samples << ",\n"
    "    \"gate_point_index\": " << audit.cube_release_retreat_gate_point << ",\n"
    "    \"gate_subdivision\": " <<
    audit.cube_release_retreat_gate_subdivision << ",\n"
    "    \"gate_ordinal\": " << audit.cube_release_retreat_gate_ordinal << ",\n"
    "    \"last_below_gate_ordinal\": " <<
    audit.cube_release_retreat_last_below_gate_ordinal << ",\n"
    "    \"gate_reached\": " << audit.cube_release_retreat_gate_reached << ",\n"
    "    \"minimum_exact_distance_m\": " <<
    audit.cube_release_retreat_minimum_exact_distance << ",\n"
    "    \"maximum_exact_distance_m\": " <<
    audit.cube_release_retreat_maximum_exact_distance << ",\n"
    "    \"final_exact_distance_m\": " <<
    audit.cube_release_retreat_final_exact_distance << ",\n"
    "    \"maximum_exact_penetration_m\": " <<
    audit.cube_release_retreat_maximum_exact_penetration << ",\n"
    "    \"minimum_time_step_ns\": " <<
    audit.cube_release_retreat_minimum_time_step_ns << ",\n"
    "    \"minimum_path_parameter_step\": " <<
    audit.cube_release_retreat_minimum_path_parameter_step << ",\n"
    "    \"minimum_tcp_separation_projection_step_m\": " <<
    audit.cube_release_retreat_minimum_projection_step << ",\n"
    "    \"final_tcp_projection_m\": " <<
    audit.cube_release_retreat_final_projection << ",\n"
    "    \"post_gate_minimum_exact_distance_m\": " <<
    audit.cube_release_retreat_post_gate_minimum_exact_distance << ",\n"
    "    \"post_gate_minimum_guard_physical_lower_bound_m\": " <<
    audit.cube_release_retreat_post_gate_minimum_guard_physical_lower_bound << ",\n"
    "    \"penetration_geometry\": \"green_cube_dynamic_exact_physical_only\",\n"
    "    \"guard_geometry\": \"green_cube_10mm_guard_conservative_nonphysical\",\n"
    "    \"observed_pair_semantics\": \"nearest_signed_distance_observation_not_collision_event\",\n"
    "    \"observed_exact_pairs\": " <<
    json_pairs(audit.cube_release_retreat_observed_exact_pairs) << ",\n"
    "    \"observed_guard_pairs\": " <<
    json_pairs(audit.cube_release_retreat_observed_guard_pairs) << ",\n"
    "    \"observed_exact_links\": [";
  std::size_t observed_index = 0U;
  for (const auto & link : audit.cube_release_retreat_observed_exact_links) {
    if (observed_index++ != 0U) {
      output << ',';
    }
    output << '"' << json_escape(link) << '"';
  }
  output << "],\n"
    "    \"observed_guard_links\": [";
  observed_index = 0U;
  for (const auto & link : audit.cube_release_retreat_observed_guard_links) {
    if (observed_index++ != 0U) {
      output << ',';
    }
    output << '"' << json_escape(link) << '"';
  }
  output << "],\n"
    "    \"gate_never_regressed\": true\n"
    "  },\n"
    "  \"per_phase_metrics\": [\n";
  for (std::size_t index = 0U; index < audit.phases.size(); ++index) {
    const auto & metric = audit.phases[index];
    output << "    {\"phase\":\"" << json_escape(metric.name) <<
      "\",\"asset_id\":\"" << asset_ids[index] <<
      "\",\"message_range\":[" << formal_ranges[index][0] << ',' <<
      formal_ranges[index][1] << ']' <<
      ",\"samples\":" << metric.samples <<
      ",\"subdivisions\":" << metric.subdivisions <<
      ",\"right_handle_authorized_samples\":" <<
      metric.right_handle_authorized_samples <<
      ",\"minimum_is_lower_bound\":true" <<
      ",\"minimum_environment_clearance_m\":" <<
      metric.minimum_environment_clearance <<
      ",\"minimum_table_clearance_m\":" << metric.minimum_table_clearance <<
      ",\"minimum_door_clearance_m\":" << metric.minimum_door_clearance <<
      ",\"minimum_handle_clearance_m\":" << metric.minimum_handle_clearance <<
      ",\"minimum_cube_clearance_m\":" << metric.minimum_cube_clearance <<
      ",\"minimum_self_clearance_m\":" << metric.minimum_self_clearance <<
      ",\"minimum_left_handle_clearance_m\":" <<
      metric.minimum_left_handle_clearance <<
      ",\"minimum_torso_rad\":" << metric.minimum_torso <<
      ",\"maximum_torso_rad\":" << metric.maximum_torso <<
      ",\"minimum_right_handle_nonpad_clearance_m\":";
    if (std::isfinite(metric.minimum_right_handle_nonpad_clearance)) {
      output << metric.minimum_right_handle_nonpad_clearance;
    } else {
      output << "null";
    }
    output << "}";
    output << (index + 1U == audit.phases.size() ? "\n" : ",\n");
  }
  output <<
    "  ],\n"
    "  \"raw_audit_complete\": true,\n"
    "  \"geometry_valid\": true,\n"
    "  \"valid\": false,\n"
    "  \"status\": \"RAW_EXACT_PASS_NOT_SEALED_RESULT\"\n"
    "}\n";
  return output.str();
}

std::string failure_json(
  const std::string & status, const Projection * projection = nullptr)
{
  std::ostringstream output;
  output << std::boolalpha <<
    "{\n"
    "  \"schema\": \"" << kFailureSchema << "\",\n"
    "  \"auditor_version\": \"" << kAuditorVersion << "\",\n"
    "  \"audit_method\": \"FCL_UNPADDED_DYNAMIC_EXACT_STL\",\n"
    "  \"status\": \"" << json_escape(status) << "\",\n"
    "  \"valid\": false,\n"
    "  \"controller_commands_issued\": 0";
  if (projection != nullptr) {
    output << ",\n"
      "  \"raw_scene_hash\": \"" << projection->binding.raw_scene_hash << "\",\n"
      "  \"raw_scene_cdr_sha256\": \"" << projection->raw_cdr_sha256 << "\",\n"
      "  \"physical_scene_hash\": \"" << projection->physical_cdr_sha256 << "\",\n"
      "  \"removed_visual_mask_ids\": " << json_strings(projection->removed) << ",\n"
      "  \"unknown_visual_mask_ids\": " << json_strings(projection->unknown);
  }
  output << "\n}\n";
  return output.str();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  std::filesystem::path output_path;
  std::optional<Projection> projection;
  try {
    const auto args = arguments(argc, argv);
    output_path = args.output;
    const auto cdr = read_sealed_bytes(
      args.scene,
      "dacf67c055f5ba1b8d46c30b6be4aafdd80a6d453faee069b2d22e5a0e791568",
      0400);
    const auto raw_scene = deserialize_scene(cdr);
    projection = project_physical_scene(raw_scene, cdr);
    if (args.inspect_projection) {
      atomic_write(args.output, projection_json(*projection));
      std::cout << "CANDIDATE3_MIDDLE_PROJECTION=PASS physical_scene_hash=" <<
        projection->physical_cdr_sha256 << std::endl;
      rclcpp::shutdown();
      return 0;
    }

    if (!projection->unknown.empty()) {
      throw std::runtime_error("UNKNOWN_VISUAL_MASK_ID");
    }
    const std::set<std::string> removed(
      projection->removed.begin(), projection->removed.end());
    if (!removed.empty() &&
      (removed.count(kHandleMaskId) == 0U ||
      removed.count(kLeftFingerMaskId) == 0U))
    {
      throw std::runtime_error("INCOMPLETE_MASK_CONFLICT_PAIR");
    }
    if (!projection->handle_exact_retained) {
      throw std::runtime_error("HANDLE_EXACT_REMOVED");
    }
    const auto manifest_bytes = read_sealed_bytes(
      args.plan,
      "cf15eebffedbbb6c5ec0fd7fb14abc013fa213d8e96013031f4f5cdfec3119f6",
      0400);
    const auto manifest_sha256 = sha256_hex(manifest_bytes);
    if (manifest_sha256 !=
      "cf15eebffedbbb6c5ec0fd7fb14abc013fa213d8e96013031f4f5cdfec3119f6")
    {
      throw std::runtime_error("RAW_MANIFEST_FILE_SHA256");
    }
    const auto manifest_root = YAML::Load(std::string(
      manifest_bytes.begin(), manifest_bytes.end()));
    require_raw_manifest(manifest_root);
    const auto robot_model_snapshot = require_robot_model_snapshot(args);
    const auto assets = require_assets(args);
    const auto handle_stl_bytes = read_sealed_bytes(
      args.handle_stl, kExpectedHandleSha, 0400);
    verify_handle_mesh(projection->physical, handle_stl_bytes);
    auto robot_model = load_robot_model(
      robot_model_snapshot.urdf_xml, robot_model_snapshot.srdf_xml);
    reverify_robot_model_snapshot(robot_model_snapshot);
    planning_scene::PlanningScene physical_scene(robot_model);
    planning_scene::PlanningScene raw_planning_scene(robot_model);
    if (!physical_scene.usePlanningSceneMsg(projection->physical) ||
      !raw_planning_scene.usePlanningSceneMsg(raw_scene))
    {
      throw std::runtime_error("PLANNING_SCENE_LOAD");
    }
    const auto audit = audit_dynamic(
      physical_scene, raw_planning_scene, assets,
      projection->mask_conflict_pair_present, args.park_roll, args.static_only,
      args.tail_scan, args.diagnose_cube_door);
    atomic_write(
      args.output, evidence_json(
        manifest_sha256, *projection, robot_model_snapshot, audit));
    std::cout << "CANDIDATE3_MIDDLE_EXACT_AUDIT=PASS samples=" <<
      audit.samples << " minimum_environment_clearance_m=" <<
      std::setprecision(17) << audit.minimum_environment_clearance <<
      " minimum_left_handle_clearance_m=" <<
      audit.minimum_left_handle_clearance << std::endl;
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    const std::string status = std::string("ERROR:") + error.what();
    std::cerr << "CANDIDATE3_MIDDLE_EXACT_AUDIT=" << status << std::endl;
    if (!output_path.empty()) {
      try {
        atomic_write(
          output_path, failure_json(status, projection ? &*projection : nullptr));
      } catch (const std::exception &) {
      }
    }
    rclcpp::shutdown();
    return 3;
  }
}
