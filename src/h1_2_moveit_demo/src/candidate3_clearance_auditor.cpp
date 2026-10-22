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
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>

namespace
{
constexpr double kRequiredClearance = 0.010;
constexpr double kQueryMargin = 1e-6;
constexpr char kSchema[] = "h1_candidate3_cpp_clearance_audit/v1";

struct Arguments
{
  std::filesystem::path scene;
  std::filesystem::path plan;
  std::filesystem::path urdf;
  std::filesystem::path srdf;
  std::filesystem::path output;
  std::size_t sample_limit{std::numeric_limits<std::size_t>::max()};
};

struct AuditResult
{
  bool collision_free{true};
  bool clearance_passed{true};
  bool exact_distance_observed{false};
  std::size_t samples{0U};
  double minimum_clearance{kRequiredClearance + kQueryMargin};
  std::string status{"PASS"};
  std::string first_body;
  std::string second_body;
  std::string stage;
  std::size_t stage_index{0U};
  std::size_t message_index{0U};
  std::size_t point_index{0U};
  double nearest_first_point[3]{0.0, 0.0, 0.0};
  double nearest_second_point[3]{0.0, 0.0, 0.0};
};

std::string read_text(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open " + path.string());
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

std::vector<std::uint8_t> read_bytes(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) {
    throw std::runtime_error("cannot open " + path.string());
  }
  const auto size = stream.tellg();
  if (size <= 0) {
    throw std::runtime_error("empty input " + path.string());
  }
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
  stream.seekg(0);
  stream.read(
    reinterpret_cast<char *>(bytes.data()),
    static_cast<std::streamsize>(bytes.size()));
  if (!stream) {
    throw std::runtime_error("short read " + path.string());
  }
  return bytes;
}

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

Arguments arguments(const int argc, char ** argv)
{
  const auto required = [&](const std::string & name) {
      const auto value = option(argc, argv, name);
      if (!value || value->empty()) {
        throw std::runtime_error("missing argument " + name);
      }
      return std::filesystem::path(*value);
    };
  Arguments result{
    required("--scene"), required("--plan"), required("--urdf"),
    required("--srdf"), required("--output")};
  const auto sample_limit = option(argc, argv, "--sample-limit");
  if (sample_limit) {
    const auto parsed = std::stoull(*sample_limit);
    if (parsed == 0U) {
      throw std::runtime_error("sample limit must be positive");
    }
    result.sample_limit = static_cast<std::size_t>(parsed);
  }
  return result;
}

moveit_msgs::msg::PlanningScene deserialize_scene(
  const std::filesystem::path & path)
{
  const auto bytes = read_bytes(path);
  rclcpp::SerializedMessage serialized(bytes.size());
  auto & raw = serialized.get_rcl_serialized_message();
  if (raw.buffer_capacity < bytes.size()) {
    throw std::runtime_error("serialized buffer capacity");
  }
  std::memcpy(raw.buffer, bytes.data(), bytes.size());
  raw.buffer_length = bytes.size();
  moveit_msgs::msg::PlanningScene message;
  rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serialization;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

moveit::core::RobotModelPtr load_robot_model(
  const std::filesystem::path & urdf_path,
  const std::filesystem::path & srdf_path)
{
  const auto urdf_model = urdf::parseURDF(read_text(urdf_path));
  if (!urdf_model) {
    throw std::runtime_error("URDF_PARSE");
  }
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initString(*urdf_model, read_text(srdf_path))) {
    throw std::runtime_error("SRDF_PARSE");
  }
  auto model = std::make_shared<moveit::core::RobotModel>(
    urdf_model, srdf_model);
  if (model->getName() != "h1_2") {
    throw std::runtime_error("ROBOT_MODEL_NAME");
  }
  return model;
}

std::vector<std::string> strings(const YAML::Node & sequence)
{
  if (!sequence.IsSequence() || sequence.size() == 0U) {
    throw std::runtime_error("EMPTY_JOINT_NAMES");
  }
  std::vector<std::string> output;
  output.reserve(sequence.size());
  for (const auto & value : sequence) {
    output.push_back(value.as<std::string>());
  }
  return output;
}

std::vector<double> doubles(const YAML::Node & sequence)
{
  if (!sequence.IsSequence()) {
    throw std::runtime_error("POSITIONS_NOT_SEQUENCE");
  }
  std::vector<double> output;
  output.reserve(sequence.size());
  for (const auto & value : sequence) {
    const double number = value.as<double>();
    if (!std::isfinite(number)) {
      throw std::runtime_error("NONFINITE_POSITION");
    }
    output.push_back(number);
  }
  return output;
}

void remove_stage_objects(
  planning_scene::PlanningScene & scene, const std::size_t stage_index,
  const bool post_nav_right_open)
{
  static const std::map<std::size_t, std::vector<std::string>> legacy_removals{
    {0U, {"incubator_handle_exact"}},
    {2U, {"incubator_door_exact"}},
    {3U, {"green_cube_dynamic_exact", "green_cube_10mm_guard"}},
  };
  static const std::map<std::size_t, std::vector<std::string>> post_nav_removals{
    {0U, {"incubator_handle_exact"}},
    {2U, {"incubator_door_exact"}},
  };
  const auto & removals = post_nav_right_open ?
    post_nav_removals : legacy_removals;
  const auto found = removals.find(stage_index);
  if (found == removals.end()) {
    return;
  }
  for (const auto & id : found->second) {
    if (!scene.getWorld()->hasObject(id)) {
      throw std::runtime_error("WORLD_REMOVE_MISSING:" + id);
    }
    scene.getWorldNonConst()->removeObject(id);
  }
}

void record_failure(
  AuditResult & output, const std::string & status,
  const std::string & stage, const std::size_t stage_index,
  const std::size_t message_index, const std::size_t point_index)
{
  output.status = status;
  output.stage = stage;
  output.stage_index = stage_index;
  output.message_index = message_index;
  output.point_index = point_index;
}

AuditResult audit(
  planning_scene::PlanningScene & scene, const YAML::Node & plan,
  const std::size_t sample_limit)
{
  if (!plan || plan["candidate3_only"].as<bool>() != true ||
    plan["contains_partial"].as<bool>() != false ||
    plan["door_arc_target_count"].as<std::size_t>() != 601U)
  {
    throw std::runtime_error("PLAN_CONTRACT");
  }
  const auto topology_node = plan["audit_topology"];
  const std::string topology = topology_node ?
    topology_node.as<std::string>() : std::string{};
  const bool post_nav_right_open =
    topology == "post_nav_right_handle_open/v1" ||
    topology == "post_nav_right_handle_open_roundtrip_audit/v1";
  if (topology_node && !post_nav_right_open) {
    throw std::runtime_error("AUDIT_TOPOLOGY");
  }
  const auto stages = plan["stages"];
  const std::size_t expected_stage_count =
    topology == "post_nav_right_handle_open_roundtrip_audit/v1" ? 6U :
    (post_nav_right_open ? 3U : 7U);
  if (!stages.IsSequence() || stages.size() != expected_stage_count) {
    throw std::runtime_error("STAGE_COUNT");
  }
  auto state = scene.getCurrentState();
  auto collision_acm = scene.getAllowedCollisionMatrix();
  auto distance_acm = collision_acm;
  for (const auto & link : scene.getRobotModel()->getLinkModelNamesWithCollisionGeometry()) {
    // These objects already encode a 10 mm expansion. Keep them strict in
    // collision checking, but do not count the same margin twice here.
    distance_acm.setEntry("environment_table_10mm_margin", link, true);
    distance_acm.setEntry("green_cube_10mm_guard", link, true);
  }

  collision_detection::CollisionRequest self_request;
  self_request.contacts = true;
  self_request.max_contacts = 1U;
  self_request.max_contacts_per_pair = 1U;
  self_request.pad_self_collisions = false;
  collision_detection::CollisionRequest world_request = self_request;
  world_request.pad_environment_collisions = false;

  collision_detection::DistanceRequest distance_request;
  distance_request.type = collision_detection::DistanceRequestTypes::SINGLE;
  distance_request.enable_nearest_points = true;
  distance_request.enable_signed_distance = true;
  distance_request.max_contacts_per_body = 1U;
  distance_request.distance_threshold = kRequiredClearance + kQueryMargin;
  distance_request.acm = &distance_acm;

  AuditResult output;
  for (std::size_t stage_index = 0U; stage_index < stages.size(); ++stage_index) {
    remove_stage_objects(scene, stage_index, post_nav_right_open);
    const auto stage = stages[stage_index];
    const auto stage_name = stage["name"].as<std::string>();
    const auto messages = stage["messages"];
    if (!messages.IsSequence() || messages.size() == 0U) {
      throw std::runtime_error("EMPTY_STAGE:" + stage_name);
    }
    for (std::size_t message_index = 0U; message_index < messages.size(); ++message_index) {
      const auto message = messages[message_index];
      const auto names = strings(message["joint_names"]);
      const auto points = message["points"];
      if (!points.IsSequence() || points.size() == 0U) {
        throw std::runtime_error("EMPTY_MESSAGE:" + stage_name);
      }
      for (std::size_t point_index = 0U; point_index < points.size(); ++point_index) {
        const auto positions = doubles(points[point_index]["positions"]);
        if (positions.size() != names.size()) {
          throw std::runtime_error("JOINT_WIDTH:" + stage_name);
        }
        state.setVariablePositions(names, positions);
        state.updateCollisionBodyTransforms();
        ++output.samples;
        if (!state.satisfiesBounds(1e-6)) {
          output.collision_free = false;
          output.clearance_passed = false;
          record_failure(
            output, "JOINT_BOUNDS", stage_name, stage_index,
            message_index, point_index);
          return output;
        }

        collision_detection::CollisionResult self_result;
        scene.checkSelfCollision(
          self_request, self_result, state, collision_acm);
        if (self_result.collision) {
          if (!self_result.contacts.empty()) {
            output.first_body = self_result.contacts.begin()->first.first;
            output.second_body = self_result.contacts.begin()->first.second;
          }
          output.collision_free = false;
          output.clearance_passed = false;
          record_failure(
            output, "COLLISION:SELF", stage_name, stage_index,
            message_index, point_index);
          return output;
        }

        collision_detection::CollisionResult world_result;
        scene.getCollisionEnvUnpadded()->checkRobotCollision(
          world_request, world_result, state, collision_acm);
        if (world_result.collision) {
          if (!world_result.contacts.empty()) {
            output.first_body = world_result.contacts.begin()->first.first;
            output.second_body = world_result.contacts.begin()->first.second;
          }
          output.collision_free = false;
          output.clearance_passed = false;
          record_failure(
            output, "COLLISION:WORLD", stage_name, stage_index,
            message_index, point_index);
          return output;
        }

        collision_detection::DistanceResult distance_result;
        scene.getCollisionEnvUnpadded()->distanceRobot(
          distance_request, distance_result, state);
        for (const auto & [pair, values] : distance_result.distances) {
          (void)pair;
          for (const auto & value : values) {
            if (!std::isfinite(value.distance)) {
              continue;
            }
            output.exact_distance_observed = true;
            if (value.distance < output.minimum_clearance) {
              output.minimum_clearance = value.distance;
              output.first_body = value.link_names[0];
              output.second_body = value.link_names[1];
              for (std::size_t axis = 0U; axis < 3U; ++axis) {
                output.nearest_first_point[axis] = value.nearest_points[0][axis];
                output.nearest_second_point[axis] = value.nearest_points[1][axis];
              }
              output.stage = stage_name;
              output.stage_index = stage_index;
              output.message_index = message_index;
              output.point_index = point_index;
            }
          }
        }
        if (output.minimum_clearance + 1e-9 < kRequiredClearance) {
          output.clearance_passed = false;
          output.status = "CLEARANCE";
        }
        if (output.samples >= sample_limit) {
          return output;
        }
      }
    }
  }
  return output;
}

void write_result(
  const std::filesystem::path & output_path, const AuditResult & result,
  const std::string & scene_name, const std::string & plan_hash,
  const std::string & status_override = {})
{
  const auto temporary = output_path.string() + ".tmp";
  std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("cannot create output");
  }
  const std::string status = status_override.empty() ? result.status : status_override;
  stream << std::boolalpha << std::setprecision(17) <<
    "{\n"
    "  \"schema\": \"" << kSchema << "\",\n"
    "  \"scene_name\": \"" << json_escape(scene_name) << "\",\n"
    "  \"plan_hash\": \"" << json_escape(plan_hash) << "\",\n"
    "  \"samples\": " << result.samples << ",\n"
    "  \"collision_free\": " << result.collision_free << ",\n"
    "  \"clearance_passed\": " << result.clearance_passed << ",\n"
    "  \"minimum_environment_clearance_m\": " << result.minimum_clearance << ",\n"
    "  \"minimum_is_lower_bound\": " << !result.exact_distance_observed << ",\n"
    "  \"nearest_first_body\": \"" << json_escape(result.first_body) << "\",\n"
    "  \"nearest_second_body\": \"" << json_escape(result.second_body) << "\",\n"
    "  \"nearest_first_point\": [" << result.nearest_first_point[0] << ", " <<
    result.nearest_first_point[1] << ", " << result.nearest_first_point[2] << "],\n"
    "  \"nearest_second_point\": [" << result.nearest_second_point[0] << ", " <<
    result.nearest_second_point[1] << ", " << result.nearest_second_point[2] << "],\n"
    "  \"stage\": \"" << json_escape(result.stage) << "\",\n"
    "  \"stage_index\": " << result.stage_index << ",\n"
    "  \"message_index\": " << result.message_index << ",\n"
    "  \"point_index\": " << result.point_index << ",\n"
    "  \"valid\": " << (result.collision_free && result.clearance_passed) << ",\n"
    "  \"status\": \"" << json_escape(status) << "\"\n"
    "}\n";
  stream.flush();
  if (!stream) {
    throw std::runtime_error("output write failed");
  }
  stream.close();
  std::filesystem::rename(temporary, output_path);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  std::filesystem::path output_path;
  try {
    const auto args = arguments(argc, argv);
    output_path = args.output;
    const auto root = YAML::LoadFile(args.plan.string());
    const auto plan = root["plan"];
    const auto plan_hash = root["plan_hash"].as<std::string>();
    const auto scene_message = deserialize_scene(args.scene);
    const auto expected_name =
      "h1_scene_epoch:" + std::to_string(plan["scene_epoch_id"].as<std::uint64_t>()) +
      ":" + plan["scene_hash"].as<std::string>();
    if (scene_message.is_diff || scene_message.name != expected_name) {
      throw std::runtime_error("SCENE_EPOCH_BINDING");
    }
    auto robot_model = load_robot_model(args.urdf, args.srdf);
    planning_scene::PlanningScene scene(robot_model);
    if (!scene.usePlanningSceneMsg(scene_message)) {
      throw std::runtime_error("PLANNING_SCENE");
    }
    const auto result = audit(scene, plan, args.sample_limit);
    write_result(args.output, result, scene_message.name, plan_hash);
    std::cout << "CANDIDATE3_CPP_CLEARANCE=" << result.status <<
      " samples=" << result.samples <<
      " minimum_m=" << std::setprecision(17) << result.minimum_clearance <<
      std::endl;
    const int exit_code = result.collision_free && result.clearance_passed ? 0 : 2;
    rclcpp::shutdown();
    return exit_code;
  } catch (const std::exception & error) {
    std::cerr << "CANDIDATE3_CPP_CLEARANCE=ERROR " << error.what() << std::endl;
    if (!output_path.empty()) {
      try {
        AuditResult failed;
        failed.collision_free = false;
        failed.clearance_passed = false;
        failed.status = "ERROR";
        write_result(output_path, failed, "", "", std::string("ERROR:") + error.what());
      } catch (const std::exception &) {
      }
    }
    rclcpp::shutdown();
    return 3;
  }
}
