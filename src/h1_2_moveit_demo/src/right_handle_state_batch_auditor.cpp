#define RIGHT_HANDLE_PATH_EXACT_AUDITOR_NO_MAIN
#include "right_handle_path_exact_auditor.cpp"

#include <fstream>
#include <sstream>

namespace
{
struct BatchRow
{
  std::string id;
  double handle_angle{0.0};
  double torso{0.0};
  double phase_deg{0.0};
  double z_mm{0.0};
  double palm_tilt_deg{0.0};
  double side_tilt_deg{0.0};
  double outward_mm{0.0};
  double local_x_mm{0.0};
  double q1_mm{0.0};
  double q2_mm{0.0};
  std::array<double, 7U> arm{};
};

BatchRow parse_batch_row(const std::string & line, const std::size_t line_number)
{
  BatchRow row;
  std::string arm;
  std::istringstream stream(line);
  double handle_deg = 0.0;
  double torso_deg = 0.0;
  if (!(stream >> row.id >> handle_deg >> torso_deg >> row.phase_deg >> row.z_mm >>
    row.palm_tilt_deg >> row.side_tilt_deg >> row.outward_mm >> row.local_x_mm >>
    row.q1_mm >> row.q2_mm >> arm))
  {
    throw std::runtime_error("BATCH_ROW_SHAPE:" + std::to_string(line_number));
  }
  std::string extra;
  if (stream >> extra) {
    throw std::runtime_error("BATCH_ROW_EXTRA:" + std::to_string(line_number));
  }
  row.handle_angle = handle_deg * kPi / 180.0;
  row.torso = torso_deg * kPi / 180.0;
  std::istringstream arm_stream(arm);
  std::string field;
  std::size_t index = 0U;
  while (std::getline(arm_stream, field, ',')) {
    if (index >= row.arm.size()) {
      throw std::runtime_error("BATCH_ARM_WIDTH:" + std::to_string(line_number));
    }
    row.arm[index++] = std::stod(field);
  }
  const std::array<double, 10U> scalar_values{
    row.handle_angle, row.torso, row.phase_deg, row.z_mm, row.palm_tilt_deg,
    row.side_tilt_deg, row.outward_mm, row.local_x_mm, row.q1_mm, row.q2_mm};
  if (row.id.empty() || index != row.arm.size() ||
    !std::all_of(scalar_values.begin(), scalar_values.end(),
    [](const double value) {return std::isfinite(value);}) ||
    !std::all_of(row.arm.begin(), row.arm.end(),
    [](const double value) {return std::isfinite(value);}) ||
    row.handle_angle < -1e-12 || row.handle_angle > kPi / 2.0 + 1e-12 ||
    std::abs(row.torso) > kTorsoLimit + 1e-12)
  {
    throw std::runtime_error("BATCH_ROW_RANGE:" + std::to_string(line_number));
  }
  return row;
}

double signed_pair_distance(
  const planning_scene::PlanningScene & scene,
  const moveit::core::RobotState & state,
  const collision_detection::AllowedCollisionMatrix & pair_acm,
  const std::string & first,
  const std::string & second)
{
  collision_detection::DistanceRequest request;
  request.type = collision_detection::DistanceRequestTypes::SINGLE;
  request.enable_signed_distance = true;
  request.max_contacts_per_body = 1U;
  request.distance_threshold = std::numeric_limits<double>::infinity();
  request.acm = &pair_acm;
  collision_detection::DistanceResult result;
  scene.getCollisionEnvUnpadded()->distanceSelf(request, result, state);
  const auto & minimum = result.minimum_distance;
  const bool expected =
    (minimum.link_names[0] == first && minimum.link_names[1] == second) ||
    (minimum.link_names[0] == second && minimum.link_names[1] == first);
  if (!expected || !std::isfinite(minimum.distance)) {
    throw std::runtime_error("PAIR_DISTANCE_RESULT:" + first + ":" + second);
  }
  return minimum.distance;
}

Eigen::Isometry3d target_pose(
  const HandleGeometry & handle, const BatchRow & row)
{
  Eigen::Isometry3d grasp_q0 = Eigen::Isometry3d::Identity();
  grasp_q0.linear().col(0) = handle.q0_pose.linear().col(0);
  grasp_q0.linear().col(1) = -handle.q0_pose.linear().col(1);
  grasp_q0.linear().col(2) = -handle.q0_pose.linear().col(2);
  grasp_q0.linear() = grasp_q0.linear() *
    Eigen::AngleAxisd(
    row.phase_deg * kPi / 180.0,
    Eigen::Vector3d::UnitY()).toRotationMatrix() *
    Eigen::AngleAxisd(
    -row.palm_tilt_deg * kPi / 180.0,
    Eigen::Vector3d::UnitX()).toRotationMatrix() *
    Eigen::AngleAxisd(
    row.side_tilt_deg * kPi / 180.0,
    Eigen::Vector3d::UnitZ()).toRotationMatrix();
  grasp_q0.translation() = handle.q0_pose * Eigen::Vector3d(
    0.010, 0.0250, row.z_mm * 0.001) +
    row.outward_mm * 0.001 * handle.q0_pose.linear().col(1) +
    row.local_x_mm * 0.001 * grasp_q0.linear().col(0);
  Eigen::Isometry3d pivot_q0(handle.q0_pose);
  pivot_q0.translation() = handle.pivot;
  Eigen::Isometry3d pivot_at_angle(pivot_q0);
  pivot_at_angle.linear() = pivot_q0.linear() *
    Eigen::AngleAxisd(
    row.handle_angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
  return pivot_at_angle * pivot_q0.inverse() * grasp_q0;
}

std::string finite_json(const double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream output;
  output << std::setprecision(17) << value;
  return output.str();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto required = [&](const std::string & name) {
        const auto value = option(argc, argv, name);
        if (!value || value->empty()) {
          throw std::runtime_error("ARGUMENT_MISSING:" + name);
        }
        return *value;
      };
    const std::filesystem::path scene_path = required("--scene");
    const std::filesystem::path path_path = required("--path");
    const std::filesystem::path urdf_path = required("--urdf");
    const std::filesystem::path srdf_path = required("--srdf");
    const std::filesystem::path batch_path = required("--batch-file");
    const double park_left_roll = std::stod(required("--park-left-roll"));
    if (!std::isfinite(park_left_roll) || std::filesystem::is_symlink(batch_path)) {
      throw std::runtime_error("ARGUMENT_RANGE");
    }

    const auto cdr = read_bytes(scene_path);
    const auto raw_scene = deserialize_scene(cdr);
    const auto projection = project_physical_scene(raw_scene, cdr);
    if (!projection.unknown.empty() || !projection.handle_exact_retained) {
      throw std::runtime_error("SCENE_PROJECTION");
    }
    (void)load_path_asset(path_path);
    auto robot_model = load_robot_model(
      read_text_file(urdf_path), read_text_file(srdf_path));
    planning_scene::PlanningScene scene(robot_model);
    if (!scene.usePlanningSceneMsg(projection.physical)) {
      throw std::runtime_error("PLANNING_SCENE_LOAD");
    }
    verify_collision_geometry_fail_closed(scene);
    moveit::core::RobotState seam(scene.getCurrentState());
    verify_seam(seam);
    seam.setVariablePosition("left_shoulder_roll_joint", park_left_roll);
    seam.updateCollisionBodyTransforms();
    HandleGeometry handle = prepare_handle_geometry(scene);
    const auto * tcp_link = robot_model->getLinkModel("right_dex1_tcp");
    if (tcp_link == nullptr) {
      throw std::runtime_error("ROBOT_TCP");
    }

    auto acm = scene.getAllowedCollisionMatrix();
    const auto collision_links = robot_model->getLinkModelNamesWithCollisionGeometry();
    std::vector<std::string> left_links;
    std::vector<std::string> nonpad_nonleft_links;
    for (const auto & object : scene.getWorld()->getObjectIds()) {
      for (const auto & link : collision_links) {
        acm.setEntry(object, link, false);
      }
    }
    acm.setEntry("right_elbow_link", "torso_link", false);
    for (const auto & link : collision_links) {
      const bool pad = link == kRightHandlePads[0] || link == kRightHandlePads[1];
      if (link.rfind("left_", 0U) == 0U) {
        left_links.push_back(link);
      } else if (!pad) {
        nonpad_nonleft_links.push_back(link);
      }
    }
    for (const auto & pad : kRightHandlePads) {
      acm.setEntry(kHandleId, pad, true);
    }
    collision_detection::AllowedCollisionMatrix shoulder_acm(
      robot_model->getLinkModelNames(), true);
    shoulder_acm.setEntry("right_shoulder_yaw_link", "torso_link", false);
    collision_detection::AllowedCollisionMatrix elbow_acm(
      robot_model->getLinkModelNames(), true);
    elbow_acm.setEntry("right_elbow_link", "torso_link", false);
    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_request.max_contacts = 8U;
    collision_request.max_contacts_per_pair = 2U;
    collision_request.pad_self_collisions = false;
    auto world_request = collision_request;
    world_request.pad_environment_collisions = false;
    FclGeometryCache patch_cache;

    std::ifstream input(batch_path);
    if (!input) {
      throw std::runtime_error("BATCH_OPEN");
    }
    std::string line;
    std::size_t line_number = 0U;
    std::size_t evaluated = 0U;
    std::size_t passed = 0U;
    while (std::getline(input, line)) {
      ++line_number;
      const auto first = line.find_first_not_of(" \t\r");
      if (first == std::string::npos || line[first] == '#') {
        continue;
      }
      const BatchRow row = parse_batch_row(line, line_number);
      ++evaluated;
      const Eigen::Isometry3d expected = target_pose(handle, row);
      set_single_shape_pose(
        scene, kHandleId, handle.shape, handle_pose(handle, row.handle_angle));
      moveit::core::RobotState candidate(seam);
      candidate.setVariablePosition("torso_joint", row.torso);
      for (std::size_t index = 0U; index < row.arm.size(); ++index) {
        candidate.setVariablePosition(kRightArmVariables[index], row.arm[index]);
      }
      candidate.setVariablePosition(kRightFingerVariables[0], row.q1_mm * 0.001);
      candidate.setVariablePosition(kRightFingerVariables[1], row.q2_mm * 0.001);
      candidate.updateCollisionBodyTransforms();

      const double margin = arm_margin(candidate);
      const Eigen::Isometry3d actual = candidate.getGlobalLinkTransform(tcp_link);
      const double position_error = (actual.translation() - expected.translation()).norm();
      const double orientation_error = Eigen::AngleAxisd(
        expected.linear().transpose() * actual.linear()).angle();
      const double shoulder_distance = signed_pair_distance(
        scene, candidate, shoulder_acm, "right_shoulder_yaw_link", "torso_link");
      const double elbow_distance = signed_pair_distance(
        scene, candidate, elbow_acm, "right_elbow_link", "torso_link");
      collision_detection::CollisionResult self_result;
      scene.checkSelfCollision(collision_request, self_result, candidate, acm);
      collision_detection::CollisionResult world_result;
      scene.checkCollision(world_request, world_result, candidate, acm);
      const auto nonpad = handle_distance(
        scene, candidate, acm, nonpad_nonleft_links, 0.020);
      const auto left = handle_distance(scene, candidate, acm, left_links, 0.050);
      const Eigen::Isometry3d global_handle = handle_pose(handle, row.handle_angle);
      const auto pad1 = pad_patch(
        patch_cache, candidate, kRightHandlePads[0], handle, global_handle);
      const auto pad2 = pad_patch(
        patch_cache, candidate, kRightHandlePads[1], handle, global_handle);
      double radius1 = 0.0;
      double radius2 = 0.0;
      bool valid1 = valid_patch(pad1, radius1);
      bool valid2 = valid_patch(pad2, radius2);
      if (valid1) {
        radius1 = (pad1.center() - global_handle * Eigen::Vector3d(
          0.009999668689, -0.000027284668, 0.119003896682)).norm();
        valid1 = radius1 + 1e-12 >= kPatchRadius;
      }
      if (valid2) {
        radius2 = (pad2.center() - global_handle * Eigen::Vector3d(
          0.009999668689, -0.000027284668, 0.119003896682)).norm();
        valid2 = radius2 + 1e-12 >= kPatchRadius;
      }

      std::string reason = "PASS";
      if (!candidate.satisfiesBounds(1e-9) || margin + 1e-12 < kArmMargin) {
        reason = "BOUNDS";
      } else if (position_error > 0.00010 || orientation_error > 0.1 * kPi / 180.0) {
        reason = "TCP";
      } else if (self_result.collision) {
        reason = "SELF:" + collision_pair(self_result);
      } else if (world_result.collision) {
        reason = "WORLD:" + collision_pair(world_result);
      } else if (nonpad.first + 1e-12 < kNonpadHandleClearance) {
        reason = "NONPAD:" + nonpad.second;
      } else if (left.first + 1e-12 < kLeftHandleClearance) {
        reason = "LEFT:" + left.second;
      } else if (!valid1) {
        reason = "PAD1";
      }
      const bool pass = reason == "PASS";
      passed += pass ? 1U : 0U;
      std::cout << std::boolalpha << std::setprecision(17)
                << "BATCH_STATE id=" << row.id
                << " pass=" << pass
                << " reason=" << reason
                << " handle_deg=" << row.handle_angle * 180.0 / kPi
                << " torso_deg=" << row.torso * 180.0 / kPi
                << " arm_margin_deg=" << margin * 180.0 / kPi
                << " tcp_mm=" << position_error * 1000.0
                << " tcp_deg=" << orientation_error * 180.0 / kPi
                << " shoulder_torso_signed_mm=" << shoulder_distance * 1000.0
                << " elbow_torso_signed_mm=" << elbow_distance * 1000.0
                << " min_pair_signed_mm=" <<
        std::min(shoulder_distance, elbow_distance) * 1000.0
                << " nonpad_handle_mm=" << nonpad.first * 1000.0
                << " left_handle_mm=" << left.first * 1000.0
                << " pad1_valid=" << valid1
                << " pad2_valid=" << valid2
                << " pad1_span_mm=" << pad1.span() * 1000.0
                << " pad1_station_mm=" << pad1.station() * 1000.0
                << " pad1_radius_mm=" << finite_json(radius1 * 1000.0)
                << '\n';
    }
    std::cout << "BATCH_SUMMARY evaluated=" << evaluated << " passed=" << passed << '\n';
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "RIGHT_HANDLE_STATE_BATCH_AUDITOR=ERROR:" << error.what() << '\n';
    rclcpp::shutdown();
    return 3;
  }
}
