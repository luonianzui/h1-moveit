#define RIGHT_HANDLE_PATH_EXACT_AUDITOR_NO_MAIN
#include "right_handle_path_exact_auditor.cpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <pluginlib/class_loader.hpp>
#include <random>

namespace
{
struct GeometryOverride
{
  std::string id;
  double phase_deg{0.0};
  double z_mm{0.0};
  double palm_down_tilt_deg{0.0};
  double side_tilt_deg{0.0};
  double outward_mm{0.0};
  double local_x_mm{0.0};
  double q1_mm{0.0};
  double q2_mm{0.0};
};

geometry_msgs::msg::Pose eigen_pose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose output;
  output.position.x = transform.translation().x();
  output.position.y = transform.translation().y();
  output.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.linear());
  output.orientation.x = quaternion.x();
  output.orientation.y = quaternion.y();
  output.orientation.z = quaternion.z();
  output.orientation.w = quaternion.w();
  return output;
}

std::string arm_json(const moveit::core::RobotState & state)
{
  std::array<double, 7U> values{};
  for (std::size_t index = 0U; index < values.size(); ++index) {
    values[index] = state.getVariablePosition(kRightArmVariables[index]);
  }
  return vector7_json(values);
}

double maximum_arm_difference(
  const moveit::core::RobotState & first,
  const moveit::core::RobotState & second)
{
  double output = 0.0;
  for (const auto & variable : kRightArmVariables) {
    output = std::max(output, std::abs(
      first.getVariablePosition(variable) -
      second.getVariablePosition(variable)));
  }
  return output;
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
  request.enable_nearest_points = false;
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
    const double park_left_roll = std::stod(required("--park-left-roll"));
    const double handle_angle = std::stod(required("--handle-angle-deg")) * kPi / 180.0;
    const double torso = std::stod(required("--torso-deg")) * kPi / 180.0;
    const std::size_t samples = std::stoull(required("--samples"));
    const std::size_t maximum_results = std::stoull(required("--maximum-results"));
    const std::uint64_t seed = std::stoull(required("--seed"));
    const bool report_best_failed = flag(argc, argv, "--report-best-failed");
    const auto parse_arm = [&](const std::string & name) {
      std::array<double, 7U> parsed{};
      const auto value = option(argc, argv, name);
      if (!value || value->empty()) {
        throw std::runtime_error("ARGUMENT_MISSING:" + name);
      }
      std::istringstream stream(*value);
      std::string field;
      std::size_t index = 0U;
      while (std::getline(stream, field, ',')) {
        if (index >= parsed.size()) {
          throw std::runtime_error("ARGUMENT_VECTOR_WIDTH:" + name);
        }
        parsed[index++] = std::stod(field);
      }
      if (index != parsed.size() ||
        !std::all_of(parsed.begin(), parsed.end(),
        [](const double position) {return std::isfinite(position);}))
      {
        throw std::runtime_error("ARGUMENT_VECTOR:" + name);
      }
      return parsed;
    };
    std::optional<std::array<double, 7U>> test_arm;
    if (option(argc, argv, "--test-arm")) {
      test_arm = parse_arm("--test-arm");
    }
    std::optional<std::array<double, 7U>> ik_seed_arm;
    if (option(argc, argv, "--ik-seed-arm")) {
      ik_seed_arm = parse_arm("--ik-seed-arm");
    }
    if (test_arm && ik_seed_arm) {
      throw std::runtime_error("ARGUMENT_ARM_MODE_EXCLUSIVE");
    }
    std::optional<GeometryOverride> geometry_override;
    const std::array<std::string, 8U> geometry_options{
      "--phase-deg", "--z-mm", "--palm-down-tilt-deg",
      "--side-tilt-deg", "--outward-mm", "--local-x-mm",
      "--q1-mm", "--q2-mm"};
    const bool any_geometry = std::any_of(
      geometry_options.begin(), geometry_options.end(),
      [&](const auto & name) {return option(argc, argv, name).has_value();});
    const bool all_geometry = std::all_of(
      geometry_options.begin(), geometry_options.end(),
      [&](const auto & name) {return option(argc, argv, name).has_value();});
    if (any_geometry && !all_geometry) {
      throw std::runtime_error("ARGUMENT_INCOMPLETE_GEOMETRY");
    }
    if (all_geometry) {
      GeometryOverride geometry;
      geometry.id = option(argc, argv, "--geometry-id").value_or("override");
      geometry.phase_deg = std::stod(required("--phase-deg"));
      geometry.z_mm = std::stod(required("--z-mm"));
      geometry.palm_down_tilt_deg = std::stod(required("--palm-down-tilt-deg"));
      geometry.side_tilt_deg = std::stod(required("--side-tilt-deg"));
      geometry.outward_mm = std::stod(required("--outward-mm"));
      geometry.local_x_mm = std::stod(required("--local-x-mm"));
      geometry.q1_mm = std::stod(required("--q1-mm"));
      geometry.q2_mm = std::stod(required("--q2-mm"));
      geometry_override = geometry;
    }
    if (!std::isfinite(handle_angle) || handle_angle < -1e-12 ||
      handle_angle > kPi / 2.0 + 1e-12 ||
      std::abs(torso) > kTorsoLimit + 1e-12 || samples == 0U ||
      samples > 1000000U || maximum_results == 0U || maximum_results > 1000U)
    {
      throw std::runtime_error("ARGUMENT_RANGE");
    }

    const auto cdr = read_bytes(scene_path);
    const auto raw_scene = deserialize_scene(cdr);
    const auto projection = project_physical_scene(raw_scene, cdr);
    if (!projection.unknown.empty() || !projection.handle_exact_retained) {
      throw std::runtime_error("SCENE_PROJECTION");
    }
    const auto asset = load_path_asset(path_path);
    const auto row_iterator = std::min_element(
      asset.rows.begin(), asset.rows.end(),
      [&](const PathRow & first, const PathRow & second) {
        return std::abs(first.handle_angle - handle_angle) <
               std::abs(second.handle_angle - handle_angle);
      });
    if (row_iterator == asset.rows.end()) {
      throw std::runtime_error("PATH_EMPTY");
    }
    const PathRow & row = *row_iterator;
    if (!geometry_override && std::abs(row.handle_angle - handle_angle) > 1e-9) {
      throw std::runtime_error("PATH_HANDLE_ANGLE_NOT_FOUND");
    }
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
    moveit::core::RobotState reference = row_state(seam, row, false);
    reference.updateCollisionBodyTransforms();
    HandleGeometry handle = prepare_handle_geometry(scene);
    const auto * base_link = robot_model->getLinkModel("torso_link");
    const auto * tcp_link = robot_model->getLinkModel("right_dex1_tcp");
    const auto * group = robot_model->getJointModelGroup("right_arm");
    if (!base_link || !tcp_link || !group ||
      group->getVariableCount() != 7U)
    {
      throw std::runtime_error("ROBOT_GROUP_CONTRACT");
    }
    Eigen::Isometry3d target_global = reference.getGlobalLinkTransform(tcp_link);
    double q1_m = row.q1_m;
    double q2_m = row.q2_m;
    if (geometry_override) {
      const auto & geometry = *geometry_override;
      Eigen::Isometry3d grasp_q0 = Eigen::Isometry3d::Identity();
      grasp_q0.linear().col(0) = handle.q0_pose.linear().col(0);
      grasp_q0.linear().col(1) = -handle.q0_pose.linear().col(1);
      grasp_q0.linear().col(2) = -handle.q0_pose.linear().col(2);
      grasp_q0.linear() = grasp_q0.linear() *
        Eigen::AngleAxisd(
        geometry.phase_deg * kPi / 180.0,
        Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(
        -geometry.palm_down_tilt_deg * kPi / 180.0,
        Eigen::Vector3d::UnitX()).toRotationMatrix() *
        Eigen::AngleAxisd(
        geometry.side_tilt_deg * kPi / 180.0,
        Eigen::Vector3d::UnitZ()).toRotationMatrix();
      grasp_q0.translation() = handle.q0_pose * Eigen::Vector3d(
        0.010, 0.0250, geometry.z_mm * 0.001) +
        geometry.outward_mm * 0.001 * handle.q0_pose.linear().col(1) +
        geometry.local_x_mm * 0.001 * grasp_q0.linear().col(0);
      Eigen::Isometry3d pivot_q0(handle.q0_pose);
      pivot_q0.translation() = handle.pivot;
      Eigen::Isometry3d pivot_at_angle(pivot_q0);
      pivot_at_angle.linear() = pivot_q0.linear() *
        Eigen::AngleAxisd(
        handle_angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
      target_global = pivot_at_angle * pivot_q0.inverse() * grasp_q0;
      q1_m = geometry.q1_mm * 0.001;
      q2_m = geometry.q2_mm * 0.001;
    }
    moveit::core::RobotState torso_state(seam);
    torso_state.setVariablePosition("torso_joint", torso);
    torso_state.updateCollisionBodyTransforms();
    const Eigen::Isometry3d target_in_base =
      torso_state.getGlobalLinkTransform(base_link).inverse() * target_global;

    const auto node = std::make_shared<rclcpp::Node>("right_handle_ik_branch_search");
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
      "moveit_core", "kinematics::KinematicsBase");
    auto solver = loader.createSharedInstance(
      "kdl_kinematics_plugin/KDLKinematicsPlugin");
    if (!solver || !solver->initialize(
        node, *robot_model, "right_arm", "torso_link",
        {"right_dex1_tcp"}, 0.002))
    {
      throw std::runtime_error("KDL_INITIALIZE");
    }
    const auto & solver_names = solver->getJointNames();
    if (solver_names.size() != 7U) {
      throw std::runtime_error("KDL_JOINT_COUNT");
    }

    set_single_shape_pose(
      scene, kHandleId, handle.shape, handle_pose(handle, handle_angle));
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
    collision_detection::AllowedCollisionMatrix shoulder_torso_acm(
      robot_model->getLinkModelNames(), true);
    shoulder_torso_acm.setEntry(
      "right_shoulder_yaw_link", "torso_link", false);
    collision_detection::AllowedCollisionMatrix elbow_torso_acm(
      robot_model->getLinkModelNames(), true);
    elbow_torso_acm.setEntry("right_elbow_link", "torso_link", false);
    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_request.max_contacts = 8U;
    collision_request.max_contacts_per_pair = 2U;
    collision_request.pad_self_collisions = false;
    auto world_request = collision_request;
    world_request.pad_environment_collisions = false;

    std::mt19937_64 random(seed);
    std::vector<std::uniform_real_distribution<double>> distributions;
    distributions.reserve(solver_names.size());
    for (const auto & variable : solver_names) {
      const auto & bounds = robot_model->getVariableBounds(variable);
      if (!bounds.position_bounded_) {
        throw std::runtime_error("ARM_UNBOUNDED:" + variable);
      }
      distributions.emplace_back(
        bounds.min_position_ + kArmMargin,
        bounds.max_position_ - kArmMargin);
    }

    std::vector<moveit::core::RobotState> accepted;
    std::size_t ik_solved = 0U;
    std::size_t bounds_rejected = 0U;
    std::size_t self_rejected = 0U;
    std::size_t world_rejected = 0U;
    std::size_t residual_rejected = 0U;
    std::size_t clearance_rejected = 0U;
    std::size_t patch_rejected = 0U;
    std::map<std::string, std::size_t> self_pairs;
    std::map<std::string, std::size_t> world_pairs;
    FclGeometryCache patch_cache;
    std::optional<moveit::core::RobotState> best_failed_state;
    double best_failed_score = -std::numeric_limits<double>::infinity();
    double best_failed_shoulder_torso = -std::numeric_limits<double>::infinity();
    double best_failed_elbow_torso = -std::numeric_limits<double>::infinity();
    if (geometry_override) {
      const auto & geometry = *geometry_override;
      std::cout << std::setprecision(17)
                << "IK_BRANCH_GEOMETRY id=" << geometry.id
                << " phase_deg=" << geometry.phase_deg
                << " z_mm=" << geometry.z_mm
                << " tilt_deg=" << geometry.palm_down_tilt_deg
                << " side_tilt_deg=" << geometry.side_tilt_deg
                << " outward_mm=" << geometry.outward_mm
                << " local_x_mm=" << geometry.local_x_mm
                << " q1_mm=" << geometry.q1_mm
                << " q2_mm=" << geometry.q2_mm << '\n';
    }
    const geometry_msgs::msg::Pose target_pose = eigen_pose(target_in_base);
    const std::size_t attempts = (test_arm || ik_seed_arm) ? 1U : samples;
    for (std::size_t attempt = 0U;
      attempt < attempts && accepted.size() < maximum_results; ++attempt)
    {
      moveit::core::RobotState candidate(torso_state);
      if (test_arm) {
        for (std::size_t index = 0U; index < test_arm->size(); ++index) {
          candidate.setVariablePosition(kRightArmVariables[index], (*test_arm)[index]);
        }
      } else {
        std::vector<double> ik_seed(solver_names.size(), 0.0);
        for (std::size_t index = 0U; index < ik_seed.size(); ++index) {
          ik_seed[index] = ik_seed_arm ?
            (*ik_seed_arm)[index] : distributions[index](random);
        }
        std::vector<double> solution;
        moveit_msgs::msg::MoveItErrorCodes error;
        if (!solver->searchPositionIK(
            target_pose, ik_seed, 0.010, solution, error) ||
          error.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
          solution.size() != solver_names.size())
        {
          continue;
        }
        ++ik_solved;
        for (std::size_t index = 0U; index < solution.size(); ++index) {
          candidate.setVariablePosition(solver_names[index], solution[index]);
        }
      }
      candidate.setVariablePosition(kRightFingerVariables[0], q1_m);
      candidate.setVariablePosition(kRightFingerVariables[1], q2_m);
      candidate.updateCollisionBodyTransforms();
      if (!candidate.satisfiesBounds(1e-9) ||
        arm_margin(candidate) + 1e-12 < kArmMargin)
      {
        ++bounds_rejected;
        continue;
      }
      const Eigen::Isometry3d actual = candidate.getGlobalLinkTransform(tcp_link);
      const double position_error =
        (actual.translation() - target_global.translation()).norm();
      const double orientation_error = Eigen::AngleAxisd(
        target_global.linear().transpose() * actual.linear()).angle();
      if (position_error > 0.00010 || orientation_error > 0.1 * kPi / 180.0) {
        ++residual_rejected;
        continue;
      }
      double shoulder_torso_distance = std::numeric_limits<double>::infinity();
      double elbow_torso_distance = std::numeric_limits<double>::infinity();
      if (report_best_failed) {
        shoulder_torso_distance = signed_pair_distance(
          scene, candidate, shoulder_torso_acm,
          "right_shoulder_yaw_link", "torso_link");
        elbow_torso_distance = signed_pair_distance(
          scene, candidate, elbow_torso_acm,
          "right_elbow_link", "torso_link");
        const double score = std::min(
          shoulder_torso_distance, elbow_torso_distance);
        if (score > best_failed_score) {
          best_failed_score = score;
          best_failed_shoulder_torso = shoulder_torso_distance;
          best_failed_elbow_torso = elbow_torso_distance;
          best_failed_state = candidate;
        }
      }
      collision_detection::CollisionResult self_result;
      scene.checkSelfCollision(collision_request, self_result, candidate, acm);
      if (self_result.collision) {
        ++self_rejected;
        ++self_pairs[collision_pair(self_result)];
        continue;
      }
      collision_detection::CollisionResult world_result;
      scene.checkCollision(world_request, world_result, candidate, acm);
      if (world_result.collision) {
        ++world_rejected;
        ++world_pairs[collision_pair(world_result)];
        continue;
      }
      const auto nonpad = handle_distance(
        scene, candidate, acm, nonpad_nonleft_links, 0.020);
      const auto left = handle_distance(
        scene, candidate, acm, left_links, 0.050);
      if (nonpad.first + 1e-12 < kNonpadHandleClearance ||
        left.first + 1e-12 < kLeftHandleClearance)
      {
        ++clearance_rejected;
        continue;
      }
      const Eigen::Isometry3d global_handle = handle_pose(handle, handle_angle);
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
      if (!valid1 && !valid2) {
        ++patch_rejected;
        continue;
      }
      bool duplicate = false;
      for (const auto & prior : accepted) {
        if (maximum_arm_difference(prior, candidate) <= 0.25 * kPi / 180.0) {
          duplicate = true;
          break;
        }
      }
      if (duplicate) {
        continue;
      }
      std::cout << std::setprecision(17)
                << "IK_BRANCH_CANDIDATE index=" << accepted.size()
                << " attempt=" << attempt
                << " geometry=" <<
        (geometry_override ? geometry_override->id : row.geometry_source)
                << " torso_deg=" << torso * 180.0 / kPi
                << " arm=" << arm_json(candidate)
                << " margin_deg=" << arm_margin(candidate) * 180.0 / kPi
                << " tcp_mm=" << position_error * 1000.0
                << " tcp_deg=" << orientation_error * 180.0 / kPi
                << " nonpad_handle_mm=" << nonpad.first * 1000.0
                << " left_handle_mm=" << left.first * 1000.0;
      if (report_best_failed) {
        std::cout << " shoulder_torso_signed_mm=" <<
          shoulder_torso_distance * 1000.0
                  << " elbow_torso_signed_mm=" <<
          elbow_torso_distance * 1000.0
                  << " min_pair_signed_mm=" <<
          std::min(shoulder_torso_distance, elbow_torso_distance) * 1000.0;
      }
      std::cout << '\n';
      accepted.push_back(std::move(candidate));
    }
    std::cout << "IK_BRANCH_SEARCH status="
              << (accepted.empty() ? "NO_VALID_BRANCH" : "PASS")
              << " mode=" << (test_arm ? "TEST_ARM" :
        (ik_seed_arm ? "SEEDED_IK" : "RANDOM_IK"))
              << " accepted=" << accepted.size()
              << " samples=" << attempts
              << " ik_solved=" << ik_solved
              << " bounds_rejected=" << bounds_rejected
              << " self_rejected=" << self_rejected
              << " world_rejected=" << world_rejected
              << " residual_rejected=" << residual_rejected
              << " clearance_rejected=" << clearance_rejected
              << " patch_rejected=" << patch_rejected << '\n';
    for (const auto & [pair, count] : self_pairs) {
      std::cout << "IK_BRANCH_SELF_PAIR pair=" << pair
                << " count=" << count << '\n';
    }
    for (const auto & [pair, count] : world_pairs) {
      std::cout << "IK_BRANCH_WORLD_PAIR pair=" << pair
                << " count=" << count << '\n';
    }
    if (report_best_failed && best_failed_state) {
      std::cout << std::setprecision(17)
                << "IK_BRANCH_BEST_FAILED arm=" << arm_json(*best_failed_state)
                << " shoulder_torso_signed_mm=" <<
        best_failed_shoulder_torso * 1000.0
                << " elbow_torso_signed_mm=" <<
        best_failed_elbow_torso * 1000.0
                << " min_pair_signed_mm=" << best_failed_score * 1000.0
                << '\n';
    }
    rclcpp::shutdown();
    return accepted.empty() ? 2 : 0;
  } catch (const std::exception & error) {
    std::cerr << "RIGHT_HANDLE_IK_BRANCH_SEARCH=ERROR:" << error.what() << '\n';
    rclcpp::shutdown();
    return 3;
  }
}
