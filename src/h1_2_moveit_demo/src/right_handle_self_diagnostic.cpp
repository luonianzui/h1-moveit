#define RIGHT_HANDLE_PATH_EXACT_AUDITOR_NO_MAIN
#include "right_handle_path_exact_auditor.cpp"

namespace
{
constexpr double kDiagnosticStep = 0.1 * kPi / 180.0;
constexpr double kDiagnosticSliderStep = 0.0001;
constexpr char kElbowLink[] = "right_elbow_link";
constexpr char kTorsoLink[] = "torso_link";

struct PairMeasurement
{
  bool collision{false};
  double maximum_depth{0.0};
  double signed_distance{std::numeric_limits<double>::infinity()};
  Eigen::Vector3d contact_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d nearest_first{Eigen::Vector3d::Zero()};
  Eigen::Vector3d nearest_second{Eigen::Vector3d::Zero()};
};

PairMeasurement measure_pair(
  const planning_scene::PlanningScene & scene,
  const collision_detection::AllowedCollisionMatrix & pair_acm,
  const moveit::core::RobotState & state,
  const std::string & first_link,
  const std::string & second_link)
{
  PairMeasurement output;
  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_request.max_contacts = 100U;
  collision_request.max_contacts_per_pair = 100U;
  collision_request.pad_self_collisions = false;
  collision_detection::CollisionResult collision_result;
  scene.checkSelfCollision(
    collision_request, collision_result, state, pair_acm);
  output.collision = collision_result.collision;
  for (const auto & entry : collision_result.contacts) {
    for (const auto & contact : entry.second) {
      if (contact.depth >= output.maximum_depth) {
        output.maximum_depth = contact.depth;
        output.contact_position = contact.pos;
      }
    }
  }

  collision_detection::DistanceRequest distance_request;
  distance_request.type = collision_detection::DistanceRequestTypes::SINGLE;
  distance_request.enable_nearest_points = true;
  distance_request.enable_signed_distance = true;
  distance_request.max_contacts_per_body = 4U;
  distance_request.distance_threshold = std::numeric_limits<double>::infinity();
  distance_request.acm = &pair_acm;
  collision_detection::DistanceResult distance_result;
  scene.getCollisionEnvUnpadded()->distanceSelf(
    distance_request, distance_result, state);
  const auto & minimum = distance_result.minimum_distance;
  if ((minimum.link_names[0] == first_link &&
    minimum.link_names[1] == second_link) ||
    (minimum.link_names[1] == first_link &&
    minimum.link_names[0] == second_link))
  {
    output.signed_distance = minimum.distance;
    output.nearest_first = minimum.nearest_points[0];
    output.nearest_second = minimum.nearest_points[1];
  }
  return output;
}

std::size_t edge_subdivisions(
  const moveit::core::RobotState & first,
  const moveit::core::RobotState & second)
{
  double maximum_angle = std::abs(
    second.getVariablePosition("torso_joint") -
    first.getVariablePosition("torso_joint"));
  for (const auto & variable : kRightArmVariables) {
    maximum_angle = std::max(maximum_angle, std::abs(
      second.getVariablePosition(variable) -
      first.getVariablePosition(variable)));
  }
  double maximum_slider = 0.0;
  for (const auto & variable : kRightFingerVariables) {
    maximum_slider = std::max(maximum_slider, std::abs(
      second.getVariablePosition(variable) -
      first.getVariablePosition(variable)));
  }
  return std::max<std::size_t>({
      1U,
      static_cast<std::size_t>(std::ceil(maximum_angle / kDiagnosticStep)),
      static_cast<std::size_t>(std::ceil(maximum_slider / kDiagnosticSliderStep))});
}

void print_measurement(const PairMeasurement & measurement)
{
  std::cout << std::boolalpha << std::setprecision(17)
            << " collision=" << measurement.collision
            << " max_depth_mm=" << measurement.maximum_depth * 1000.0
            << " signed_distance_mm=" << measurement.signed_distance * 1000.0
            << " contact_xyz_m=[" << measurement.contact_position.transpose() << "]"
            << " nearest0_xyz_m=[" << measurement.nearest_first.transpose() << "]"
            << " nearest1_xyz_m=[" << measurement.nearest_second.transpose() << "]";
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
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
    const std::string first_link = option(argc, argv, "--first-link").value_or(kElbowLink);
    const std::string second_link = option(argc, argv, "--second-link").value_or(kTorsoLink);

    const auto cdr = read_bytes(scene_path);
    const auto raw_scene = deserialize_scene(cdr);
    const auto projection = project_physical_scene(raw_scene, cdr);
    if (!projection.unknown.empty() || !projection.handle_exact_retained) {
      throw std::runtime_error("SCENE_PROJECTION");
    }
    const auto asset = load_path_asset(path_path);
    auto robot_model = load_robot_model(urdf_path, srdf_path);
    planning_scene::PlanningScene scene(robot_model);
    if (!scene.usePlanningSceneMsg(projection.physical)) {
      throw std::runtime_error("PLANNING_SCENE_LOAD");
    }
    verify_collision_geometry_fail_closed(scene);
    moveit::core::RobotState seam(scene.getCurrentState());
    verify_seam(seam);
    seam.setVariablePosition("left_shoulder_roll_joint", park_left_roll);
    seam.updateCollisionBodyTransforms();

    collision_detection::AllowedCollisionMatrix pair_acm(
      robot_model->getLinkModelNames(), true);
    pair_acm.setEntry(first_link, second_link, false);

    bool found = false;
    std::size_t dense_index = 0U;
    std::size_t first_row = 0U;
    std::size_t first_substep = 0U;
    std::size_t first_subdivisions = 1U;
    double first_fraction = 0.0;
    double first_handle = 0.0;
    PairMeasurement first_measurement;
    moveit::core::RobotState previous = row_state(seam, asset.rows.front(), false);
    previous.updateCollisionBodyTransforms();
    first_measurement = measure_pair(scene, pair_acm, previous, first_link, second_link);
    if (first_measurement.collision) {
      found = true;
    }
    for (std::size_t row = 1U; row < asset.rows.size() && !found; ++row) {
      moveit::core::RobotState next = row_state(seam, asset.rows[row], false);
      const std::size_t subdivisions = edge_subdivisions(previous, next);
      for (std::size_t substep = 1U; substep <= subdivisions; ++substep) {
        ++dense_index;
        const double fraction = static_cast<double>(substep) /
          static_cast<double>(subdivisions);
        moveit::core::RobotState sample(previous);
        previous.interpolate(next, fraction, sample);
        sample.updateCollisionBodyTransforms();
        const auto measurement = measure_pair(
          scene, pair_acm, sample, first_link, second_link);
        if (measurement.collision) {
          found = true;
          first_row = row;
          first_substep = substep;
          first_subdivisions = subdivisions;
          first_fraction = fraction;
          first_handle = asset.rows[row - 1U].handle_angle + fraction *
            (asset.rows[row].handle_angle - asset.rows[row - 1U].handle_angle);
          first_measurement = measurement;
          break;
        }
      }
      previous = std::move(next);
    }

    std::cout << "FIRST_PAIR_COLLISION found=" << std::boolalpha << found
              << " dense_index=" << dense_index
              << " row=" << first_row
              << " edge_from_row=" << (first_row == 0U ? 0U : first_row - 1U)
              << " substep=" << first_substep << '/' << first_subdivisions
              << " fraction=" << std::setprecision(17) << first_fraction
              << " handle_deg=" << first_handle * 180.0 / kPi;
    print_measurement(first_measurement);
    std::cout << '\n';

    moveit::core::RobotState endpoint = row_state(seam, asset.rows.back(), false);
    endpoint.updateCollisionBodyTransforms();
    std::cout << "ENDPOINT";
    print_measurement(measure_pair(
      scene, pair_acm, endpoint, first_link, second_link));
    std::cout << '\n';

    const std::array<std::string, 8U> variables{
      "torso_joint", "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
      "right_elbow_joint", "right_wrist_roll_joint",
      "right_wrist_pitch_joint", "right_wrist_yaw_joint"};
    for (const auto & variable : variables) {
      for (const double sign : {-1.0, 1.0}) {
        moveit::core::RobotState perturbed(endpoint);
        perturbed.setVariablePosition(
          variable, endpoint.getVariablePosition(variable) +
          sign * kDiagnosticStep);
        perturbed.updateCollisionBodyTransforms();
        std::cout << "SENSITIVITY variable=" << variable
                  << " delta_deg=" << sign * 0.1;
        print_measurement(measure_pair(
          scene, pair_acm, perturbed, first_link, second_link));
        std::cout << '\n';
      }
    }
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "RIGHT_HANDLE_SELF_DIAGNOSTIC=ERROR:" << error.what() << '\n';
    rclcpp::shutdown();
    return 3;
  }
}
