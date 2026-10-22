#define RIGHT_HANDLE_PATH_EXACT_AUDITOR_NO_MAIN
#include "right_handle_path_exact_auditor.cpp"

#include <random>

namespace
{
constexpr char kSpliceSearchSchema[] = "h1_right_handle_splice_search/v1";
constexpr double kSearchEdgeStep = 0.25 * kPi / 180.0;
constexpr double kStrictEdgeStep = 0.1 * kPi / 180.0;
constexpr double kExtendStep = 7.0 * kPi / 180.0;
using Configuration = std::array<double, 8>;

struct SpliceSearchArguments
{
  std::filesystem::path scene;
  std::filesystem::path path;
  std::filesystem::path output;
  std::filesystem::path urdf;
  std::filesystem::path srdf;
  std::filesystem::path handle_stl;
  double park_left_roll{0.0};
  double handle_angle{0.0};
  std::uint64_t seed{0U};
  std::size_t iterations{0U};
  std::array<double, 7> goal_arm{};
  std::optional<Configuration> start_configuration;
  std::optional<double> goal_torso;
  double minimum_environment_clearance{0.0};
};

SpliceSearchArguments splice_arguments(const int argc, char ** argv)
{
  const auto required = [&](const std::string & name) {
      const auto value = option(argc, argv, name);
      if (!value || value->empty()) {
        throw std::runtime_error("ARGUMENT_MISSING:" + name);
      }
      return *value;
    };
  SpliceSearchArguments output;
  output.scene = required("--scene");
  output.path = required("--path");
  output.output = required("--output");
  output.urdf = required("--urdf");
  output.srdf = required("--srdf");
  output.handle_stl = required("--handle-stl");
  output.park_left_roll = std::stod(required("--park-left-roll"));
  output.handle_angle = std::stod(required("--handle-angle-deg")) * kPi / 180.0;
  output.seed = std::stoull(required("--seed"));
  output.iterations = std::stoull(required("--iterations"));
  {
    std::istringstream stream(required("--goal-arm"));
    std::string field;
    std::size_t index = 0U;
    while (std::getline(stream, field, ',')) {
      if (index >= output.goal_arm.size()) {
        throw std::runtime_error("ARGUMENT_VECTOR_WIDTH:--goal-arm");
      }
      output.goal_arm[index++] = std::stod(field);
    }
    if (index != output.goal_arm.size() ||
      !std::all_of(output.goal_arm.begin(), output.goal_arm.end(),
      [](const double value) {return std::isfinite(value);}))
    {
      throw std::runtime_error("ARGUMENT_VECTOR:--goal-arm");
    }
  }
  if (const auto start = option(argc, argv, "--start-config")) {
    std::istringstream stream(*start);
    std::string field;
    Configuration parsed{};
    std::size_t index = 0U;
    while (std::getline(stream, field, ',')) {
      if (index >= parsed.size()) {
        throw std::runtime_error("ARGUMENT_VECTOR_WIDTH:--start-config");
      }
      parsed[index++] = std::stod(field);
    }
    if (index != parsed.size() ||
      !std::all_of(parsed.begin(), parsed.end(),
      [](const double value) {return std::isfinite(value);}))
    {
      throw std::runtime_error("ARGUMENT_VECTOR:--start-config");
    }
    output.start_configuration = parsed;
  }
  if (const auto torso_degrees = option(argc, argv, "--goal-torso-deg")) {
    output.goal_torso = std::stod(*torso_degrees) * kPi / 180.0;
    if (!std::isfinite(*output.goal_torso) ||
      std::abs(*output.goal_torso) > kTorsoLimit + 1e-12)
    {
      throw std::runtime_error("ARGUMENT_RANGE:--goal-torso-deg");
    }
  }
  if (const auto clearance = option(argc, argv, "--minimum-environment-clearance-m")) {
    output.minimum_environment_clearance = std::stod(*clearance);
  }
  if (!std::isfinite(output.park_left_roll) || !std::isfinite(output.handle_angle) ||
    output.handle_angle < -1e-12 || output.handle_angle > kPi / 2.0 + 1e-12 ||
    output.iterations < 100U || output.iterations > 1000000U ||
    !std::isfinite(output.minimum_environment_clearance) ||
    output.minimum_environment_clearance < 0.0 ||
    output.minimum_environment_clearance > 0.020)
  {
    throw std::runtime_error("ARGUMENT_RANGE");
  }
  return output;
}

double maximum_delta(const Configuration & first, const Configuration & second)
{
  double output = 0.0;
  for (std::size_t index = 0U; index < first.size(); ++index) {
    output = std::max(output, std::abs(second[index] - first[index]));
  }
  return output;
}

double squared_distance(const Configuration & first, const Configuration & second)
{
  double output = 0.0;
  for (std::size_t index = 0U; index < first.size(); ++index) {
    const double delta = second[index] - first[index];
    output += delta * delta;
  }
  return output;
}

Configuration interpolate_configuration(
  const Configuration & first, const Configuration & second, const double fraction)
{
  Configuration output{};
  for (std::size_t index = 0U; index < output.size(); ++index) {
    output[index] = first[index] + (second[index] - first[index]) * fraction;
  }
  return output;
}

class SpliceCollisionChecker
{
public:
  SpliceCollisionChecker(
    planning_scene::PlanningScene & scene, const double park_left_roll,
    const double handle_angle, const double minimum_environment_clearance)
  : scene_(scene), state_(scene.getCurrentState()),
    handle_(prepare_handle_geometry(scene)), base_acm_(scene.getAllowedCollisionMatrix()),
    distance_acm_(base_acm_),
    minimum_environment_clearance_(minimum_environment_clearance)
  {
    verify_seam(state_);
    state_.setVariablePosition("left_shoulder_roll_joint", park_left_roll);
    for (const auto & finger : kRightFingerVariables) {
      state_.setVariablePosition(finger, kOpenFingerPosition);
    }
    state_.updateCollisionBodyTransforms();
    const auto handle_global = handle_pose(handle_, handle_angle);
    set_single_shape_pose(scene_, kHandleId, handle_.shape, handle_global);
    collision_links_ = scene_.getRobotModel()->getLinkModelNamesWithCollisionGeometry();
    for (const auto & link : collision_links_) {
      const bool pad = link == kRightHandlePads[0] || link == kRightHandlePads[1];
      if (link.rfind("left_", 0U) == 0U) {
        left_links_.push_back(link);
      } else if (!pad) {
        nonpad_nonleft_links_.push_back(link);
      }
    }
    for (const auto & object : scene_.getWorld()->getObjectIds()) {
      for (const auto & link : collision_links_) {
        base_acm_.setEntry(object, link, false);
      }
    }
    for (const auto & link : collision_links_) {
      distance_acm_.setEntry(kHandleId, link, true);
      distance_acm_.setEntry("environment_table_10mm_margin", link, true);
      distance_acm_.setEntry("green_cube_10mm_guard", link, true);
    }
    base_acm_.setEntry("right_elbow_link", "torso_link", false);
    collision_request_.contacts = true;
    collision_request_.max_contacts = 4U;
    collision_request_.max_contacts_per_pair = 1U;
    collision_request_.pad_self_collisions = false;
    world_request_ = collision_request_;
    world_request_.pad_environment_collisions = false;
    distance_request_.type = collision_detection::DistanceRequestTypes::SINGLE;
    distance_request_.enable_nearest_points = false;
    distance_request_.enable_signed_distance = true;
    distance_request_.max_contacts_per_body = 1U;
    distance_request_.distance_threshold = minimum_environment_clearance_ + 1e-6;
    distance_request_.acm = &distance_acm_;

    lower_[0] = -kTorsoLimit;
    upper_[0] = kTorsoLimit;
    for (std::size_t index = 0U; index < kRightArmVariables.size(); ++index) {
      const auto & bounds = scene_.getRobotModel()->getVariableBounds(kRightArmVariables[index]);
      if (!bounds.position_bounded_) {
        throw std::runtime_error("SEARCH_ARM_UNBOUNDED");
      }
      lower_[index + 1U] = bounds.min_position_ + kArmMargin;
      upper_[index + 1U] = bounds.max_position_ - kArmMargin;
      if (lower_[index + 1U] >= upper_[index + 1U]) {
        throw std::runtime_error("SEARCH_ARM_MARGIN_RANGE");
      }
    }
  }

  Configuration captured_start() const
  {
    Configuration output{};
    output[0] = state_.getVariablePosition("torso_joint");
    for (std::size_t index = 0U; index < kRightArmVariables.size(); ++index) {
      output[index + 1U] = state_.getVariablePosition(kRightArmVariables[index]);
    }
    return output;
  }

  Configuration path_goal(
    const PathRow & row, const std::array<double, 7> & goal_arm) const
  {
    Configuration output{};
    output[0] = row.torso;
    std::copy(goal_arm.begin(), goal_arm.end(), output.begin() + 1U);
    return output;
  }

  const Configuration & lower() const {return lower_;}
  const Configuration & upper() const {return upper_;}

  bool valid(
    const Configuration & configuration, const bool clearance = true,
    std::string * reason = nullptr)
  {
    for (std::size_t index = 0U; index < configuration.size(); ++index) {
      if (!std::isfinite(configuration[index]) ||
        configuration[index] < lower_[index] - 1e-12 ||
        configuration[index] > upper_[index] + 1e-12)
      {
        if (reason != nullptr) {*reason = "BOUNDS:" + std::to_string(index);}
        return false;
      }
    }
    state_.setVariablePosition("torso_joint", configuration[0]);
    for (std::size_t index = 0U; index < kRightArmVariables.size(); ++index) {
      state_.setVariablePosition(kRightArmVariables[index], configuration[index + 1U]);
    }
    state_.updateCollisionBodyTransforms();
    if (!state_.satisfiesBounds(1e-9)) {
      if (reason != nullptr) {*reason = "MODEL_BOUNDS";}
      return false;
    }
    collision_detection::CollisionResult self_result;
    scene_.checkSelfCollision(collision_request_, self_result, state_, base_acm_);
    if (self_result.collision) {
      if (reason != nullptr) {*reason = "SELF:" + collision_pair(self_result);}
      return false;
    }
    collision_detection::CollisionResult world_result;
    scene_.getCollisionEnvUnpadded()->checkRobotCollision(
      world_request_, world_result, state_, base_acm_);
    if (world_result.collision) {
      if (reason != nullptr) {*reason = "WORLD:" + collision_pair(world_result);}
      return false;
    }
    if (!clearance) {
      return true;
    }
    if (minimum_environment_clearance_ > 0.0) {
      collision_detection::DistanceResult distance_result;
      scene_.getCollisionEnvUnpadded()->distanceRobot(
        distance_request_, distance_result, state_);
      if (std::isfinite(distance_result.minimum_distance.distance) &&
        distance_result.minimum_distance.distance + 1e-9 <
        minimum_environment_clearance_)
      {
        if (reason != nullptr) {*reason = "ENVIRONMENT_CLEARANCE";}
        return false;
      }
    }
    const auto nonpad = handle_distance(
      scene_, state_, base_acm_, nonpad_nonleft_links_, 0.020);
    if (nonpad.first + 1e-12 < kNonpadHandleClearance) {
      if (reason != nullptr) {*reason = "NONPAD:" + nonpad.second;}
      return false;
    }
    const auto left = handle_distance(scene_, state_, base_acm_, left_links_, 0.050);
    if (left.first + 1e-12 < kPathLeftHandleClearance) {
      if (reason != nullptr) {*reason = "LEFT:" + left.second;}
      return false;
    }
    return true;
  }

  bool edge(
    const Configuration & first, const Configuration & second,
    const double step, std::size_t * sample_counter = nullptr)
  {
    const std::size_t subdivisions = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(maximum_delta(first, second) / step)));
    for (std::size_t index = 1U; index <= subdivisions; ++index) {
      if (sample_counter != nullptr) {
        ++*sample_counter;
      }
      if (!valid(interpolate_configuration(
          first, second, static_cast<double>(index) / static_cast<double>(subdivisions))))
      {
        return false;
      }
    }
    return true;
  }

private:
  planning_scene::PlanningScene & scene_;
  moveit::core::RobotState state_;
  HandleGeometry handle_;
  collision_detection::AllowedCollisionMatrix base_acm_;
  collision_detection::AllowedCollisionMatrix distance_acm_;
  collision_detection::CollisionRequest collision_request_;
  collision_detection::CollisionRequest world_request_;
  collision_detection::DistanceRequest distance_request_;
  std::vector<std::string> collision_links_;
  std::vector<std::string> left_links_;
  std::vector<std::string> nonpad_nonleft_links_;
  Configuration lower_{};
  Configuration upper_{};
  double minimum_environment_clearance_{0.0};
};

struct SearchNode
{
  Configuration q{};
  std::size_t parent{0U};
};

struct SearchTree
{
  bool start_root{false};
  std::vector<SearchNode> nodes;
};

enum class ExtendStatus {Trapped, Advanced, Reached};

struct ExtendResult
{
  ExtendStatus status{ExtendStatus::Trapped};
  std::size_t index{0U};
};

std::size_t nearest_node(const SearchTree & tree, const Configuration & target)
{
  std::size_t best = 0U;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < tree.nodes.size(); ++index) {
    const double candidate = squared_distance(tree.nodes[index].q, target);
    if (candidate < distance) {
      distance = candidate;
      best = index;
    }
  }
  return best;
}

ExtendResult extend_tree(
  SearchTree & tree, const Configuration & target,
  SpliceCollisionChecker & checker, std::size_t & collision_samples)
{
  const std::size_t nearest = nearest_node(tree, target);
  const auto & source = tree.nodes[nearest].q;
  const double delta = maximum_delta(source, target);
  const double fraction = delta <= kExtendStep ? 1.0 : kExtendStep / delta;
  const auto next = interpolate_configuration(source, target, fraction);
  if (!checker.edge(source, next, kSearchEdgeStep, &collision_samples)) {
    return {ExtendStatus::Trapped, nearest};
  }
  tree.nodes.push_back(SearchNode{next, nearest});
  return {
    fraction >= 1.0 - 1e-12 ? ExtendStatus::Reached : ExtendStatus::Advanced,
    tree.nodes.size() - 1U};
}

std::vector<Configuration> root_chain(
  const SearchTree & tree, const std::size_t node)
{
  std::vector<Configuration> output;
  std::size_t current = node;
  while (true) {
    output.push_back(tree.nodes[current].q);
    if (current == 0U) {
      break;
    }
    current = tree.nodes[current].parent;
  }
  std::reverse(output.begin(), output.end());
  return output;
}

std::vector<Configuration> connected_path(
  const SearchTree & first, const std::size_t first_node,
  const SearchTree & second, const std::size_t second_node)
{
  auto first_chain = root_chain(first, first_node);
  auto second_chain = root_chain(second, second_node);
  std::vector<Configuration> start;
  std::vector<Configuration> goal;
  if (first.start_root) {
    start = std::move(first_chain);
    goal = std::move(second_chain);
  } else {
    start = std::move(second_chain);
    goal = std::move(first_chain);
  }
  std::reverse(goal.begin(), goal.end());
  if (!start.empty() && !goal.empty() && maximum_delta(start.back(), goal.front()) <= 1e-10) {
    goal.erase(goal.begin());
  }
  start.insert(start.end(), goal.begin(), goal.end());
  return start;
}

void shortcut_path(
  std::vector<Configuration> & path, SpliceCollisionChecker & checker,
  std::mt19937_64 & random, std::size_t & collision_samples)
{
  for (std::size_t attempt = 0U; attempt < 1000U && path.size() > 2U; ++attempt) {
    std::uniform_int_distribution<std::size_t> distribution(0U, path.size() - 1U);
    std::size_t first = distribution(random);
    std::size_t second = distribution(random);
    if (first > second) {
      std::swap(first, second);
    }
    if (second <= first + 1U) {
      continue;
    }
    if (checker.edge(path[first], path[second], kSearchEdgeStep, &collision_samples)) {
      path.erase(path.begin() + static_cast<std::ptrdiff_t>(first + 1U),
        path.begin() + static_cast<std::ptrdiff_t>(second));
    }
  }
}

std::optional<std::vector<Configuration>> search_path(
  SpliceCollisionChecker & checker, const Configuration & start,
  const Configuration & goal, const std::uint64_t seed,
  const std::size_t iterations, std::size_t & collision_samples,
  std::size_t & used_iterations)
{
  std::string endpoint_reason;
  if (!checker.valid(start, true, &endpoint_reason)) {
    throw std::runtime_error("SEARCH_START_INVALID:" + endpoint_reason);
  }
  if (!checker.valid(goal, true, &endpoint_reason)) {
    throw std::runtime_error("SEARCH_GOAL_INVALID:" + endpoint_reason);
  }
  if (checker.edge(start, goal, kStrictEdgeStep, &collision_samples)) {
    used_iterations = 0U;
    return std::vector<Configuration>{start, goal};
  }
  std::mt19937_64 random(seed);
  SearchTree first{true, {{start, 0U}}};
  SearchTree second{false, {{goal, 0U}}};
  std::uniform_real_distribution<double> probability(0.0, 1.0);
  for (std::size_t iteration = 1U; iteration <= iterations; ++iteration) {
    Configuration target{};
    if (probability(random) < 0.18) {
      target = second.nodes.front().q;
    } else {
      for (std::size_t joint = 0U; joint < target.size(); ++joint) {
        std::uniform_real_distribution<double> sample(checker.lower()[joint], checker.upper()[joint]);
        target[joint] = sample(random);
      }
    }
    const auto advanced = extend_tree(first, target, checker, collision_samples);
    if (advanced.status != ExtendStatus::Trapped) {
      ExtendResult connected;
      do {
        connected = extend_tree(
          second, first.nodes[advanced.index].q, checker, collision_samples);
      } while (connected.status == ExtendStatus::Advanced);
      if (connected.status == ExtendStatus::Reached) {
        auto path = connected_path(first, advanced.index, second, connected.index);
        shortcut_path(path, checker, random, collision_samples);
        bool strict = true;
        for (std::size_t index = 1U; index < path.size(); ++index) {
          if (!checker.edge(path[index - 1U], path[index], kStrictEdgeStep,
              &collision_samples))
          {
            strict = false;
            break;
          }
        }
        if (strict) {
          used_iterations = iteration;
          return path;
        }
      }
    }
    std::swap(first, second);
  }
  used_iterations = iterations;
  return std::nullopt;
}

std::string configuration_json(const Configuration & q)
{
  std::ostringstream output;
  output << std::setprecision(17) << '[';
  for (std::size_t index = 0U; index < q.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << q[index];
  }
  output << ']';
  return output.str();
}

std::string splice_json(
  const SpliceSearchArguments & args, const PathAsset & asset,
  const Projection & projection, const std::vector<Configuration> & path,
  const std::size_t iterations, const std::size_t collision_samples)
{
  std::ostringstream output;
  output << std::boolalpha << std::setprecision(17) <<
    "{\n"
    "  \"schema\": \"" << kSpliceSearchSchema << "\",\n"
    "  \"status\": \"PASS\",\n"
    "  \"valid\": true,\n"
    "  \"controller_commands_issued\": 0,\n"
    "  \"offline_only\": true,\n"
    "  \"path_sha256\": \"" << asset.sha256 << "\",\n"
    "  \"scene_cdr_sha256\": \"" << projection.raw_cdr_sha256 << "\",\n"
    "  \"handle_angle_deg\": " << args.handle_angle * 180.0 / kPi << ",\n"
    "  \"park_left_shoulder_roll_rad\": " << args.park_left_roll << ",\n"
    "  \"minimum_environment_clearance_m\": " <<
    args.minimum_environment_clearance << ",\n"
    "  \"goal_right_arm_rad\": " << vector7_json(args.goal_arm) << ",\n"
    "  \"seed\": " << args.seed << ",\n"
    "  \"iterations_used\": " << iterations << ",\n"
    "  \"collision_samples\": " << collision_samples << ",\n"
    "  \"search_edge_step_rad\": " << kSearchEdgeStep << ",\n"
    "  \"strict_revalidation_step_rad\": " << kStrictEdgeStep << ",\n"
    "  \"waypoint_count\": " << path.size() << ",\n"
    "  \"configuration_order\": [\"torso_joint\","
    "\"right_shoulder_pitch_joint\",\"right_shoulder_roll_joint\","
    "\"right_shoulder_yaw_joint\",\"right_elbow_joint\","
    "\"right_wrist_roll_joint\",\"right_wrist_pitch_joint\","
    "\"right_wrist_yaw_joint\"],\n"
    "  \"right_fingers_rad_or_m\": {\"right_dex1_finger_1_joint\":-0.019,"
    "\"right_dex1_finger_2_joint\":-0.019},\n"
    "  \"waypoints\": [\n";
  for (std::size_t index = 0U; index < path.size(); ++index) {
    output << "    " << configuration_json(path[index]) <<
      (index + 1U == path.size() ? "\n" : ",\n");
  }
  output << "  ]\n}\n";
  return output.str();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  std::filesystem::path output_path;
  try {
    const auto args = splice_arguments(argc, argv);
    output_path = args.output;
    const auto cdr = read_bytes(args.scene);
    const auto raw_scene = deserialize_scene(cdr);
    const auto projection = project_physical_scene(raw_scene, cdr);
    if (!projection.unknown.empty() || !projection.handle_exact_retained) {
      throw std::runtime_error("SCENE_PROJECTION");
    }
    const auto asset = load_path_asset(args.path);
    verify_handle_mesh(projection.physical, read_bytes(args.handle_stl));
    auto robot_model = load_robot_model(
      read_text_file(args.urdf), read_text_file(args.srdf));
    planning_scene::PlanningScene scene(robot_model);
    if (!scene.usePlanningSceneMsg(projection.physical)) {
      throw std::runtime_error("PLANNING_SCENE_LOAD");
    }
    verify_collision_geometry_fail_closed(scene);
    SpliceCollisionChecker checker(
      scene, args.park_left_roll, args.handle_angle,
      args.minimum_environment_clearance);
    const auto start = args.start_configuration.value_or(checker.captured_start());
    const auto row_iterator = std::min_element(
      asset.rows.begin(), asset.rows.end(),
      [&](const PathRow & first, const PathRow & second) {
        return std::abs(first.handle_angle - args.handle_angle) <
               std::abs(second.handle_angle - args.handle_angle);
      });
    if (row_iterator == asset.rows.end()) {
      throw std::runtime_error("PATH_EMPTY");
    }
    const auto & row = *row_iterator;
    auto goal = checker.path_goal(row, args.goal_arm);
    if (args.goal_torso) {
      goal[0] = *args.goal_torso;
    }
    std::size_t collision_samples = 0U;
    std::size_t iterations = 0U;
    const auto path = search_path(
      checker, start, goal, args.seed, args.iterations,
      collision_samples, iterations);
    if (!path) {
      throw std::runtime_error("RRT_NO_PATH");
    }
    atomic_write(args.output, splice_json(
      args, asset, projection, *path, iterations, collision_samples));
    std::cout << "RIGHT_HANDLE_SPLICE_SEARCH=PASS handle_deg=" <<
      args.handle_angle * 180.0 / kPi << " waypoints=" << path->size() <<
      " iterations=" << iterations << " collision_samples=" << collision_samples << std::endl;
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "RIGHT_HANDLE_SPLICE_SEARCH=ERROR:" << error.what() << std::endl;
    if (!output_path.empty()) {
      try {
        atomic_write(output_path,
          std::string("{\"schema\":\"") + kSpliceSearchSchema +
          "\",\"status\":\"ERROR:" + json_escape(error.what()) +
          "\",\"valid\":false,\"controller_commands_issued\":0}\n");
      } catch (const std::exception &) {
      }
    }
    rclcpp::shutdown();
    return 3;
  }
}
