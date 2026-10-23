#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/BoxedLcpSolver.hpp>
#include <dart/constraint/ConstrainedGroup.hpp>
#include <dart/constraint/ConstraintBase.hpp>
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

using Solver = dart::constraint::BoxedLcpConstraintSolver;
using Group = dart::constraint::ConstrainedGroup;
using Function = void (*)(Solver*, Group&);
using Clock = std::chrono::steady_clock;
const char* symbol = "_ZN4dart10constraint24BoxedLcpConstraintSolver21solveConstrainedGroupERNS0_16ConstrainedGroupE";

void require(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}
template<typename Value>
void append(std::vector<unsigned char>& bytes, const Value* values, std::size_t count)
{
  const auto* start = reinterpret_cast<const unsigned char*>(values);
  bytes.insert(bytes.end(), start, start + count * sizeof(Value));
}
struct Fixture {
  std::size_t total = 0;
  std::size_t selected = 0;
  std::vector<unsigned char> input;
  std::vector<unsigned char> output;
  std::vector<std::size_t> events;
  bool record = true;
  void event(std::size_t identity, std::size_t kind)
  {
    if (record) events.push_back(identity * 1000 + kind);
  }
};
class Constraint : public dart::constraint::ConstraintBase {
public:
  Fixture& fixture;
  std::size_t offset;
  Constraint(Fixture& state, std::size_t first, std::size_t dimension) : fixture(state), offset(first) { mDim = dimension; }
  void update() override {}
  void getInformation(dart::constraint::ConstraintInfo* information) override
  {
    fixture.event(offset, 1);
    for (std::size_t index = 0; index < mDim; ++index) {
      information->x[index] = 0.;
      information->b[index] = (offset + index + 1) / 128.;
      information->lo[index] = index == 0 ? 0. : -.8;
      information->hi[index] = index == 0 ? std::numeric_limits<double>::infinity() : .8;
      information->findex[index] = index == 0 ? -1 : 0;
      require(information->w[index] == 0., "Slack initialization changed");
    }
  }
  void applyUnitImpulse(std::size_t index) override { fixture.selected = offset + index; fixture.event(offset, 10 + index); }
  void getVelocityChange(double* values, bool withCfm) override
  {
    fixture.event(offset, withCfm ? 110 : 111);
    for (std::size_t index = 0; index < mDim; ++index) {
      const auto column = offset + index;
      values[index] = fixture.selected == column ? 2. + fixture.total / 32. : .015625;
      if (withCfm && fixture.selected == column) values[index] += 1e-8;
    }
  }
  void excite() override { fixture.event(offset, 2); }
  void unexcite() override { fixture.event(offset, 3); }
  void applyImpulse(double* impulse) override
  {
    fixture.event(offset, 4);
    if (fixture.record) append(fixture.output, impulse, mDim);
  }
  bool isActive() const override { return true; }
  dart::dynamics::SkeletonPtr getRootSkeleton() const override { return nullptr; }
};
class CapturingSolver : public dart::constraint::BoxedLcpSolver {
public:
  Fixture& fixture;
  int mode;
  dart::constraint::DantzigBoxedLcpSolver actual;
  CapturingSolver(Fixture& state, int behavior) : fixture(state), mode(behavior) {}
  const std::string& getType() const override { static const std::string name = "ExactQualification"; return name; }
  bool solve(int dimension, double* matrix, double* impulse, double* bias, int unbounded,
             double* low, double* high, int* friction, bool early) override
  {
    const int stride = dimension <= 1 ? dimension : (dimension + 3) & ~3;
    if (fixture.record) {
      append(fixture.input, &dimension, 1);
      append(fixture.input, &unbounded, 1);
      append(fixture.input, &early, 1);
      for (int row = 0; row < dimension; ++row) append(fixture.input, matrix + row * stride, dimension);
      for (auto* values : {impulse, bias, low, high}) append(fixture.input, values, dimension);
      append(fixture.input, friction, dimension);
    }
    if (mode == 1) {
      for (int row = 0; row < dimension; ++row) {
        matrix[row * stride + row] = -9.;
        impulse[row] = 8.;
        bias[row] = 7.;
        low[row] = -6.;
        high[row] = 6.;
        friction[row] = -1;
      }
      return false;
    }
    if (mode == 2) {
      impulse[0] = std::numeric_limits<double>::quiet_NaN();
      return true;
    }
    return actual.solve(dimension, matrix, impulse, bias, unbounded, low, high, friction, early);
  }
};
struct Outcome { std::vector<unsigned char> input, output; std::vector<std::size_t> events; double elapsed; };
Outcome exercise(Function function, const std::vector<std::size_t>& dimensions,
                 int primaryMode, int secondaryMode, bool secondary, int repeats, bool recording)
{
  Fixture fixture;
  fixture.record = recording;
  Group group;
  for (auto dimension : dimensions) {
    group.addConstraint(std::make_shared<Constraint>(fixture, fixture.total, dimension));
    fixture.total += dimension;
  }
  auto primary = std::make_shared<CapturingSolver>(fixture, primaryMode);
  auto fallback = std::make_shared<CapturingSolver>(fixture, secondaryMode);
  Solver solver(primary, secondary ? fallback : nullptr);
  solver.setTimeStep(.0005);
  const auto start = Clock::now();
  for (int repeat = 0; repeat < repeats; ++repeat) function(&solver, group);
  return {fixture.input, fixture.output, fixture.events, std::chrono::duration<double>(Clock::now() - start).count()};
}
class SelectedSolver : public Solver {
public:
  Function function;
  explicit SelectedSolver(Function selected) : function(selected) {}
protected:
  void solveConstrainedGroup(Group& group) override { function(this, group); }
};
dart::simulation::WorldPtr world(Function function)
{
  auto result = dart::simulation::World::create();
  result->setTimeStep(.0005);
  result->setGravity(Eigen::Vector3d(0., 0., -9.81));
  result->setConstraintSolver(std::make_unique<SelectedSolver>(function));
  auto floor = dart::dynamics::Skeleton::create("floor");
  auto floorPair = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  Eigen::Isometry3d floorPose = Eigen::Isometry3d::Identity();
  floorPose.translation().z() = -.05;
  floorPair.first->setTransformFromParentBodyNode(floorPose);
  floorPair.second->createShapeNodeWith<dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(2., 2., .1)));
  result->addSkeleton(floor);
  auto box = dart::dynamics::Skeleton::create("box");
  auto boxPair = box->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  boxPair.second->createShapeNodeWith<dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(.1, .1, .1)));
  boxPair.second->setMass(.5);
  auto positions = box->getPositions();
  positions[5] = .08;
  box->setPositions(positions);
  result->addSkeleton(box);
  return result;
}
int main(int argc, char** argv)
{
  require(argc == 2, "Expected candidate library");
  setenv("H1_DART_CACHE_MODE", "cached", 1);
  unsetenv("H1_DART_CACHE_LOG");
  auto* nativeHandle = dlopen("/opt/ros/jazzy/opt/gz_dartsim_vendor/lib/libdart.so.6.13.2", RTLD_NOW | RTLD_LOCAL);
  auto* candidateHandle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  require(nativeHandle && candidateHandle, dlerror());
  auto native = reinterpret_cast<Function>(dlsym(nativeHandle, symbol));
  auto candidate = reinterpret_cast<Function>(dlsym(candidateHandle, symbol));
  require(native && candidate && native != candidate, "Distinct exact native/candidate functions required");
  std::size_t cases = 0;
  for (int count : {0, 1, 2, 4, 16, 64, 128}) {
    for (int scheme = 0; scheme < 3; ++scheme) {
      std::vector<std::size_t> dimensions;
      for (int index = 0; index < count; ++index) dimensions.push_back(scheme == 0 ? 1 : scheme == 1 ? 3 : 1 + index % 6);
      for (int behavior = 0; behavior < 5; ++behavior) {
        const int primaryMode = behavior == 0 ? 0 : behavior == 1 ? 1 : 2;
        const bool secondary = behavior != 4;
        const int secondaryMode = behavior == 3 ? 2 : 0;
        const auto reference = exercise(native, dimensions, primaryMode, secondaryMode, secondary, 2, true);
        const auto optimized = exercise(candidate, dimensions, primaryMode, secondaryMode, secondary, 2, true);
        require(reference.input == optimized.input, "LCP input bytes differ");
        require(reference.output == optimized.output, "Impulse bytes differ");
        require(reference.events == optimized.events, "Physical callback order differs");
        ++cases;
      }
    }
  }
  auto referenceWorld = world(native);
  auto candidateWorld = world(candidate);
  std::size_t contactSteps = 0;
  for (int step = 0; step < 1200; ++step) {
    referenceWorld->step();
    candidateWorld->step();
    const auto referencePositions = referenceWorld->getSkeleton("box")->getPositions();
    const auto candidatePositions = candidateWorld->getSkeleton("box")->getPositions();
    require(referencePositions.size() == candidatePositions.size(), "World size mismatch");
    require(std::memcmp(referencePositions.data(), candidatePositions.data(), sizeof(double) * referencePositions.size()) == 0,
            "Actual DART world positions differ");
    const auto referenceVelocity = referenceWorld->getSkeleton("box")->getVelocities();
    const auto candidateVelocity = candidateWorld->getSkeleton("box")->getVelocities();
    require(std::memcmp(referenceVelocity.data(), candidateVelocity.data(), sizeof(double) * referenceVelocity.size()) == 0,
            "Actual DART world velocities differ");
    if (referenceWorld->getConstraintSolver()->getLastCollisionResult().getNumContacts()) ++contactSteps;
  }
  require(contactSteps > 100, "Contact-world coverage missing");
  std::vector<std::size_t> dimensions(80, 3);
  double referenceSeconds = 0., candidateSeconds = 0.;
  for (int round = 0; round < 4; ++round) {
    if (round % 2) {
      candidateSeconds += exercise(candidate, dimensions, 0, 0, true, 300, false).elapsed;
      referenceSeconds += exercise(native, dimensions, 0, 0, true, 300, false).elapsed;
    } else {
      referenceSeconds += exercise(native, dimensions, 0, 0, true, 300, false).elapsed;
      candidateSeconds += exercise(candidate, dimensions, 0, 0, true, 300, false).elapsed;
    }
  }
  std::cout.precision(17);
  std::cout << "{\"status\":\"PASS_EXACT_SOLVER_COMPONENT_NOT_FULL_TASK\",\"matrix_cases\":" << cases
            << ",\"real_world_steps\":1200,\"real_contact_steps\":" << contactSteps
            << ",\"native_s\":" << referenceSeconds << ",\"candidate_s\":" << candidateSeconds
            << ",\"component_speedup\":" << referenceSeconds / candidateSeconds << "}" << std::endl;
}

