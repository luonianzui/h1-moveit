#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ConstrainedGroup.hpp>
#include <dart/constraint/ConstraintBase.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>

using Body = dart::dynamics::BodyNode;
using Native = const dart::math::Inertia& (*)(const Body*);
using Counter = unsigned long long (*)();
using Mode = void (*)(bool);
using Solve = void (*)(dart::constraint::BoxedLcpConstraintSolver*, dart::constraint::ConstrainedGroup&);

void require(bool value, const char* message)
{
  if (!value) throw std::runtime_error(message);
}

class Probe : public dart::constraint::ConstraintBase {
public:
  dart::dynamics::SkeletonPtr skeleton;
  Native native;
  Native implicit;
  Mode mode;
  std::size_t checks = 0;
  std::size_t rejections = 0;
  Probe(dart::dynamics::SkeletonPtr owner, Native regular, Native damped, Mode change)
      : skeleton(owner), native(regular), implicit(damped), mode(change)
  {
    mDim = 1;
  }
  void compare()
  {
    for (auto* body : skeleton->getBodyNodes()) {
      for (bool damped : {false, true}) {
        const dart::math::Inertia actual = damped ? body->getArticulatedInertiaImplicit() : body->getArticulatedInertia();
        const dart::math::Inertia expected = (damped ? implicit : native)(body);
        require(std::memcmp(actual.data(), expected.data(), sizeof(double) * 36) == 0,
                "Dirty or clean inertia differs inside solve");
        ++checks;
      }
    }
  }
  void update() override {}
  void getInformation(dart::constraint::ConstraintInfo* information) override
  {
    compare();
    auto* body = skeleton->getBodyNode(1);
    body->setMass(body->getMass() + .01);
    compare();
    body->setLocalCOM(Eigen::Vector3d(.01, .02, .03));
    compare();
    skeleton->setPosition(6, skeleton->getPosition(6) + .001);
    compare();
    body->getParentJoint()->setDampingCoefficient(0, .2);
    skeleton->setTimeStep(.0006);
    compare();
    try {
      mode(false);
      require(false, "Mode changed inside solve");
    } catch (const std::runtime_error& error) {
      require(std::string(error.what()) == "Inertia mode change during solve", "Wrong mode rejection");
      ++rejections;
    }
    information->x[0] = 0.;
    information->b[0] = 0.;
    information->lo[0] = 0.;
    information->hi[0] = 1.;
    information->findex[0] = -1;
  }
  void applyUnitImpulse(std::size_t) override { compare(); }
  void getVelocityChange(double* values, bool) override { values[0] = 1.; }
  void excite() override {}
  void unexcite() override {}
  void applyImpulse(double*) override { compare(); }
  bool isActive() const override { return true; }
  dart::dynamics::SkeletonPtr getRootSkeleton() const override { return skeleton; }
};

int main(int argc, char** argv)
{
  require(argc == 2, "Expected candidate");
  auto nativeHandle = dlopen("/opt/ros/jazzy/opt/gz_dartsim_vendor/lib/libdart.so.6.13.2", RTLD_NOW | RTLD_LOCAL);
  auto candidateHandle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  require(nativeHandle && candidateHandle, "Library unavailable");
  auto native = reinterpret_cast<Native>(dlsym(nativeHandle, "_ZNK4dart8dynamics8BodyNode21getArticulatedInertiaEv"));
  auto implicit = reinterpret_cast<Native>(dlsym(nativeHandle, "_ZNK4dart8dynamics8BodyNode29getArticulatedInertiaImplicitEv"));
  auto mode = reinterpret_cast<Mode>(dlsym(candidateHandle, "h1_inertia_set_enabled"));
  auto refreshes = reinterpret_cast<Counter>(dlsym(candidateHandle, "h1_inertia_refreshes"));
  auto retained = reinterpret_cast<Counter>(dlsym(candidateHandle, "h1_queries_retained"));
  auto solve = reinterpret_cast<Solve>(dlsym(candidateHandle, "_ZN4dart10constraint24BoxedLcpConstraintSolver21solveConstrainedGroupERNS0_16ConstrainedGroupE"));
  require(native && implicit && mode && refreshes && retained && solve, "Missing symbols");
  auto skeleton = dart::dynamics::Skeleton::create("dirty_probe");
  auto root = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(root.second);
  skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto probe = std::make_shared<Probe>(skeleton, native, implicit, mode);
  dart::constraint::ConstrainedGroup group;
  group.addConstraint(probe);
  dart::constraint::BoxedLcpConstraintSolver solver;
  mode(true);
  for (int repeat = 0; repeat < 30; ++repeat) {
    solve(&solver, group);
    require(retained() == 0, "References escaped scope");
  }
  require(probe->checks == 1260 && probe->rejections == 30 && refreshes() >= 60, "Coverage missing");
  probe->compare();
  std::cout << "{\"status\":\"PASS_DIRTY_INVALIDATION_INSIDE_SOLVE\",\"checks\":" << probe->checks
            << ",\"mode_rejections\":" << probe->rejections << ",\"dirty_refreshes\":" << refreshes()
            << ",\"retained\":" << retained() << "}" << std::endl;
}
