#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>
#include <chrono>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>
#include <vector>

using Clock = std::chrono::steady_clock;
using Contact = dart::constraint::ContactConstraint;
using Response = void (*)(Contact*, double*, bool);
using Mode = void (*)(bool);
using Calls = unsigned long long (*)();

void require(bool value, const char* message)
{
  if (!value) throw std::runtime_error(message);
}

dart::simulation::WorldPtr world()
{
  auto result = dart::simulation::World::create();
  result->setTimeStep(.0005);
  auto floor = dart::dynamics::Skeleton::create("floor");
  auto floorPair = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  Eigen::Isometry3d floorPose = Eigen::Isometry3d::Identity();
  floorPose.translation().z() = -.05;
  floorPair.first->setTransformFromParentBodyNode(floorPose);
  floorPair.second->createShapeNodeWith<dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(2., 2., .1)));
  result->addSkeleton(floor);
  for (int index = 0; index < 3; ++index) {
    auto box = dart::dynamics::Skeleton::create("box" + std::to_string(index));
    auto pair = box->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
    pair.second->createShapeNodeWith<dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(
        std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(.1, .1, .1)));
    pair.second->setMass(.5);
    auto positions = box->getPositions();
    positions[0] = .01 * index;
    positions[5] = .08 + .10 * index;
    box->setPositions(positions);
    result->addSkeleton(box);
  }
  return result;
}

int main(int argc, char** argv)
{
  require(argc == 2, "Expected loaded candidate library");
  auto* nativeHandle = dlopen("/opt/ros/jazzy/opt/gz_dartsim_vendor/lib/libdart.so.6.13.2", RTLD_NOW | RTLD_LOCAL);
  auto* handle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  const char* symbol = "_ZN4dart10constraint17ContactConstraint17getVelocityChangeEPdb";
  auto native = reinterpret_cast<Response>(dlsym(nativeHandle, symbol));
  auto candidate = reinterpret_cast<Response>(dlsym(handle, symbol));
  auto mode = reinterpret_cast<Mode>(dlsym(handle, "h1_response_set_mode"));
  auto calls = reinterpret_cast<Calls>(dlsym(handle, "h1_response_candidate_calls"));
  require(native && candidate && native != candidate && mode && calls, "Distinct bound functions required");
  auto reference = world();
  auto optimized = world();
  std::size_t contactSteps = 0;
  for (int step = 0; step < 2400; ++step) {
    mode(false);
    reference->step();
    mode(true);
    optimized->step();
    for (int index = 0; index < 3; ++index) {
      const auto referenceBody = reference->getSkeleton("box" + std::to_string(index));
      const auto optimizedBody = optimized->getSkeleton("box" + std::to_string(index));
      const auto first = referenceBody->getPositions();
      const auto second = optimizedBody->getPositions();
      require(std::memcmp(first.data(), second.data(), sizeof(double) * first.size()) == 0, "World position bytes changed");
      const auto firstVelocity = referenceBody->getVelocities();
      const auto secondVelocity = optimizedBody->getVelocities();
      require(std::memcmp(firstVelocity.data(), secondVelocity.data(), sizeof(double) * firstVelocity.size()) == 0, "World velocity bytes changed");
    }
    if (reference->getConstraintSolver()->getLastCollisionResult().getNumContacts()) ++contactSteps;
  }
  require(calls() > 1000 && contactSteps > 1000, "Actual contact method not exercised");
  auto contacts = optimized->getConstraintSolver()->getLastCollisionResult();
  std::size_t cases = 0;
  double nativeSeconds = 0., candidateSeconds = 0.;
  for (std::size_t index = 0; index < contacts.getNumContacts(); ++index) {
    auto contactData = contacts.getContact(index);
    for (bool friction : {false, true}) {
      dart::constraint::ContactSurfaceParams parameters;
      parameters.mPrimaryFrictionCoeff = friction ? .6 : 0.;
      parameters.mSecondaryFrictionCoeff = friction ? .6 : 0.;
      Contact contact(contactData, .0005, parameters);
      dart::constraint::ConstraintBase& base = contact;
      base.update();
      base.excite();
      for (std::size_t axis = 0; axis < base.getDimension(); ++axis) {
        base.applyUnitImpulse(axis);
        for (bool cfm : {false, true}) {
          std::vector<double> first(base.getDimension()), second(base.getDimension());
          native(&contact, first.data(), cfm);
          candidate(&contact, second.data(), cfm);
          require(std::memcmp(first.data(), second.data(), sizeof(double) * first.size()) == 0, "Contact response bytes differ");
          ++cases;
          for (int round = 0; round < 6; ++round) {
            for (int position = 0; position < 2; ++position) {
              const bool optimizedFirst = (round + position) % 2;
              const auto function = optimizedFirst ? candidate : native;
              const auto began = Clock::now();
              for (int repeat = 0; repeat < 10000; ++repeat)
                function(&contact, first.data(), cfm);
              const double elapsed = std::chrono::duration<double>(Clock::now() - began).count();
              if (optimizedFirst) candidateSeconds += elapsed; else nativeSeconds += elapsed;
            }
          }
        }
      }
      base.unexcite();
    }
  }
  std::cout.precision(17);
  std::cout << "{\"status\":\"PASS_EXACT_CONTACT_RESPONSE_NOT_FULL_TASK\",\"world_steps\":2400,"
            << "\"contact_steps\":" << contactSteps << ",\"response_cases\":" << cases
            << ",\"native_s\":" << nativeSeconds << ",\"candidate_s\":" << candidateSeconds
            << ",\"component_speedup\":" << nativeSeconds / candidateSeconds
            << ",\"candidate_calls\":" << calls() << "}" << std::endl;
}
