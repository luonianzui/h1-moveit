#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>

using Mode = void (*)(bool);
using Counter = unsigned long long (*)();
using Clock = std::chrono::steady_clock;

void require(bool value, const char* message)
{
  if (!value) throw std::runtime_error(message);
}

void shape(dart::dynamics::BodyNode* body, const Eigen::Vector3d& size)
{
  body->createShapeNodeWith<dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(size));
  body->setMass(.5);
}

dart::simulation::WorldPtr world(int scenario)
{
  auto result = dart::simulation::World::create();
  result->setTimeStep(.0005);
  auto floor = dart::dynamics::Skeleton::create("floor");
  auto ground = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().z() = -.05;
  ground.first->setTransformFromParentBodyNode(pose);
  shape(ground.second, Eigen::Vector3d(2., 2., .1));
  result->addSkeleton(floor);
  for (int index = 0; index < 3; ++index) {
    auto object = dart::dynamics::Skeleton::create("body" + std::to_string(index));
    auto root = object->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
    shape(root.second, Eigen::Vector3d(.1, .1, .1));
    auto positions = object->getPositions();
    positions[0] = .01 * index;
    positions[5] = .08 + .12 * index;
    object->setPositions(positions);
    if (scenario > 0) {
      auto parent = root.second;
      for (int child = 0; child < 4; ++child) {
        auto jointBody = object->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(parent);
        jointBody.first->setAxis(Eigen::Vector3d::UnitY());
        pose.setIdentity();
        pose.translation().x() = .08;
        jointBody.first->setTransformFromParentBodyNode(pose);
        jointBody.first->setDampingCoefficient(0, .1);
        jointBody.first->setPositionLowerLimit(0, -.3);
        jointBody.first->setPositionUpperLimit(0, .3);
        jointBody.first->setLimitEnforcement(true);
        shape(jointBody.second, Eigen::Vector3d(.07, .05, .05));
        parent = jointBody.second;
      }
    }
    if (scenario == 2) {
      auto independent = object->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
      shape(independent.second, Eigen::Vector3d(.1, .1, .1));
      Eigen::Vector6d independentPosition = Eigen::Vector6d::Zero();
      independentPosition[4] = .02;
      independentPosition[5] = .25 + .12 * index;
      independent.first->setPositions(independentPosition);
      object->enableSelfCollisionCheck();
    }
    result->addSkeleton(object);
  }
  return result;
}

void changeBetweenSolves(const dart::simulation::WorldPtr& scene, int step)
{
  auto object = scene->getSkeleton("body2");
  if (step == 800) object->setMobile(false);
  if (step == 900) object->setMobile(true);
  if (object->getNumBodyNodes() > 1) {
    if (step == 1000) object->getBodyNode(1)->getParentJoint()->setActuatorType(dart::dynamics::Joint::LOCKED);
    if (step == 1200) object->getBodyNode(1)->getParentJoint()->setActuatorType(dart::dynamics::Joint::PASSIVE);
  }
}

int main(int argc, char** argv)
{
  require(argc == 2, "Expected candidate library");
  auto handle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  require(handle != nullptr, "Candidate library unavailable");
  auto mode = reinterpret_cast<Mode>(dlsym(handle, "h1_queries_set_enabled"));
  auto hits = reinterpret_cast<Counter>(dlsym(handle, "h1_queries_hits"));
  auto misses = reinterpret_cast<Counter>(dlsym(handle, "h1_queries_misses"));
  auto responses = reinterpret_cast<Counter>(dlsym(handle, "h1_queries_responses"));
  auto retained = reinterpret_cast<Counter>(dlsym(handle, "h1_queries_retained"));
  require(mode && hits && misses && responses && retained, "Candidate counters unavailable");
  std::size_t steps = 0, contactSteps = 0, vectorChecks = 0;
  double baselineSeconds = 0., candidateSeconds = 0.;
  for (int scenario = 0; scenario < 3; ++scenario) {
    auto reference = world(scenario);
    auto candidate = world(scenario);
    for (int step = 0; step < 1800; ++step) {
      changeBetweenSolves(reference, step);
      changeBetweenSolves(candidate, step);
      for (int order = 0; order < 2; ++order) {
        const bool optimized = (step + order) % 2;
        mode(optimized);
        const auto began = Clock::now();
        (optimized ? candidate : reference)->step();
        const auto elapsed = std::chrono::duration<double>(Clock::now() - began).count();
        if (optimized) candidateSeconds += elapsed; else baselineSeconds += elapsed;
        require(retained() == 0, "Cached references escaped solve scope");
      }
      for (std::size_t index = 0; index < reference->getNumSkeletons(); ++index) {
        auto first = reference->getSkeleton(index);
        auto second = candidate->getSkeleton(index);
        for (const auto& values : {std::make_pair(first->getPositions(), second->getPositions()),
                                   std::make_pair(first->getVelocities(), second->getVelocities())}) {
          require(values.first.allFinite() && values.second.allFinite(), "Nonfinite world state");
          require(values.first.size() == values.second.size(), "World dimensions differ");
          require(std::memcmp(values.first.data(), values.second.data(), sizeof(double) * values.first.size()) == 0,
                  "World state differs from r353 solver");
          ++vectorChecks;
        }
        for (std::size_t body = 0; body < first->getNumBodyNodes(); ++body)
          require(first->getBodyNode(body)->isReactive() == second->getBodyNode(body)->isReactive(),
                  "Outside-solve reactive fallback differs");
      }
      auto firstContacts = reference->getConstraintSolver()->getLastCollisionResult().getNumContacts();
      require(firstContacts == candidate->getConstraintSolver()->getLastCollisionResult().getNumContacts(),
              "Contact count differs");
      if (firstContacts) ++contactSteps;
      ++steps;
    }
  }
  require(hits() > 10000 && misses() > 1000 && responses() > 10000, "Query cache not exercised");
  require(contactSteps > 1000, "Contact coverage missing");
  std::cout.precision(17);
  std::cout << "{\"status\":\"PASS_EXACT_GROUP_QUERIES_NOT_FULL_TASK\",\"world_steps\":" << steps
            << ",\"vector_checks\":" << vectorChecks << ",\"contact_steps\":" << contactSteps
            << ",\"hits\":" << hits() << ",\"misses\":" << misses() << ",\"responses\":" << responses()
            << ",\"retained_after_solve\":" << retained()
            << ",\"baseline_s\":" << baselineSeconds << ",\"candidate_s\":" << candidateSeconds
            << ",\"component_speedup\":" << baselineSeconds / candidateSeconds << "}" << std::endl;
}

