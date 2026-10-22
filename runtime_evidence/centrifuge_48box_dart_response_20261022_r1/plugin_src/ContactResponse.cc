#include <dart/constraint/ContactConstraint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dlfcn.h>
#include <stdexcept>

namespace {
thread_local bool enabled = true;
thread_local unsigned long long candidateCalls = 0;
using Response = void (*)(dart::constraint::ContactConstraint*, double*, bool);
Response original()
{
  static auto address = reinterpret_cast<Response>(dlsym(RTLD_NEXT,
      "_ZN4dart10constraint17ContactConstraint17getVelocityChangeEPdb"));
  if (!address)
    throw std::runtime_error("Original contact response is unavailable");
  return address;
}
}

extern "C" void h1_response_set_mode(bool value) { enabled = value; }
extern "C" unsigned long long h1_response_candidate_calls() { return candidateCalls; }

namespace dart::constraint {
void ContactConstraint::getVelocityChange(double* velocity, bool withCfm)
{
  if (!enabled) {
    original()(this, velocity, withCfm);
    return;
  }
  ++candidateCalls;
  Eigen::Map<Eigen::VectorXd> velocityMap(velocity, static_cast<int>(mDim));
  velocityMap.setZero();
  if (mBodyNodeA->isReactive() && mBodyNodeA->getSkeleton()->isImpulseApplied())
    velocityMap += mSpatialNormalA.transpose() * mBodyNodeA->getBodyVelocityChange();
  if (mBodyNodeB->isReactive() && mBodyNodeB->getSkeleton()->isImpulseApplied())
    velocityMap += mSpatialNormalB.transpose() * mBodyNodeB->getBodyVelocityChange();
  if (withCfm) {
    velocity[mAppliedImpulseIndex] += velocity[mAppliedImpulseIndex] * mConstraintForceMixing;
    switch (mAppliedImpulseIndex) {
      case 1:
        velocity[1] += mPrimarySlipCompliance / mTimeStep;
        break;
      case 2:
        velocity[2] += mSecondarySlipCompliance / mTimeStep;
        break;
      default:
        break;
    }
  }
}
}
