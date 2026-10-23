#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/BoxedLcpSolver.hpp>
#include <dart/constraint/ConstrainedGroup.hpp>
#include <dart/constraint/ConstraintBase.hpp>
#include <dart/common/Console.hpp>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <map>
#include <sstream>
#include <stdexcept>
#include <sys/syscall.h>
#include <unistd.h>
#include <vector>

#include <dart/constraint/ContactConstraint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <unordered_map>

namespace {
using Body = dart::dynamics::BodyNode;
using ReactiveFunction = bool (*)(const Body*);
using ResponseFunction = void (*)(dart::constraint::ContactConstraint*, double*, bool);
template<typename Function>
Function nativeQuery(const char* name)
{
  auto address = reinterpret_cast<Function>(dlsym(RTLD_NEXT, name));
  if (!address)
    throw std::runtime_error("Original DART query unavailable");
  return address;
}
struct BodyQuery {
  dart::dynamics::ConstSkeletonPtr owner;
  bool reactive;
};
thread_local bool queriesEnabled = true;
thread_local bool queriesActive = false;
thread_local std::unordered_map<const Body*, BodyQuery> bodyQueries;
thread_local unsigned long long queryHits = 0;
thread_local unsigned long long queryMisses = 0;
thread_local unsigned long long responseCalls = 0;
thread_local bool inertiaEnabled = true;
thread_local unsigned long long inertiaQueries = 0;
thread_local unsigned long long implicitInertiaQueries = 0;
thread_local unsigned long long inertiaRefreshes = 0;
bool nativeReactive(const Body* body)
{
  static const auto function = nativeQuery<ReactiveFunction>("_ZNK4dart8dynamics8BodyNode10isReactiveEv");
  return function(body);
}
const BodyQuery& queryBody(const Body* body)
{
  const auto found = bodyQueries.find(body);
  if (found != bodyQueries.end()) {
    ++queryHits;
    return found->second;
  }
  ++queryMisses;
  return bodyQueries.emplace(body, BodyQuery{body->getSkeleton(), nativeReactive(body)}).first->second;
}
struct QueryScope {
  bool entered;
  QueryScope() : entered(queriesEnabled && !queriesActive)
  {
    if (entered) {
      bodyQueries.clear();
      queriesActive = true;
    }
  }
  ~QueryScope()
  {
    if (entered) {
      queriesActive = false;
      bodyQueries.clear();
    }
  }
};
}

extern "C" void h1_queries_set_enabled(bool value)
{
  if (queriesActive)
    throw std::runtime_error("Query mode change during solve");
  queriesEnabled = value;
}
extern "C" unsigned long long h1_queries_hits() { return queryHits; }
extern "C" unsigned long long h1_queries_misses() { return queryMisses; }
extern "C" unsigned long long h1_queries_responses() { return responseCalls; }
extern "C" unsigned long long h1_queries_retained() { return bodyQueries.size(); }
extern "C" void h1_inertia_set_enabled(bool value)
{
  if (queriesActive)
    throw std::runtime_error("Inertia mode change during solve");
  inertiaEnabled = value;
}
extern "C" unsigned long long h1_inertia_queries() { return inertiaQueries; }
extern "C" unsigned long long h1_inertia_implicit_queries() { return implicitInertiaQueries; }
extern "C" unsigned long long h1_inertia_refreshes() { return inertiaRefreshes; }

namespace dart::dynamics {
const math::Inertia& BodyNode::getArticulatedInertia() const
{
  if (!queriesActive || !inertiaEnabled) {
    using Function = const math::Inertia& (*)(const Body*);
    static const auto function = nativeQuery<Function>("_ZNK4dart8dynamics8BodyNode21getArticulatedInertiaEv");
    return function(this);
  }
  ++inertiaQueries;
  const auto& owner = queryBody(this).owner;
  if (owner && owner->mTreeCache[mTreeIndex].mDirty.mArticulatedInertia) {
    ++inertiaRefreshes;
    owner->updateArticulatedInertia(mTreeIndex);
  }
  return mArtInertia;
}
const math::Inertia& BodyNode::getArticulatedInertiaImplicit() const
{
  if (!queriesActive || !inertiaEnabled) {
    using Function = const math::Inertia& (*)(const Body*);
    static const auto function = nativeQuery<Function>("_ZNK4dart8dynamics8BodyNode29getArticulatedInertiaImplicitEv");
    return function(this);
  }
  ++implicitInertiaQueries;
  const auto& owner = queryBody(this).owner;
  if (owner && owner->mTreeCache[mTreeIndex].mDirty.mArticulatedInertia) {
    ++inertiaRefreshes;
    owner->updateArticulatedInertia(mTreeIndex);
  }
  return mArtInertiaImplicit;
}
bool BodyNode::isReactive() const
{
  return queriesActive ? queryBody(this).reactive : nativeReactive(this);
}
}

namespace dart::constraint {
void ContactConstraint::getVelocityChange(double* velocity, bool withCfm)
{
  if (!queriesActive) {
    static const auto function = nativeQuery<ResponseFunction>("_ZN4dart10constraint17ContactConstraint17getVelocityChangeEPdb");
    function(this, velocity, withCfm);
    return;
  }
  ++responseCalls;
  const auto& bodyA = queryBody(mBodyNodeA);
  const auto& bodyB = queryBody(mBodyNodeB);
  Eigen::Map<Eigen::VectorXd> velocityMap(velocity, static_cast<int>(mDim));
  velocityMap.setZero();
  if (bodyA.owner->isImpulseApplied() && bodyA.reactive)
    velocityMap += mSpatialNormalA.transpose() * mBodyNodeA->getBodyVelocityChange();
  if (bodyB.owner->isImpulseApplied() && bodyB.reactive)
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

namespace {
using Clock = std::chrono::steady_clock;
using Solver = dart::constraint::BoxedLcpConstraintSolver;
using Group = dart::constraint::ConstrainedGroup;
using Original = void (*)(Solver*, Group&);
const char* symbol = "_ZN4dart10constraint24BoxedLcpConstraintSolver21solveConstrainedGroupERNS0_16ConstrainedGroupE";

Original original()
{
  static auto address = reinterpret_cast<Original>(dlsym(RTLD_NEXT, symbol));
  if (!address)
    throw std::runtime_error("Original DART solver symbol unavailable");
  return address;
}

struct SampleLog {
  int descriptor = -1;
  std::size_t calls = 0;
  SampleLog()
  {
    const char* path = std::getenv("H1_DART_CACHE_LOG");
    if (path && *path) {
      descriptor = ::open(path, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
      if (descriptor < 0)
        throw std::runtime_error("DART sample log must be new");
    }
  }
  ~SampleLog() { if (descriptor >= 0) ::close(descriptor); }
  void write(const std::string& data)
  {
    std::size_t offset = 0;
    while (offset < data.size()) {
      const auto count = ::write(descriptor, data.data() + offset, data.size() - offset);
      if (count <= 0)
        throw std::runtime_error("DART sample log write failed");
      offset += count;
    }
  }
};
double seconds(Clock::time_point start, Clock::time_point finish)
{
  return std::chrono::duration<double>(finish - start).count();
}
}

namespace dart::constraint {
void BoxedLcpConstraintSolver::solveConstrainedGroup(ConstrainedGroup& group)
{
  static const bool enabled = [] {
    const char* mode = std::getenv("H1_DART_CACHE_MODE");
    return mode && std::strcmp(mode, "cached") == 0;
  }();
  if (!enabled) {
    original()(this, group);
    return;
  }
  QueryScope queryScope;
  const std::size_t constraintCount = group.getNumConstraints();
  const std::size_t totalDimension = group.getTotalDimension();
  if (totalDimension == 0)
    return;
  thread_local SampleLog samples;
  const bool sampled = samples.descriptor >= 0 && (++samples.calls % 1024 == 1);
  const auto started = sampled ? Clock::now() : Clock::time_point{};
  thread_local std::vector<ConstraintBase*> constraints;
  thread_local std::vector<std::size_t> dimensions;
  constraints.resize(constraintCount);
  dimensions.resize(constraintCount);
  mOffset.resize(constraintCount);
  int offset = 0;
  for (std::size_t constraintIndex = 0; constraintIndex < constraintCount; ++constraintIndex) {
    constraints[constraintIndex] = group.getConstraint(constraintIndex).get();
    dimensions[constraintIndex] = constraints[constraintIndex]->getDimension();
    mOffset[constraintIndex] = offset;
    offset += dimensions[constraintIndex];
  }
  const int rowStride = totalDimension <= 1 ? totalDimension : ((totalDimension + 3) & ~std::size_t(3));
  mA.resize(totalDimension, rowStride);
  mX.resize(totalDimension);
  mB.resize(totalDimension);
  mW.setZero(totalDimension);
  mLo.resize(totalDimension);
  mHi.resize(totalDimension);
  mFIndex.setConstant(totalDimension, -1);
  ConstraintInfo information;
  information.invTimeStep = 1.0 / mTimeStep;
  for (std::size_t constraintIndex = 0; constraintIndex < constraintCount; ++constraintIndex) {
    auto* constraint = constraints[constraintIndex];
    const int firstRow = mOffset[constraintIndex];
    information.x = mX.data() + firstRow;
    information.lo = mLo.data() + firstRow;
    information.hi = mHi.data() + firstRow;
    information.b = mB.data() + firstRow;
    information.findex = mFIndex.data() + firstRow;
    information.w = mW.data() + firstRow;
    constraint->getInformation(&information);
    constraint->excite();
    for (std::size_t localRow = 0; localRow < dimensions[constraintIndex]; ++localRow) {
      const int row = firstRow + localRow;
      if (mFIndex[row] >= 0)
        mFIndex[row] += firstRow;
      constraint->applyUnitImpulse(localRow);
      double* rowData = mA.data() + rowStride * row;
      constraint->getVelocityChange(rowData + firstRow, true);
      for (std::size_t otherIndex = constraintIndex + 1; otherIndex < constraintCount; ++otherIndex)
        constraints[otherIndex]->getVelocityChange(rowData + mOffset[otherIndex], false);
      for (int column = 0; column < firstRow; ++column)
        rowData[column] = mA.data()[rowStride * column + row];
    }
    constraint->unexcite();
  }
  const auto assembled = sampled ? Clock::now() : Clock::time_point{};
  if (mSecondaryBoxedLcpSolver) {
    mABackup = mA;
    mXBackup = mX;
    mBBackup = mB;
    mLoBackup = mLo;
    mHiBackup = mHi;
    mFIndexBackup = mFIndex;
  }
  const bool earlyTermination = mSecondaryBoxedLcpSolver != nullptr;
  bool success = mBoxedLcpSolver->solve(totalDimension, mA.data(), mX.data(),
      mB.data(), 0, mLo.data(), mHi.data(), mFIndex.data(), earlyTermination);
  if (success && mX.hasNaN())
    success = false;
  const bool fallback = !success && mSecondaryBoxedLcpSolver;
  if (fallback) {
    mSecondaryBoxedLcpSolver->solve(totalDimension, mABackup.data(), mXBackup.data(),
        mBBackup.data(), 0, mLoBackup.data(), mHiBackup.data(), mFIndexBackup.data(), false);
    mX = mXBackup;
  }
  if (mX.hasNaN()) {
    dterr << "[H1DartCache] Original NaN impulse-to-zero policy applied\n";
    mX.setZero();
  }
  const auto solved = sampled ? Clock::now() : Clock::time_point{};
  for (std::size_t constraintIndex = 0; constraintIndex < constraintCount; ++constraintIndex) {
    constraints[constraintIndex]->applyImpulse(mX.data() + mOffset[constraintIndex]);
    constraints[constraintIndex]->excite();
  }
  if (sampled) {
    const auto completed = Clock::now();
    std::map<std::string, std::size_t> types;
    for (auto* constraint : constraints)
      ++types[constraint->getType()];
    std::ostringstream record;
    record.precision(17);
    record << "{\"pid\":" << getpid() << ",\"tid\":" << syscall(SYS_gettid)
           << ",\"call\":" << samples.calls << ",\"mono\":"
           << seconds(Clock::time_point{}, started) << ",\"constraints\":" << constraintCount
           << ",\"dimension\":" << totalDimension << ",\"assembly_s\":" << seconds(started, assembled)
           << ",\"lcp_s\":" << seconds(assembled, solved) << ",\"apply_s\":" << seconds(solved, completed)
           << ",\"fallback\":" << (fallback ? "true" : "false")
           << ",\"query_hits\":" << queryHits << ",\"query_misses\":" << queryMisses
           << ",\"query_responses\":" << responseCalls
           << ",\"inertia_queries\":" << inertiaQueries
           << ",\"implicit_inertia_queries\":" << implicitInertiaQueries
           << ",\"inertia_refreshes\":" << inertiaRefreshes << ",\"types\":{";
    bool first = true;
    for (const auto& entry : types) {
      if (!first) record << ',';
      first = false;
      record << '"' << entry.first << "\":" << entry.second;
    }
    record << "}}\n";
    samples.write(record.str());
  }
}
}
