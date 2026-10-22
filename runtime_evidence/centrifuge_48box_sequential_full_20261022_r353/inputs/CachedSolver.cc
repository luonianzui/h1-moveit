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
           << ",\"fallback\":" << (fallback ? "true" : "false") << ",\"types\":{";
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
