#pragma once

#include <algorithm>
#include <cmath>
#include <string>

struct CentrifugeSafety
{
  static constexpr double MaxLid = 2.0943951023931953;
  static constexpr double ClosedTolerance = 0.003490658503988659;
  static constexpr double StopTolerance = 0.02;
  static constexpr double RunSpeed = 3.141592653589793;
  double lidTarget = 0.0;
  double rotorTarget = 0.0;
  double lidPosition = 0.0;
  double lidVelocity = 0.0;
  double rotorVelocity = 0.0;
  bool ready = false;
  bool enabled = false;
  bool openLidDemo = false;
  std::string rejected;
  unsigned int blockedLid = 0;
  unsigned int blockedStart = 0;
  unsigned int starts = 0;
  unsigned int stops = 0;

  bool CanOpen() const
  {
    return ready && !enabled && std::abs(rotorVelocity) < StopTolerance &&
      std::abs(rotorTarget) < StopTolerance;
  }

  bool LidReady() const
  {
    if (!ready || !std::isfinite(lidPosition) || !std::isfinite(lidVelocity) ||
        !std::isfinite(lidTarget) || std::abs(lidVelocity) >= 0.01)
      return false;
    if (openLidDemo)
      return lidPosition >= -ClosedTolerance && lidPosition <= MaxLid + ClosedTolerance &&
        lidTarget >= 0.0 && lidTarget <= MaxLid &&
        std::abs(lidPosition - lidTarget) < ClosedTolerance;
    return std::abs(lidPosition) < ClosedTolerance && lidTarget < ClosedTolerance;
  }

  bool CanStart() const
  {
    return CanOpen() && LidReady();
  }

  bool RequestLid(double target)
  {
    if (!ready || !std::isfinite(target) || target < 0.0 || target > MaxLid)
    {
      rejected = "INVALID_LID_REQUEST";
      ++blockedLid;
      return false;
    }
    if ((target > ClosedTolerance || openLidDemo) && !CanOpen())
    {
      rejected = "ROTOR_NOT_STOPPED";
      ++blockedLid;
      return false;
    }
    lidTarget = target;
    rejected.clear();
    return true;
  }

  bool RequestRotor(bool run, bool leaseFresh)
  {
    if (!run)
    {
      if (enabled) ++stops;
      enabled = false;
      rejected.clear();
      return true;
    }
    if (!leaseFresh || !CanStart())
    {
      rejected = leaseFresh ? "LID_NOT_READY_OR_ROTOR_STOPPING" : "NO_COMMAND_LEASE";
      ++blockedStart;
      return false;
    }
    enabled = true;
    ++starts;
    rejected.clear();
    return true;
  }

  void Step(double duration, bool leaseFresh)
  {
    if (enabled && (!leaseFresh || !LidReady()))
    {
      enabled = false;
      rejected = "SAFETY_STOP";
      ++stops;
    }
    const double goal = enabled ? RunSpeed : 0.0;
    rotorTarget += std::clamp(goal - rotorTarget, -duration, duration);
  }
};
