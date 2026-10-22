#include "../systems/CentrifugeSafety.hh"

#include <limits>
#include <stdexcept>

void require(bool condition)
{
  if (!condition) throw std::runtime_error("Centrifuge safety assertion failed");
}

int main()
{
  CentrifugeSafety safety;
  require(!safety.openLidDemo);
  require(!safety.RequestRotor(true, true));
  require(!safety.RequestLid(1.0));
  safety.ready = true;
  require(!safety.RequestRotor(true, false));
  require(safety.RequestLid(CentrifugeSafety::MaxLid));
  require(!safety.RequestRotor(true, true));
  require(safety.RequestLid(0.0));
  safety.lidPosition = 0.1;
  require(!safety.RequestRotor(true, true));
  safety.lidPosition = 0.0;
  require(safety.RequestRotor(true, true));
  require(!safety.RequestLid(1.0));
  require(safety.lidTarget == 0.0);
  safety.Step(0.1, true);
  require(std::abs(safety.rotorTarget - 0.1) < 1e-9);
  safety.rotorVelocity = 1.0;
  require(safety.RequestRotor(false, true));
  require(!safety.RequestLid(1.0));
  safety.Step(0.1, true);
  safety.rotorVelocity = 0.0;
  require(safety.RequestLid(1.0));
  require(!safety.RequestLid(std::numeric_limits<double>::quiet_NaN()));
  require(!safety.RequestLid(-0.1));
  require(!safety.RequestLid(CentrifugeSafety::MaxLid + 0.01));
  safety.RequestLid(0.0);
  require(safety.RequestRotor(true, true));
  safety.Step(0.1, false);
  require(!safety.enabled);
  require(safety.RequestRotor(true, true));
  safety.lidPosition = 0.1;
  safety.Step(0.1, true);
  require(!safety.enabled);

  CentrifugeSafety demo;
  demo.openLidDemo = true;
  demo.ready = true;
  demo.lidPosition = CentrifugeSafety::MaxLid;
  require(demo.RequestLid(CentrifugeSafety::MaxLid));
  require(!demo.RequestRotor(true, false));
  require(demo.RequestRotor(true, true));
  for (int step = 0; step < 100; ++step)
    demo.Step(0.1, true);
  require(demo.enabled);
  require(std::abs(demo.rotorTarget - CentrifugeSafety::RunSpeed) < 1e-9);
  require(!demo.RequestLid(0.0));
  require(!demo.RequestLid(1.0));
  require(demo.lidTarget == CentrifugeSafety::MaxLid);
  demo.rotorVelocity = CentrifugeSafety::RunSpeed;
  require(demo.RequestRotor(false, true));
  require(!demo.RequestLid(0.0));
  for (int step = 0; step < 100; ++step)
    demo.Step(0.1, true);
  require(!demo.CanStart());
  demo.rotorVelocity = 0.0;
  require(demo.CanStart());
  require(demo.RequestRotor(true, true));
  demo.Step(0.1, false);
  require(!demo.enabled && demo.rejected == "SAFETY_STOP");
  require(demo.RequestRotor(true, true));
  demo.lidVelocity = 0.02;
  demo.Step(0.1, true);
  require(!demo.enabled);
  require(!demo.CanStart());
  demo.lidVelocity = 0.0;
  demo.lidPosition = 1.0;
  require(!demo.CanStart());
  require(demo.RequestLid(1.0));
  require(demo.RequestRotor(true, true));
  demo.lidPosition = 1.1;
  demo.Step(0.1, true);
  require(!demo.enabled);
  demo.lidPosition = CentrifugeSafety::MaxLid + 0.01;
  require(!demo.CanStart());
  demo.lidPosition = std::numeric_limits<double>::quiet_NaN();
  require(!demo.CanStart());
  demo.lidPosition = 1.0;
  demo.ready = false;
  require(!demo.CanStart());
  return 0;
}
