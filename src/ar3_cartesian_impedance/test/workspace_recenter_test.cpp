#include "ar3_cartesian_impedance/workspace_recenter.hpp"
#include <iostream>
#include <limits>
using namespace ar3_cartesian_impedance;
void require(bool ok) { if(!ok) throw std::runtime_error("workspace recenter regression"); }
int main() {
  WorkspaceRecenterGate gate;
  Command target{}; target.pose[0] = .05;
  Six measured = target.pose, output = target.pose;
  require(!gate.observe(measured, target, output, 1000000000));
  require(!gate.observe(measured, target, output, 1299000000));
  require(gate.observe(measured, target, output, 1300000000));
  require(!gate.observe(measured, target, output, 2300000000));  // once only
  gate.reset(); target.wrench[0] = 1;
  require(!gate.observe(measured, target, output, 3000000000));
  require(!gate.observe(measured, target, output, 4000000000));
  target.wrench[0] = 0;
  require(!gate.observe(measured, target, output, 5000000000));
  measured[0] += .002;  // drift restarts the stability window
  require(!gate.observe(measured, target, output, 5300000000));
  require(!gate.observe(measured, target, output, 5599000000));
  require(gate.observe(measured, target, output, 5600000000));
  gate.reset(); output[0] += .01;  // follower still moving
  require(!gate.observe(measured, target, output, 6000000000));
  require(!gate.observe(measured, target, output, 7000000000));
  output = target.pose; measured[0] = .1;  // not settled
  require(!gate.observe(measured, target, output, 8000000000));
  require(!gate.observe(measured, target, output, 9000000000));
  measured = target.pose; measured[3] = std::numeric_limits<double>::quiet_NaN();
  require(!gate.observe(measured, target, output, 10000000000));
  measured = target.pose;
  require(!gate.observe(measured, target, output, 11000000000));
  target.pose[5] += .005; output = target.pose;
  require(!gate.observe(measured, target, output, 11300000000));
  require(gate.observe(measured, target, output, 11600000000));
  Six end = measured; end[0] += .251;
  bool rejected = false;
  try { validate_target(end, end, measured, .05, .15, .25); }
  catch(const std::invalid_argument &) { rejected = true; }
  require(rejected);
  std::cout << "PASS: stable hold, one-shot, reset, drift, moving target, finite poses, zero wrench, radius retained\n";
}
