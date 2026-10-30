#include "ar3_cartesian_impedance/command.hpp"
#include <iostream>
#include <limits>

using namespace ar3_cartesian_impedance;

template<typename F> void must_reject(F call) {
  try { call(); } catch(const std::invalid_argument &) { return; }
  throw std::runtime_error("unsafe command unexpectedly accepted");
}
void require(bool condition, const char *description) {
  if(!condition) throw std::runtime_error(description);
}
int main() {
  Six zero{}, limits{20,20,20,3,3,3}, max_k{1000,1000,1000,100,100,100};
  Command sample{zero, zero, {500,500,500,50,50,50}, zero, zero};
  auto values = encode(sample);
  require(decode(values, zero, zero, limits, max_k).stiffness[0] == 500, "normal command");
  auto check_invalid = [&](std::size_t index, double value) {
    auto invalid = values; invalid[index] = value;
    must_reject([&] { decode(invalid, zero, zero, limits, max_k); });
  };
  must_reject([&] { decode(std::vector<double>(29), zero, zero, limits, max_k); });
  check_invalid(0, std::numeric_limits<double>::quiet_NaN());
  check_invalid(6, 20.1);
  check_invalid(9, -3.1);
  check_invalid(12, -1);
  check_invalid(12, 1001);
  check_invalid(18, 1);  // this RT implementation preserves unspecified controller damping
  check_invalid(24, 1);  // no Cartesian virtual-mass setter
  Six near{0.02,0,0,0,0,0}, far{0.04,0,0,0,0,0}, rotated{0,0,0,0,0,0.2};
  validate_target(zero, near, zero, 0.03, 0.15, 0.25);
  must_reject([&] { validate_target(zero, far, zero, 0.03, 0.15, 0.25); });
  must_reject([&] { validate_target(zero, rotated, zero, 0.03, 0.15, 0.25); });
  const Six wrap_a{0,0,0,0,0,3.14}, wrap_b{0,0,0,0,0,-3.14};
  validate_target(wrap_a, wrap_b, zero, 0.03, 0.15, 0.25);  // Euler wrap is a small physical rotation
  const Six quarter_turn{0,0,0,0,0,1.5707963267948966};
  const auto rotated_wrench = wrench_in_base({1,0,0,0,2,0}, quarter_turn);
  require(std::abs(rotated_wrench[1]-1) < 1e-10 && std::abs(rotated_wrench[3]+2) < 1e-10,
    "TCP force and torque must both rotate into base without a moment-arm shift");
  std::cout << "Command bounds, fixed D/M, angular wrap and wrench frames passed\n";
}
