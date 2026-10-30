#include "ar3_cartesian_impedance/load.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ar3_cartesian_impedance;

namespace {
void require(bool condition, const char *message) {
  if(!condition) throw std::runtime_error(message);
}

template<typename F>
void require_invalid(F action) {
  try { action(); }
  catch(const std::invalid_argument &) { return; }
  throw std::runtime_error("invalid payload parameters unexpectedly accepted");
}

bool same_load(const PayloadLoad &a, const PayloadLoad &b) {
  return a.mass_kg == b.mass_kg && a.cog_m == b.cog_m && a.inertia_kg_m2 == b.inertia_kg_m2;
}

void parameter_validation_and_selection() {
  const std::vector<double> cog{-0.01, 0.02, -0.03};
  const std::vector<double> inertia{0.001, 0, 0.003};
  const auto configured = payload_load(1.2, cog, inertia);
  require(configured.mass_kg == 1.2 && configured.cog_m == std::array<double, 3>{-0.01, 0.02, -0.03}
      && configured.inertia_kg_m2 == std::array<double, 3>{0.001, 0, 0.003},
    "payload units or signed center of gravity changed");
  require(payload_load(0, {0, 0, 0}, {0, 0, 0}).mass_kg == 0,
    "zero payload and zero inertia must be accepted");

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for(const auto mass : {-0.01, nan, inf, -inf})
    require_invalid([&] { payload_load(mass, cog, inertia); });
  for(const auto count : {0U, 2U, 4U}) {
    require_invalid([&] { payload_load(1.2, std::vector<double>(count), inertia); });
    require_invalid([&] { payload_load(1.2, cog, std::vector<double>(count)); });
  }
  for(std::size_t axis = 0; axis < 3; ++axis) {
    for(const auto invalid : {nan, inf, -inf}) {
      auto invalid_cog = cog;
      invalid_cog[axis] = invalid;
      require_invalid([&] { payload_load(1.2, invalid_cog, inertia); });
    }
    for(const auto invalid : {-0.001, nan, inf, -inf}) {
      auto invalid_inertia = inertia;
      invalid_inertia[axis] = invalid;
      require_invalid([&] { payload_load(1.2, cog, invalid_inertia); });
    }
  }

  const auto controller = payload_load(0.8, {0.04, -0.05, 0.06}, {0.004, 0.005, 0.006});
  require(same_load(select_payload_load(false, controller, configured), controller),
    "disabled override must preserve every controller load field");
  require(same_load(select_payload_load(true, controller, configured), configured),
    "enabled override must select every configured load field");
}

}  // namespace

int main() {
  parameter_validation_and_selection();
  std::cout << "RT payload parameter validation and existing-controller/override selection passed\n";
}
