#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ar3_cartesian_impedance {
struct PayloadLoad {
  double mass_kg{};
  std::array<double, 3> cog_m{}, inertia_kg_m2{};
};

inline PayloadLoad payload_load(double mass, const std::vector<double> &cog, const std::vector<double> &inertia) {
  if(!std::isfinite(mass) || mass < 0) throw std::invalid_argument("load_mass_kg must be finite and nonnegative");
  if(cog.size() != 3 || !std::all_of(cog.begin(), cog.end(), [](double value) { return std::isfinite(value); }))
    throw std::invalid_argument("load_cog_m must contain three finite values");
  if(inertia.size() != 3 || !std::all_of(inertia.begin(), inertia.end(), [](double value) { return std::isfinite(value) && value >= 0; }))
    throw std::invalid_argument("load_inertia_kg_m2 must contain three finite, nonnegative values");
  PayloadLoad result;
  result.mass_kg = mass;
  std::copy(cog.begin(), cog.end(), result.cog_m.begin());
  std::copy(inertia.begin(), inertia.end(), result.inertia_kg_m2.begin());
  return result;
}

inline PayloadLoad select_payload_load(bool override_enabled, const PayloadLoad &controller, const PayloadLoad &override) {
  return override_enabled ? override : controller;
}

}  // namespace ar3_cartesian_impedance
