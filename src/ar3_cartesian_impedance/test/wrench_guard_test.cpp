#include "ar3_cartesian_impedance/wrench_guard.hpp"
#include <iostream>
#include <limits>

using namespace ar3_cartesian_impedance;
void require(bool condition, const char *message) { if(!condition) throw std::runtime_error(message); }
int main() {
  const Six limits{40,40,40,6,6,6};
  // Actual reported free-space takeover regression: must fail before startMove,
  // as well as at runtime, without interpreting or zeroing the raw channels.
  const Six reported{0.14,-14.58,0.85,6.04,-0.07,-5.0};
  const auto trip = check_wrench_limits(reported,limits);
  require(trip && trip.channel == 3, "reported 6.04 takeover no longer trips channel 4");
  require(trip.raw == reported && trip.limits == limits, "guard altered feedback or thresholds");
  const auto text = describe_wrench_violation(trip);
  require(text.find("channel 4=6.040 exceeds abs limit=6.000") != std::string::npos,
    "diagnostic lacks exact trip channel/value/limit");
  require(text.find("-14.580") != std::string::npos && text.find("no bias removed") != std::string::npos,
    "diagnostic lost signed feedback or provenance");
  require(describe_wrench_violation(trip, {0.2,-0.1,0.4,0.,0.,0.}).find("measured TCP/base=[0.20000,-0.10000,0.40000") != std::string::npos,
    "diagnostic omitted measured TCP needed to investigate the feedback reference point");
  require(!check_wrench_limits(limits,limits), "equal-to-limit sample spuriously trips");
  for(std::size_t axis=0; axis<6; ++axis) for(double sign : {-1.,1.}) {
    Six value{}; value[axis] = sign*(limits[axis] + 0.001);
    require(check_wrench_limits(value,limits).channel == axis, "signed over-limit component missed");
    value[axis] = std::numeric_limits<double>::quiet_NaN();
    require(check_wrench_limits(value,limits).channel == axis, "nonfinite component accepted");
  }
  require(!check_wrench_limits({},limits), "zero sample falsely rejected");
  std::cout << "Raw six-channel preflight/runtime limits and reported 6.04 failure diagnostics passed\n";
}
