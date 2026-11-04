#include "ar3_cartesian_impedance/wrench_guard.hpp"
#include <iostream>
#include <limits>

using namespace ar3_cartesian_impedance;
void require(bool condition, const char *message) { if(!condition) throw std::runtime_error(message); }
int main() {
  const Six limits{40,40,40,6,10,20};
  const Six previous_limits{40,40,40,6,6,20};
  // Preserve the complete signed feedback from the two reported failures.
  for(const Six reported_channel5 : {
      Six{-18.929,5.017,2.326,-1.819,-6.019,-4.755},
      Six{-24.932,1.789,2.104,-1.420,-7.709,-7.451}}) {
    const auto old_trip = check_wrench_limits(reported_channel5,previous_limits);
    require(old_trip && old_trip.channel == 4, "reported sample must reproduce the old channel 5 trip");
    require(!check_wrench_limits(reported_channel5,limits),
      "reported -6.019/-7.709 sample must fit the updated channel 5 limit");
  }
  const Six reported_channel6{0.583,12.283,12.725,-3.655,-3.916,7.154};
  require(!check_wrench_limits(reported_channel6,limits),
    "reported 7.154 sample must fit the updated channel 6 limit");
  const Six reported_startup{31.626,5.411,5.449,-3.695,4.747,16.174};
  require(!check_wrench_limits(reported_startup,limits),
    "reported 16.174 startup sample must fit the updated channel 6 limit");
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
  // Includes channel 5 at +/-10 and +/-10.001, and unchanged channel 6 at +/-20.
  for(std::size_t axis=0; axis<6; ++axis) for(double sign : {-1.,1.}) {
    Six value{}; value[axis] = sign*limits[axis];
    require(!check_wrench_limits(value,limits), "signed equal-to-limit component spuriously trips");
    value[axis] = sign*(limits[axis] + 0.001);
    const auto over_limit = check_wrench_limits(value,limits);
    require(over_limit && over_limit.channel == axis, "signed over-limit component missed");
    value[axis] = std::numeric_limits<double>::quiet_NaN();
    const auto nonfinite = check_wrench_limits(value,limits);
    require(nonfinite && nonfinite.channel == axis, "nonfinite component accepted");
  }
  require(!check_wrench_limits({},limits), "zero sample falsely rejected");
  std::cout << "Raw signed limits, -6.019/-7.709 channel 5 samples, unchanged channel 6, and failure diagnostics passed\n";
}
