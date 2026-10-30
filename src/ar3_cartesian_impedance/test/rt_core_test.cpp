#include "ar3_cartesian_impedance/rt_core.hpp"
#include <iostream>
#include <limits>
#include <thread>

using namespace ar3_cartesian_impedance;
namespace {
constexpr double dt = 0.001, pi = 3.14159265358979323846;
void require(bool ok, const char *message) { if(!ok) throw std::runtime_error(message); }
template<class F> void rejects(F f) {
  try { f(); } catch(const std::exception &) { return; }
  throw std::runtime_error("invalid RT operation was accepted");
}
Eigen::Vector3d velocity(const Six &a, const Six &b) {
  return Eigen::Vector3d(b[0]-a[0], b[1]-a[1], b[2]-a[2]) / dt;
}
Eigen::Vector3d angular_velocity(const Six &a, const Six &b) {
  Eigen::Quaterniond delta = orientation(b) * orientation(a).conjugate();
  if(delta.w() < 0) delta.coeffs() *= -1;
  const Eigen::AngleAxisd angle(delta);
  return angle.axis() * angle.angle() / dt;
}
void movement_in_six_axes_and_reversals() {
  for(std::size_t axis = 0; axis < 6; ++axis) for(double sign : {-1., 1.}) {
    RtPoseFollower follower;
    Six previous{}, target{}; follower.reset(previous);
    target[axis] = sign * (axis < 3 ? 0.01 : 0.05);
    Eigen::Vector3d v = Eigen::Vector3d::Zero(), w = Eigen::Vector3d::Zero();
    for(int tick = 0; tick < 3000; ++tick) {
      if(tick == 1500) target[axis] *= -1;
      const auto next = follower.step(target, 0.02, 0.15, 0.1, 0.5);
      const auto next_v = velocity(previous, next), next_w = angular_velocity(previous, next);
      require(next_v.norm() <= 0.020000001 && next_w.norm() <= 0.150000001, "RT output exceeded linear/angular speed");
      require((next_v-v).norm() <= 0.000100001 && (next_w-w).norm() <= 0.00050001,
        "RT direction reversal violated acceleration limits");
      if(tick == 1400) require(sign * next[axis] > 0.8 * std::abs(target[axis]), "positive/negative axis motion did not approach target");
      previous=next; v=next_v; w=next_w;
    }
    require(sign * previous[axis] < 0, "RT target reversal failed to move back");
  }
}
void close_target_brakes_without_velocity_reset() {
  for(std::size_t axis = 0; axis < 6; ++axis) {
    RtPoseFollower follower; Six pose{}, prior{}, goal{}; follower.reset(pose); goal[axis] = 0.3;
    for(int tick=0; tick<100; ++tick) { prior=pose; pose=follower.step(goal, 0.02, 0.15, 0.1, 0.5); }
    const auto v = velocity(prior,pose), w = angular_velocity(prior,pose);
    const auto next = follower.step(pose,0.02,0.15,0.1,0.5);  // target becomes exactly current output
    require((velocity(pose,next)-v).norm() <= 0.000100001, "nearby target snapped translational velocity to zero");
    require((angular_velocity(pose,next)-w).norm() <= 0.00050001, "nearby target snapped angular velocity to zero");
    require(next[axis] > pose[axis], "moving output must keep braking over a finite stopping distance");
  }
  RtPoseFollower follower; Six start{0,0,0,0,0,pi-0.01}, target{0,0,0,0,0,-pi+0.01}; follower.reset(start);
  auto pose=start;
  for(int tick=0;tick<1000;++tick) pose=follower.step(target,0.02,0.15,0.1,0.5);
  require(orientation(pose).angularDistance(orientation(target)) < 0.001, "Euler wrap followed the long rotation");
}
void base_world_force_contract() {
  const Six side{0.4,0.1,0.7,pi/2,0,0};
  const auto w = wrench_in_base({1,2,3,4,5,6},side);
  const Six expected{1,-3,2,4,-6,5};
  for(int i=0;i<6;++i) require(std::abs(w[i]-expected[i]) < 1e-10, "base->world wrench axis mapping wrong");
  const auto k=stiffness_in_world({100,200,300,10,20,30},side);
  const Six expected_k{100,300,200,10,30,20};
  for(int i=0;i<6;++i) require(std::abs(k[i]-expected_k[i]) < 1e-10, "side-mount stiffness permutation wrong");
  rejects([&] { stiffness_in_world({100,200,300,10,20,30},{0,0,0,0.4,0.5,0.6}); });
  const Six isotropic{500,500,500,50,50,50};
  const auto rotated=stiffness_in_world(isotropic,{1,2,3,0.4,0.5,0.6});
  for(int i=0;i<6;++i) require(std::abs(rotated[i]-isotropic[i]) < 1e-10, "isotropic K changed under base mounting rotation");
  const Six pose{0.2,0.4,-0.3,0.3,-0.5,1.2};
  require(pose_transform(matrix_pose(pose_matrix(pose))).matrix().isApprox(pose_transform(pose).matrix(),1e-10), "row-major TCP matrix contract changed");
  auto invalid=pose_matrix(pose); invalid[15]=0; rejects([&] { matrix_pose(invalid); });
  invalid=pose_matrix(pose); invalid[0]=std::numeric_limits<double>::quiet_NaN(); rejects([&] { matrix_pose(invalid); });
}
void session_and_freshness_guards() {
  Command config{}; config.stiffness={500,500,500,50,50,50};
  RtCommand command{config,1000000000,7};
  require(rt_session_gate(command,config,7,1000000000,1001000000,.5,.01,false)==RtExit::none,"fresh valid command rejected");
  require(rt_session_gate(command,config,8,1000000000,1001000000,.5,.01,false)==RtExit::command_timeout,"old-session target replayed after re-enable");
  require(rt_session_gate(command,config,7,1600000000,1600000000,.5,.01,false)==RtExit::command_timeout,"fresh feedback renewed an expired command");
  command.received_ns=1600000000;
  require(rt_session_gate(command,config,7,1000000000,1600000000,.5,.01,false)==RtExit::state_timeout,"fresh command renewed stale measured state");
  require(rt_session_gate(command,config,7,1600000000,1600000000,.5,.01,true)==RtExit::requested,"stop request did not dominate latest goal");
  for(int field=0;field<4;++field) {
    command.value=config;
    auto *values = field==0 ? &command.value.stiffness : field==1 ? &command.value.wrench : field==2 ? &command.value.damping : &command.value.mass;
    (*values)[0]+=1;
    require(rt_session_gate(command,config,7,1600000000,1600000000,.5,.01,false)==RtExit::parameter_change,"session parameter change was accepted while moving");
  }
  unsigned pending=42;
  const auto drained=drain_rt_packets([&](bool) { if(!pending) return false; --pending; return true; },4096);
  require(drained==42 && !pending,"startup RPC packet backlog was not drained before callback handoff");
  pending=42; rejects([&] { drain_rt_packets([&](bool) { --pending; return true; },8); });
  require(pending==34,"active callback backlog drain was not bounded to eight packets");
  require(drain_rt_packets([](bool) { return false; },8)==0,"empty state queue fabricated a fresh measurement");
}
void latest_mailbox_stress_and_raw_channels() {
  LatestMailbox<RtSnapshot> mailbox;
  RtSnapshot sample{}; sample.measured.wrench={1,2,3,4,5,6}; sample.sequence=1;
  mailbox.publish(sample); sample.sequence=2; mailbox.publish(sample);
  RtSnapshot got{}; require(mailbox.read(got) && got.sequence==2, "mailbox replayed an obsolete queued command");
  require(got.measured.wrench==sample.measured.wrench,"RT six external channels were reordered/scaled/shifted");
  require(!mailbox.read(got),"reading cached mailbox value fabricated a new packet");
  std::atomic<bool> done{false};
  std::thread producer([&] {
    for(std::uint64_t sequence=3;sequence<=100000;++sequence) {
      RtSnapshot value{}; value.sequence=sequence;
      for(std::size_t i=0;i<6;++i) value.measured.wrench[i]=static_cast<double>(sequence*10+i);
      mailbox.publish(value);
    }
    done.store(true);
  });
  std::uint64_t last=2;
  auto inspect=[&] {
    require(got.sequence>=last,"mailbox sequence moved backwards"); last=got.sequence;
    for(std::size_t i=0;i<6;++i) require(got.measured.wrench[i]==static_cast<double>(got.sequence*10+i),"mailbox returned a torn cross-thread snapshot");
  };
  while(!done.load()) if(mailbox.read(got)) inspect();
  producer.join(); if(mailbox.read(got)) inspect();
  require(last==100000,"mailbox lost the final latest publication");
}
void stop_does_not_require_duplicate_stop_move() {
  RtStopProgress prepared; prepared.start_attempted();
  const bool mechanically_idle = true;  // startMove prepared, no first command sent yet
  int prepared_stop_calls = 0;
  if(prepared.needs_stop_rpc()) { ++prepared_stop_calls; prepared.stop_acknowledged(); }
  require(mechanically_idle && prepared_stop_calls == 1, "idle robot did not clear its prepared RT start state");
  RtStopProgress stop; stop.start_attempted();
  require(stop.needs_stop_rpc(),"uncertain start outcome lost stop obligation");
  stop.loop_returned_normally(true);  // setFinished already stopped the controller
  int stop_calls=0;
  if(stop.needs_stop_rpc()) { ++stop_calls; throw std::runtime_error("SDK rejects duplicate stopMove"); }
  require(stop_calls==0,"normal setFinished exit depended on undocumented stopMove idempotence");
  stop.start_attempted(); stop.loop_returned_normally(false);
  require(stop.needs_stop_rpc(),"loop failure cleared required explicit stop");
  // Failed stop RPC must retain its obligation; only success or confirmed idle clears it.
  require(stop.needs_stop_rpc(),"failed stop may not permit re-enable");
  stop.stop_acknowledged(); require(!stop.needs_stop_rpc(),"successful stop acknowledgement not retained");
  // The client's 0.2 s stop polls previously caused an unbounded sequence of
  // 3 s waits and overwrote the failure status before ROS could publish it.
  require(stop.begin_confirmation(), "first stop confirmation was suppressed");
  for(int poll=0;poll<100;++poll)
    require(!stop.begin_confirmation(), "stop poll restarted a latched failed/finished confirmation");
  stop.reset_for_session();
  require(stop.begin_confirmation(), "new explicitly armed session cannot confirm its own stop");
  // Enable service can run after worker committed stopped, but before the ROS
  // timer consumed that status. It must neither disarm nor publish the old stop
  // into the newly armed Python clutch session.
  require(obsolete_session_status(7,8,true),"queued old stopped status can cancel a newly armed session");
  require(!obsolete_session_status(8,8,true),"current-session stop status was suppressed");
  require(!obsolete_session_status(7,8,false),"unarmed client lost stop status");
}
void stop_exits_rt_mode_before_confirming_idle() {
  struct FakeController {
    bool moving{}, rci{true}, receiving{true}, connected{true}, ready{true}, confirmed{};
    bool fail_idle{}, fail_restore{};
    int stop_calls{}, motion_starts{}, frames{1};
    int operation_state() const { return rci ? 2 : moving ? 9 : 0; }
    void stop_motion() {
      require(moving, "normal setFinished was followed by a duplicate stopMove");
      ++stop_calls; moving = false;
    }
    void suspend_monitor() {
      require(!moving, "RT ports closed before issuing the required motion stop");
      ready = receiving = connected = false;
    }
    void leave_mode() {
      require(!receiving && !connected, "invalid RT fields remained published during Idle transition");
      if(fail_idle) throw std::runtime_error("controller rejected eci.disable");
      rci = false;
    }
    void confirm_idle() {
      require(operation_state() == 0, "RCI main state was mistaken for confirmed physical idle");
      confirmed = true;
    }
    void resume_monitor() {
      require(confirmed, "RT monitoring resumed before controller idle was confirmed");
      rci = true; connected = true;
      if(fail_restore) throw std::runtime_error("state subscription failed");
      receiving = true; ++frames; ready = true;
    }
    void finish(bool explicit_stop) {
      finish_rt_session(explicit_stop, [this] { stop_motion(); }, [this] { suspend_monitor(); },
        [this] { leave_mode(); }, [this] { confirm_idle(); });
    }
  };
  FakeController normal;  // setFinished ended motion, but sys.state is still rci/idle
  require(normal.operation_state() == 2, "fake does not reproduce the RCI main-state contract");
  rejects([&] { normal.confirm_idle(); });
  normal.finish(false);
  require(normal.confirmed && normal.stop_calls == 0 && !normal.ready && !normal.receiving,
    "normal stop failed to exit RT with feedback isolated");
  normal.resume_monitor();
  require(normal.ready && normal.frames == 2 && normal.motion_starts == 0 && !normal.moving,
    "monitor restoration reused stale state or started motion");
  FakeController exceptional; exceptional.moving = true;
  exceptional.finish(true);
  require(exceptional.stop_calls == 1 && exceptional.confirmed, "exceptional loop did not stop before leaving RT");
  FakeController refused; refused.fail_idle = true;
  rejects([&] { refused.finish(false); });
  require(!refused.confirmed && !refused.ready && !refused.receiving,
    "failed Idle transition claimed a stop or published invalid state");
  FakeController restore_failed; restore_failed.finish(false); restore_failed.fail_restore = true;
  rejects([&] { restore_failed.resume_monitor(); });
  require(restore_failed.confirmed && !restore_failed.ready && !restore_failed.moving && restore_failed.motion_starts == 0,
    "monitor failure lost confirmed stop or allowed motion");
  // Shutdown performs the same stop sequence without reopening RT monitoring.
  normal.finish(false);
  require(!normal.rci && !normal.receiving && normal.stop_calls == 0, "shutdown left RT mode enabled or repeated stopMove");
}
}
int main() {
  movement_in_six_axes_and_reversals(); close_target_brakes_without_velocity_reset(); base_world_force_contract();
  session_and_freshness_guards(); latest_mailbox_stress_and_raw_channels(); stop_does_not_require_duplicate_stop_move();
  stop_exits_rt_mode_before_confirming_idle();
  std::cout << "RT six-axis/reversal speed and acceleration, world force frame, session/watchdogs, startup backlog, mailbox and stop regressions passed\n";
}
