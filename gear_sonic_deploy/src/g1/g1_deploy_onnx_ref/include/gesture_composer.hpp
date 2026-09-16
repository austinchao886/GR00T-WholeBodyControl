#pragma once

#include <array>
#include <bit>
#include <cstdint>
#include <limits>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

// Reference-space only. No DDS, motor commands, or input ownership changes.
namespace sonic_gesture {
// SONIC builds with -ffast-math, which permits removing std::isfinite checks.
// Inspect IEEE-754 exponent bits so validation survives those build flags.
inline bool IsFinite(double value) {
  static_assert(sizeof(double)==sizeof(std::uint64_t) && std::numeric_limits<double>::is_iec559);
  return (std::bit_cast<std::uint64_t>(value) & 0x7ff0000000000000ULL) != 0x7ff0000000000000ULL;
}
inline constexpr std::array<int, 7> kRightArmIsaacLab{12,16,20,22,24,26,28};

struct JointReference {
  std::array<double, 29> q{};
  std::array<double, 29> dq{};
};

struct ArmSample {
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  double weight = 0;
  double weight_rate = 0;
};

// Immutable physics-tick reference buffer: 256 samples at 200 Hz cover the
// installed 0.9-second horizon plus bounded transport delay. Transport is not
// implemented here; its receiver must verify session and approved plan identity.
struct GestureSnapshot {
  std::uint64_t execution_id = 0;
  std::array<ArmSample, 256> frames{};
  std::uint32_t origin_sim_tick = 0;
  std::chrono::steady_clock::time_point received_at{};
};

inline bool SnapshotCovers(const GestureSnapshot& snapshot, std::uint32_t tick,
                           std::chrono::steady_clock::time_point now) {
  const auto age=now-snapshot.received_at;
  // Tick wrap/reset requires a new session and fresh buffer, never reinterpret.
  return tick>=snapshot.origin_sim_tick && tick-snapshot.origin_sim_tick<=40 &&
         age>=std::chrono::steady_clock::duration::zero() &&
         age<=std::chrono::milliseconds(200);
}

inline const ArmSample& SampleAt(const GestureSnapshot& snapshot,
                                 std::uint32_t tick, int future_policy_frames) {
  if (tick<snapshot.origin_sim_tick || future_policy_frames<0 || future_policy_frames>45)
    throw std::invalid_argument("gesture tick/horizon mismatch");
  const auto index=std::uint64_t(tick-snapshot.origin_sim_tick)+4*future_policy_frames;
  if (index>=snapshot.frames.size()) throw std::invalid_argument("gesture buffer exhausted");
  return snapshot.frames[index];
}

inline std::pair<double, double> TransitionWeight(double elapsed, double duration) {
  if (!IsFinite(elapsed) || !IsFinite(duration) || duration <= 0)
    throw std::invalid_argument("invalid gesture envelope time");
  if (elapsed <= 0) return {0,0};
  if (elapsed >= duration) return {1,0};
  const double u = elapsed/duration;
  const double v=std::min(u,1-u);
  const double tail=v*v*v*(10+v*(-15+6*v));
  const double weight=u<=.5 ? tail : 1-tail;
  return {weight, (weight==0 || weight==1) ? 0 : 30*u*u*(1-u)*(1-u)/duration};
}

inline JointReference Compose(const JointReference& base, const ArmSample& arm) {
  if (!IsFinite(arm.weight) || !IsFinite(arm.weight_rate) ||
      arm.weight < 0 || arm.weight > 1 ||
      ((arm.weight == 0 || arm.weight == 1) && arm.weight_rate != 0))
    throw std::invalid_argument("invalid gesture envelope");
  for (int i=0; i<29; ++i)
    if (!IsFinite(base.q[i]) || !IsFinite(base.dq[i]))
      throw std::invalid_argument("nonfinite planner reference");
  for (int i=0; i<7; ++i)
    if (!IsFinite(arm.q[i]) || !IsFinite(arm.dq[i]))
      throw std::invalid_argument("nonfinite gesture reference");
  JointReference output = base;
  for (int i=0; i<7; ++i) {
    const int joint = kRightArmIsaacLab[i];
    const double delta = arm.q[i]-base.q[joint];
    output.q[joint] = base.q[joint]+arm.weight*delta;
    output.dq[joint] = (1-arm.weight)*base.dq[joint] +
                      arm.weight*arm.dq[i]+arm.weight_rate*delta;
    if (!IsFinite(output.q[joint]) || !IsFinite(output.dq[joint]))
      throw std::invalid_argument("gesture composition overflow");
  }
  return output;
}

// An observation horizon must contain distinct samples, not one repeated pose.
// All samples belong to one immutable snapshot selected by the caller. The
// caller remains responsible for timestamps, session, approval, and freshness.
template <std::size_t N>
std::array<JointReference, N> ComposeWindow(
    const std::array<JointReference,N>& planner,
    const std::array<ArmSample,N>& gesture) {
  std::array<JointReference,N> result;
  for (std::size_t i=0; i<N; ++i) result[i] = Compose(planner[i],gesture[i]);
  return result;
}
}  // namespace sonic_gesture
