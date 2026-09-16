#pragma once
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>

// Measurement only: never changes reference or motor commands.
namespace sonic_diagnostics {
struct PlannerSeam {
  unsigned samples = 0;
  unsigned invalid = 0;
  double max_position_gap_rad = 0.;
  double max_omitted_velocity_rad_s = 0.;
  static bool Finite(double x) {
    // Materialize bits: clang fast-math can otherwise fold known NaN checks.
    volatile std::uint64_t bits = std::bit_cast<std::uint64_t>(x);
    return (bits & UINT64_C(0x7ff0000000000000)) !=
           UINT64_C(0x7ff0000000000000);
  }
  void Observe(double old_q, double new_q, int frame, int start,
               int width, double frame_dt) {
    // Linear weight derivative is nonzero strictly inside the blend interval.
    // Endpoints have a derivative discontinuity; do not pretend it is defined.
    if (width <= 0 || !Finite(frame_dt) || frame_dt <= 0. ||
        !Finite(old_q) || !Finite(new_q)) { ++invalid; return; }
    if (frame <= start || frame-start >= width) return;
    const double gap = std::abs(new_q-old_q);
    const double omitted = gap/(width*frame_dt);
    if (!Finite(gap) || !Finite(omitted)) { ++invalid; return; }
    ++samples;
    max_position_gap_rad = std::max(max_position_gap_rad, gap);
    max_omitted_velocity_rad_s = std::max(max_omitted_velocity_rad_s, omitted);
  }
};
}
