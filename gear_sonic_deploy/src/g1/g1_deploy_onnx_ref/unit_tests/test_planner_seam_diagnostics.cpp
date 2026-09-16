#include "planner_seam_diagnostics.hpp"
#include <limits>
#include <iostream>
#define REQUIRE(x) do { if (!(x)) { std::cerr << #x << '\n'; return 1; } } while(false)
static int RunSeamChecks() {
  sonic_diagnostics::PlannerSeam s;
  s.Observe(0., .16, 4, 0, 8, .02);
  REQUIRE(s.samples == 1);
  REQUIRE(std::abs(s.max_omitted_velocity_rad_s-1.) < 1e-12);
  // Constant old/new trajectories: blended q moves at 1 rad/s, independently
  // blended zero velocities stay zero. This is a kinematic, not balance test.
  const double q_before = .16*(3./8.);
  const double q_after = .16*(5./8.);
  REQUIRE(std::abs((q_after-q_before)/.04-s.max_omitted_velocity_rad_s) < 1e-12);
  for (int f : {-1,0,8,9}) s.Observe(0., 100., f, 0, 8, .02);
  REQUIRE(s.samples == 1);
  s.Observe(1., 1., 2, 0, 8, .02);
  REQUIRE(s.samples == 2 && s.max_position_gap_rad == .16);
  s.Observe(std::numeric_limits<double>::quiet_NaN(), 0., 1, 0, 8, .02);
  s.Observe(0., 0., 1, 0, 8, 0.);
  REQUIRE(s.invalid == 2);
  std::cout << "planner seam diagnostic checks passed\n";
  return 0;
}
#ifdef SONIC_STANDALONE_SEAM_TEST
int main() { return RunSeamChecks(); }
#else
#include <gtest/gtest.h>
TEST(PlannerSeamDiagnostics, MeasuresMissingDerivativeAndRejectsInvalidData) {
  EXPECT_EQ(RunSeamChecks(), 0);
}
#endif
