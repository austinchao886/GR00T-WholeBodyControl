#include "planner_blend.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#define CHECK(x) do { if (!(x)) { std::cerr << #x << '\n'; return 1; } } while(false)
static int RunBlendChecks() {
  using namespace sonic_reference;
  for (double u : {-1.,0.,1.,2.}) {
    const auto w=QuinticWeight(u,.16);
    CHECK(w.value==(u<=0.?0.:1.) && w.rate==0.);
  }
  CHECK(std::abs(QuinticWeight(.5,.16).value-.5)<1e-12);
  for(double t : {.001,.03,.08,.12,.159}) {
    // Both input trajectories are moving; derivative must include their
    // derivatives AND the time-varying blend weight term.
    auto sample=[](double s) {
      return BlendJoint(.1+2.*s,2.,-.2-.5*s,-.5,QuinticWeight(s/.16,.16));
    };
    constexpr double h=1e-7;
    CHECK(std::abs((sample(t+h).q-sample(t-h).q)/(2.*h)-sample(t).dq)<1e-7);
  }
  auto start=BlendJoint(.1,2.,-.2,-.5,QuinticWeight(0.,.16));
  auto end=BlendJoint(.1,2.,-.2,-.5,QuinticWeight(1.,.16));
  CHECK(start.q==.1 && start.dq==2. && end.q==-.2 && end.dq==-.5);
  auto same=BlendJoint(.3,.2,.3,.2,QuinticWeight(.4,.16));
  CHECK(std::abs(same.q-.3)<1e-12 && std::abs(same.dq-.2)<1e-12);
  for(double dt : {0.,-1.,std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected=false;try {QuinticWeight(.5,dt);} catch(const std::invalid_argument&) {rejected=true;}
    CHECK(rejected);
  }
  std::cout << "planner blend derivative checks passed\n";return 0;
}
#ifdef SONIC_STANDALONE_BLEND_TEST
int main() {return RunBlendChecks();}
#else
#include <gtest/gtest.h>
TEST(PlannerBlend, DerivativeEndpointsAndInvalidIntervals) {EXPECT_EQ(RunBlendChecks(),0);}
#endif
