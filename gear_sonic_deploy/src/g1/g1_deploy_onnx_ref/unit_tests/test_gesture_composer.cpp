#include <gtest/gtest.h>
#include <limits>
#include "gesture_composer.hpp"

using namespace sonic_gesture;

TEST(GestureComposer, PreservesPlannerExceptRightArmAndIncludesWeightDerivative) {
  JointReference base; base.q.fill(.1); base.dq.fill(.2);
  ArmSample arm; arm.q.fill(.5); arm.dq.fill(-.1);
  arm.weight=.5; arm.weight_rate=.2;
  auto out=Compose(base,arm);
  for (int j=0;j<29;++j) {
    bool selected=false;
    for (auto index:kRightArmIsaacLab) selected |= j==index;
    EXPECT_NEAR(out.q[j],selected?.3:.1,1e-14);
    EXPECT_NEAR(out.dq[j],selected?.13:.2,1e-14);
  }
}

TEST(GestureComposer, DistinctFutureFramesAndBaseUnchanged) {
  std::array<JointReference,10> base{};
  std::array<ArmSample,10> arm{};
  for (int i=0;i<10;++i) {
    base[i].q.fill(i*.01); arm[i].q.fill(i*.02);
    arm[i].weight=1;
  }
  const auto out=ComposeWindow(base,arm);
  for (int i=0;i<10;++i) {
    EXPECT_DOUBLE_EQ(out[i].q[12],i*.02);
    EXPECT_DOUBLE_EQ(out[i].q[2],i*.01); // waist remains planner-owned
    EXPECT_DOUBLE_EQ(base[i].q[12],i*.01);
  }
}

TEST(GestureComposer, RejectsInvalidDataAndOverflow) {
  JointReference base; ArmSample arm;
  arm.weight=.5;
  arm.q[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Compose(base,arm),std::invalid_argument);
  arm.q[0]=0; arm.weight=2;
  EXPECT_THROW(Compose(base,arm),std::invalid_argument);
  arm.weight=0;arm.weight_rate=.1;
  EXPECT_THROW(Compose(base,arm),std::invalid_argument);
  arm.weight=.5;arm.weight_rate=1;
  arm.q[0]=std::numeric_limits<double>::max();
  base.q[12]=-std::numeric_limits<double>::max();
  EXPECT_THROW(Compose(base,arm),std::invalid_argument);
}

TEST(GestureComposer, AnalyticEnvelopeVelocityMatchesFiniteDifference) {
  for(double t:{.1,.3,.7,1.3,1.8}) {
    const double h=1e-6;
    auto [w,dw]=TransitionWeight(t,2);
    const double numerical=(TransitionWeight(t+h,2).first-TransitionWeight(t-h,2).first)/(2*h);
    EXPECT_NEAR(dw,numerical,1e-8);
  }
  EXPECT_EQ(TransitionWeight(0,2),std::make_pair(0.,0.));
  EXPECT_EQ(TransitionWeight(2,2),std::make_pair(1.,0.));
}

TEST(GestureComposer, BufferedTicksPreserveHorizonWithoutRequiringArrivalOnExactTick) {
  GestureSnapshot snapshot;
  snapshot.origin_sim_tick=1000;
  snapshot.received_at=std::chrono::steady_clock::now();
  for (int i=0;i<256;++i) snapshot.frames[i].q[0]=i;
  EXPECT_TRUE(SnapshotCovers(snapshot,1005,snapshot.received_at+std::chrono::milliseconds(25)));
  EXPECT_DOUBLE_EQ(SampleAt(snapshot,1005,0).q[0],5);
  EXPECT_DOUBLE_EQ(SampleAt(snapshot,1005,45).q[0],185);
  EXPECT_DOUBLE_EQ(SampleAt(snapshot,1040,45).q[0],220);
  EXPECT_FALSE(SnapshotCovers(snapshot,1041,snapshot.received_at));
  EXPECT_FALSE(SnapshotCovers(snapshot,999,snapshot.received_at));
  EXPECT_FALSE(SnapshotCovers(snapshot,1000,snapshot.received_at+std::chrono::milliseconds(201)));
  EXPECT_FALSE(SnapshotCovers(snapshot,1000,snapshot.received_at-std::chrono::milliseconds(1)));
  EXPECT_THROW(SampleAt(snapshot,999,0),std::invalid_argument);
  EXPECT_THROW(SampleAt(snapshot,1000,46),std::invalid_argument);
}
