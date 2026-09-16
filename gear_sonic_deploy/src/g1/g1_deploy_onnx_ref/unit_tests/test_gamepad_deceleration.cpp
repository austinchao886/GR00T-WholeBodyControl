#include <gtest/gtest.h>
#include "input_interface/gamepad.hpp"

struct DecelerationFixture : testing::Test {
  unitree::common::Gamepad pad;
  MotionDataReader reader;
  std::shared_ptr<const MotionSequence> motion;
  int frame=0;
  OperatorState state;
  bool heading=false, temperature=false;
  DataBuffer<HeadingState> headings;
  PlannerState planner;
  DataBuffer<MovementState> commands;
  std::mutex mutex;
  void SetUp() override {
    setenv("SONIC_EXPERIMENTAL_ZERO_SPEED_DECEL","1",1);
    pad.gamepad_data={};pad.RequestPlannerMode(true);pad.update();
    pad.smooth=1.;pad.gamepad_data.RF_RX.btn.components.F2=1;
    pad.gamepad_data.RF_RX.ly=.25;
    planner.enabled=true;planner.initialized=true;
    for(int i=0;i<100;++i)Step();
  }
  void TearDown() override {unsetenv("SONIC_EXPERIMENTAL_ZERO_SPEED_DECEL");}
  MovementState Step() {
    pad.update();
    pad.handle_input(reader,motion,frame,state,heading,headings,true,planner,
                     commands,mutex,temperature);
    return *commands.GetDataWithTime().data;
  }
};

TEST_F(DecelerationFixture, CenterDoesNotJumpFromPointTwoToIdle) {
  const auto moving=Step();
  ASSERT_EQ(moving.locomotion_mode,static_cast<int>(LocomotionMode::SLOW_WALK));
  pad.gamepad_data.RF_RX.ly=0;
  auto command=Step();
  EXPECT_EQ(command.locomotion_mode,static_cast<int>(LocomotionMode::SLOW_WALK));
  EXPECT_GT(command.movement_speed,0.);
  EXPECT_LT(command.movement_speed,.2);
}

TEST_F(DecelerationFixture, DeadmanStillStopsImmediately) {
  pad.gamepad_data.RF_RX.btn.components.F2=0;
  const auto command=Step();
  EXPECT_EQ(command.locomotion_mode,static_cast<int>(LocomotionMode::IDLE));
  EXPECT_EQ(command.movement_direction,(std::array<double,3>{0.,0.,0.}));
}

TEST_F(DecelerationFixture, UnqualifiedExperimentIsDisabledByDefault) {
  unsetenv("SONIC_EXPERIMENTAL_ZERO_SPEED_DECEL");
  pad.gamepad_data.RF_RX.ly=0;
  EXPECT_EQ(Step().locomotion_mode,static_cast<int>(LocomotionMode::IDLE));
}

TEST_F(DecelerationFixture, CenterReachesZeroBeforeIdleWithBoundedDecrements) {
  double previous=Step().movement_speed;
  pad.gamepad_data.RF_RX.ly=0;
  bool idle=false;
  for(int i=0;i<40;++i) {
    const auto command=Step();
    if(command.locomotion_mode==static_cast<int>(LocomotionMode::IDLE)) {
      EXPECT_LE(previous,.01500001);idle=true;break;
    }
    EXPECT_EQ(command.locomotion_mode,static_cast<int>(LocomotionMode::SLOW_WALK));
    EXPECT_GT(command.movement_speed,0.);
    EXPECT_LE(command.movement_speed,previous);
    EXPECT_LE(previous-command.movement_speed,.01500001);
    previous=command.movement_speed;
  }
  EXPECT_TRUE(idle);
}
