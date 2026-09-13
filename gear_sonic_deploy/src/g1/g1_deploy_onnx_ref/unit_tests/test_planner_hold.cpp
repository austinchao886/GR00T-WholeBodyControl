#include <gtest/gtest.h>
#include "input_interface/gamepad.hpp"

TEST(PlannerHold, RuntimeYawIntegratesAtInputRateNotPolicyRate) {
  unitree::common::Gamepad pad;
  pad.gamepad_data = {};
  pad.RequestPlannerMode(true);
  pad.update();
  pad.smooth = 1.0;
  pad.gamepad_data.RF_RX.rx = 1;
  pad.gamepad_data.RF_RX.btn.components.F2 = 1;
  for (int i = 0; i < 100; ++i) pad.update();
  // One second of 100-Hz input: acceleration-limited yaw stays below 0.3rad.
  EXPECT_GT(std::abs(pad.planner_facing_angle), 0.20);
  EXPECT_LE(std::abs(pad.planner_facing_angle), 0.30);
}

TEST(PlannerHold, KeepsPlannerAndHeadingButSuppressesFreshMovement) {
  unitree::common::Gamepad pad;
  pad.gamepad_data = {};
  pad.RequestPlannerMode(true);
  pad.update();
  pad.planner_facing_angle = 0.7;
  pad.RequestPlannerHold();
  pad.gamepad_data.RF_RX.lx = 1;
  pad.gamepad_data.RF_RX.ly = 1;
  pad.gamepad_data.RF_RX.rx = 1;
  pad.gamepad_data.RF_RX.btn.components.F2 = 1;
  pad.gamepad_data.RF_RX.btn.components.F1 = 1;
  pad.gamepad_data.RF_RX.btn.components.Y = 1;
  for (int i = 0; i < 10; ++i) pad.update();
  EXPECT_TRUE(pad.use_planner);
  EXPECT_DOUBLE_EQ(pad.planner_facing_angle, 0.7);
  EXPECT_EQ(pad.planner_use_movement_mode, static_cast<int>(LocomotionMode::IDLE));
  EXPECT_FLOAT_EQ(pad.lx, 0);
  EXPECT_FLOAT_EQ(pad.rx, 0);
  EXPECT_FALSE(pad.reinitialize);
}

TEST(PlannerHold, PreservesEmergencyStopAndAllowsExplicitModeExit) {
  unitree::common::Gamepad pad;
  pad.gamepad_data = {};
  pad.RequestPlannerMode(true);
  pad.update();
  pad.RequestPlannerHold();
  pad.gamepad_data.RF_RX.btn.components.select = 1;
  pad.gamepad_data.RF_RX.btn.components.B = 1;
  pad.update();
  EXPECT_TRUE(pad.stop_control);
  EXPECT_TRUE(pad.planner_emergency_stop);
  pad.gamepad_data = {};
  pad.RequestPlannerMode(false);
  pad.update();
  EXPECT_FALSE(pad.use_planner);
}
