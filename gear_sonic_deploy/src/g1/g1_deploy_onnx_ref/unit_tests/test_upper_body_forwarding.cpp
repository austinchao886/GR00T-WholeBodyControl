#include <gtest/gtest.h>
#include "input_interface/interface_manager.hpp"

class InterfaceManagerTestPeer {
 public:
  static void SetInput(InterfaceManager& manager, InputInterface* input) {
    manager.current_ = input;
  }
};

class UpperBodyInput : public InputInterface {
 public:
  std::shared_ptr<const sonic_gesture::GestureSnapshot> snapshot;
  std::shared_ptr<const sonic_gesture::GestureSnapshot> GetGestureSnapshot() const override {
    return snapshot;
  }
  void update() override {}
  void handle_input(MotionDataReader&, std::shared_ptr<const MotionSequence>&,
                    int&, OperatorState&, bool&, DataBuffer<HeadingState>&,
                    bool, PlannerState&, DataBuffer<MovementState>&,
                    std::mutex&, bool&) override {}
  void Set(double position, double velocity) {
    std::array<double, 17> q{}, dq{};
    for (int i = 0; i < 17; ++i) {
      q[i] = position + i * 0.01;
      dq[i] = velocity - i * 0.01;
    }
    upper_body_joint_positions_.SetData(q);
    upper_body_joint_velocities_.SetData(dq);
    has_upper_body_control_ = true;
  }
  void Clear() { has_upper_body_control_ = false; }
};

TEST(UpperBodyForwarding, ReflectsActiveInputUpdatesAndWithdrawal) {
  UpperBodyInput input;
  InterfaceManager manager("127.0.0.1", 59999, "pose", true, false);
  InterfaceManagerTestPeer::SetInput(manager, &input);
  InputInterface& proxy = manager;
  EXPECT_FALSE(proxy.HasUpperBodyControl());
  EXPECT_FALSE(proxy.GetUpperBodyJointPositions().first);
  input.Set(0.2, 0.3);
  EXPECT_TRUE(proxy.HasUpperBodyControl());
  for (int i = 0; i < 17; ++i) {
    EXPECT_DOUBLE_EQ(proxy.GetUpperBodyJointPositions().second[i], 0.2 + i * 0.01);
    EXPECT_DOUBLE_EQ(proxy.GetUpperBodyJointVelocities().second[i], 0.3 - i * 0.01);
  }
  input.Set(0.4, -0.2);
  EXPECT_DOUBLE_EQ(proxy.GetUpperBodyJointPositions().second[0], 0.4);
  EXPECT_DOUBLE_EQ(proxy.GetUpperBodyJointVelocities().second[0], -0.2);
  input.Clear();
  EXPECT_FALSE(proxy.HasUpperBodyControl());
  EXPECT_FALSE(proxy.GetUpperBodyJointPositions().first);
  EXPECT_FALSE(proxy.GetUpperBodyJointVelocities().first);
}

TEST(UpperBodyForwarding, DoesNotReusePreviousDelegateData) {
  UpperBodyInput first, second;
  InterfaceManager manager("127.0.0.1", 59999, "pose", true, false);
  first.Set(0.2, 0.3);
  InterfaceManagerTestPeer::SetInput(manager, &first);
  EXPECT_TRUE(manager.GetUpperBodyJointPositions().first);
  InterfaceManagerTestPeer::SetInput(manager, &second);
  EXPECT_FALSE(manager.HasUpperBodyControl());
  EXPECT_FALSE(manager.GetUpperBodyJointPositions().first);
  InterfaceManagerTestPeer::SetInput(manager, nullptr);
  EXPECT_FALSE(manager.GetUpperBodyJointPositions().first);
  EXPECT_FALSE(manager.GetUpperBodyJointVelocities().first);
}

TEST(UpperBodyForwarding, ForwardsPairedGestureSnapshotWithoutRetainingOldDelegate) {
  UpperBodyInput first, second;
  InterfaceManager manager("127.0.0.1", 59999, "pose", true, false);
  auto snapshot=std::make_shared<sonic_gesture::GestureSnapshot>();
  snapshot->origin_sim_tick=100;
  snapshot->frames[5].q[0]=.3;
  snapshot->frames[5].dq[0]=.2;
  first.snapshot=snapshot;
  InterfaceManagerTestPeer::SetInput(manager,&first);
  EXPECT_EQ(manager.GetGestureSnapshot(),snapshot);
  EXPECT_DOUBLE_EQ(manager.GetGestureSnapshot()->frames[5].dq[0],.2);
  InterfaceManagerTestPeer::SetInput(manager,&second);
  EXPECT_EQ(manager.GetGestureSnapshot(),nullptr);
  InterfaceManagerTestPeer::SetInput(manager,nullptr);
  EXPECT_EQ(manager.GetGestureSnapshot(),nullptr);
}
