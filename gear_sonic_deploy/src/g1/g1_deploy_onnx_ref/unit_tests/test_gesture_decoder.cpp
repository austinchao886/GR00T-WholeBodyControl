#include <gtest/gtest.h>
#include "gesture_decoder.hpp"
using namespace sonic_gesture;

static nlohmann::json Packet() {
  return {{"schema_version",1},{"session_id","s"},{"plan_id",std::string(64,'a')},
          {"sequence",0},{"origin_sim_tick",1000},{"physics_dt_s",.005},
          {"joint_indices",kRightArmIsaacLab},
          {"frames",std::vector<std::vector<double>>(256,std::vector<double>(16,0.))}};
}

TEST(GestureDecoder, AcceptsCompletePacketAndRejectsReplay) {
  GestureDecoder decoder("s",std::string(64,'a'));
  const auto now=std::chrono::steady_clock::now();
  auto packet=Packet(); packet["frames"][2][0]=.3;
  auto snapshot=decoder.Decode(packet.dump(),1004,now);
  EXPECT_DOUBLE_EQ(snapshot->frames[2].q[0],.3);
  EXPECT_THROW(decoder.Decode(packet.dump(),1004,now),std::invalid_argument);
  packet["sequence"]=1;packet["origin_sim_tick"]=1004;
  EXPECT_NO_THROW(decoder.Decode(packet.dump(),1005,now));
}

TEST(GestureDecoder, RejectsWrongIdentityShapeClockAndTypesWithoutConsumingSequence) {
  const auto now=std::chrono::steady_clock::now();
  for (int fault=0;fault<9;++fault) {
    GestureDecoder decoder("s",std::string(64,'a'));
    auto packet=Packet();
    switch(fault) {
      case 0:packet["session_id"]="other";break;
      case 1:packet["plan_id"]=std::string(64,'b');break;
      case 2:packet["frames"].erase(0);break;
      case 3:packet["frames"][255][15]="bad";break;
      case 4:packet["origin_sim_tick"]=1005;break;
      case 5:packet["origin_sim_tick"]=900;break;
      case 6:packet["sequence"]=0.5;break;
      case 7:packet["sequence"]=-1;break;
      case 8:packet["frames"][255][14]=2.;break;
    }
    EXPECT_ANY_THROW(decoder.Decode(packet.dump(),1004,now));
    EXPECT_NO_THROW(decoder.Decode(Packet().dump(),1004,now));
  }
}

TEST(GestureDecoder, RejectsOversizedAndMalformedPackets) {
  GestureDecoder decoder("s",std::string(64,'a'));
  const auto now=std::chrono::steady_clock::now();
  EXPECT_ANY_THROW(decoder.Decode(std::string(262145,' '),1000,now));
  EXPECT_ANY_THROW(decoder.Decode("{",1000,now));
  EXPECT_ANY_THROW(decoder.Decode("[[[[[[[[0]]]]]]]]",1000,now));
}
