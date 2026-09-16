#include <gtest/gtest.h>
#include "gesture_receiver.hpp"
using namespace sonic_gesture;

struct LocalPair {
  int fd[2];
  LocalPair(){if(socketpair(AF_UNIX,SOCK_SEQPACKET,0,fd))throw std::runtime_error("socketpair");}
  ~LocalPair(){for(auto f:fd)if(f>=0)close(f);}
};

static std::string Packet() {
  nlohmann::json packet={{"schema_version",1},{"session_id","s"},
    {"plan_id",std::string(64,'a')},{"sequence",0},{"origin_sim_tick",1000},
    {"physics_dt_s",.005},{"joint_indices",kRightArmIsaacLab},
    {"frames",std::vector<std::vector<double>>(256,std::vector<double>(16,0.))}};
  return packet.dump();
}

TEST(GestureReceiver, DeliversImmutableSnapshotAndLatchesReplayFailure) {
  LocalPair pair;
  GestureReceiver receiver(pair.fd[1],"s",std::string(64,'a'),1004);
  auto packet=Packet();
  ASSERT_EQ(send(pair.fd[0],packet.data(),packet.size(),MSG_NOSIGNAL),packet.size());
  std::shared_ptr<const GestureSnapshot> snapshot;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!snapshot && std::chrono::steady_clock::now()<deadline) {
    snapshot=receiver.Snapshot();std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(snapshot); EXPECT_EQ(snapshot->origin_sim_tick,1000);
  ASSERT_EQ(send(pair.fd[0],packet.data(),packet.size(),MSG_NOSIGNAL),packet.size());
  bool failed=false;
  while(!failed && std::chrono::steady_clock::now()<deadline) {
    try {receiver.Snapshot();}catch(const std::runtime_error&){failed=true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_TRUE(failed); EXPECT_THROW(receiver.Snapshot(),std::runtime_error);
  EXPECT_EQ(snapshot->origin_sim_tick,1000); // previous snapshot remains immutable
}

TEST(GestureReceiver, DisconnectBecomesExplicitFailure) {
  LocalPair pair;
  GestureReceiver receiver(pair.fd[1],"s",std::string(64,'a'),1000);
  close(pair.fd[0]);pair.fd[0]=-1;
  bool failed=false;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!failed && std::chrono::steady_clock::now()<deadline) {
    try {receiver.Snapshot();}catch(const std::runtime_error&){failed=true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_TRUE(failed);
}

TEST(GestureReceiver, RejectsNonSocketDescriptor) {
  EXPECT_THROW(GestureReceiver(-1,"s",std::string(64,'a'),1000),std::invalid_argument);
}

TEST(GestureReceiver, ReportsNativeTicksWithoutBlockingControlThread) {
  LocalPair pair;
  GestureReceiver receiver(pair.fd[1],"s",1000);
  receiver.SetTick(1004);
  std::array<unsigned char,20> bytes{};
  EXPECT_EQ(recv(pair.fd[0],bytes.data(),bytes.size(),MSG_DONTWAIT),-1);
  const std::array<unsigned char,8> nonce{1,2,3,4,5,6,7,8};
  ASSERT_EQ(send(pair.fd[0],nonce.data(),nonce.size(),MSG_NOSIGNAL),8);
  pollfd descriptor{pair.fd[0],POLLIN,0};
  ASSERT_EQ(poll(&descriptor,1,1000),1);
  ASSERT_EQ(recv(pair.fd[0],bytes.data(),bytes.size(),MSG_DONTWAIT),20);
  for(int i=12;i<20;++i)EXPECT_EQ(bytes[i],0);
  for(int i=0;i<8;++i)EXPECT_EQ(bytes[i],nonce[i]);
  const auto tick=std::uint32_t(bytes[8]) | (std::uint32_t(bytes[9])<<8) |
                  (std::uint32_t(bytes[10])<<16) | (std::uint32_t(bytes[11])<<24);
  EXPECT_EQ(tick,1004);
  receiver.SetTick(1004);
  EXPECT_EQ(recv(pair.fd[0],bytes.data(),bytes.size(),MSG_DONTWAIT),-1);
}

TEST(GestureReceiver, PersistentChannelAcceptsSequentialGrantsWithoutReconnect) {
  LocalPair pair;
  GestureReceiver receiver(pair.fd[1],"s",1004);
  auto send_json=[&](const nlohmann::json& value) {
    auto bytes=value.dump();
    ASSERT_EQ(send(pair.fd[0],bytes.data(),bytes.size(),MSG_NOSIGNAL),bytes.size());
  };
  auto consumed=[&]() -> std::uint64_t {
    std::array<unsigned char,8> nonce{};
    if(send(pair.fd[0],nonce.data(),nonce.size(),MSG_NOSIGNAL)!=8)
      throw std::runtime_error("clock send");
    pollfd descriptor{pair.fd[0],POLLIN,0};
    if(poll(&descriptor,1,1000)!=1)throw std::runtime_error("clock timeout");
    std::array<unsigned char,20> bytes{};
    if(recv(pair.fd[0],bytes.data(),bytes.size(),MSG_DONTWAIT)!=20)
      throw std::runtime_error("clock reply");
    std::uint64_t result=0;
    for(int i=0;i<8;++i)result|=std::uint64_t(bytes[12+i])<<(8*i);
    return result;
  };
  for(int id=1;id<=3;++id) {
    send_json({{"kind","grant"},{"session_id","s"},{"execution_id",id},{"plan_id",std::string(64,'a')}});
    send_json({{"kind","snapshot"},{"session_id","s"},{"execution_id",id},
               {"payload",nlohmann::json::parse(Packet())}});
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(!receiver.Snapshot() && std::chrono::steady_clock::now()<deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_TRUE(receiver.Snapshot());
    EXPECT_EQ(consumed(),id-1); // decoding is not control consumption
    auto invalid=*receiver.Snapshot();invalid.frames[0].weight=.5;
    receiver.MarkConsumed(invalid);
    EXPECT_EQ(consumed(),id-1); // active envelope cannot acknowledge exit
    invalid=*receiver.Snapshot();invalid.origin_sim_tick=900;
    receiver.MarkConsumed(invalid);
    EXPECT_EQ(consumed(),id-1); // stale zero envelope cannot acknowledge exit
    receiver.MarkConsumed(*receiver.Snapshot());
    EXPECT_EQ(consumed(),id);
    send_json({{"kind","release"},{"session_id","s"},{"execution_id",id}});
    while(receiver.Snapshot() && std::chrono::steady_clock::now()<deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_EQ(receiver.Snapshot(),nullptr);
  }
}
