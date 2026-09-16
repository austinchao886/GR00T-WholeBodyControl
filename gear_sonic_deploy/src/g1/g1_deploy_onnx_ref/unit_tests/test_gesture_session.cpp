#include <gtest/gtest.h>
#include "gesture_session.hpp"
using namespace sonic_gesture;
static nlohmann::json Grant(int id) {
  return {{"kind","grant"},{"session_id","s"},{"execution_id",id},{"plan_id",std::string(64,'a')}};
}
static nlohmann::json Release(int id) {
  return {{"kind","release"},{"session_id","s"},{"execution_id",id}};
}
static nlohmann::json Snapshot(int id,bool active=false) {
  std::vector<std::vector<double>> frames(256,std::vector<double>(16,0.));
  if(active)frames[4][14]=.5;
  nlohmann::json payload={{"schema_version",1},{"session_id","s"},{"plan_id",std::string(64,'a')},
    {"sequence",0},{"origin_sim_tick",1000},{"physics_dt_s",.005},
    {"joint_indices",kRightArmIsaacLab},{"frames",frames}};
  return {{"kind","snapshot"},{"session_id","s"},{"execution_id",id},{"payload",payload}};
}
TEST(GestureSession, MultipleExecutionsWithoutSessionRestart) {
  GestureSession session("s");auto now=std::chrono::steady_clock::now();
  for(int id=1;id<=10;++id) {
    EXPECT_EQ(session.Process(Grant(id).dump(),1000,now),nullptr);
    auto snapshot=session.Process(Snapshot(id).dump(),1004,now);
    ASSERT_TRUE(snapshot);EXPECT_EQ(snapshot->execution_id,id);
    EXPECT_THROW(session.Process(Release(id).dump(),1004,now,id-1),std::invalid_argument);
    EXPECT_EQ(session.Process(Release(id).dump(),1004,now,id),nullptr);
  }
  EXPECT_THROW(session.Process(Grant(1).dump(),1004,now),std::invalid_argument);
  EXPECT_THROW(session.Process(Snapshot(10).dump(),1004,now),std::invalid_argument);
}
TEST(GestureSession, RejectsOverlapAndIncompleteOrStaleExit) {
  GestureSession session("s");auto now=std::chrono::steady_clock::now();
  session.Process(Grant(1).dump(),1000,now);
  EXPECT_THROW(session.Process(Grant(2).dump(),1000,now),std::invalid_argument);
  EXPECT_THROW(session.Process(Release(1).dump(),1000,now),std::invalid_argument);
  session.Process(Snapshot(1,true).dump(),1004,now);
  EXPECT_THROW(session.Process(Release(1).dump(),1004,now,1),std::invalid_argument);
  auto complete=Snapshot(1);complete["payload"]["sequence"]=1;
  session.Process(complete.dump(),1004,now);
  EXPECT_THROW(session.Process(Release(1).dump(),1041,now,1),std::invalid_argument);
  EXPECT_NO_THROW(session.Process(Release(1).dump(),1004,now,1));
}
TEST(GestureSession, RejectsCrossSessionAndCrossExecution) {
  GestureSession session("s");auto now=std::chrono::steady_clock::now();
  auto grant=Grant(1);grant["session_id"]="old";
  EXPECT_THROW(session.Process(grant.dump(),1000,now),std::invalid_argument);
  session.Process(Grant(1).dump(),1000,now);
  EXPECT_THROW(session.Process(Snapshot(2).dump(),1004,now),std::invalid_argument);
}
