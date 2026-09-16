#pragma once
#include "gesture_composer.hpp"
#include <nlohmann/json.hpp>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace sonic_gesture {
// Construct only after the supervisor's grant is accepted. Matching identifiers
// prevent cross-plan/session reuse but do not authenticate a network sender.
// Single receiver-thread ownership; callers publish the returned immutable ptr.
class GestureDecoder {
 public:
  GestureDecoder(std::string session, std::string approved_plan)
      : session_(std::move(session)), plan_(std::move(approved_plan)) {
    if (session_.empty() || session_.size()>128 || plan_.size()!=64 ||
        plan_.find_first_not_of("0123456789abcdef")!=std::string::npos)
      throw std::invalid_argument("invalid gesture grant identity");
  }

  std::shared_ptr<const GestureSnapshot> Decode(
      std::string_view bytes, std::uint32_t current_tick,
      std::chrono::steady_clock::time_point received_at) {
    if (bytes.empty() || bytes.size()>262144)
      throw std::invalid_argument("gesture packet size exceeded");
    auto j=nlohmann::json::parse(bytes.begin(),bytes.end(),
        [](int depth,nlohmann::json::parse_event_t,nlohmann::json&) {
          if (depth>6) throw std::invalid_argument("gesture JSON nesting exceeded");
          return true;
        });
    if (!j.is_object() || j.size()!=8 || UInt(j.at("schema_version"))!=1 ||
        j.at("session_id")!=session_ || j.at("plan_id")!=plan_ ||
        !j.at("physics_dt_s").is_number() || j.at("physics_dt_s").get<double>()!=.005 ||
        j.at("joint_indices")!=nlohmann::json(kRightArmIsaacLab))
      throw std::invalid_argument("gesture schema/identity mismatch");
    auto sequence=UInt(j.at("sequence"));
    auto tick=UInt(j.at("origin_sim_tick"));
    if (sequence>=(1ULL<<53) || tick>std::numeric_limits<std::uint32_t>::max() ||
        (last_sequence_ && sequence<=*last_sequence_) ||
        (last_tick_ && tick<*last_tick_))
      throw std::invalid_argument("gesture replay or clock regression");
    auto result=std::make_shared<GestureSnapshot>();
    result->origin_sim_tick=static_cast<std::uint32_t>(tick);
    result->received_at=received_at;
    if (!SnapshotCovers(*result,current_tick,received_at))
      throw std::invalid_argument("gesture buffer does not cover current tick");
    const auto& frames=j.at("frames");
    if (!frames.is_array() || frames.size()!=result->frames.size())
      throw std::invalid_argument("gesture frame count mismatch");
    for (std::size_t i=0;i<frames.size();++i) {
      if (!frames[i].is_array() || frames[i].size()!=16)
        throw std::invalid_argument("gesture frame width mismatch");
      std::array<double,16> values;
      for (int k=0;k<16;++k) {
        if (!frames[i][k].is_number()) throw std::invalid_argument("nonnumeric gesture sample");
        values[k]=frames[i][k].get<double>();
        if (!IsFinite(values[k])) throw std::invalid_argument("nonfinite gesture sample");
      }
      auto& arm=result->frames[i];
      std::copy_n(values.begin(),7,arm.q.begin());
      std::copy_n(values.begin()+7,7,arm.dq.begin());
      arm.weight=values[14]; arm.weight_rate=values[15];
      Compose(JointReference{},arm); // validate envelope, including zero endpoints
    }
    // Commit anti-replay state only after the complete packet is validated.
    last_sequence_=sequence; last_tick_=static_cast<std::uint32_t>(tick);
    return result;
  }
 private:
  static std::uint64_t UInt(const nlohmann::json& j) {
    if (!j.is_number_unsigned()) throw std::invalid_argument("expected unsigned integer");
    return j.get<std::uint64_t>();
  }
  const std::string session_,plan_;
  std::optional<std::uint64_t> last_sequence_;
  std::optional<std::uint32_t> last_tick_;
};
}
