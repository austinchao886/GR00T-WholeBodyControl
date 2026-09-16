#pragma once
#include "gesture_decoder.hpp"

namespace sonic_gesture {
// Only the private Supervisor channel may call this state machine. These are
// execution grants issued AFTER DDS approval, not requests for self-approval.
class GestureSession {
 public:
  explicit GestureSession(std::string session):session_(std::move(session)) {
    if(session_.empty() || session_.size()>128)throw std::invalid_argument("invalid session");
  }
  std::shared_ptr<const GestureSnapshot> Process(std::string_view bytes,std::uint32_t tick,
                                                std::chrono::steady_clock::time_point now,
                                                std::uint64_t consumed_execution=0) {
    if(bytes.empty() || bytes.size()>262144)throw std::invalid_argument("grant packet size");
    auto message=nlohmann::json::parse(bytes.begin(),bytes.end(),
      [](int depth,nlohmann::json::parse_event_t,nlohmann::json&){
        if(depth>8)throw std::invalid_argument("grant nesting");return true;
      });
    if(!message.is_object() || message.at("session_id")!=session_ ||
       !message.at("execution_id").is_number_unsigned())
      throw std::invalid_argument("grant session/identity mismatch");
    auto id=message.at("execution_id").get<std::uint64_t>();
    if(id==0 || id>=(1ULL<<53))throw std::invalid_argument("invalid execution id");
    const auto kind=message.at("kind").get<std::string>();
    if(kind=="grant") {
      if(message.size()!=4 || decoder_ || id<=last_execution_)
        throw std::invalid_argument("active/replayed grant");
      auto candidate=std::make_unique<GestureDecoder>(session_,message.at("plan_id").get<std::string>());
      decoder_=std::move(candidate);last_execution_=id;latest_.reset();
      return nullptr;
    }
    if(!decoder_ || id!=last_execution_)throw std::invalid_argument("no matching execution grant");
    if(kind=="snapshot") {
      if(message.size()!=4)throw std::invalid_argument("snapshot envelope mismatch");
      auto candidate=decoder_->Decode(message.at("payload").dump(),tick,now);
      auto tagged=std::make_shared<GestureSnapshot>(*candidate);
      tagged->execution_id=id;
      latest_=std::move(tagged);
      return latest_;
    }
    if(kind=="release") {
      if(message.size()!=3 || !latest_ || consumed_execution!=id || !SnapshotCovers(*latest_,tick,now))
        throw std::invalid_argument("release requires fresh completed exit");
      const auto offset=tick-latest_->origin_sim_tick;
      for(std::size_t i=offset;i<latest_->frames.size();++i)
        if(latest_->frames[i].weight!=0 || latest_->frames[i].weight_rate!=0)
          throw std::invalid_argument("gesture exit incomplete");
      decoder_.reset();latest_.reset();return nullptr;
    }
    throw std::invalid_argument("unknown gesture message");
  }
 private:
  const std::string session_;
  std::uint64_t last_execution_=0;
  std::unique_ptr<GestureDecoder> decoder_;
  std::shared_ptr<const GestureSnapshot> latest_;
};
}
