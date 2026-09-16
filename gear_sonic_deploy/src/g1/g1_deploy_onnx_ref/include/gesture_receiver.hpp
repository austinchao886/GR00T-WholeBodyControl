#pragma once
#include "gesture_decoder.hpp"
#include "gesture_session.hpp"
#include <atomic>
#include <cerrno>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

namespace sonic_gesture {
// Linux local-only transport. Supervisor supplies an already-connected private
// SOCK_SEQPACKET descriptor and a validated grant. No bind/listen/network port.
class GestureReceiver {
 public:
  GestureReceiver(int borrowed_fd, std::string session, std::string plan,
                  std::uint32_t current_tick)
      : decoder_(std::make_unique<GestureDecoder>(std::move(session),std::move(plan))), tick_(current_tick) {
    Open(borrowed_fd);
  }
  // Persistent mode: private channel must grant each execution before snapshots.
  GestureReceiver(int borrowed_fd, std::string session, std::uint32_t current_tick)
      : session_(std::make_unique<GestureSession>(std::move(session))), tick_(current_tick) {
    Open(borrowed_fd);
  }
 private:
  void Open(int borrowed_fd) {
    int type=0; socklen_t length=sizeof(type);
    sockaddr_storage address{}; socklen_t address_length=sizeof(address);
    ucred peer{}; socklen_t peer_length=sizeof(peer);
    if (getsockopt(borrowed_fd,SOL_SOCKET,SO_TYPE,&type,&length)!=0 || type!=SOCK_SEQPACKET ||
        getsockname(borrowed_fd,reinterpret_cast<sockaddr*>(&address),&address_length)!=0 ||
        address.ss_family!=AF_UNIX ||
        getsockopt(borrowed_fd,SOL_SOCKET,SO_PEERCRED,&peer,&peer_length)!=0 || peer.uid!=geteuid())
      throw std::invalid_argument("gesture transport must be connected same-uid Unix seqpacket");
    fd_=fcntl(borrowed_fd,F_DUPFD_CLOEXEC,3);
    if(fd_<0) throw std::runtime_error("cannot duplicate gesture descriptor");
    try { worker_=std::thread([this]{ Run(); }); }
    catch (...) {close(fd_);throw;}
  }
 public:
  ~GestureReceiver() {
    stopping_.store(true);
    if(worker_.joinable()) worker_.join();
    close(fd_);
  }
  GestureReceiver(const GestureReceiver&)=delete;
  GestureReceiver& operator=(const GestureReceiver&)=delete;
  void SetTick(std::uint32_t tick) {
    tick_.store(tick); // no unsolicited backlog while Python is idle
  }
  void MarkConsumed(const GestureSnapshot& snapshot) {
    // Called only AFTER a successful policy command was created from this
    // snapshot. Receipt/decoding alone is not completion.
    if(snapshot.execution_id==0 || !SnapshotCovers(snapshot,tick_.load(),std::chrono::steady_clock::now()))return;
    for(const auto& frame:snapshot.frames)
      if(frame.weight!=0 || frame.weight_rate!=0)return;
    consumed_execution_.store(snapshot.execution_id);
  }
  std::shared_ptr<const GestureSnapshot> Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!error_.empty()) throw std::runtime_error(error_);
    return latest_;
  }
 private:
  void Fail(const char* message) {
    std::lock_guard<std::mutex> lock(mutex_);
    error_=message; // latched: caller must take explicit safe recovery path
  }
  void Run() {
    try {
      std::array<char,262144> bytes;
      while(!stopping_.load()) {
        pollfd descriptor{fd_,POLLIN,0};
        int ready=poll(&descriptor,1,25);
        if(ready<0) {if(errno==EINTR)continue;Fail("gesture poll failed");return;}
        if(ready==0)continue;
        if(!(descriptor.revents&POLLIN)) {Fail("gesture channel disconnected");return;}
        const auto count=recv(fd_,bytes.data(),bytes.size(),MSG_DONTWAIT|MSG_TRUNC);
        if(count<0 && (errno==EAGAIN||errno==EINTR))continue;
        if(count<=0 || static_cast<std::size_t>(count)>bytes.size()) {
          Fail("gesture channel closed or oversized packet");return;
        }
        if(count==8) {
          // Echo the opaque request nonce followed by the current little-endian
          // physics tick. Query processing runs off the control thread.
          std::array<unsigned char,20> reply{};
          std::copy_n(bytes.begin(),8,reply.begin());
          const auto tick=tick_.load();
          for(int i=0;i<4;++i)reply[8+i]=static_cast<unsigned char>(tick>>(8*i));
          const auto consumed=consumed_execution_.load();
          for(int i=0;i<8;++i)reply[12+i]=static_cast<unsigned char>(consumed>>(8*i));
          if(send(fd_,reply.data(),reply.size(),MSG_DONTWAIT|MSG_NOSIGNAL)!=20) {
            Fail("gesture clock response blocked");return;
          }
          continue;
        }
        const auto message=std::string_view(bytes.data(),count);
        const auto now=std::chrono::steady_clock::now();
        auto snapshot=session_ ? session_->Process(message,tick_.load(),now,consumed_execution_.load())
                               : decoder_->Decode(message,tick_.load(),now);
        std::lock_guard<std::mutex> lock(mutex_);
        latest_=std::move(snapshot);
      }
    } catch(const std::exception& error) {
      Fail((std::string("gesture packet rejected: ")+error.what()).c_str());
    }
  }
  std::unique_ptr<GestureDecoder> decoder_;
  std::unique_ptr<GestureSession> session_;
  std::atomic<std::uint32_t> tick_;
  std::atomic<std::uint64_t> consumed_execution_{0};
  std::atomic<bool> stopping_{false};
  int fd_=-1;
  std::thread worker_;
  mutable std::mutex mutex_;
  std::shared_ptr<const GestureSnapshot> latest_;
  std::string error_;
};
}
