// @file        rex/net/discord_presence.h
// @brief       Discord Rich Presence over the local IPC socket (no Discord SDK).
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace rex::net {

/// Discord Rich Presence for a title, spoken straight to the Discord client's
/// local IPC endpoint: a handshake, then SET_ACTIVITY frames carrying the two
/// lines Discord shows and, while the local session is joinable, a party and a
/// join secret. The secret is RexNet's own join material (`RexNet::JoinSecret`),
/// so a Discord "Join" lands as `RexNet::JoinFromSecret` on the other machine.
///
/// Every frame is a little-endian opcode and length followed by JSON. The
/// socket is non-blocking in both directions: reads drain what is there, and
/// writes go through a bounded outbox flushed as the socket accepts them, so
/// `Pump` never waits on Discord. `Pump` is called from the owner's pump
/// thread; `SetActivity` and `ReplyJoinRequest` may be called from any thread
/// and take effect on the next pump. PING is answered with PONG and a CLOSE
/// from Discord drops the connection.
///
/// Discord not running is the ordinary case and is silent; a connection that
/// drops is retried every 15 s. A handshake Discord refuses (an application id
/// it does not know) backs off 15 s -> 60 s -> 300 s, and only the first
/// refusal and the first close are logged above debug. Nothing here depends on
/// the title: the application id and the two default lines come from the
/// title's rexnet.toml.
class DiscordPresence {
 public:
  struct Activity {
    std::string details;
    std::string state;
    /// Discord's party id while joinable; the session id in hex. Empty = no party.
    std::string party_id;
    std::string join_secret;
    bool joinable = false;
    int party_size = 1;
    int party_max = 2;
    bool operator==(const Activity&) const = default;
  };

  struct Callbacks {
    /// The player accepted an invite: the secret the host advertised.
    std::function<void(const std::string& secret)> on_join;
    /// Someone asked to join. true sends them the invite now, false closes the
    /// request now, nullopt leaves it open for the owner to answer later with
    /// ReplyJoinRequest. No callback = closed.
    std::function<std::optional<bool>(const std::string& user_id, const std::string& user_name)>
        on_join_request;
  };

  struct Assets {
    std::string large_image;
    std::string large_text;
  };

  DiscordPresence(std::string application_id, Assets assets, Callbacks callbacks);
  ~DiscordPresence();
  DiscordPresence(const DiscordPresence&) = delete;
  DiscordPresence& operator=(const DiscordPresence&) = delete;

  /// What Discord should show. Sent on the next pump if it changed.
  void SetActivity(const Activity& activity);
  /// Connects when needed, sends a changed activity, drains Discord's frames.
  void Pump();
  /// Clears the activity and closes the connection.
  void Close();
  /// Answer an open Ask to Join: accept = SEND_ACTIVITY_JOIN_INVITE, decline =
  /// CLOSE_ACTIVITY_REQUEST. Queued and sent on the next pump while connected;
  /// dropped if the connection is down (Discord has already timed it out).
  void ReplyJoinRequest(const std::string& user_id, bool accept);

  bool connected() const { return state_ == State::kReady; }

 private:
  enum class State { kDisconnected, kHandshaking, kReady };

  bool TryConnect();
  bool OpenEndpoint(const std::string& endpoint);
  static std::vector<std::string> Endpoints();
  void Drop();
  /// Queue one frame in the outbox and flush what the socket takes.
  bool WriteFrame(uint32_t op, const std::string& json);
  /// Push outbox bytes until the socket would block; false = connection gone.
  bool FlushOutbox();
  /// Read what is there without waiting, then dispatch whole frames by opcode.
  void ReadAvailable();
  void DispatchInbox();
  void HandleFrame(const std::string& json);
  void HandleFrameChecked(const std::string& json);
  void PumpChecked();
  bool SendCommand(const std::string& json);
  std::string BuildSetActivity(const Activity& activity) const;
  void ApplyWanted();
  void SendPendingReplies();
  /// The handshake was refused: lengthen the retry interval one step.
  void BackOff();

  std::string application_id_;
  Assets assets_;
  Callbacks callbacks_;
  int64_t started_unix_ = 0;

  mutable std::mutex mutex_;  // wanted_ and pending_replies_: any thread
  Activity wanted_;
  struct Reply {
    std::string user_id;
    bool accept = false;
  };
  std::vector<Reply> pending_replies_;
  Activity shown_;
  bool shown_valid_ = false;

  State state_ = State::kDisconnected;
  std::chrono::steady_clock::time_point last_attempt_{};
  std::chrono::steady_clock::time_point handshake_started_{};
  /// Current reconnect interval: 15 s, lengthened by each refused handshake
  /// (BackOff), reset by a READY.
  std::chrono::seconds retry_interval_{15};
  /// Logged once at info, then at debug, until a READY resets them.
  bool refusal_logged_ = false;
  bool close_logged_ = false;
  std::string endpoint_;
  std::vector<uint8_t> inbox_;
  /// Bytes the socket has not taken yet (kMaxOutbox bounds it; past that the
  /// connection is dropped rather than the game thread held).
  std::vector<uint8_t> outbox_;
#if defined(_WIN32)
  void* pipe_ = nullptr;  // HANDLE, opened overlapped
  /// One overlapped write in flight at a time: `write_buffer_` holds the
  /// bytes handed to the kernel and is not touched until it completes, so
  /// `outbox_` may grow meanwhile. The OVERLAPPED blocks live on the heap so
  /// windows.h stays out of this header.
  void* write_overlapped_ = nullptr;  // OVERLAPPED*, owns its hEvent
  void* read_overlapped_ = nullptr;   // OVERLAPPED*, owns its hEvent
  bool write_pending_ = false;
  std::vector<uint8_t> write_buffer_;
#else
  int fd_ = -1;
#endif
};

}  // namespace rex::net
