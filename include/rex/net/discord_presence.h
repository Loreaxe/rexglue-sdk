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
/// Every frame is a little-endian opcode and length followed by JSON. Reads are
/// non-blocking and happen from `Pump`, which the owner calls from its own pump
/// thread; `SetActivity` may be called from any thread.
///
/// Discord not running is the ordinary case and is silent; a connection that
/// drops is retried on the usual cadence. Nothing here depends on the title: the
/// application id and the two default lines come from the title's rexnet.toml.
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
    /// Someone asked to join: answer true to send them the invite.
    std::function<bool(const std::string& user_id, const std::string& user_name)> on_join_request;
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

  bool connected() const { return state_ == State::kReady; }

 private:
  enum class State { kDisconnected, kHandshaking, kReady };

  bool TryConnect();
  bool OpenEndpoint(const std::string& endpoint);
  static std::vector<std::string> Endpoints();
  void Drop();
  bool WriteFrame(uint32_t op, const std::string& json);
  void ReadAvailable();
  void HandleFrame(const std::string& json);
  void HandleFrameChecked(const std::string& json);
  void PumpChecked();
  bool SendCommand(const std::string& json);
  std::string BuildSetActivity(const Activity& activity) const;
  void ApplyWanted();

  std::string application_id_;
  Assets assets_;
  Callbacks callbacks_;
  int64_t started_unix_ = 0;

  mutable std::mutex mutex_;
  Activity wanted_;
  Activity shown_;
  bool shown_valid_ = false;

  State state_ = State::kDisconnected;
  std::chrono::steady_clock::time_point last_attempt_{};
  std::chrono::steady_clock::time_point handshake_started_{};
  std::string endpoint_;
  std::vector<uint8_t> inbox_;
#if defined(_WIN32)
  void* pipe_ = nullptr;  // HANDLE
#else
  int fd_ = -1;
#endif
};

}  // namespace rex::net
