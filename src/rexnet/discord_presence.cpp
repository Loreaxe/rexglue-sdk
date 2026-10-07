// @file        rexnet/discord_presence.cpp
// @brief       Discord Rich Presence over the local IPC socket (no Discord SDK).
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

#include "rex/net/discord_presence.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>

#include <nlohmann/json.hpp>

#include "rex/logging.h"
#include "rex/net/rexnet.h"

#if defined(_WIN32)
#include <windows.h>
#else
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace rex::net {
namespace {

#define DISCORD_INFO(...) REXLOG_CAT_INFO(::rex::log::net(), __VA_ARGS__)
#define DISCORD_DEBUG(...) REXLOG_CAT_DEBUG(::rex::log::net(), __VA_ARGS__)

constexpr uint32_t kOpHandshake = 0;
constexpr uint32_t kOpFrame = 1;
constexpr uint32_t kOpClose = 2;

/// Discord stops reading a client that goes quiet; connecting is cheap, so a
/// client that is not there is retried on this cadence and never logged twice.
constexpr std::chrono::seconds kRetryInterval{15};
constexpr std::chrono::seconds kHandshakeTimeout{2};
/// A frame larger than this is not Discord talking to us.
constexpr size_t kMaxFrame = 1u << 20;

std::string Nonce() {
  static thread_local std::mt19937_64 rng{std::random_device{}()};
  char buf[33];
  std::snprintf(buf, sizeof(buf), "%016llx%016llx", static_cast<unsigned long long>(rng()),
                static_cast<unsigned long long>(rng()));
  return buf;
}

int64_t NowUnix() {
  return std::chrono::duration_cast<std::chrono::seconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

uint32_t ProcessId() {
#if defined(_WIN32)
  return static_cast<uint32_t>(GetCurrentProcessId());
#else
  return static_cast<uint32_t>(getpid());
#endif
}

}  // namespace

DiscordPresence::DiscordPresence(std::string application_id, Assets assets, Callbacks callbacks)
    : application_id_(std::move(application_id)),
      assets_(std::move(assets)),
      callbacks_(std::move(callbacks)),
      started_unix_(NowUnix()) {
  // First attempt on the first pump: an interval ago, so nothing waits 15 s.
  last_attempt_ = std::chrono::steady_clock::now() - kRetryInterval;
}

DiscordPresence::~DiscordPresence() { Close(); }

void DiscordPresence::SetActivity(const Activity& activity) {
  std::lock_guard lock(mutex_);
  wanted_ = activity;
}

void DiscordPresence::Close() {
  if (state_ != State::kDisconnected) {
    // A clear before the close, so the presence does not outlive the game.
    nlohmann::json clear = {
        {"cmd", "SET_ACTIVITY"},
        {"args", {{"pid", ProcessId()}}},
        {"nonce", Nonce()},
    };
    WriteFrame(kOpFrame, clear.dump());
    WriteFrame(kOpClose, "{}");
  }
  Drop();
}

void DiscordPresence::Pump() {
  try {
    PumpChecked();
  } catch (const std::exception& error) {
    DISCORD_INFO("discord: pump failed ({}); reconnecting", error.what());
    Drop();
  }
}

void DiscordPresence::PumpChecked() {
  if (state_ == State::kDisconnected) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_attempt_ < kRetryInterval) {
      return;
    }
    last_attempt_ = now;
    if (!TryConnect()) {
      return;
    }
  }

  ReadAvailable();

  if (state_ == State::kHandshaking &&
      std::chrono::steady_clock::now() - handshake_started_ > kHandshakeTimeout) {
    DISCORD_INFO("discord: no answer to the handshake on {}", endpoint_);
    Drop();
    return;
  }

  if (state_ == State::kReady) {
    ApplyWanted();
  }
}

void DiscordPresence::ApplyWanted() {
  Activity wanted;
  {
    std::lock_guard lock(mutex_);
    wanted = wanted_;
  }
  if (shown_valid_ && wanted == shown_) {
    return;
  }
  if (SendCommand(BuildSetActivity(wanted))) {
    shown_ = wanted;
    shown_valid_ = true;
    DISCORD_INFO("discord: {} — {}{}", wanted.details, wanted.state,
                 wanted.party_id.empty() ? "" : " (joinable)");
  }
}

std::string DiscordPresence::BuildSetActivity(const Activity& activity) const {
  nlohmann::json act = {
      {"details", activity.details},
      {"timestamps", {{"start", started_unix_}}},
  };
  if (!assets_.large_image.empty()) {
    act["assets"] = {{"large_image", assets_.large_image}, {"large_text", assets_.large_text}};
  }
  if (!activity.state.empty()) {
    act["state"] = activity.state;
  }
  if (!activity.party_id.empty()) {
    act["party"] = {
        {"id", activity.party_id},
        {"size", {activity.party_size, activity.party_max}},
    };
    act["secrets"] = {{"join", activity.join_secret}};
  }
  nlohmann::json cmd = {
      {"cmd", "SET_ACTIVITY"},
      {"args", {{"pid", ProcessId()}, {"activity", act}}},
      {"nonce", Nonce()},
  };
  return cmd.dump();
}

bool DiscordPresence::SendCommand(const std::string& json) {
  if (state_ == State::kDisconnected) {
    return false;
  }
  if (!WriteFrame(kOpFrame, json)) {
    DISCORD_INFO("discord: connection lost");
    Drop();
    return false;
  }
  return true;
}

bool DiscordPresence::TryConnect() {
  for (const std::string& endpoint : Endpoints()) {
    if (!OpenEndpoint(endpoint)) {
      continue;
    }
    nlohmann::json hello = {{"v", 1}, {"client_id", application_id_}};
    if (!WriteFrame(kOpHandshake, hello.dump())) {
      Drop();
      continue;
    }
    endpoint_ = endpoint;
    state_ = State::kHandshaking;
    handshake_started_ = std::chrono::steady_clock::now();
    inbox_.clear();
    shown_valid_ = false;
    return true;
  }
  return false;
}

namespace {

/// A string field, or empty: Discord sends `"evt": null` and `"data": null` on
/// every command reply, and nlohmann's `value()` throws on a null where a
/// string is expected. Nothing here may throw: the frame is read on the pump
/// thread, and an exception there terminated the process.
std::string StringField(const nlohmann::json& object, const char* key) {
  if (!object.is_object()) {
    return {};
  }
  const auto it = object.find(key);
  return it != object.end() && it->is_string() ? it->get<std::string>() : std::string();
}

const nlohmann::json& ObjectField(const nlohmann::json& object, const char* key) {
  static const nlohmann::json kNone;
  if (!object.is_object()) {
    return kNone;
  }
  const auto it = object.find(key);
  return it != object.end() && it->is_object() ? *it : kNone;
}

}  // namespace

void DiscordPresence::HandleFrame(const std::string& json) {
  try {
    HandleFrameChecked(json);
  } catch (const std::exception& error) {
    // A frame we did not understand is Discord's business, not a reason to
    // take the game down; the connection is dropped and retried.
    DISCORD_INFO("discord: frame not understood ({}); reconnecting", error.what());
    Drop();
  }
}

void DiscordPresence::HandleFrameChecked(const std::string& json) {
  nlohmann::json doc = nlohmann::json::parse(json, nullptr, false);
  if (doc.is_discarded() || !doc.is_object()) {
    return;
  }
  const std::string evt = StringField(doc, "evt");
  const nlohmann::json& data = ObjectField(doc, "data");
  if (state_ == State::kHandshaking) {
    if (evt == "READY") {
      state_ = State::kReady;
      DISCORD_INFO("discord: connected on {}", endpoint_);
      // Without these Discord shows the party but tells us nothing when it is used.
      for (const char* name : {"ACTIVITY_JOIN", "ACTIVITY_JOIN_REQUEST"}) {
        nlohmann::json sub = {
            {"cmd", "SUBSCRIBE"},
            {"evt", name},
            {"args", nlohmann::json::object()},
            {"nonce", Nonce()},
        };
        SendCommand(sub.dump());
      }
    } else if (evt == "ERROR") {
      DISCORD_INFO("discord: handshake refused: {}", data.dump());
      Drop();
    }
    return;
  }

  if (evt == "ERROR") {
    DISCORD_INFO("discord: {}", data.dump());
  } else if (evt == "ACTIVITY_JOIN") {
    const std::string secret = StringField(data, "secret");
    if (!secret.empty() && callbacks_.on_join) {
      callbacks_.on_join(secret);
    }
  } else if (evt == "ACTIVITY_JOIN_REQUEST") {
    const nlohmann::json& user = ObjectField(data, "user");
    const std::string user_id = StringField(user, "id");
    const std::string name = StringField(user, "username");
    if (user_id.empty()) {
      return;
    }
    const bool approve = callbacks_.on_join_request ? callbacks_.on_join_request(user_id, name) : false;
    DISCORD_INFO("discord: {} asks to join: {}", name, approve ? "approved" : "declined");
    nlohmann::json answer = {
        {"cmd", approve ? "SEND_ACTIVITY_JOIN_INVITE" : "CLOSE_ACTIVITY_REQUEST"},
        {"args", {{"user_id", user_id}}},
        {"nonce", Nonce()},
    };
    SendCommand(answer.dump());
  }
}

// --- platform ------------------------------------------------------------

std::vector<std::string> DiscordPresence::Endpoints() {
  std::vector<std::string> out;
#if defined(_WIN32)
  for (int i = 0; i < 10; ++i) {
    out.push_back("\\\\.\\pipe\\discord-ipc-" + std::to_string(i));
  }
#else
  const char* runtime = std::getenv("XDG_RUNTIME_DIR");
  if (!runtime || !*runtime) {
    runtime = std::getenv("TMPDIR");
  }
  if (!runtime || !*runtime) {
    runtime = "/tmp";
  }
  const std::string base(runtime);
  // The client's own runtime dir first, then where a sandboxed Discord puts it.
  for (const std::string& root :
       {base, base + "/app/com.discordapp.Discord", base + "/snap.discord",
        base + "/snap.discord-canary"}) {
    for (int i = 0; i < 10; ++i) {
      out.push_back(root + "/discord-ipc-" + std::to_string(i));
    }
  }
#endif
  return out;
}

#if defined(_WIN32)

bool DiscordPresence::OpenEndpoint(const std::string& endpoint) {
  HANDLE pipe = CreateFileA(endpoint.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                            OPEN_EXISTING, 0, nullptr);
  if (pipe == INVALID_HANDLE_VALUE) {
    return false;
  }
  pipe_ = pipe;
  return true;
}

void DiscordPresence::Drop() {
  if (pipe_ != nullptr) {
    CloseHandle(static_cast<HANDLE>(pipe_));
    pipe_ = nullptr;
  }
  state_ = State::kDisconnected;
  shown_valid_ = false;
  inbox_.clear();
}

bool DiscordPresence::WriteFrame(uint32_t op, const std::string& json) {
  if (pipe_ == nullptr) {
    return false;
  }
  std::vector<uint8_t> frame(8 + json.size());
  const uint32_t len = static_cast<uint32_t>(json.size());
  std::memcpy(frame.data(), &op, 4);
  std::memcpy(frame.data() + 4, &len, 4);
  std::memcpy(frame.data() + 8, json.data(), json.size());
  DWORD written = 0;
  return WriteFile(static_cast<HANDLE>(pipe_), frame.data(), static_cast<DWORD>(frame.size()),
                   &written, nullptr) &&
         written == frame.size();
}

void DiscordPresence::ReadAvailable() {
  if (pipe_ == nullptr) {
    return;
  }
  for (;;) {
    DWORD available = 0;
    if (!PeekNamedPipe(static_cast<HANDLE>(pipe_), nullptr, 0, nullptr, &available, nullptr)) {
      DISCORD_INFO("discord: connection closed");
      Drop();
      return;
    }
    if (available == 0) {
      break;
    }
    uint8_t buf[4096];
    DWORD got = 0;
    if (!ReadFile(static_cast<HANDLE>(pipe_), buf, sizeof(buf), &got, nullptr) || got == 0) {
      Drop();
      return;
    }
    inbox_.insert(inbox_.end(), buf, buf + got);
  }
  while (inbox_.size() >= 8) {
    uint32_t len = 0;
    std::memcpy(&len, inbox_.data() + 4, 4);
    if (len > kMaxFrame) {
      Drop();
      return;
    }
    if (inbox_.size() < 8 + len) {
      break;
    }
    std::string json(reinterpret_cast<const char*>(inbox_.data() + 8), len);
    inbox_.erase(inbox_.begin(), inbox_.begin() + 8 + len);
    HandleFrame(json);
  }
}

#else

bool DiscordPresence::OpenEndpoint(const std::string& endpoint) {
  struct stat st {};
  if (stat(endpoint.c_str(), &st) != 0 || !S_ISSOCK(st.st_mode)) {
    return false;
  }
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    return false;
  }
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (endpoint.size() >= sizeof(addr.sun_path)) {
    close(fd);
    return false;
  }
  std::strncpy(addr.sun_path, endpoint.c_str(), sizeof(addr.sun_path) - 1);
  if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    return false;
  }
  fd_ = fd;
  return true;
}

void DiscordPresence::Drop() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  state_ = State::kDisconnected;
  shown_valid_ = false;
  inbox_.clear();
}

bool DiscordPresence::WriteFrame(uint32_t op, const std::string& json) {
  if (fd_ < 0) {
    return false;
  }
  std::vector<uint8_t> frame(8 + json.size());
  const uint32_t len = static_cast<uint32_t>(json.size());
  std::memcpy(frame.data(), &op, 4);
  std::memcpy(frame.data() + 4, &len, 4);
  std::memcpy(frame.data() + 8, json.data(), json.size());
  size_t sent = 0;
  while (sent < frame.size()) {
    const ssize_t n = send(fd_, frame.data() + sent, frame.size() - sent, MSG_NOSIGNAL);
    if (n <= 0) {
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

void DiscordPresence::ReadAvailable() {
  if (fd_ < 0) {
    return;
  }
  for (;;) {
    pollfd pfd{fd_, POLLIN, 0};
    const int ready = poll(&pfd, 1, 0);
    if (ready <= 0) {
      break;
    }
    uint8_t buf[4096];
    const ssize_t got = recv(fd_, buf, sizeof(buf), MSG_DONTWAIT);
    if (got == 0 || (got < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
      DISCORD_INFO("discord: connection closed");
      Drop();
      return;
    }
    if (got < 0) {
      break;
    }
    inbox_.insert(inbox_.end(), buf, buf + got);
  }
  while (inbox_.size() >= 8) {
    uint32_t len = 0;
    std::memcpy(&len, inbox_.data() + 4, 4);
    if (len > kMaxFrame) {
      Drop();
      return;
    }
    if (inbox_.size() < 8 + len) {
      break;
    }
    std::string json(reinterpret_cast<const char*>(inbox_.data() + 8), len);
    inbox_.erase(inbox_.begin(), inbox_.begin() + 8 + len);
    HandleFrame(json);
  }
}

#endif

}  // namespace rex::net
