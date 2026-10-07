// @file        rexnet/discord_presence.cpp
// @brief       Discord Rich Presence over the local IPC socket (no Discord SDK).
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

#include "rex/net/discord_presence.h"

#include <algorithm>
#include <chrono>
#include <cerrno>
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
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace rex::net {
namespace {

#define DISCORD_INFO(...) REXLOG_CAT_INFO(::rex::log::net(), __VA_ARGS__)
#define DISCORD_DEBUG(...) REXLOG_CAT_DEBUG(::rex::log::net(), __VA_ARGS__)
/// Info the first time, debug after, until `flag` is cleared again.
#define DISCORD_INFO_ONCE(flag, ...) \
  do {                               \
    if (flag) {                      \
      DISCORD_DEBUG(__VA_ARGS__);    \
    } else {                         \
      DISCORD_INFO(__VA_ARGS__);     \
      (flag) = true;                 \
    }                                \
  } while (0)

/// Discord IPC opcodes.
constexpr uint32_t kOpHandshake = 0;
constexpr uint32_t kOpFrame = 1;
constexpr uint32_t kOpClose = 2;
constexpr uint32_t kOpPing = 3;
constexpr uint32_t kOpPong = 4;

/// Discord stops reading a client that goes quiet; connecting is cheap, so a
/// client that is not there is retried on this cadence and never logged twice.
constexpr std::chrono::seconds kRetryInterval{15};
/// Reconnect interval after a refused handshake: one step per refusal.
constexpr std::chrono::seconds kRetryRefusedOnce{60};
constexpr std::chrono::seconds kRetryRefusedMax{300};
constexpr std::chrono::seconds kHandshakeTimeout{2};
/// A frame larger than this is not Discord talking to us.
constexpr size_t kMaxFrame = 1u << 20;
/// Bytes the socket may owe us before the connection is dropped instead of
/// the pump thread waiting on it.
constexpr size_t kMaxOutbox = 256u << 10;

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

/// The answer to an Ask to Join, as Discord's IPC takes it.
std::string JoinReplyCommand(const std::string& user_id, bool accept) {
  nlohmann::json answer = {
      {"cmd", accept ? "SEND_ACTIVITY_JOIN_INVITE" : "CLOSE_ACTIVITY_REQUEST"},
      {"args", {{"user_id", user_id}}},
      {"nonce", Nonce()},
  };
  return answer.dump();
}

}  // namespace

DiscordPresence::DiscordPresence(std::string application_id, Assets assets, Callbacks callbacks)
    : application_id_(std::move(application_id)),
      assets_(std::move(assets)),
      callbacks_(std::move(callbacks)),
      started_unix_(NowUnix()) {
  // First attempt on the first pump: an interval ago, so nothing waits 15 s.
  last_attempt_ = std::chrono::steady_clock::now() - retry_interval_;
}

DiscordPresence::~DiscordPresence() { Close(); }

void DiscordPresence::SetActivity(const Activity& activity) {
  std::lock_guard lock(mutex_);
  wanted_ = activity;
}

void DiscordPresence::ReplyJoinRequest(const std::string& user_id, bool accept) {
  std::lock_guard lock(mutex_);
  pending_replies_.push_back({user_id, accept});
}

void DiscordPresence::Close() {
  if (state_ != State::kDisconnected) {
    // A clear before the close, so the presence does not outlive the game.
    // Best effort: the socket takes what it takes without waiting.
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
    if (now - last_attempt_ < retry_interval_) {
      return;
    }
    last_attempt_ = now;
    if (!TryConnect()) {
      return;
    }
  }

  // Whatever the socket would not take last time goes first.
  if (!FlushOutbox()) {
    DISCORD_INFO_ONCE(close_logged_, "discord: connection closed");
    Drop();
    return;
  }

  ReadAvailable();
  if (state_ == State::kDisconnected) {
    return;
  }

  if (state_ == State::kHandshaking &&
      std::chrono::steady_clock::now() - handshake_started_ > kHandshakeTimeout) {
    DISCORD_INFO_ONCE(refusal_logged_, "discord: no answer to the handshake on {}", endpoint_);
    Drop();
    return;
  }

  if (state_ == State::kReady) {
    SendPendingReplies();
    ApplyWanted();
  }
}

void DiscordPresence::SendPendingReplies() {
  std::vector<Reply> replies;
  {
    std::lock_guard lock(mutex_);
    replies.swap(pending_replies_);
  }
  for (const Reply& reply : replies) {
    if (!SendCommand(JoinReplyCommand(reply.user_id, reply.accept))) {
      return;
    }
    DISCORD_INFO("discord: join request from {} {}", reply.user_id,
                 reply.accept ? "approved" : "declined");
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
    DISCORD_INFO_ONCE(close_logged_, "discord: connection lost");
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

void DiscordPresence::BackOff() {
  retry_interval_ = retry_interval_ < kRetryRefusedOnce ? kRetryRefusedOnce : kRetryRefusedMax;
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
      retry_interval_ = kRetryInterval;
      refusal_logged_ = false;
      close_logged_ = false;
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
      // An application id Discord does not know: it will say so every time,
      // so the retries stretch out and only the first is worth a line.
      DISCORD_INFO_ONCE(refusal_logged_, "discord: handshake refused: {}", data.dump());
      BackOff();
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
    const std::optional<bool> answer = callbacks_.on_join_request
                                           ? callbacks_.on_join_request(user_id, name)
                                           : std::optional<bool>(false);
    if (!answer.has_value()) {
      DISCORD_INFO("discord: {} asks to join: waiting for the host", name);
      return;
    }
    DISCORD_INFO("discord: {} asks to join: {}", name, *answer ? "approved" : "declined");
    SendCommand(JoinReplyCommand(user_id, *answer));
  }
}

void DiscordPresence::DispatchInbox() {
  while (state_ != State::kDisconnected && inbox_.size() >= 8) {
    uint32_t op = 0;
    uint32_t len = 0;
    std::memcpy(&op, inbox_.data(), 4);
    std::memcpy(&len, inbox_.data() + 4, 4);
    if (len > kMaxFrame) {
      Drop();
      return;
    }
    if (inbox_.size() < 8 + len) {
      break;
    }
    std::string payload(reinterpret_cast<const char*>(inbox_.data() + 8), len);
    inbox_.erase(inbox_.begin(), inbox_.begin() + 8 + len);
    switch (op) {
      case kOpFrame:
        HandleFrame(payload);
        break;
      case kOpPing:
        // Answered with the same payload, as the protocol asks.
        if (!WriteFrame(kOpPong, payload)) {
          Drop();
          return;
        }
        break;
      case kOpClose:
        DISCORD_DEBUG("discord: close from the client: {}", payload);
        Drop();
        return;
      case kOpPong:
      default:
        break;
    }
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

bool DiscordPresence::WriteFrame(uint32_t op, const std::string& json) {
#if defined(_WIN32)
  if (pipe_ == nullptr) {
    return false;
  }
#else
  if (fd_ < 0) {
    return false;
  }
#endif
  if (outbox_.size() + 8 + json.size() > kMaxOutbox) {
    // Discord has stopped reading; the connection is dropped by the caller.
    return false;
  }
  const uint32_t len = static_cast<uint32_t>(json.size());
  const size_t at = outbox_.size();
  outbox_.resize(at + 8 + json.size());
  std::memcpy(outbox_.data() + at, &op, 4);
  std::memcpy(outbox_.data() + at + 4, &len, 4);
  std::memcpy(outbox_.data() + at + 8, json.data(), json.size());
  return FlushOutbox();
}

#if defined(_WIN32)

namespace {
OVERLAPPED* NewOverlapped() {
  auto* ov = new OVERLAPPED{};
  ov->hEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
  return ov;
}

void FreeOverlapped(void*& slot) {
  auto* ov = static_cast<OVERLAPPED*>(slot);
  if (ov) {
    if (ov->hEvent) {
      CloseHandle(ov->hEvent);
    }
    delete ov;
  }
  slot = nullptr;
}
}  // namespace

bool DiscordPresence::OpenEndpoint(const std::string& endpoint) {
  HANDLE pipe = CreateFileA(endpoint.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                            OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
  if (pipe == INVALID_HANDLE_VALUE) {
    return false;
  }
  pipe_ = pipe;
  write_overlapped_ = NewOverlapped();
  read_overlapped_ = NewOverlapped();
  write_pending_ = false;
  write_buffer_.clear();
  outbox_.clear();
  return true;
}

void DiscordPresence::Drop() {
  if (pipe_ != nullptr) {
    auto* pipe = static_cast<HANDLE>(pipe_);
    if (write_pending_) {
      // The kernel still owns write_buffer_ and the OVERLAPPED: cancel and
      // wait for the cancellation to land before either is freed.
      auto* ov = static_cast<OVERLAPPED*>(write_overlapped_);
      CancelIoEx(pipe, ov);
      DWORD done = 0;
      GetOverlappedResult(pipe, ov, &done, TRUE);
      write_pending_ = false;
    }
    CloseHandle(pipe);
    pipe_ = nullptr;
  }
  FreeOverlapped(write_overlapped_);
  FreeOverlapped(read_overlapped_);
  write_buffer_.clear();
  outbox_.clear();
  state_ = State::kDisconnected;
  shown_valid_ = false;
  inbox_.clear();
  std::lock_guard lock(mutex_);
  pending_replies_.clear();
}

bool DiscordPresence::FlushOutbox() {
  if (pipe_ == nullptr) {
    return false;
  }
  auto* pipe = static_cast<HANDLE>(pipe_);
  auto* ov = static_cast<OVERLAPPED*>(write_overlapped_);
  if (write_pending_) {
    DWORD done = 0;
    if (!GetOverlappedResult(pipe, ov, &done, FALSE)) {
      return GetLastError() == ERROR_IO_INCOMPLETE;
    }
    write_pending_ = false;
    write_buffer_.clear();
  }
  if (outbox_.empty()) {
    return true;
  }
  write_buffer_.swap(outbox_);
  outbox_.clear();
  ResetEvent(ov->hEvent);
  ov->Offset = 0;
  ov->OffsetHigh = 0;
  DWORD written = 0;
  if (WriteFile(pipe, write_buffer_.data(), static_cast<DWORD>(write_buffer_.size()), &written,
                ov)) {
    write_buffer_.clear();
    return true;
  }
  if (GetLastError() == ERROR_IO_PENDING) {
    write_pending_ = true;
    return true;
  }
  return false;
}

void DiscordPresence::ReadAvailable() {
  if (pipe_ == nullptr) {
    return;
  }
  auto* pipe = static_cast<HANDLE>(pipe_);
  auto* ov = static_cast<OVERLAPPED*>(read_overlapped_);
  for (;;) {
    DWORD available = 0;
    if (!PeekNamedPipe(pipe, nullptr, 0, nullptr, &available, nullptr)) {
      DISCORD_INFO_ONCE(close_logged_, "discord: connection closed");
      Drop();
      return;
    }
    if (available == 0) {
      break;
    }
    uint8_t buf[4096];
    const DWORD want = std::min<DWORD>(available, sizeof(buf));
    ResetEvent(ov->hEvent);
    ov->Offset = 0;
    ov->OffsetHigh = 0;
    DWORD got = 0;
    if (!ReadFile(pipe, buf, want, &got, ov)) {
      // Only what PeekNamedPipe reported is asked for, so a pending read
      // completes at once; waiting on it is bounded.
      if (GetLastError() != ERROR_IO_PENDING || !GetOverlappedResult(pipe, ov, &got, TRUE)) {
        DISCORD_INFO_ONCE(close_logged_, "discord: connection closed");
        Drop();
        return;
      }
    }
    if (got == 0) {
      break;
    }
    inbox_.insert(inbox_.end(), buf, buf + got);
    if (inbox_.size() > kMaxFrame + 8) {
      Drop();
      return;
    }
  }
  DispatchInbox();
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
  fcntl(fd, F_SETFD, FD_CLOEXEC);
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (endpoint.size() >= sizeof(addr.sun_path)) {
    close(fd);
    return false;
  }
  std::strncpy(addr.sun_path, endpoint.c_str(), sizeof(addr.sun_path) - 1);
  // A unix-socket connect completes in the call; non-blocking from then on so
  // neither a full send buffer nor a quiet client can hold the pump thread.
  if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    return false;
  }
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
    close(fd);
    return false;
  }
  fd_ = fd;
  outbox_.clear();
  return true;
}

void DiscordPresence::Drop() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  outbox_.clear();
  state_ = State::kDisconnected;
  shown_valid_ = false;
  inbox_.clear();
  std::lock_guard lock(mutex_);
  pending_replies_.clear();
}

bool DiscordPresence::FlushOutbox() {
  if (fd_ < 0) {
    return false;
  }
  size_t sent = 0;
  while (sent < outbox_.size()) {
    const ssize_t n =
        send(fd_, outbox_.data() + sent, outbox_.size() - sent, MSG_NOSIGNAL | MSG_DONTWAIT);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      break;  // the rest waits for the next pump
    }
    outbox_.erase(outbox_.begin(), outbox_.begin() + static_cast<std::ptrdiff_t>(sent));
    return false;
  }
  outbox_.erase(outbox_.begin(), outbox_.begin() + static_cast<std::ptrdiff_t>(sent));
  return true;
}

void DiscordPresence::ReadAvailable() {
  if (fd_ < 0) {
    return;
  }
  for (;;) {
    uint8_t buf[4096];
    const ssize_t got = recv(fd_, buf, sizeof(buf), MSG_DONTWAIT);
    if (got > 0) {
      inbox_.insert(inbox_.end(), buf, buf + got);
      if (inbox_.size() > kMaxFrame + 8) {
        Drop();
        return;
      }
      continue;
    }
    if (got < 0 && errno == EINTR) {
      continue;
    }
    if (got < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      break;
    }
    DISCORD_INFO_ONCE(close_logged_, "discord: connection closed");
    Drop();
    return;
  }
  DispatchInbox();
}

#endif

}  // namespace rex::net
