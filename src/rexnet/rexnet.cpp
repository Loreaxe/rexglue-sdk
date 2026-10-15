/**
 * @file        rexnet.cpp
 * @brief       RexNet module lifecycle and per-frame event pump.
 *
 * @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
 *              All rights reserved.
 *
 * @license     BSD 3-Clause License
 *              See LICENSE file in the project root for full license text.
 */
#include "rex/net/rexnet.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string_view>
#include <utility>

#include <sstream>
#include <toml++/toml.hpp>

#include "rex/net/pseudonym.h"
#include "rex/ui/overlay/rexnet_overlay.h"

REXCVAR_DEFINE_BOOL(rexnet, true, "RexNet", "Enable the RexNet netplay module (when built in)");
REXCVAR_DEFINE_STRING(rexnet_display_name, "", "RexNet",
                      "Self-asserted display name shown to friends (max 32 bytes)");
REXCVAR_DEFINE_STRING(rexnet_bootstrap, "amino", "RexNet",
                      "DHT bootstrap: 'amino' (public IPFS set), 'none', or "
                      "comma-separated multiaddrs");
REXCVAR_DEFINE_BOOL(rexnet_syslink, false, "RexNet",
                    "Carry System Link traffic over RexNet instead of the "
                    "local network, giving LAN-only titles internet play. "
                    "Off by default so System Link stays what it is: a real "
                    "broadcast to the real LAN.");
REXCVAR_DEFINE_BOOL(rexnet_force_tunnel, false, "RexNet",
                    "Skip NAT hole punching and carry game traffic over the "
                    "control connection (the degraded path a CGNAT player "
                    "gets). For testing that path without a CGNAT.");
REXCVAR_DEFINE_UINT32(rexnet_listen_port, 0, "RexNet",
                      "Fixed control-plane (QUIC/TCP) listen port; 0 = ephemeral. "
                      "Set a stable port so a UPnP mapping or manual forward stays "
                      "valid across launches -- then the address a peer pastes for "
                      "a direct connect keeps working.");
REXCVAR_DEFINE_UINT32(rexnet_game_port, 0, "RexNet",
                      "Fixed game-plane UDP port; 0 = ephemeral. Pin it alongside "
                      "rexnet_listen_port when forwarding a port by hand.");

namespace rex::net {

RexNet* RexNet::shared_instance_ = nullptr;

std::unique_ptr<RexNet> RexNet::Create(const RexNetOptions& options) {
  const std::string data_dir = options.data_dir.string();

  std::vector<const char*> bootstrap;
  bootstrap.reserve(options.bootstrap.size());
  for (const auto& addr : options.bootstrap) {
    bootstrap.push_back(addr.c_str());
  }

  std::vector<const char*> relays;
  relays.reserve(options.relays.size());
  for (const auto& addr : options.relays) {
    relays.push_back(addr.c_str());
  }

  RexNetConfig cfg{};
  cfg.data_dir = data_dir.c_str();
  cfg.title_id = options.title_id;
  cfg.display_name = options.display_name.c_str();
  cfg.bootstrap = bootstrap.empty() ? nullptr : bootstrap.data();
  cfg.bootstrap_len = static_cast<uint32_t>(bootstrap.size());
  cfg.relays = relays.empty() ? nullptr : relays.data();
  cfg.relays_len = static_cast<uint32_t>(relays.size());
  cfg.use_default_bootstrap = options.use_default_bootstrap;
  if (!relays.empty()) {
    REXNET_INFO("{} relay(s) configured (§9: accelerant, not a dependency)", relays.size());
  }
  cfg.force_tunnel = options.force_tunnel;
  cfg.listen_port = options.listen_port;
  cfg.game_port = options.game_port;
  if (options.force_tunnel) {
    REXNET_WARN(
        "force_tunnel: hole punching disabled; all game traffic "
        "will take the degraded control-tunnel path");
  }

  // Route the Rust core's logs into ours before it starts: otherwise every
  // engine line (shard placement, punch, DHT) is discarded and the P2P
  // layer is a black box precisely when it misbehaves.
  rexnet_set_log_sink([](uint32_t level, const char* message) {
    if (!message) {
      return;
    }
    switch (level) {
      case REXNET_LOG_ERROR:
        REXNET_ERROR("core: {}", message);
        break;
      case REXNET_LOG_WARN:
        REXNET_WARN("core: {}", message);
        break;
      case REXNET_LOG_INFO:
        REXNET_INFO("core: {}", message);
        break;
      case REXNET_LOG_DEBUG:
        REXNET_DEBUG("core: {}", message);
        break;
      default:
        REXNET_TRACE("core: {}", message);
        break;
    }
  });

  RexNetHandle* handle = rexnet_init(&cfg);
  if (!handle) {
    REXNET_ERROR("rexnet-core initialization failed");
    return nullptr;
  }
  auto instance = std::unique_ptr<RexNet>(new RexNet(handle));
  instance->title_id_ = options.title_id;
  instance->xex_title_id_ = options.title_id;
  instance->data_dir_ = options.data_dir;
  instance->display_name_ = options.display_name;
  {
    std::lock_guard lock(instance->mutex_);
    instance->LoadFriendNamesLocked();
  }
  if (options.start_pump_thread) {
    instance->StartPumpThread();
  }
  return instance;
}

static ui::RexNetOverlayStatus BuildOverlayStatus() {
  auto* net = RexNet::shared();
  ui::RexNetOverlayStatus status;
  if (!net) {
    return status;
  }
  status.active = true;
  status.peer_id = RexNet::PeerIdString(net->local_peer_id());
  status.display_name = net->display_name();
  status.friend_code = RexNet::FriendCode(net->local_peer_id(), status.display_name);
  status.public_endpoints = net->PublicEndpoints();
  status.syslink_over_wan = REXCVAR_GET(rexnet_syslink);
  if (auto sid = net->current_session_id()) {
    status.session_active = true;
  }
  for (const auto& conn : net->SnapshotConnections()) {
    ui::RexNetOverlayStatus::Peer peer;
    peer.virtual_ip =
        fmt::format("10.77.{}.{}", (conn.virtual_ip >> 8) & 0xFF, conn.virtual_ip & 0xFF);
    switch (conn.status) {
      case ConnectStatus::kConnected:
        peer.status = conn.degraded ? "connected (tunnel)" : "connected";
        break;
      case ConnectStatus::kPending:
        peer.status = "punching";
        break;
      case ConnectStatus::kLost:
        peer.status = "lost";
        break;
      default:
        peer.status = conn.control_linked ? "linked" : "seen";
        break;
    }
    // Already resolved per §17.2 -- never re-derive a name here.
    peer.name = conn.display_name;
    peer.is_friend = conn.is_friend;
    status.peers.push_back(std::move(peer));
  }
  for (const auto& member : net->SnapshotShardMembers()) {
    ui::RexNetOverlayStatus::ShardMember m;
    m.name = member.display_name;  // already §17.2-resolved
    m.is_friend = member.is_friend;
    m.state = member.state;
    m.has_session = member.has_session;
    status.shard_members.push_back(std::move(m));
  }
  for (const auto& info : net->SnapshotFriends()) {
    ui::RexNetOverlayStatus::Friend f;
    f.peer_id = RexNet::PeerIdString(info.peer);
    f.name = info.display_name;
    f.online = info.online;
    f.title_id = info.title_id;
    f.title_name = info.title_name;
    status.friends.push_back(std::move(f));
  }
  status.friend_count = static_cast<uint32_t>(status.friends.size());
  for (const auto& request : net->PendingFriendRequests()) {
    ui::RexNetOverlayStatus::FriendRequest r;
    r.peer_id = RexNet::PeerIdString(request.peer);
    r.name = request.display_name;
    r.note = request.note;
    status.friend_requests.push_back(std::move(r));
  }
  for (const auto& invite : net->PendingInvites()) {
    ui::RexNetOverlayStatus::Invite inv;
    inv.peer_id = RexNet::PeerIdString(invite.peer);
    inv.title_id = invite.title_id;
    // §17.2 again: an invite may arrive from a non-friend, so their own
    // asserted name must not be rendered. (A friend *request* is the one
    // place a stranger's asserted name is shown -- you cannot decide on it
    // otherwise -- and it is labelled as self-asserted there.)
    if (net->IsFriend(invite.peer)) {
      if (auto presence = net->GetPresence(invite.peer)) {
        inv.name = presence->display_name;
      }
    } else {
      inv.name = Pseudonym(invite.peer);
    }
    status.invites.push_back(std::move(inv));
  }
  return status;
}

RexNet* RexNet::InitializeShared(const RexNetOptions& options) {
  if (shared_instance_) {
    return shared_instance_;
  }
  shared_instance_ = Create(options).release();

  // Wire the F6 diagnostics overlay (no-ops if the UI layer is absent).
  ui::SetRexNetStatusProvider(&BuildOverlayStatus);
  ui::RexNetOverlayActions actions;
  actions.set_syslink = [](bool over_wan) {
    REXCVAR_SET(rexnet_syslink, over_wan);
    REXNET_INFO("System Link routing set to {}", over_wan ? "over RexNet (WAN)" : "LAN only");
    // WAN needs the shard subnet to carry the redirected broadcasts. The
    // XNetStartup coupling only runs once, so enable it here too for a live
    // toggle. Turning it off leaves the shard as configured -- broadcasts
    // simply route back to the real LAN (xsocket reads the cvar per send).
    if (over_wan) {
      if (auto* net = RexNet::shared()) {
        net->EnableAmbientShard();
      }
    }
  };
  actions.connect_manual = [](const std::string& multiaddr) {
    if (auto* net = RexNet::shared()) {
      net->ConnectManual(multiaddr);
    }
  };
  actions.set_display_name = [](const std::string& name) {
    if (auto* net = RexNet::shared()) {
      net->SetDisplayName(name);
    }
  };
  actions.send_friend_request = [](const std::string& peer_str) {
    auto* net = RexNet::shared();
    if (!net) {
      return false;
    }
    // A pasted friend code also tells us what to call them until
    // presence arrives.
    if (auto parsed = RexNet::ParseFriendCode(peer_str)) {
      net->RememberFriendName(parsed->first, parsed->second);
      net->FriendRequest(parsed->first, /*note=*/"");
      return true;
    }
    auto peer = RexNet::PeerIdFromString(peer_str);
    if (!peer) {
      return false;
    }
    net->FriendRequest(*peer, /*note=*/"");
    return true;
  };
  actions.accept_friend_request = [](const std::string& peer_str) {
    if (auto* net = RexNet::shared()) {
      for (const auto& request : net->PendingFriendRequests()) {
        if (RexNet::PeerIdString(request.peer) == peer_str) {
          net->AcceptFriendRequest(request.peer);
          return;
        }
      }
    }
  };
  actions.dismiss_friend_request = [](const std::string& peer_str) {
    if (auto* net = RexNet::shared()) {
      for (const auto& request : net->PendingFriendRequests()) {
        if (RexNet::PeerIdString(request.peer) == peer_str) {
          net->DismissFriendRequest(request.peer);
          return;
        }
      }
    }
  };
  actions.remove_friend = [](const std::string& peer_str) {
    if (auto* net = RexNet::shared()) {
      for (const auto& info : net->SnapshotFriends()) {
        if (RexNet::PeerIdString(info.peer) == peer_str) {
          net->FriendRemove(info.peer);
          return;
        }
      }
    }
  };
  actions.invite_friend = [](const std::string& peer_str) {
    auto* net = RexNet::shared();
    if (!net || !net->current_session_id()) {
      return;
    }
    for (const auto& info : net->SnapshotFriends()) {
      if (RexNet::PeerIdString(info.peer) == peer_str) {
        net->SendInvite(info.peer);
        return;
      }
    }
  };
  actions.accept_invite = [](const std::string& peer_str) {
    if (auto* net = RexNet::shared()) {
      for (const auto& invite : net->PendingInvites()) {
        if (RexNet::PeerIdString(invite.peer) == peer_str) {
          net->AcceptInvite(invite);
          return;
        }
      }
    }
  };
  actions.decline_invite = [](const std::string& peer_str) {
    if (auto* net = RexNet::shared()) {
      for (const auto& invite : net->PendingInvites()) {
        if (RexNet::PeerIdString(invite.peer) == peer_str) {
          net->DeclineInvite(invite.peer);
          return;
        }
      }
    }
  };
  ui::SetRexNetOverlayActions(std::move(actions));
  return shared_instance_;
}

RexNet* RexNet::shared() {
  return shared_instance_;
}

void RexNet::DestroyShared() {
  delete shared_instance_;
  shared_instance_ = nullptr;
}

RexNet::RexNet(RexNetHandle* handle) : handle_(handle) {
  rexnet_local_peer_id(handle_, &local_peer_id_);
}

RexNet::~RexNet() {
  if (pump_thread_.joinable()) {
    pump_running_.store(false, std::memory_order_relaxed);
    pump_thread_.join();
  }
  // After the pump thread, which is the only caller of its Pump.
  discord_.reset();
  if (handle_) {
    rexnet_shutdown(handle_);
    handle_ = nullptr;
  }
}

void RexNet::SetStreamSinks(StreamAcceptSink accept, StreamDataSink data, StreamCloseSink close,
                            StreamConnectSink connect) {
  std::lock_guard lock(mutex_);
  stream_accept_sink_ = std::move(accept);
  stream_data_sink_ = std::move(data);
  stream_close_sink_ = std::move(close);
  stream_connect_sink_ = std::move(connect);
}

void RexNet::StreamConnect(uint32_t virtual_ip, uint16_t src_port, uint16_t dst_port) {
  rexnet_stream_connect(handle_, virtual_ip, src_port, dst_port);
}

void RexNet::StreamSend(uint64_t stream_id, const void* data, uint32_t len) {
  rexnet_stream_send(handle_, stream_id, static_cast<const uint8_t*>(data), len);
}

void RexNet::StreamClose(uint64_t stream_id) {
  rexnet_stream_close(handle_, stream_id);
}

void RexNet::SetDatagramSink(DatagramSink sink) {
  std::lock_guard lock(mutex_);
  datagram_sink_ = std::move(sink);
}

void RexNet::SetNotifySink(NotifySink sink) {
  std::lock_guard lock(mutex_);
  notify_sink_ = std::move(sink);
}

void RexNet::SetStatus(uint32_t virtual_ip, ConnectStatus status) {
  std::lock_guard lock(mutex_);
  statuses_[virtual_ip].status = status;
}

void RexNet::SetNoticeSink(NoticeSink sink) {
  std::lock_guard lock(mutex_);
  notice_sink_ = std::move(sink);
}

std::vector<RexNet::Notice> RexNet::TakeNotices() {
  std::lock_guard lock(mutex_);
  return std::exchange(notices_, {});
}

void RexNet::RaiseNotice(Notice notice) {
  NoticeSink sink;
  {
    std::lock_guard lock(mutex_);
    sink = notice_sink_;
    notices_.push_back(notice);
  }
  if (sink) {
    sink(notice);
  }
}

bool RexNet::Connect(uint32_t virtual_ip) {
  RexNetPeerId peer;
  {
    std::lock_guard lock(mutex_);
    auto found = virtual_ips_.Lookup(virtual_ip);
    if (!found) {
      return false;
    }
    peer = *found;
    auto it = statuses_.find(virtual_ip);
    if (it != statuses_.end() && it->second.status == ConnectStatus::kConnected) {
      return true;  // already up (punched or over the tunnel)
    }
    statuses_[virtual_ip] = PeerLink{ConnectStatus::kPending, false};
  }
  rexnet_connect_peer(handle_, &peer);
  // If the control connection already exists no PeerConnected event will
  // fire, so kick the punch here as well (idempotent in core).
  rexnet_punch_peer(handle_, &peer);
  return true;
}

ConnectStatus RexNet::GetConnectStatus(uint32_t virtual_ip) {
  std::lock_guard lock(mutex_);
  if (auto it = statuses_.find(virtual_ip); it != statuses_.end()) {
    return it->second.status;
  }
  return ConnectStatus::kIdle;
}

void RexNet::SendDatagram(uint32_t virtual_ip, uint16_t src_port, uint16_t dst_port,
                          const void* data, uint32_t len) {
  rexnet_send_datagram(handle_, virtual_ip, src_port, dst_port, static_cast<const uint8_t*>(data),
                       len,
                       /*reliable=*/false);
}

std::array<uint8_t, 16> RexNet::SessionCreate(bool is_public, uint8_t slots_total,
                                              uint8_t slots_open) {
  std::array<uint8_t, 16> id;
  uint64_t token = 0;
  {
    std::random_device rd;
    for (size_t i = 0; i < id.size(); i += 4) {
      uint32_t word = rd();
      std::memcpy(id.data() + i, &word, 4);
    }
    // The join token: what proves a joiner holds material the host handed
    // out. Seeded from the device, never 0 (0 = no token).
    std::mt19937_64 rng(rd());
    do {
      token = rng();
    } while (token == 0);
  }
  {
    std::lock_guard lock(mutex_);
    session_id_ = id;
    join_token_ = token;
    slots_total_ = slots_total;
    slots_open_ = slots_open;
    join_secret_dirty_ = true;
    // Advertise the session to friends so join-in-progress UIs (orbs)
    // light up: JOINABLE + XNKID flow from this KV into their
    // XFriendsCreateEnumerator records.
    RichValue rich;
    rich.is_string = false;
    rich.bytes.assign(id.begin(), id.end());
    rich_kvs_[kRichKeySessionId] = std::move(rich);
    PushRichLocked();
  }
  rexnet_session_create(handle_, id.data(), slots_total, slots_open, is_public);
  if (rexnet_session_set_join_token(handle_, token) != 0) {
    REXNET_WARN("session: the core refused the join token; joins by material will not resolve");
  }
  REXNET_INFO(
      "session created ({}, {}/{} slots) xnkid {:02X} {:02X} {:02X} {:02X} "
      "{:02X} {:02X} {:02X} {:02X}",
      is_public ? "public" : "private", slots_open, slots_total, id[0], id[1], id[2], id[3], id[4],
      id[5], id[6], id[7]);
  return id;
}

void RexNet::SessionDelete() {
  {
    std::lock_guard lock(mutex_);
    session_id_.reset();
    join_token_ = 0;
    slots_total_ = 0;
    slots_open_ = 0;
    join_secret_dirty_ = true;
    rich_kvs_.erase(kRichKeySessionId);
    PushRichLocked();
  }
  rexnet_session_set_join_token(handle_, 0);
  rexnet_session_delete(handle_);
}

uint64_t RexNet::join_token() {
  std::lock_guard lock(mutex_);
  return join_token_;
}

void RexNet::SetSessionSlots(uint32_t open, uint32_t total) {
  {
    std::lock_guard lock(mutex_);
    slots_open_ = std::min(open, total);
    slots_total_ = total;
  }
  rexnet_session_set_slots(handle_, std::min(open, total), total);
}

void RexNet::SessionSearch() {
  {
    std::lock_guard lock(mutex_);
    session_results_.clear();
  }
  rexnet_session_search(handle_);
}

std::vector<RexNet::SessionResult> RexNet::TakeSessionResults() {
  std::lock_guard lock(mutex_);
  return std::exchange(session_results_, {});
}

void RexNet::SetPresence(uint32_t title_id, uint8_t state, const void* rich, uint32_t rich_len) {
  {
    // Track the last-set state so rich-KV pushes don't revert it.
    std::lock_guard lock(mutex_);
    presence_state_ = state;
    if (title_id && title_id != title_id_) {
      title_id_ = title_id;
      join_secret_dirty_ = true;
    }
  }
  rexnet_set_presence(handle_, title_id, state, static_cast<const uint8_t*>(rich), rich_len);
}

void RexNet::FriendRequest(const RexNetPeerId& peer, const std::string& note) {
  rexnet_friend_request(handle_, &peer, note.c_str());
}

void RexNet::FriendAccept(const RexNetPeerId& peer) {
  rexnet_friend_accept(handle_, &peer);
}

void RexNet::FriendRemove(const RexNetPeerId& peer) {
  rexnet_friend_remove(handle_, &peer);
}

void RexNet::SendInvite(const RexNetPeerId& peer) {
  rexnet_send_invite(handle_, &peer);
}

void RexNet::InviteReply(const RexNetPeerId& peer, bool accept) {
  rexnet_invite_reply(handle_, &peer, accept);
}

std::vector<RexNet::FriendRequestInfo> RexNet::PendingFriendRequests() {
  std::lock_guard lock(mutex_);
  return friend_requests_;
}

std::vector<RexNet::InviteInfo> RexNet::PendingInvites() {
  std::lock_guard lock(mutex_);
  return invites_;
}

namespace {
bool SamePeer(const RexNetPeerId& a, const RexNetPeerId& b) {
  return a.len == b.len && !std::memcmp(a.bytes, b.bytes, a.len);
}
}  // namespace

void RexNet::AcceptFriendRequest(const RexNetPeerId& peer) {
  {
    std::lock_guard lock(mutex_);
    std::erase_if(friend_requests_,
                  [&](const FriendRequestInfo& r) { return SamePeer(r.peer, peer); });
  }
  FriendAccept(peer);
}

// --- Shard and join material -----------------------------------------------

namespace {

std::string Hex16(const std::array<uint8_t, 16>& id) {
  std::string hex;
  for (uint8_t byte : id) {
    char pair[3];
    std::snprintf(pair, sizeof(pair), "%02x", byte);
    hex += pair;
  }
  return hex;
}

bool ParseHex16(const std::string& text, std::array<uint8_t, 16>& out) {
  if (text.size() != 32) {
    return false;
  }
  for (size_t i = 0; i < 16; ++i) {
    unsigned value = 0;
    if (std::sscanf(text.c_str() + 2 * i, "%2x", &value) != 1) {
      return false;
    }
    out[i] = static_cast<uint8_t>(value);
  }
  return true;
}

/// A bare `host:port` as the external-address event states it, to the
/// multiaddr a manual dial takes. v6 hosts carry colons of their own, so the
/// port is the part after the LAST one.
std::string EndpointToMultiaddr(const std::string& endpoint, const std::string& peer) {
  const size_t colon = endpoint.rfind(':');
  if (colon == std::string::npos || colon + 1 >= endpoint.size()) {
    return {};
  }
  std::string host = endpoint.substr(0, colon);
  const std::string port = endpoint.substr(colon + 1);
  if (!host.empty() && host.front() == '[' && host.back() == ']') {
    host = host.substr(1, host.size() - 2);
  }
  const bool v6 = host.find(':') != std::string::npos;
  return std::string(v6 ? "/ip6/" : "/ip4/") + host + "/udp/" + port + "/quic-v1/p2p/" + peer;
}

/// Text join material, read and never written: REXJ3 with the join token,
/// REXJ2 without it. Both run past Discord's limit with a shard set.
constexpr const char* kJoinSecretVersion = "REXJ3";
constexpr const char* kJoinSecretVersionNoToken = "REXJ2";

/// Discord refuses a SET_ACTIVITY whose secrets.join is longer than this, and
/// the party with it: no Join button at all.
constexpr size_t kJoinSecretMax = 128;

/// REXJ4, the written form: bytes in base64url without padding. Byte 0 is
/// version << 4 | flags, then title (BE), peer, session, [shard], [endpoint],
/// token (BE). At most 95 bytes, 127 characters.
constexpr uint8_t kJoinV4 = 0x40;
constexpr uint8_t kJoinHasShard = 0x01;
constexpr uint8_t kJoinEndpointV4 = 0x02;
constexpr uint8_t kJoinEndpointV6 = 0x04;
/// The peer is the 32-byte ed25519 key; otherwise a length byte and the id.
constexpr uint8_t kJoinPeerEd25519 = 0x08;
/// An ed25519 peer id is this identity-multihash prefix and the key.
constexpr uint8_t kEd25519PeerPrefix[6] = {0x00, 0x24, 0x08, 0x01, 0x12, 0x20};

struct JoinMaterial {
  uint32_t title = 0;
  RexNetPeerId peer{};
  std::array<uint8_t, 16> session{};
  std::optional<std::array<uint8_t, 16>> shard;
  /// `host:port` as the external-address event states it; empty = none.
  std::string endpoint;
  uint64_t token = 0;
};

constexpr char kBase64Url[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

std::string Base64UrlEncode(const std::vector<uint8_t>& in) {
  std::string out;
  uint32_t acc = 0;
  int bits = 0;
  for (uint8_t byte : in) {
    acc = (acc << 8) | byte;
    bits += 8;
    while (bits >= 6) {
      bits -= 6;
      out += kBase64Url[(acc >> bits) & 0x3F];
    }
  }
  if (bits > 0) {
    out += kBase64Url[(acc << (6 - bits)) & 0x3F];
  }
  return out;
}

std::optional<std::vector<uint8_t>> Base64UrlDecode(const std::string& in) {
  std::vector<uint8_t> out;
  uint32_t acc = 0;
  int bits = 0;
  for (char c : in) {
    const char* at = std::strchr(kBase64Url, c);
    if (c == '\0' || at == nullptr) {
      return std::nullopt;
    }
    acc = (acc << 6) | static_cast<uint32_t>(at - kBase64Url);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<uint8_t>(acc >> bits));
    }
  }
  return out;
}

/// `a.b.c.d:port` or `[v6]:port` to its address bytes and port. An address
/// this cannot read (a v6 with an embedded v4) is left out of the material,
/// and the join finds the host through the DHT instead.
bool ParseEndpoint(const std::string& endpoint, bool& v6, std::array<uint8_t, 16>& addr,
                   uint16_t& port) {
  const size_t colon = endpoint.rfind(':');
  if (colon == std::string::npos || colon + 1 >= endpoint.size()) {
    return false;
  }
  unsigned long port_value = 0;
  {
    char* end = nullptr;
    port_value = std::strtoul(endpoint.c_str() + colon + 1, &end, 10);
    if (*end != '\0' || port_value == 0 || port_value > 0xFFFF) {
      return false;
    }
  }
  port = static_cast<uint16_t>(port_value);
  std::string host = endpoint.substr(0, colon);
  addr.fill(0);
  if (host.size() >= 2 && host.front() == '[' && host.back() == ']') {
    host = host.substr(1, host.size() - 2);
  }
  if (host.find(':') == std::string::npos) {
    v6 = false;
    unsigned a, b, c, d;
    char extra;
    if (std::sscanf(host.c_str(), "%u.%u.%u.%u%c", &a, &b, &c, &d, &extra) != 4 || a > 255 ||
        b > 255 || c > 255 || d > 255) {
      return false;
    }
    addr[0] = static_cast<uint8_t>(a);
    addr[1] = static_cast<uint8_t>(b);
    addr[2] = static_cast<uint8_t>(c);
    addr[3] = static_cast<uint8_t>(d);
    return true;
  }
  v6 = true;
  // Groups either side of a "::"; the gap between them is zeros.
  std::vector<uint16_t> head, tail;
  std::vector<uint16_t>* side = &head;
  size_t i = 0;
  if (host.compare(0, 2, "::") == 0) {
    side = &tail;
    i = 2;
  }
  while (i < host.size()) {
    size_t end = host.find(':', i);
    if (end == std::string::npos) {
      end = host.size();
    }
    const std::string group = host.substr(i, end - i);
    if (group.empty() || group.size() > 4 ||
        group.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos) {
      return false;
    }
    side->push_back(static_cast<uint16_t>(std::strtoul(group.c_str(), nullptr, 16)));
    i = end + 1;
    if (end + 1 < host.size() && host[end + 1] == ':') {
      if (side == &tail) {
        return false;
      }
      side = &tail;
      i = end + 2;
    }
  }
  const bool gap = side == &tail;
  if (head.size() + tail.size() > (gap ? 7u : 8u) || (!gap && head.size() != 8)) {
    return false;
  }
  for (size_t g = 0; g < head.size(); ++g) {
    addr[2 * g] = static_cast<uint8_t>(head[g] >> 8);
    addr[2 * g + 1] = static_cast<uint8_t>(head[g]);
  }
  for (size_t g = 0; g < tail.size(); ++g) {
    const size_t at = 16 - 2 * (tail.size() - g);
    addr[at] = static_cast<uint8_t>(tail[g] >> 8);
    addr[at + 1] = static_cast<uint8_t>(tail[g]);
  }
  return true;
}

std::string FormatEndpoint(bool v6, const uint8_t* addr, uint16_t port) {
  if (!v6) {
    return fmt::format("{}.{}.{}.{}:{}", addr[0], addr[1], addr[2], addr[3], port);
  }
  std::string host;
  for (int g = 0; g < 8; ++g) {
    host += fmt::format("{}{:x}", g ? ":" : "", (addr[2 * g] << 8) | addr[2 * g + 1]);
  }
  return fmt::format("[{}]:{}", host, port);
}

void PutBE(std::vector<uint8_t>& out, uint64_t value, int bytes) {
  for (int i = bytes - 1; i >= 0; --i) {
    out.push_back(static_cast<uint8_t>(value >> (8 * i)));
  }
}

std::string EncodeJoinMaterial(const JoinMaterial& m) {
  bool v6 = false;
  std::array<uint8_t, 16> addr{};
  uint16_t port = 0;
  const bool has_endpoint = !m.endpoint.empty() && ParseEndpoint(m.endpoint, v6, addr, port);
  const bool ed25519 =
      m.peer.len == 38 && std::memcmp(m.peer.bytes, kEd25519PeerPrefix, 6) == 0;

  std::vector<uint8_t> out;
  uint8_t head = kJoinV4;
  head |= m.shard ? kJoinHasShard : 0;
  head |= has_endpoint ? (v6 ? kJoinEndpointV6 : kJoinEndpointV4) : 0;
  head |= ed25519 ? kJoinPeerEd25519 : 0;
  out.push_back(head);
  PutBE(out, m.title, 4);
  if (ed25519) {
    out.insert(out.end(), m.peer.bytes + 6, m.peer.bytes + 38);
  } else {
    out.push_back(m.peer.len);
    out.insert(out.end(), m.peer.bytes, m.peer.bytes + m.peer.len);
  }
  out.insert(out.end(), m.session.begin(), m.session.end());
  if (m.shard) {
    out.insert(out.end(), m.shard->begin(), m.shard->end());
  }
  if (has_endpoint) {
    out.insert(out.end(), addr.begin(), addr.begin() + (v6 ? 16 : 4));
    PutBE(out, port, 2);
  }
  PutBE(out, m.token, 8);
  return Base64UrlEncode(out);
}

std::optional<JoinMaterial> DecodeJoinMaterial(const std::string& text) {
  const std::optional<std::vector<uint8_t>> bytes = Base64UrlDecode(text);
  if (!bytes || bytes->empty() || ((*bytes)[0] & 0xF0) != kJoinV4) {
    return std::nullopt;
  }
  const std::vector<uint8_t>& in = *bytes;
  const uint8_t head = in[0];
  size_t at = 1;
  auto take = [&](size_t n) -> const uint8_t* {
    if (at + n > in.size()) {
      return nullptr;
    }
    const uint8_t* p = in.data() + at;
    at += n;
    return p;
  };
  auto take_be = [&](int n, uint64_t& value) {
    const uint8_t* p = take(n);
    if (!p) {
      return false;
    }
    value = 0;
    for (int i = 0; i < n; ++i) {
      value = (value << 8) | p[i];
    }
    return true;
  };

  JoinMaterial m;
  uint64_t title = 0;
  if (!take_be(4, title)) {
    return std::nullopt;
  }
  m.title = static_cast<uint32_t>(title);
  if (head & kJoinPeerEd25519) {
    const uint8_t* key = take(32);
    if (!key) {
      return std::nullopt;
    }
    m.peer.len = 38;
    std::memcpy(m.peer.bytes, kEd25519PeerPrefix, 6);
    std::memcpy(m.peer.bytes + 6, key, 32);
  } else {
    const uint8_t* len = take(1);
    const uint8_t* id = len && *len <= sizeof(m.peer.bytes) ? take(*len) : nullptr;
    if (!id) {
      return std::nullopt;
    }
    m.peer.len = *len;
    std::memcpy(m.peer.bytes, id, *len);
  }
  const uint8_t* session = take(16);
  if (!session) {
    return std::nullopt;
  }
  std::memcpy(m.session.data(), session, 16);
  if (head & kJoinHasShard) {
    const uint8_t* shard = take(16);
    if (!shard) {
      return std::nullopt;
    }
    m.shard.emplace();
    std::memcpy(m.shard->data(), shard, 16);
  }
  if (head & (kJoinEndpointV4 | kJoinEndpointV6)) {
    const bool v6 = (head & kJoinEndpointV6) != 0;
    const uint8_t* addr = take(v6 ? 16 : 4);
    uint64_t port = 0;
    if (!addr || !take_be(2, port)) {
      return std::nullopt;
    }
    m.endpoint = FormatEndpoint(v6, addr, static_cast<uint16_t>(port));
  }
  if (!take_be(8, m.token) || at != in.size()) {
    return std::nullopt;
  }
  return m;
}

/// REXJ3|title|peer|session|shard|endpoint|token, REXJ2 without the token,
/// or the first cut's peer|title|session, as the host bridge still writes it.
bool ParseJoinText(const std::string& secret, JoinMaterial& m) {
  std::vector<std::string> parts;
  {
    std::stringstream in(secret);
    std::string part;
    while (std::getline(in, part, '|')) {
      parts.push_back(part);
    }
  }
  std::string peer_text, title_text, session_text, shard_text, endpoint_text, token_text;
  if (parts.size() == 7 && parts[0] == kJoinSecretVersion) {
    title_text = parts[1];
    peer_text = parts[2];
    session_text = parts[3];
    shard_text = parts[4];
    endpoint_text = parts[5];
    token_text = parts[6];
  } else if (parts.size() == 6 && parts[0] == kJoinSecretVersionNoToken) {
    title_text = parts[1];
    peer_text = parts[2];
    session_text = parts[3];
    shard_text = parts[4];
    endpoint_text = parts[5];
  } else if (parts.size() == 3) {
    peer_text = parts[0];
    title_text = parts[1];
    session_text = parts[2];
  } else {
    REXNET_WARN("join material not understood ({} field(s))", parts.size());
    return false;
  }

  if (std::sscanf(title_text.c_str(), "%x", &m.title) != 1) {
    return false;
  }
  if (!token_text.empty()) {
    unsigned long long parsed = 0;
    if (std::sscanf(token_text.c_str(), "%llx", &parsed) != 1) {
      REXNET_WARN("join material: bad token");
      return false;
    }
    m.token = parsed;
  }
  const std::optional<RexNetPeerId> peer = RexNet::PeerIdFromString(peer_text);
  if (!peer || !ParseHex16(session_text, m.session)) {
    REXNET_WARN("join material: bad peer or session id");
    return false;
  }
  m.peer = *peer;
  std::array<uint8_t, 16> shard{};
  if (!shard_text.empty() && shard_text != "-" && ParseHex16(shard_text, shard)) {
    m.shard = shard;
  }
  if (endpoint_text != "-") {
    m.endpoint = endpoint_text;
  }
  return true;
}

}  // namespace

void RexNet::SessionQuery(const RexNetPeerId& peer, uint64_t token) {
  rexnet_session_query(handle_, &peer, token);
}

void RexNet::ShardJoin(const std::array<uint8_t, 16>& shard_id) {
  rexnet_shard_join(handle_, shard_id.data());
}

std::optional<std::array<uint8_t, 16>> RexNet::current_shard_id() {
  std::lock_guard lock(mutex_);
  return current_shard_;
}

std::string RexNet::JoinSecret() {
  std::lock_guard lock(mutex_);
  return JoinSecretLocked();
}

std::string RexNet::JoinSecretLocked() {
  if (!join_secret_dirty_) {
    return join_secret_cache_;
  }
  join_secret_dirty_ = false;
  join_secret_cache_.clear();
  if (!session_id_.has_value()) {
    return join_secret_cache_;
  }
  JoinMaterial material;
  material.title = title_id_;
  material.peer = local_peer_id_;
  material.session = *session_id_;
  material.shard = current_shard_;
  material.endpoint = public_endpoints_.empty() ? std::string() : *public_endpoints_.begin();
  material.token = join_token_;
  std::string secret = EncodeJoinMaterial(material);
  // Only a peer id that is not ed25519 runs past the limit. The endpoint goes
  // first (the DHT finds the host without it), then the shard.
  if (secret.size() > kJoinSecretMax) {
    material.endpoint.clear();
    secret = EncodeJoinMaterial(material);
  }
  if (secret.size() > kJoinSecretMax) {
    material.shard.reset();
    secret = EncodeJoinMaterial(material);
  }
  join_secret_cache_ = std::move(secret);
  return join_secret_cache_;
}

bool RexNet::JoinFromSecret(const std::string& secret) {
  JoinMaterial material;
  if (secret.find('|') == std::string::npos) {
    std::optional<JoinMaterial> decoded = DecodeJoinMaterial(secret);
    if (!decoded) {
      REXNET_WARN("join material not understood");
      return false;
    }
    material = std::move(*decoded);
  } else if (!ParseJoinText(secret, material)) {
    return false;
  }

  uint32_t our_title = 0;
  uint32_t our_xex_title = 0;
  {
    std::lock_guard lock(mutex_);
    our_title = title_id_;
    our_xex_title = xex_title_id_;
  }
  if (material.title != our_title && material.title != our_xex_title) {
    REXNET_WARN("join refused: title {:08X} is not ours ({:08X})", material.title, our_title);
    return false;
  }
  InviteInfo invite;
  invite.peer = material.peer;
  invite.session_id = material.session;
  invite.title_id = material.title;
  const std::string peer_text = PeerIdString(material.peer);

  // The endpoint first: a confirmed public address dials in one step where
  // the DHT would take a lookup, and the invite's accept resolves on the host's
  // descriptor, which needs the connection.
  if (!material.endpoint.empty()) {
    const std::string multiaddr = EndpointToMultiaddr(material.endpoint, peer_text);
    if (!multiaddr.empty()) {
      ConnectManual(multiaddr);
    }
  }
  if (!AcceptInvite(invite)) {
    // Already on its way: the descriptor or the timeout resolves it, and a
    // second arm would run the title's join twice.
    REXNET_INFO("join to {}'s session already pending; not re-armed", peer_text);
    return true;
  }
  // The host is known: ask it outright, with the token that proves the
  // material came from it, and keep the DHT search as the fallback for a dial
  // that has not landed yet. Friendship is the players' business, not the
  // join's: nothing here sends a request.
  SessionQuery(material.peer, material.token);
  SessionSearch();
  if (material.shard) {
    ShardJoin(*material.shard);
  }
  REXNET_INFO("joining {}'s session from invite material{}", peer_text,
              material.token ? "" : " (no token: a private session will not answer)");
  return true;
}

// --- Discord Rich Presence ---------------------------------------------------

void RexNet::ConfigureDiscordLocked() {
  if (game_config_.discord_application_id.empty()) {
    discord_.reset();
    return;
  }
  DiscordPresence::Callbacks callbacks;
  callbacks.on_join = [this](const std::string& secret) {
    if (!JoinFromSecret(secret)) {
      REXNET_WARN("discord join: the invite could not be used");
    }
  };
  callbacks.on_join_request = [this](const std::string& user_id,
                                     const std::string& user_name) -> std::optional<bool> {
    std::lock_guard lock(mutex_);
    const bool joinable = discord_joinable_.value_or(true) && session_id_.has_value();
    if (!joinable) {
      return false;
    }
    switch (join_request_policy_) {
      case JoinRequestPolicy::All:
        return true;
      case JoinRequestPolicy::Friends:
        if (auto it = discord_user_peers_.find(user_id); it != discord_user_peers_.end()) {
          uint8_t key[20];
          OnlineKey(it->second, key);
          if (friends_.contains(std::string(reinterpret_cast<const char*>(key), 20))) {
            return true;
          }
        }
        [[fallthrough]];  // unknown to us: the host decides
      case JoinRequestPolicy::Ask:
        break;
    }
    join_requests_.push_back(
        JoinRequestEntry{JoinRequest{user_id, user_name}, std::chrono::steady_clock::now()});
    return std::nullopt;
  };
  DiscordPresence::Assets assets;
  assets.large_image = game_config_.discord_large_image;
  assets.large_text = game_config_.discord_large_text;
  discord_ = std::make_shared<DiscordPresence>(game_config_.discord_application_id,
                                               std::move(assets), std::move(callbacks));
  REXNET_INFO("discord rich presence on (application {})", game_config_.discord_application_id);
}

void RexNet::SetDiscordActivity(const std::string& details, const std::string& state,
                                std::optional<bool> joinable) {
  std::lock_guard lock(mutex_);
  discord_details_ = details;
  discord_state_ = state;
  discord_joinable_ = joinable;
}

void RexNet::SetDiscordEnabled(bool enabled) {
  std::lock_guard lock(mutex_);
  discord_enabled_ = enabled;
}

void RexNet::SetJoinRequestPolicy(JoinRequestPolicy policy) {
  std::lock_guard lock(mutex_);
  join_request_policy_ = policy;
}

RexNet::JoinRequestPolicy RexNet::join_request_policy() {
  std::lock_guard lock(mutex_);
  return join_request_policy_;
}

std::vector<RexNet::JoinRequest> RexNet::TakeJoinRequests() {
  std::lock_guard lock(mutex_);
  std::vector<JoinRequest> out;
  out.reserve(join_requests_.size());
  for (auto& entry : join_requests_) {
    out.push_back(std::move(entry.request));
  }
  join_requests_.clear();
  return out;
}

void RexNet::ReplyJoinRequest(const std::string& user_id, bool accept) {
  std::shared_ptr<DiscordPresence> discord;
  {
    std::lock_guard lock(mutex_);
    std::erase_if(join_requests_,
                  [&](const JoinRequestEntry& e) { return e.request.user_id == user_id; });
    discord = discord_;
  }
  if (discord) {
    discord->ReplyJoinRequest(user_id, accept);
  }
}

void RexNet::LinkDiscordUser(const std::string& user_id, const RexNetPeerId& peer) {
  std::lock_guard lock(mutex_);
  discord_user_peers_[user_id] = peer;
}

void RexNet::DiscordPump() {
  // The pump thread owns discord_; everything it reads is taken under the lock
  // and the Discord frames go out without it.
  std::shared_ptr<DiscordPresence> discord;
  DiscordPresence::Activity wanted;
  bool enabled;
  {
    std::lock_guard lock(mutex_);
    discord = discord_;
    if (!discord) {
      return;
    }
    enabled = discord_enabled_;
    wanted.details = !discord_details_.empty()    ? discord_details_
                     : !game_config_.discord_details.empty() ? game_config_.discord_details
                                                             : "Playing";
    wanted.state = !discord_state_.empty() ? discord_state_ : game_config_.discord_state;
    const bool joinable = discord_joinable_.value_or(true) && session_id_.has_value();
    if (joinable) {
      wanted.joinable = true;
      wanted.party_id = Hex16(*session_id_);
      wanted.join_secret = JoinSecretLocked();
      if (slots_total_) {
        wanted.party_size = static_cast<int>(slots_total_ - slots_open_);
        wanted.party_max = static_cast<int>(slots_total_);
      }
    }
  }
  if (!enabled) {
    discord->Close();
    return;
  }
  discord->SetActivity(wanted);
  discord->Pump();
}

std::vector<RexNetPeerId> RexNet::TakeDeliveredFriendRequests() {
  std::lock_guard lock(mutex_);
  return std::exchange(delivered_requests_, {});
}

std::vector<RexNetPeerId> RexNet::TakeDeclinedFriendRequests() {
  std::lock_guard lock(mutex_);
  return std::exchange(declined_requests_, {});
}

void RexNet::DismissFriendRequest(const RexNetPeerId& peer) {
  {
    std::lock_guard lock(mutex_);
    std::erase_if(friend_requests_,
                  [&](const FriendRequestInfo& r) { return SamePeer(r.peer, peer); });
  }
  // The core keeps inbound requests across runs; without this it would show
  // the dismissed one again at the next start.
  rexnet_friend_dismiss(handle_, &peer);
}

void RexNet::DeclineInvite(const RexNetPeerId& peer) {
  {
    std::lock_guard lock(mutex_);
    std::erase_if(invites_, [&](const InviteInfo& i) { return SamePeer(i.peer, peer); });
  }
  InviteReply(peer, false);
}

std::vector<RexNetPeerId> RexNet::GetFriends() {
  std::lock_guard lock(mutex_);
  std::vector<RexNetPeerId> out;
  out.reserve(friends_.size());
  for (auto& [key, peer] : friends_) {
    out.push_back(peer);
  }
  return out;
}

std::vector<RexNet::FriendInfo> RexNet::SnapshotFriends() {
  std::lock_guard lock(mutex_);
  std::vector<FriendInfo> out;
  out.reserve(friends_.size());
  for (auto& [key, peer] : friends_) {
    FriendInfo info;
    info.peer = peer;
    if (auto it = control_linked_.find(key); it != control_linked_.end()) {
      info.online = it->second;
    }
    if (auto it = presence_.find(key); it != presence_.end()) {
      info.display_name = it->second.display_name;
      info.title_id = it->second.title_id;
      info.title_name = it->second.title_name;
      info.has_session = it->second.has_session;
      info.session_id = it->second.session_id;
      info.presence_state = it->second.state;
    }
    // Presence wins; a name learned from a pasted friend code or the
    // request payload fills in until it arrives.
    if (info.display_name.empty()) {
      if (auto it = friend_names_.find(key); it != friend_names_.end()) {
        info.display_name = it->second;
      }
    }
    out.push_back(std::move(info));
  }
  return out;
}

std::optional<GameConfig> RexNet::LoadGameConfig(const std::filesystem::path& path) {
  try {
    auto table = toml::parse_file(path.string());
    GameConfig config;
    config.session_model = table["game"]["session_model"].value_or(std::string("private-invite"));
    config.game_channel = table["game"]["game_channel"].value_or(std::string("unreliable"));
    config.advertised_title_id =
        static_cast<uint32_t>(table["game"]["advertised_title_id"].value_or(int64_t(0)));
    // [network] — §9. Hosted infrastructure is opt-in per project; the SDK
    // ships no addresses, so nothing depends on anyone's servers by default.
    auto read_list = [&](const char* key, std::vector<std::string>& out) {
      if (auto arr = table["network"][key].as_array()) {
        for (auto& node : *arr) {
          if (auto entry = node.value<std::string>()) {
            out.push_back(*entry);
          }
        }
      }
    };
    read_list("relays", config.relays);
    read_list("bootstrap", config.bootstrap);

    // [shard] — §17.5. Absent section means disabled, so existing configs
    // keep their current behaviour untouched.
    config.shard_enabled = table["shard"]["enabled"].value_or(false);
    config.shard_cap = static_cast<uint16_t>(table["shard"]["cap"].value_or(int64_t(0)));
    config.shard_surface_as_friends = table["shard"]["surface_as_friends"].value_or(false);

    // [discord] — Rich Presence. Absent section means none, as with [shard].
    config.discord_application_id = table["discord"]["application_id"].value_or(std::string());
    config.discord_details = table["discord"]["details"].value_or(std::string());
    config.discord_state = table["discord"]["state"].value_or(std::string());
    config.discord_large_image = table["discord"]["large_image"].value_or(std::string());
    config.discord_large_text = table["discord"]["large_text"].value_or(std::string());

    if (auto rich = table["presence"]["rich"].as_table()) {
      for (auto& [name, node] : *rich) {
        auto* entry = node.as_table();
        if (!entry) {
          continue;
        }
        GameConfig::RichMapping mapping;
        mapping.context_id = static_cast<uint32_t>((*entry)["context"].value_or(int64_t(0)));
        mapping.key = static_cast<uint16_t>((*entry)["key"].value_or(int64_t(0)));
        mapping.is_string = (*entry)["type"].value_or(std::string("u32")) == "str";
        if (mapping.context_id && mapping.key) {
          config.presence_rich.push_back(mapping);
        } else {
          REXNET_WARN("rexnet.toml: rich mapping '{}' missing context/key",
                      std::string(name.str()));
        }
      }
    }
    return config;
  } catch (const toml::parse_error& error) {
    REXNET_ERROR("rexnet.toml parse failed: {}", error.description());
    return std::nullopt;
  }
}

void RexNet::SetGameConfig(GameConfig config) {
  std::lock_guard lock(mutex_);
  game_config_ = std::move(config);
  if (game_config_.advertised_title_id) {
    title_id_ = game_config_.advertised_title_id;
    join_secret_dirty_ = true;
    PushRichLocked();  // re-push presence under the advertised id
  }
  // §17.5: ambient shard is opt-in per title. Applied here rather than at
  // Create() because the config is loaded after the core is up.
  if (game_config_.shard_enabled) {
    rexnet_shard_enable(handle_, game_config_.shard_cap);
  } else {
    rexnet_shard_disable(handle_);
  }
  ConfigureDiscordLocked();
  REXNET_INFO(
      "game config: session_model={} game_channel={} rich mappings={} "
      "advertised_title_id={:08X} shard={}",
      game_config_.session_model, game_config_.game_channel, game_config_.presence_rich.size(),
      title_id_,
      game_config_.shard_enabled
          ? fmt::format("enabled(cap={})", game_config_.shard_cap ? game_config_.shard_cap : 255)
          : "disabled");
}

GameConfig RexNet::game_config() {
  std::lock_guard lock(mutex_);
  return game_config_;
}

void RexNet::EnableAmbientShard() {
  std::lock_guard lock(mutex_);
  // Reflect it in game_config_ so game_config() and later SetGameConfig calls
  // agree the shard is on, and pass the loaded cap through (0 => core's 255).
  game_config_.shard_enabled = true;
  rexnet_shard_enable(handle_, game_config_.shard_cap);
}

void RexNet::PushRichLocked() {
  // Blob: [count u8] then per KV [key u16 LE][type u8][len u8][bytes].
  std::vector<uint8_t> blob;
  blob.push_back(static_cast<uint8_t>(rich_kvs_.size()));
  for (auto& [key, value] : rich_kvs_) {
    blob.push_back(static_cast<uint8_t>(key));
    blob.push_back(static_cast<uint8_t>(key >> 8));
    blob.push_back(value.is_string ? 1 : 0);
    blob.push_back(static_cast<uint8_t>(value.bytes.size()));
    blob.insert(blob.end(), value.bytes.begin(), value.bytes.end());
  }
  rexnet_set_presence(handle_, title_id_, presence_state_, blob.data(),
                      static_cast<uint32_t>(blob.size()));
}

void RexNet::OnGuestContext(uint32_t context_id, uint32_t value) {
  std::lock_guard lock(mutex_);
  for (const auto& mapping : game_config_.presence_rich) {
    if (mapping.context_id != context_id || mapping.is_string) {
      continue;
    }
    RichValue rich;
    rich.is_string = false;
    rich.bytes = {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
                  static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 24)};
    rich_kvs_[mapping.key] = std::move(rich);
    PushRichLocked();
    return;
  }
}

void RexNet::OnGuestProperty(uint32_t property_id, const uint8_t* value, uint32_t len) {
  if (!value || !len) {
    return;
  }
  std::lock_guard lock(mutex_);
  for (const auto& mapping : game_config_.presence_rich) {
    if (mapping.context_id != property_id) {
      continue;
    }
    RichValue rich;
    rich.is_string = mapping.is_string;
    rich.bytes.assign(value, value + std::min<uint32_t>(len, 64));
    rich_kvs_[mapping.key] = std::move(rich);
    PushRichLocked();
    return;
  }
}

bool RexNet::AcceptInvite(const InviteInfo& invite) {
  {
    std::lock_guard lock(mutex_);
    if (pending_accept_ && SamePeer(pending_accept_->first, invite.peer) &&
        pending_accept_->second == invite.session_id) {
      return false;  // already armed for this host and session
    }
    pending_accept_ = {invite.peer, invite.session_id};
    pending_accept_since_ = std::chrono::steady_clock::now();
    accepted_invite_.reset();
    std::erase_if(invites_, [&](const InviteInfo& i) { return SamePeer(i.peer, invite.peer); });
  }
  InviteReply(invite.peer, true);
  return true;
}

std::optional<RexNet::AcceptedInvite> RexNet::accepted_invite() {
  std::lock_guard lock(mutex_);
  return accepted_invite_;
}

std::optional<RexNet::PresenceInfo> RexNet::GetPresence(const RexNetPeerId& peer) {
  uint8_t key[20];
  OnlineKey(peer, key);
  std::lock_guard lock(mutex_);
  if (auto it = presence_.find(std::string(reinterpret_cast<const char*>(key), 20));
      it != presence_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::optional<std::array<uint8_t, 16>> RexNet::current_session_id() {
  std::lock_guard lock(mutex_);
  return session_id_;
}

void RexNet::FillGuestXnAddr(uint8_t out[36], uint32_t virtual_ip, const RexNetPeerId& peer) {
  // XNADDR layout: ina(4) inaOnline(4) wPortOnline(2) abEnet(6) abOnline(20),
  // everything already guest (network/big) endian.
  const uint8_t ip_be[4] = {
      static_cast<uint8_t>(virtual_ip >> 24), static_cast<uint8_t>(virtual_ip >> 16),
      static_cast<uint8_t>(virtual_ip >> 8), static_cast<uint8_t>(virtual_ip)};
  std::memcpy(out + 0, ip_be, 4);
  std::memcpy(out + 4, ip_be, 4);
  out[8] = 3074 >> 8;  // wPortOnline, big endian
  out[9] = 3074 & 0xFF;
  EnetAddr(peer, out + 10);
  OnlineKey(peer, out + 16);
}

std::string RexNet::PeerIdString(const RexNetPeerId& peer) {
  char buf[64] = {};
  rexnet_peer_id_string(&peer, buf, sizeof(buf));
  return std::string(buf);
}

std::string RexNet::FriendCode(const RexNetPeerId& peer, const std::string& name) {
  char buf[128] = {};
  rexnet_friend_code(&peer, name.c_str(), buf, sizeof(buf));
  return std::string(buf);
}

std::optional<std::pair<RexNetPeerId, std::string>> RexNet::ParseFriendCode(const std::string& s) {
  RexNetPeerId peer{};
  char name[32] = {};
  if (!rexnet_friend_code_parse(s.c_str(), &peer, name, sizeof(name))) {
    return std::nullopt;
  }
  return std::make_pair(peer, std::string(name));
}

void RexNet::SetDisplayName(const std::string& name) {
  rexnet_set_display_name(handle_, name.c_str());
  std::filesystem::path path;
  {
    std::lock_guard lock(mutex_);
    display_name_ = name;
    path = data_dir_ / "display_name.txt";
  }
  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);
  if (FILE* f = std::fopen(path.string().c_str(), "wb")) {
    std::fwrite(name.data(), 1, name.size(), f);
    std::fclose(f);
  } else {
    REXNET_WARN("failed to persist display name to {}", path.string());
  }
}

std::string RexNet::display_name() {
  std::lock_guard lock(mutex_);
  return display_name_;
}

namespace {

/// A remembered friend name as friend_names.txt holds it: control characters
/// (newlines included -- the file is line-oriented) stripped, at most
/// kFriendNameMax bytes, cut on a UTF-8 boundary.
constexpr size_t kFriendNameMax = 64;
/// A line is `<base58 peer id> <name>\n`; peer ids are under 64 characters.
constexpr size_t kFriendNameLine = 64 + 1 + kFriendNameMax + 2;

std::string SanitizeFriendName(std::string_view name) {
  std::string out;
  out.reserve(std::min(name.size(), kFriendNameMax));
  for (const char c : name) {
    const auto byte = static_cast<unsigned char>(c);
    if (byte < 0x20 || byte == 0x7F) {
      continue;
    }
    out.push_back(c);
  }
  if (out.size() > kFriendNameMax) {
    size_t cut = kFriendNameMax;
    while (cut > 0 && (static_cast<unsigned char>(out[cut]) & 0xC0) == 0x80) {
      --cut;  // never split a multi-byte sequence
    }
    out.resize(cut);
  }
  return out;
}

}  // namespace

void RexNet::RememberFriendName(const RexNetPeerId& peer, const std::string& name) {
  const std::string clean = SanitizeFriendName(name);
  if (clean.empty()) {
    return;
  }
  uint8_t key[20];
  OnlineKey(peer, key);
  std::lock_guard lock(mutex_);
  friend_names_[std::string(reinterpret_cast<const char*>(key), 20)] = clean;
  SaveFriendNamesLocked();
}

void RexNet::SetGameTitleName(const std::string& name) {
  std::lock_guard lock(mutex_);
  RichValue rich;
  rich.is_string = true;
  const size_t n = std::min<size_t>(name.size(), 64);
  rich.bytes.assign(name.begin(), name.begin() + n);
  rich_kvs_[kRichKeyTitleName] = std::move(rich);
  PushRichLocked();
}

// friend_names.txt: one "<base58 peer id> <name>" per line; the name is
// sanitized on every path that stores one (SanitizeFriendName), so a newline
// can never make one entry read as two.
void RexNet::SaveFriendNamesLocked() {
  const auto path = data_dir_ / "friend_names.txt";
  FILE* f = std::fopen(path.string().c_str(), "wb");
  if (!f) {
    return;
  }
  for (const auto& [key, name] : friend_names_) {
    RexNetPeerId peer{};
    // The online key is the truncated multihash; only entries whose
    // peer id we still know (friends_ or vip table) can be rendered.
    auto it = friends_.find(key);
    if (it != friends_.end()) {
      peer = it->second;
    } else if (auto vip =
                   virtual_ips_.FindByOnlineKey(reinterpret_cast<const uint8_t*>(key.data()))) {
      if (auto p = virtual_ips_.Lookup(*vip)) {
        peer = *p;
      }
    }
    if (!peer.len) {
      continue;
    }
    const std::string id = PeerIdString(peer);
    const std::string clean = SanitizeFriendName(name);
    if (clean.empty()) {
      continue;
    }
    std::fprintf(f, "%s %s\n", id.c_str(), clean.c_str());
  }
  std::fclose(f);
}

void RexNet::LoadFriendNamesLocked() {
  const auto path = data_dir_ / "friend_names.txt";
  FILE* f = std::fopen(path.string().c_str(), "rb");
  if (!f) {
    return;
  }
  char line[kFriendNameLine];
  while (std::fgets(line, sizeof(line), f)) {
    std::string_view view(line);
    const bool whole = !view.empty() && view.back() == '\n';
    if (!whole) {
      // Longer than any line this file writes: skip the rest of it rather
      // than read its tail as another entry.
      int c;
      while ((c = std::fgetc(f)) != EOF && c != '\n') {
      }
    }
    while (!view.empty() && (view.back() == '\n' || view.back() == '\r')) {
      view.remove_suffix(1);
    }
    const size_t space = view.find(' ');
    if (space == std::string_view::npos) {
      continue;
    }
    auto peer = PeerIdFromString(std::string(view.substr(0, space)));
    if (!peer) {
      continue;
    }
    const std::string name = SanitizeFriendName(view.substr(space + 1));
    if (name.empty()) {
      continue;
    }
    uint8_t key[20];
    OnlineKey(*peer, key);
    friend_names_[std::string(reinterpret_cast<const char*>(key), 20)] = name;
  }
  std::fclose(f);
}

std::optional<RexNetPeerId> RexNet::PeerIdFromString(const std::string& s) {
  RexNetPeerId peer{};
  if (!rexnet_peer_id_parse(s.c_str(), &peer)) {
    return std::nullopt;
  }
  return peer;
}

uint64_t RexNet::XuidFromPeer(const RexNetPeerId& peer) {
  uint64_t xuid = 0xE000000000000000ull;
  if (peer.len >= 9) {
    uint64_t tail = 0;
    std::memcpy(&tail, peer.bytes + 2, 7);  // skip constant multihash prefix
    xuid |= tail & 0x00FFFFFFFFFFFFFFull;
  }
  return xuid;
}

// Accept the everyday case -- a bare "host:port" a peer read off their overlay
// -- as well as a full libp2p multiaddr for power users. A leading '/' means it
// is already a multiaddr; otherwise build a QUIC dial address, since that is
// what the control plane listens on. IPv6 must be bracketed ("[::1]:47100") so
// the port stays unambiguous. Returns empty on malformed input.
static std::string NormalizeConnectTarget(const std::string& input) {
  std::string s = input;
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
    s.erase(s.begin());
  }
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
    s.pop_back();
  }
  if (s.empty()) {
    return {};
  }
  if (s.front() == '/') {
    return s;  // already a multiaddr; pass through untouched
  }

  std::string host;
  std::string port;
  if (s.front() == '[') {
    const auto close = s.find(']');
    if (close == std::string::npos || close + 1 >= s.size() || s[close + 1] != ':') {
      return {};
    }
    host = s.substr(1, close - 1);
    port = s.substr(close + 2);
  } else {
    const auto colon = s.rfind(':');
    if (colon == std::string::npos) {
      return {};
    }
    host = s.substr(0, colon);
    port = s.substr(colon + 1);
  }
  if (host.empty() || port.empty()) {
    return {};
  }
  for (const char c : port) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      return {};
    }
  }

  // "[...]" or a colon in the host marks IPv6; digits-and-dots marks IPv4;
  // anything else is a DNS name libp2p resolves at dial time.
  const char* proto;
  if (input.find('[') != std::string::npos || host.find(':') != std::string::npos) {
    proto = "/ip6/";
  } else if (host.find_first_not_of("0123456789.") == std::string::npos) {
    proto = "/ip4/";
  } else {
    proto = "/dns/";
  }
  return std::string(proto) + host + "/udp/" + port + "/quic-v1";
}

void RexNet::ConnectManual(const std::string& target) {
  const std::string multiaddr = NormalizeConnectTarget(target);
  if (multiaddr.empty()) {
    REXNET_WARN("connect: '{}' is neither a host:port nor a multiaddr", target);
    return;
  }
  REXNET_INFO("connect: dialing {}", multiaddr);
  rexnet_connect_manual(handle_, multiaddr.c_str());
}

bool RexNet::IsFriend(const RexNetPeerId& peer) {
  std::lock_guard lock(mutex_);
  uint8_t key[20];
  OnlineKey(peer, key);
  return friends_.contains(std::string(reinterpret_cast<const char*>(key), 20));
}

std::vector<RexNet::ShardMember> RexNet::SnapshotShardMembers() {
  // Peers beat every PRESENCE_HEARTBEAT (30 s) in rexnet-core. There is no
  // leave message -- a peer that crashes or walks out simply stops -- so
  // silence is the only liveness signal we get. Three missed beats before
  // dropping tolerates ordinary gossip jitter without leaving ghost orbs
  // standing around for minutes.
  constexpr auto kLiveness = std::chrono::seconds(95);
  const auto now = std::chrono::steady_clock::now();

  std::lock_guard lock(mutex_);
  std::erase_if(shard_members_,
                [&](const auto& entry) { return now - entry.second.last_seen > kLiveness; });

  std::vector<ShardMember> out;
  out.reserve(shard_members_.size());
  for (const auto& [key, entry] : shard_members_) {
    out.push_back(entry.member);
  }
  // Freshest first, then by name so the order is stable frame to frame
  // rather than following the hash table.
  std::sort(out.begin(), out.end(), [](const ShardMember& a, const ShardMember& b) {
    if (a.is_friend != b.is_friend) {
      return a.is_friend;  // friends at the top
    }
    return a.display_name < b.display_name;
  });
  return out;
}

std::vector<RexNet::Connection> RexNet::SnapshotConnections() {
  std::lock_guard lock(mutex_);
  std::vector<Connection> out;
  for (const auto& [vip, peer] : virtual_ips_.Entries()) {
    Connection conn;
    conn.virtual_ip = vip;
    conn.peer = peer;
    if (auto it = statuses_.find(vip); it != statuses_.end()) {
      conn.status = it->second.status;
      conn.degraded = it->second.degraded;
    }
    uint8_t key[20];
    OnlineKey(peer, key);
    const std::string online(reinterpret_cast<const char*>(key), 20);
    if (auto it = control_linked_.find(online); it != control_linked_.end()) {
      conn.control_linked = it->second;
    }
    conn.is_friend = friends_.contains(online);
    conn.is_relay = relays_.contains(online);
    // §17.2 display rule. A non-friend is never shown their own asserted
    // name -- we substitute a pseudonym even if presence carried one. The
    // core is also expected not to send names to non-friends (§17.3.8);
    // ignoring it here means one missed check upstream cannot leak a name
    // into the UI.
    if (conn.is_friend) {
      if (auto it = presence_.find(online); it != presence_.end()) {
        conn.display_name = it->second.display_name;
      }
      if (conn.display_name.empty()) {
        if (auto it = friend_names_.find(online); it != friend_names_.end()) {
          conn.display_name = it->second;
        }
      }
    } else if (conn.is_relay) {
      // §17.2: a relaying non-friend is surfaced by role, not identity.
      // No pseudonym either -- infrastructure, not a player.
      conn.display_name = "relay";
    } else {
      conn.display_name = Pseudonym(peer);
    }
    out.push_back(std::move(conn));
  }

  // Disambiguate colliding pseudonyms (§17.3.7). Ordinals are assigned in
  // peer-id order, not iteration order, so a given peer keeps the same
  // decorated name across snapshots instead of shuffling as the table
  // rehashes.
  std::unordered_map<std::string, std::vector<size_t>> by_name;
  for (size_t i = 0; i < out.size(); ++i) {
    if (!out[i].is_friend && !out[i].is_relay) {
      by_name[out[i].display_name].push_back(i);
    }
  }
  for (auto& [name, indices] : by_name) {
    if (indices.size() < 2) {
      continue;
    }
    std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
      const auto& pa = out[a].peer;
      const auto& pb = out[b].peer;
      // len arrives over the wire and may exceed the buffer; clamp
      // before comparing or memcmp reads past bytes[].
      const size_t la = std::min<size_t>(pa.len, sizeof(pa.bytes));
      const size_t lb = std::min<size_t>(pb.len, sizeof(pb.bytes));
      const int cmp = std::memcmp(pa.bytes, pb.bytes, std::min(la, lb));
      return cmp != 0 ? cmp < 0 : la < lb;
    });
    for (size_t ordinal = 0; ordinal < indices.size(); ++ordinal) {
      out[indices[ordinal]].display_name =
          PseudonymWithDiscriminator(out[indices[ordinal]].peer, static_cast<uint32_t>(ordinal));
    }
  }
  return out;
}

void RexNet::FillGuestSessionInfo(uint8_t out[60], const std::array<uint8_t, 16>& session_id,
                                  uint32_t host_vip, const RexNetPeerId& host) {
  std::memcpy(out + 0, session_id.data(), 8);           // XNKID sessionID
  FillGuestXnAddr(out + 8, host_vip, host);             // XNADDR hostAddress
  std::memcpy(out + 8 + 36, session_id.data() + 8, 8);  // XNKEY (first half)
  std::memset(out + 8 + 36 + 8, 0, 8);                  // XNKEY (rest)
}

void RexNet::OnlineKey(const RexNetPeerId& peer, uint8_t out[20]) {
  std::memset(out, 0, 20);
  std::memcpy(out, peer.bytes, std::min<size_t>(peer.len, 20));
}

void RexNet::EnetAddr(const RexNetPeerId& peer, uint8_t out[6]) {
  // Stable pseudo-MAC: skip the constant multihash prefix (0x00 0x24).
  std::memset(out, 0xCC, 6);
  if (peer.len >= 8) {
    std::memcpy(out, peer.bytes + 2, 6);
  }
  out[0] &= 0xFE;  // clear multicast bit
  out[0] |= 0x02;  // locally administered
}

std::optional<std::pair<RexNetPeerId, std::array<uint8_t, 16>>> RexNet::FindFriendSessionByXnkid(
    const uint8_t xnkid[8]) {
  std::lock_guard lock(mutex_);
  for (const auto& [key, peer] : friends_) {
    auto it = presence_.find(key);
    if (it == presence_.end() || !it->second.has_session) {
      continue;
    }
    if (!std::memcmp(it->second.session_id.data(), xnkid, 8)) {
      return std::make_pair(peer, it->second.session_id);
    }
  }
  return std::nullopt;
}

std::optional<uint32_t> RexNet::VipFromOnlineKey(const uint8_t key[20]) {
  std::lock_guard lock(mutex_);
  return virtual_ips_.FindByOnlineKey(key);
}

uint32_t RexNet::PeerRttMs(uint32_t virtual_ip) const {
  std::lock_guard lock(mutex_);
  auto it = peer_rtt_ms_.find(virtual_ip);
  return it == peer_rtt_ms_.end() ? 0 : it->second;
}

std::optional<RexNetPeerId> RexNet::PeerFromVip(uint32_t virtual_ip) {
  std::lock_guard lock(mutex_);
  return virtual_ips_.Lookup(virtual_ip);
}

void RexNet::StartPumpThread() {
  pump_running_.store(true, std::memory_order_relaxed);
  pump_thread_ = std::thread([this] {
    // Woken by the core on every event; the timeout only bounds how long a
    // stop request can go unnoticed.
    while (pump_running_.load(std::memory_order_relaxed)) {
      rexnet_wait_event(handle_, 16);
      Pump();
    }
  });
}

void RexNet::Pump() {
  RexNetEvent event;
  while (rexnet_poll_event(handle_, &event)) {
    HandleEvent(event);
  }
  const auto now = std::chrono::steady_clock::now();
  Housekeep(now);
  // Discord's IPC is not delivery work: at most 4 Hz, never per event wake.
  if (now - discord_last_pump_ >= kDiscordPumpInterval) {
    discord_last_pump_ = now;
    DiscordPump();
  }
}

void RexNet::Housekeep(std::chrono::steady_clock::time_point now) {
  std::optional<Notice> timed_out;
  {
    std::lock_guard lock(mutex_);
    if (pending_accept_ && now - pending_accept_since_ > kAcceptTimeout) {
      Notice notice;
      notice.kind = Notice::Kind::kJoinTimedOut;
      notice.peer = pending_accept_->first;
      notice.message = "the host did not answer";
      timed_out = std::move(notice);
      pending_accept_.reset();
    }
    std::erase_if(join_requests_, [&](const JoinRequestEntry& e) {
      return now - e.received > kJoinRequestLifetime;
    });
    std::erase_if(invites_, [&](const InviteInfo& i) { return now >= i.expires; });
  }
  if (timed_out) {
    REXNET_WARN("join to {}'s session: the host did not answer within {} s",
                PeerIdString(timed_out->peer), kAcceptTimeout.count());
    RaiseNotice(std::move(*timed_out));
  }
}

void RexNet::HandleEvent(const RexNetEvent& event) {
  switch (event.kind) {
    case REXNET_EVENT_PEER_CONNECTED: {
      bool pending = false;
      {
        std::lock_guard lock(mutex_);
        virtual_ips_.Insert(event.peer, event.virtual_ip);
        uint8_t key[20];
        OnlineKey(event.peer, key);
        control_linked_[std::string(reinterpret_cast<const char*>(key), 20)] = true;
        auto it = statuses_.find(event.virtual_ip);
        pending = it != statuses_.end() && it->second.status == ConnectStatus::kPending;
      }
      // Control connection is up; XNET_CONNECT_STATUS_CONNECTED waits
      // for the game plane, so kick the punch now (§8.4). Also punch
      // for inbound connections we never called Connect() on — the
      // remote side is about to do the same and duplicates are cheap.
      if (pending) {
        rexnet_punch_peer(handle_, &event.peer);
      }
      break;
    }
    case REXNET_EVENT_PUNCH_RESULT: {
      std::lock_guard lock(mutex_);
      if (auto vip = virtual_ips_.Find(event.peer)) {
        PeerLink& link = statuses_[*vip];
        if (event.flag) {
          link = PeerLink{ConnectStatus::kConnected, false};
        } else if (link.status != ConnectStatus::kConnected) {
          // A failed punch is not a lost peer: the core follows it with
          // DEGRADED when the tunnel carries the peer, or DISCONNECTED when
          // nothing does. Until then the link stays pending.
          link = PeerLink{ConnectStatus::kPending, false};
        }
      }
      break;
    }
    case REXNET_EVENT_DEGRADED: {
      // The peer is reachable through the control tunnel: CONNECTED, so the
      // title sends, flagged so the overlay can say which path it is.
      std::lock_guard lock(mutex_);
      if (auto vip = virtual_ips_.Find(event.peer)) {
        statuses_[*vip] = PeerLink{ConnectStatus::kConnected, true};
        REXNET_INFO("10.77.{}.{} reachable over the tunnel (punch failed)", (*vip >> 8) & 0xFF,
                    *vip & 0xFF);
      } else {
        REXNET_DEBUG("degraded event for a peer without a virtual ip");
      }
      break;
    }
    case REXNET_EVENT_PEER_DISCONNECTED: {
      std::lock_guard lock(mutex_);
      if (auto vip = virtual_ips_.Find(event.peer)) {
        statuses_[*vip] = PeerLink{ConnectStatus::kLost, false};
      }
      uint8_t key[20];
      OnlineKey(event.peer, key);
      control_linked_[std::string(reinterpret_cast<const char*>(key), 20)] = false;
      break;
    }
    case REXNET_EVENT_DATAGRAM: {
      DatagramSink sink;
      {
        std::lock_guard lock(mutex_);
        sink = datagram_sink_;
      }
      if (!sink ||
          !sink(event.virtual_ip, event.src_port, event.port, event.data, event.data_len)) {
        REXNET_DEBUG(
            "inbound datagram dropped (no bound guest socket "
            "for port {})",
            static_cast<uint32_t>(event.port));
      }
      break;
    }
    case REXNET_EVENT_FRIEND_REQUEST: {
      FriendRequestInfo info;
      info.peer = event.peer;
      // data = [name_len u8][name][note]
      if (event.data_len >= 1) {
        const uint8_t name_len =
            std::min<uint8_t>(event.data[0], static_cast<uint8_t>(event.data_len - 1));
        info.display_name.assign(reinterpret_cast<const char*>(event.data + 1), name_len);
        info.note.assign(reinterpret_cast<const char*>(event.data + 1 + name_len),
                         event.data_len - 1 - name_len);
      }
      info.display_name = SanitizeFriendName(info.display_name);
      // The name a request carries labels its SENDER and nobody else: it is
      // keyed by the peer the core says the request came from, never by
      // anything in the payload.
      uint8_t key[20];
      OnlineKey(event.peer, key);
      {
        std::lock_guard lock(mutex_);
        if (!info.display_name.empty()) {
          friend_names_[std::string(reinterpret_cast<const char*>(key), 20)] = info.display_name;
        }
        // One pending request per peer; a repeat replaces the older.
        std::erase_if(friend_requests_,
                      [&](const FriendRequestInfo& r) { return SamePeer(r.peer, event.peer); });
        friend_requests_.push_back(std::move(info));
      }
      break;
    }
    case REXNET_EVENT_FRIEND_ACCEPTED: {
      uint8_t key[20];
      OnlineKey(event.peer, key);
      std::lock_guard lock(mutex_);
      friends_[std::string(reinterpret_cast<const char*>(key), 20)] = event.peer;
      // Crossed requests auto-accept in the core; drop any pending
      // inbound request from this (now) friend.
      std::erase_if(friend_requests_,
                    [&](const FriendRequestInfo& r) { return SamePeer(r.peer, event.peer); });
      // A name learned from their request/code can be rendered now
      // that the peer id is in friends_.
      SaveFriendNamesLocked();
      break;
    }
    case REXNET_EVENT_FRIEND_REMOVED: {
      uint8_t key[20];
      OnlineKey(event.peer, key);
      std::lock_guard lock(mutex_);
      friends_.erase(std::string(reinterpret_cast<const char*>(key), 20));
      break;
    }
    case REXNET_EVENT_SHARD_JOINED:
    case REXNET_EVENT_SHARD_LEFT: {
      std::lock_guard lock(mutex_);
      if (event.kind == REXNET_EVENT_SHARD_JOINED && event.data_len >= 16) {
        std::array<uint8_t, 16> id{};
        std::memcpy(id.data(), event.data, 16);
        current_shard_ = id;
      } else {
        current_shard_.reset();
      }
      join_secret_dirty_ = true;
      break;
    }
    case REXNET_EVENT_FRIEND_DELIVERED: {
      REXNET_INFO("friend request delivered to {}", PeerIdString(event.peer));
      std::lock_guard lock(mutex_);
      delivered_requests_.push_back(event.peer);
      break;
    }
    case REXNET_EVENT_FRIEND_DECLINED: {
      REXNET_INFO("friend request declined by {}", PeerIdString(event.peer));
      std::lock_guard lock(mutex_);
      declined_requests_.push_back(event.peer);
      break;
    }
    case REXNET_EVENT_INVITE_RECEIVED: {
      InviteInfo info;
      info.peer = event.peer;
      info.title_id = event.virtual_ip;  // repurposed: title id
      if (event.data_len >= 16) {
        std::memcpy(info.session_id.data(), event.data, 16);
      }
      info.expires = std::chrono::steady_clock::now() + kInviteLifetime;
      std::lock_guard lock(mutex_);
      // One pending invite per peer: a newer one replaces the older, which
      // named a session that may no longer exist.
      std::erase_if(invites_, [&](const InviteInfo& i) { return SamePeer(i.peer, event.peer); });
      invites_.push_back(std::move(info));
      break;
    }
    case REXNET_EVENT_INVITE_FAILED: {
      // The invite we sent did not reach the peer (no route, old build, or
      // they are gone); the title tells the player.
      std::string why(reinterpret_cast<const char*>(event.data), event.data_len);
      REXNET_WARN("invite to {} failed{}{}", PeerIdString(event.peer), why.empty() ? "" : ": ",
                  why);
      Notice notice;
      notice.kind = Notice::Kind::kInviteFailed;
      notice.peer = event.peer;
      notice.message = why.empty() ? "the invite could not be delivered" : why;
      RaiseNotice(std::move(notice));
      break;
    }
    case REXNET_EVENT_INVITE_REPLIED:
      // Deliberately no XN_LIVE_INVITE_ACCEPTED here: that notification tells
      // a title the *local* user accepted an invite and should now join. This
      // is the far side answering an invite we sent, so raising it would send
      // us to join our own session. The accept path is handled where the
      // session descriptor arrives.
      REXNET_INFO("invite {}", event.flag ? "accepted" : "declined");
      break;
    case REXNET_EVENT_PRESENCE_UPDATED: {
      PresenceInfo info;
      info.title_id = event.virtual_ip;  // repurposed: title id
      info.state = event.flag;
      // data = [name_len u8][name][rich]
      if (event.data_len >= 1) {
        const uint8_t name_len =
            std::min<uint8_t>(event.data[0], static_cast<uint8_t>(event.data_len - 1));
        info.display_name.assign(reinterpret_cast<const char*>(event.data + 1), name_len);
        info.rich.assign(event.data + 1 + name_len, event.data + event.data_len);
      }
      // Rich blob: [count u8] then per KV [key u16 LE][type][len][bytes];
      // the reserved title-name key carries the friend's game name.
      if (!info.rich.empty()) {
        const uint8_t* p = info.rich.data();
        const uint8_t* end = p + info.rich.size();
        uint8_t count = *p++;
        while (count-- && p + 4 <= end) {
          const uint16_t key = static_cast<uint16_t>(p[0] | (p[1] << 8));
          const bool is_string = p[2] != 0;
          const uint8_t len = p[3];
          p += 4;
          if (p + len > end) {
            break;
          }
          if (key == kRichKeyTitleName && is_string) {
            info.title_name.assign(reinterpret_cast<const char*>(p), len);
          } else if (key == kRichKeySessionId && !is_string && len == 16) {
            std::memcpy(info.session_id.data(), p, 16);
            info.has_session = true;
          }
          p += len;
        }
      }
      uint8_t key[20];
      OnlineKey(event.peer, key);
      NotifySink sink;
      {
        std::lock_guard lock(mutex_);
        presence_[std::string(reinterpret_cast<const char*>(key), 20)] = std::move(info);
        sink = notify_sink_;
      }
      if (sink) {
        sink(kXNotifyFriendsPresenceChanged, 0);
      }
      break;
    }
    case REXNET_EVENT_STREAM_OPENED: {
      if (event.data_len < 8) {
        break;
      }
      uint64_t stream_id = 0;
      for (int i = 0; i < 8; ++i) {
        stream_id = (stream_id << 8) | event.data[i];
      }
      const bool outbound = event.flag != 0;
      StreamAcceptSink accept;
      StreamConnectSink connect;
      {
        std::lock_guard lock(mutex_);
        accept = stream_accept_sink_;
        connect = stream_connect_sink_;
      }
      if (outbound) {
        // Our own connect completing.
        if (connect) {
          connect(event.virtual_ip, event.src_port, stream_id);
        }
      } else if (accept) {
        // Inbound: port is the guest port being listened on.
        if (!accept(stream_id, event.virtual_ip, event.port, event.src_port)) {
          // Nothing listening there; do not leave the peer waiting.
          REXNET_DEBUG("guest TCP: no listener on port {}, closing stream",
                       static_cast<uint16_t>(event.port));
          StreamClose(stream_id);
        }
      }
      break;
    }
    case REXNET_EVENT_STREAM_DATA: {
      if (event.data_len < 8) {
        break;
      }
      uint64_t stream_id = 0;
      for (int i = 0; i < 8; ++i) {
        stream_id = (stream_id << 8) | event.data[i];
      }
      StreamDataSink sink;
      {
        std::lock_guard lock(mutex_);
        sink = stream_data_sink_;
      }
      if (sink) {
        sink(stream_id, event.data + 8, event.data_len - 8);
      }
      break;
    }
    case REXNET_EVENT_STREAM_CLOSED: {
      if (event.data_len < 8) {
        break;
      }
      uint64_t stream_id = 0;
      for (int i = 0; i < 8; ++i) {
        stream_id = (stream_id << 8) | event.data[i];
      }
      StreamCloseSink sink;
      {
        std::lock_guard lock(mutex_);
        sink = stream_close_sink_;
      }
      if (sink) {
        sink(stream_id);
      }
      break;
    }
    case REXNET_EVENT_STREAM_CONNECT_FAILED: {
      REXNET_WARN("guest TCP connect to 10.77.{}.{}:{} failed: {}", (event.virtual_ip >> 8) & 0xFF,
                  event.virtual_ip & 0xFF, static_cast<uint16_t>(event.port),
                  std::string(reinterpret_cast<const char*>(event.data), event.data_len));
      StreamConnectSink sink;
      {
        std::lock_guard lock(mutex_);
        sink = stream_connect_sink_;
      }
      if (sink) {
        sink(event.virtual_ip, event.port, 0);  // 0 = failed
      }
      break;
    }
    case REXNET_EVENT_LOCAL_ADDRESS: {
      const uint32_t previous = local_vip_.exchange(event.virtual_ip);
      if (previous != event.virtual_ip) {
        REXNET_INFO("local address: 10.{}.{}.{} (was 10.{}.{}.{})", (event.virtual_ip >> 16) & 0xFF,
                    (event.virtual_ip >> 8) & 0xFF, event.virtual_ip & 0xFF,
                    (previous >> 16) & 0xFF, (previous >> 8) & 0xFF, previous & 0xFF);
      }
      break;
    }
    case REXNET_EVENT_RELAY_STATUS: {
      uint8_t key[20];
      OnlineKey(event.peer, key);
      const std::string online(reinterpret_cast<const char*>(key), 20);
      std::lock_guard lock(mutex_);
      if (event.flag) {
        relays_[online] = true;
      } else {
        relays_.erase(online);
      }
      break;
    }
    case REXNET_EVENT_SHARD_PRESENCE: {
      // data = [has_session u8][session_id 16 if set][rich...]
      ShardMember member;
      member.peer = event.peer;
      member.state = event.flag;
      const uint8_t* p = event.data;
      const uint8_t* end = event.data + event.data_len;
      if (p < end) {
        const bool has_session = *p++ != 0;
        if (has_session && p + 16 <= end) {
          std::memcpy(member.session_id.data(), p, 16);
          member.has_session = true;
          p += 16;
        } else if (has_session) {
          // Truncated: treat as no session rather than reading past.
          p = end;
        }
      }
      if (p < end) {
        member.rich.assign(p, end);
      }

      uint8_t key[20];
      OnlineKey(event.peer, key);
      const std::string online(reinterpret_cast<const char*>(key), 20);
      {
        std::lock_guard lock(mutex_);
        // §17.2: the beat carries no name by design. A friend gets
        // theirs from the presence/friend-name tables; everyone else
        // gets a pseudonym, resolved here so no caller can leak one.
        member.is_friend = friends_.contains(online);
        if (member.is_friend) {
          if (auto it = presence_.find(online); it != presence_.end()) {
            member.display_name = it->second.display_name;
          }
          if (member.display_name.empty()) {
            if (auto it = friend_names_.find(online); it != friend_names_.end()) {
              member.display_name = it->second;
            }
          }
        }
        if (member.display_name.empty()) {
          member.display_name = Pseudonym(event.peer);
        }
        const bool is_new = !shard_members_.contains(online);
        const std::string shown = member.display_name;
        const uint8_t state = member.state;
        shard_members_[online] =
            ShardMemberEntry{std::move(member), std::chrono::steady_clock::now()};
        if (is_new) {
          // Arrivals only -- heartbeats repeat every 30 s and would
          // otherwise fill the log. This is the visible proof that
          // ambient presence is actually reaching us, as opposed to
          // merely sharing a shard id.
          REXNET_INFO("shard member joined: {} (state={}, roster={})", shown, state,
                      shard_members_.size());
        }
      }
      break;
    }
    case REXNET_EVENT_SESSION_FOUND: {
      SessionResult result;
      result.host = event.peer;
      if (event.data_len >= 16) {
        std::memcpy(result.session_id.data(), event.data, 16);
      }
      result.slots_total = static_cast<uint8_t>(event.port);
      result.slots_open = static_cast<uint8_t>(event.src_port);
      result.requires_invite = event.flag != 0;

      NotifySink sink;
      bool accepted = false;
      {
        std::lock_guard lock(mutex_);
        session_results_.push_back(result);
        // A pending invite accept resolves once the host's
        // descriptor arrives: record the join material and let the
        // title's own join flow take over.
        if (pending_accept_) {
          uint8_t host_key[20], want_key[20];
          OnlineKey(event.peer, host_key);
          OnlineKey(pending_accept_->first, want_key);
          if (!std::memcmp(host_key, want_key, 20) &&
              pending_accept_->second == result.session_id) {
            AcceptedInvite info;
            info.host = event.peer;
            info.host_vip = virtual_ips_.FindByOnlineKey(host_key).value_or(0);
            info.session_id = result.session_id;
            accepted_invite_ = info;
            pending_accept_.reset();
            sink = notify_sink_;
            accepted = true;
          }
        }
      }
      if (accepted) {
        const auto& s = result.session_id;
        REXNET_INFO(
            "invite accepted: join material ready, xnkid {:02X} {:02X} "
            "{:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
            s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]);
        if (sink) {
          sink(kXNotifyLiveInviteAccepted, 0);
        }
      }
      break;
    }
    case REXNET_EVENT_PEER_RTT: {
      std::lock_guard lock(mutex_);
      peer_rtt_ms_[event.virtual_ip] = event.port;
      break;
    }
    case REXNET_EVENT_EXTERNAL_ADDRESS: {
      std::string endpoint(reinterpret_cast<const char*>(event.data), event.data_len);
      REXNET_INFO("public endpoint {} (share for a direct connect)", endpoint);
      std::lock_guard lock(mutex_);
      if (public_endpoints_.insert(std::move(endpoint)).second) {
        join_secret_dirty_ = true;
      }
      break;
    }
    case REXNET_EVENT_ERROR: {
      std::string text(reinterpret_cast<const char*>(event.data), event.data_len);
      REXNET_WARN("{}", text);
      Notice notice;
      notice.kind = Notice::Kind::kError;
      notice.peer = event.peer;
      notice.message = std::move(text);
      RaiseNotice(std::move(notice));
      break;
    }
    default:
      break;
  }
}

}  // namespace rex::net
