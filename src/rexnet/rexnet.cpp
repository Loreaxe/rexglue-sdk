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
#include <chrono>
#include <cstdio>
#include <cstring>
#include <random>
#include <string_view>
#include <utility>

#include <toml++/toml.hpp>

#include "rex/net/pseudonym.h"
#include "rex/ui/overlay/rexnet_overlay.h"

REXCVAR_DEFINE_BOOL(rexnet, true, "RexNet",
                    "Enable the RexNet netplay module (when built in)");
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

namespace rex::net {

RexNet* RexNet::shared_instance_ = nullptr;

std::unique_ptr<RexNet> RexNet::Create(const RexNetOptions& options) {
    const std::string data_dir = options.data_dir.string();

    std::vector<const char*> bootstrap;
    bootstrap.reserve(options.bootstrap.size());
    for (const auto& addr : options.bootstrap) {
        bootstrap.push_back(addr.c_str());
    }

    RexNetConfig cfg{};
    cfg.data_dir = data_dir.c_str();
    cfg.title_id = options.title_id;
    cfg.display_name = options.display_name.c_str();
    cfg.bootstrap = bootstrap.empty() ? nullptr : bootstrap.data();
    cfg.bootstrap_len = static_cast<uint32_t>(bootstrap.size());
    cfg.use_default_bootstrap = options.use_default_bootstrap;
    cfg.force_tunnel = options.force_tunnel;
    if (options.force_tunnel) {
        REXNET_WARN("force_tunnel: hole punching disabled; all game traffic "
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
            case REXNET_LOG_ERROR: REXNET_ERROR("core: {}", message); break;
            case REXNET_LOG_WARN: REXNET_WARN("core: {}", message); break;
            case REXNET_LOG_INFO: REXNET_INFO("core: {}", message); break;
            case REXNET_LOG_DEBUG: REXNET_DEBUG("core: {}", message); break;
            default: REXNET_TRACE("core: {}", message); break;
        }
    });

    RexNetHandle* handle = rexnet_init(&cfg);
    if (!handle) {
        REXNET_ERROR("rexnet-core initialization failed");
        return nullptr;
    }
    auto instance = std::unique_ptr<RexNet>(new RexNet(handle));
    instance->title_id_ = options.title_id;
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
    if (auto sid = net->current_session_id()) {
        status.session_active = true;
    }
    for (const auto& conn : net->SnapshotConnections()) {
        ui::RexNetOverlayStatus::Peer peer;
        peer.virtual_ip = fmt::format("10.77.{}.{}", (conn.virtual_ip >> 8) & 0xFF,
                                      conn.virtual_ip & 0xFF);
        switch (conn.status) {
            case ConnectStatus::kConnected: peer.status = "connected"; break;
            case ConnectStatus::kPending: peer.status = "punching"; break;
            case ConnectStatus::kLost: peer.status = "lost"; break;
            default: peer.status = conn.control_linked ? "linked" : "seen"; break;
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
    if (handle_) {
        rexnet_shutdown(handle_);
        handle_ = nullptr;
    }
}

void RexNet::SetStreamSinks(StreamAcceptSink accept, StreamDataSink data,
                            StreamCloseSink close, StreamConnectSink connect) {
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
    statuses_[virtual_ip] = status;
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
        if (it != statuses_.end() && it->second == ConnectStatus::kConnected) {
            return true;  // already up
        }
        statuses_[virtual_ip] = ConnectStatus::kPending;
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
        return it->second;
    }
    return ConnectStatus::kIdle;
}

void RexNet::SendDatagram(uint32_t virtual_ip, uint16_t src_port, uint16_t dst_port,
                          const void* data, uint32_t len) {
    rexnet_send_datagram(handle_, virtual_ip, src_port, dst_port,
                         static_cast<const uint8_t*>(data), len,
                         /*reliable=*/false);
}

std::array<uint8_t, 16> RexNet::SessionCreate(bool is_public, uint8_t slots_total,
                                              uint8_t slots_open) {
    std::array<uint8_t, 16> id;
    {
        std::random_device rd;
        for (size_t i = 0; i < id.size(); i += 4) {
            uint32_t word = rd();
            std::memcpy(id.data() + i, &word, 4);
        }
    }
    {
        std::lock_guard lock(mutex_);
        session_id_ = id;
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
    REXNET_INFO("session created ({}, {}/{} slots) xnkid {:02X} {:02X} {:02X} {:02X} "
                "{:02X} {:02X} {:02X} {:02X}",
                is_public ? "public" : "private", slots_open, slots_total,
                id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
    return id;
}

void RexNet::SessionDelete() {
    {
        std::lock_guard lock(mutex_);
        session_id_.reset();
        rich_kvs_.erase(kRichKeySessionId);
        PushRichLocked();
    }
    rexnet_session_delete(handle_);
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

void RexNet::SetPresence(uint32_t title_id, uint8_t state, const void* rich,
                         uint32_t rich_len) {
    {
        // Track the last-set state so rich-KV pushes don't revert it.
        std::lock_guard lock(mutex_);
        presence_state_ = state;
        if (title_id) {
            title_id_ = title_id;
        }
    }
    rexnet_set_presence(handle_, title_id, state,
                        static_cast<const uint8_t*>(rich), rich_len);
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

void RexNet::DismissFriendRequest(const RexNetPeerId& peer) {
    std::lock_guard lock(mutex_);
    std::erase_if(friend_requests_,
                  [&](const FriendRequestInfo& r) { return SamePeer(r.peer, peer); });
}

void RexNet::DeclineInvite(const RexNetPeerId& peer) {
    {
        std::lock_guard lock(mutex_);
        std::erase_if(invites_,
                      [&](const InviteInfo& i) { return SamePeer(i.peer, peer); });
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
        config.session_model =
            table["game"]["session_model"].value_or(std::string("private-invite"));
        config.game_channel =
            table["game"]["game_channel"].value_or(std::string("unreliable"));
        config.advertised_title_id = static_cast<uint32_t>(
            table["game"]["advertised_title_id"].value_or(int64_t(0)));
        // [shard] — §17.5. Absent section means disabled, so existing configs
        // keep their current behaviour untouched.
        config.shard_enabled = table["shard"]["enabled"].value_or(false);
        config.shard_cap =
            static_cast<uint16_t>(table["shard"]["cap"].value_or(int64_t(0)));
        config.shard_surface_as_friends =
            table["shard"]["surface_as_friends"].value_or(false);

        if (auto rich = table["presence"]["rich"].as_table()) {
            for (auto& [name, node] : *rich) {
                auto* entry = node.as_table();
                if (!entry) {
                    continue;
                }
                GameConfig::RichMapping mapping;
                mapping.context_id =
                    static_cast<uint32_t>((*entry)["context"].value_or(int64_t(0)));
                mapping.key = static_cast<uint16_t>((*entry)["key"].value_or(int64_t(0)));
                mapping.is_string =
                    (*entry)["type"].value_or(std::string("u32")) == "str";
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
        PushRichLocked();  // re-push presence under the advertised id
    }
    // §17.5: ambient shard is opt-in per title. Applied here rather than at
    // Create() because the config is loaded after the core is up.
    if (game_config_.shard_enabled) {
        rexnet_shard_enable(handle_, game_config_.shard_cap);
    } else {
        rexnet_shard_disable(handle_);
    }
    REXNET_INFO("game config: session_model={} game_channel={} rich mappings={} "
                "advertised_title_id={:08X} shard={}",
                game_config_.session_model, game_config_.game_channel,
                game_config_.presence_rich.size(), title_id_,
                game_config_.shard_enabled
                    ? fmt::format("enabled(cap={})",
                                  game_config_.shard_cap ? game_config_.shard_cap : 255)
                    : "disabled");
}

GameConfig RexNet::game_config() {
    std::lock_guard lock(mutex_);
    return game_config_;
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

void RexNet::AcceptInvite(const InviteInfo& invite) {
    {
        std::lock_guard lock(mutex_);
        pending_accept_ = {invite.peer, invite.session_id};
        accepted_invite_.reset();
        std::erase_if(invites_,
                      [&](const InviteInfo& i) { return SamePeer(i.peer, invite.peer); });
    }
    InviteReply(invite.peer, true);
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

void RexNet::FillGuestXnAddr(uint8_t out[36], uint32_t virtual_ip,
                             const RexNetPeerId& peer) {
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

std::optional<std::pair<RexNetPeerId, std::string>> RexNet::ParseFriendCode(
    const std::string& s) {
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

void RexNet::RememberFriendName(const RexNetPeerId& peer, const std::string& name) {
    if (name.empty()) {
        return;
    }
    uint8_t key[20];
    OnlineKey(peer, key);
    std::lock_guard lock(mutex_);
    friend_names_[std::string(reinterpret_cast<const char*>(key), 20)] = name;
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

// friend_names.txt: one "<base58 peer id> <name>" per line.
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
        } else if (auto vip = virtual_ips_.FindByOnlineKey(
                       reinterpret_cast<const uint8_t*>(key.data()))) {
            if (auto p = virtual_ips_.Lookup(*vip)) {
                peer = *p;
            }
        }
        if (!peer.len) {
            continue;
        }
        const std::string id = PeerIdString(peer);
        std::fprintf(f, "%s %s\n", id.c_str(), name.c_str());
    }
    std::fclose(f);
}

void RexNet::LoadFriendNamesLocked() {
    const auto path = data_dir_ / "friend_names.txt";
    FILE* f = std::fopen(path.string().c_str(), "rb");
    if (!f) {
        return;
    }
    char line[192];
    while (std::fgets(line, sizeof(line), f)) {
        std::string_view view(line);
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
        uint8_t key[20];
        OnlineKey(*peer, key);
        friend_names_[std::string(reinterpret_cast<const char*>(key), 20)] =
            std::string(view.substr(space + 1));
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

void RexNet::ConnectManual(const std::string& multiaddr) {
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
    std::erase_if(shard_members_, [&](const auto& entry) {
        return now - entry.second.last_seen > kLiveness;
    });

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
            conn.status = it->second;
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
            out[indices[ordinal]].display_name = PseudonymWithDiscriminator(
                out[indices[ordinal]].peer, static_cast<uint32_t>(ordinal));
        }
    }
    return out;
}

void RexNet::FillGuestSessionInfo(uint8_t out[60], const std::array<uint8_t, 16>& session_id,
                                  uint32_t host_vip, const RexNetPeerId& host) {
    std::memcpy(out + 0, session_id.data(), 8);      // XNKID sessionID
    FillGuestXnAddr(out + 8, host_vip, host);        // XNADDR hostAddress
    std::memcpy(out + 8 + 36, session_id.data() + 8, 8);  // XNKEY (first half)
    std::memset(out + 8 + 36 + 8, 0, 8);             // XNKEY (rest)
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

std::optional<std::pair<RexNetPeerId, std::array<uint8_t, 16>>>
RexNet::FindFriendSessionByXnkid(const uint8_t xnkid[8]) {
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

std::optional<RexNetPeerId> RexNet::PeerFromVip(uint32_t virtual_ip) {
    std::lock_guard lock(mutex_);
    return virtual_ips_.Lookup(virtual_ip);
}

void RexNet::StartPumpThread() {
    pump_running_.store(true, std::memory_order_relaxed);
    pump_thread_ = std::thread([this] {
        while (pump_running_.load(std::memory_order_relaxed)) {
            Pump();
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    });
}

void RexNet::Pump() {
    RexNetEvent event;
    while (rexnet_poll_event(handle_, &event)) {
        HandleEvent(event);
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
                pending = it != statuses_.end() && it->second == ConnectStatus::kPending;
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
                statuses_[*vip] =
                    event.flag ? ConnectStatus::kConnected : ConnectStatus::kLost;
            }
            break;
        }
        case REXNET_EVENT_PEER_DISCONNECTED: {
            std::lock_guard lock(mutex_);
            if (auto vip = virtual_ips_.Find(event.peer)) {
                statuses_[*vip] = ConnectStatus::kLost;
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
            if (!sink || !sink(event.virtual_ip, event.src_port, event.port,
                               event.data, event.data_len)) {
                REXNET_DEBUG("inbound datagram dropped (no bound guest socket "
                             "for port {})", static_cast<uint32_t>(event.port));
            }
            break;
        }
        case REXNET_EVENT_FRIEND_REQUEST: {
            FriendRequestInfo info;
            info.peer = event.peer;
            // data = [name_len u8][name][note]
            if (event.data_len >= 1) {
                const uint8_t name_len =
                    std::min<uint8_t>(event.data[0],
                                      static_cast<uint8_t>(event.data_len - 1));
                info.display_name.assign(reinterpret_cast<const char*>(event.data + 1),
                                         name_len);
                info.note.assign(reinterpret_cast<const char*>(event.data + 1 + name_len),
                                 event.data_len - 1 - name_len);
            }
            uint8_t key[20];
            OnlineKey(event.peer, key);
            std::lock_guard lock(mutex_);
            if (!info.display_name.empty()) {
                friend_names_[std::string(reinterpret_cast<const char*>(key), 20)] =
                    info.display_name;
            }
            friend_requests_.push_back(std::move(info));
            break;
        }
        case REXNET_EVENT_FRIEND_ACCEPTED: {
            uint8_t key[20];
            OnlineKey(event.peer, key);
            std::lock_guard lock(mutex_);
            friends_[std::string(reinterpret_cast<const char*>(key), 20)] = event.peer;
            // Crossed requests auto-accept in the core; drop any pending
            // inbound request from this (now) friend.
            std::erase_if(friend_requests_, [&](const FriendRequestInfo& r) {
                return SamePeer(r.peer, event.peer);
            });
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
        case REXNET_EVENT_INVITE_RECEIVED: {
            InviteInfo info;
            info.peer = event.peer;
            info.title_id = event.virtual_ip;  // repurposed: title id
            if (event.data_len >= 16) {
                std::memcpy(info.session_id.data(), event.data, 16);
            }
            std::lock_guard lock(mutex_);
            invites_.push_back(std::move(info));
            break;
        }
        case REXNET_EVENT_INVITE_REPLIED:
            // TODO(rexnet): notify_sink_ -> XN_LIVE_INVITE_ACCEPTED once the
            // accepted-invite bridge into XamInviteGetAcceptedInfo exists.
            REXNET_INFO("invite {}", event.flag ? "accepted" : "declined");
            break;
        case REXNET_EVENT_PRESENCE_UPDATED: {
            PresenceInfo info;
            info.title_id = event.virtual_ip;  // repurposed: title id
            info.state = event.flag;
            // data = [name_len u8][name][rich]
            if (event.data_len >= 1) {
                const uint8_t name_len =
                    std::min<uint8_t>(event.data[0],
                                      static_cast<uint8_t>(event.data_len - 1));
                info.display_name.assign(reinterpret_cast<const char*>(event.data + 1),
                                         name_len);
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
            REXNET_WARN("guest TCP connect to 10.77.{}.{}:{} failed: {}",
                        (event.virtual_ip >> 8) & 0xFF, event.virtual_ip & 0xFF,
                        static_cast<uint16_t>(event.port),
                        std::string(reinterpret_cast<const char*>(event.data),
                                    event.data_len));
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
                REXNET_INFO("local address: 10.{}.{}.{} (was 10.{}.{}.{})",
                            (event.virtual_ip >> 16) & 0xFF, (event.virtual_ip >> 8) & 0xFF,
                            event.virtual_ip & 0xFF, (previous >> 16) & 0xFF,
                            (previous >> 8) & 0xFF, previous & 0xFF);
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
                        info.host_vip =
                            virtual_ips_.FindByOnlineKey(host_key).value_or(0);
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
                REXNET_INFO("invite accepted: join material ready, xnkid {:02X} {:02X} "
                            "{:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
                            s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]);
                if (sink) {
                    sink(kXNotifyLiveInviteAccepted, 0);
                }
            }
            break;
        }
        case REXNET_EVENT_ERROR:
            REXNET_WARN("{}", std::string_view(
                                  reinterpret_cast<const char*>(event.data),
                                  event.data_len));
            break;
        default:
            // TODO(rexnet, milestone 5): invites/presence/session results ->
            // guest XNotify queue via notify_sink_.
            break;
    }
}

}  // namespace rex::net
