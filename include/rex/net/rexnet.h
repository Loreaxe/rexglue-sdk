/**
 * @file        net/rexnet.h
 * @brief       RexNet module — serverless Xbox Live replacement (L3 shim).
 *
 * @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
 *              All rights reserved.
 *
 * @license     BSD 3-Clause License
 *              See LICENSE file in the project root for full license text.
 *
 * Owns the rexnet-core handle and the guest-facing state the XAM shim needs:
 * the virtual-IP table and the per-frame event pump. XNet/XSession/XUser/
 * XPresence/XNotify/XInvite exports call into this module; it contains all
 * Xbox semantics, while rexnet-core stays Xbox-free.
 */
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rex/cvar.h"
#include "rex/logging.h"
#include "rex/net/rexnet_ffi.h"
#include "rex/net/virtual_ip.h"

REXCVAR_DECLARE(bool, rexnet);
REXCVAR_DECLARE(std::string, rexnet_display_name);
REXCVAR_DECLARE(std::string, rexnet_bootstrap);
REXCVAR_DECLARE(bool, rexnet_force_tunnel);
REXCVAR_DECLARE(bool, rexnet_syslink);

REXLOG_DEFINE_CATEGORY(net)
#define REXNET_TRACE(...) REXLOG_CAT_TRACE(::rex::log::net(), __VA_ARGS__)
#define REXNET_DEBUG(...) REXLOG_CAT_DEBUG(::rex::log::net(), __VA_ARGS__)
#define REXNET_INFO(...) REXLOG_CAT_INFO(::rex::log::net(), __VA_ARGS__)
#define REXNET_WARN(...) REXLOG_CAT_WARN(::rex::log::net(), __VA_ARGS__)
#define REXNET_ERROR(...) REXLOG_CAT_ERROR(::rex::log::net(), __VA_ARGS__)

namespace rex::net {

struct RexNetOptions {
    /// Directory for identity.key and friend/session state.
    std::filesystem::path data_dir;
    /// From the XEX execution-info header (module->title_id()).
    uint32_t title_id = 0;
    /// Self-asserted display name (<= 32 bytes UTF-8).
    std::string display_name;
    /// Bootstrap multiaddrs; empty is valid (LAN/manual/v6 still work).
    std::vector<std::string> bootstrap;
    /// Also merge in the standard public (Amino) bootstrap set.
    bool use_default_bootstrap = true;
    /// Skip hole punching; run game traffic over the control tunnel (§14).
    /// For reproducing a CGNAT player's degraded path locally.
    bool force_tunnel = false;
    /// Drain rexnet-core events on a dedicated ~60 Hz thread. The runtime
    /// library does not own the frame loop, so this is the default; callers
    /// that want to drive Pump() themselves can turn it off.
    bool start_pump_thread = true;
};

/// Values match XNET_CONNECT_STATUS_* (XRNM gates every send on CONNECTED).
enum class ConnectStatus : uint32_t {
    kIdle = 0,
    kPending = 1,
    kConnected = 2,
    kLost = 3,
};

/// Guest notification ids (XNotificationKey: mask_index<<25 | ver<<16 | id).
constexpr uint32_t kXNotifySystemSignInChanged = 0x0000000A;
constexpr uint32_t kXNotifyLiveConnectionChanged = 0x02000001;
constexpr uint32_t kXNotifyLiveInviteAccepted = 0x02000002;
constexpr uint32_t kXNotifyFriendsPresenceChanged = 0x04000001;
/// XN_LIVE_CONNECTIONCHANGED payload: XONLINE_S_LOGON_CONNECTION_ESTABLISHED.
/// Exact value the guest compares against (Fable 2 gates Live presence on
/// it: sub_822CAD38 cmp 0x1510F0). Getting this wrong makes the title read
/// the event as connection-lost and never enable presence.
constexpr uint32_t kXOnlineLogonConnectionEstablished = 0x001510F0;

/// Reserved presence rich-KV keys (game config mappings use guest context
/// ids; this range is module-internal). kRichKeyTitleName carries the
/// advertised game name; kRichKeySessionId the host's 16-byte session id
/// while one is active — the friends-enumerator JOINABLE/sessionID source
/// that drives join-in-progress UIs (Fable 2 orbs).
constexpr uint16_t kRichKeyTitleName = 0xFF00;
constexpr uint16_t kRichKeySessionId = 0xFF01;

/// Per-game configuration (design spec §11.1). Everything derivable at
/// runtime (title id, ports) stays out; only genuinely per-game knowledge
/// lives here.
struct GameConfig {
    /// "private-invite" (no public lobby browser) or "public".
    std::string session_model = "private-invite";
    /// "unreliable": the title runs its own reliability (e.g. XRNM) and the
    /// game plane must never wrap it in a reliable carrier (§6).
    std::string game_channel = "unreliable";
    /// Title id to advertise in presence/friend records instead of the XEX
    /// header's (0 = use the header id). Xbox Live grouped SKUs (retail /
    /// GOTY / episodic) under one identity, and titles hardcode that id
    /// when checking friend records — Fable 2's episodic XEX runs as
    /// 0x4D5307F1 but its join code checks for retail 0x4D530910.
    uint32_t advertised_title_id = 0;

    /// Ambient title shard, §17.3/§17.5. Opt-in: a title that has no use for
    /// passive presence never joins one and pays no ambient traffic.
    bool shard_enabled = false;
    /// 0 = spec default (255). Lower it for titles whose ambient payload is
    /// heavy enough that a full shard would be noisy.
    uint16_t shard_cap = 0;
    /// Surface shard members through the XFriends enumerator, so a title
    /// whose player-visible UI is friend-driven shows ambient players too.
    /// This is how Fable 2's passive orbs get populated -- its orb list is
    /// built from XONLINE_FRIEND records, so shard members are invisible
    /// without it. Opt-in: it puts non-friends in a list the XDK names
    /// "friends", which not every title will want.
    bool shard_surface_as_friends = false;

    /// XUserSetContext/SetProperty id -> presence rich KV mapping.
    struct RichMapping {
        uint32_t context_id = 0;
        uint16_t key = 0;
        bool is_string = false;
    };
    std::vector<RichMapping> presence_rich;
};

class RexNet {
 public:
    /// Default virtual IP reported in our own XNADDR before we are on a
    /// shard's subnet: 10.77.255.254, inside the reserved /24 (§17.3.5).
    /// Remote XNADDRs are matched by abOnline key, never by this value, so
    /// every instance sharing it is fine.
    static constexpr uint32_t kLocalVip = VirtualIpTable::kNetworkBase | 0xFFFE;

    /// Our current address. Equals kLocalVip until we join a shard, then our
    /// host on that shard's /24 — being genuinely *on* the subnet is what
    /// makes broadcast to it meaningful (§17.3.6).
    uint32_t local_vip() const { return local_vip_.load(std::memory_order_relaxed); }

    /// Delivers an inbound game datagram to the guest (return false = drop).
    using DatagramSink =
        std::function<bool(uint32_t src_vip, uint16_t src_port, uint16_t dst_port,
                           const uint8_t* data, uint32_t len)>;
    /// Injects a notification into the guest XNotify queue.
    using NotifySink = std::function<void(uint32_t id, uint32_t param)>;

    // --- Guest TCP (§18) -------------------------------------------------
    //
    // System Link titles use stream sockets, not only datagrams: SoulCalibur
    // IV listens on TCP 1001 and runs its session there. These bridge a guest
    // SOCK_STREAM socket onto a libp2p stream.

    /// An inbound connection arrived for a listening guest port. Return false
    /// if nothing is listening, so the stream can be closed rather than left
    /// dangling.
    using StreamAcceptSink = std::function<bool(uint64_t stream_id, uint32_t peer_vip,
                                                uint16_t local_port, uint16_t remote_port)>;
    /// Bytes arrived on a guest connection.
    using StreamDataSink =
        std::function<void(uint64_t stream_id, const uint8_t* data, uint32_t len)>;
    /// A guest connection ended, from either side.
    using StreamCloseSink = std::function<void(uint64_t stream_id)>;
    /// An outbound connect resolved: `stream_id` is 0 on failure.
    using StreamConnectSink =
        std::function<void(uint32_t peer_vip, uint16_t dst_port, uint64_t stream_id)>;

    void SetStreamSinks(StreamAcceptSink accept, StreamDataSink data, StreamCloseSink close,
                        StreamConnectSink connect);

    /// Open a guest TCP connection to a peer's virtual IP. The result arrives
    /// on the connect sink, since the FFI never blocks the guest thread.
    void StreamConnect(uint32_t virtual_ip, uint16_t src_port, uint16_t dst_port);
    void StreamSend(uint64_t stream_id, const void* data, uint32_t len);
    void StreamClose(uint64_t stream_id);

    /// Returns nullptr if rexnet-core fails to initialize.
    static std::unique_ptr<RexNet> Create(const RexNetOptions& options);

    /// Process-wide instance management. The XAM shim initializes on the
    /// first XNetStartup and destroys on the last XNetCleanup; an instance
    /// left alive at process exit is deliberately leaked (joining the tokio
    /// runtime during static destruction is deadlock-prone).
    static RexNet* InitializeShared(const RexNetOptions& options);
    static RexNet* shared();
    static void DestroyShared();

    ~RexNet();

    RexNet(const RexNet&) = delete;
    RexNet& operator=(const RexNet&) = delete;

    /// Drain rexnet-core events (runs on the pump thread by default).
    void Pump();

    /// Sinks bridging into the kernel layer (registered by the XAM shim so
    /// this module never links against rexsystem).
    void SetDatagramSink(DatagramSink sink);
    void SetNotifySink(NotifySink sink);

    // --- Guest-thread API (XNet exports) --------------------------------

    /// XNetConnect: dial + punch the peer behind a virtual IP.
    /// Returns false if the vip is unknown.
    bool Connect(uint32_t virtual_ip);
    /// XNetGetConnectStatus.
    ConnectStatus GetConnectStatus(uint32_t virtual_ip);
    /// sendto on a virtual IP: route through the punched game socket.
    void SendDatagram(uint32_t virtual_ip, uint16_t src_port, uint16_t dst_port,
                      const void* data, uint32_t len);

    // --- Sessions (§10, generic lifecycle) ------------------------------

    /// XSessionCreate (host): allocates the 128-bit session id (guest sees
    /// XNKID = bytes 0-7, synthesized XNKEY = bytes 8-15), registers it with
    /// the core, and publishes a DHT provider record if public.
    std::array<uint8_t, 16> SessionCreate(bool is_public, uint8_t slots_total,
                                          uint8_t slots_open);
    void SessionDelete();
    /// Kick a DHT session search; results accumulate (poll TakeSessionResults).
    void SessionSearch();
    std::optional<std::array<uint8_t, 16>> current_session_id();

    struct SessionResult {
        RexNetPeerId host;
        std::array<uint8_t, 16> session_id;
        uint8_t slots_total = 0;
        uint8_t slots_open = 0;
        bool requires_invite = false;
    };
    /// Drain accumulated search results (host peers are already in the
    /// virtual-IP table — the query required a connection).
    std::vector<SessionResult> TakeSessionResults();

    // --- Presence / friends / invites (§8.1–8.3) ------------------------

    /// XUserSetContext/SetProperty land here (per-game config maps them to
    /// rich KVs); pushed to mutual friends and heartbeated by the core.
    void SetPresence(uint32_t title_id, uint8_t state, const void* rich, uint32_t rich_len);

    void FriendRequest(const RexNetPeerId& peer, const std::string& note);
    void FriendAccept(const RexNetPeerId& peer);
    void FriendRemove(const RexNetPeerId& peer);

    /// Invite a peer to the current local session.
    void SendInvite(const RexNetPeerId& peer);
    void InviteReply(const RexNetPeerId& peer, bool accept);

    struct FriendRequestInfo {
        RexNetPeerId peer;
        std::string display_name;  // requester's self-asserted name
        std::string note;
    };
    struct InviteInfo {
        RexNetPeerId peer;
        uint32_t title_id = 0;
        std::array<uint8_t, 16> session_id{};
    };
    struct PresenceInfo {
        uint32_t title_id = 0;
        uint8_t state = 0;
        std::string display_name;
        std::string title_name;  // from the reserved rich KV, may be empty
        bool has_session = false;
        std::array<uint8_t, 16> session_id{};  // valid when has_session
        std::vector<uint8_t> rich;
    };

    /// Pending inbound requests/invites (snapshots; resolved explicitly by
    /// the calls below, typically from the F6 overlay).
    std::vector<FriendRequestInfo> PendingFriendRequests();
    std::vector<InviteInfo> PendingInvites();
    /// Accept a pending friend request (mutual consent completes when the
    /// FriendAccepted event lands) / dismiss it locally without replying.
    void AcceptFriendRequest(const RexNetPeerId& peer);
    void DismissFriendRequest(const RexNetPeerId& peer);
    /// Decline a pending invite (replies to the host, drops it locally).
    void DeclineInvite(const RexNetPeerId& peer);

    /// Latest presence for a peer (keyed by abOnline identity).
    std::optional<PresenceInfo> GetPresence(const RexNetPeerId& peer);
    /// Mirror of the core's persisted friend list (bootstrapped at init).
    std::vector<RexNetPeerId> GetFriends();

    /// Friend list joined with live presence/link state (XFriends
    /// enumerator + overlay).
    struct FriendInfo {
        RexNetPeerId peer;
        std::string display_name;  // presence name, else remembered name
        bool online = false;       // libp2p control connection is up
        uint32_t title_id = 0;     // last presence, 0 = unknown
        std::string title_name;    // advertised game name, may be empty
        bool has_session = false;  // friend is hosting a joinable session
        std::array<uint8_t, 16> session_id{};
        uint8_t presence_state = 0;
    };
    std::vector<FriendInfo> SnapshotFriends();

    /// Resolve a friend's advertised session by its XNKID (first 8 bytes of
    /// the session id): host peer + full 16-byte id. XSessionSearchByID.
    std::optional<std::pair<RexNetPeerId, std::array<uint8_t, 16>>>
    FindFriendSessionByXnkid(const uint8_t xnkid[8]);

    // --- Per-game config + guest context mapping (§11.1) ----------------

    /// Parse a rexnet.toml (returns nullopt on parse failure).
    static std::optional<GameConfig> LoadGameConfig(const std::filesystem::path& path);
    void SetGameConfig(GameConfig config);
    GameConfig game_config();

    /// XUserSetContext: map a context id through the game config into a
    /// presence rich KV (u32) and push. Unmapped ids are ignored.
    void OnGuestContext(uint32_t context_id, uint32_t value);
    /// XUserSetProperty: same, raw value bytes (strings stay guest-encoded).
    void OnGuestProperty(uint32_t property_id, const uint8_t* value, uint32_t len);

    // --- Invite accept bridge (XN_LIVE_INVITE_ACCEPTED) -----------------

    struct AcceptedInvite {
        RexNetPeerId host;
        uint32_t host_vip = 0;
        std::array<uint8_t, 16> session_id{};
    };
    /// Accept a received invite: replies to the host and, once the session
    /// descriptor arrives, records it and fires XN_LIVE_INVITE_ACCEPTED so
    /// the title's existing join flow runs (§8.3).
    void AcceptInvite(const InviteInfo& invite);
    std::optional<AcceptedInvite> accepted_invite();

    /// Write a guest-endian XNADDR (36 bytes: ina, inaOnline, wPortOnline,
    /// abEnet, abOnline) for a peer, for XSESSION_INFO and friends.
    static void FillGuestXnAddr(uint8_t out[36], uint32_t virtual_ip,
                                const RexNetPeerId& peer);

    /// Write a guest XSESSION_INFO (60 bytes, XDK order: XNKID(8) +
    /// XNADDR(36) + XNKEY(16)). The 128-bit session id fills XNKID and the
    /// first half of XNKEY (transport is already encrypted, §11).
    static void FillGuestSessionInfo(uint8_t out[60],
                                     const std::array<uint8_t, 16>& session_id,
                                     uint32_t host_vip, const RexNetPeerId& host);

    /// abOnline key (20 bytes) for a peer id — the stable identity carried
    /// inside synthesized XNADDRs.
    static void OnlineKey(const RexNetPeerId& peer, uint8_t out[20]);
    /// Pseudo-MAC for abEnet (6 stable bytes from the peer id).
    static void EnetAddr(const RexNetPeerId& peer, uint8_t out[6]);

    /// Resolve an abOnline key to a virtual IP (host order).
    std::optional<uint32_t> VipFromOnlineKey(const uint8_t key[20]);
    /// Peer behind a virtual IP.
    std::optional<RexNetPeerId> PeerFromVip(uint32_t virtual_ip);

    const RexNetPeerId& local_peer_id() const { return local_peer_id_; }
    static std::string PeerIdString(const RexNetPeerId& peer);
    /// Parse a base58 peer id or a REXN- friend code (nullopt on malformed
    /// input, including checksum mismatches).
    static std::optional<RexNetPeerId> PeerIdFromString(const std::string& s);
    /// REXN- friend code for a peer with an embedded display name (name may
    /// be empty; result empty for non-ed25519 peer ids).
    static std::string FriendCode(const RexNetPeerId& peer, const std::string& name);
    /// Parse a friend code, also yielding the embedded display name.
    static std::optional<std::pair<RexNetPeerId, std::string>> ParseFriendCode(
        const std::string& s);

    /// Self-asserted display name: applied live (presence re-push), stored
    /// in the current-name accessor, and persisted to display_name.txt in
    /// the data dir so the next boot picks it up.
    void SetDisplayName(const std::string& name);
    std::string display_name();
    /// Remember a friend's name learned outside presence (friend-code
    /// paste, friend-request payload); persisted to friend_names.txt.
    void RememberFriendName(const RexNetPeerId& peer, const std::string& name);

    /// Advertised local game name: carried to friends as a reserved
    /// presence rich KV so their UI can show a title string, not an id.
    void SetGameTitleName(const std::string& name);
    /// Stable offline-format XUID (0xE0...) derived from a peer identity;
    /// the same formula every instance applies to itself, so a friend's
    /// derived XUID here matches what they present in e.g. msgJoinSession.
    static uint64_t XuidFromPeer(const RexNetPeerId& peer);

    /// Dial a manual connect string (multiaddr, §4 preservation floor).
    void ConnectManual(const std::string& multiaddr);

    // --- Overlay / diagnostics snapshot ---------------------------------

    struct Connection {
        RexNetPeerId peer;
        uint32_t virtual_ip = 0;
        ConnectStatus status = ConnectStatus::kIdle;
        bool control_linked = false;  // libp2p control connection is up
        bool is_friend = false;       // mutual-consent friend (§8.2)
        bool is_relay = false;        // currently carrying our traffic (§9)
        /// Safe to render as-is (§17.2): a friend's self-asserted name, or a
        /// generated pseudonym for everyone else. Never a non-friend's own
        /// name, whatever presence happened to carry.
        std::string display_name;
    };
    /// One entry per peer with an allocated virtual IP (connected or seen).
    std::vector<Connection> SnapshotConnections();

    /// Mutual-consent friendship test (§8.2). Drives the §17.2 display rule.
    bool IsFriend(const RexNetPeerId& peer);

    // --- Ambient title shard (§17.3) ------------------------------------

    /// One member of our shard, as last heard on the topic. Populated from
    /// ShardPresence events; entries age out when a peer stops beating.
    struct ShardMember {
        RexNetPeerId peer;
        /// §17.2-resolved and safe to render: a friend's own name, else a
        /// pseudonym. The wire record carries no name at all (§17.3.8).
        std::string display_name;
        bool is_friend = false;
        uint8_t state = 0;  // 0 offline, 1 online, 2 in-game, 3 joinable
        bool has_session = false;
        std::array<uint8_t, 16> session_id{};
        /// Per-title ambient KVs, exactly as the sender packed them.
        std::vector<uint8_t> rich;
    };
    /// Live shard roster, freshest first. Excludes members not heard from
    /// within the liveness window.
    std::vector<ShardMember> SnapshotShardMembers();

    /// Raw handle for shim code that needs FFI calls not yet wrapped.
    RexNetHandle* handle() { return handle_; }

 private:
    explicit RexNet(RexNetHandle* handle);

    void StartPumpThread();
    void HandleEvent(const RexNetEvent& event);
    void SetStatus(uint32_t virtual_ip, ConnectStatus status);

    RexNetHandle* handle_ = nullptr;
    RexNetPeerId local_peer_id_{};

    // Guards the tables below: the pump thread writes, guest threads read.
    std::mutex mutex_;
    VirtualIpTable virtual_ips_;
    std::unordered_map<uint32_t, ConnectStatus> statuses_;
    std::optional<std::array<uint8_t, 16>> session_id_;
    std::vector<SessionResult> session_results_;
    std::vector<FriendRequestInfo> friend_requests_;
    std::vector<InviteInfo> invites_;
    // Keyed by 20-byte abOnline identity string.
    std::unordered_map<std::string, PresenceInfo> presence_;
    /// Peers currently relaying for us (§17.2), keyed by online key.
    std::unordered_map<std::string, bool> relays_;
    /// Written by the pump thread on LocalAddress, read from guest threads.
    std::atomic<uint32_t> local_vip_{kLocalVip};

    StreamAcceptSink stream_accept_sink_;
    StreamDataSink stream_data_sink_;
    StreamCloseSink stream_close_sink_;
    StreamConnectSink stream_connect_sink_;
    /// Shard roster keyed by online key, with the last beat's arrival time.
    /// A peer that goes quiet is dropped rather than lingering as a ghost
    /// orb -- there is no leave message, so silence is the only signal.
    struct ShardMemberEntry {
        ShardMember member;
        std::chrono::steady_clock::time_point last_seen;
    };
    std::unordered_map<std::string, ShardMemberEntry> shard_members_;
    std::unordered_map<std::string, RexNetPeerId> friends_;
    // Peers with a live libp2p control connection (by 20-byte online key).
    std::unordered_map<std::string, bool> control_linked_;

    GameConfig game_config_;
    uint32_t title_id_ = 0;
    uint8_t presence_state_ = 2;  // in-game
    std::filesystem::path data_dir_;
    std::string display_name_;
    // Names learned outside presence (friend-code paste, request payload),
    // keyed by 20-byte abOnline identity; persisted to friend_names.txt.
    std::unordered_map<std::string, std::string> friend_names_;
    void SaveFriendNamesLocked();
    void LoadFriendNamesLocked();
    // Rich KV store: key -> (is_string, value bytes).
    struct RichValue {
        bool is_string = false;
        std::vector<uint8_t> bytes;
    };
    std::map<uint16_t, RichValue> rich_kvs_;
    void PushRichLocked();  // rebuild blob + SetPresence; call with mutex_ held

    std::optional<std::pair<RexNetPeerId, std::array<uint8_t, 16>>> pending_accept_;
    std::optional<AcceptedInvite> accepted_invite_;
    DatagramSink datagram_sink_;
    NotifySink notify_sink_;

    std::atomic<bool> pump_running_{false};
    std::thread pump_thread_;

    static RexNet* shared_instance_;
};

}  // namespace rex::net
