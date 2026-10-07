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
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rex/cvar.h"
#include "rex/logging.h"
#include "rex/net/discord_presence.h"
#include "rex/net/rexnet_ffi.h"
#include "rex/net/virtual_ip.h"

REXCVAR_DECLARE(bool, rexnet);
REXCVAR_DECLARE(std::string, rexnet_display_name);
REXCVAR_DECLARE(std::string, rexnet_bootstrap);
REXCVAR_DECLARE(bool, rexnet_force_tunnel);
REXCVAR_DECLARE(uint32_t, rexnet_listen_port);
REXCVAR_DECLARE(uint32_t, rexnet_game_port);
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
  /// Circuit-v2 relays to hold reservations on (§9). Several are held at
  /// once rather than one chosen; empty is fully supported. Accelerant, never
  /// authority — a dead entry costs speed, never capability.
  std::vector<std::string> relays;
  /// Also merge in the standard public (Amino) bootstrap set.
  bool use_default_bootstrap = true;
  /// Skip hole punching; run game traffic over the control tunnel (§14).
  /// For reproducing a CGNAT player's degraded path locally.
  bool force_tunnel = false;
  /// Fixed control-plane (QUIC/TCP) listen port; 0 = ephemeral. A stable port
  /// keeps a UPnP mapping or manual forward valid across launches.
  uint16_t listen_port = 0;
  /// Fixed game-plane UDP port; 0 = ephemeral.
  uint16_t game_port = 0;
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

/// Game-plane link state for one peer. `status` is what the guest reads;
/// `degraded` says the connected path is the control tunnel rather than a
/// punched socket (REXNET_EVENT_DEGRADED). A degraded peer is CONNECTED: the
/// title must send, and the core carries the datagrams over the tunnel.
struct PeerLink {
  ConnectStatus status = ConnectStatus::kIdle;
  bool degraded = false;
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
  /// [network] relays — hosted circuit-v2 relays, merged with anything the
  /// host passed programmatically. Per-project, so the SDK ships no
  /// addresses of its own and depends on nobody's infrastructure.
  std::vector<std::string> relays;
  /// [network] bootstrap — extra DHT entry points, same reasoning.
  std::vector<std::string> bootstrap;
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

  /// [discord] — Rich Presence for this title, spoken to the Discord client on
  /// this machine. Empty application id = no presence. The id is the title's
  /// own Discord application; the two lines are what Discord shows under the
  /// game's name until the host states better ones (SetDiscordActivity).
  std::string discord_application_id;
  std::string discord_details;
  std::string discord_state;
  std::string discord_large_image;
  std::string discord_large_text;

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
  using DatagramSink = std::function<bool(uint32_t src_vip, uint16_t src_port, uint16_t dst_port,
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
  using StreamDataSink = std::function<void(uint64_t stream_id, const uint8_t* data, uint32_t len)>;
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
  void SendDatagram(uint32_t virtual_ip, uint16_t src_port, uint16_t dst_port, const void* data,
                    uint32_t len);

  // --- Sessions (§10, generic lifecycle) ------------------------------

  /// XSessionCreate (host): allocates the 128-bit session id (guest sees
  /// XNKID = bytes 0-7, synthesized XNKEY = bytes 8-15), registers it with
  /// the core, and publishes a DHT provider record if public.
  std::array<uint8_t, 16> SessionCreate(bool is_public, uint8_t slots_total, uint8_t slots_open);
  void SessionDelete();
  /// Kick a DHT session search; results accumulate (poll TakeSessionResults).
  void SessionSearch();
  /// Ask one host for its descriptor directly; the answer lands like a search
  /// result. `token` is the host's join token (from its join material); a
  /// private session answers only a query that carries the current one, 0 =
  /// none (public sessions, legacy material).
  void SessionQuery(const RexNetPeerId& peer, uint64_t token = 0);
  /// The join token the local session currently advertises (0 = none).
  /// Generated at SessionCreate; carried in REXJ3 join material.
  uint64_t join_token();
  /// Occupancy of the local session as the core advertises it (descriptor,
  /// Discord party size). RexNet never sees the title's join/leave calls --
  /// the XGI layer or the title states them here after each change.
  void SetSessionSlots(uint32_t open, uint32_t total);
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
    /// When the invite stops being offered (kInviteLifetime after receipt);
    /// expired entries are dropped by the pump. Default = never, for
    /// invites a caller raises for itself.
    std::chrono::steady_clock::time_point expires = std::chrono::steady_clock::time_point::max();
  };
  /// How long a received invite stays pending before the pump drops it.
  static constexpr std::chrono::seconds kInviteLifetime{300};
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
  /// Peers that dismissed a request we sent, since the last call; drained on read.
  std::vector<RexNetPeerId> TakeDeclinedFriendRequests();
  /// Peers a request we sent REACHED (acknowledged), since the last call; drained on read.
  std::vector<RexNetPeerId> TakeDeliveredFriendRequests();
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
  std::optional<std::pair<RexNetPeerId, std::array<uint8_t, 16>>> FindFriendSessionByXnkid(
      const uint8_t xnkid[8]);

  // --- Per-game config + guest context mapping (§11.1) ----------------

  /// Parse a rexnet.toml (returns nullopt on parse failure).
  static std::optional<GameConfig> LoadGameConfig(const std::filesystem::path& path);
  void SetGameConfig(GameConfig config);
  GameConfig game_config();
  /// Turn on the ambient title shard (§17.3) independent of the per-title
  /// rexnet.toml opt-in. Idempotent. Used to back System Link over RexNet: the
  /// shard subnet IS the virtual LAN that System Link broadcasts land on, so
  /// enabling `rexnet_syslink` without a shard would be a silent no-op. Uses
  /// the configured shard cap when one was loaded, else the core default (255).
  void EnableAmbientShard();

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
  ///
  /// One accept at a time per host and session: while one is pending (armed
  /// and not yet resolved or timed out) a repeat for the same host and
  /// session is ignored and returns false, so a second press, a second
  /// Discord Join or a second JoinFromSecret cannot re-arm it and run the
  /// title's join twice. A pending accept the host never answers expires
  /// after kAcceptTimeout and reaches the title as Notice::kJoinTimedOut.
  bool AcceptInvite(const InviteInfo& invite);
  static constexpr std::chrono::seconds kAcceptTimeout{20};
  std::optional<AcceptedInvite> accepted_invite();

  // --- Notices (errors the title shows the player) --------------------

  /// Something the title should tell the player, as the core or this module
  /// stated it. Delivered to the notice sink from the pump thread and kept
  /// in a queue drained by TakeNotices for callers that poll instead.
  struct Notice {
    enum class Kind : uint32_t {
      kError = 0,         // REXNET_EVENT_ERROR: `message` is the core's text
      kInviteFailed = 1,  // REXNET_EVENT_INVITE_FAILED: the invite to `peer` did not land
      kJoinTimedOut = 2,  // AcceptInvite: `peer` (the host) did not answer in time
    };
    Kind kind = Kind::kError;
    RexNetPeerId peer{};  // valid for kInviteFailed and kJoinTimedOut
    std::string message;
  };
  using NoticeSink = std::function<void(const Notice& notice)>;
  void SetNoticeSink(NoticeSink sink);
  /// Notices since the last call; drained on read.
  std::vector<Notice> TakeNotices();

  /// Write a guest-endian XNADDR (36 bytes: ina, inaOnline, wPortOnline,
  /// abEnet, abOnline) for a peer, for XSESSION_INFO and friends.
  static void FillGuestXnAddr(uint8_t out[36], uint32_t virtual_ip, const RexNetPeerId& peer);

  /// Write a guest XSESSION_INFO (60 bytes, XDK order: XNKID(8) +
  /// XNADDR(36) + XNKEY(16)). The 128-bit session id fills XNKID and the
  /// first half of XNKEY (transport is already encrypted, §11).
  static void FillGuestSessionInfo(uint8_t out[60], const std::array<uint8_t, 16>& session_id,
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

  /// Measured round trip to a peer in milliseconds, 0 when unmeasured.
  /// Fed by punch probe acks; titles surface it as ping (spec §11).
  uint32_t PeerRttMs(uint32_t virtual_ip) const;

  const RexNetPeerId& local_peer_id() const { return local_peer_id_; }
  /// Snapshot of our confirmed directly dialable public endpoints
  /// (`host:port`), for the overlay to show and the player to share.
  std::vector<std::string> PublicEndpoints() const {
    std::lock_guard lock(mutex_);
    return {public_endpoints_.begin(), public_endpoints_.end()};
  }
  static std::string PeerIdString(const RexNetPeerId& peer);
  /// Parse a base58 peer id or a REXN- friend code (nullopt on malformed
  /// input, including checksum mismatches).
  static std::optional<RexNetPeerId> PeerIdFromString(const std::string& s);
  /// REXN- friend code for a peer with an embedded display name (name may
  /// be empty; result empty for non-ed25519 peer ids).
  static std::string FriendCode(const RexNetPeerId& peer, const std::string& name);
  /// Parse a friend code, also yielding the embedded display name.
  static std::optional<std::pair<RexNetPeerId, std::string>> ParseFriendCode(const std::string& s);

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

  // --- Shard and invites by material (Discord and other out-of-band joins) ---

  /// Enter the named title shard outright (an invite carried its id).
  void ShardJoin(const std::array<uint8_t, 16>& shard_id);
  /// The title shard we are in, if any.
  std::optional<std::array<uint8_t, 16>> current_shard_id();

  /// Everything a joiner needs, as one string to hand out of band — a Discord
  /// join secret, a pasted code: `REXJ3|title|peer|session|shard|endpoint|token`.
  /// The token is the session's join token (SessionCreate), the proof that
  /// the material came from the host; the core serves a private session's
  /// descriptor only to a query carrying it. Empty when there is no local
  /// session to join. Cached; rebuilt only when one of its inputs changes.
  std::string JoinSecret();
  /// Join from that material: dial the endpoint if it carried one, arm the
  /// accept so the title's own invite path runs (AcceptInvite, with its
  /// one-pending-per-session guard), query the host's descriptor with the
  /// token, search as the fallback, and enter the host's shard. Accepts
  /// REXJ3, REXJ2 (no token) and the first cut's `peer|title|session`. False
  /// when the material does not parse, names another title, or the node is
  /// down.
  bool JoinFromSecret(const std::string& secret);

  // --- Discord "Ask to Join" ---------------------------------------------

  /// Who may join through Discord's Ask to Join without the host's say-so.
  /// Ask (default): every request is queued for the title to answer through
  /// ReplyJoinRequest. Friends: a Discord user the title has linked to a
  /// RexNet friend (LinkDiscordUser) is approved at once, anyone else is
  /// queued as under Ask. All: every request is approved while the session
  /// admits a joiner. A request is never approved while the session is not
  /// joinable, whatever the policy.
  enum class JoinRequestPolicy { Ask, Friends, All };
  void SetJoinRequestPolicy(JoinRequestPolicy policy);
  JoinRequestPolicy join_request_policy();

  /// A Discord user asking to join, as Discord named them.
  struct JoinRequest {
    std::string user_id;
    std::string user_name;
  };
  /// Queued requests since the last call; drained on read. A request the
  /// title has not answered within kJoinRequestLifetime is dropped silently
  /// (Discord's own timeout is shorter, so there is nobody left to answer).
  std::vector<JoinRequest> TakeJoinRequests();
  static constexpr std::chrono::seconds kJoinRequestLifetime{60};
  /// Answer a queued request: accept sends the Discord invite, decline
  /// closes it. Any thread.
  void ReplyJoinRequest(const std::string& user_id, bool accept);
  /// Tell the Friends policy which RexNet peer a Discord user is. The title
  /// owns that knowledge (its own account link, a friend's profile); RexNet
  /// keeps only the map.
  void LinkDiscordUser(const std::string& user_id, const RexNetPeerId& peer);

  /// What Discord shows: the two lines, and whether the local session admits a
  /// joiner (nullopt = whenever a local session exists). Any thread.
  void SetDiscordActivity(const std::string& details, const std::string& state,
                          std::optional<bool> joinable);
  /// Rich Presence on or off (a player setting). Off clears it at Discord.
  void SetDiscordEnabled(bool enabled);

  // --- Overlay / diagnostics snapshot ---------------------------------

  struct Connection {
    RexNetPeerId peer;
    uint32_t virtual_ip = 0;
    ConnectStatus status = ConnectStatus::kIdle;
    bool degraded = false;        // connected over the control tunnel, not a punched socket
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
  /// Expire what has a lifetime (pending accept, join requests, invites);
  /// notices raised here are delivered after the lock is released.
  void Housekeep(std::chrono::steady_clock::time_point now);
  /// Deliver one notice: sink if set, queue for TakeNotices either way.
  void RaiseNotice(Notice notice);

  RexNetHandle* handle_ = nullptr;
  RexNetPeerId local_peer_id_{};

  // Guards the tables below: the pump thread writes, guest threads read.
  // mutable: const accessors (PeerRttMs) still need to lock.
  mutable std::mutex mutex_;
  VirtualIpTable virtual_ips_;
  std::unordered_map<uint32_t, PeerLink> statuses_;
  std::optional<std::array<uint8_t, 16>> session_id_;
  /// Join token advertised with the local session (0 = none); REXJ3 carries it.
  uint64_t join_token_ = 0;
  uint32_t slots_open_ = 0;
  uint32_t slots_total_ = 0;
  /// JoinSecret cache; dirty whenever one of its inputs (session, token,
  /// title id, shard, public endpoints) changes.
  std::string join_secret_cache_;
  bool join_secret_dirty_ = true;
  std::string JoinSecretLocked();
  std::vector<SessionResult> session_results_;
  std::vector<FriendRequestInfo> friend_requests_;
  std::vector<RexNetPeerId> declined_requests_;
  std::vector<RexNetPeerId> delivered_requests_;
  std::vector<InviteInfo> invites_;
  // Keyed by 20-byte abOnline identity string.
  std::unordered_map<std::string, PresenceInfo> presence_;
  /// Peers currently relaying for us (§17.2), keyed by online key.
  std::unordered_map<std::string, bool> relays_;
  /// Written by the pump thread on LocalAddress, read from guest threads.
  std::atomic<uint32_t> local_vip_{kLocalVip};
  /// vip -> round trip in ms, mirrored from REXNET_EVENT_PEER_RTT.
  std::unordered_map<uint32_t, uint32_t> peer_rtt_ms_;
  /// Our confirmed directly dialable public endpoints (UPnP/AutoNAT), as bare
  /// `host:port` strings a player hands a peer for a direct connect. A set so
  /// re-confirmations dedup and the overlay shows a stable list.
  std::set<std::string> public_endpoints_;
  /// Mirrored from ShardJoined / ShardLeft.
  std::optional<std::array<uint8_t, 16>> current_shard_;

  /// Rich Presence, when the title's config names a Discord application. Pumped
  /// from the pump thread; the activity it shows is computed in DiscordPump.
  /// Shared, not unique: the pump thread holds its own reference for the length
  /// of one pump, so a config reload replacing it cannot pull it out from under.
  std::shared_ptr<DiscordPresence> discord_;
  bool discord_enabled_ = true;
  std::string discord_details_;
  std::string discord_state_;
  std::optional<bool> discord_joinable_;
  /// The XEX's own title id, kept beside the advertised one so a join secret
  /// written under either is accepted.
  uint32_t xex_title_id_ = 0;
  /// DiscordPump runs from Pump at most every kDiscordPumpInterval, never on
  /// every event wake: the pump thread delivers datagrams and must not pay
  /// for Discord's IPC on each one.
  static constexpr std::chrono::milliseconds kDiscordPumpInterval{250};
  std::chrono::steady_clock::time_point discord_last_pump_{};
  void DiscordPump();
  void ConfigureDiscordLocked();
  JoinRequestPolicy join_request_policy_ = JoinRequestPolicy::Ask;
  struct JoinRequestEntry {
    JoinRequest request;
    std::chrono::steady_clock::time_point received;
  };
  std::vector<JoinRequestEntry> join_requests_;
  /// Discord user id -> RexNet peer, as the title linked them (Friends policy).
  std::unordered_map<std::string, RexNetPeerId> discord_user_peers_;
  NoticeSink notice_sink_;
  std::vector<Notice> notices_;

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
  std::chrono::steady_clock::time_point pending_accept_since_{};
  std::optional<AcceptedInvite> accepted_invite_;
  DatagramSink datagram_sink_;
  NotifySink notify_sink_;

  std::atomic<bool> pump_running_{false};
  std::thread pump_thread_;

  static RexNet* shared_instance_;
};

}  // namespace rex::net
