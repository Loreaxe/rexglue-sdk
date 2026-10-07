// @file        rexnet/core/src/engine.rs
// @brief       Async engine: the libp2p swarm, the game-plane UDP socket, and the.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Async engine: the libp2p swarm, the game-plane UDP socket, and the
//! command/event loop.
//!
//! Control plane per design spec §5/§6: QUIC (+TCP fallback) transport,
//! Kademlia peer routing (Amino-compatible, client mode), identify, AutoNAT,
//! circuit-v2 relay client, DCUtR hole punching, and LAN mDNS. The FFI layer
//! enqueues [`Command`]s and drains [`Event`]s; the CLI drives the same
//! engine directly (Rust-level commands carry oneshot reply channels).
//!
//! Game plane per §6/§8.4: a dedicated UDP socket multiplexing all guest
//! ports, established per-peer by a simultaneous-open punch coordinated over
//! the control connection. Every peer gets a virtual IP in 10.77.0.0/16
//! (allocated here, mirrored by the C++ shim via `PeerConnected`).
//!
//! The §7 presence/friend/invite/session protocols, the encrypted game plane
//! (§6), the §5 tunnel fallback for hostile-NAT pairs, the §17 ambient shard,
//! and the §9 mesh-relay register/discovery are all implemented; see the
//! per-item comments below.

use std::collections::{HashMap, HashSet};
use std::net::{IpAddr, SocketAddr};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use libp2p::futures::StreamExt;
use libp2p::identity::Keypair;
use libp2p::kad::store::MemoryStore;
use libp2p::multiaddr::Protocol;
use libp2p::request_response::{self, OutboundRequestId, ProtocolSupport};
use libp2p::swarm::dial_opts::{DialOpts, PeerCondition};
use libp2p::swarm::{NetworkBehaviour, SwarmEvent};
use libp2p::{
    autonat, connection_limits, dcutr, gossipsub, identify, kad, mdns, relay, upnp, Multiaddr,
    PeerId, StreamProtocol, Swarm,
};
use serde::{Deserialize, Serialize};
use tokio::net::UdpSocket;
use tokio::sync::{mpsc, oneshot, Notify};

use crate::friends::FriendStore;
use crate::shard::{self, Candidate, Migration, Placement, ShardDesc, DEFAULT_SHARD_CAP};
use crate::subnet;
use crate::tcp::{StreamHeader, StreamId, StreamInfo, StreamTable, STREAM_HEADER_LEN, STREAM_SCHEMA};
use crate::crypto::{self, LocalKeyAgreement};
use crate::game_plane::{now_micros, Emitter, GamePlane};
use crate::tunnel;

pub type BoxError = Box<dyn std::error::Error + Send + Sync>;

/// Base of the virtual-IP network handed to peers: 10.77.0.0/16 (§11).
pub const VIP_NETWORK_BASE: u32 = (10 << 24) | (77 << 16);

/// Dotted-quad rendering for logs; virtual IPs are held in host order.
fn format_vip(vip: u32) -> String {
    format!(
        "{}.{}.{}.{}",
        (vip >> 24) & 0xFF,
        (vip >> 16) & 0xFF,
        (vip >> 8) & 0xFF,
        vip & 0xFF
    )
}

/// Reduce a listen/external multiaddr to the bare `host:port` a player pastes
/// into the direct-connect field. IPv6 is bracketed so the port stays
/// unambiguous. Returns `None` for addresses without an ip+port pair (e.g. a
/// relay circuit address), which are not directly dialable.
fn dialable_endpoint(addr: &Multiaddr) -> Option<String> {
    let mut host: Option<String> = None;
    let mut port: Option<u16> = None;
    for proto in addr.iter() {
        match proto {
            Protocol::Ip4(ip) => host = Some(ip.to_string()),
            Protocol::Ip6(ip) => host = Some(format!("[{ip}]")),
            Protocol::Udp(p) | Protocol::Tcp(p) => port = Some(p),
            _ => {}
        }
    }
    Some(format!("{}:{}", host?, port?))
}

/// Conservative "reachable from the public internet" test for a multiaddr:
/// reject loopback, RFC1918 private, link-local, ULA and unspecified ranges.
/// Used so an auto relay reservation is only held on a peer other NAT'd peers
/// could actually reach us through.
fn multiaddr_is_global(addr: &Multiaddr) -> bool {
    for proto in addr.iter() {
        match proto {
            Protocol::Ip4(ip) => {
                return !(ip.is_loopback()
                    || ip.is_private()
                    || ip.is_link_local()
                    || ip.is_unspecified()
                    || ip.is_broadcast()
                    || ip.is_documentation());
            }
            Protocol::Ip6(ip) => {
                // Stable Rust has no is_unique_local/is_global for v6, so test
                // the fc00::/7 ULA and fe80::/10 link-local prefixes by hand.
                let seg = ip.segments();
                let is_ula = (seg[0] & 0xfe00) == 0xfc00;
                let is_link_local = (seg[0] & 0xffc0) == 0xfe80;
                return !(ip.is_loopback() || ip.is_unspecified() || is_ula || is_link_local);
            }
            _ => {}
        }
    }
    false
}

/// Smallest RTT change worth telling the shim about; below this it is noise.
const RTT_REPORT_DELTA_MS: u32 = 5;

/// `[type][nonce 16][echo 8]`.
const PROBE_FRAME_LEN: usize = 25;

/// Once per key per window, for diagnostics that would otherwise fire per
/// packet. Keys are forgotten once their window has passed, so the table is
/// bounded by what is actually being complained about.
struct WarnLimiter<K> {
    window: Duration,
    last: HashMap<K, Instant>,
}

impl<K: std::hash::Hash + Eq + Copy> WarnLimiter<K> {
    fn new(window: Duration) -> Self {
        Self { window, last: HashMap::new() }
    }

    fn allow(&mut self, key: K) -> bool {
        let now = Instant::now();
        if self.last.len() >= 256 {
            let window = self.window;
            self.last.retain(|_, at| now.saturating_duration_since(*at) < window);
        }
        match self.last.get(&key) {
            Some(at) if now.saturating_duration_since(*at) < self.window => false,
            _ => {
                self.last.insert(key, now);
                true
            }
        }
    }
}

/// Window of the per-packet warnings (unknown virtual IP, no keys).
const PACKET_WARN_WINDOW: Duration = Duration::from_secs(10);

/// Game-plane frame types (first byte on the game socket).
pub const FRAME_DATA: u8 = 0x01;
const FRAME_PROBE: u8 = 0x02;
const FRAME_PROBE_ACK: u8 = 0x03;

/// Shard rescan cadence and migration cooldown (§17.3.3, §17.6 open item:
/// untuned). The sweep is cheap -- one provider lookup -- but a migration is
/// a resubscribe, so moves are rate-limited far more aggressively than scans.
/// How often a relay with no live reservation is re-dialled. Slow on purpose:
/// relays are an accelerant, and a node with none still works.
const RELAY_RETRY_INTERVAL: Duration = Duration::from_secs(60);

/// Cap on automatic circuit reservations held on mesh peers (§9 torrent model).
/// A couple is enough redundancy for DCUtR to find a rendezvous path; more
/// would turn a large swarm into an N-squared reservation storm.
const MAX_AUTO_RELAYS: usize = 2;

const SHARD_RESCAN_INTERVAL: Duration = Duration::from_secs(20);
/// How long to keep looking after a fruitless sweep before founding a shard.
///
/// The sweep can finish before a peer that is already up has connected and
/// identified -- on LAN that gap was measured at ~6 s -- and a node that founds
/// a shard inside that window abandons it moments later when the peer's
/// descriptor finally arrives. Convergence handles the collision correctly, but
/// every hop re-rolls the shard id and therefore the subnet, re-addressing the
/// whole membership for nothing.
///
/// Costs a node that really is alone a short wait before ambient presence
/// starts; it blocks nothing else, and friend invites do not go through here.
const SHARD_CREATE_GRACE: Duration = Duration::from_secs(10);
/// Below this many members, migrate without the jitter roll. Jitter exists to
/// stop a large shard stampeding into another one; with a handful of members
/// there is no stampede to prevent, and the roll just leaves two players
/// sitting in separate shards for minutes -- which defeats the whole point of
/// the tier.
const SHARD_JITTER_MIN_MEMBERS: usize = 5;
const SHARD_MIGRATION_COOLDOWN: Duration = Duration::from_secs(300);
/// Probability a node acts on a migration decision in any given rescan.
/// Every member of a shard reaches the same conclusion from the same DHT
/// state at roughly the same moment, so without this they would all
/// resubscribe in lockstep -- a self-inflicted thundering herd on the shard
/// they are moving to. Direction is deliberately unconditional (§17.3.3),
/// which leaves spreading the *timing* as the only defence. Deferring
/// re-rolls next rescan, so a population drains over several intervals
/// (~half each) rather than in one instant.
const SHARD_MIGRATION_CHANCE: f64 = 0.5;
/// While a shard is at most this many members, hold a control connection to
/// every one of them. Gossipsub only meshes with ~6 peers regardless of topic
/// size, so membership alone leaves most of a shard unreachable: no address,
/// no connection, and an invite to them cold-starts with a DHT lookup that
/// often fails outright. Below this size the connections are cheap and every
/// member is instantly invitable -- which is the point of the Connected tier.
/// Above it we fall back to resolving addresses only, so a 255-member shard
/// does not mean 255 connections.
const SHARD_WARM_CONNECT_LIMIT: usize = 24;
/// How long an invite may wait for its peer to become reachable. An invite
/// that finally lands minutes after the user asked is worse than none: they
/// have moved on, and the session it names may be gone. Matches the 5 minute
/// expiry carried in the invite itself. Also bounds a queued Decline, and how
/// long an accepted invite counts as a session before title traffic has to
/// speak for itself.
const INVITE_DEFER_MAX: Duration = Duration::from_secs(300);
/// Delivery attempts for one invite before the host is told it failed: the
/// first send, and one re-queue for the next connection.
const INVITE_ATTEMPTS: u8 = 2;
/// A peer that exchanged a title datagram with us this recently is in a
/// session with us, whatever the control plane thinks. Generous against
/// titles that heartbeat slowly; short enough that a finished game frees
/// the addresses within the minute.
const TITLE_TRAFFIC_WINDOW: Duration = Duration::from_secs(20);
/// A shard member that has not been heard on the topic (beat or descriptor)
/// for this long is gone: three beats missed, with slack for gossip delay.
const SHARD_MEMBER_TIMEOUT: Duration = Duration::from_secs(95);
/// Friends and peers we owe a message are looked up in the DHT and dialled on
/// this backoff while they stay away: first after 30 s, doubling to 10 min,
/// with +-20 % jitter so a population does not retry in lockstep.
const PEER_LOOKUP_FIRST: Duration = Duration::from_secs(30);
const PEER_LOOKUP_MAX: Duration = Duration::from_secs(600);
const RETRY_JITTER: f64 = 0.2;
/// Sources whose frames are held before they map to a peer. Bounds what an
/// unmapped sender -- or a flood of them -- can make us hold.
const ORPHAN_SOURCES_MAX: usize = 64;
/// Our own reachability record (`protocol::peer_key`) is published at each
/// heartbeat until a DHT node stores it, then refreshed every ten minutes so
/// new addresses and lost stores recover (kad's own republish is 12 h).
const OWN_KEY_REPUBLISH: Duration = Duration::from_secs(600);
/// A relay that refused or failed our reservation is not picked again for this long.
const AUTO_RELAY_BACKOFF: Duration = Duration::from_secs(600);
/// UPnP lease on the game socket, renewed at half-life as libp2p-upnp does.
const GAME_PORT_LEASE: Duration = Duration::from_secs(3600);
/// A game peer whose last control connection dropped is given this long to
/// come back before the title is told it left: the title's own timeouts are
/// longer, and a QUIC path that died often has a TCP or relay path alive.
/// Longer than a punch can take to resolve, so a peer whose connection
/// returns late in the grace still gets its game path back (punched or
/// tunnelled) before the title hears it left.
const PEER_RECOVERY_GRACE: Duration = Duration::from_secs(8);
const PEER_RECOVERY_TICK: Duration = Duration::from_millis(250);
const PEER_RECOVERY_RETRY: Duration = Duration::from_secs(1);
/// After a direct QUIC path failed, other routes to that peer are tried first
/// for this long.
const QUIC_AVOID: Duration = Duration::from_secs(60);
/// Validated routes to peers we have spoken to, for a recovery dial that
/// needs no lookup: bounded, and forgotten after a while unused.
const ROUTE_CACHE_TTL: Duration = Duration::from_secs(600);
const ROUTE_CACHE_PEERS: usize = MAX_ESTABLISHED_CONNECTIONS as usize;
const ROUTE_CACHE_ADDRS: usize = 8;

struct PeerRoutes {
    updated: Instant,
    addresses: Vec<Multiaddr>,
    avoid_quic_until: Option<Instant>,
}

struct PeerRecovery {
    started: Instant,
    next_dial: Instant,
    looked_up: bool,
    /// The peer rode the tunnel when its connection dropped. Its punch
    /// failed once already, so the reconnect goes straight back to the
    /// tunnel instead of waiting out a second failure.
    tunneled: bool,
}

/// An invite (offer or reply) held for a connection, with how many sends it
/// has already survived.
struct PendingInvite {
    msg: InviteMsg,
    queued: Instant,
    attempts: u8,
}

fn direct_quic(addr: &Multiaddr) -> bool {
    addr.iter().any(|p| matches!(p, Protocol::QuicV1))
        && !addr.iter().any(|p| matches!(p, Protocol::P2pCircuit))
}

/// TCP to the peer first, then a route through a relay, then QUIC: after a
/// QUIC path died, the others are the ones most likely to be alive.
fn route_priority(addr: &Multiaddr) -> u8 {
    if addr.iter().any(|p| matches!(p, Protocol::P2pCircuit)) {
        1
    } else if direct_quic(addr) {
        2
    } else {
        0
    }
}

/// The address a connection can be redialled on: what we dialled, or a direct
/// QUIC remote (an inbound TCP remote carries an ephemeral source port).
fn reconnect_address(endpoint: &libp2p::core::ConnectedPoint) -> Option<Multiaddr> {
    (endpoint.is_dialer() || direct_quic(endpoint.get_remote_address()))
        .then(|| endpoint.get_remote_address().clone())
}

fn remember_routes(
    cache: &mut HashMap<PeerId, PeerRoutes>,
    peer: PeerId,
    addresses: impl IntoIterator<Item = Multiaddr>,
    now: Instant,
) {
    cache.retain(|_, routes| now.saturating_duration_since(routes.updated) < ROUTE_CACHE_TTL);
    for addr in addresses {
        let usable = addr.len() <= 512
            && addr.iter().any(|p| matches!(p, Protocol::Tcp(n) if n != 0) || matches!(p, Protocol::QuicV1))
            && !addr.iter().any(|p| {
                matches!(p, Protocol::Ws(_) | Protocol::Wss(_) | Protocol::WebTransport)
                    || matches!(p, Protocol::Ip4(ip) if ip.is_unspecified())
                    || matches!(p, Protocol::Ip6(ip) if ip.is_unspecified())
                    || matches!(p, Protocol::Tcp(0) | Protocol::Udp(0))
            })
            && !matches!(addr.iter().last(), Some(Protocol::P2p(id)) if id != peer);
        if !usable {
            continue;
        }
        if !cache.contains_key(&peer) && cache.len() >= ROUTE_CACHE_PEERS {
            if let Some(oldest) = cache.iter().min_by_key(|(_, r)| r.updated).map(|(p, _)| *p) {
                cache.remove(&oldest);
            }
        }
        let routes = cache.entry(peer).or_insert_with(|| PeerRoutes {
            updated: now,
            addresses: Vec::new(),
            avoid_quic_until: None,
        });
        routes.updated = now;
        routes.addresses.retain(|old| old != &addr);
        routes.addresses.insert(0, addr);
        routes.addresses.sort_by_key(route_priority);
        // Many local NIC addresses must not crowd out the relay or the
        // working QUIC endpoint: one entry of each path kind is kept first.
        let mut distinct: Vec<Multiaddr> = (0..3)
            .filter_map(|priority| routes.addresses.iter().find(|a| route_priority(a) == priority).cloned())
            .collect();
        for addr in routes.addresses.drain(..) {
            if distinct.len() < ROUTE_CACHE_ADDRS && !distinct.contains(&addr) {
                distinct.push(addr);
            }
        }
        routes.addresses = distinct;
    }
}

fn reconnect_routes(routes: Option<&PeerRoutes>, now: Instant) -> Vec<Multiaddr> {
    let Some(routes) = routes.filter(|r| now.saturating_duration_since(r.updated) < ROUTE_CACHE_TTL)
    else {
        return Vec::new();
    };
    let alternatives = routes.addresses.iter().any(|a| !direct_quic(a));
    let avoid_quic = alternatives && routes.avoid_quic_until.is_some_and(|at| now < at);
    let mut addresses: Vec<Multiaddr> =
        routes.addresses.iter().filter(|a| !avoid_quic || !direct_quic(a)).cloned().collect();
    addresses.sort_by_key(route_priority);
    addresses
}
/// How often the deferred-message queues (friend messages, invites) are
/// swept. This is only the polling cadence: a peer that stays away is
/// dialled and looked up on the `PEER_LOOKUP_FIRST`..`PEER_LOOKUP_MAX`
/// backoff, so its first retry comes 30 s (+-20 %) after the first failure,
/// not 15 s. The sweep itself is quiet; what it does cost is a dial.
const FRIEND_RETRY_INTERVAL: Duration = Duration::from_secs(15);
/// A friend message may ride a relayed connection that has to be set up first
/// (reservation, circuit, noise); the library's 10 s covered a warm LAN dial.
const FRIEND_REQUEST_TIMEOUT: Duration = Duration::from_secs(30);
/// Datagrams held per peer while its game-plane punch completes, and how long
/// they stay worth sending.
///
/// A title resolves a peer's address and immediately sends to it -- Fable 2
/// fires its join handshake in the same millisecond as XNetXnAddrToInAddr --
/// but the punch needs a round trip. Dropping what arrives in that window
/// loses the *first* message of a conversation, which is invariably the one
/// that starts it, and titles rarely retry a join handshake. The cap bounds a
/// peer that never punches; the age bound stops a stale handshake arriving
/// long after the title gave up, which would confuse it more than silence.
const PUNCH_QUEUE_MAX: usize = 32;
/// Must exceed PUNCH_TIMEOUT. Held datagrams are flushed when the punch
/// resolves -- either by succeeding, or by timing out into the tunnel at
/// PUNCH_TIMEOUT. With this shorter than that timeout, a slow punch meant
/// every queued frame aged out microseconds before the tunnel could carry it,
/// so the fallback delivered nothing and the join handshake was lost anyway.
const PUNCH_QUEUE_MAX_AGE: Duration = Duration::from_secs(12);
/// How long to wait for a punch before falling back to the tunnel (§14).
/// Slightly longer than the probe run (PROBE_ROUNDS * PROBE_INTERVAL) so a
/// late-but-successful punch is still preferred over the degraded path.
const PUNCH_TIMEOUT: Duration = Duration::from_secs(6);
// Invariant: PUNCH_QUEUE_MAX_AGE > PUNCH_TIMEOUT, or the tunnel fallback has
// nothing left to send by the time it engages.
const _: () = assert!(
    PUNCH_QUEUE_MAX_AGE.as_secs() > PUNCH_TIMEOUT.as_secs(),
    "queued datagrams must outlive the punch timeout"
);
// Invariant: a reconnect inside the grace re-punches, and that punch must be
// allowed to resolve (succeed or fall back) before recovery gives up.
const _: () = assert!(
    PEER_RECOVERY_GRACE.as_secs() > PUNCH_TIMEOUT.as_secs(),
    "recovery grace must outlast a punch by at least a second"
);
/// How often to check for punches that have run out of time.
const PUNCH_WATCHDOG_INTERVAL: Duration = Duration::from_secs(1);
/// How often to print the one-line shard summary. Frequent enough to watch a
/// shard form, rare enough not to become noise itself.
const SHARD_STATUS_INTERVAL: Duration = Duration::from_secs(30);

/// A datagram waiting for its peer's punch to resolve.
struct PendingDatagram {
    queued_at: Instant,
    src_port: u16,
    dst_port: u16,
    data: Vec<u8>,
}

/// How long and how often both sides blast punch probes (§8.4: up to 5 s).
const PROBE_INTERVAL: Duration = Duration::from_millis(100);
const PROBE_ROUNDS: u32 = 50;

/// Commands enqueued by the FFI layer (game thread side, non-blocking).
/// The oneshot-carrying variants are Rust-level (CLI/tests); the FFI surface
/// never constructs them.
// Several payloads are carried but unread until the §7 protocols land.
#[allow(dead_code)]
#[derive(Debug)]
pub enum Command {
    SetPresence { title_id: u32, state: u8, rich: Vec<u8> },
    /// Change the self-asserted display name at runtime (overlay rename);
    /// re-pushes presence so friends see it immediately.
    SetDisplayName { name: String },
    ConnectPeer { peer: PeerId },
    ConnectManual { multiaddr: String },
    /// Invite `peer` to the current local session (§8.3).
    SendInvite { peer: PeerId },
    /// Reply to a received invite. Accepting also fetches the host's
    /// session descriptor so the shim can build join material.
    InviteReply { peer: PeerId, accept: bool },
    FriendRequest { peer: PeerId, note: String },
    FriendAccept { peer: PeerId },
    FriendDismiss { peer: PeerId },
    FriendRemove { peer: PeerId },
    /// Enter a named shard outright (an invite carried its id).
    ShardJoin { shard_id: [u8; 16] },
    /// Register the local session (id allocated by the shim). Public
    /// sessions additionally publish a DHT provider record (§10).
    SessionCreate { id: [u8; 16], slots_total: u8, slots_open: u8, is_public: bool },
    SessionDelete,
    /// Set the local session's join token (0 = none): a query carrying it is
    /// served a private descriptor and its sender is remembered as invited.
    SessionSetJoinToken { token: u64 },
    /// Update the slots served in the local descriptor.
    SessionSetSlots { open: u32, total: u32 },
    /// Find public sessions for our title via DHT provider records, then
    /// query each advertiser; results arrive as [`Event::SessionFound`].
    SessionSearch,
    /// Ask one peer for its session descriptor, presenting `token` (0 =
    /// none) for a private one.
    SessionQuery { peer: PeerId, token: u64 },
    /// Opt in to the ambient title shard (§17.3, §17.5). Idempotent; a cap of
    /// 0 means the spec default.
    ShardEnable { cap: u16 },
    /// Opt out: leave any shard and stop publishing/scanning.
    ShardDisable,
    /// Which shard we are in, if any. Rust-level (CLI/tests/diagnostics).
    ShardStatus { reply: oneshot::Sender<Option<[u8; 16]>> },
    /// Guest TCP connect to a peer's virtual IP (§18).
    StreamConnect { virtual_ip: u32, src_port: u16, dst_port: u16 },
    /// Guest wrote to a connected stream socket.
    StreamSend { stream_id: StreamId, data: Vec<u8> },
    /// Guest closed a stream socket.
    StreamClose { stream_id: StreamId },
    SendDatagram { virtual_ip: u32, src_port: u16, dst_port: u16, data: Vec<u8>, reliable: bool },
    /// `reply` is None for fire-and-forget (FFI); results also surface as
    /// [`Event::PunchResult`].
    Punch { peer: PeerId, reply: Option<oneshot::Sender<Result<SocketAddr, String>>> },
    Echo { peer: PeerId, reply: oneshot::Sender<Result<Duration, String>> },
    FindPeer { peer: PeerId, reply: oneshot::Sender<Result<Vec<Multiaddr>, String>> },
    LocalAddrs { reply: oneshot::Sender<Vec<Multiaddr>> },

    Shutdown,
}

/// Events surfaced to the FFI layer, drained once per frame by the shim.
// Some variants unconstructed until the §7 protocols land.
#[allow(dead_code)]
#[derive(Debug)]
pub enum Event {
    /// `virtual_ip` is the 10.77.0.0/16 address allocated for this peer.
    PeerConnected { peer: PeerId, virtual_ip: u32 },
    PeerDisconnected { peer: PeerId },
    PresenceUpdated { peer: PeerId, title_id: u32, state: u8, display_name: String, rich: Vec<u8> },
    FriendRequest { peer: PeerId, display_name: String, note: String },
    /// Also emitted once per stored friend at engine start, so the shim's
    /// mirror converges without a separate list query.
    FriendAccepted { peer: PeerId },
    /// A request we sent was dismissed by the other side.
    FriendDeclined { peer: PeerId },
    /// A request we sent reached the other side (they acknowledged it).
    FriendDelivered { peer: PeerId },
    FriendRemoved { peer: PeerId },
    /// We entered a title shard (data = shard id).
    ShardJoined { shard_id: [u8; 16] },
    /// We left a title shard (data = shard id).
    ShardLeft { shard_id: [u8; 16] },
    InviteReceived { peer: PeerId, title_id: u32, session_id: [u8; 16] },
    /// reply: 1 = accepted, 0 = declined.
    InviteReplied { peer: PeerId, reply: u8 },
    /// An invite or reply to `peer` was not delivered after its retry, or
    /// waited out INVITE_DEFER_MAX. The host must hear it: nothing else says
    /// the invitee never saw the invite.
    InviteFailed { peer: PeerId, message: String },
    PunchResult { peer: PeerId, ok: bool },
    SessionFound {
        host: PeerId,
        session_id: [u8; 16],
        slots_total: u8,
        slots_open: u8,
        requires_invite: bool,
    },
    /// A guest TCP connection came up: either one we opened (`outbound`) or
    /// one accepted on a listening port.
    StreamOpened {
        stream_id: StreamId,
        virtual_ip: u32,
        local_port: u16,
        remote_port: u16,
        outbound: bool,
    },
    /// Bytes arrived on a guest TCP connection.
    StreamData { stream_id: StreamId, data: Vec<u8> },
    /// A guest TCP connection ended (either side, or the peer vanished).
    StreamClosed { stream_id: StreamId },
    /// An outbound guest TCP connect failed.
    StreamConnectFailed { virtual_ip: u32, dst_port: u16, message: String },
    /// Measured round trip to a peer, milliseconds. Emitted only on a
    /// material change; the shim caches it for QoS.
    PeerRtt { virtual_ip: u32, rtt_ms: u32 },
    /// Our own address on the virtual network changed — we joined, left or
    /// migrated shards (§17.3.5). The shim reports this as the local XNADDR.
    LocalAddress { virtual_ip: u32 },
    /// A directly dialable public endpoint for us was confirmed (UPnP mapping
    /// or AutoNAT). `addr` is a bare `host:port` the player can hand to a peer
    /// for a manual direct connect. Emitted on each newly confirmed endpoint.
    ExternalAddress { addr: String },
    /// A peer started or stopped carrying our traffic as a circuit relay
    /// (§17.2). Drives the `relay` label: a non-friend relay is the one
    /// non-friend the UI names at all, and it is named by role, not identity.
    RelayStatus { peer: PeerId, is_relay: bool },
    /// Ambient shard presence (§17.3.4). `display_name` is deliberately not
    /// carried: the shim renders a pseudonym unless the peer is a friend.
    ShardPresence {
        peer: PeerId,
        state: u8,
        rich: Vec<u8>,
        session_id: Option<[u8; 16]>,
    },
    Datagram { virtual_ip: u32, src_port: u16, dst_port: u16, data: Vec<u8> },
    /// Punch failed; game traffic is tunneled over the control connection.
    Degraded { peer: PeerId },
    Error { message: String },
}

#[allow(dead_code)] // data_dir/display_name unread until friend/presence state lands
pub struct EngineConfig {
    pub data_dir: PathBuf,
    pub title_id: u32,
    pub display_name: String,
    /// Kademlia bootstrap multiaddrs (must end in /p2p/<peer>). Empty is
    /// valid: LAN mDNS, manual connect strings, and direct v6 still work.
    pub bootstrap: Vec<String>,
    /// Fixed QUIC listen port; 0 = ephemeral.
    pub listen_port: u16,
    /// Fixed game-plane UDP port; 0 = ephemeral.
    pub game_port: u16,
    /// Circuit-v2 relays to hold reservations on. Several are held at once
    /// rather than one being chosen: a reservation takes a round trip to
    /// establish, so discovering a dead relay at the moment you need it is
    /// too late.
    ///
    /// Accelerant, never authority (§9). An empty list is fully supported and
    /// a configured relay that never answers must cost nothing but speed.
    pub relays: Vec<String>,
    /// Skip hole punching and carry all game traffic over the control tunnel
    /// (§14 degraded path).
    ///
    /// A CGNAT or symmetric-NAT pair reaches this state on its own, but nobody
    /// developing on a LAN ever does -- every local punch succeeds, so the
    /// fallback goes untested until it fails for a real player who cannot
    /// report why. This makes that path reproducible on any machine.
    pub force_tunnel: bool,
}

pub struct EngineHandles {
    pub cmd_tx: mpsc::UnboundedSender<Command>,
    pub evt_rx: mpsc::UnboundedReceiver<Event>,
    /// Pinged on every event; `rexnet_wait_event` blocks on it.
    pub evt_notify: Arc<Notify>,
    /// Game-plane fast path, shared with the FFI send call.
    pub plane: Arc<GamePlane>,
    pub peer_id: PeerId,
}

/// Echo payload for the milestone-1 connectivity proof.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct EchoPayload(Vec<u8>);

/// §10.2 session directory, carried over `/rexnet/session/1.0.0`.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SessionQuery {
    schema: u8, // = 1
    /// The host's join token, 0 for none. Defaulted on decode so a query
    /// from a build without the field still parses as "no token".
    #[serde(default)]
    token: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SessionDescMsg {
    schema: u8, // = 1
    session_id: [u8; 16],
    title_id: u32,
    slots_total: u8,
    slots_open: u8,
    requires_invite: bool,
}

struct LocalSession {
    id: [u8; 16],
    slots_total: u8,
    slots_open: u8,
    is_public: bool,
    /// Admits a querier to a private descriptor; 0 = none. Lives and dies
    /// with the session.
    join_token: u64,
}

/// §8.1 presence, pushed over `/rexnet/presence/1.0.0` to mutual friends
/// on change and heartbeated. The transport (Noise/TLS) authenticates the
/// sender; the spec's detached signature only matters once records are
/// relayed/gossiped, so it is deferred until then.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct PresenceMsg {
    schema: u8, // = 1
    seq: u64,
    display_name: String,
    title_id: u32,
    state: u8, // 0 offline, 1 online, 2 in-game, 3 joinable
    rich: Vec<u8>,
}

/// §8.2 friendship, over `/rexnet/friend/1.0.0`.
#[derive(Debug, Clone, Serialize, Deserialize)]
enum FriendMsg {
    Request { schema: u8, display_name: String, note: String },
    Accept { schema: u8 },
    /// The request was dismissed. The requester stops waiting for an Accept
    /// and stops re-sending; nothing stops them asking again.
    Decline { schema: u8 },
    Remove { schema: u8 },
}

/// §8.3 invites, over `/rexnet/invite/1.0.0`. Offer and Reply are separate
/// one-shot messages (the human decision happens between them).
#[derive(Debug, Clone, Serialize, Deserialize)]
enum InviteMsg {
    Offer { schema: u8, title_id: u32, session_id: [u8; 16], expires_unix: u64 },
    Reply { schema: u8, session_id: [u8; 16], accept: bool },
}

/// Generic delivery ack for one-shot protocol messages.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct Ack {
    schema: u8,
}

const PRESENCE_HEARTBEAT: Duration = Duration::from_secs(30);

/// §8.4 punch signaling, carried over `/rexnet/punch/1.0.0`. Offer/answer
/// map onto request-response; probes then run on the game socket itself.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct PunchOffer {
    nonce: [u8; 16],
    candidates: Vec<SocketAddr>,
    /// X25519 public key for game-plane encryption (§6). Safe to send here:
    /// the control connection is already authenticated and confidential.
    eph_pub: [u8; 32],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct PunchAnswer {
    nonce: [u8; 16],
    candidates: Vec<SocketAddr>,
    eph_pub: [u8; 32],
}

/// Ambient presence broadcast on a shard's gossipsub topic (§17.3.4).
///
/// Note what is **absent**: no display name. This reaches the whole shard,
/// most of whom are not friends, and §17.3.8 says a non-friend never learns
/// your self-asserted name. Receivers derive a pseudonym from the sender's
/// peer id instead; friends get the real name over the separate §8.1 push.
/// Adding a name field here would silently undo the privacy rule for every
/// title that enables shards.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct ShardBeat {
    schema: u8, // = 1
    /// Same vocabulary as presence (§8.1): 0 offline, 1 online, 2 in-game,
    /// 3 joinable.
    state: u8,
    /// Per-title ambient KVs -- orb positions and the like. Opaque to core.
    rich: Vec<u8>,
    /// Set when hosting something joinable, so shard members can join in
    /// progress without first becoming friends. Public sessions only: a
    /// private session is the host's to hand out, and the beat reaches the
    /// whole shard.
    session_id: Option<[u8; 16]>,
}

/// §17.3.1 shard descriptor query over `/rexnet/shard/1.0.0`. Asked of a
/// peer that advertised itself as a shard provider; `None` means it no longer
/// holds one (records outlive membership).
#[derive(Debug, Clone, Serialize, Deserialize)]
struct ShardQuery {
    schema: u8, // = 1
    title_id: u32,
}

/// Ceiling on concurrently established connections.
///
/// The public DHT crawl is unbounded by nature: it dials whatever routing
/// wants, and we were holding hundreds of connections to IPFS peers with no
/// relationship to any game. That churn drowned the log and, more seriously,
/// exhausted the inbound substream allowance ("maximum number of inbound
/// substreams attempts has been exceeded") — a limit that is per-connection
/// but reached far sooner when thousands of connections are cycling.
///
/// The budget only has to cover what RexNet actually needs: a warm shard is
/// capped at 24 members (SHARD_WARM_CONNECT_LIMIT), plus friends, plus a
/// working DHT routing table. 192 leaves generous headroom for all of that
/// while stopping the crawl from growing without bound.
const MAX_ESTABLISHED_CONNECTIONS: u32 = 192;
/// Inbound is capped lower than the total: an unsolicited peer must never be
/// able to fill the table and crowd out the outbound dials we chose to make
/// (shard members, friends, a peer we are joining).
const MAX_ESTABLISHED_INCOMING: u32 = 64;

/// Messages from the per-stream pump tasks back into the engine.
///
/// The tasks own the socket halves and do the blocking work; the engine only
/// keeps the registry and turns these into guest-facing events. Stream ids are
/// allocated by the tasks from a shared counter so no round trip is needed to
/// name a stream that has only just arrived.
#[derive(Debug)]
enum StreamEvent {
    Opened {
        id: StreamId,
        peer: PeerId,
        local_port: u16,
        remote_port: u16,
        outbound: bool,
        writer: mpsc::UnboundedSender<Vec<u8>>,
    },
    Data { id: StreamId, data: Vec<u8> },
    Closed { id: StreamId },
    ConnectFailed { peer: PeerId, dst_port: u16, error: String },
}

/// Largest guest read handed over in one event: the event payload less the
/// 8-byte stream id that prefixes it. A larger read would be truncated by
/// the fixed event, silently losing stream bytes; a smaller one just means
/// more events.
const STREAM_READ_CHUNK: usize = crate::ffi::REXNET_EVENT_DATA_MAX - 8;

/// Drive one guest connection: header already exchanged, now shuttle bytes
/// until either side closes.
async fn pump_stream(
    id: StreamId,
    mut stream: libp2p::Stream,
    mut outgoing: mpsc::UnboundedReceiver<Vec<u8>>,
    events: mpsc::UnboundedSender<StreamEvent>,
) {
    use libp2p::futures::io::{AsyncReadExt, AsyncWriteExt};
    let mut buf = vec![0u8; STREAM_READ_CHUNK];
    loop {
        tokio::select! {
            // Guest -> peer.
            outbound = outgoing.recv() => {
                match outbound {
                    Some(bytes) => {
                        if stream.write_all(&bytes).await.is_err() {
                            break;
                        }
                        if stream.flush().await.is_err() {
                            break;
                        }
                    }
                    // Sender dropped: the guest closed its socket.
                    None => break,
                }
            }
            // Peer -> guest.
            read = stream.read(&mut buf) => {
                match read {
                    Ok(0) => break,          // clean EOF
                    Ok(n) => {
                        if events
                            .send(StreamEvent::Data { id, data: buf[..n].to_vec() })
                            .is_err()
                        {
                            break;           // engine gone
                        }
                    }
                    Err(_) => break,
                }
            }
        }
    }
    let _ = stream.close().await;
    let _ = events.send(StreamEvent::Closed { id });
}

/// Accept inbound guest TCP connections for the lifetime of the engine.
///
/// Each stream opens with a header naming the destination guest port, so the
/// header is read here before the connection is announced -- the shim cannot
/// match it to a listening socket without knowing the port.
fn spawn_stream_acceptor(
    mut control: crate::stream::Control,
    events: mpsc::UnboundedSender<StreamEvent>,
    next_id: Arc<AtomicU64>,
) {
    let protocol = StreamProtocol::new(crate::protocol::TCP);
    let mut incoming = match control.accept(protocol) {
        Ok(incoming) => incoming,
        Err(err) => {
            tracing::error!(%err, "guest TCP: cannot accept streams");
            return;
        }
    };
    tokio::spawn(async move {
        use libp2p::futures::StreamExt;
        while let Some((peer, mut stream)) = incoming.next().await {
            let events = events.clone();
            let next_id = next_id.clone();
            tokio::spawn(async move {
                use libp2p::futures::io::AsyncReadExt;
                let mut header = [0u8; STREAM_HEADER_LEN];
                if stream.read_exact(&mut header).await.is_err() {
                    return;
                }
                let Some(header) = StreamHeader::decode(&header) else {
                    tracing::debug!(%peer, "guest TCP: bad stream header");
                    return;
                };
                let id = next_id.fetch_add(1, Ordering::Relaxed).max(1);
                let (writer_tx, writer_rx) = mpsc::unbounded_channel::<Vec<u8>>();
                // The initiator's dst_port is the port we are listening on;
                // its src_port is the far side from our point of view.
                if events
                    .send(StreamEvent::Opened {
                        id,
                        peer,
                        local_port: header.dst_port,
                        remote_port: header.src_port,
                        outbound: false,
                        writer: writer_tx,
                    })
                    .is_err()
                {
                    return;
                }
                pump_stream(id, stream, writer_rx, events).await;
            });
        }
    });
}

// --- Degraded game plane (§5, §14) ---
//
// One stream per peer per direction: we write on ours, read from theirs. Both
// sides degrade at once here, so a shared stream would need a tie-break.

/// Messages from the tunnel tasks back into the engine.
#[derive(Debug)]
enum TunnelEvent {
    /// A datagram arrived over a peer's tunnel.
    Frame { peer: PeerId, src_port: u16, dst_port: u16, data: Vec<u8> },
    /// Our outbound tunnel to this peer went away (opened and later died, or
    /// never opened at all). Either way the writer is dead and a later send
    /// has to open a fresh one.
    WriterGone { peer: PeerId, error: Option<String> },
}

/// Bounded: the guest thinks it is sending UDP, so congestion should drop
/// rather than grow memory and latency.
const TUNNEL_WRITE_QUEUE: usize = 64;

/// Write queued frames to a peer's tunnel stream until it dies.
async fn pump_tunnel_writer(
    peer: PeerId,
    mut stream: libp2p::Stream,
    mut outgoing: mpsc::Receiver<Vec<u8>>,
    events: mpsc::UnboundedSender<TunnelEvent>,
) {
    use libp2p::futures::io::AsyncWriteExt;
    while let Some(frame) = outgoing.recv().await {
        if stream.write_all(&frame).await.is_err() {
            break;
        }
        // Per frame: a datagram held for company is a datagram delivered late.
        if stream.flush().await.is_err() {
            break;
        }
    }
    let _ = stream.close().await;
    let _ = events.send(TunnelEvent::WriterGone { peer, error: None });
}

/// Open the outbound tunnel to `peer` and start writing to it.
fn spawn_tunnel_writer(
    mut control: crate::stream::Control,
    peer: PeerId,
    outgoing: mpsc::Receiver<Vec<u8>>,
    events: mpsc::UnboundedSender<TunnelEvent>,
) {
    tokio::spawn(async move {
        let protocol = StreamProtocol::new(crate::protocol::TUNNEL);
        match control.open_stream(peer, protocol).await {
            Ok(stream) => {
                tracing::info!(%peer, "tunnel stream open (degraded game plane)");
                pump_tunnel_writer(peer, stream, outgoing, events).await;
            }
            Err(err) => {
                let _ = events.send(TunnelEvent::WriterGone {
                    peer,
                    error: Some(err.to_string()),
                });
            }
        }
    });
}

/// Read framed datagrams from one peer's tunnel until it closes.
async fn pump_tunnel_reader(
    peer: PeerId,
    mut stream: libp2p::Stream,
    events: mpsc::UnboundedSender<TunnelEvent>,
) {
    use libp2p::futures::io::{AsyncReadExt, AsyncWriteExt};
    let mut header = [0u8; tunnel::TUNNEL_HEADER_LEN];
    loop {
        if stream.read_exact(&mut header).await.is_err() {
            break; // EOF or the connection went away
        }
        let Some((src_port, dst_port, len)) = tunnel::decode_header(&header) else {
            // Framing is lost and a byte stream cannot resynchronise.
            tracing::warn!(%peer, "tunnel: bad frame header, closing stream");
            break;
        };
        let mut data = vec![0u8; len];
        if stream.read_exact(&mut data).await.is_err() {
            break;
        }
        if events.send(TunnelEvent::Frame { peer, src_port, dst_port, data }).is_err() {
            break; // engine gone
        }
    }
    let _ = stream.close().await;
}

/// Accept inbound tunnels for the lifetime of the engine.
fn spawn_tunnel_acceptor(
    mut control: crate::stream::Control,
    events: mpsc::UnboundedSender<TunnelEvent>,
) {
    let protocol = StreamProtocol::new(crate::protocol::TUNNEL);
    let mut incoming = match control.accept(protocol) {
        Ok(incoming) => incoming,
        Err(err) => {
            tracing::error!(%err, "tunnel: cannot accept streams");
            return;
        }
    };
    tokio::spawn(async move {
        use libp2p::futures::StreamExt;
        while let Some((peer, stream)) = incoming.next().await {
            tracing::info!(%peer, "peer opened a tunnel to us (their punch failed)");
            tokio::spawn(pump_tunnel_reader(peer, stream, events.clone()));
        }
    });
}

#[derive(NetworkBehaviour)]
struct Behaviour {
    /// Must stay first: the derive polls fields in order, so the limiter sees
    /// (and can deny) a connection before other behaviours act on it.
    limits: connection_limits::Behaviour,
    identify: identify::Behaviour,
    kad: kad::Behaviour<MemoryStore>,
    mdns: mdns::tokio::Behaviour,
    autonat: autonat::Behaviour,
    /// Automatic NAT port mapping (§9 torrent-model reachability). On a
    /// UPnP/IGD router this opens the listen port unattended, so a peer becomes
    /// directly dialable without hand-forwarding anything -- the "seed if you
    /// can" half of the mesh.
    upnp: upnp::tokio::Behaviour,
    relay_client: relay::client::Behaviour,
    /// Circuit-v2 relay *server* (§9 torrent model): every node offers to relay
    /// for others, so a publicly reachable peer brokers rendezvous + DCUtR for
    /// NAT'd peers with no dedicated server. A NAT'd node still runs it, but no
    /// one can reach it to reserve, so relaying is self-selecting -- reachable
    /// peers seed, firewalled peers cannot, exactly like a torrent swarm.
    /// Reservation/data/time caps in `relay::Config` bound what we carry.
    relay_server: relay::Behaviour,
    dcutr: dcutr::Behaviour,
    echo: request_response::cbor::Behaviour<EchoPayload, EchoPayload>,
    punch: request_response::cbor::Behaviour<PunchOffer, PunchAnswer>,
    session: request_response::cbor::Behaviour<SessionQuery, Option<SessionDescMsg>>,
    presence: request_response::cbor::Behaviour<PresenceMsg, Ack>,
    friend: request_response::cbor::Behaviour<FriendMsg, Ack>,
    invite: request_response::cbor::Behaviour<InviteMsg, Ack>,
    shard: request_response::cbor::Behaviour<ShardQuery, Option<ShardDesc>>,
    /// Guest TCP (§18) *and* the degraded game plane (§14). Raw libp2p
    /// streams rather than request-response protocols: a guest stream socket
    /// needs ordered bytes, and the tunnel needs a carrier whose cost does
    /// not scale with datagram rate.
    stream: crate::stream::Behaviour,
    /// Ambient shard broadcast (§17.3.4). Deliberately gossip rather than a
    /// mesh of direct links: 255 peers fully connected is ~32k connections.
    gossipsub: gossipsub::Behaviour,
}

/// Build the swarm and spawn the engine task onto the current tokio runtime.
/// Must be called from within a runtime context (the FFI wraps this in
/// `Runtime::block_on`).
pub async fn spawn(keypair: Keypair, config: EngineConfig) -> Result<EngineHandles, BoxError> {
    let peer_id = keypair.public().to_peer_id();

    let mut swarm = libp2p::SwarmBuilder::with_existing_identity(keypair)
        .with_tokio()
        .with_tcp(
            libp2p::tcp::Config::default().nodelay(true),
            libp2p::noise::Config::new,
            libp2p::yamux::Config::default,
        )?
        .with_quic()
        .with_dns()?
        .with_relay_client(libp2p::noise::Config::new, libp2p::yamux::Config::default)?
        .with_behaviour(|key, relay_client| {
            let local_id = key.public().to_peer_id();
            Ok(Behaviour {
                limits: connection_limits::Behaviour::new(
                    connection_limits::ConnectionLimits::default()
                        .with_max_established(Some(MAX_ESTABLISHED_CONNECTIONS))
                        .with_max_established_incoming(Some(MAX_ESTABLISHED_INCOMING))
                        // One connection per peer is all we ever need, and it
                        // stops a peer opening many in parallel.
                        .with_max_established_per_peer(Some(2)),
                ),
                identify: identify::Behaviour::new(
                    identify::Config::new("rexnet/0.1.0".into(), key.public())
                        .with_agent_version(format!("rexnet-core/{}", env!("CARGO_PKG_VERSION"))),
                ),
                // Amino-compatible protocol id so public-DHT FindPeer works
                // (spec §4). Client mode: we don't serve records for the
                // public DHT (server-mode etiquette is an open question, §16).
                kad: kad::Behaviour::with_config(
                    local_id,
                    MemoryStore::new(local_id),
                    kad::Config::new(kad::PROTOCOL_NAME),
                ),
                mdns: mdns::tokio::Behaviour::new(mdns::Config::default(), local_id)?,
                autonat: autonat::Behaviour::new(local_id, autonat::Config::default()),
                upnp: upnp::tokio::Behaviour::default(),
                relay_client,
                relay_server: relay::Behaviour::new(local_id, relay::Config::default()),
                dcutr: dcutr::Behaviour::new(local_id),
                echo: request_response::cbor::Behaviour::new(
                    [(
                        StreamProtocol::new(crate::protocol::ECHO),
                        ProtocolSupport::Full,
                    )],
                    request_response::Config::default(),
                ),
                punch: request_response::cbor::Behaviour::new(
                    [(
                        StreamProtocol::new(crate::protocol::PUNCH),
                        ProtocolSupport::Full,
                    )],
                    request_response::Config::default(),
                ),
                session: request_response::cbor::Behaviour::new(
                    [(
                        StreamProtocol::new(crate::protocol::SESSION),
                        ProtocolSupport::Full,
                    )],
                    request_response::Config::default(),
                ),
                presence: request_response::cbor::Behaviour::new(
                    [(
                        StreamProtocol::new(crate::protocol::PRESENCE),
                        ProtocolSupport::Full,
                    )],
                    request_response::Config::default(),
                ),
                friend: request_response::cbor::Behaviour::new(
                    [(
                        StreamProtocol::new(crate::protocol::FRIEND),
                        ProtocolSupport::Full,
                    )],
                    request_response::Config::default()
                        .with_request_timeout(FRIEND_REQUEST_TIMEOUT),
                ),
                invite: request_response::cbor::Behaviour::new(
                    [(
                        StreamProtocol::new(crate::protocol::INVITE),
                        ProtocolSupport::Full,
                    )],
                    request_response::Config::default(),
                ),
                shard: request_response::cbor::Behaviour::new(
                    [(
                        StreamProtocol::new(crate::protocol::SHARD),
                        ProtocolSupport::Full,
                    )],
                    request_response::Config::default(),
                ),
                stream: crate::stream::Behaviour::new(),
                gossipsub: gossipsub::Behaviour::new(
                    gossipsub::MessageAuthenticity::Signed(key.clone()),
                    gossipsub::Config::default(),
                )
                .map_err(|err| err.to_string())?,
            })
        })?
        .with_swarm_config(|c| c.with_idle_connection_timeout(Duration::from_secs(60)))
        .build();

    // On the public Amino DHT we stay in client mode (server-mode etiquette
    // is an open question, §16). With no public bootstrap — LAN or private
    // mesh — every node serves, so provider records work peer-to-peer.
    let kad_mode = if config.bootstrap.is_empty() {
        kad::Mode::Server
    } else {
        kad::Mode::Client
    };
    swarm.behaviour_mut().kad.set_mode(Some(kad_mode));

    let port = config.listen_port;
    for addr in [
        format!("/ip4/0.0.0.0/udp/{port}/quic-v1"),
        format!("/ip6/::/udp/{port}/quic-v1"),
        "/ip4/0.0.0.0/tcp/0".to_string(),
    ] {
        let addr: Multiaddr = addr.parse()?;
        if let Err(err) = swarm.listen_on(addr.clone()) {
            tracing::warn!(%addr, %err, "listen failed");
        }
    }

    let mut bootstrap_peers: HashSet<PeerId> = HashSet::new();
    for entry in &config.bootstrap {
        match entry.parse::<Multiaddr>() {
            Ok(addr) => match addr.iter().last() {
                Some(Protocol::P2p(peer)) => {
                    bootstrap_peers.insert(peer);
                    swarm.behaviour_mut().kad.add_address(&peer, addr.clone());
                }
                _ => tracing::warn!(%addr, "bootstrap addr lacks /p2p/ suffix, skipped"),
            },
            Err(err) => tracing::warn!(entry, %err, "bad bootstrap multiaddr, skipped"),
        }
    }
    if !config.bootstrap.is_empty() {
        if let Err(err) = swarm.behaviour_mut().kad.bootstrap() {
            tracing::warn!(%err, "kad bootstrap failed to start");
        }
    }

    // Deliberately non-fatal, all of it. A relay that is unparseable, down,
    // or withdrawn must not stop the engine starting -- that would make hosted
    // infrastructure a dependency, which is the thing §9 exists to prevent.
    let mut relay_addrs: Vec<(PeerId, Multiaddr)> = Vec::new();
    for entry in &config.relays {
        let addr: Multiaddr = match entry.parse() {
            Ok(addr) => addr,
            Err(err) => {
                tracing::warn!(entry, %err, "bad relay multiaddr, skipped");
                continue;
            }
        };
        let Some(Protocol::P2p(peer)) = addr.iter().last() else {
            // Without a peer id we cannot tell whether its reservation is
            // live, so it could never be retried or reported.
            tracing::warn!(%addr, "relay addr lacks /p2p/, skipped");
            continue;
        };
        if let Err(err) = swarm.listen_on(addr.clone().with(Protocol::P2pCircuit)) {
            tracing::warn!(%addr, %err, "relay reservation could not be started");
        }
        relay_addrs.push((peer, addr));
    }
    if !relay_addrs.is_empty() {
        tracing::info!(count = relay_addrs.len(), "relays configured");
    }

    // Game-plane socket: one UDP socket multiplexing every guest port; the
    // punch establishes one peer endpoint per remote (§6). A reader task
    // forwards inbound packets so the main loop can select! without
    // borrowing the socket.
    let game_socket =
        Arc::new(UdpSocket::bind(("0.0.0.0", config.game_port)).await?);
    tracing::info!(game_port = game_socket.local_addr()?.port(), "game socket bound");
    if !config.force_tunnel {
        tokio::spawn(map_game_port(game_socket.local_addr()?.port()));
    }
    let (cmd_tx, cmd_rx) = mpsc::unbounded_channel::<Command>();
    let (evt_tx, evt_rx) = mpsc::unbounded_channel::<Event>();
    let evt_notify = Arc::new(Notify::new());
    let emitter = Emitter::new(evt_tx, evt_notify.clone());
    let plane = Arc::new(GamePlane::new(game_socket.clone(), emitter.clone()));

    // Ready peers' data frames are decrypted and delivered right here; only
    // probes and not-yet-mapped frames go through the engine.
    let (game_tx, game_rx) = mpsc::unbounded_channel::<(SocketAddr, Vec<u8>)>();
    {
        let socket = game_socket.clone();
        let plane = plane.clone();
        tokio::spawn(async move {
            let mut buf = vec![0u8; 2048];
            while let Ok((len, addr)) = socket.recv_from(&mut buf).await {
                if plane.try_receive(addr, &buf[..len]) {
                    continue;
                }
                if game_tx.send((addr, buf[..len].to_vec())).is_err() {
                    break;
                }
            }
        });
    }

    let friend_store = FriendStore::load(&config.data_dir);

    // Guest TCP (§18). The control opens outbound streams; the accept loop
    // below takes inbound ones. Both run outside the swarm task, so ids come
    // from a shared counter rather than a round trip.
    let stream_control = swarm.behaviour().stream.new_control();
    let (stream_tx, stream_rx) = mpsc::unbounded_channel::<StreamEvent>();
    let next_stream_id = Arc::new(AtomicU64::new(1));
    spawn_stream_acceptor(stream_control.clone(), stream_tx.clone(), next_stream_id.clone());

    // Degraded game plane (§14) rides the same stream behaviour; it accepts a
    // different protocol, so it needs its own control and accept loop.
    let tunnel_control = swarm.behaviour().stream.new_control();
    let (tunnel_tx, tunnel_rx) = mpsc::unbounded_channel::<TunnelEvent>();
    spawn_tunnel_acceptor(tunnel_control.clone(), tunnel_tx.clone());

    tokio::spawn(
        Engine {
            swarm,
            emitter,
            plane: plane.clone(),
            game_socket,
            title_id: config.title_id,
            display_name: config.display_name.clone(),
            friend_store,
            connected: HashSet::new(),
            wanted_peers: HashSet::new(),
            announced: HashSet::new(),
            presence_seq: 0,
            my_presence: PresenceMsg {
                schema: 1,
                seq: 0,
                display_name: config.display_name.clone(),
                title_id: config.title_id,
                state: 1, // online
                rich: Vec::new(),
            },
            announced_vip: None,
            invited: HashSet::new(),
            session_peers: HashMap::new(),
            pending_invites: HashMap::new(),
            inflight_invites: HashMap::new(),
            pending_friend_msgs: HashMap::new(),
            inflight_friend: HashMap::new(),
            unsupported_reported: HashSet::new(),
            peer_lookups: HashMap::new(),
            own_key_query: None,
            own_key_at: None,
            lookup_retry_at: HashMap::new(),
            bootstrap_peers,
            auto_relay_backoff: HashMap::new(),
            auto_relay_listeners: HashMap::new(),
            peer_routes: HashMap::new(),
            recovering: HashMap::new(),
            local_session: None,
            pending_session_search: None,
            relays: HashSet::new(),
            relay_addrs,
            relay_retry_at: HashMap::new(),
            rexnet_peers: HashSet::new(),
            shard_enabled: false,
            shard_cap: DEFAULT_SHARD_CAP,
            current_shard: None,
            shard_candidates: HashMap::new(),
            shard_preferred: None,
            pending_shard_search: None,
            pending_shard_queries: HashMap::new(),
            shard_member_peers: HashSet::new(),
            shard_member_heard: HashMap::new(),
            shard_hosts: std::collections::BTreeMap::new(),
            readdress_deferred: false,
            shard_reachable_attempted: HashSet::new(),
            shard_swept_at: None,
            last_shard_move: None,
            listen_addrs: Vec::new(),
            external_ips: Vec::new(),
            announced_external: HashSet::new(),
            auto_relays: HashSet::new(),
            relay_provider_registered: false,
            pending_relay_search: None,
            vip_by_peer: HashMap::new(),
            peer_by_vip: HashMap::new(),
            pending_datagrams: HashMap::new(),
            orphan_frames: HashMap::new(),
            punch_deadline: HashMap::new(),
            streams: StreamTable::new(),
            stream_writers: HashMap::new(),
            stream_control,
            next_stream_id,
            stream_tx,
            force_tunnel: config.force_tunnel,
            tunneled_peers: HashSet::new(),
            peer_rtt_ms: HashMap::new(),
            tunnel_control,
            tunnel_writers: HashMap::new(),
            tunnel_tx,
            tunnel_drops: HashMap::new(),
            peer_game_pubs: HashMap::new(),
            key_agreements: HashMap::new(),
            pending_punch_nonce: HashMap::new(),
            punch_replies: HashMap::new(),
            pending_punch_offers: HashMap::new(),
            pending_find: HashMap::new(),
            pending_echo: HashMap::new(),
            unknown_vip_warns: WarnLimiter::new(PACKET_WARN_WINDOW),
            no_keys_warns: WarnLimiter::new(PACKET_WARN_WINDOW),
        }
        .run(cmd_rx, game_rx, stream_rx, tunnel_rx),
    );

    tracing::info!(%peer_id, title_id = %format!("{:08X}", config.title_id), "rexnet engine up");
    Ok(EngineHandles { cmd_tx, evt_rx, evt_notify, plane, peer_id })
}

struct Engine {
    swarm: Swarm<Behaviour>,
    emitter: Emitter,
    /// Punched endpoints, session keys and the vip table live here so the
    /// fast paths can read them; this loop is their only writer.
    plane: Arc<GamePlane>,
    game_socket: Arc<UdpSocket>,
    title_id: u32,
    display_name: String,
    friend_store: FriendStore,
    connected: HashSet<PeerId>,
    /// Peers the app explicitly asked to connect to; only their dial
    /// failures surface as Event::Error — the kad crawl constantly fails
    /// against unreachable public peers and stays at tracing level.
    wanted_peers: HashSet<PeerId>,
    /// Peers surfaced to the app via PeerConnected. Gates the vip pool and
    /// the peer table to app-relevant peers; kad-crawl connections to
    /// random public DHT nodes never enter either.
    announced: HashSet<PeerId>,
    presence_seq: u64,
    my_presence: PresenceMsg,
    /// Last address handed to the shim, so re-announcing is free.
    announced_vip: Option<u32>,
    /// Peers admitted to the current session's descriptor even when it is
    /// private: those we invited, and those whose query carried the join
    /// token. Cleared with the session, both on create and delete.
    invited: HashSet<PeerId>,
    /// Peers in a session with us by invite -- ours they accepted, or theirs
    /// we accepted -- and when. Counts toward `game_in_progress` for
    /// INVITE_DEFER_MAX; after that title traffic has to speak for itself.
    session_peers: HashMap<PeerId, Instant>,
    /// Invites waiting for a connection. An invite is usually the *first*
    /// thing we ever send a peer, so at that moment there is frequently no
    /// connection and often no known address either -- sending blind meant
    /// the first invite reliably failed and the user had to try twice.
    /// Replies take the same path: the inviter's connection may have gone
    /// while the human decided.
    pending_invites: HashMap<PeerId, PendingInvite>,
    /// Invites in flight, so a failure can re-queue once and then be told.
    inflight_invites: HashMap<OutboundRequestId, (PeerId, PendingInvite)>,
    /// Friend messages waiting for a connection, exactly as invites
    /// wait above. A friend request is the FIRST thing we ever send a peer -- a
    /// friend code carries a key and a name, no address -- so sending it blind
    /// failed at trace level and the player saw "request sent" for a message that
    /// never left. Held until the peer connects (shard scan, DHT lookup, mdns or
    /// a relay), retried on every heartbeat, never silently dropped.
    pending_friend_msgs: HashMap<PeerId, (FriendMsg, Instant)>,
    /// Friend messages in flight, so an outbound failure can put the
    /// message back in the queue instead of losing it.
    inflight_friend: HashMap<OutboundRequestId, (PeerId, FriendMsg)>,
    /// (peer, protocol) pairs already reported as an older build, so the
    /// player hears it once and not on every retry.
    unsupported_reported: HashSet<(PeerId, &'static str)>,
    /// DHT lookups for one peer's own reachability record, by query.
    peer_lookups: HashMap<kad::QueryId, PeerId>,
    /// The publish of our own record in flight, and when one last stuck.
    own_key_query: Option<kad::QueryId>,
    own_key_at: Option<Instant>,
    /// Per-peer backoff for lookups and dials of peers that stay away.
    lookup_retry_at: HashMap<PeerId, (Instant, Duration)>,
    /// Configured bootstrap nodes: never relays, never friends, never announced.
    bootstrap_peers: HashSet<PeerId>,
    /// Relays that refused or dropped a reservation, with the time to try again.
    auto_relay_backoff: HashMap<PeerId, Instant>,
    /// Circuit listeners we hold, by listener, so a closed one frees its relay.
    auto_relay_listeners: HashMap<libp2p::core::transport::ListenerId, PeerId>,
    /// Validated routes to peers, for recovery dials.
    peer_routes: HashMap<PeerId, PeerRoutes>,
    /// Game peers whose transport dropped and are being brought back.
    recovering: HashMap<PeerId, PeerRecovery>,
    local_session: Option<LocalSession>,
    /// Active session-search provider query, with peers already asked.
    pending_session_search: Option<(kad::QueryId, Vec<PeerId>)>,

    // --- Ambient title shard, §17.3 -------------------------------------
    /// Peers currently carrying our traffic as a circuit relay (§9, §17.2).
    relays: HashSet<PeerId>,
    /// Configured relays and their addresses, for retrying a reservation that
    /// never came up or has since lapsed.
    relay_addrs: Vec<(PeerId, Multiaddr)>,
    /// When each relay was last dialled, so a dead one is retried slowly
    /// rather than hammered.
    relay_retry_at: HashMap<PeerId, Instant>,
    /// Connected peers whose identify shows they speak our shard protocol --
    /// i.e. other RexNet nodes, as opposed to the public DHT peers we are
    /// also connected to. Asked directly during shard discovery.
    rexnet_peers: HashSet<PeerId>,
    /// Opt-in per title (§17.5); false means we never join or publish.
    shard_enabled: bool,
    shard_cap: u16,
    /// The shard we belong to, if any. We publish a provider record for it
    /// and are subscribed to its topic.
    current_shard: Option<ShardDesc>,
    /// Descriptors learned from the last sweep, keyed by shard id.
    shard_candidates: HashMap<[u8; 16], ShardDesc>,
    /// A shard the player asked for by id (`Command::ShardJoin`); entered as
    /// soon as its descriptor is known, cleared once we are in it.
    shard_preferred: Option<[u8; 16]>,
    pending_shard_search: Option<(kad::QueryId, Vec<PeerId>)>,
    pending_shard_queries: HashMap<OutboundRequestId, PeerId>,
    /// Peer ids heard on our shard topic. Larger than the gossipsub mesh:
    /// these are everyone in the shard, not just who we relay through.
    /// Pruned on unsubscribe, departure, and SHARD_MEMBER_TIMEOUT silence.
    shard_member_peers: HashSet<PeerId>,
    /// When each member was last heard (beat or descriptor), for the prune.
    shard_member_heard: HashMap<PeerId, Instant>,
    /// The shard's host table, ourselves included: rebuilt from the whole
    /// membership by `readdress_all`, extended one newcomer at a time
    /// without moving anyone while a game is up. Every shard address is
    /// read from here, never re-derived on the spot, so a membership change
    /// can only move an address when this table is rebuilt.
    shard_hosts: std::collections::BTreeMap<PeerId, u8>,
    /// Membership changed while addresses were pinned by a game; the full
    /// table is rebuilt once no game is in progress.
    readdress_deferred: bool,
    /// Shard members we have already tried to make reachable, so a 30 s
    /// heartbeat does not re-dial or re-query them every time.
    shard_reachable_attempted: HashSet<PeerId>,
    /// Set once a provider sweep has completed. Guards shard *creation*:
    /// without it the first rescan tick (which fires immediately) would
    /// create a shard before discovery had a chance to answer, so every node
    /// would found its own and the population would start fully fragmented.
    /// When the last provider sweep finished, or `None` if none has. Doubles
    /// as the swept flag so the two cannot drift apart.
    shard_swept_at: Option<Instant>,
    /// Migration rate limiting (§17.3.3). Direction is unconditional, so this
    /// is the only thing standing between a rescan and a stampede.
    last_shard_move: Option<Instant>,
    listen_addrs: Vec<Multiaddr>,
    external_ips: Vec<IpAddr>,
    /// Bare `host:port` endpoints already surfaced via [`Event::ExternalAddress`],
    /// so a re-confirmation of the same address does not re-emit.
    announced_external: HashSet<String>,
    /// Mesh peers we hold an automatic circuit reservation on (§9 torrent
    /// model), distinct from configured relays. Capped at [`MAX_AUTO_RELAYS`].
    auto_relays: HashSet<PeerId>,
    /// Whether we have published a `rexnet/v1/relays` provider record (§9). Set
    /// once we are confirmed reachable, so NAT'd peers can find us as a relay.
    relay_provider_registered: bool,
    /// In-flight `get_providers(rexnet/v1/relays)` query, so its result routes
    /// to relay handling rather than the session/shard registries.
    pending_relay_search: Option<kad::QueryId>,

    // Virtual-IP allocation (authoritative; the C++ shim mirrors it).
    vip_by_peer: HashMap<PeerId, u32>,
    peer_by_vip: HashMap<u32, PeerId>,

    // Punched game-plane endpoints.
    /// Datagrams waiting for a peer's punch to resolve, oldest first. Held
    /// unframed so they can leave by either the UDP game plane or the tunnel.
    pending_datagrams: HashMap<PeerId, Vec<PendingDatagram>>,
    /// Inbound frames from an endpoint we have not mapped to a peer yet,
    /// keyed by source address. Delivered once the punch completes.
    orphan_frames: HashMap<SocketAddr, Vec<(Instant, Vec<u8>)>>,
    /// When each in-flight punch stops being worth waiting for.
    punch_deadline: HashMap<PeerId, Instant>,
    /// Live guest TCP connections (§18).
    streams: StreamTable,
    /// Write half of each live stream, keyed the same way.
    stream_writers: HashMap<StreamId, mpsc::UnboundedSender<Vec<u8>>>,
    /// Handle used to open outbound streams; cloned into spawned tasks.
    stream_control: crate::stream::Control,
    /// Shared id allocator so a pump task can name a stream without a round
    /// trip back to the engine.
    next_stream_id: Arc<AtomicU64>,
    /// Pump tasks report here.
    stream_tx: mpsc::UnboundedSender<StreamEvent>,
    /// Punching disabled; everything takes the degraded path (§14).
    force_tunnel: bool,
    /// Peers whose punch failed and whose traffic now rides the control
    /// connection (§14). Cleared if a later punch (or DCUtR upgrade) succeeds.
    tunneled_peers: HashSet<PeerId>,
    /// Smoothed round-trip time per peer, milliseconds. Fed by probe acks and
    /// read by the shim for QoS, where titles show it as ping.
    peer_rtt_ms: HashMap<PeerId, u32>,
    /// Handle used to open outbound tunnel streams; cloned into the tasks.
    tunnel_control: crate::stream::Control,
    /// Write half of each peer's outbound tunnel. Bounded, because the guest
    /// thinks it is sending UDP and a backed-up link should drop, not grow.
    tunnel_writers: HashMap<PeerId, mpsc::Sender<Vec<u8>>>,
    /// Tunnel tasks report here.
    tunnel_tx: mpsc::UnboundedSender<TunnelEvent>,
    /// Datagrams the tunnel dropped, per peer, since the last report. Only
    /// counted so loss can be seen in the log rather than guessed at from a
    /// title behaving oddly.
    tunnel_drops: HashMap<PeerId, u64>,
    /// The public key each peer's current game-plane keys were derived
    /// from. A different key in a later offer means the peer restarted and
    /// lost its state, and ours must follow or nothing authenticates until
    /// the stale connection times out.
    peer_game_pubs: HashMap<PeerId, [u8; 32]>,
    /// Our half of the game-plane key agreement, one per peer session (§6).
    /// Created when we first punch toward a peer and reused for every attempt,
    /// so simultaneous punches cannot derive mismatched keys.
    key_agreements: HashMap<PeerId, LocalKeyAgreement>,
    pending_punch_nonce: HashMap<[u8; 16], PeerId>,
    punch_replies: HashMap<PeerId, oneshot::Sender<Result<SocketAddr, String>>>,
    pending_punch_offers: HashMap<OutboundRequestId, PeerId>,

    pending_find: HashMap<kad::QueryId, (PeerId, oneshot::Sender<Result<Vec<Multiaddr>, String>>)>,
    pending_echo: HashMap<OutboundRequestId, (Instant, oneshot::Sender<Result<Duration, String>>)>,

    /// Per-packet complaints, said once per address per PACKET_WARN_WINDOW.
    unknown_vip_warns: WarnLimiter<u32>,
    no_keys_warns: WarnLimiter<PeerId>,
}

impl Engine {
    async fn run(
        mut self,
        mut cmd_rx: mpsc::UnboundedReceiver<Command>,
        mut game_rx: mpsc::UnboundedReceiver<(SocketAddr, Vec<u8>)>,
        mut stream_rx: mpsc::UnboundedReceiver<StreamEvent>,
        mut tunnel_rx: mpsc::UnboundedReceiver<TunnelEvent>,
    ) {
        // Bootstrap the shim's friend-list mirror from persisted state.
        let stored: Vec<PeerId> = self.friend_store.friends().copied().collect();
        for peer in stored {
            self.emit(Event::FriendAccepted { peer });
        }
        // Requests sent in an earlier run are still owed. The
        // receiver answers a repeat with the same Accept it would have sent, and
        // an Accept only counts here while the peer is in pending_out, which is
        // why that set is persisted now (friends.rs).
        let shown: Vec<(PeerId, String, String)> = self.friend_store.inbound().collect();
        for (peer, display_name, note) in shown {
            self.emit(Event::FriendRequest { peer, display_name, note });
        }
        let owed: Vec<PeerId> = self.friend_store.pending_out.iter().copied().collect();
        for peer in owed {
            let msg = FriendMsg::Request {
                schema: 1,
                display_name: self.display_name.clone(),
                note: String::new(),
            };
            self.pending_friend_msgs.insert(peer, (msg, Instant::now()));
        }
        self.announce_local_address();

        // Every timer starts one period out. A `tokio::time::interval` fires
        // at once, and seven of them firing together at startup -- before a
        // single connection or DHT answer exists -- was a burst of work that
        // could only fail. The one exception is the relay check: a NAT'd
        // node wants a mesh relay early, and ten seconds is enough for the
        // kad bootstrap to have something to answer with.
        let now = tokio::time::Instant::now();
        let mut heartbeat = tokio::time::interval_at(now + PRESENCE_HEARTBEAT, PRESENCE_HEARTBEAT);
        let mut shard_rescan =
            tokio::time::interval_at(now + SHARD_RESCAN_INTERVAL, SHARD_RESCAN_INTERVAL);
        let mut punch_watchdog =
            tokio::time::interval_at(now + PUNCH_WATCHDOG_INTERVAL, PUNCH_WATCHDOG_INTERVAL);
        let mut shard_status =
            tokio::time::interval_at(now + SHARD_STATUS_INTERVAL, SHARD_STATUS_INTERVAL);
        let mut relay_check =
            tokio::time::interval_at(now + Duration::from_secs(10), RELAY_RETRY_INTERVAL);
        let mut friend_retry =
            tokio::time::interval_at(now + FRIEND_RETRY_INTERVAL, FRIEND_RETRY_INTERVAL);
        let mut recovery_tick =
            tokio::time::interval_at(now + PEER_RECOVERY_TICK, PEER_RECOVERY_TICK);
        loop {
            tokio::select! {
                cmd = cmd_rx.recv() => {
                    let Some(cmd) = cmd else { break };
                    if !self.handle_command(cmd) {
                        break;
                    }
                }
                event = self.swarm.select_next_some() => self.handle_swarm_event(event),
                Some((addr, frame)) = game_rx.recv() => self.handle_game_frame(addr, frame),
                Some(event) = stream_rx.recv() => self.handle_stream_event(event),
                Some(event) = tunnel_rx.recv() => self.handle_tunnel_event(event),
                _ = heartbeat.tick() => {
                    self.push_presence(None);
                    self.shard_beat();
                    self.publish_own_key();
                }
                _ = friend_retry.tick() => self.retry_deferred_messages(),
                _ = recovery_tick.tick() => self.maintain_recoveries(),
                _ = punch_watchdog.tick() => self.watchdog(),
                _ = shard_status.tick() => self.log_shard_status(),
                _ = relay_check.tick() => {
                    self.retry_dead_relays();
                    // Actively look for mesh relays in the DHT (§9) when short.
                    self.discover_relays();
                }
                _ = shard_rescan.tick() => {
                    // Sweep, then settle on the *next* tick once descriptors
                    // have arrived; shard_settle also runs when a sweep
                    // completes, so a slow DHT does not stall placement.
                    self.shard_settle();
                    self.shard_discover();
                }
            }
        }
        tracing::info!("rexnet engine down");
    }

    /// Record (or clear) a peer's relay role, announcing only real changes.
    /// DCUtR upgrades relayed connections to direct as soon as it can, so
    /// this flips more than once in a normal session.
    /// Re-dial configured relays with no live reservation.
    ///
    /// Reservations lapse when a relay restarts or the network drops, and
    /// nothing else notices: the node keeps working over direct paths, which
    /// is exactly why a silently dead relay would otherwise stay dead until
    /// the next launch.
    fn retry_dead_relays(&mut self) {
        let now = Instant::now();
        let dead: Vec<(PeerId, Multiaddr)> = self
            .relay_addrs
            .iter()
            .filter(|(peer, _)| !self.relays.contains(peer))
            .filter(|(peer, _)| self.relay_retry_at.get(peer).is_none_or(|at| now >= *at))
            .cloned()
            .collect();
        for (peer, addr) in dead {
            self.relay_retry_at.insert(peer, now + RELAY_RETRY_INTERVAL);
            match self.swarm.listen_on(addr.clone().with(Protocol::P2pCircuit)) {
                Ok(_) => tracing::debug!(%addr, "retrying relay reservation"),
                Err(err) => tracing::debug!(%addr, %err, "relay retry failed"),
            }
        }
    }

    fn mark_relay(&mut self, peer: PeerId, is_relay: bool) {
        let changed =
            if is_relay { self.relays.insert(peer) } else { self.relays.remove(&peer) };
        if !changed {
            return;
        }
        tracing::info!(
            %peer, is_relay,
            live = self.relays.len(), configured = self.relay_addrs.len(),
            "relay role changed"
        );
        // A relay is app-relevant even when it is nothing else to us: the UI
        // has to be able to show it.
        if is_relay {
            self.announce_peer(peer);
        }
        self.emit(Event::RelayStatus { peer, is_relay });
    }

    fn emit(&self, event: Event) {
        self.emitter.emit(event);
    }

    // --- Ambient title shard (§17.3) ------------------------------------

    /// Live members: gossipsub topic peers plus ourselves. This is the
    /// authoritative count; `ShardDesc::members_hint` is a stale advisory for
    /// peers that have not subscribed yet (§17.3.4).
    fn shard_member_count(&self) -> u16 {
        let Some(current) = &self.current_shard else {
            return 0;
        };
        let topic = gossipsub::IdentTopic::new(crate::protocol::shard_topic(&current.shard_id));
        let hash = topic.hash();
        let peers = self
            .swarm
            .behaviour()
            .gossipsub
            .all_peers()
            .filter(|(_, topics)| topics.iter().any(|t| **t == hash))
            .count();
        u16::try_from(peers.saturating_add(1)).unwrap_or(u16::MAX)
    }

    /// Subscribe to a shard's topic and advertise ourselves as a provider so
    /// later arrivals can find it.
    /// Send a friend message now if the peer is connected, else hold
    /// it and go looking for the peer. See `pending_friend_msgs`.
    fn friend_send(&mut self, peer: PeerId, msg: FriendMsg) {
        if self.connected.contains(&peer) {
            let id = self.swarm.behaviour_mut().friend.send_request(&peer, msg.clone());
            self.inflight_friend.insert(id, (peer, msg));
            return;
        }
        tracing::info!(%peer, "friend message deferred until connected");
        self.pending_friend_msgs.insert(peer, (msg, Instant::now()));
        self.friend_seek(peer);
    }

    /// Every way we have of reaching a peer we hold no address for -- a
    /// dial over whatever kad, mdns or identify already know, a DHT peer
    /// lookup, and a shard sweep, since another player of the same title
    /// publishes a shard provider record that carries their addresses.
    ///
    /// Never marks the peer wanted: this runs on every background retry, and
    /// a dial failure here is the normal state of a friend who is offline,
    /// not an error the player should see each time. Delivery is reported
    /// when it happens (FriendDelivered).
    fn friend_seek(&mut self, peer: PeerId) {
        self.seek_peer(peer);
        if self.shard_enabled && self.pending_shard_search.is_none() {
            self.shard_discover();
        }
    }

    /// Dial on what is known and look the peer up both ways: its own
    /// reachability record and the closest-peers walk.
    fn seek_peer(&mut self, peer: PeerId) {
        if let Err(err) = self.swarm.dial(peer) {
            tracing::debug!(%peer, %err, "dial failed; trying DHT");
        }
        self.lookup_peer(peer);
        self.swarm.behaviour_mut().kad.get_closest_peers(peer);
    }

    /// Find one peer's addresses through its own reachability record. One
    /// lookup per peer at a time; the result dials once and finishes.
    fn lookup_peer(&mut self, peer: PeerId) {
        if self.peer_lookups.values().any(|target| *target == peer) {
            return;
        }
        let key = kad::RecordKey::new(&crate::protocol::peer_key(&peer));
        let id = self.swarm.behaviour_mut().kad.get_providers(key);
        self.peer_lookups.insert(id, peer);
    }

    /// Our own reachability record, so friends holding only our peer id can
    /// find us: at every heartbeat until one DHT node has stored it, then at
    /// OWN_KEY_REPUBLISH.
    fn publish_own_key(&mut self) {
        if self.own_key_query.is_some() || !own_key_due(self.own_key_at, Instant::now()) {
            return;
        }
        let local = *self.swarm.local_peer_id();
        let key = kad::RecordKey::new(&crate::protocol::peer_key(&local));
        match self.swarm.behaviour_mut().kad.start_providing(key) {
            Ok(id) => self.own_key_query = Some(id),
            Err(err) => tracing::warn!(%err, "own reachability record publish failed"),
        }
    }

    /// Every FRIEND_RETRY_INTERVAL: expire what has waited too long, send
    /// what can be sent now, and go looking for the rest. Quiet retries: no
    /// peer is marked wanted, so one that is simply offline raises no error.
    fn retry_deferred_messages(&mut self) {
        let now = Instant::now();
        // A Decline that could not be delivered in INVITE_DEFER_MAX is not
        // worth delivering: the requester has long since stopped waiting.
        self.pending_friend_msgs.retain(|peer, (msg, queued)| {
            let expired = matches!(msg, FriendMsg::Decline { .. })
                && now.saturating_duration_since(*queued) > INVITE_DEFER_MAX;
            if expired {
                tracing::debug!(%peer, "undeliverable friend decline expired");
            }
            !expired
        });
        // An invite that waited out its deferral is a failure the host hears.
        let stale: Vec<PeerId> = self
            .pending_invites
            .iter()
            .filter(|(_, pending)| now.saturating_duration_since(pending.queued) > INVITE_DEFER_MAX)
            .map(|(peer, _)| *peer)
            .collect();
        for peer in stale {
            self.pending_invites.remove(&peer);
            self.invite_failed(peer, "peer did not become reachable in time");
        }

        // Connected peers with a message queued and nothing in flight: the
        // message was put back by a failure while the connection stayed up,
        // and nothing else would send it until the peer reconnected.
        let inflight_friend: HashSet<PeerId> =
            self.inflight_friend.values().map(|(peer, _)| *peer).collect();
        let sendable: Vec<PeerId> = self
            .pending_friend_msgs
            .keys()
            .filter(|peer| self.connected.contains(peer) && !inflight_friend.contains(peer))
            .copied()
            .collect();
        for peer in sendable {
            if let Some((msg, _)) = self.pending_friend_msgs.remove(&peer) {
                tracing::debug!(%peer, "re-sending queued friend message on the live connection");
                let id = self.swarm.behaviour_mut().friend.send_request(&peer, msg.clone());
                self.inflight_friend.insert(id, (peer, msg));
            }
        }
        let inflight_invite: HashSet<PeerId> =
            self.inflight_invites.values().map(|(peer, _)| *peer).collect();
        let sendable: Vec<PeerId> = self
            .pending_invites
            .keys()
            .filter(|peer| self.connected.contains(peer) && !inflight_invite.contains(peer))
            .copied()
            .collect();
        for peer in sendable {
            if let Some(pending) = self.pending_invites.remove(&peer) {
                self.send_invite_now(peer, pending);
            }
        }

        // Friends we are not connected to as well as peers owed a message: a
        // friend who comes back online is found the same way a stranger is.
        let mut waiting: HashSet<PeerId> = self.pending_friend_msgs.keys().copied().collect();
        waiting.extend(self.pending_invites.keys().copied());
        waiting.extend(self.friend_store.friends().copied());
        waiting.retain(|peer| !self.connected.contains(peer));
        for peer in waiting {
            if !retry_due(&mut self.lookup_retry_at, peer, now, PEER_LOOKUP_FIRST, PEER_LOOKUP_MAX) {
                continue;
            }
            tracing::debug!(%peer, "looking up peer");
            self.seek_peer(peer);
        }
        let keep: HashSet<PeerId> = self
            .pending_friend_msgs
            .keys()
            .chain(self.pending_invites.keys())
            .chain(self.friend_store.friends())
            .copied()
            .collect();
        self.lookup_retry_at.retain(|peer, _| keep.contains(peer));
    }

    /// Hand an invite (offer or reply) to the connection now, or hold it
    /// for the next one. One path for both directions: the inviter's
    /// connection can be gone by the time the human has answered, and a
    /// reply sent blind then was lost exactly as the first offer used to be.
    fn invite_send(&mut self, peer: PeerId, msg: InviteMsg) {
        let pending = PendingInvite { msg, queued: Instant::now(), attempts: 0 };
        if self.connected.contains(&peer) {
            self.send_invite_now(peer, pending);
            return;
        }
        tracing::info!(%peer, "invite deferred until connected");
        self.pending_invites.insert(peer, pending);
        self.seek_peer(peer);
    }

    /// Put an invite on the wire and track it. An accepting reply also asks
    /// the host for its descriptor, which is what the shim builds the join
    /// material from; the host serves it to us as an invitee.
    fn send_invite_now(&mut self, peer: PeerId, mut pending: PendingInvite) {
        self.announce_peer(peer);
        pending.attempts = pending.attempts.saturating_add(1);
        let accepting = matches!(pending.msg, InviteMsg::Reply { accept: true, .. });
        let id = self.swarm.behaviour_mut().invite.send_request(&peer, pending.msg.clone());
        self.inflight_invites.insert(id, (peer, pending));
        if accepting {
            self.swarm
                .behaviour_mut()
                .session
                .send_request(&peer, SessionQuery { schema: 1, token: 0 });
        }
    }

    /// The invite is not going to arrive: say so, once, with the peer.
    fn invite_failed(&mut self, peer: PeerId, reason: &str) {
        tracing::warn!(%peer, reason, "invite not delivered");
        self.emit(Event::InviteFailed { peer, message: format!("invite to {peer} not delivered: {reason}") });
    }

    fn shard_enter(&mut self, desc: ShardDesc) {
        let topic = gossipsub::IdentTopic::new(crate::protocol::shard_topic(&desc.shard_id));
        if let Err(err) = self.swarm.behaviour_mut().gossipsub.subscribe(&topic) {
            tracing::warn!(%err, "shard topic subscribe failed");
            return;
        }
        let key = kad::RecordKey::new(&crate::protocol::shard_key(self.title_id));
        if let Err(err) = self.swarm.behaviour_mut().kad.start_providing(key) {
            // Non-fatal: we are still in the shard, just harder to discover.
            tracing::warn!(%err, "shard provider record publish failed");
        }
        tracing::info!(
            shard = %hex16(&desc.shard_id), created_at = desc.created_at,
            "joined shard"
        );
        if self.shard_preferred == Some(desc.shard_id) {
            self.shard_preferred = None;
        }
        self.emit(Event::ShardJoined { shard_id: desc.shard_id });
        self.current_shard = Some(desc);
        self.readdress_all();
        // Say so at once rather than at the next heartbeat, so the members we are
        // already connected to place us on the /24 before anyone dials.
        self.shard_beat();
        let peers: Vec<PeerId> = self.rexnet_peers.iter().copied().collect();
        for peer in peers {
            self.shard_query_peer(peer);
        }
    }

    /// A session is up and addresses must not move: we host one, a peer is
    /// in one with us by invite (for INVITE_DEFER_MAX), or a peer exchanged
    /// title datagrams with us inside TITLE_TRAFFIC_WINDOW. A warmed or
    /// tunnelled game plane that carries nothing is not a game: counting it
    /// pinned every shard member's address for as long as the shard lived.
    fn game_in_progress(&self) -> bool {
        if self.local_session.is_some() {
            return true;
        }
        let now = Instant::now();
        if self
            .session_peers
            .values()
            .any(|since| now.saturating_duration_since(*since) < INVITE_DEFER_MAX)
        {
            return true;
        }
        !self.plane.peers_active_within(TITLE_TRAFFIC_WINDOW).is_empty()
    }

    /// A peer joined our shard. Mid-game nobody's address may move, so the
    /// newcomer takes a free host beside the pinned table and the full
    /// re-derivation waits for the game to end; otherwise the whole table is
    /// rebuilt from one snapshot, as placement requires.
    fn shard_member_arrived(&mut self, peer: PeerId) {
        self.shard_member_heard.insert(peer, Instant::now());
        if !self.shard_member_peers.insert(peer) {
            return;
        }
        if !self.game_in_progress() {
            self.readdress_all();
            return;
        }
        self.readdress_deferred = true;
        match subnet::assign_without_moving(&self.shard_hosts, &peer) {
            Some(host) => {
                self.shard_hosts.insert(peer, host);
            }
            None => tracing::warn!(%peer, "shard subnet full; newcomer left on the reserved range"),
        }
        // A newcomer that already holds an address keeps it: it may be the
        // very partner the game is with. Its vip is read from the pinned
        // table when it is first announced, and settles with everyone else's
        // once the game ends.
        tracing::info!(
            %peer, "shard member placed without moving anyone; full readdress deferred until the game ends"
        );
    }

    /// A member left the shard (unsubscribed, departed, or fell silent). Its
    /// host is freed; nobody moves while a game is up.
    fn shard_member_left(&mut self, peer: PeerId, why: &str) {
        if !self.shard_member_peers.remove(&peer) {
            return;
        }
        self.shard_member_heard.remove(&peer);
        self.shard_reachable_attempted.remove(&peer);
        self.shard_hosts.remove(&peer);
        tracing::info!(%peer, why, remaining = self.shard_member_peers.len(), "shard member removed");
        if !self.connected.contains(&peer) {
            // No control connection: its punch nonces and tunnel are stale,
            // and a peer the shim never heard of needs no address.
            self.pending_punch_nonce.retain(|_, p| *p != peer);
            self.punch_deadline.remove(&peer);
            self.pending_datagrams.remove(&peer);
            self.tunneled_peers.remove(&peer);
            self.close_tunnel(&peer);
            if !self.announced.contains(&peer) {
                if let Some(vip) = self.vip_by_peer.remove(&peer) {
                    self.peer_by_vip.remove(&vip);
                    self.plane.clear_vip(&peer);
                }
            }
        }
        if self.game_in_progress() {
            self.readdress_deferred = true;
        } else {
            self.readdress_all();
        }
    }

    /// Members not heard for SHARD_MEMBER_TIMEOUT are gone. One sweep for
    /// all of them, from the watchdog.
    fn prune_shard_members(&mut self) {
        let now = Instant::now();
        let silent: Vec<PeerId> = self
            .shard_member_peers
            .iter()
            .filter(|peer| {
                self.shard_member_heard
                    .get(peer)
                    .is_none_or(|at| now.saturating_duration_since(*at) >= SHARD_MEMBER_TIMEOUT)
            })
            .copied()
            .collect();
        for peer in silent {
            self.shard_member_left(peer, "silent");
        }
    }

    /// Leave the current shard, unsubscribing and withdrawing our record.
    fn shard_leave(&mut self) {
        let Some(current) = self.current_shard.take() else {
            return;
        };
        let topic = gossipsub::IdentTopic::new(crate::protocol::shard_topic(&current.shard_id));
        let _ = self.swarm.behaviour_mut().gossipsub.unsubscribe(&topic);
        let key = kad::RecordKey::new(&crate::protocol::shard_key(self.title_id));
        self.swarm.behaviour_mut().kad.stop_providing(&key);
        self.shard_member_peers.clear();
        self.shard_member_heard.clear();
        self.shard_reachable_attempted.clear();
        self.readdress_deferred = false;
        tracing::info!(shard = %hex16(&current.shard_id), "left shard");
        self.emit(Event::ShardLeft { shard_id: current.shard_id });
        self.readdress_all();
    }

    fn shard_create(&mut self) {
        let mut shard_id = [0u8; 16];
        rand::Rng::fill(&mut rand::thread_rng(), &mut shard_id);
        let created_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let desc = ShardDesc {
            schema: 1,
            shard_id,
            title_id: self.title_id,
            created_at,
            cap: self.shard_cap,
            members_hint: 1,
            anchor: self.swarm.local_peer_id().to_bytes(),
        };
        tracing::info!(shard = %hex16(&shard_id), "created shard");
        self.shard_enter(desc);
    }

    /// Ask one peer directly what shard it is in.
    fn shard_query_peer(&mut self, peer: PeerId) {
        tracing::debug!(%peer, "querying peer for its shard");
        let title_id = self.title_id;
        let req = self
            .swarm
            .behaviour_mut()
            .shard
            .send_request(&peer, ShardQuery { schema: 1, title_id });
        self.pending_shard_queries.insert(req, peer);
    }

    /// Kick discovery: sweep the DHT *and* ask every RexNet peer we are
    /// already connected to.
    ///
    /// The direct half is not an optimisation. With a bootstrap configured we
    /// run kad in client mode (§4), so a node does not answer other peers'
    /// provider queries from its own store -- two instances on one LAN, already
    /// mDNS-connected, would otherwise have to round-trip their shard records
    /// through the public DHT and hope they propagate. Asking a peer we are
    /// holding a connection to is immediate and always works.
    fn shard_discover(&mut self) {
        if !self.shard_enabled {
            return;
        }
        let key = kad::RecordKey::new(&crate::protocol::shard_key(self.title_id));
        let id = self.swarm.behaviour_mut().kad.get_providers(key);
        self.pending_shard_search = Some((id, Vec::new()));

        let peers: Vec<PeerId> = self.rexnet_peers.iter().copied().collect();
        for peer in peers {
            self.shard_query_peer(peer);
        }
    }

    /// Apply §17.3.2 placement / §17.3.3 convergence to what we have learned.
    fn shard_settle(&mut self) {
        if !self.shard_enabled {
            return;
        }
        // Count friends per shard so affinity can outrank age. A shard we
        // know nothing about scores zero rather than being excluded -- an
        // unknown shard is still better than fragmenting into a new one.
        let candidates: Vec<Candidate> = self
            .shard_candidates
            .values()
            .map(|desc| Candidate {
                friends_present: self.shard_friend_count(&desc.shard_id),
                desc: desc.clone(),
            })
            .collect();

        // A GAME IN PROGRESS PINS EVERY ADDRESS. Entering a shard re-derives the
        // virtual IPs of every peer (readdress_all), and the titles on both ends
        // hold those addresses in live sockets. The migration path below already
        // refuses to move a player mid-co-op; the FIRST placement did not, and with
        // the shard newly enabled it fired while two players sat on the co-op
        // screen: the host moved to 10.77.34.x, the joiner's packets went to an
        // address nobody held, and the session dropped. Placement waits until no
        // game path is up; the rescan picks it up then.
        if self.game_in_progress() {
            tracing::debug!("shard placement deferred: a game path is up");
            return;
        }
        // A named shard that is still being looked for holds placement off:
        // creating or joining another shard now would just be left again.
        if let Some(preferred) = self.shard_preferred {
            if !self.shard_candidates.contains_key(&preferred) {
                tracing::debug!(shard = %hex16(&preferred), "shard settle waiting on the named shard");
                return;
            }
        }
        match self.current_shard.clone() {
            None => {
                match shard::choose(self.title_id, &candidates) {
                    Placement::Join(id) => {
                        if let Some(desc) = self.shard_candidates.get(&id).cloned() {
                            self.shard_enter(desc);
                        }
                    }
                    Placement::Create => {
                        // Only found a shard once we have actually looked and
                        // every descriptor query has come back. Creating on
                        // incomplete knowledge is how a population fragments.
                        if !self.pending_shard_queries.is_empty() {
                            return;
                        }
                        match self.shard_swept_at {
                            None => {} // never looked
                            Some(at) if at.elapsed() >= SHARD_CREATE_GRACE => {
                                self.shard_create();
                            }
                            Some(_) => {
                                // A peer still connecting has not had the
                                // chance to answer yet (SHARD_CREATE_GRACE).
                                tracing::debug!("shard creation deferred; still looking");
                            }
                        }
                    }
                }
            }
            Some(current) => {
                // An Active session pins us in place: never relocate a player
                // mid-co-op to tidy up topology (§17.3.3).
                let session_active = self.local_session.is_some();
                if !self.shard_cooldown_elapsed() {
                    tracing::debug!("shard settle skipped (migration cooldown)");
                    return;
                }
                tracing::debug!(
                    current = %hex16(&current.shard_id),
                    candidates = candidates.len(),
                    session_active,
                    "shard settle"
                );
                if let Migration::MoveTo(id) =
                    shard::should_migrate(self.title_id, &current, &candidates, session_active)
                {
                    // Roll per decision, not per node: a node that sits out
                    // this round re-decides next rescan, so the herd spreads
                    // across several intervals instead of one instant. Skipped
                    // for small shards, where there is no herd and the delay is
                    // pure cost.
                    let crowded = self.shard_member_peers.len() >= SHARD_JITTER_MIN_MEMBERS;
                    if crowded
                        && !rand::Rng::gen_bool(&mut rand::thread_rng(), SHARD_MIGRATION_CHANCE)
                    {
                        tracing::debug!("shard migration deferred (jitter)");
                        return;
                    }
                    if let Some(desc) = self.shard_candidates.get(&id).cloned() {
                        tracing::info!(
                            from = %hex16(&current.shard_id), to = %hex16(&id),
                            "migrating shard"
                        );
                        self.shard_leave();
                        self.shard_enter(desc);
                        self.last_shard_move = Some(Instant::now());
                    }
                }
            }
        }
    }

    /// Make a shard member reachable, so promoting them to Active (an invite
    /// or a join) does not have to start from nothing.
    ///
    /// Being in a shard is meant to mean "I can reach these people". Gossipsub
    /// alone does not deliver that: it meshes with a handful of peers and the
    /// rest are just peer ids arriving in messages. Small shards get a real
    /// connection each; large ones get an address resolved into the routing
    /// table, which is enough for a dial to succeed later.
    fn shard_ensure_reachable(&mut self, peer: PeerId) {
        if peer == *self.swarm.local_peer_id() || self.connected.contains(&peer) {
            return;
        }
        // Once per peer per session: heartbeats arrive every 30 s and we do
        // not want to re-dial an unreachable member forever.
        if !self.shard_reachable_attempted.insert(peer) {
            return;
        }
        if self.shard_member_peers.len() <= SHARD_WARM_CONNECT_LIMIT {
            tracing::debug!(%peer, "shard member: dialling to keep warm");
            if self.swarm.dial(peer).is_ok() {
                return;
            }
            // No address known -- fall through to the DHT.
        }
        tracing::debug!(%peer, "shard member: resolving address via DHT");
        self.swarm.behaviour_mut().kad.get_closest_peers(peer);
    }

    /// Punch the game plane to a shard member ahead of time.
    ///
    /// A control connection alone does not make a peer playable: the game
    /// plane is a separate punched UDP path, and establishing it on demand is
    /// what makes the first co-op invite race (both sides punch independently,
    /// and whoever finishes first sends into an endpoint the other has not
    /// mapped yet). Being in a shard is supposed to mean "ready to play", so
    /// members of a small shard get their game plane punched up front and an
    /// invite finds it already there.
    ///
    /// Only for shards within the warm limit: punching 254 members would be
    /// both pointless and hostile to NAT tables.
    fn shard_warm_game_plane(&mut self, peer: PeerId) {
        if self.shard_member_peers.len() > SHARD_WARM_CONNECT_LIMIT {
            return;
        }
        if self.game_plane_ready(&peer)
            || self.punch_deadline.contains_key(&peer)
            || self.tunneled_peers.contains(&peer)
        {
            return; // already punched, punching, or knowingly degraded
        }
        if !self.connected.contains(&peer) {
            return; // the punch offer needs a control connection to ride
        }
        tracing::debug!(%peer, "warming game plane for shard member");
        self.begin_punch(peer);
    }

    /// Broadcast our ambient state to the shard (§17.3.4). Cheap and
    /// low-rate: this is the only traffic a Connected-tier peer generates for
    /// players it never interacts with.
    fn shard_beat(&mut self) {
        let Some(current) = self.current_shard.clone() else {
            return;
        };
        let beat = ShardBeat {
            schema: 1,
            state: self.my_presence.state,
            rich: self.my_presence.rich.clone(),
            // A private session id reaching the whole shard is a leak: the
            // host hands it out by invite or token, not by broadcast.
            session_id: self.local_session.as_ref().filter(|s| s.is_public).map(|s| s.id),
        };
        let Ok(bytes) = postcard::to_allocvec(&beat) else {
            return;
        };
        let topic = gossipsub::IdentTopic::new(crate::protocol::shard_topic(&current.shard_id));
        if let Err(err) = self.swarm.behaviour_mut().gossipsub.publish(topic, bytes) {
            // InsufficientPeers is the normal case for a shard of one; it is
            // not worth a warning every heartbeat.
            match err {
                gossipsub::PublishError::NoPeersSubscribedToTopic => {
                    tracing::trace!("shard beat: no topic peers yet");
                }
                other => tracing::debug!(err = %other, "shard beat publish failed"),
            }
        }
    }

    /// Migration cooldown. Direction is unconditional by design (§17.3.3), so
    /// without this every member of a younger shard would move at once.
    fn shard_cooldown_elapsed(&self) -> bool {
        match self.last_shard_move {
            None => true,
            Some(at) => at.elapsed() >= SHARD_MIGRATION_COOLDOWN,
        }
    }

    /// Friends currently subscribed to a shard's topic.
    fn shard_friend_count(&self, shard_id: &[u8; 16]) -> u16 {
        let topic = gossipsub::IdentTopic::new(crate::protocol::shard_topic(shard_id));
        let hash = topic.hash();
        let count = self
            .swarm
            .behaviour()
            .gossipsub
            .all_peers()
            .filter(|(peer, topics)| {
                self.friend_store.is_friend(peer) && topics.iter().any(|t| **t == hash)
            })
            .count();
        u16::try_from(count).unwrap_or(u16::MAX)
    }



    /// Push our presence record to one connected mutual friend, or to all
    /// of them (§8.1: pushed on change, heartbeated every 30 s).
    fn push_presence(&mut self, only: Option<PeerId>) {
        self.presence_seq += 1;
        self.my_presence.seq = self.presence_seq;
        let msg = self.my_presence.clone();
        let targets: Vec<PeerId> = match only {
            Some(peer) => vec![peer],
            None => self
                .friend_store
                .friends()
                .filter(|p| self.connected.contains(p))
                .copied()
                .collect(),
        };
        for peer in targets {
            if self.friend_store.is_friend(&peer) {
                self.swarm.behaviour_mut().presence.send_request(&peer, msg.clone());
            }
        }
    }

    /// Surface a connected peer to the app: allocate its vip and emit
    /// PeerConnected, once. Called from every point where a peer proves
    /// app-relevant — explicit dial, friendship, session hit, inbound
    /// rexnet protocol traffic. No-op for peers we aren't connected to
    /// (or have already announced), so callers can invoke it freely.
    fn announce_peer(&mut self, peer: PeerId) {
        if !self.connected.contains(&peer) || !self.announced.insert(peer) {
            return;
        }
        let virtual_ip = self.vip_for(peer);
        self.emit(Event::PeerConnected { peer, virtual_ip });
        // Membership feeds the off-shard table, so our own address can move.
        // Only a collision actually shifts it, which needs a near-full subnet.
        self.announce_local_address();
    }

    /// Allocate (or return) the virtual IP for a peer.
    ///
    /// **Addresses are stable within a shard epoch, never inside one.** The
    /// guest holds an address in live sockets -- XRNM addresses its link by
    /// virtual IP -- so nothing here revises one on a whim.
    ///
    /// A shard change is the exception, because it moves the whole /24: see
    /// [`Self::readdress_all`].
    fn vip_for(&mut self, peer: PeerId) -> u32 {
        if let Some(vip) = self.vip_by_peer.get(&peer) {
            return *vip;
        }
        let vip = self.derive_vip(peer);
        self.vip_by_peer.insert(peer, vip);
        self.peer_by_vip.insert(vip, peer);
        self.plane.set_vip(peer, vip);
        vip
    }

    /// Re-derive one peer's address, reporting whether it moved.
    fn readdress_peer(&mut self, peer: PeerId) -> bool {
        let previous = self.vip_by_peer.get(&peer).copied();
        let vip = self.derive_vip(peer);
        if previous == Some(vip) {
            return false;
        }
        if let Some(old) = previous {
            self.peer_by_vip.remove(&old);
        }
        self.vip_by_peer.insert(peer, vip);
        self.peer_by_vip.insert(vip, peer);
        self.plane.set_vip(peer, vip);
        tracing::info!(
            %peer,
            from = %previous.map(format_vip).unwrap_or_else(|| "-".to_string()),
            to = %format_vip(vip),
            "peer readdressed"
        );
        // The shim mirrors the table off this event.
        if self.connected.contains(&peer) {
            self.emit(Event::PeerConnected { peer, virtual_ip: vip });
        }
        true
    }

    /// Move every address onto the current shard's /24, together.
    ///
    /// Our own address is derived fresh on every read, so it follows a shard
    /// change on its own. Peers must follow in the same step or the two sides
    /// end up on different subnets -- which silently disables the one thing a
    /// shard is for, since subnet broadcast (System Link discovery) can only
    /// reach the /24 we are actually on.
    ///
    /// Placement recomputes the whole table from one membership snapshot
    /// rather than peer by peer: `subnet::assign` resolves collisions by
    /// probing, so a partial update can hand a newcomer an address another
    /// member is still holding.
    ///
    /// Only ever called with no game in progress -- callers that learn of a
    /// membership change mid-game extend the pinned table instead and set
    /// `readdress_deferred`, which the watchdog turns into this call once the
    /// game is over -- so this never re-addresses anyone mid-co-op.
    fn readdress_all(&mut self) {
        self.readdress_deferred = false;
        self.rebuild_shard_hosts();
        // A peer in neither membership table cannot be derived -- `assign`
        // would not place it, and every such peer would collapse onto the same
        // fallback address. Drop them; a reconnect derives a fresh one.
        let stale: Vec<PeerId> = self
            .vip_by_peer
            .keys()
            .copied()
            .filter(|p| !self.announced.contains(p) && !self.shard_member_peers.contains(p))
            .collect();
        for peer in stale {
            if let Some(vip) = self.vip_by_peer.remove(&peer) {
                self.peer_by_vip.remove(&vip);
                self.plane.clear_vip(&peer);
            }
        }
        let peers: Vec<PeerId> = self.vip_by_peer.keys().copied().collect();
        for peer in peers {
            self.readdress_peer(peer);
        }
        self.announce_local_address();
    }

    /// The shard host table from the whole membership, ourselves included:
    /// our own peer id has to be in it too, or a remote could be handed the
    /// host octet we occupy. Empty outside a shard.
    fn rebuild_shard_hosts(&mut self) {
        self.shard_hosts.clear();
        if self.current_shard.is_none() {
            return;
        }
        let mut members: Vec<PeerId> = self.shard_member_peers.iter().copied().collect();
        members.push(*self.swarm.local_peer_id());
        self.shard_hosts = subnet::assign(&members);
    }

    /// Shard members get their address on the shard's /24 (§17.3.5) from the
    /// pinned host table; everyone else gets one from the reserved subnet.
    fn derive_vip(&mut self, peer: PeerId) -> u32 {
        if let Some(current) = self.current_shard.clone() {
            if self.shard_member_peers.contains(&peer) {
                let host = match self.shard_hosts.get(&peer) {
                    Some(host) => Some(*host),
                    // A member the table has not caught up with yet is placed
                    // beside it, never by rebuilding it: a rebuild is what
                    // moves addresses, and it has its own moment.
                    None => {
                        let placed = subnet::assign_without_moving(&self.shard_hosts, &peer);
                        if let Some(host) = placed {
                            self.shard_hosts.insert(peer, host);
                        }
                        placed
                    }
                };
                if let Some(host) = host {
                    let vip = subnet::address(VIP_NETWORK_BASE, &current.shard_id, host);
                    tracing::debug!(
                        %peer, addr = %format_vip(vip), "shard address assigned"
                    );
                    return vip;
                }
                // Subnet full -- fall through rather than leave them
                // unaddressable and therefore unreachable by the game.
                tracing::warn!(%peer, "shard subnet full; using reserved range");
            }
        }
        self.reserved_vip_for(&peer)
    }

    /// Sequential address in 10.77.255.0/24 for a peer outside our shard.
    /// Off-shard peers, plus ourselves. Both nodes derive the same table from
    /// the same membership, which a local counter cannot do.
    fn reserved_members(&self) -> Vec<PeerId> {
        let mut members: Vec<PeerId> = self
            .announced
            .iter()
            .copied()
            .filter(|p| !self.shard_member_peers.contains(p))
            .collect();
        members.push(*self.swarm.local_peer_id());
        members
    }

    /// Address in the reserved /24, derived rather than allocated: a counter
    /// gives each node its own private numbering, so two peers never agree on
    /// who is who — fine on one machine, useless across two.
    fn reserved_vip_for(&self, peer: &PeerId) -> u32 {
        let base = VIP_NETWORK_BASE | (u32::from(subnet::RESERVED_SUBNET) << 8);
        match subnet::assign(&self.reserved_members()).get(peer) {
            Some(host) => base | u32::from(*host),
            None => {
                tracing::error!(%peer, "reserved virtual-IP subnet exhausted");
                base | u32::from(subnet::HOST_MIN)
            }
        }
    }

    /// Send anything that was waiting on this peer's punch, in order, over
    /// whichever path is now available.
    fn flush_pending_datagrams(&mut self, peer: PeerId, addr: Option<SocketAddr>) {
        let Some(queue) = self.pending_datagrams.remove(&peer) else {
            return;
        };
        let mut sent = 0usize;
        let mut stale = 0usize;
        for pending in queue {
            // A handshake that arrives after the title has given up confuses
            // it more than silence would.
            if pending.queued_at.elapsed() > PUNCH_QUEUE_MAX_AGE {
                stale += 1;
                continue;
            }
            match addr {
                Some(endpoint) => {
                    if let Some(frame) =
                        self.seal_datagram(peer, pending.src_port, pending.dst_port, &pending.data)
                    {
                        let _ = self.game_socket.try_send_to(&frame, endpoint);
                    }
                }
                None => self.send_tunneled(
                    peer,
                    pending.src_port,
                    pending.dst_port,
                    pending.data,
                ),
            }
            sent += 1;
        }
        if sent > 0 || stale > 0 {
            let path = if addr.is_some() { "punched" } else { "tunnel" };
            tracing::info!(%peer, sent, stale, path, "flushed datagrams held for punch");
        }
    }

    /// Deliver frames that arrived from `addr` before it was mapped to a peer.
    fn deliver_orphan_frames(&mut self, peer: PeerId, addr: SocketAddr) {
        let Some(queue) = self.orphan_frames.remove(&addr) else {
            return;
        };
        let virtual_ip = self.vip_for(peer);
        let mut delivered = 0usize;
        for (received_at, frame) in queue {
            if received_at.elapsed() > PUNCH_QUEUE_MAX_AGE
                || frame.len() < 1 + crypto::CRYPTO_OVERHEAD
            {
                continue;
            }
            // Buffered before the address mapped to a peer, so they could not
            // be decrypted then: keys are per peer and records carry nothing
            // identifying.
            let Some((src_port, dst_port, data)) = self.open_datagram(peer, &frame) else {
                continue;
            };
            self.emit(Event::Datagram { virtual_ip, src_port, dst_port, data });
            delivered += 1;
        }
        if delivered > 0 {
            tracing::info!(%peer, %addr, delivered, "delivered frames held before punch");
        }
    }

    /// Drop orphan frames whose sender never became a known peer.
    fn prune_orphan_frames(&mut self) {
        self.orphan_frames.retain(|_, queue| {
            queue.retain(|(at, _)| at.elapsed() <= PUNCH_QUEUE_MAX_AGE);
            !queue.is_empty()
        });
    }

    /// A punched endpoint is only usable once it is also keyed; before that
    /// nothing can be sealed, so treating it as ready strands the peer.
    fn game_plane_ready(&self, peer: &PeerId) -> bool {
        self.plane.ready(peer)
    }

    /// The punched endpoint, only once it is also keyed.
    fn ready_endpoint(&self, peer: &PeerId) -> Option<SocketAddr> {
        self.plane.endpoint(peer).filter(|_| self.plane.has_keys(peer))
    }

    /// Reused across punch attempts so both sides agree (§6.1).
    fn local_game_pubkey(&mut self, peer: PeerId) -> [u8; 32] {
        self.key_agreements.entry(peer).or_default().public_key()
    }

    /// An established session is left alone when the same key arrives again
    /// (simultaneous punches deliver it twice): re-deriving would rewind the
    /// nonce counter and replay window. A *different* key can only come from
    /// a peer that restarted, so the session is rebuilt around it -- endpoint
    /// included, since the old one belongs to the old process.
    fn establish_game_keys(&mut self, peer: PeerId, peer_pub: [u8; 32]) {
        if self.plane.has_keys(&peer) {
            if self.peer_game_pubs.get(&peer) == Some(&peer_pub) {
                return;
            }
            tracing::info!(%peer, "peer presented a new game-plane key; rebuilding its session");
            self.plane.remove_keys(&peer);
            self.plane.unmap_endpoint(&peer);
        }
        let keys = self.key_agreements.entry(peer).or_default().derive(&peer_pub);
        self.plane.install_keys(peer, keys);
        self.peer_game_pubs.insert(peer, peer_pub);
        tracing::info!(%peer, "game-plane keys established");
        if let Some(addr) = self.plane.endpoint(&peer) {
            self.complete_game_plane(peer, addr);
        }
    }

    /// Engine-side sealing for held and broadcast traffic; live traffic goes
    /// through `GamePlane::try_send` without touching this loop. `None` means
    /// no keys: dropped rather than sent in clear, and said once per peer per
    /// window rather than per packet.
    fn seal_datagram(
        &mut self,
        peer: PeerId,
        src_port: u16,
        dst_port: u16,
        data: &[u8],
    ) -> Option<Vec<u8>> {
        let frame = self.plane.seal(peer, src_port, dst_port, data);
        if frame.is_none() && self.no_keys_warns.allow(peer) {
            tracing::warn!(%peer, "no game-plane keys; datagram dropped rather than sent in clear");
        }
        frame
    }

    /// Engine-side decryption for frames held before their endpoint mapped.
    fn open_datagram(&mut self, peer: PeerId, frame: &[u8]) -> Option<(u16, u16, Vec<u8>)> {
        self.plane.open(peer, frame)
    }

    /// Record a round trip measured from a probe ack.
    ///
    /// Smoothed rather than replaced: a single sample on a busy link is noise,
    /// and a title showing ping wants a stable number.
    fn note_rtt(&mut self, addr: SocketAddr, sent_micros: u64) {
        let Some(peer) = self.plane.peer_at(&addr) else {
            return;
        };
        let now = now_micros();
        if now < sent_micros {
            return; // stamp from a previous engine, or a forged ack
        }
        let sample = ((now - sent_micros) / 1000).min(u32::MAX as u64) as u32;
        let smoothed = match self.peer_rtt_ms.get(&peer) {
            // 1/4 weight on the newest sample, as TCP's SRTT does.
            Some(prev) => (prev * 3 + sample) / 4,
            None => sample,
        };
        let changed = self
            .peer_rtt_ms
            .insert(peer, smoothed)
            .map_or(true, |prev| prev.abs_diff(smoothed) >= RTT_REPORT_DELTA_MS);
        if changed {
            let virtual_ip = self.vip_for(peer);
            self.emit(Event::PeerRtt { virtual_ip, rtt_ms: smoothed });
        }
    }

    /// Carry one datagram over the control connection (§14 degraded path).
    fn send_tunneled(&mut self, peer: PeerId, src_port: u16, dst_port: u16, data: Vec<u8>) {
        let Some(frame) = tunnel::encode(src_port, dst_port, &data) else {
            tracing::warn!(%peer, len = data.len(), "datagram too large for the tunnel, dropped");
            return;
        };
        // Opening is asynchronous, so the channel exists before the stream
        // does: frames sent in that window queue rather than being lost, and
        // the writer task drains them the moment the stream comes up.
        let writer = self.tunnel_writers.entry(peer).or_insert_with(|| {
            let (tx, rx) = mpsc::channel::<Vec<u8>>(TUNNEL_WRITE_QUEUE);
            spawn_tunnel_writer(self.tunnel_control.clone(), peer, rx, self.tunnel_tx.clone());
            tx
        });
        if writer.try_send(frame).is_err() {
            // Full (congested) or closed (the task is on its way out and will
            // report WriterGone). Either way this datagram is gone -- which is
            // what "UDP" means to the guest.
            *self.tunnel_drops.entry(peer).or_insert(0) += 1;
        } else {
            self.plane.note_traffic(peer);
        }
    }

    fn handle_tunnel_event(&mut self, event: TunnelEvent) {
        match event {
            TunnelEvent::Frame { peer, src_port, dst_port, data } => {
                // A peer only tunnels to us because its punch failed, which
                // means ours to it will fail too. Marking it here is what lets
                // our *replies* take the tunnel: without it the return traffic
                // would sit in the punch queue until it aged out, and only the
                // side that degraded first would ever be heard.
                if !self.game_plane_ready(&peer) {
                    self.tunnel_fallback(peer);
                }
                self.announce_peer(peer);
                let virtual_ip = self.vip_for(peer);
                self.plane.note_traffic(peer);
                // Surfaced exactly as a punched datagram would be: the guest
                // must not be able to tell which path carried it.
                self.emit(Event::Datagram { virtual_ip, src_port, dst_port, data });
            }
            TunnelEvent::WriterGone { peer, error } => {
                // Drop the writer rather than the tunneled marker: the peer is
                // still unreachable by punch, so the next datagram should open
                // a fresh stream instead of falling back to nothing.
                self.tunnel_writers.remove(&peer);
                let drops = self.tunnel_drops.remove(&peer).unwrap_or(0);
                match error {
                    Some(err) => tracing::warn!(%peer, %err, "tunnel stream failed to open"),
                    None => tracing::info!(%peer, drops, "tunnel stream closed"),
                }
            }
        }
    }

    /// Only safe because the peer has stopped being a tunnel destination:
    /// `WriterGone` names a peer, not a writer, so a new writer created before
    /// the old event arrived would be torn down by it.
    fn close_tunnel(&mut self, peer: &PeerId) {
        // Dropping the sender ends the writer task, which closes the stream.
        if self.tunnel_writers.remove(peer).is_some() {
            let drops = self.tunnel_drops.remove(peer).unwrap_or(0);
            if drops > 0 {
                tracing::warn!(%peer, drops, "tunnel dropped datagrams while degraded");
            }
        }
        self.tunnel_drops.remove(peer);
    }

    /// A punch has run out of time: fall back to the tunnel so the player can
    /// still connect, and tell the shim the link is degraded.
    ///
    /// Only for a peer we hold a control connection to. The tunnel rides
    /// that connection, so a peer that was never connected has nothing to
    /// fall back to; telling the shim it was degraded would tell it the peer
    /// was reachable, which is the one thing it is not.
    fn tunnel_fallback(&mut self, peer: PeerId) {
        if !self.connected.contains(&peer) {
            tracing::debug!(%peer, "no control connection; nothing to tunnel over");
            return;
        }
        if !self.tunneled_peers.insert(peer) {
            return;
        }
        tracing::warn!(
            %peer,
            "punch failed; game traffic falling back to the control tunnel (degraded)"
        );
        self.emit(Event::Degraded { peer });
        self.flush_pending_datagrams(peer, None);
        self.game_path_recovered(peer, "tunnel");
    }

    /// The once-a-second sweep: punches past their deadline, held frames past
    /// their age, silent shard members, and a readdress that waited for a
    /// game to end.
    fn watchdog(&mut self) {
        self.prune_orphan_frames();
        let expired: Vec<PeerId> = self
            .punch_deadline
            .iter()
            .filter(|(peer, deadline)| {
                Instant::now() >= **deadline && !self.game_plane_ready(peer)
            })
            .map(|(peer, _)| *peer)
            .collect();
        for peer in expired {
            self.punch_deadline.remove(&peer);
            self.tunnel_fallback(peer);
        }
        self.prune_shard_members();
        if self.readdress_deferred && !self.game_in_progress() {
            tracing::info!("game over; applying the deferred shard readdress");
            self.readdress_all();
        }
    }

    /// One line saying what the shard is doing, so its state can be read at a
    /// glance instead of reconstructed from scattered events.
    fn log_shard_status(&self) {
        if !self.shard_enabled {
            return;
        }
        let Some(current) = &self.current_shard else {
            tracing::info!(
                candidates = self.shard_candidates.len(),
                swept = self.shard_swept_at.is_some(),
                "shard: none yet"
            );
            return;
        };
        let topic = gossipsub::IdentTopic::new(crate::protocol::shard_topic(&current.shard_id));
        let hash = topic.hash();
        let subscribed = self
            .swarm
            .behaviour()
            .gossipsub
            .all_peers()
            .filter(|(_, topics)| topics.iter().any(|t| **t == hash))
            .count();
        tracing::info!(
            shard = %hex16(&current.shard_id),
            addr = %format_vip(self.local_vip()),
            members_heard = self.shard_member_peers.len(),
            topic_peers = subscribed,
            reachable = self
                .shard_member_peers
                .iter()
                .filter(|p| self.connected.contains(p))
                .count(),
            tunneled = self.tunneled_peers.len(),
            oversized_drops = self.plane.oversized_drops(),
            "shard status"
        );
    }

    /// Turn a pump-task report into guest-facing state and events.
    fn handle_stream_event(&mut self, event: StreamEvent) {
        match event {
            StreamEvent::Opened { id, peer, local_port, remote_port, outbound, writer } => {
                let virtual_ip = self.vip_for(peer);
                self.streams.insert(StreamInfo { peer, local_port, remote_port, outbound });
                self.stream_writers.insert(id, writer);
                // A peer we are exchanging TCP with is app-relevant even if
                // nothing else made it so.
                self.announce_peer(peer);
                tracing::info!(
                    stream_id = id, %peer, local_port, remote_port, outbound,
                    "guest TCP connection open"
                );
                self.emit(Event::StreamOpened {
                    stream_id: id,
                    virtual_ip,
                    local_port,
                    remote_port,
                    outbound,
                });
            }
            StreamEvent::Data { id, data } => {
                self.emit(Event::StreamData { stream_id: id, data });
            }
            StreamEvent::Closed { id } => {
                self.streams.remove(id);
                self.stream_writers.remove(&id);
                tracing::debug!(stream_id = id, "guest TCP connection closed");
                self.emit(Event::StreamClosed { stream_id: id });
            }
            StreamEvent::ConnectFailed { peer, dst_port, error } => {
                let virtual_ip = self.vip_for(peer);
                tracing::warn!(%peer, dst_port, %error, "guest TCP connect failed");
                self.emit(Event::StreamConnectFailed {
                    virtual_ip,
                    dst_port,
                    message: error,
                });
            }
        }
    }

    /// Kick a NAT punch toward `peer`.
    fn begin_punch(&mut self, peer: PeerId) {
        if self.force_tunnel {
            tracing::debug!(%peer, "punch skipped (force_tunnel)");
            if !self.connected.contains(&peer) {
                // The tunnel needs the connection; held datagrams go when
                // it comes (ConnectionEstablished starts the game path).
                self.seek_peer(peer);
                return;
            }
            self.tunnel_fallback(peer);
            return;
        }
        self.punch_deadline.entry(peer).or_insert_with(|| Instant::now() + PUNCH_TIMEOUT);
        let nonce: [u8; 16] = rand::random();
        let candidates = self.local_candidates();
        self.pending_punch_nonce.insert(nonce, peer);
        let eph_pub = self.local_game_pubkey(peer);
        tracing::debug!(
            %peer,
            candidates = ?candidates,
            local_addr = %format_vip(self.local_vip()),
            "punch offer sent"
        );
        let id = self
            .swarm
            .behaviour_mut()
            .punch
            .send_request(&peer, PunchOffer { nonce, candidates, eph_pub });
        self.pending_punch_offers.insert(id, peer);
    }

    /// Is this address a broadcast for our shard?
    ///
    /// Accepts both the limited broadcast a System Link title typically uses
    /// (255.255.255.255) and our own subnet's directed broadcast. Titles use
    /// either, and on a real LAN both would reach the same machines.
    fn is_broadcast_vip(&self, vip: u32) -> bool {
        if vip == u32::MAX {
            return true;
        }
        match &self.current_shard {
            Some(current) => {
                vip == subnet::broadcast_address(VIP_NETWORK_BASE, &current.shard_id)
            }
            None => false,
        }
    }

    /// Deliver a datagram to every shard member.
    ///
    /// Members without a punched game-plane endpoint cannot be reached, so a
    /// punch is started for them: discovery broadcasts repeat, and the next
    /// one gets through. Punching lazily like this means only titles that
    /// actually broadcast pay for it, rather than every shard punching a full
    /// mesh on the chance someone might.
    fn broadcast_datagram(&mut self, src_port: u16, dst_port: u16, data: &[u8]) {
        // Sealed once per recipient rather than built once and reused: keys
        // are per peer (§6), so there is no such thing as a frame every member
        // can read. At a full 255-member shard that is 255 encryptions per
        // broadcast — cheap per datagram, but it scales with the shard and is
        // one of the things §17.6 flags as unmeasured at that size.
        let members: Vec<PeerId> = self.shard_member_peers.iter().copied().collect();
        let mut sent = 0usize;
        let mut tunneled = 0usize;
        let mut punching = 0usize;
        let mut unreachable = 0usize;
        for peer in members {
            match self.plane.endpoint(&peer) {
                Some(endpoint) if self.plane.has_keys(&peer) => {
                    if let Some(frame) = self.seal_datagram(peer, src_port, dst_port, data) {
                        let _ = self.game_socket.try_send_to(&frame, endpoint);
                        sent += 1;
                    }
                }
                // Punched but not yet keyed. Skipped quietly: the beat repeats.
                Some(_) => {}
                None if self.tunneled_peers.contains(&peer) => {
                    // Degraded members take the same path a unicast would.
                    self.send_tunneled(peer, src_port, dst_port, data.to_vec());
                    tunneled += 1;
                }
                None if self.punch_deadline.contains_key(&peer)
                    || self.pending_punch_offers.values().any(|p| *p == peer) =>
                {
                    punching += 1; // a punch is already in flight
                }
                None if self.connected.contains(&peer) => {
                    self.begin_punch(peer);
                    punching += 1;
                }
                // No control connection to carry the offer: the beat repeats,
                // and shard_ensure_reachable is working on the connection.
                None => unreachable += 1,
            }
        }
        tracing::debug!(dst_port, sent, tunneled, punching, unreachable, "shard broadcast");
    }

    /// Tell the shim our current address, so the XNADDR it reports to the
    /// game matches the subnet we are actually on.
    /// Record a confirmed public endpoint (from UPnP or AutoNAT) and, the first
    /// time we see it, surface a bare `host:port` for the player to hand to a
    /// peer for a manual direct connect (§9 direct path).
    fn note_external_addr(&mut self, address: Multiaddr) {
        for proto in address.iter() {
            match proto {
                Protocol::Ip4(ip) => self.external_ips.push(ip.into()),
                Protocol::Ip6(ip) => self.external_ips.push(ip.into()),
                _ => {}
            }
        }
        self.external_ips.sort();
        self.external_ips.dedup();

        if let Some(endpoint) = dialable_endpoint(&address) {
            if self.announced_external.insert(endpoint.clone()) {
                self.emit(Event::ExternalAddress { addr: endpoint });
            }
        }

        // Now that we are confirmed reachable on a routable address, advertise
        // ourselves as a relay in the DHT (§9): NAT'd peers query the same key
        // to find a relay to punch through, including ones they have never met.
        if !self.relay_provider_registered && multiaddr_is_global(&address) {
            let key = kad::RecordKey::new(&crate::protocol::RELAYS_KEY);
            match self.swarm.behaviour_mut().kad.start_providing(key) {
                Ok(_) => {
                    self.relay_provider_registered = true;
                    tracing::info!("registered as a mesh relay provider (reachable)");
                }
                Err(err) => tracing::debug!(%err, "relay provider registration failed"),
            }
        }
    }

    /// Query the DHT for `rexnet/v1/relays` providers (§9) when we still need
    /// relays. Found providers are dialled; the identify handler then reserves a
    /// circuit through any that offer the hop on a routable address. This is the
    /// active half of relay discovery -- it finds relays we have never connected
    /// to, which identify-on-existing-connections alone cannot.
    fn discover_relays(&mut self) {
        if self.auto_relays.len() >= MAX_AUTO_RELAYS || self.pending_relay_search.is_some() {
            return;
        }
        let key = kad::RecordKey::new(&crate::protocol::RELAYS_KEY);
        let id = self.swarm.behaviour_mut().kad.get_providers(key);
        self.pending_relay_search = Some(id);
    }

    /// Hold an automatic circuit reservation on a connected, publicly reachable
    /// mesh peer that offers the circuit-v2 hop (§9 torrent model). This is how
    /// a NAT'd node gets a relay from the swarm itself, with no configured
    /// server. Capped at [`MAX_AUTO_RELAYS`] so a large mesh does not become an
    /// N-squared reservation storm.
    fn maybe_auto_reserve_relay(
        &mut self,
        peer_id: PeerId,
        protocols: &[StreamProtocol],
        listen_addrs: &[Multiaddr],
    ) {
        if self.auto_relays.len() >= MAX_AUTO_RELAYS || self.auto_relays.contains(&peer_id) {
            return;
        }
        // Never a bootstrap node (they are DHT entry points, not our relays),
        // and not one that refused us lately.
        if self.bootstrap_peers.contains(&peer_id) {
            return;
        }
        if let Some(until) = self.auto_relay_backoff.get(&peer_id) {
            if Instant::now() < *until {
                return;
            }
        }
        // Peer must speak the circuit-v2 hop, or it cannot relay for us.
        let offers_hop = protocols
            .iter()
            .any(|p| p.as_ref() == "/libp2p/circuit/relay/0.2.0/hop");
        if !offers_hop {
            return;
        }
        // Reserve only through a globally routable address that is not itself a
        // circuit: a LAN-only relay is useless to a NAT'd peer elsewhere, and a
        // circuit inside a circuit cannot be listened on.
        let Some(base) = reservable_addr(listen_addrs) else {
            return;
        };
        // Circuit = <relay transport>/p2p/<relay>/p2p-circuit. Drop any trailing
        // /p2p the relay already advertised so it is not duplicated.
        let mut circuit = base.clone();
        if matches!(circuit.iter().last(), Some(Protocol::P2p(_))) {
            circuit.pop();
        }
        let circuit = circuit.with(Protocol::P2p(peer_id)).with(Protocol::P2pCircuit);
        match self.swarm.listen_on(circuit.clone()) {
            Ok(listener) => {
                self.auto_relays.insert(peer_id);
                self.auto_relay_listeners.insert(listener, peer_id);
                tracing::info!(%peer_id, %circuit,
                    "mesh relay: auto-reserving a circuit through a reachable peer");
            }
            Err(err) => {
                tracing::debug!(%peer_id, %err, "auto relay reservation failed to start");
            }
        }
    }

    /// A request-response failure worth saying. UnsupportedProtocols is the
    /// one that is not a network fault: the peer is a RexNet build without
    /// this protocol, and the player should hear that -- once per peer and
    /// protocol, not on every retry -- rather than "no answer".
    fn note_outbound_failure(
        &mut self,
        peer: PeerId,
        what: &'static str,
        error: &request_response::OutboundFailure,
    ) {
        if matches!(error, request_response::OutboundFailure::UnsupportedProtocols) {
            if self.unsupported_reported.insert((peer, what)) {
                tracing::warn!(%peer, what, "peer runs a build without this protocol");
                self.emit(Event::Error {
                    message: format!("{peer} runs an older build that cannot take a {what} message"),
                });
            }
        } else {
            tracing::debug!(%peer, what, %error, "request failed");
        }
    }

    fn remember_peer_routes(&mut self, peer: PeerId, addresses: impl IntoIterator<Item = Multiaddr>) {
        remember_routes(&mut self.peer_routes, peer, addresses, Instant::now());
    }

    fn maintain_recoveries(&mut self) {
        let now = Instant::now();
        let peers: Vec<PeerId> = self.recovering.keys().copied().collect();
        for peer in peers {
            if now.saturating_duration_since(self.recovering[&peer].started) >= PEER_RECOVERY_GRACE {
                self.expire_peer_recovery(peer);
            } else {
                self.recover_peer(peer);
            }
        }
    }

    fn recover_peer(&mut self, peer: PeerId) {
        let now = Instant::now();
        let Some(recovery) = self.recovering.get_mut(&peer) else { return };
        if self.connected.contains(&peer) || now < recovery.next_dial {
            return;
        }
        recovery.next_dial = now + PEER_RECOVERY_RETRY;
        let addresses = reconnect_routes(self.peer_routes.get(&peer), now);
        // Explicit alternatives must not be extended with the failed QUIC path.
        let dial = if addresses.is_empty() {
            DialOpts::peer_id(peer).condition(PeerCondition::NotDialing).build()
        } else {
            DialOpts::peer_id(peer).condition(PeerCondition::NotDialing).addresses(addresses).build()
        };
        if let Err(error) = self.swarm.dial(dial) {
            tracing::debug!(%peer, ?error, "recovery dial not started");
            if let Some(recovery) = self.recovering.get_mut(&peer) {
                if !recovery.looked_up {
                    recovery.looked_up = true;
                    self.lookup_peer(peer);
                }
            }
        }
    }

    fn expire_peer_recovery(&mut self, peer: PeerId) {
        if let Some(recovery) = self.recovering.remove(&peer) {
            tracing::info!(%peer, elapsed_ms = recovery.started.elapsed().as_millis(),
                control_connected = self.connected.contains(&peer), "game peer recovery timed out");
            self.pending_datagrams.remove(&peer);
            if let Some(reply) = self.punch_replies.remove(&peer) {
                let _ = reply.send(Err("peer recovery timed out".into()));
            }
            self.report_peer_departure(peer);
        }
    }

    /// A game path to a recovering peer is up again: the title never heard it
    /// leave. Too late, and it is told now rather than kept in limbo.
    fn game_path_recovered(&mut self, peer: PeerId, path: &str) {
        if !self.connected.contains(&peer) {
            return;
        }
        if self.recovering.get(&peer).is_some_and(|r| r.started.elapsed() >= PEER_RECOVERY_GRACE) {
            self.expire_peer_recovery(peer);
            return;
        }
        if let Some(recovery) = self.recovering.remove(&peer) {
            tracing::info!(%peer, path, elapsed_ms = recovery.started.elapsed().as_millis(), "game path recovered");
        }
    }

    /// The peer is gone for good (no recovery, or recovery failed): it is no
    /// longer in a session with us, no longer a shard member, and the shim
    /// hears it left if it ever heard it arrive.
    fn report_peer_departure(&mut self, peer: PeerId) {
        self.session_peers.remove(&peer);
        self.plane.forget_traffic(&peer);
        if self.announced.remove(&peer) {
            self.emit(Event::PeerDisconnected { peer });
        }
        self.shard_member_left(peer, "disconnected");
    }

    /// A circuit listener ended: free its relay slot; if it failed, back the
    /// relay off so the next identify does not pick it again at once.
    fn auto_relay_listener_closed(
        &mut self,
        listener: libp2p::core::transport::ListenerId,
        failed: bool,
    ) {
        let Some(peer) = self.auto_relay_listeners.remove(&listener) else { return };
        if !self.auto_relay_listeners.values().any(|other| *other == peer) {
            self.auto_relays.remove(&peer);
        }
        if failed {
            let now = Instant::now();
            self.auto_relay_backoff.retain(|_, until| now < *until);
            self.auto_relay_backoff.insert(peer, now + AUTO_RELAY_BACKOFF);
            tracing::info!(%peer, "mesh relay: reservation refused or failed; slot freed, relay backed off");
        }
    }

    /// Close an inbound connection from a peer that is nothing to us: not
    /// wanted, not a relay of ours, not a friend, not a bootstrap node.
    fn close_public_inbound(&mut self, peer: PeerId, connection: libp2p::swarm::ConnectionId) {
        if self.wanted_peers.contains(&peer)
            || self.auto_relays.contains(&peer)
            || self.relay_addrs.iter().any(|(relay, _)| *relay == peer)
            || self.relays.contains(&peer)
            || self.friend_store.is_friend(&peer)
            || self.bootstrap_peers.contains(&peer)
            || self.pending_friend_msgs.contains_key(&peer)
            || self.rexnet_peers.contains(&peer)
        {
            return;
        }
        if self.swarm.close_connection(connection) {
            tracing::debug!(%peer, "closed a public inbound connection");
        }
    }

    fn announce_local_address(&mut self) {
        let vip = self.local_vip();
        if self.announced_vip == Some(vip) {
            return;
        }
        self.announced_vip = Some(vip);
        tracing::info!(addr = %format_vip(vip), "local address");
        self.emit(Event::LocalAddress { virtual_ip: vip });
    }

    /// Our own address: our host on the shard's /24 when we are in one, read
    /// from the pinned table so it moves only when the table is rebuilt; else
    /// the reserved-range identity the shim uses before any shard exists.
    fn local_vip(&self) -> u32 {
        if let Some(current) = &self.current_shard {
            if let Some(host) = self.shard_hosts.get(self.swarm.local_peer_id()) {
                return subnet::address(VIP_NETWORK_BASE, &current.shard_id, *host);
            }
        }
        self.reserved_vip_for(self.swarm.local_peer_id())
    }

    /// Candidate endpoints for the punch: every local interface we listen on
    /// plus AutoNAT-confirmed external addresses, all with the game port.
    /// (External candidates assume the NAT maps the game port unchanged —
    /// true for full-cone/port-preserving NATs; the punch tries all pairs.)
    fn local_candidates(&self) -> Vec<SocketAddr> {
        let game_port = match self.game_socket.local_addr() {
            Ok(addr) => addr.port(),
            Err(_) => return Vec::new(),
        };
        let mut ips: Vec<IpAddr> = Vec::new();
        for addr in &self.listen_addrs {
            for proto in addr.iter() {
                match proto {
                    Protocol::Ip4(ip) if !ip.is_unspecified() => ips.push(ip.into()),
                    Protocol::Ip6(ip) if !ip.is_unspecified() => ips.push(ip.into()),
                    _ => {}
                }
            }
        }
        ips.extend(self.external_ips.iter().copied());
        ips.sort();
        ips.dedup();
        ips.into_iter().map(|ip| SocketAddr::new(ip, game_port)).collect()
    }

    fn spawn_probes(&self, nonce: [u8; 16], candidates: Vec<SocketAddr>) {
        let socket = self.game_socket.clone();
        tokio::spawn(async move {
            for _ in 0..PROBE_ROUNDS {
                // Stamped per round: the peer echoes it back untouched, so the
                // round trip is measured entirely against our own clock and
                // needs no agreement about time.
                let mut probe = Vec::with_capacity(PROBE_FRAME_LEN);
                probe.push(FRAME_PROBE);
                probe.extend_from_slice(&nonce);
                probe.extend_from_slice(&now_micros().to_be_bytes());
                for addr in &candidates {
                    let _ = socket.send_to(&probe, addr).await;
                }
                tokio::time::sleep(PROBE_INTERVAL).await;
            }
        });
    }

    /// A probe (or probe-ack) with a known nonce arrived from `addr`:
    /// that address is the peer's working game endpoint.
    fn lock_game_endpoint(&mut self, nonce: [u8; 16], addr: SocketAddr) {
        let Some(peer) = self.pending_punch_nonce.get(&nonce).copied() else {
            tracing::trace!(%addr, "probe with unknown nonce ignored");
            return;
        };
        if self.plane.endpoint(&peer).is_some() {
            return; // first authenticated pair wins (§8.4)
        }
        tracing::info!(
            %peer, %addr,
            peer_addr = %format_vip(self.vip_by_peer.get(&peer).copied().unwrap_or(0)),
            "game endpoint punched"
        );
        self.plane.map_endpoint(peer, addr);
        self.vip_for(peer);
        // The probe travels direct while the punch answer comes back over the
        // control connection, so the endpoint can map before keys exist.
        // Whichever lands second finishes the job; until then the deadline
        // stays armed so the watchdog can still fall back to the tunnel.
        if self.plane.has_keys(&peer) {
            self.complete_game_plane(peer, addr);
        } else {
            tracing::warn!(%peer, %addr, "endpoint punched but no keys yet; holding traffic");
        }
    }

    /// Endpoint and keys are both in place: the plane can carry traffic.
    fn complete_game_plane(&mut self, peer: PeerId, addr: SocketAddr) {
        self.punch_deadline.remove(&peer);
        if self.tunneled_peers.remove(&peer) {
            tracing::info!(%peer, "punch succeeded; leaving the degraded tunnel");
            self.close_tunnel(&peer);
        }
        self.flush_pending_datagrams(peer, Some(addr));
        self.deliver_orphan_frames(peer, addr);
        self.game_path_recovered(peer, "udp");
        self.emit(Event::PunchResult { peer, ok: true });
        if let Some(reply) = self.punch_replies.remove(&peer) {
            let _ = reply.send(Ok(addr));
        }
    }

    fn handle_game_frame(&mut self, addr: SocketAddr, frame: Vec<u8>) {
        match frame.first().copied() {
            Some(FRAME_DATA) => {
                // [0x01][counter u64be][ChaCha20-Poly1305 ciphertext+tag]
                if frame.len() < 1 + crypto::CRYPTO_OVERHEAD {
                    return;
                }
                // Unmapped, or mapped but not yet keyed because the probe beat
                // the punch answer. Either way hold rather than drop: this
                // window is exactly when a join's opening handshake arrives.
                let mapped = self.plane.peer_at(&addr);
                let Some(peer) = mapped.filter(|p| self.plane.has_keys(p)) else {
                    // The two sides complete their punches independently, and
                    // the peer flushes whatever it queued the instant *its*
                    // side completes -- which can be milliseconds before ours.
                    // Dropping here discards the opening handshake of a join
                    // and the title then waits out its own timeout. Hold the
                    // frames until the endpoint is mapped instead; this is the
                    // receive-side mirror of the send-side punch queue.
                    //
                    // Bounded in sources as well as depth: the source whose
                    // oldest frame is oldest makes room for a new one.
                    if !self.orphan_frames.contains_key(&addr)
                        && self.orphan_frames.len() >= ORPHAN_SOURCES_MAX
                    {
                        let oldest = self
                            .orphan_frames
                            .iter()
                            .filter_map(|(source, queue)| queue.first().map(|(at, _)| (*at, *source)))
                            .min()
                            .map(|(_, source)| source);
                        if let Some(source) = oldest {
                            self.orphan_frames.remove(&source);
                        }
                    }
                    let queue = self.orphan_frames.entry(addr).or_default();
                    if queue.len() >= PUNCH_QUEUE_MAX {
                        queue.remove(0);
                    }
                    queue.push((Instant::now(), frame));
                    tracing::debug!(%addr, queued = queue.len(), "datagram held: endpoint not mapped yet");
                    return;
                };
                let Some((src_port, dst_port, data)) = self.open_datagram(peer, &frame) else {
                    return;
                };
                let virtual_ip = self.vip_for(peer);
                self.emit(Event::Datagram { virtual_ip, src_port, dst_port, data });
            }
            Some(FRAME_PROBE) | Some(FRAME_PROBE_ACK) if frame.len() == PROBE_FRAME_LEN => {
                let mut nonce = [0u8; 16];
                nonce.copy_from_slice(&frame[1..17]);
                let is_probe = frame[0] == FRAME_PROBE;
                self.lock_game_endpoint(nonce, addr);
                if is_probe {
                    // Echo the stamp back verbatim; its meaning is the
                    // sender's alone.
                    let mut ack = Vec::with_capacity(PROBE_FRAME_LEN);
                    ack.push(FRAME_PROBE_ACK);
                    ack.extend_from_slice(&frame[1..PROBE_FRAME_LEN]);
                    let _ = self.game_socket.try_send_to(&ack, addr);
                } else {
                    let mut stamp = [0u8; 8];
                    stamp.copy_from_slice(&frame[17..PROBE_FRAME_LEN]);
                    self.note_rtt(addr, u64::from_be_bytes(stamp));
                }
            }
            _ => tracing::trace!(%addr, len = frame.len(), "bad game frame dropped"),
        }
    }

    /// Returns false on shutdown.
    fn handle_command(&mut self, cmd: Command) -> bool {
        match cmd {
            Command::ConnectPeer { peer } => {
                // Addresses come from kad/mdns/identify via the behaviours'
                // address book; kick a DHT lookup in parallel for peers we
                // have never seen.
                self.swarm.behaviour_mut().kad.get_closest_peers(peer);
                self.wanted_peers.insert(peer);
                // Already connected (e.g. via a session descriptor query):
                // the dial below no-ops, so surface the peer here.
                self.announce_peer(peer);
                if let Err(err) = self.swarm.dial(peer) {
                    self.emit(Event::Error { message: format!("dial {peer}: {err}") });
                }
            }
            Command::ConnectManual { multiaddr } => match multiaddr.parse::<Multiaddr>() {
                Ok(addr) => {
                    if let Some(Protocol::P2p(peer)) = addr.iter().last() {
                        self.wanted_peers.insert(peer);
                    }
                    if let Err(err) = self.swarm.dial(addr) {
                        self.emit(Event::Error { message: format!("dial {multiaddr}: {err}") });
                    }
                }
                Err(err) => {
                    self.emit(Event::Error { message: format!("bad multiaddr {multiaddr}: {err}") });
                }
            },
            Command::Punch { peer, reply } => {
                // A punch target is app-relevant by definition; the
                // request-response below dials if not yet connected.
                self.wanted_peers.insert(peer);
                self.announce_peer(peer);
                if let Some(endpoint) = self.ready_endpoint(&peer) {
                    if let Some(reply) = reply {
                        let _ = reply.send(Ok(endpoint));
                    } else {
                        self.emit(Event::PunchResult { peer, ok: true });
                    }
                    return true;
                }
                if let Some(reply) = reply {
                    self.punch_replies.insert(peer, reply);
                }
                self.begin_punch(peer);
            }
            Command::SendDatagram { virtual_ip, src_port, dst_port, data, reliable: _ } => {
                // Broadcast goes to the whole shard rather than one peer --
                // this is what lets a System Link title discover players by
                // shouting instead of naming a host (§17.3.6).
                if self.is_broadcast_vip(virtual_ip) {
                    self.broadcast_datagram(src_port, dst_port, &data);
                    return true;
                }
                let Some(peer) = self.peer_by_vip.get(&virtual_ip) else {
                    if self.unknown_vip_warns.allow(virtual_ip) {
                        tracing::warn!(
                            addr = %format_vip(virtual_ip),
                            "datagram to unknown virtual IP dropped (further drops to this address silent for a while)"
                        );
                    }
                    return true;
                };
                let peer = *peer;
                if let Some(endpoint) = self.ready_endpoint(&peer) {
                    if let Some(frame) = self.seal_datagram(peer, src_port, dst_port, &data) {
                        let _ = self.game_socket.try_send_to(&frame, endpoint);
                    }
                } else if self.tunneled_peers.contains(&peer) {
                    self.send_tunneled(peer, src_port, dst_port, data);
                } else {
                    // Hold it: the punch is usually milliseconds away and this
                    // is typically the opening message of a join.
                    let queue = self.pending_datagrams.entry(peer).or_default();
                    if queue.len() >= PUNCH_QUEUE_MAX {
                        queue.remove(0);
                    }
                    queue.push(PendingDatagram {
                        queued_at: Instant::now(),
                        src_port,
                        dst_port,
                        data,
                    });
                    tracing::debug!(
                        %peer, dst_port, queued = queue.len(),
                        "datagram held until punch resolves"
                    );
                }
            }
            Command::SetPresence { title_id, state, rich } => {
                self.my_presence.title_id = title_id;
                self.my_presence.state = state;
                self.my_presence.rich = rich;
                self.push_presence(None);
            }
            Command::SetDisplayName { name } => {
                let name = crate::identity::sanitize_display_name(&name);
                self.display_name = name.clone();
                self.my_presence.display_name = name;
                self.push_presence(None);
            }
            Command::FriendRequest { peer, note } => {
                if peer == *self.swarm.local_peer_id() {
                    // Two installs made by copying one carry the same identity.key, so
                    // the other machine's friend code IS ours: nothing can ever connect.
                    tracing::warn!("friend request to our own peer id: a copied install shares identity.key");
                    self.emit(Event::Error {
                        message: "that friend code is this node's own id: a copied install shares \
                                  identity.key; delete the copy's rexnet/identity.key and restart it"
                            .into(),
                    });
                    return true;
                }
                if self.friend_store.is_friend(&peer) {
                    self.emit(Event::FriendAccepted { peer });
                    return true;
                }
                self.friend_store.request_sent(peer);
                let msg = FriendMsg::Request {
                    schema: 1,
                    display_name: self.display_name.clone(),
                    note,
                };
                self.friend_send(peer, msg);
            }
            Command::FriendAccept { peer } => {
                if !self.friend_store.pending_in.contains(&peer) {
                    self.emit(Event::Error { message: format!("no pending request from {peer}") });
                    return true;
                }
                self.friend_store.add(peer);
                self.friend_send(peer, FriendMsg::Accept { schema: 1 });
                self.emit(Event::FriendAccepted { peer });
                self.push_presence(Some(peer));
            }
            Command::FriendDismiss { peer } => {
                if self.friend_store.dismiss(&peer) {
                    self.friend_send(peer, FriendMsg::Decline { schema: 1 });
                }
            }
            Command::FriendRemove { peer } => {
                self.friend_store.remove(&peer);
                self.pending_friend_msgs.remove(&peer);
                if self.connected.contains(&peer) {
                    self.swarm.behaviour_mut().friend.send_request(&peer, FriendMsg::Remove { schema: 1 });
                }
                self.emit(Event::FriendRemoved { peer });
            }
            Command::SendInvite { peer } => {
                let Some(session) = self.local_session.as_ref() else {
                    self.emit(Event::Error { message: "invite without a local session".into() });
                    return true;
                };
                let expires = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_secs() + 300)
                    .unwrap_or(0);
                let msg = InviteMsg::Offer {
                    schema: 1,
                    title_id: self.title_id,
                    session_id: session.id,
                    expires_unix: expires,
                };
                // Invited peers are served the descriptor even when the
                // session is private; the invite is the host handing it out.
                self.invited.insert(peer);
                // The player asked for this peer: a dial failure is theirs to
                // hear. Retries after that are quiet.
                self.wanted_peers.insert(peer);
                self.invite_send(peer, msg);
            }
            Command::InviteReply { peer, accept } => {
                if accept {
                    // Accepting puts us in their session: addresses are
                    // pinned from here, until title traffic takes over or
                    // the deferral runs out.
                    self.session_peers.insert(peer, Instant::now());
                }
                let msg = InviteMsg::Reply { schema: 1, session_id: [0; 16], accept };
                // Same deferral as the offer; the descriptor query that an
                // accept needs goes with the reply (send_invite_now).
                self.invite_send(peer, msg);
            }
            Command::SessionCreate { id, slots_total, slots_open, is_public } => {
                tracing::info!(
                    session_id = %hex16(&id), slots_total, slots_open, is_public,
                    "session created"
                );
                // Admissions belong to one session: nobody invited to the
                // last one is invited to this one.
                self.invited.clear();
                self.local_session =
                    Some(LocalSession { id, slots_total, slots_open, is_public, join_token: 0 });
                if is_public {
                    let key = kad::RecordKey::new(&crate::protocol::session_key(self.title_id));
                    if let Err(err) = self.swarm.behaviour_mut().kad.start_providing(key) {
                        tracing::warn!(%err, "session provider record publish failed");
                    }
                }
            }
            Command::SessionDelete => {
                self.invited.clear();
                if let Some(session) = self.local_session.take() {
                    tracing::info!(session_id = %hex16(&session.id), "session deleted");
                    if session.is_public {
                        let key =
                            kad::RecordKey::new(&crate::protocol::session_key(self.title_id));
                        self.swarm.behaviour_mut().kad.stop_providing(&key);
                    }
                }
            }
            Command::SessionSetJoinToken { token } => match self.local_session.as_mut() {
                Some(session) => {
                    session.join_token = token;
                    tracing::info!(has_token = token != 0, "session join token set");
                }
                None => tracing::debug!("join token set with no local session; ignored"),
            },
            Command::SessionSetSlots { open, total } => match self.local_session.as_mut() {
                Some(session) => {
                    session.slots_open = u8::try_from(open).unwrap_or(u8::MAX);
                    session.slots_total = u8::try_from(total).unwrap_or(u8::MAX);
                    tracing::info!(
                        slots_open = session.slots_open, slots_total = session.slots_total,
                        "session slots updated"
                    );
                }
                None => tracing::debug!("session slots set with no local session; ignored"),
            },
            Command::ShardEnable { cap } => {
                self.shard_cap = if cap == 0 { DEFAULT_SHARD_CAP } else { cap };
                if !self.shard_enabled {
                    self.shard_enabled = true;
                    tracing::info!(cap = self.shard_cap, "ambient shard enabled");
                    // Look before creating: joining an existing shard is the
                    // whole point, so never create one on the enable path.
                    self.shard_discover();
                }
            }
            Command::ShardJoin { shard_id } => {
                // An explicit join -- a Discord invite names the host's shard -- outranks
                // placement and the migration cooldown: the player asked to be with
                // this person. Known already: go now. Not yet: remembered, and taken the
                // moment that shard's descriptor arrives (the host is dialled for the
                // session join, and a rexnet peer is queried for its shard on identify).
                if !self.shard_enabled {
                    self.shard_enabled = true;
                    self.shard_cap = DEFAULT_SHARD_CAP;
                }
                if self.current_shard.as_ref().is_some_and(|c| c.shard_id == shard_id) {
                    return true;
                }
                self.shard_preferred = Some(shard_id);
                if self.game_in_progress() {
                    tracing::info!(shard = %hex16(&shard_id), "named shard remembered; a game path is up, joining after it");
                    return true;
                }
                if let Some(desc) = self.shard_candidates.get(&shard_id).cloned() {
                    tracing::info!(shard = %hex16(&shard_id), "joining the named shard");
                    self.shard_leave();
                    self.shard_enter(desc);
                    self.last_shard_move = Some(Instant::now());
                } else {
                    tracing::info!(shard = %hex16(&shard_id), "named shard not seen yet; waiting for its descriptor");
                    self.shard_discover();
                }
            }
            Command::ShardDisable => {
                if self.shard_enabled {
                    self.shard_enabled = false;
                    self.shard_leave();
                    self.shard_candidates.clear();
                    self.pending_shard_search = None;
                    self.shard_swept_at = None;
                    tracing::info!("ambient shard disabled");
                }
            }
            Command::StreamConnect { virtual_ip, src_port, dst_port } => {
                let Some(peer) = self.peer_by_vip.get(&virtual_ip).copied() else {
                    tracing::warn!(virtual_ip, "guest TCP connect to unknown virtual IP");
                    self.emit(Event::StreamConnectFailed {
                        virtual_ip,
                        dst_port,
                        message: "unknown virtual IP".into(),
                    });
                    return true;
                };
                let mut control = self.stream_control.clone();
                let events = self.stream_tx.clone();
                let next_id = self.next_stream_id.clone();
                tokio::spawn(async move {
                    use libp2p::futures::io::AsyncWriteExt;
                    let protocol = StreamProtocol::new(crate::protocol::TCP);
                    let mut stream = match control.open_stream(peer, protocol).await {
                        Ok(stream) => stream,
                        Err(err) => {
                            let _ = events.send(StreamEvent::ConnectFailed {
                                peer,
                                dst_port,
                                error: err.to_string(),
                            });
                            return;
                        }
                    };
                    // Name the destination port before any guest bytes: the
                    // acceptor cannot match the stream to a listener without it.
                    let header =
                        StreamHeader { schema: STREAM_SCHEMA, dst_port, src_port }.encode();
                    if stream.write_all(&header).await.is_err() || stream.flush().await.is_err() {
                        let _ = events.send(StreamEvent::ConnectFailed {
                            peer,
                            dst_port,
                            error: "header write failed".into(),
                        });
                        return;
                    }
                    let id = next_id.fetch_add(1, Ordering::Relaxed).max(1);
                    let (writer_tx, writer_rx) = mpsc::unbounded_channel::<Vec<u8>>();
                    if events
                        .send(StreamEvent::Opened {
                            id,
                            peer,
                            local_port: src_port,
                            remote_port: dst_port,
                            outbound: true,
                            writer: writer_tx,
                        })
                        .is_err()
                    {
                        return;
                    }
                    pump_stream(id, stream, writer_rx, events).await;
                });
            }
            Command::StreamSend { stream_id, data } => {
                match self.stream_writers.get(&stream_id) {
                    Some(writer) => {
                        if writer.send(data).is_err() {
                            tracing::debug!(stream_id, "guest TCP write to a closed stream");
                        }
                    }
                    None => tracing::debug!(stream_id, "guest TCP write to an unknown stream"),
                }
            }
            Command::StreamClose { stream_id } => {
                // Dropping the writer ends the pump, which reports Closed.
                self.stream_writers.remove(&stream_id);
            }
            Command::ShardStatus { reply } => {
                let _ = reply.send(self.current_shard.as_ref().map(|d| d.shard_id));
            }
            Command::SessionQuery { peer, token } => {
                // Ask one host for its descriptor directly: a join that already
                // knows the host (an invite, a pasted code) need not wait on a
                // DHT search. The answer takes the same path a search result
                // does. The token, if any, is what admits us to a private one.
                self.wanted_peers.insert(peer);
                self.swarm
                    .behaviour_mut()
                    .session
                    .send_request(&peer, SessionQuery { schema: 1, token });
            }
            Command::SessionSearch => {
                let key = kad::RecordKey::new(&crate::protocol::session_key(self.title_id));
                let id = self.swarm.behaviour_mut().kad.get_providers(key);
                self.pending_session_search = Some((id, Vec::new()));
            }
            Command::Echo { peer, reply } => {
                let id = self
                    .swarm
                    .behaviour_mut()
                    .echo
                    .send_request(&peer, EchoPayload(b"rexnet-echo".to_vec()));
                self.pending_echo.insert(id, (Instant::now(), reply));
            }
            Command::FindPeer { peer, reply } => {
                let id = self.swarm.behaviour_mut().kad.get_closest_peers(peer);
                self.pending_find.insert(id, (peer, reply));
            }
            Command::LocalAddrs { reply } => {
                let _ = reply.send(self.listen_addrs.clone());
            }
            Command::Shutdown => return false,
        }
        true
    }

    fn handle_swarm_event(&mut self, event: SwarmEvent<BehaviourEvent>) {
        match event {
            SwarmEvent::ListenerClosed { listener_id, reason, .. } => {
                self.auto_relay_listener_closed(listener_id, reason.is_err());
            }
            SwarmEvent::ListenerError { listener_id, error } => {
                tracing::debug!(?listener_id, %error, "listener error");
                self.auto_relay_listener_closed(listener_id, true);
            }
            SwarmEvent::NewListenAddr { address, .. } => {
                tracing::info!(%address, "listening");
                self.listen_addrs.push(address);
            }
            SwarmEvent::ExternalAddrConfirmed { address } => {
                tracing::info!(%address, "external address confirmed");
                self.note_external_addr(address);
            }
            SwarmEvent::ConnectionEstablished { peer_id, endpoint, num_established, .. } => {
                // Debug, not info: the public DHT crawl connects to hundreds of
                // peers that have nothing to do with us, and at 50% of the log
                // it buries everything that matters. A peer becomes interesting
                // when identify shows it speaks our protocols -- that is logged
                // at info instead.
                tracing::debug!(%peer_id, endpoint = %endpoint.get_remote_address(), "connected");
                self.remember_peer_routes(peer_id, reconnect_address(&endpoint));
                if num_established.get() == 1 {
                    let wanted = self.wanted_peers.remove(&peer_id);
                    self.connected.insert(peer_id);
                    // A game peer coming back: rebuild its game path now rather
                    // than at the title's next datagram, which may be a while.
                    // One that was tunnelled goes straight back to the tunnel:
                    // its punch has failed once already, and the restored
                    // control connection is the whole of what the tunnel needs.
                    if let Some(recovery) = self.recovering.get(&peer_id) {
                        if recovery.tunneled {
                            tracing::info!(%peer_id, "control connection back; resuming the tunnel");
                            self.tunnel_fallback(peer_id);
                        } else {
                            tracing::info!(%peer_id, "control connection back; re-punching the game path");
                            self.begin_punch(peer_id);
                        }
                    }
                    // Datagrams held for this peer were waiting on exactly
                    // this connection: start the game path they need.
                    if self.pending_datagrams.contains_key(&peer_id)
                        && !self.punch_deadline.contains_key(&peer_id)
                        && !self.tunneled_peers.contains(&peer_id)
                        && !self.game_plane_ready(&peer_id)
                    {
                        self.begin_punch(peer_id);
                    }
                    // Peers we didn't ask for (kad crawl, autonat dial-backs)
                    // stay unannounced until they speak a rexnet protocol.
                    if wanted || self.friend_store.is_friend(&peer_id) {
                        self.announce_peer(peer_id);
                    }
                    if self.friend_store.is_friend(&peer_id) {
                        self.push_presence(Some(peer_id));
                    }
                    // A shard member just became connected: get its game plane
                    // up now rather than when the player tries to join.
                    if self.shard_member_peers.contains(&peer_id) {
                        self.shard_warm_game_plane(peer_id);
                    }
                    // Deliver any invite that was waiting on this connection,
                    // unless the user has long since moved on.
                    if let Some(pending) = self.pending_invites.remove(&peer_id) {
                        if pending.queued.elapsed() <= INVITE_DEFER_MAX {
                            tracing::info!(%peer_id, "sending deferred invite");
                            self.send_invite_now(peer_id, pending);
                        } else {
                            self.invite_failed(peer_id, "peer became reachable too late");
                        }
                    }
                    // A friend message that waited for this peer goes now.
                    if let Some((msg, queued)) = self.pending_friend_msgs.remove(&peer_id) {
                        tracing::info!(
                            %peer_id, waited_s = queued.elapsed().as_secs(),
                            "sending deferred friend message"
                        );
                        self.announce_peer(peer_id);
                        let id = self.swarm.behaviour_mut().friend.send_request(&peer_id, msg.clone());
                        self.inflight_friend.insert(id, (peer_id, msg));
                    }
                }
            }
            SwarmEvent::ConnectionClosed { peer_id, endpoint, num_established, cause, .. } => {
                if num_established == 0 {
                    // Only worth a line if we ever cared about this peer.
                    if self.rexnet_peers.contains(&peer_id) || self.announced.contains(&peer_id) {
                        tracing::info!(%peer_id, ?cause, "rexnet peer disconnected");
                    } else {
                        tracing::debug!(%peer_id, ?cause, "disconnected");
                    }
                    // A GAME PEER GETS A GRACE PERIOD. Its transport is torn down
                    // below as before -- a reconnect negotiates fresh keys -- but
                    // the title is not told it left until recovery has had its
                    // chance: cached routes are dialled, a QUIC path that died is
                    // tried last, and one lookup runs if no route is known.
                    let game_peer = self.announced.contains(&peer_id)
                        && (self.recovering.contains_key(&peer_id) || self.recovering.len() < ROUTE_CACHE_PEERS)
                        && (self.tunneled_peers.contains(&peer_id)
                            || self.game_plane_ready(&peer_id)
                            || self.punch_deadline.contains_key(&peer_id)
                            || self.recovering.contains_key(&peer_id));
                    if game_peer {
                        self.remember_peer_routes(peer_id, reconnect_address(&endpoint));
                        let now = Instant::now();
                        if direct_quic(endpoint.get_remote_address()) {
                            if let Some(routes) = self.peer_routes.get_mut(&peer_id) {
                                routes.avoid_quic_until = Some(now + QUIC_AVOID);
                            }
                        }
                        if !self.recovering.contains_key(&peer_id) {
                            let tunneled = self.tunneled_peers.contains(&peer_id);
                            self.recovering.insert(
                                peer_id,
                                PeerRecovery { started: now, next_dial: now, looked_up: false, tunneled },
                            );
                            tracing::info!(%peer_id, tunneled, "game peer transport lost; attempting bounded recovery");
                        }
                    }
                    self.connected.remove(&peer_id);
                    // Virtual IP stays allocated (stable across reconnects);
                    // the punched endpoint is dropped as stale.
                    self.plane.unmap_endpoint(&peer_id);
                    // A relay that goes away stops being one; leaving it
                    // labelled would strand a stale "relay" row in the UI.
                    self.rexnet_peers.remove(&peer_id);
                    self.pending_datagrams.remove(&peer_id);
                    // Guest TCP sockets to a peer that went away must be
                    // closed, or the game waits on a socket that will never
                    // produce anything again.
                    for id in self.streams.streams_for_peer(&peer_id) {
                        self.streams.remove(id);
                        self.stream_writers.remove(&id);
                        self.emit(Event::StreamClosed { stream_id: id });
                    }
                    self.punch_deadline.remove(&peer_id);
                    // Nonces are only ever added, so this is where they leave.
                    self.pending_punch_nonce.retain(|_, p| *p != peer_id);
                    self.tunneled_peers.remove(&peer_id);
                    self.close_tunnel(&peer_id);
                    // A reconnect negotiates fresh keys, so counters and the
                    // replay window are never reused.
                    self.peer_rtt_ms.remove(&peer_id);
                    self.key_agreements.remove(&peer_id);
                    self.peer_game_pubs.remove(&peer_id);
                    self.plane.remove_keys(&peer_id);
                    // Allow a retry if they come back on the topic.
                    self.shard_reachable_attempted.remove(&peer_id);
                    if self.relays.remove(&peer_id) {
                        self.emit(Event::RelayStatus { peer: peer_id, is_relay: false });
                    }
                    // Free the auto-relay slot so another reachable peer can take
                    // its place next time one identifies.
                    self.auto_relays.remove(&peer_id);
                    if !self.recovering.contains_key(&peer_id) {
                        self.report_peer_departure(peer_id);
                    }
                }
            }
            SwarmEvent::OutgoingConnectionError { peer_id, error, .. } => {
                let wanted = peer_id.map_or(true, |p| self.wanted_peers.remove(&p));
                // A failed dial only matters if we asked for that peer. The
                // kad crawl fails against unreachable public nodes constantly
                // and each one was printing a multi-line transport error.
                if wanted {
                    tracing::warn!(?peer_id, %error, "outgoing connection failed");
                } else {
                    tracing::debug!(?peer_id, %error, "dial failed");
                }
                if wanted {
                    self.emit(Event::Error { message: format!("connect {peer_id:?}: {error}") });
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Identify(identify::Event::Received {
                peer_id,
                info,
                connection_id,
            })) => {
                // A peer that speaks our shard protocol is another RexNet
                // node. Worth knowing separately from `connected`, which is
                // mostly public DHT peers we will never ask anything.
                if info
                    .protocols
                    .iter()
                    .any(|p| p.as_ref() == crate::protocol::SHARD)
                {
                    self.remember_peer_routes(peer_id, info.listen_addrs.iter().cloned());
                    if self.rexnet_peers.insert(peer_id) {
                        tracing::info!(%peer_id, "rexnet peer identified");
                        // Late arrivals matter: if we already hold a shard and
                        // this peer holds an older one, we want to find out now
                        // rather than at the next sweep.
                        if self.shard_enabled {
                            self.shard_query_peer(peer_id);
                        }
                    }
                }
                // Torrent-model relaying: if this peer offers the circuit-v2
                // hop on a routable address, hold a reservation on it so a NAT'd
                // node gets a relay from the mesh itself, no configured server.
                self.maybe_auto_reserve_relay(peer_id, &info.protocols, &info.listen_addrs);
                // After the relay pick, so a relay it just chose is kept. A public
                // DHT peer that dialled us and speaks nothing of ours is let go:
                // the crawl otherwise fills the inbound budget and a friend
                // cannot get in.
                if !info.protocols.iter().any(|p| p.as_ref() == crate::protocol::SHARD) {
                    self.close_public_inbound(peer_id, connection_id);
                }
                // Feed identify-learned addresses into the routing table so
                // kad queries and dials can use them.
                for addr in info.listen_addrs {
                    self.swarm.behaviour_mut().kad.add_address(&peer_id, addr);
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Mdns(mdns::Event::Discovered(peers))) => {
                for (peer, addr) in peers {
                    tracing::info!(%peer, %addr, "mdns discovered");
                    self.swarm.behaviour_mut().kad.add_address(&peer, addr);
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Kad(kad::Event::OutboundQueryProgressed {
                id,
                result,
                step,
                stats,
            })) => self.handle_kad_result(id, result, step.last, stats.num_successes()),
            SwarmEvent::Behaviour(BehaviourEvent::Echo(request_response::Event::Message {
                peer,
                message,
                ..
            })) => match message {
                request_response::Message::Request { request, channel, .. } => {
                    tracing::debug!(%peer, "echo request");
                    self.announce_peer(peer);
                    let _ = self.swarm.behaviour_mut().echo.send_response(channel, request);
                }
                request_response::Message::Response { request_id, .. } => {
                    if let Some((start, reply)) = self.pending_echo.remove(&request_id) {
                        let _ = reply.send(Ok(start.elapsed()));
                    }
                }
            },
            SwarmEvent::Behaviour(BehaviourEvent::Echo(
                request_response::Event::OutboundFailure { request_id, error, .. },
            )) => {
                if let Some((_, reply)) = self.pending_echo.remove(&request_id) {
                    let _ = reply.send(Err(error.to_string()));
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Session(
                request_response::Event::OutboundFailure { peer, error, .. },
            )) => self.note_outbound_failure(peer, "session", &error),
            SwarmEvent::Behaviour(BehaviourEvent::Invite(
                request_response::Event::OutboundFailure { peer, request_id, error, .. },
            )) => {
                self.note_outbound_failure(peer, "invite", &error);
                let Some((_, pending)) = self.inflight_invites.remove(&request_id) else { return };
                let unsupported =
                    matches!(error, request_response::OutboundFailure::UnsupportedProtocols);
                if pending.attempts < INVITE_ATTEMPTS && !unsupported {
                    // Once more, on the next connection; the second failure
                    // is reported, not retried into silence.
                    tracing::info!(%peer, %error, attempt = pending.attempts, "invite not delivered; queued once more");
                    self.pending_invites.insert(peer, pending);
                    self.seek_peer(peer);
                } else {
                    self.invite_failed(peer, &error.to_string());
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Session(request_response::Event::Message {
                peer,
                message,
                ..
            })) => match message {
                request_response::Message::Request { request, channel, .. } => {
                    // Serve our descriptor to queriers. A private session is
                    // served only to a peer we invited or one presenting the
                    // join token (they never touch the DHT either). The right
                    // token admits the peer for good: its later queries and
                    // its join need not carry it.
                    let token_ok = request.token != 0
                        && self.local_session.as_ref().is_some_and(|s| s.join_token == request.token);
                    if token_ok && self.invited.insert(peer) {
                        tracing::info!(%peer, "session query carried the join token; peer admitted");
                    }
                    let invited = self.invited.contains(&peer);
                    let desc = self
                        .local_session
                        .as_ref()
                        .filter(|s| s.is_public || invited)
                        .map(|s| SessionDescMsg {
                            schema: 1,
                            session_id: s.id,
                            title_id: self.title_id,
                            slots_total: s.slots_total,
                            slots_open: s.slots_open,
                            requires_invite: !s.is_public,
                        });
                    let _ = self.swarm.behaviour_mut().session.send_response(channel, desc);
                }
                request_response::Message::Response { response, .. } => {
                    if let Some(desc) = response {
                        if desc.schema == 1 && desc.title_id == self.title_id {
                            // Surface the host before SessionFound so the
                            // shim's vip table has it when it builds the
                            // search result / join material.
                            self.announce_peer(peer);
                            self.emit(Event::SessionFound {
                                host: peer,
                                session_id: desc.session_id,
                                slots_total: desc.slots_total,
                                slots_open: desc.slots_open,
                                requires_invite: desc.requires_invite,
                            });
                        }
                    }
                }
            },
            SwarmEvent::Behaviour(BehaviourEvent::Presence(request_response::Event::Message {
                peer,
                message,
                ..
            })) => {
                if let request_response::Message::Request { request, channel, .. } = message {
                    let _ =
                        self.swarm.behaviour_mut().presence.send_response(channel, Ack { schema: 1 });
                    // Protocol-enforced: presence is dropped unless the
                    // sender is a mutual friend (§8.1).
                    if request.schema == 1 && self.friend_store.is_friend(&peer) {
                        self.announce_peer(peer);
                        self.emit(Event::PresenceUpdated {
                            peer,
                            title_id: request.title_id,
                            state: request.state,
                            display_name: request.display_name,
                            rich: request.rich,
                        });
                    }
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Friend(
                request_response::Event::OutboundFailure { peer, request_id, error, .. },
            )) => {
                // The message is not lost. It goes back in the queue and
                // leaves on the next connection, or on this one at the next
                // sweep. Quietly: delivery is reported when it happens
                // (FriendDelivered), and a peer that is offline is not an
                // error to show the player on every retry. The one exception
                // is a peer that cannot take the message at all.
                if let Some((_, msg)) = self.inflight_friend.remove(&request_id) {
                    self.note_outbound_failure(peer, "friend", &error);
                    if matches!(error, request_response::OutboundFailure::UnsupportedProtocols) {
                        tracing::debug!(%peer, "friend message dropped: peer cannot take it");
                        return;
                    }
                    tracing::debug!(%peer, %error, "friend message failed; queued for retry");
                    self.pending_friend_msgs.entry(peer).or_insert((msg, Instant::now()));
                    self.friend_seek(peer);
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Friend(request_response::Event::Message {
                peer,
                message,
                ..
            })) => {
                if let request_response::Message::Response { request_id, .. } = &message {
                    if let Some((_, msg)) = self.inflight_friend.remove(request_id) {
                        tracing::info!(%peer, ?msg, "friend message delivered");
                        if matches!(msg, FriendMsg::Request { .. }) {
                            self.emit(Event::FriendDelivered { peer });
                        }
                    }
                }
                if let request_response::Message::Request { request, channel, .. } = message {
                    let _ =
                        self.swarm.behaviour_mut().friend.send_response(channel, Ack { schema: 1 });
                    match request {
                        FriendMsg::Request { display_name, note, .. } => {
                            if self.friend_store.is_friend(&peer) {
                                // Already friends; re-confirm.
                                self.announce_peer(peer);
                                self.swarm
                                    .behaviour_mut()
                                    .friend
                                    .send_request(&peer, FriendMsg::Accept { schema: 1 });
                            } else if self.friend_store.pending_out.contains(&peer) {
                                // Both sides asked: that's mutual consent.
                                self.friend_store.add(peer);
                                self.announce_peer(peer);
                                self.swarm
                                    .behaviour_mut()
                                    .friend
                                    .send_request(&peer, FriendMsg::Accept { schema: 1 });
                                self.emit(Event::FriendAccepted { peer });
                            } else if self.friend_store.request_received(peer, &display_name, &note) {
                                // Shown as stored: a stranger's text, sanitised
                                // and capped by the store. A repeat of a request
                                // already shown stays shown once and rewrites
                                // nothing; the sender re-sends after every
                                // restart (see run()).
                                let (display_name, note) =
                                    self.friend_store.inbound_text(&peer).unwrap_or_default();
                                self.emit(Event::FriendRequest { peer, display_name, note });
                            }
                        }
                        FriendMsg::Accept { .. } => {
                            if self.friend_store.pending_out.contains(&peer) {
                                self.friend_store.add(peer);
                                self.announce_peer(peer);
                                self.emit(Event::FriendAccepted { peer });
                                self.push_presence(Some(peer));
                            }
                        }
                        FriendMsg::Decline { .. } => {
                            if self.friend_store.request_declined(&peer) {
                                self.pending_friend_msgs.remove(&peer);
                                tracing::info!(%peer, "friend request declined");
                                self.emit(Event::FriendDeclined { peer });
                            }
                        }
                        FriendMsg::Remove { .. } => {
                            self.friend_store.remove(&peer);
                            self.emit(Event::FriendRemoved { peer });
                        }
                    }
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Invite(request_response::Event::Message {
                peer,
                message,
                ..
            })) => {
                match message {
                    request_response::Message::Response { request_id, .. } => {
                        if let Some((_, pending)) = self.inflight_invites.remove(&request_id) {
                            tracing::debug!(%peer, msg = ?pending.msg, "invite message delivered");
                        }
                    }
                    request_response::Message::Request { request, channel, .. } => {
                        let _ = self
                            .swarm
                            .behaviour_mut()
                            .invite
                            .send_response(channel, Ack { schema: 1 });
                        match request {
                            InviteMsg::Offer { title_id, session_id, expires_unix, .. } => {
                                let now = std::time::SystemTime::now()
                                    .duration_since(std::time::UNIX_EPOCH)
                                    .map(|d| d.as_secs())
                                    .unwrap_or(0);
                                if expires_unix != 0 && now > expires_unix {
                                    tracing::debug!(%peer, "expired invite dropped");
                                } else {
                                    self.announce_peer(peer);
                                    self.emit(Event::InviteReceived { peer, title_id, session_id });
                                }
                            }
                            InviteMsg::Reply { accept, .. } => {
                                if accept {
                                    // They are in our session now: pinned
                                    // like any session peer.
                                    self.session_peers.insert(peer, Instant::now());
                                }
                                self.emit(Event::InviteReplied { peer, reply: accept as u8 });
                            }
                        }
                    }
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Punch(request_response::Event::Message {
                peer,
                message,
                ..
            })) => match message {
                request_response::Message::Request { request, channel, .. } => {
                    // Responder half of §8.4: answer with our candidates
                    // under the offer's nonce, then probe toward theirs.
                    tracing::debug!(
                        %peer,
                        theirs = ?request.candidates,
                        "punch offer received"
                    );
                    self.announce_peer(peer);
                    let candidates = self.local_candidates();
                    let nonce = request.nonce;
                    self.pending_punch_nonce.insert(nonce, peer);
                    let eph_pub = self.local_game_pubkey(peer);
                    self.establish_game_keys(peer, request.eph_pub);
                    let _ = self
                        .swarm
                        .behaviour_mut()
                        .punch
                        .send_response(channel, PunchAnswer { nonce, candidates, eph_pub });
                    self.spawn_probes(nonce, request.candidates);
                }
                request_response::Message::Response { request_id, response } => {
                    if self.pending_punch_offers.remove(&request_id).is_some() {
                        self.establish_game_keys(peer, response.eph_pub);
                        self.spawn_probes(response.nonce, response.candidates);
                    }
                }
            },
            SwarmEvent::Behaviour(BehaviourEvent::Punch(
                request_response::Event::OutboundFailure { request_id, error, .. },
            )) => {
                if let Some(peer) = self.pending_punch_offers.remove(&request_id) {
                    self.emit(Event::PunchResult { peer, ok: false });
                    if let Some(reply) = self.punch_replies.remove(&peer) {
                        let _ = reply.send(Err(error.to_string()));
                    }
                    // The offer itself never landed, so no probes are coming.
                    // No reason to sit out the full timeout. A peer we never
                    // connected to (DialFailure) has no tunnel to fall back
                    // to either; tunnel_fallback declines it.
                    self.punch_deadline.remove(&peer);
                    self.tunnel_fallback(peer);
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Shard(request_response::Event::Message {
                peer,
                message,
                ..
            })) => match message {
                request_response::Message::Request { request, channel, .. } => {
                    // Answer with our shard only if it is for the title being
                    // asked about; a peer may run a different game.
                    let desc = self
                        .current_shard
                        .as_ref()
                        .filter(|d| d.title_id == request.title_id)
                        .map(|d| {
                            let mut d = d.clone();
                            // Publish the live count, not the stale creation
                            // value -- this is what other peers place against.
                            d.members_hint = self.shard_member_count();
                            d
                        });
                    let _ = self.swarm.behaviour_mut().shard.send_response(channel, desc);
                }
                request_response::Message::Response { request_id, response, .. } => {
                    self.pending_shard_queries.remove(&request_id);
                    if let Some(desc) = response {
                        if desc.title_id == self.title_id && desc.schema == 1 {
                            tracing::debug!(
                                %peer, shard = %hex16(&desc.shard_id),
                                members = desc.members_hint, "shard descriptor received"
                            );
                            let arrived = desc.shard_id;
                            self.shard_candidates.insert(desc.shard_id, desc.clone());
                            // A peer that answers with OUR shard is a member of it now,
                            // beat or no beat: its address on our /24 must agree with
                            // the one it derives for itself, and waiting up to a
                            // heartbeat for the gossip left the two sides disagreeing.
                            if self.current_shard.as_ref().is_some_and(|c| c.shard_id == arrived) {
                                self.shard_member_arrived(peer);
                            }
                            if self.shard_preferred == Some(arrived)
                                && !self.current_shard.as_ref().is_some_and(|c| c.shard_id == arrived)
                                && !self.game_in_progress()
                            {
                                tracing::info!(shard = %hex16(&arrived), "named shard found; joining it");
                                self.shard_leave();
                                self.shard_enter(desc);
                                self.last_shard_move = Some(Instant::now());
                                return;
                            }
                        }
                    }
                    // Act the moment the picture is complete. Waiting for the
                    // next rescan would leave a joining player shardless for
                    // up to a full interval for no reason.
                    if self.pending_shard_queries.is_empty() {
                        self.shard_settle();
                    }
                }
            },
            SwarmEvent::Behaviour(BehaviourEvent::Shard(
                request_response::Event::OutboundFailure { request_id, .. },
            )) => {
                // An unreachable advertiser must not stall placement -- its
                // record may be stale long after it left.
                if let Some(peer) = self.pending_shard_queries.remove(&request_id) {
                    tracing::warn!(%peer, "shard query failed");
                }
                if self.pending_shard_queries.is_empty() {
                    self.shard_settle();
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Gossipsub(gossipsub::Event::Message {
                message,
                ..
            })) => {
                // Trust the gossipsub source over anything in the payload:
                // ShardBeat carries no identity of its own, precisely so a
                // peer cannot claim to be someone else.
                let Some(source) = message.source else { return };
                match postcard::from_bytes::<ShardBeat>(&message.data) {
                    Ok(beat) if beat.schema == 1 => {
                        // A peer we already knew off-shard belongs on our
                        // /24 now; shard_member_arrived decides whether that
                        // can be applied yet.
                        self.shard_member_arrived(source);
                        self.shard_ensure_reachable(source);
                        self.shard_warm_game_plane(source);
                        self.emit(Event::ShardPresence {
                            peer: source,
                            state: beat.state,
                            rich: beat.rich,
                            session_id: beat.session_id,
                        });
                    }
                    Ok(_) => tracing::debug!(%source, "shard beat: unknown schema"),
                    Err(err) => tracing::debug!(%source, %err, "shard beat: malformed"),
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::Gossipsub(gossipsub::Event::Unsubscribed {
                peer_id,
                topic,
            })) => {
                // Leaving our topic is leaving the shard; its host is freed.
                let ours = self.current_shard.as_ref().is_some_and(|current| {
                    gossipsub::IdentTopic::new(crate::protocol::shard_topic(&current.shard_id)).hash()
                        == topic
                });
                if ours {
                    self.shard_member_left(peer_id, "unsubscribed");
                }
            }
            SwarmEvent::Behaviour(BehaviourEvent::RelayClient(
                relay::client::Event::ReservationReqAccepted { relay_peer_id, .. },
            )) => self.mark_relay(relay_peer_id, true),
            SwarmEvent::Behaviour(BehaviourEvent::RelayClient(
                relay::client::Event::OutboundCircuitEstablished { relay_peer_id, .. },
            )) => self.mark_relay(relay_peer_id, true),
            // We are serving as someone's relay -- the "seed" half of the mesh.
            // Per-circuit detail at debug; the role change itself is what
            // `mark_relay` says at info.
            SwarmEvent::Behaviour(BehaviourEvent::RelayServer(event)) => {
                tracing::debug!(?event, "relay server (we are relaying for the mesh)");
            }
            SwarmEvent::Behaviour(BehaviourEvent::Dcutr(event)) => {
                tracing::debug!(?event, "dcutr");
            }
            SwarmEvent::Behaviour(BehaviourEvent::Autonat(event)) => {
                tracing::debug!(?event, "autonat");
            }
            SwarmEvent::Behaviour(BehaviourEvent::Upnp(event)) => match event {
                upnp::Event::NewExternalAddr(addr) => {
                    // The router mapped our port: this is a directly dialable
                    // address. The swarm records it as an external address on
                    // its own; we log and surface it so the player can hand the
                    // bare host:port to a peer.
                    tracing::info!(%addr, "upnp: external address mapped");
                    self.note_external_addr(addr);
                }
                upnp::Event::ExpiredExternalAddr(addr) => {
                    tracing::info!(%addr, "upnp: external address expired");
                }
                upnp::Event::GatewayNotFound => {
                    tracing::info!("upnp: no IGD gateway found (router has UPnP off or absent)");
                }
                upnp::Event::NonRoutableGateway => {
                    tracing::info!("upnp: gateway is not internet-routable (CGNAT/double NAT)");
                }
            },
            other => tracing::trace!(?other, "swarm event"),
        }
    }

    fn handle_kad_result(
        &mut self,
        id: kad::QueryId,
        result: kad::QueryResult,
        last: bool,
        successes: u32,
    ) {
        match result {
            kad::QueryResult::GetClosestPeers(res) => {
                let Some((target, _)) = self.pending_find.get(&id) else { return };
                let target = *target;
                let found = match &res {
                    Ok(ok) => ok.peers.iter().find(|p| p.peer_id == target).cloned(),
                    Err(kad::GetClosestPeersError::Timeout { peers, .. }) => {
                        peers.iter().find(|p| p.peer_id == target).cloned()
                    }
                };
                if let Some(info) = found {
                    if let Some((_, reply)) = self.pending_find.remove(&id) {
                        let _ = reply.send(Ok(info.addrs));
                    }
                } else if last {
                    if let Some((_, reply)) = self.pending_find.remove(&id) {
                        let _ = reply.send(Err(format!("peer {target} not found in DHT")));
                    }
                }
            }
            kad::QueryResult::GetProviders(res) => {
                if let Some(&peer) = self.peer_lookups.get(&id) {
                    if let Ok(kad::GetProvidersOk::FoundProviders { providers, .. }) = &res {
                        if providers.contains(&peer) {
                            // Many DHT servers return the same provider. Once our
                            // target is found, further walking and repeated dials
                            // waste bandwidth while it is offline or behind a NAT.
                            self.peer_lookups.remove(&id);
                            if let Some(mut query) = self.swarm.behaviour_mut().kad.query_mut(&id) {
                                query.finish();
                            }
                            if !self.connected.contains(&peer) {
                                tracing::info!(%peer, "peer found in DHT; dialling once");
                                let _ = self.swarm.dial(peer);
                            }
                        }
                    }
                    if last {
                        self.peer_lookups.remove(&id);
                    }
                    return;
                }
                // One result type serves two registries -- the §10 session
                // directory and the §17.3 shard registry. Dispatch on which
                // query this belongs to; matching only one would silently
                // swallow the other's providers.
                let is_session = self
                    .pending_session_search
                    .as_ref()
                    .is_some_and(|(qid, _)| *qid == id);
                let is_shard = self
                    .pending_shard_search
                    .as_ref()
                    .is_some_and(|(qid, _)| *qid == id);
                let is_relay = self.pending_relay_search == Some(id);
                let local = *self.swarm.local_peer_id();

                if is_session {
                    let Some((search_id, mut asked)) = self.pending_session_search.take() else {
                        return;
                    };
                    if let Ok(kad::GetProvidersOk::FoundProviders { providers, .. }) = res {
                        for peer in providers {
                            if peer == local || asked.contains(&peer) {
                                continue;
                            }
                            asked.push(peer);
                            self.swarm
                                .behaviour_mut()
                                .session
                                .send_request(&peer, SessionQuery { schema: 1, token: 0 });
                        }
                    }
                    if !last {
                        self.pending_session_search = Some((search_id, asked));
                    }
                } else if is_shard {
                    let Some((search_id, mut asked)) = self.pending_shard_search.take() else {
                        return;
                    };
                    if let Ok(kad::GetProvidersOk::FoundProviders { providers, .. }) = res {
                        for peer in providers {
                            if peer == local || asked.contains(&peer) {
                                continue;
                            }
                            asked.push(peer);
                            let title_id = self.title_id;
                            let req_id = self
                                .swarm
                                .behaviour_mut()
                                .shard
                                .send_request(&peer, ShardQuery { schema: 1, title_id });
                            self.pending_shard_queries.insert(req_id, peer);
                        }
                    }
                    if !last {
                        self.pending_shard_search = Some((search_id, asked));
                    } else {
                        // Sweep finished. If nobody advertised a shard there
                        // are no descriptor queries outstanding, so this
                        // settles into creating one immediately -- the first
                        // node in should not wait a full rescan interval.
                        self.shard_swept_at = Some(Instant::now());
                        self.shard_settle();
                    }
                } else if is_relay {
                    // Relays found in the DHT (§9). Dial each one we do not
                    // already reserve through; the identify handler then holds a
                    // circuit reservation on any that offer the hop on a routable
                    // address. Dialing (not reserving here) reuses that one path.
                    if let Ok(kad::GetProvidersOk::FoundProviders { providers, .. }) = res {
                        for peer in providers {
                            if peer == local || self.auto_relays.contains(&peer) {
                                continue;
                            }
                            if self.auto_relays.len() >= MAX_AUTO_RELAYS {
                                break;
                            }
                            self.wanted_peers.insert(peer);
                            if let Err(err) = self.swarm.dial(peer) {
                                tracing::debug!(%peer, %err, "dial of discovered relay failed");
                            }
                        }
                    }
                    if last {
                        self.pending_relay_search = None;
                    }
                }
            }
            kad::QueryResult::StartProviding(res) => {
                tracing::info!(ok = res.is_ok(), stored = successes, "provider record published");
                if self.own_key_query == Some(id) {
                    self.own_key_query = None;
                    // kad reports Ok even when no DHT node took the record.
                    if res.is_ok() && successes > 0 {
                        self.own_key_at = Some(Instant::now());
                    }
                }
            }
            kad::QueryResult::Bootstrap(res) => {
                if last {
                    tracing::info!(ok = res.is_ok(), "kad bootstrap finished");
                }
            }
            _ => {}
        }
    }
}

fn own_key_due(last_stored: Option<Instant>, now: Instant) -> bool {
    last_stored.map_or(true, |at| now.saturating_duration_since(at) >= OWN_KEY_REPUBLISH)
}

/// Exponential retry per peer: true when `peer` is due, scheduling the next
/// at the doubled delay with +-RETRY_JITTER, so nodes that failed together
/// do not retry together.
fn retry_due(
    retries: &mut HashMap<PeerId, (Instant, Duration)>,
    peer: PeerId,
    now: Instant,
    first: Duration,
    max: Duration,
) -> bool {
    let delay = match retries.get(&peer) {
        Some(&(next, _)) if now < next => return false,
        Some(&(_, delay)) => (delay * 2).min(max),
        None => first,
    };
    let jitter = rand::Rng::gen_range(&mut rand::thread_rng(), -RETRY_JITTER..=RETRY_JITTER);
    let scheduled = delay.mul_f64(1.0 + jitter);
    retries.insert(peer, (now + scheduled, delay));
    true
}

/// The address to reserve an auto relay through: a global one that is not a
/// circuit. A relay that holds a reservation itself advertises that circuit,
/// a circuit inside a circuit cannot be listened on, and identify lists
/// addresses in no set order, so taking the first global one failed at random.
fn reservable_addr(listen_addrs: &[Multiaddr]) -> Option<&Multiaddr> {
    listen_addrs
        .iter()
        .find(|a| multiaddr_is_global(a) && !a.iter().any(|p| matches!(p, Protocol::P2pCircuit)))
}

/// Map the game socket on a UPnP router. libp2p-upnp maps only the swarm's
/// listeners, so a NAT'd peer's probes to our public game port were dropped
/// and the pair fell back to a relayed tunnel even though we were reachable.
async fn map_game_port(port: u16) {
    use igd_next::{aio::tokio::search_gateway, PortMappingProtocol, SearchOptions};
    let gateway = match search_gateway(SearchOptions::default()).await {
        Ok(gateway) => gateway,
        Err(error) => {
            tracing::info!(?error, "upnp: game port not mapped (no gateway)");
            return;
        }
    };
    // The address our route to the gateway leaves from is the one it can forward to.
    let local = std::net::UdpSocket::bind(("0.0.0.0", 0))
        .and_then(|socket| socket.connect(gateway.addr).and_then(|()| socket.local_addr()));
    let local_ip = match local {
        Ok(addr) => addr.ip(),
        Err(error) => {
            tracing::warn!(?error, "upnp: no route to the gateway; game port not mapped");
            return;
        }
    };
    loop {
        let internal = SocketAddr::new(local_ip, port);
        let lease = GAME_PORT_LEASE.as_secs() as u32;
        match gateway.add_port(PortMappingProtocol::UDP, port, internal, lease, "rexnet game").await {
            Ok(()) => tracing::info!(port, %internal, "upnp: game port mapped"),
            Err(error) => {
                tracing::warn!(port, ?error, "upnp: game port mapping failed");
                return;
            }
        }
        tokio::time::sleep(GAME_PORT_LEASE / 2).await;
    }
}

fn hex16(bytes: &[u8; 16]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}
