# RexNet — A Serverless Xbox Live Replacement Module for ReXGlue

**Status:** Draft / RFC
**Version:** 0.2.0 — presence tiers, guest TCP, stream-carried tunnel, module
build gate.
**Audience:** ReXGlue maintainers and recomp project developers.

> **Scope.** This is an SDK document and is deliberately title-agnostic.
> Where a concrete XDK behaviour is described it is because *some* title
> depends on it, not because any particular game is being targeted; titles
> are cited as evidence for a requirement, never as the requirement itself.
> Anything true of exactly one game — title IDs, port numbers, protocol
> quirks, session-model choices — belongs in that project's `rexnet.toml`
> (§11.1), not here.

---

## 1. Summary

RexNet is a proposed ReXGlue SDK module that reimplements the Xbox 360's Live
networking API surface (XNet / XSession / XUser / XPresence / XNotify /
XInvite) on top of a fully open-source, serverless peer-to-peer stack built on
rust-libp2p.

Because the XLive API surface is XDK-standard rather than game-specific, the
module is written once at the SDK layer. Any recomp built on ReXGlue gains
online play by supplying a small per-game configuration file — no networking
code required.

Every participating recomp joins the **same** shared DHT and relay mesh,
namespaced by title ID. A player idling in one game strengthens discovery and
connectivity for players of every other game. The network grows with the
ecosystem instead of decaying like abandoned master servers.

### Design pillars

1. **No operated infrastructure.** Discovery rides the public Amino (IPFS)
   DHT; hole-punch coordination uses public circuit-v2 relays and
   opt-in peer relays. Nobody runs a master server.
2. **Preservation floor.** Manual connect strings, LAN mDNS discovery, and
   IPv6 direct dial always work, independent of DHT or mesh health.
3. **One identity, many games.** A single ed25519 keypair acts as a
   decentralized gamertag across all recomps.
4. **Game traffic stays out of the mesh.** libp2p is the signaling/control
   plane. Game datagrams run over a directly punched UDP socket with
   unreliable-datagram semantics preserved.
5. **Versioned, published wire spec.** Independent projects sharing a live
   network interoperate deliberately, not accidentally.

### Non-goals

- Connecting to, emulating, or interoperating with Microsoft's Xbox Live
  services in any form.
- Server-authoritative or anti-cheat infrastructure. Trust model is
  "playing with friends."
- Voice chat in v1 (schema reserves room for it; see §12).

---

## 2. Layered architecture

```
+---------------------------------------------------------------+
| Recompiled game code (PPC -> native, big-endian guest memory) |
+---------------------------------------------------------------+
| L3  ReXGlue XAM module (C++)                                  |
|     XNet / XSession / XUser / XPresence / XNotify / XInvite   |
|     virtual-IP mapping, BE<->LE marshalling, notify queue     |
+---------------------------------------------------------------+
| L2  FFI boundary (C ABI, cbindgen header, POD structs,        |
|     command queue in / event queue out, catch_unwind)         |
+---------------------------------------------------------------+
| L1  rexnet-core (Rust staticlib, tokio, rust-libp2p)          |
|     identity, kad, gossipsub, identify, dcutr, relay          |
|     client+service, mdns, autonat, request-response, libp2p   |
|     streams (guest TCP + tunnel), UDP punch engine            |
+---------------------------------------------------------------+
| L0  Transports: QUIC (libp2p) for control; raw UDP socket     |
|     with a 5-byte RexNet frame header for game traffic        |
+---------------------------------------------------------------+
| L4  Per-game config (TOML): session model, channel mode,      |
|     presence mapping, quirk flags (title ID + ports are       |
|     auto-derived at runtime, §11.0)                           |
+---------------------------------------------------------------+
```

`rexnet-core` contains **no Xbox concepts** beyond an opaque namespace tag.
Non-recomp projects may adopt it, widening the mesh.

The whole stack is **optional**: RexNet is the SDK's only Rust dependency, and
it is compiled out entirely unless asked for. See §19.

---

## 3. Identity

- Each install generates an **ed25519 keypair** on first run
  (`identity.key`, mode 0600). The libp2p PeerId derived from the public key
  is the player's permanent identity.
- **Friend code** = base32(multihash(pubkey)), rendered in grouped form for
  human exchange, e.g. `REXN-K51Q-ZP0M-3F7A-...`. Checksummed; case-
  insensitive on input.
- **Display name** is a self-asserted UTF-8 string (max 32 bytes) carried in
  presence records. Names are not unique; the friend code is the identity.
- Friendship is **mutual-consent**: a friend request/accept exchange
  (§9.3) is required before presence flows in either direction.
- Key rotation is out of scope for v1; losing the key means a new identity.
  Document export/import of `identity.key` for users.

---

## 4. Discovery

Ordered by preference; all paths coexist.

1. **LAN mDNS** (`_rexnet._udp.local`) — zero-config same-network discovery.
2. **Kademlia peer routing** on the public Amino DHT — resolving a friend's
   PeerId to current multiaddrs (`FindPeer`). No provider records needed for
   friend connectivity.
3. **DHT provider records for public sessions** (§10) — namespaced per title.
4. **Manual multiaddr connect string** — the preservation floor. A join code
   pasted out-of-band (Discord etc.) encodes PeerId + endpoints:
   `/ip4/203.0.113.7/udp/4242/quic-v1/p2p/12D3KooW...`
5. **IPv6 direct dial** whenever both peers have global v6 — no traversal
   needed.

Bootstrap peers default to the standard Amino bootstrap set and MUST be
user-editable configuration. The module MUST function (paths 1, 4, 5) with
an empty bootstrap list.

---

## 5. NAT traversal

- **AutoNAT** determines reachability class on startup.
- **Circuit-v2 relay client + DCUtR** establish a control connection to
  unreachable peers and upgrade it to a direct QUIC connection where
  possible.
- **Second-socket UDP punch** (§6) for the game-traffic socket, coordinated
  over the already-established control connection (simultaneous-open with
  live signaling — substantially higher success than blind punching).
- **UPnP/NAT-PMP/PCP port mapping** attempted opportunistically.
- **Fallback:** if the second-socket punch fails (e.g. hostile CGNAT pairs),
  game datagrams are tunneled through the libp2p QUIC control connection —
  **one long-lived stream per peer per direction**, carrying length-prefixed
  frames (`[u16 len][u16 src_port][u16 dst_port][payload]`, big-endian, payload
  capped at 2048 to match the game socket's receive buffer). Frames are not
  acknowledged and are dropped when the writer queue (64 frames) backs up: the
  guest believes it is sending UDP, and a tunnel that silently added
  reliability would change the timing the title was written against. Playable
  for low-rate co-op, flagged to the user as degraded.

  Each direction gets its own stream. One stream used both ways would cost one
  fewer substream, but both ends degrade at once in the case this exists for,
  so both would open simultaneously and need a tie-break to decide whose
  survives; two one-way streams have no race.

  A peer that receives a tunneled frame and has no punched endpoint marks that
  peer tunneled itself. Without that inference only the side that degraded
  first would be heard — the other's replies would sit in the punch queue
  until they aged out, which is a one-way session, i.e. none.

  **Tradeoff, stated plainly:** this is *more* head-of-line blocking than the
  original `1.0.0` request-response tunnel, where each datagram had its own
  substream and QUIC ordered them independently. That version collapsed under
  game-rate traffic — one substream per datagram against a per-connection
  inbound cap, which we hit in a real session. Added latency on packet loss is
  a worse link; exhausting the substream allowance is no link at all.

Expected direct-connection success in the wild is roughly 70–80% via
DCUtR + punch; IPv6 and manual strings cover much of the remainder.

---

## 6. Transport split (control plane vs. game plane)

| Plane   | Carrier                                  | Semantics                          |
|---------|------------------------------------------|------------------------------------|
| Control | libp2p QUIC (direct or relayed)          | Reliable, ordered, encrypted       |
| Game    | Dedicated punched UDP socket             | Unreliable datagrams (VDP-like)    |

Rationale: public relays only permit brief low-bandwidth reservations, and
libp2p streams are reliable/ordered — wrong for 360 titles' VDP-style
traffic. The control connection exchanges observed external endpoints and
coordinates the punch; the game socket then carries all XNet datagram
traffic. ENet or GameNetworkingSockets framing runs on the game socket to
provide optional reliable channels, sequencing, and (with GNS) encryption.

Many 360 titles run their own reliability protocol on top of UDP (e.g. the
XDK's XRNM library: its own acks, SACKs,
retries, and probes). For such games the framing MUST be a
pure-unreliable channel — stacking a reliable carrier underneath a
retransmitting protocol double-buffers loss recovery and adds retransmit
latency at the worst moments. The per-game config selects the channel mode
(§11.1); GNS remains useful purely as encrypted framing.

Game-plane packets are encrypted with keys derived from the Noise/TLS
session of the control connection (export keying material), so relay
operators and on-path observers cannot read or forge game traffic.

---

## 7. Wire protocol identifiers

All protocol IDs are semver'd. Capability negotiation via `identify`
`protocols` list. Breaking changes bump the major version; peers advertise
all versions they speak.

| Purpose            | Protocol ID                     |
|--------------------|---------------------------------|
| Kademlia           | `/rexnet/kad/1.0.0`             |
| Presence           | `/rexnet/presence/1.0.0`        |
| Friendship         | `/rexnet/friend/1.0.0`          |
| Invites            | `/rexnet/invite/1.0.0`          |
| Session directory  | `/rexnet/session/1.0.0`         |
| Punch signaling    | `/rexnet/punch/1.0.0`           |
| Game tunnel (fallback) | `/rexnet/tunnel/2.0.0`      |

DHT namespace keys:

```
rexnet/v1/title/<title_id_hex>/sessions        (provider records)
```

Message encoding: postcard (compact serde) with a 1-byte schema version
prefix per message. All multi-byte integers little-endian on the wire.

---

## 8. Message schemas (v1)

### 8.1 Presence record (pushed over `/rexnet/presence/1.0.0`)

```rust
struct Presence {
    schema: u8,              // = 1
    seq: u64,                // monotonic, replay protection
    display_name: String,    // <= 32 bytes
    title_id: u32,           // 0 = online, no game
    state: u8,               // 0 offline, 1 online, 2 in-game, 3 joinable
    rich: Vec<KV>,           // game-defined, <= 512 bytes total
    game_endpoints_hint: Vec<Endpoint>,  // optional, for fast joins
    sig: [u8; 64],           // ed25519 over all prior fields
}
struct KV { key: u16, value: Value }   // Value: U32 | I32 | Str(<=64B)
```

`rich` carries whatever the XAM shim translates from XUserSetContext /
XUserSetProperty; the meaning of each context id is the title's own.
Presence is pushed to mutual friends on change and heartbeated every 30 s;
peers treat a record older than 90 s as stale.

### 8.2 Friendship (`/rexnet/friend/1.0.0`)

```rust
enum FriendMsg {
    Request { schema: u8, display_name: String, note: String },  // note <= 128B
    Accept  { schema: u8 },
    Remove  { schema: u8 },
}
```

Friend list is stored locally only. Presence MUST NOT be sent to, or
accepted from, non-friends (protocol-enforced, not UI-enforced).

### 8.3 Invite (`/rexnet/invite/1.0.0`)

```rust
struct Invite {
    schema: u8,
    title_id: u32,
    session_id: [u8; 16],    // maps to XNKID
    context: Vec<KV>,
    expires_unix: u64,
}
enum InviteReply { Accept, Decline, Busy }
```

On `Accept`, the sender's module returns session join material (§10.2) and
the recipient's XAM shim injects the accepted-invite notification into the
guest XNotify queue, letting the game's existing join flow run unmodified.

### 8.4 Punch signaling (`/rexnet/punch/1.0.0`)

```rust
struct PunchOffer  { schema: u8, nonce: [u8;16], candidates: Vec<Endpoint> }
struct PunchAnswer { schema: u8, nonce: [u8;16], candidates: Vec<Endpoint> }
struct PunchResult { schema: u8, nonce: [u8;16], ok: bool, chosen: Option<Endpoint> }
struct Endpoint    { ip: IpAddr, port: u16, kind: u8 }  // 0 local, 1 srflx, 2 v6
```

Simultaneous-open: both sides transmit spaced probe datagrams (containing
the nonce, HMAC'd with the exported session key) to all candidate pairs for
up to 5 s; first authenticated probe pair wins.

---

## 9. Relay policy (opt-in mesh relays)

- Relay **service** (circuit-v2) is offered only by peers that AutoNAT
  confirms publicly reachable (or with a working port mapping / global v6).
- **Opt-in**, default **off** on metered or battery-powered devices; a
  single settings toggle elsewhere.
- Hard caps via circuit-v2 limits: default 64 concurrent reservations,
  2 min / 128 KiB per circuit — enough for signaling and punch
  coordination, deliberately unusable for sustained game traffic.
- Relayed bytes are end-to-end encrypted; operators cannot read them.
  DCUtR upgrades relayed connections to direct as fast as possible, so
  relays mostly carry brief coordination bursts.
- Relay discovery: relays register as providers under
  `rexnet/v1/relays` in addition to standard libp2p relay discovery.

---

## 10. Sessions and matchmaking

### 10.1 Session lifecycle (maps to XSession)

- `XSessionCreate` → module allocates a random 128-bit `session_id`
  (surfaced to the game as XNKID; the XNKEY security key is synthesized and
  otherwise ignored — §11).
- Public sessions (`XSESSION_CREATE_USES_MATCHMAKING`) additionally publish
  a provider record under `rexnet/v1/title/<title>/sessions`.
- `XSessionSearch` → DHT provider lookup + direct query of each advertiser
  over `/rexnet/session/1.0.0` for its session descriptor; contexts/
  properties are filtered client-side per the game's config mapping.
- Private/friends sessions never touch the DHT; joins occur via presence
  (`joinable` state + `game_endpoints_hint`) or invites.
- Many titles use only the private/invite path; the DHT session directory
  exists because the *generic* module must also support lobby-browser titles.
- **Not to be confused with the ambient title shards of §17.3**, which
  live under a separate `/shards` DHT key precisely so they never
  surface to `XSessionSearch`.

### 10.2 Session descriptor (`/rexnet/session/1.0.0` query response)

```rust
struct SessionDesc {
    schema: u8,
    session_id: [u8;16],
    title_id: u32,
    host: PeerIdBytes,
    slots_total: u8,
    slots_open: u8,
    attrs: Vec<KV>,          // from game config mapping
    requires_invite: bool,
}
```

---

## 11. XAM shim mapping (Layer 3)

| XDK surface | RexNet implementation |
|---|---|
| `XNetStartup` / `XNetCleanup` | init/teardown module handle |
| `XNetXnAddrToInAddr` / `InAddrToXnAddr` | virtual-IP table: each peer is assigned an address in `10.77.0.0/16`; XNADDR carries PeerId hash |
| `XNetConnect` | initiates connect/punch to the peer behind the virtual IP (`rexnet_connect_peer`). **Not a no-op:** XRNM-style link state machines call it themselves as part of bringing a link up |
| `XNetGetConnectStatus` | `XNET_CONNECT_STATUS_PENDING` while punching, `_CONNECTED` once the game socket (or tunnel fallback) is up, `_LOST` on drop. Load-bearing: XRNM-style netcode gates every send on it; also gives games truthful disconnect behavior for free |
| `sendto`/`recvfrom` on virtual IPs | routed to that peer's punched game socket (or tunnel fallback) |
| `XNetQosListen` / `XNetQosRelease` | success no-ops (hosts call Listen on session create; joiners Release their lookup handle) |
| `XNetQosLookup` | synthetic-but-sane results, ideally populated with the real RTT measured by the punch engine |
| `XNetRegisterKey` / `XNetUnregisterKey` / SA functions | no-ops (transport already encrypted) |
| VDP sockets | unreliable channel on game socket — semantics preserved |
| `XSession*` | §10 lifecycle |
| `XUserGetSigninState` / `GetSigninInfo` | local profile from identity + display name |
| `XUserSetContext` / `SetProperty` | presence `rich` KVs per game config |
| `XPresence*` / friends enumeration | local friend list + live presence cache |
| `XInvite*` | §8.3; accepted invite injected via XNotify queue |
| `XNotifyGetNext` | module event queue drained into guest notification queue |
| XHV voice | stubbed in v1 |

All structures crossing into guest memory are converted to big-endian by the
shim; `rexnet-core` sees native-endian only.

### 11.0 Runtime derivation (no scanning required)

Two values the earlier draft put in config are self-deriving at startup —
no binary scanning, no per-game entry:

- **Title ID** comes from the XEX2 `XEX_HEADER_EXECUTION_INFO` optional
  header, which ReXGlue already parses at module load
  (`module->title_id()`, the same field `XamGetCurrentTitleId` returns).
  RexNet init reads it from kernel state; it is exact by construction.
- **UDP ports** are learned at the NetDll boundary, where every guest
  socket already passes: `NetDll_bind` registers a listening guest port in
  the virtual-IP router the instant the game binds it, and `NetDll_sendto`
  carries an explicit destination port in every call. Nothing needs
  pre-opening host-side — the punched game socket multiplexes all guest
  ports and punching is per-peer, not per-port, so a port learned one
  millisecond before its first packet is learned in time by construction.
  Static-scanning the XEX for port immediates would be strictly worse:
  fragile (`li r5, 1000` is indistinguishable from any other constant) and
  unnecessary.

Caveat: port learning assumes game networking flows through NetDll — true
for anything XDK-linked. The only cost of a late-learned port is losing the
opportunistic UPnP *pre*-mapping (§5), which degrades to mapping on first
bind.

What genuinely cannot be derived stays in config: `session_model`,
`game_channel` (own-reliability cannot be safely inferred from traffic),
the presence context mapping, and quirks.

### 11.1 Per-game configuration

Everything title-specific lives in the consuming project's `rexnet.toml`.
The SDK ships no per-title data and no built-in title list: a recomp project
describes its own game, and the module stays generic.

```toml
[game]
# title_id: auto-derived from the XEX execution-info header (§11.0).
# Override only for multi-region or alternate-ID packaging.
#title_id     = 0x00000000

# udp_ports: auto-learned from NetDll_bind/sendto (§11.0). An optional hint,
# used only for opportunistic UPnP pre-mapping and documentation.
#udp_ports    = [1000, 1001]

session_model = "private-invite"  # or "public-browser"
game_channel  = "unreliable"      # set for titles that run their own
                                  # reliability layer (§6); such traffic must
                                  # never be wrapped in a reliable carrier

[presence.rich]                   # XUserSetContext id -> rich KV key
# context ids and their meanings are defined by the title, not by RexNet
#region      = { context = 0x0001, key = 1, type = "str" }

[shard]                           # ambient title shard (§17.3), opt-in
enabled           = false
cap               = 255
surface_as_friends = false

[quirks]
# e.g. relaxed_session_state = true for titles that call Start twice
```

### 11.2 What a bring-up actually requires

Generalised from the titles audited in §18. The pattern has held across every
one of them, which is why it belongs here rather than in a per-title note:

- **The join handshake is the title's own code.** In every title examined,
  session join is application-level traffic between guest sockets — RexNet
  supplies discovery and datagram routing and implements none of the
  handshake. A shim that tried to emulate the handshake would be writing the
  game's netcode for it.
- **The only data the shim must inject** is the host's XNADDR / port / session
  key into whatever structure the title reads its results from. Everything
  downstream is guest code.
- **Some titles rewrite local-address targets to `127.0.0.1`** in their own
  send path. Observed in one title so far, but it makes same-machine
  two-instance testing a first-class topology worth supporting regardless:
  under virtual IPs, guest port collisions do not exist host-side, which is
  what makes that topology work at all.
- **Voice may be carried in-band** by the title's own transport rather than
  through XHV — again seen in one title. Where that holds, a voice stub costs
  capture/encode rather than transport. Do not assume it without checking.
- **The blocker is usually the wake-up path, not the data path** — see §18.
  This one *is* general: it held in three of the five titles audited.

---

## 12. FFI surface (Layer 2)

Non-blocking command-in / poll-event-out. The library owns its tokio
runtime; the game thread never blocks. Header generated with cbindgen;
every export wrapped in `catch_unwind`.

```c
typedef struct RexNetHandle RexNetHandle;

RexNetHandle* rexnet_init(const RexNetConfig* cfg);      // cfg: paths, title_id (shim-derived from XEX, §11.0), bootstrap list
void          rexnet_shutdown(RexNetHandle*);

/* commands (enqueue, return immediately) */
void rexnet_set_presence(RexNetHandle*, const RexNetPresence*);
void rexnet_connect_peer(RexNetHandle*, const RexNetPeerId*);
void rexnet_connect_manual(RexNetHandle*, const char* multiaddr);
void rexnet_send_invite(RexNetHandle*, const RexNetPeerId*, const RexNetInvite*);
void rexnet_friend_request(RexNetHandle*, const RexNetPeerId*, const char* note);
void rexnet_session_create(RexNetHandle*, const RexNetSessionCfg*);
void rexnet_session_search(RexNetHandle*, const RexNetSearchFilter*);
void rexnet_send_datagram(RexNetHandle*, uint32_t virtual_ip,
                          uint16_t port, const uint8_t* data, uint32_t len,
                          bool reliable);
/* `reliable` exists for module-internal traffic; guest-originated datagrams
   MUST pass false when the game config sets game_channel = "unreliable"
   (games running their own reliability, e.g. XRNM — see §6). */

/* guest TCP (§18.1) — a stream socket whose peer is a virtual IP */
void rexnet_stream_connect(RexNetHandle*, uint32_t virtual_ip,
                           uint16_t src_port, uint16_t dst_port);
void rexnet_stream_send(RexNetHandle*, uint64_t stream_id,
                        const uint8_t* data, uint32_t len);
void rexnet_stream_close(RexNetHandle*, uint64_t stream_id);

/* events (drain once per frame) */
bool rexnet_poll_event(RexNetHandle*, RexNetEvent* out);  // fixed-size POD union
```

`RexNetEvent` kinds: `PeerConnected`, `PeerDisconnected`, `PresenceUpdated`,
`FriendRequest`, `FriendAccepted`, `FriendRemoved`, `InviteReceived`,
`InviteReplied`, `PunchResult`, `SessionFound`, `Datagram`,
`Degraded(tunnel)`, `Error`, plus:

- `StreamOpened` / `StreamData` / `StreamClosed` / `StreamConnectFailed` —
  guest TCP (§18.1).
- `LocalAddress` — our own virtual IP changed because we joined, left or
  migrated a shard (§17.3.5). The shim reports this as the local XNADDR.
- `RelayStatus` — a peer started or stopped carrying our traffic (§17.2).
  Drives the `relay` label, the one case where a non-friend is named at all,
  and it is named by role rather than identity.
- `ShardPresence` — ambient presence from the title shard (§17.3.4).
  Deliberately carries no display name: the shim renders a pseudonym unless
  the peer is a friend.

Stream ids are allocated locally and never sent, so the two ends number the
same connection differently. Zero is reserved for "no stream", because a
zeroed event field is otherwise indistinguishable from a real id.

Voice: reserved event kinds `VoiceFrameIn/Out` for a future Opus channel on
the game socket; absent in v1.

---

## 13. Security & privacy

- All control traffic: libp2p Noise/TLS. All game traffic: keys exported
  from the control session (§6). No plaintext paths.
- Presence only between mutual friends; enforced at the protocol layer.
- Public session records expose: title ID, host PeerId, slot counts, and
  the game-config attribute set — nothing else. Private sessions expose
  nothing to the DHT.
- Replay protection: monotonic `seq` on presence; nonced, HMAC'd punch
  probes; invite expiry.
- Threat model is friend-scale griefing resistance, not anti-cheat.
  Blocklist (drop all traffic from PeerId) is local and immediate.
- No telemetry. Optional, off-by-default anonymous punch-success metric
  may be proposed separately if the community wants tuning data.

---

## 14. Fallback matrix (preservation guarantees)

| Condition | What still works |
|---|---|
| DHT bootstrap peers gone | manual connect strings, LAN mDNS, IPv6 direct |
| All public relays gone | direct dial, port-mapped/UPnP hosts, manual strings |
| Punch fails (CGNAT pair) | tunneled game traffic over control connection (degraded) |
| Total internet-scale infrastructure loss | LAN play via mDNS, unchanged |

---

## 15. Milestones

Status is recorded here deliberately. "Implemented" and "observed working"
are different claims, and a spec that blurs them stops being useful as a
record of what is actually true.

1. ✅ **P2P core proof (pure Rust CLI):** identity, FindPeer across real NATs,
   DCUtR, echo over a stream. De-risks everything not under our control.
2. ✅ **FFI + event queue** integrated into the ReXGlue runtime.
3. ✅ **Second-socket punch** carrying live traffic; tunnel fallback (§5).
   *GNS is not in this path — see §16.*
4. ✅ **XAM shim generic module:** XNet virtual IPs, XSession private path,
   XNotify plumbing.
5. 🟡 **First title bring-up:** config file, invites and two-instance co-op
   working end to end on one title. Passive presence is **not** yet driving a
   title's own in-game presence display; that path runs over the title's own
   protocol and needs gates the shim does not yet satisfy. Diagnosis of any
   specific title belongs in that project, not here.
6. 🟡 **Presence tiers (§17):** Linked and Connected implemented, shard
   convergence tested node-to-node. Untuned at scale (§17.6).
7. 🟡 **Guest TCP (§18.1):** Rust half tested node-to-node; the C++ half —
   `XSocket` connect/accept/recv/send — is compile-checked only and has
   never executed. No System Link title has been recompiled to exercise it.
8. ⬜ **Session directory + search** for lobby-browser titles; relay service
   opt-in; spec v1.0 published as its own repo.

**Never exercised at all:** LAN across separate machines, real NAT/CGNAT
between different networks, and a `force_tunnel` session carrying an actual
game. Every result so far is loopback or same-host.

## 16. Open questions

- Amino DHT as default vs. a rexnet-flavored Kademlia overlay bootstrapped
  from Amino (server-mode participation etiquette on the public DHT).
- rust-libp2p relay-service resource tuning for consumer uplinks.
- Whether `game_endpoints_hint` in presence leaks too much (endpoints to
  friends only — acceptable?).
- Governance: where the wire-spec repo lives and who signs off on protocol
  major versions (proposal: ReXGlue org, two-maintainer rule).
- GNS vs. ENet as the blessed game-socket framing (GNS: encryption built
  in; ENet: smaller). Current lean: GNS — with the caveat that for
  own-reliability titles (§6) only its unreliable channel and encryption are
  used, so ENet's reliability features carry no weight in the comparison.
  **Neither is in the path today:** the game plane uses a plain 5-byte RexNet
  header, and `REXGLUE_REXNET_GNS` defaults OFF. Co-op has been made to work
  without either, so this is now a question about game-traffic encryption
  rather than framing, and should be decided on that basis.
- **`libp2p-stream` is pinned at `0.4.0-alpha`** and carries both guest TCP
  (§18.1) and the tunnel (§5). An alpha crate under a preservation project is
  the wrong shape of dependency: the whole point is to still build in ten
  years. Options are to vendor it, to wait for a stable release, or to
  re-implement the two narrow uses directly on a `NetworkBehaviour`.
- Whether the tunnel should reduce head-of-line blocking by striping across a
  small fixed pool of streams rather than one (§5). One stream is correct for
  substream budget and wrong for loss recovery; nobody has measured which
  matters more on a real CGNAT link, because no such link has been tested.

---

## 17. Presence tiers (Linked / Connected / Active)

Three ambient tiers sit under the game. Only the middle one is new; the outer
two are existing machinery given an explicit name and display rule.

This section **adds to** §9 (relay) and §10 (sessions); it does not change
them. In particular the shard registry below is a *separate* DHT namespace
from the XSession directory — see §17.3.1.

### 17.1 The tiers

| Tier | Scope | Membership | Backed by |
|---|---|---|---|
| **Linked** | Any title, any peer | Implicit (you are on the mesh) | §3 identity, §4 discovery, §9 relay |
| **Connected** | One title, <= 255 peers | Automatic placement into a *shard* | §17.3 (new) |
| **Active** | One game session | Game-driven (or manual invite) | §6 game plane, §10 XSession |

### 17.2 Linked

Mesh membership, title-agnostic. Already provided by identity, kad/mDNS
discovery and the relay policy. The only new behaviour is a **display rule**:

- **Friends** — real display name (§3), full presence (§8.1).
- **Non-friend peers serving as your relay** — rendered as `relay`, no name.
- **All other peers** — not surfaced at all.

### 17.3 Connected (title shard)

An ambient, automatically-joined group of up to 255 peers playing the same
title. It is *not* the game's session: it carries passive presence — the
ambient "other players exist" layer some titles render in-world — and is the
population that matchmaking/lobby UIs draw from. A player
stays in their shard across Active sessions.

**Opt-in per title** (§17.5). Titles that declare no use for it never join a
shard and pay no ambient traffic.

#### 17.3.1 Shard registry (distinct from §10)

Published under `rexnet/v1/title/<advertised_title_id>/shards`, deliberately
**not** the `/sessions` key used by §10. Ambient shards must never appear to
`XSessionSearch` / `XSessionSearchByID`, or a title would try to join a
255-person shard as if it were a co-op game (several titles would).

```rust
struct ShardDesc {
    schema: u8,
    shard_id: [u8;16],   // random at creation
    title_id: u32,       // advertised id (§11.1 advertised_title_id)
    created_at: u64,     // unix seconds, creator's clock (advisory)
    cap: u16,            // default 255
    members_hint: u16,   // advisory; authoritative count is live topic peers
    anchor: PeerIdBytes, // creator; may have left, never authoritative
}
```

#### 17.3.2 Placement

Friends win, then age:

1. Resolve providers for the shards key; fetch each `ShardDesc`.
2. Discard wrong-title and at-capacity shards (`members_hint` advisory only).
3. Join the shard containing the **most friends**, if any has capacity.
4. Otherwise join the **oldest** (`created_at`) shard with capacity.
5. Otherwise **create** a shard.

Ties at any step break by lower `shard_id`, so all peers agree.

#### 17.3.3 Convergence

Concurrent creation is expected: two peers that both find nothing will both
create a shard, and without a merge rule the population fragments forever.

Members re-scan periodically and migrate toward the **canonical** shard for
the title: the one sorting first by `(created_at, shard_id)` among those with
capacity. That order is total and fixed, so it has a unique minimum and
migration provably terminates there.

> **Do not add a size condition.** An earlier draft of this section also
> required "the local shard is the smaller of the two", intending to damp
> churn. It does not converge: when the older shard is the *smaller* one,
> direction (toward older) and the size guard (toward bigger) disagree,
> neither side yields, and the two populations never merge. Verified by test
> — `older_but_smaller_still_converges` in `core/src/shard.rs`. Population
> must not influence *direction*; a stampede is a question of **rate**, which
> belongs in rate limiting.

Migration is rate-limited, and **suppressed while an Active session is in
progress** — never move a player mid-co-op to tidy up shard topology.

#### 17.3.4 Membership and broadcast

A 255-peer shard is a *logical* group, never a full mesh: 255 peers fully
connected is ~32k links and would exhaust NAT state long before that.

- One gossipsub topic per shard, `rexnet/v1/shard/<shard_id>`, carrying
  low-rate ambient data (liveness, per-title rich KVs).
- **Membership implies reachability.** Gossipsub meshes with only ~6 peers
  whatever the topic size, so being in a shard would otherwise leave most
  members as bare peer ids: no address, no connection, and an invite to them
  starting from a cold DHT lookup that frequently fails. That defeats the
  purpose — a shard is meant to be the set of people you *can already reach*.
  So on hearing a member:
  - shard of <= `SHARD_WARM_CONNECT_LIMIT` (24): dial them, keeping a control
    connection to every member. Cheap at this size, and every member is
    instantly invitable.
  - larger: resolve their address into the routing table instead, so a later
    dial succeeds without a discovery round trip. A 255-member shard must not
    mean 255 connections.

  Attempts are made once per peer per session; a 30 s heartbeat must not
  re-dial an unreachable member forever.
- **The game plane is warmed too, not just the control connection.** A control
  connection does not make a peer playable: the game plane is a separate
  punched UDP path. Establishing it on demand is what makes a first invite
  race — both sides punch independently, and whichever finishes first sends
  into an endpoint the other has not mapped yet, so the opening handshake is
  lost and the title waits out its own timeout. Members of a shard within the
  warm limit therefore have their game plane punched **up front**, so
  promotion to Active finds it already established. This is the substance of
  the tier: Connected means *ready to play*, not merely *known about*.
- Beyond the warm limit, direct connections are opened when the game actually
  connects — i.e. on promotion to Active (§6 game plane).

  Receivers must also tolerate the reverse race: a frame arriving from an
  endpoint not yet mapped to a peer is **held**, not dropped, and delivered
  once the punch completes.
- Live topic peers are the real member count; `members_hint` is a stale
  advisory for peers that have not joined the topic yet.

This requires `libp2p-gossipsub`, which rexnet-core does not currently
depend on (it carries kad, request-response, relay, mDNS, identify, AutoNAT,
DCUtR).

#### 17.3.5 Subnet addressing (a shard is a /24)

The 255 cap is not arbitrary: a shard **is** a temporary /24, and its member
limit is that subnet's host count. Read strictly it is **254** (`.0` is the
network address, `.255` broadcast), so `cap = 255` in §17.5 means "a /24".

For that to be more than a metaphor, every member must resolve a given peer to
the *same* address. Otherwise `10.77.5.9` names a different player depending
on who you ask, and subnet broadcast has nothing meaningful to address.

> **This is not how virtual IPs worked before §17.** The original allocator
> handed out `10.77.0.1, .2, .3…` from a flat counter over the /16 in the
> order each node happened to *meet* peers, never exchanged them, and had
> every node call itself `10.77.255.254`. The same player was `.0.1` to one
> node and `.0.7` to another. Those were local lookup handles, not a network.

Addresses are **derived, not negotiated** — no coordinator to elect or fail
over:

- Subnet: `10.77.<h(shard_id)>.0/24`. Two shards may collide on a subnet;
  harmless, since addresses only resolve within a shard and a node is in one
  shard at a time.
- Host: each peer prefers `h(peer_id)` in `1..=254`; collisions resolve by
  linear probing with members processed in **peer-id order**, so nodes that
  learned the membership in different orders still derive identical tables.
- A member beyond a full subnet gets **no** address rather than a colliding
  one.

Caveat: an arrival can displace a later peer that had probed into the slot it
wants, so addresses are not perfectly stable as membership grows. Rare below
~50 members; a stability guarantee needs a coordinator, which deriving the
table exists to avoid.

#### 17.3.6 Broadcast and System Link

`10.77.<subnet>.255` addresses every member of the shard. This is what makes
the subnet worth having: **System Link** titles discover peers by broadcasting
rather than naming a host, so a shard that behaves like a LAN can carry them
without per-title work — including the many 360 titles that supported System
Link but never Live.

**Off by default: `rexnet_syslink`.** System Link is a *local* facility, and
redirecting it unconditionally would silently break ordinary same-house play,
where two consoles should simply talk over the real network. So:

| `rexnet_syslink` | Broadcast goes to | Effect |
|---|---|---|
| `false` (default) | the real LAN, via the host stack | System Link behaves as designed |
| `true` | the shard's subnet broadcast | a LAN-only title gains internet play |

Both spellings of a broadcast take the switch: the limited broadcast
(`255.255.255.255`, requiring `SO_BROADCAST` as real Winsock does) and the
directed one a title derives from its own address — which lands inside
`10.77/16` because that is where its address is. Unicast to a peer's virtual
address is unaffected and always belongs to RexNet.

This is the one place RexNet changes what a game *is* rather than standing in
for infrastructure it can no longer reach, so it stays an explicit choice.

#### 17.3.7 Pseudonyms

Non-friends in a shard are shown a stable, inoffensive generated name rather
than their display name.

- Derived deterministically from the peer id. The algorithm is specified
  exactly, because an independent implementation must produce byte-identical
  names or observers will disagree:

  1. `h = FNV1a64("rexnet-pseudonym-v1" || len(peer_id) || peer_id)`, where
     `len` is one byte and the id is its length-prefixed multihash bytes.
     Length is mixed in so one id cannot collide with another that is its
     prefix.
  2. Avalanche `h` with the Murmur3 `fmix64` finalizer. **Not optional:**
     FNV-1a's carries propagate only toward high bits, so its low bits are
     effectively unmixed — indexing a 64-entry list with them produced 2
     distinct nouns across 255 peers in testing.
  3. `ADJ[(h >> 32) % |ADJ|]` + `NOUN[(h & 0xFFFFFFFF) % |NOUN|]` — e.g.
     *Curious Cat*, *Lucky Dog*.

  A cryptographic hash buys nothing here: the peer id is already a digest, so
  the derivation only needs even distribution, and a trivially
  reimplementable function keeps cross-implementation parity cheap.

- Wordlist **order is part of the wire contract** — entries are selected by
  index, so reordering or removing one renames every affected peer. Lists are
  append-only; a breaking change requires bumping the domain string.
- 64 x 64 = 4096 names. At a full 255-member shard expect ~8 colliding pairs
  (measured: 5 across 255 synthetic ids), resolved as below.
- Both wordlists are curated so **no** combination is offensive. Safety is a
  property of the lists, not of a filter.
- Deterministic means every observer names a peer identically, so verbal
  coordination works ("follow Curious Cat"), and the name is stable across
  sessions.
- On an observed collision within a shard, append a discriminator from the
  next hash bytes — locally, only where the collision is seen.
- Friends always override to their real display name.

Note this is a stable pseudonymous identifier, linkable across sessions —
the same property the peer id already has, so it leaks nothing new.

#### 17.3.8 Visibility

| Viewer | Sees |
|---|---|
| Friend | Display name, full presence (§8.1) |
| Shard peer (non-friend) | Pseudonym + per-title ambient KVs only |
| Linked peer (non-friend, non-shard) | Nothing, unless relaying for you (`relay`) |

Display name and friend code are never sent to non-friends, with one
deliberate exception: a **friend request** carries the requester's asserted
name (§8.2). It has to — you cannot decide whether to accept a stranger
without seeing what they call themselves. Do not "fix" this; it would make
friend requests undecidable. The UI labels such names as self-asserted.

Enforcement is layered, and both halves matter:

- **Core (authoritative).** `push_presence` targets only mutual friends, and
  inbound presence from a non-friend is dropped. `ShardBeat` (§17.3.4) has no
  name field at all, so a shard broadcast cannot carry one even by mistake.
- **Shim (defensive).** The renderer substitutes a pseudonym for any
  non-friend regardless of what arrived, so a single missed check upstream
  cannot put a real name on screen.

### 17.4 Active

Game-driven, unchanged: the title opens its own connections (an XRNM
on the game plane) via §10 sessions and §8.3 invites. The manual invite
button stays — a number of titles support direct join.

### 17.5 Configuration (extends §11.1)

```toml
[shard]
enabled = true   # opt-in; default false
cap = 255        # optional override
```

### 17.6 Open items

- Shard rebalance cadence and hysteresis (§17.3.3) — untuned; too eager
  causes churn, too lazy leaves the population fragmented.
- Whether `created_at` needs to resist a wrong/hostile clock, given ties
  already fall back to `shard_id`.
- Gossipsub message rates for presence-dense titles at a full 255.

---

## 18. Guest socket surface (evidence from real titles)

The XDK surface a title actually uses varies far more than "it does
multiplayer" suggests, and each shape breaks somewhere different. Five titles
audited so far:

| | Fable II | SoulCalibur IV | Armored Core 4 | Fable III | Sonic Unleashed |
|---|---|---|---|---|---|
| I/O model | overlapped + APC | `select` | **`WSAEventSelect`** | overlapped + `select` | `select` + `__WSAFDIsSet` |
| Transport | UDP (XRNM) | **TCP** + UDP | UDP | **TCP** + UDP | **TCP only** |
| `XSession*` imports | via XGI messages | **none** | **none** | **none** | **none** |
| Notable | `XNetConnect` gating | `listen`/`accept` on 1001 | DNS, `XNetGetOpt` | `XNetQosListen`, `XNetRandom` | Havok VDB, not multiplayer |

What that cost us, in order of discovery:

- **Fable II** — `WSARecvFrom` was missing `lpFromlen`, shifting every later
  argument. Overlapped receives completed into the caller's from-length and
  the guest never learned a datagram had arrived.
- **SoulCalibur IV** — System Link is not UDP-only. Virtual-IP routing was
  gated on `SOCK_DGRAM`, so a TCP connect to `10.77.x.x` fell through to the
  host stack, which has no route there. Hence §18's guest TCP.
- **Armored Core 4** — `WSAEventSelect` was a stub, so an event-driven title
  would wait forever on an event nothing ever signalled.
- **Fable III** — a superset of the others; no new gap, which is the first
  time the audit came back clean.
- **Sonic Unleashed** — clean, and instructive for a different reason. Its
  entire socket surface belongs to `hkBsdSocket.cpp`: it is **Havok's Visual
  Debugger**, not multiplayer. Sixteen imports, no `sendto`/`recvfrom`
  anywhere in the binary. The lesson is that a socket import table proves a
  title *has* networking, never that it has *netplay* — and that "TCP-only
  title" is a real shape the SDK must handle.

  It is nonetheless the best available exercise for §18.1: it binds, listens,
  and polls with `select`/`__WSAFDIsSet`/`accept` — precisely the untested
  path — and needs no second player, since the game is the server and a
  host-side VDB connects in. Caveat: `hkBsdSocket` being linked does not
  prove the listener starts, as retail builds often disable VDB startup.

The recurring shape is worth naming: **every blocker was at the layer that
wakes the game up** — a completion routine, a readable event, a signalled
handle. Data arriving is not the same as the guest being told, and a title
that is never told simply waits, which reads as a hang rather than an error.
Auditing a candidate title's imports costs minutes and found a real blocker
in three of the five.

A note on scope, since the same binaries get proposed for other problems:
this technique works because socket APIs are **imported by name**, so every
call site arrives pre-labelled. It does not transfer to graphics. Xbox 360
titles statically link D3D9 into the XEX and ship it symbol-stripped — these
five import no D3D at all, only `Vd*` kernel calls — so there is no import
table to read and nothing is labelled. For renderer work a title's own binary
is the only useful source.

### 18.1 Guest TCP over libp2p streams

A guest `SOCK_STREAM` socket whose peer is a virtual IP is carried by a
libp2p stream (`/rexnet/tcp/1.0.0`), not by the punched UDP game plane:
the connection we already hold is reliable, ordered and multiplexed, and
reimplementing that over datagrams would be rebuilding it worse.

A stream identifies a *peer* but never a *listener*, so each opens with a
5-byte big-endian header carrying the destination and source guest ports.
Without it the shim cannot match an inbound connection to a listening socket.

Guest-visible behaviour follows Winsock rather than convenience:

- `connect()` to a virtual IP returns `WSAEWOULDBLOCK`; System Link titles set
  `FIONBIO` first and poll, so completing synchronously would be the lie.
- `listen()` registers on **both** planes — a title cannot know whether the
  peer that answers is on RexNet or the real LAN.
- `recv()` reports EOF only once buffered bytes drain, which is what separates
  "nothing yet" from "never again".
- readability covers pending data, pending accepts, **and** close, for both
  `select()` and `WSAEventSelect`.

---

## 19. Build integration

RexNet is the SDK's **only** Rust dependency. That is a real cost to impose
on someone who just wants to recompile a single-player game, so the module is
opt-in and compiles out completely.

```
cmake -DREXGLUE_ENABLE_REXNET=ON ...     # netplay; requires a Rust toolchain
cmake -DREXGLUE_ENABLE_REXNET=OFF ...    # default; no Rust needed at all
```

**What OFF actually means** — verified, not assumed: cargo is invoked zero
times, no Rust staticlib is produced, no RexNet objects are compiled, and no
`RexNetOverlay` symbols appear in the runtime library. The XAM entry points
keep their offline behaviour and the F6 overlay bind is not registered, since
a keybind that opens nothing reads as a broken feature rather than an absent
one.

### 19.1 Toolchain gate

`cmake/rex_rust.cmake` runs **before** `thirdparty/` pulls in corrosion, so a
missing toolchain is reported as a missing toolchain. Without it the failure
surfaces from inside corrosion's own configure and never mentions Rust.

It distinguishes four cases, because they need four different fixes: cargo or
rustc absent; a rustup shim on PATH with no toolchain installed (`rustc`
exists but fails to run); a toolchain older than the minimum; and a version
string it cannot parse, which warns and continues rather than refusing a
nightly that would probably work.

The minimum is **Rust 1.88.0**, and it is *derived*: the highest
`rust-version` declared across the resolved lockfile (`time` 0.3.53).
Re-derive it after a dependency bump rather than raising it on a hunch —

```sh
# for each package in Cargo.lock, read its declared rust-version
grep -h '^rust-version' ~/.cargo/registry/src/*/<pkg>/Cargo.toml
```

### 19.2 The public-header rule

`REXGLUE_ENABLE_REXNET` is `PUBLIC` on `rexruntime`, because consumers compile
their own copy of `rex_app.cpp` and must agree with the library they link.

That is only safe while **no public header changes shape with it**.
`rex_app.h` therefore holds the RexNet overlay as `ui::ImGuiDialog` rather
than the concrete dialog type, exactly as `achievements_overlay_` beside it
already does, so the struct layout is identical either way. If a public
header ever branches on this define, a consumer built without it will
silently disagree about object layout — which fails as memory corruption, not
as a link error. Keep the define out of installed headers.

The *link* stays `PRIVATE`: `rexnet` is an OBJECT library absorbed into
`rexruntime`, so consumers never link it and exporting it would only add a
target they cannot resolve.

### 19.3 Cargo and ninja job control

Ninja treats the entire cargo invocation as **one edge**. It therefore runs
its full parallel width of C++ compiles alongside it, while cargo — told
nothing — spawns one rustc per core inside that single edge. On a 16-core
host that is roughly 18 C++ jobs plus 16 rustc jobs at once, and rustc is
memory-hungry, so the failure mode is an OOM during a full build rather than
merely slow compilation.

Cargo is therefore capped at half the logical cores by default, overridable
via `REXGLUE_REXNET_CARGO_JOBS` (0 disables the cap). Ninja 1.13 added a GNU
jobserver that solves this properly; this is the portable fix until that is
the floor.

Incremental behaviour is as it should be: editing a `.rs` file propagates
through cargo to a relink. Note that cargo runs on *every* build even when
nothing changed — corrosion delegates up-to-date checking to cargo itself, so
ninja can never report "no work to do" for this target.

---

*This document describes an independent, open-source project for game
preservation. It does not connect to or interoperate with Xbox Live, and is
not affiliated with or endorsed by Microsoft.*
