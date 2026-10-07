# rexnet

Serverless Xbox Live replacement module. Design: [docs/rexnet-design-spec.md](../../docs/rexnet-design-spec.md).

Layout (spec §2):

| Layer | Where | What |
|---|---|---|
| L3 XAM shim | `*.cpp` here + `include/rex/net/` | XNet/XSession/XUser/XPresence/XNotify/XInvite semantics, virtual-IP table, BE↔LE marshalling |
| L2 FFI | `include/rex/net/rexnet_ffi.h` ↔ `core/src/ffi.rs` | C ABI, command queue in / event queue out, `catch_unwind` |
| L1 core | `core/` (`rexnet-core`, Rust) | identity, libp2p (kad/identify/dcutr/relay/mdns), punch engine, game-plane fast path. `core/src/stream/` is a vendored copy of libp2p-stream. No Xbox concepts. |

Build: `-DREXGLUE_ENABLE_REXNET=ON` (needs a Rust toolchain >= 1.88, built
via corrosion). OFF by default; everything here is compiled out.

## Milestone-1 CLI

`cargo run --bin rexnet-cli` inside `core/`:

```
rexnet-cli id     [--dir DIR]                  # print peer id / friend code
rexnet-cli listen [--dir DIR] [--port N] [--relay MULTIADDR] [--amino]
rexnet-cli dial   <MULTIADDR> [--dir DIR]      # connect + echo round trip
rexnet-cli find   <PEER_ID>   [--dir DIR]      # Amino-DHT FindPeer + echo
```

The engine (`core/src/engine.rs`) runs QUIC+TCP transports with kad
(Amino-compatible, client mode), identify, AutoNAT, relay client, DCUtR, and
mDNS. Verified: two-instance QUIC dial + `/rexnet/echo/1.0.0` round trip and
automatic LAN mDNS discovery. Public-DHT FindPeer and relay/DCUtR across real
NATs still need testing from an open network.

## Milestone status

1. **P2P core proof** — done (loopback/LAN; open-network NAT runs pending).
2. **FFI + runtime integration** — done: first `XNetStartup` lazily creates
   the shared instance (refcounted, torn down on last `XNetCleanup`; see
   `xam/xam_net.cpp`), a ~60 Hz pump thread drains events. Cvars: `rexnet`,
   `rexnet_display_name`, `rexnet_bootstrap` (`amino`/`none`/multiaddr list).
3. **Second-socket game plane** — core done: §8.4 punch (offer/answer over
   `/rexnet/punch/1.0.0`, nonce'd probes on the dedicated UDP socket),
   core-owned virtual-IP allocation (`PeerConnected` carries the vip), and
   guest-port-multiplexed datagram framing. `PeerConnected`/vips are gated
   to app-relevant peers (explicit dials, friends, session hits, inbound
   rexnet protocol traffic); public-DHT crawl connections stay invisible
   and only app-requested dial failures surface as `Event::Error`. Verified two-instance punch +
   datagram over loopback. Game-plane encryption (§6.1) and the tunnel
   fallback (§14) followed; the hot path now runs off the swarm loop
   (`core/src/game_plane.rs`) and the pump is event-woken (§6).
4. **XAM shim** — XNet layer done (`xam/xam_net.cpp`): XNADDR synthesis
   (vip + abOnline peer key), `XNetXnAddrToInAddr`/`InAddrToXnAddr`,
   `XNetConnect`/`XNetGetConnectStatus` (XRNM-load-bearing, auto-punch on
   connect), guest `sendto` to 10.77.0.0/16 routed through the game plane
   (`XSocket::SendTo`), inbound datagrams queued to the bound guest socket
   (`XSocket::QueuePacket` + queue-aware `recvfrom`/`select`), Qos
   listen/lookup success paths, key functions as no-ops, XNotify sink
   plumbed to `BroadcastNotification`. Verified: two-instance
   connect-status-datagram loop through the exact shim API.
   Overlapped `WSARecvFrom` (the §11.2 XRNM-receive prerequisite) is done:
   one pending receive per socket, completed inline by RexNet packet
   inserts or by a background poller for host-socket data, plus a real
   `WSAGetOverlappedResult`. Verified against a live KernelState (both
   completion sources, guest buffers/event).
5. **Sessions (generic §10)** — core: local-session registry, DHT provider
   records for public sessions (`rexnet/v1/title/<id>/sessions`), and the
   `/rexnet/session/1.0.0` descriptor query; kad serves (`Mode::Server`)
   on LAN/private meshes and stays a polite client on Amino.
   Shim: `XGISessionCreate` allocates the 128-bit session id, fills real
   `XSESSION_INFO` (host XNADDR + XNKID/XNKEY) and nonce; Delete unpublishes.
   `XamUser*` now reports a stable identity-derived XUID + the
   `rexnet_display_name`. Verified: host/search/descriptor round trip.
   `XSessionSearch`/`-Ex` now complete asynchronously: apps can defer
   XMsg overlappeds (new `App::DispatchMessageAsync` hook), and XGI runs
   the DHT search on the dispatch thread with a bounded wait, marshalling
   real `XSESSION_SEARCHRESULT` arrays (header + XSESSION_INFO + slot
   counts) into the guest buffer. Marshalling is compile-verified; full
   path needs a guest title (dispatch-thread machinery is stock).
6. **Social protocols (§8.1–8.3)** — core + FFI + C++ surface done:
   friends are mutual-consent (`/rexnet/friend/1.0.0`, local `friends.txt`,
   crossed requests auto-accept), presence is friend-gated at the protocol
   layer (pushed on change, 30 s heartbeat, pushed on friend connect;
   transport auth stands in for the record signature until records are
   relayed), invites carry the current session and an accepting invitee
   automatically receives the host descriptor — private sessions are served
   to invitees only. Verified end to end in the two-instance test.
7. **Guest-facing bridges + per-game config** — done:
   - `rexnet.toml` (per-title metadata dir, §11.1) loaded at `XNetStartup`:
     `session_model`, `game_channel`, `[presence.rich]` context→KV mappings.
   - `XGIUserSetContextEx`/`SetPropertyEx` map through the config into
     presence rich KVs (`[count][key u16 LE][type][len][bytes]` blob).
   - Invite accept bridge: `RexNet::AcceptInvite` → host descriptor arrives
     → `XN_LIVE_INVITE_ACCEPTED` (0x02000002) into the XNotify queue and
     `XMessageGameInviteGetAcceptedInfo` (XLiveBase 0x00058023) fills a real
     XINVITE_INFO (inviter XUID + XSESSION_INFO). Presence updates also
     fire `XN_FRIENDS_PRESENCE_CHANGED` (0x04000001).
   - Friend-list mirror in the shim (bootstrapped by per-friend events at
     engine start; `GetFriends`/`SnapshotFriends` with presence join).
   - Friends enumerator (`xam_friends.cpp`): guest `XONLINE_FRIEND`
     marshalling into an `XamEnumerate`-able handle. Two entry points: the
     `XFriendsCreateEnumerator` export, and XLiveBase msg 0x58020 — Fable 2
     never imports the export; the XDK's title-side stub (0x82CFFAB0) is
     statically linked and marshals a CArgumentList through
     `XMsgInProcessCall(0xFC, 0x58020, ...)` (both paths share
     `xeXFriendsCreateEnumerator`). Record layout verified against the
     title's walker (`NLivePresence::ConnectToFriend`, 0x822FA988): stride
     0xC4, state 0x18, session XNKID 0x1C, title id 0x24 (early-360 XDK
     layout), guarded by offset static_asserts. Remaining: presence doesn't
     carry the friend's session XNKID yet, so JOINABLE stays clear and
     join-from-friends-list is inert — joining flows through invites.
8. **F6 diagnostics overlay** — an ImGui overlay (toggle **F6**, alongside
   F3 debug / F4 settings / F7 achievements) showing identity (base58 peer
   id), session state, and a live peer table (virtual IP / status / name),
   plus a manual-connect multiaddr box (§4 floor). The UI layer sits below
   the shim, so it reads through a provider/action registry
   (`rex/ui/overlay/rexnet_overlay.h`) that the rexnet module installs at
   `InitializeShared` — no rexui→kernel dependency, no duplicated singleton
   (both are OBJECT libs folded once into rexruntime). Data path verified
   headless (`QueryRexNetStatus` returns live identity + peer table).
   Social surface: friends table (presence name, online/in-game, Invite +
   Remove), pending friend requests and session invites with Accept/
   Dismiss/Decline, and an add-friend box (backed by the
   `rexnet_peer_id_parse` FFI; requests/invites are held in non-draining
   pending lists on the shim — `PendingFriendRequests`/`PendingInvites` —
   resolved by the overlay actions). Identity is exchanged as a **friend
   code** (`REXN-` + dash-grouped base32 of the raw ed25519 key + the
   display name (≤24 bytes) + a 20-bit CRC group; `identity.rs`): the
   overlay shows/copies it and the add-friend box accepts codes
   (case/dash/0↔O/1↔I tolerant) or raw base58 peer ids. A pasted code's
   embedded name labels the friend until presence arrives
   (`friend_names.txt`), as does the name carried in friend requests.
   The overlay's display-name box renames live (`rexnet_set_display_name`
   re-pushes presence; persisted to `display_name.txt`, which wins over
   the `rexnet_display_name` cvar at startup), and friends' Status shows
   the advertised game name (reserved presence rich KV 0xFF00 fed from
   the title's XDBF) instead of a raw title id.

`include/rex/net/rexnet_ffi.h` is generated from `core/src/ffi.rs` by
`core/build.rs` on every cargo build (config in `core/cbindgen.toml`); edit
the Rust, commit the regenerated header. `.github/workflows/rexnet.yaml`
builds the crate and the SDK with the module on, runs the C++ and cargo
tests including the loopback network tests, and fails on header drift.

## Discord Rich Presence and invites

RexNet speaks to the Discord client on the machine itself, over Discord's local IPC
endpoint (`discord-ipc-N` under the runtime directory, a named pipe on Windows) — no
Discord SDK, no service of ours. A title opts in with a `[discord]` section in its
`rexnet.toml` naming its own Discord application:

```toml
[discord]
application_id = "123456789012345678"   # the title's app in the developer portal
details = "Playing"                      # defaults for the two lines Discord shows
state = ""
large_image = "logo"                     # an art asset key on that application
large_text = "Title name"
```

While a local session admits a joiner the activity carries a party and a **join secret**,
`RexNet::JoinSecret()`: `REXJ3|title|peer|session|shard|endpoint|token`. A friend who accepts
the invite gets that secret on their side and `RexNet::JoinFromSecret()` does the whole join:
dials the carried public endpoint if there is one, arms the accept so the title's own
accepted-invite path runs (`AcceptInvite`), queries the host's descriptor with the token
(`rexnet_session_query`) with a DHT search as the fallback, and enters the host's title shard
by id (`rexnet_shard_join`), so the two are in the same ambient tier as well as the same
session. A join makes nobody a friend: friendship stays the players' own decision. The secret
is cached and rebuilt only when one of its inputs changes.

The host may state the two lines live (`RexNet::SetDiscordActivity`, the player's whereabouts
for instance) and turn presence off as a player setting (`SetDiscordEnabled`); otherwise the
config defaults stand and joinable means "a local session exists". Discord's IPC is pumped at
most four times a second from the event pump, never per event wake, over a non-blocking socket
with a bounded outbox; PING is answered, a CLOSE drops the connection, and a handshake Discord
refuses (an application id it does not know) backs the reconnect off 15 s -> 60 s -> 300 s.

### The join token

`SessionCreate` draws a random 64-bit **join token** (`std::random_device` seeding
`mt19937_64`) and hands it to the core (`rexnet_session_set_join_token`; `SessionDelete` clears
it). It is the proof that join material came from the host: the core serves a private
session's descriptor only to a `rexnet_session_query` carrying the current token, so the peer
id and session id alone -- visible to anyone the host ever played with -- no longer open a
private game. `JoinFromSecret` accepts three forms:

| Form | Fields | Token |
|---|---|---|
| `REXJ3` | `REXJ3\|title\|peer\|session\|shard\|endpoint\|token` (token = 16 hex digits) | carried |
| `REXJ2` | `REXJ2\|title\|peer\|session\|shard\|endpoint` | 0 (public sessions only) |
| first cut | `peer\|title\|session` | 0 (public sessions only) |

`RexNet::join_token()` reads the current one (0 = none).

The core's admission rule, exactly: a **public** session's descriptor is served to any
querier; a **private** one only to a peer the host invited (`rexnet_send_invite` records it)
or one whose query carries the current token. The right token admits the peer for good --
its later queries and its join need no token -- and the admitted set is cleared with the
session, on create as on delete. A private session's id never rides the shard beat; only a
public one is announced to the ambient tier. The query from a build without the token field
still parses as "no token".

### Ask to Join: the policy is the title's

`RexNet::SetJoinRequestPolicy(JoinRequestPolicy)` says who gets through Discord's "Ask to
Join" without the host's say-so; the default is **Ask**.

- `Ask` -- every request is queued. The title drains them with `TakeJoinRequests()`
  (`{user_id, user_name}` as Discord named them) and answers with
  `ReplyJoinRequest(user_id, accept)`: accept sends `SEND_ACTIVITY_JOIN_INVITE`, decline
  `CLOSE_ACTIVITY_REQUEST`. A request unanswered for 60 s is dropped silently; Discord's own
  timeout is shorter, so there is nobody left to answer.
- `Friends` -- a Discord user the title has linked to a RexNet friend
  (`LinkDiscordUser(user_id, peer)`) is approved at once; anyone else is queued as under Ask.
  RexNet keeps only the map; where the link comes from (an account link, a profile) is the
  title's knowledge.
- `All` -- approved while the session admits a joiner.

Whatever the policy, nothing is approved while the session is not joinable.

### One accept at a time, and a timeout

`AcceptInvite` (which every join path goes through: a received invite, an uninvited join to
a friend's session, Discord's Join, pasted material) arms one pending accept per host and
session. A repeat for the same host and session while one is pending returns false and is not
re-armed, so a second press or a second Discord Join cannot run the title's join twice. A host
that never answers times out after 20 s: the pending accept is cleared and the title hears it
as a notice.

### Notices

Errors the player should see go through one path: `SetNoticeSink` (called from the pump
thread) and `TakeNotices()` for titles that poll. `Notice::Kind` is `kError` (the core's
`REXNET_EVENT_ERROR` text), `kInviteFailed` (`REXNET_EVENT_INVITE_FAILED`: the invite to
`peer` did not land) or `kJoinTimedOut` (the host named by `peer` did not answer the accept).
Received invites are one per peer (a newer one replaces the older) and expire five minutes
after receipt.

### Slots

`SetSessionSlots(open, total)` states the local session's occupancy to the core
(`rexnet_session_set_slots`): it is what the descriptor advertises and what Discord shows as
the party size. RexNet never sees the title's join and leave calls -- they are XGI messages the
kernel layer handles -- so the XGI shim or the title calls this after each change; nothing is
inferred.

### DEGRADED means connected over the tunnel

`REXNET_EVENT_DEGRADED` says the punch failed and the peer is reachable through the control
tunnel. The shim marks that peer `kConnected` with `PeerLink::degraded` set (visible as
`Connection::degraded` and "connected (tunnel)" in the F6 overlay), so a title that gates its
sends on `XNET_CONNECT_STATUS_CONNECTED` (XRNM) sends and the core carries the datagrams over
the tunnel. A failed punch on its own leaves the link **pending**, never lost: the core follows
it with DEGRADED when the tunnel carries the peer or `PEER_DISCONNECTED` when nothing does.
A tunnelled peer whose control connection blips is resumed on the tunnel the moment the
connection is back (DEGRADED again, no second punch), and the recovery grace outlasts a punch
so a re-punched peer is never reported gone while its punch is still resolving. The core never
tunnels to a peer it holds no control connection to: there is nothing to carry it.

### Datagram size

`REXNET_EVENT_DATA_MAX` (1472) is the contract both ways. A guest datagram larger than that is
dropped at the sender and, should one arrive, at the receiver -- counted (`oversized_drops` in
the shard status line), never truncated into something the title would misparse. Guest TCP
reads are chunked to fit an event with their stream id.

### Shard membership and addresses

Membership is pruned: a member leaves the table on gossipsub unsubscribe, on departure
(connection gone and recovery exhausted), or after 95 s without a beat or descriptor. While a
**session is up** -- a local session, a peer in one with us by invite, or title traffic in the
last 20 s -- no address moves: a member arriving or leaving mid-game is placed beside, or
removed from, the pinned host table, and the full re-derivation waits until the session is
over. A warmed or tunnelled game plane carrying nothing is not a session and pins nothing.

## Reachability, recovery and hygiene (from the original-Xbox fork review, 2026-10-07)

Taken from Creesic's `codex/rexnet-original-xbox` branch where it was generic:

- **Each node publishes its own reachability record** (`rexnet/v1/peer/<id>`) as a DHT
  provider record, at every heartbeat until a DHT node stores it and then every ten
  minutes. Friends and peers owed a message are found through that key with a 30 s to
  10 min backoff. A client-mode node is never in a routing table, so this is the only
  lookup that works between two players on the public DHT.
- **Friend requests report delivery** (`REXNET_EVENT_FRIEND_DELIVERED`, the other side's
  ack) as well as a decline; "sent" is only the node taking the message.
- **UPnP maps the game socket**, not just the swarm's listeners, so a NAT'd peer's probes
  reach the public game port instead of falling back to a relayed tunnel.
- **Relay hygiene**: a relay that refuses or drops a reservation is backed off ten minutes,
  bootstrap nodes are never relays, and the reservation address is a global non-circuit
  one chosen deterministically. **Inbound hygiene**: a public DHT peer that dialled us
  and speaks nothing of ours is closed after identify so the crawl cannot fill the
  inbound budget.
- **Bounded game-peer recovery**: when a game peer's last control connection drops, its
  validated routes (TCP first, then a relay circuit, QUIC last for a minute after a QUIC
  failure) are redialled every second for five seconds, with one own-key lookup if no
  route is known; the title hears the peer leave only when that fails.
- **A direct session query** (`rexnet_session_query`) for a join that already knows the
  host, used by `JoinFromSecret`; and **old-build detection**: an unsupported-protocol
  failure is reported as the peer running an older build.

Left behind as title-specific: clubs, signed notices, population heartbeats, the
diagnostics recorder, and the quinn-udp Wine/Proton backport (only needed to run a
Windows build under Wine).
