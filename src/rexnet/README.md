# rexnet

Serverless Xbox Live replacement module. Design: [docs/rexnet-design-spec.md](../../docs/rexnet-design-spec.md).

Layout (spec §2):

| Layer | Where | What |
|---|---|---|
| L3 XAM shim | `*.cpp` here + `include/rex/net/` | XNet/XSession/XUser/XPresence/XNotify/XInvite semantics, virtual-IP table, BE↔LE marshalling |
| L2 FFI | `include/rex/net/rexnet_ffi.h` ↔ `core/src/ffi.rs` | C ABI, command queue in / event queue out, `catch_unwind` |
| L1 core | `core/` (`rexnet-core`, Rust) | identity, libp2p (kad/identify/dcutr/relay/mdns), punch engine. No Xbox concepts. |

Build: `-DREXGLUE_ENABLE_REXNET=ON` (needs a Rust toolchain; built via
corrosion). `-DREXGLUE_REXNET_GNS=ON` additionally builds
GameNetworkingSockets for the game plane (milestone 3+).

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
   datagram over loopback. TODO: game-plane encryption (blocked on a
   rust-libp2p keying-material exporter; probes are nonce-auth only),
   tunnel fallback for hostile NAT pairs, GNS framing decision (§16).
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

The FFI header is currently hand-maintained; keep `rexnet_ffi.h` in sync with
`core/src/ffi.rs` (cbindgen config is in `core/cbindgen.toml`).
