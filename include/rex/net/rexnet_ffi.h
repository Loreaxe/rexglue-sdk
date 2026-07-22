/**
 * @file        net/rexnet_ffi.h
 * @brief       C ABI of rexnet-core (Rust) — design spec §12.
 *
 * @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
 *              All rights reserved.
 *
 * @license     BSD 3-Clause License
 *              See LICENSE file in the project root for full license text.
 *
 * Hand-maintained mirror of src/rexnet/core/src/ffi.rs; keep in sync until
 * cbindgen generation is wired into the build (see core/cbindgen.toml).
 * Non-blocking command-in / poll-event-out: the library owns its tokio
 * runtime and the game thread never blocks.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { REXNET_EVENT_DATA_MAX = 1472 };

typedef struct RexNetHandle RexNetHandle;

typedef struct RexNetConfig {
  /** Directory holding identity.key and friend/session state. UTF-8. */
  const char* data_dir;
  /** Title id derived by the shim from the XEX execution-info header. */
  uint32_t title_id;
  /** Self-asserted display name, UTF-8, <= 32 bytes. */
  const char* display_name;
  /** Bootstrap multiaddrs; empty list is valid (LAN/manual/v6 still work). */
  const char* const* bootstrap;
  uint32_t bootstrap_len;
  /** Also merge in the standard public (Amino) bootstrap set. */
  bool use_default_bootstrap;
  /** Skip hole punching; carry game traffic over the control tunnel
   *  (spec §14). Reproduces a CGNAT player's degraded path on a LAN, where
   *  every punch would otherwise succeed and leave the fallback untested.
   *  Field order here mirrors the Rust struct exactly -- these are two
   *  adjacent bools, so transposing them swaps silently. */
  bool force_tunnel;
} RexNetConfig;

/** Multihash-encoded PeerId, length-prefixed. */
typedef struct RexNetPeerId {
  uint8_t len;
  uint8_t bytes[63];
} RexNetPeerId;

typedef enum RexNetEventKind {
  REXNET_EVENT_NONE = 0,
  REXNET_EVENT_PEER_CONNECTED,
  REXNET_EVENT_PEER_DISCONNECTED,
  REXNET_EVENT_PRESENCE_UPDATED,
  REXNET_EVENT_FRIEND_REQUEST,
  REXNET_EVENT_FRIEND_ACCEPTED,
  REXNET_EVENT_INVITE_RECEIVED,
  REXNET_EVENT_INVITE_REPLIED,
  REXNET_EVENT_PUNCH_RESULT,
  REXNET_EVENT_SESSION_FOUND,
  REXNET_EVENT_DATAGRAM,
  /** Punch failed; traffic for `peer` rides the control tunnel. */
  REXNET_EVENT_DEGRADED,
  REXNET_EVENT_ERROR,
  REXNET_EVENT_FRIEND_REMOVED,
  /** Ambient shard presence (spec §17.3.4). flag = state; data =
   *  [has_session u8][session_id 16 if set][rich...]. Carries no display
   *  name by design — the shim renders a pseudonym unless the peer is a
   *  friend (§17.3.8). Append-only: mirrors the Rust enum ordering. */
  REXNET_EVENT_SHARD_PRESENCE,
  /** A peer started/stopped carrying our traffic as a circuit relay
   *  (spec §17.2). flag = is_relay. A non-friend relay is the only
   *  non-friend the UI names, and it is named by role ("relay"), never
   *  by identity. */
  REXNET_EVENT_RELAY_STATUS,
  /** Our own virtual address changed — joined/left/migrated shards
   *  (spec §17.3.5). virtual_ip carries the new address. */
  REXNET_EVENT_LOCAL_ADDRESS,
  /** Guest TCP connection opened (spec §18). virtual_ip = peer,
   *  port = local guest port, src_port = remote guest port,
   *  flag = 1 when we opened it (0 = accepted).
   *  data = 8-byte big-endian stream id. */
  REXNET_EVENT_STREAM_OPENED,
  /** Bytes on a guest TCP connection. data = [stream id 8][payload]. */
  REXNET_EVENT_STREAM_DATA,
  /** Guest TCP connection ended. data = 8-byte big-endian stream id. */
  REXNET_EVENT_STREAM_CLOSED,
  /** Outbound guest TCP connect failed. virtual_ip = peer, port = dst,
   *  data = UTF-8 reason. */
  REXNET_EVENT_STREAM_CONNECT_FAILED,
  /** Measured round trip to a peer (spec §11). virtual_ip = peer,
   *  port = milliseconds. Emitted only on a material change. */
  REXNET_EVENT_PEER_RTT,
} RexNetEventKind;

/** Fixed-size POD event; drain once per frame with rexnet_poll_event(). */
typedef struct RexNetEvent {
  uint32_t kind; /**< RexNetEventKind */
  RexNetPeerId peer;
  /** PeerConnected: allocated 10.77.0.0/16 address; Datagram: sender vip. */
  uint32_t virtual_ip;
  uint16_t port;     /**< Datagram: guest destination port */
  uint16_t src_port; /**< Datagram: guest source port (reply routing) */
  uint8_t flag;      /**< PunchResult ok / InviteReplied reply code */
  uint8_t _pad;
  uint32_t data_len;
  uint8_t data[REXNET_EVENT_DATA_MAX];
} RexNetEvent;

/** Returns NULL on failure. */
RexNetHandle* rexnet_init(const RexNetConfig* cfg);
void rexnet_shutdown(RexNetHandle* handle);

/** Level ordinals passed to a RexNetLogSink. */
enum {
  REXNET_LOG_ERROR = 0,
  REXNET_LOG_WARN = 1,
  REXNET_LOG_INFO = 2,
  REXNET_LOG_DEBUG = 3,
  REXNET_LOG_TRACE = 4
};
/** Receives one formatted log line from rexnet-core. Called from the engine's
 *  own threads, so it must be thread-safe. The message is only valid for the
 *  duration of the call. */
typedef void (*RexNetLogSink)(uint32_t level, const char* message);
/** Route rexnet-core's internal logging to `sink` (NULL silences it).
 *  Without this the crate's `tracing` output is discarded entirely when
 *  linked as a staticlib — no subscriber is installed — leaving the whole P2P
 *  core invisible. Verbosity follows the REXNET_LOG env var (default info). */
void rexnet_set_log_sink(RexNetLogSink sink);

void rexnet_local_peer_id(RexNetHandle* handle, RexNetPeerId* out);

/** Opt in to the ambient title shard (spec §17.3). Idempotent; cap 0 uses the
 *  spec default of 255. Titles opt in via rexnet.toml (§17.5); ambient
 *  presence is never implied, so a game with no use for it pays nothing. */
/** Guest TCP connect to a peer's virtual IP (spec §18). System Link titles
 *  use stream sockets, not only datagrams -- SoulCalibur IV listens on TCP
 *  1001 -- so a guest SOCK_STREAM socket maps onto a libp2p stream rather
 *  than reimplementing TCP over the punched UDP path. Result arrives as
 *  STREAM_OPENED or STREAM_CONNECT_FAILED. */
void rexnet_stream_connect(RexNetHandle* handle, uint32_t virtual_ip, uint16_t src_port,
                           uint16_t dst_port);
/** Write to a guest TCP connection. */
void rexnet_stream_send(RexNetHandle* handle, uint64_t stream_id, const uint8_t* data,
                        uint32_t len);
/** Close a guest TCP connection. */
void rexnet_stream_close(RexNetHandle* handle, uint64_t stream_id);

void rexnet_shard_enable(RexNetHandle* handle, uint16_t cap);
/** Leave any shard and stop publishing/scanning. */
void rexnet_shard_disable(RexNetHandle* handle);
/* Render a peer id as base58 into out (NUL-terminated); returns length. */
uint32_t rexnet_peer_id_string(const RexNetPeerId* peer, char* out, uint32_t cap);
/* Parse a base58 peer-id string OR a REXN- friend code; false on malformed
 * input (checksum mismatches included). */
bool rexnet_peer_id_parse(const char* s, RexNetPeerId* out);
/* Render a REXN- friend code embedding an optional display name (NULL ok;
 * <=24 bytes used) into out (<=119 chars + NUL); returns length, or 0 for
 * non-ed25519 peer ids / insufficient cap. */
uint32_t rexnet_friend_code(const RexNetPeerId* peer, const char* name, char* out, uint32_t cap);
/* Parse a friend code, also returning the embedded display name ("" when
 * absent; name_out may be NULL); false on malformed input. */
bool rexnet_friend_code_parse(const char* s, RexNetPeerId* out_peer, char* name_out,
                              uint32_t name_cap);
/* Runtime rename: sanitized (<=24 bytes), presence re-pushed to friends. */
void rexnet_set_display_name(RexNetHandle* handle, const char* name);

/* Commands: enqueue and return immediately. Byte payloads are postcard-
 * encoded per the §8 schemas and owned by the caller. */
void rexnet_set_presence(RexNetHandle* handle, uint32_t title_id, uint8_t state,
                         const uint8_t* presence, uint32_t presence_len);
void rexnet_connect_peer(RexNetHandle* handle, const RexNetPeerId* peer);
/* Fire-and-forget game-plane punch; outcome arrives as a PunchResult event. */
void rexnet_punch_peer(RexNetHandle* handle, const RexNetPeerId* peer);
void rexnet_connect_manual(RexNetHandle* handle, const char* multiaddr);
/* Invites peer to the current local session; reply arrives as an
 * InviteReplied event (flag: 1 accepted / 0 declined). An accepting invitee
 * automatically fetches the host session descriptor (SessionFound event).
 * InviteReceived events carry: virtual_ip = title id, data = session id. */
void rexnet_send_invite(RexNetHandle* handle, const RexNetPeerId* peer);
void rexnet_invite_reply(RexNetHandle* handle, const RexNetPeerId* peer, bool accept);
/* Friendship is mutual consent (§8.2); presence only flows between mutual
 * friends. PresenceUpdated events carry: virtual_ip = title id, flag =
 * state, data = [name_len u8][name][rich]. */
void rexnet_friend_request(RexNetHandle* handle, const RexNetPeerId* peer, const char* note);
void rexnet_friend_accept(RexNetHandle* handle, const RexNetPeerId* peer);
void rexnet_friend_remove(RexNetHandle* handle, const RexNetPeerId* peer);
/* session_id: 16 bytes, allocated by the shim (guest sees XNKID+XNKEY).
 * Public sessions publish a DHT provider record; private ones never touch
 * the DHT. Search results arrive as SessionFound events (peer = host,
 * data = session id, port/src_port = total/open slots). */
void rexnet_session_create(RexNetHandle* handle, const uint8_t* session_id, uint8_t slots_total,
                           uint8_t slots_open, bool is_public);
void rexnet_session_delete(RexNetHandle* handle);
void rexnet_session_search(RexNetHandle* handle);
/* src_port/dst_port are guest ports (the game socket multiplexes them all).
 * `reliable` is for module-internal traffic only; guest-originated datagrams
 * MUST pass false for own-reliability titles (game_channel = "unreliable"). */
void rexnet_send_datagram(RexNetHandle* handle, uint32_t virtual_ip, uint16_t src_port,
                          uint16_t dst_port, const uint8_t* data, uint32_t len, bool reliable);

/** Returns true while events remain. */
bool rexnet_poll_event(RexNetHandle* handle, RexNetEvent* out);

#ifdef __cplusplus
}  // extern "C"
#endif
