// @file        rexnet/core/src/ffi.rs
// @brief       C ABI (design spec §12): non-blocking command-in / poll-event-out.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! C ABI (design spec §12): non-blocking command-in / poll-event-out.
//!
//! The library owns its tokio runtime; the game thread never blocks. Every
//! export is wrapped in `catch_unwind` so a panic can never cross the FFI
//! boundary. `include/rex/net/rexnet_ffi.h` mirrors these declarations —
//! keep them in sync until cbindgen generation is wired into the build
//! (`cbindgen.toml` is already present).

use std::ffi::{c_char, CStr, CString};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Mutex, OnceLock};

use libp2p::PeerId;
use tokio::sync::mpsc;

use crate::engine::{self, Command, Event};
use crate::identity;

// --- Logging bridge -----------------------------------------------------
//
// rexnet-core logs through `tracing`. A subscriber is only installed by the
// CLI binary, so when the crate is linked into the host runtime as a
// staticlib every engine log line is silently discarded -- the entire P2P
// core is invisible exactly when something is going wrong in it. This
// forwards events to a host callback so they land in the host's own log
// alongside everything else.

/// Level ordinals handed to the sink. Deliberately explicit rather than
/// mirroring `tracing::Level`'s internal ordering.
pub const REXNET_LOG_ERROR: u32 = 0;
pub const REXNET_LOG_WARN: u32 = 1;
pub const REXNET_LOG_INFO: u32 = 2;
pub const REXNET_LOG_DEBUG: u32 = 3;
pub const REXNET_LOG_TRACE: u32 = 4;

/// `extern "C" fn(level, nul-terminated message)`. The message borrows only
/// for the duration of the call.
pub type RexNetLogSink = extern "C" fn(u32, *const c_char);

static LOG_SINK: AtomicUsize = AtomicUsize::new(0);
static SUBSCRIBER: OnceLock<()> = OnceLock::new();

/// Flattens an event's message and fields into one line.
#[derive(Default)]
struct LogVisitor(String);

impl tracing::field::Visit for LogVisitor {
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
        use std::fmt::Write;
        if field.name() == "message" {
            let _ = write!(self.0, "{value:?}");
        } else {
            let _ = write!(self.0, " {}={value:?}", field.name());
        }
    }
    fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
        use std::fmt::Write;
        if field.name() == "message" {
            self.0.push_str(value);
        } else {
            let _ = write!(self.0, " {}={value}", field.name());
        }
    }
}

struct SinkLayer;

impl<S: tracing::Subscriber> tracing_subscriber::Layer<S> for SinkLayer {
    fn on_event(&self, event: &tracing::Event<'_>, _ctx: tracing_subscriber::layer::Context<'_, S>) {
        let raw = LOG_SINK.load(Ordering::Acquire);
        if raw == 0 {
            return;
        }
        let mut visitor = LogVisitor::default();
        event.record(&mut visitor);
        // Debug-formatted values can contain interior nuls; drop the line
        // rather than truncating it into something misleading.
        let Ok(text) = CString::new(visitor.0) else { return };
        // SAFETY: pointer came from a `RexNetLogSink` in set_log_sink.
        let sink: RexNetLogSink = unsafe { std::mem::transmute(raw) };
        let level = match *event.metadata().level() {
            tracing::Level::ERROR => REXNET_LOG_ERROR,
            tracing::Level::WARN => REXNET_LOG_WARN,
            tracing::Level::INFO => REXNET_LOG_INFO,
            tracing::Level::DEBUG => REXNET_LOG_DEBUG,
            tracing::Level::TRACE => REXNET_LOG_TRACE,
        };
        sink(level, text.as_ptr());
    }
}

/// Route rexnet-core's `tracing` output to `sink`. Safe to call more than
/// once (the subscriber installs once; the sink is swapped). Pass `None` to
/// silence. Verbosity follows `REXNET_LOG` (default `info`).
///
/// # Safety
/// `sink` must be callable from any thread for the process lifetime, since
/// the engine logs from its own tokio threads.
#[no_mangle]
pub unsafe extern "C" fn rexnet_set_log_sink(sink: Option<RexNetLogSink>) {
    LOG_SINK.store(sink.map_or(0, |f| f as usize), Ordering::Release);
    SUBSCRIBER.get_or_init(|| {
        use tracing_subscriber::layer::SubscriberExt;
        use tracing_subscriber::util::SubscriberInitExt;
        let filter = tracing_subscriber::EnvFilter::try_from_env("REXNET_LOG")
            .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));
        // A failure here means something else already installed a global
        // subscriber (the CLI does). Not fatal: that subscriber keeps
        // working, we just do not also forward.
        let _ = tracing_subscriber::registry().with(filter).with(SinkLayer).try_init();
    });
}

/// Largest datagram payload deliverable in one event (fits standard MTU).
pub const REXNET_EVENT_DATA_MAX: usize = 1472;

#[repr(C)]
pub struct RexNetConfig {
    /// Directory holding `identity.key` and friend/session state. UTF-8.
    pub data_dir: *const c_char,
    /// Title id derived by the shim from the XEX execution-info header (§11.0).
    pub title_id: u32,
    /// Self-asserted display name, UTF-8, <= 32 bytes.
    pub display_name: *const c_char,
    /// Bootstrap multiaddrs; empty list is valid (LAN/manual/v6 still work).
    pub bootstrap: *const *const c_char,
    pub bootstrap_len: u32,
    /// Circuit-v2 relays to hold reservations on (§9). Accelerant, never
    /// authority: empty is supported and a dead entry costs only speed.
    pub relays: *const *const c_char,
    pub relays_len: u32,
    /// Also merge in [`crate::AMINO_BOOTSTRAP`] (the standard public set).
    pub use_default_bootstrap: bool,
    /// Skip hole punching; carry game traffic over the control tunnel (§14).
    /// Lets a developer reproduce a CGNAT player's degraded path on a LAN,
    /// where every punch would otherwise succeed.
    pub force_tunnel: bool,
    /// Fixed control-plane (QUIC/TCP) listen port; 0 = ephemeral. A stable
    /// port is what makes a UPnP mapping or a manual forward reusable across
    /// launches, so the address a peer pastes stays valid.
    pub listen_port: u16,
    /// Fixed game-plane UDP port; 0 = ephemeral.
    pub game_port: u16,
}

/// Multihash-encoded PeerId, length-prefixed (ed25519 identity hashes fit
/// well under 64 bytes).
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RexNetPeerId {
    pub len: u8,
    pub bytes: [u8; 63],
}

impl RexNetPeerId {
    fn from_peer(peer: &PeerId) -> Self {
        let raw = peer.to_bytes();
        let mut out = Self { len: 0, bytes: [0; 63] };
        let n = raw.len().min(63);
        out.len = n as u8;
        out.bytes[..n].copy_from_slice(&raw[..n]);
        out
    }

    fn to_peer(&self) -> Option<PeerId> {
        PeerId::from_bytes(&self.bytes[..self.len as usize]).ok()
    }
}

impl Default for RexNetPeerId {
    fn default() -> Self {
        Self { len: 0, bytes: [0; 63] }
    }
}

#[repr(u32)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RexNetEventKind {
    None = 0,
    PeerConnected,
    PeerDisconnected,
    PresenceUpdated,
    FriendRequest,
    FriendAccepted,
    InviteReceived,
    InviteReplied,
    PunchResult,
    SessionFound,
    Datagram,
    /// Punch failed; traffic for `peer` rides the control tunnel (degraded).
    Degraded,
    Error,
    FriendRemoved,
    /// Ambient shard presence (§17.3.4). Append-only: the C header mirrors
    /// this ordering, so never insert above an existing variant.
    ShardPresence,
    /// A peer started/stopped relaying our traffic (§17.2); flag = is_relay.
    RelayStatus,
    /// Our own virtual address changed (§17.3.5); virtual_ip carries it.
    LocalAddress,
    /// Guest TCP connection opened (§18). virtual_ip = peer, port =
    /// local guest port, src_port = remote guest port, flag = 1 when we
    /// opened it. data = 8-byte big-endian stream id.
    StreamOpened,
    /// Bytes on a guest TCP connection. data = [stream id 8][payload].
    StreamData,
    /// Guest TCP connection ended. data = 8-byte stream id.
    StreamClosed,
    /// Outbound guest TCP connect failed. virtual_ip = peer, port = dst.
    StreamConnectFailed,
    /// Measured round trip to a peer. virtual_ip = peer, port = milliseconds.
    PeerRtt,
    /// A directly dialable public endpoint for us was confirmed (UPnP/AutoNAT).
    /// data = UTF-8 bare `host:port` to hand a peer for a direct connect.
    ExternalAddress,
}

/// Fixed-size POD event, drained once per frame via [`rexnet_poll_event`].
/// Variable payloads (presence records, session descriptors, datagrams,
/// error text) are carried in `data`/`data_len`, encoded per §8.
#[repr(C)]
pub struct RexNetEvent {
    pub kind: RexNetEventKind,
    pub peer: RexNetPeerId,
    /// PeerConnected: the peer's allocated 10.77.0.0/16 address.
    /// Datagram: the sender's virtual IP.
    pub virtual_ip: u32,
    /// Datagram: guest destination port.
    pub port: u16,
    /// Datagram: guest source port (for reply routing).
    pub src_port: u16,
    pub flag: u8, // PunchResult ok / InviteReplied reply code
    pub _pad: u8,
    pub data_len: u32,
    pub data: [u8; REXNET_EVENT_DATA_MAX],
}

pub struct RexNetHandle {
    _runtime: tokio::runtime::Runtime,
    cmd_tx: mpsc::UnboundedSender<Command>,
    /// Drained only by rexnet_poll_event; Mutex because the C side gives no
    /// single-thread guarantee.
    evt_rx: Mutex<mpsc::UnboundedReceiver<Event>>,
    pub peer_id: PeerId,
}

fn cstr_owned(ptr: *const c_char) -> String {
    if ptr.is_null() {
        return String::new();
    }
    unsafe { CStr::from_ptr(ptr) }.to_string_lossy().into_owned()
}

fn fill_event(out: &mut RexNetEvent, event: Event) {
    *out = RexNetEvent {
        kind: RexNetEventKind::None,
        peer: RexNetPeerId::default(),
        virtual_ip: 0,
        port: 0,
        src_port: 0,
        flag: 0,
        _pad: 0,
        data_len: 0,
        data: [0; REXNET_EVENT_DATA_MAX],
    };
    let set_data = |dst: &mut RexNetEvent, bytes: &[u8]| {
        let n = bytes.len().min(REXNET_EVENT_DATA_MAX);
        dst.data[..n].copy_from_slice(&bytes[..n]);
        dst.data_len = n as u32;
    };
    match event {
        Event::PeerConnected { peer, virtual_ip } => {
            out.kind = RexNetEventKind::PeerConnected;
            out.peer = RexNetPeerId::from_peer(&peer);
            out.virtual_ip = virtual_ip;
        }
        Event::StreamOpened { stream_id, virtual_ip, local_port, remote_port, outbound } => {
            out.kind = RexNetEventKind::StreamOpened;
            out.virtual_ip = virtual_ip;
            out.port = local_port;
            out.src_port = remote_port;
            out.flag = u8::from(outbound);
            set_data(out, &stream_id.to_be_bytes());
        }
        Event::StreamData { stream_id, data } => {
            out.kind = RexNetEventKind::StreamData;
            // Stream id prefixes the payload: the shim routes on it, and a
            // separate field would cap payloads at the fixed event struct.
            let mut packed = Vec::with_capacity(8 + data.len());
            packed.extend_from_slice(&stream_id.to_be_bytes());
            packed.extend_from_slice(&data);
            set_data(out, &packed);
        }
        Event::StreamClosed { stream_id } => {
            out.kind = RexNetEventKind::StreamClosed;
            set_data(out, &stream_id.to_be_bytes());
        }
        Event::StreamConnectFailed { virtual_ip, dst_port, message } => {
            out.kind = RexNetEventKind::StreamConnectFailed;
            out.virtual_ip = virtual_ip;
            out.port = dst_port;
            set_data(out, message.as_bytes());
        }
        Event::PeerRtt { virtual_ip, rtt_ms } => {
            out.kind = RexNetEventKind::PeerRtt;
            out.virtual_ip = virtual_ip;
            out.port = rtt_ms.min(u16::MAX as u32) as u16;
        }
        Event::LocalAddress { virtual_ip } => {
            out.kind = RexNetEventKind::LocalAddress;
            out.virtual_ip = virtual_ip;
        }
        Event::RelayStatus { peer, is_relay } => {
            out.kind = RexNetEventKind::RelayStatus;
            out.peer = RexNetPeerId::from_peer(&peer);
            out.flag = u8::from(is_relay);
        }
        Event::ShardPresence { peer, state, rich, session_id } => {
            out.kind = RexNetEventKind::ShardPresence;
            out.peer = RexNetPeerId::from_peer(&peer);
            out.flag = state;
            // data = [has_session u8][session_id 16 if set][rich]
            // No display name by design -- see ShardBeat (§17.3.8).
            let mut packed = Vec::with_capacity(1 + 16 + rich.len());
            match session_id {
                Some(id) => {
                    packed.push(1);
                    packed.extend_from_slice(&id);
                }
                None => packed.push(0),
            }
            packed.extend_from_slice(&rich);
            set_data(out, &packed);
        }
        Event::PeerDisconnected { peer } => {
            out.kind = RexNetEventKind::PeerDisconnected;
            out.peer = RexNetPeerId::from_peer(&peer);
        }
        Event::PresenceUpdated { peer, title_id, state, display_name, rich } => {
            out.kind = RexNetEventKind::PresenceUpdated;
            out.peer = RexNetPeerId::from_peer(&peer);
            out.virtual_ip = title_id; // repurposed: title id
            out.flag = state;
            // data = [name_len u8][name bytes][rich bytes]
            let name = display_name.as_bytes();
            let name_len = name.len().min(32);
            let mut packed = Vec::with_capacity(1 + name_len + rich.len());
            packed.push(name_len as u8);
            packed.extend_from_slice(&name[..name_len]);
            packed.extend_from_slice(&rich);
            set_data(out, &packed);
        }
        Event::FriendRequest { peer, display_name, note } => {
            out.kind = RexNetEventKind::FriendRequest;
            out.peer = RexNetPeerId::from_peer(&peer);
            // data = [name_len u8][name][note], mirroring PresenceUpdated.
            let name = display_name.as_bytes();
            let name_len = name.len().min(255);
            let mut data = Vec::with_capacity(1 + name_len + note.len());
            data.push(name_len as u8);
            data.extend_from_slice(&name[..name_len]);
            data.extend_from_slice(note.as_bytes());
            set_data(out, &data);
        }
        Event::FriendAccepted { peer } => {
            out.kind = RexNetEventKind::FriendAccepted;
            out.peer = RexNetPeerId::from_peer(&peer);
        }
        Event::FriendRemoved { peer } => {
            out.kind = RexNetEventKind::FriendRemoved;
            out.peer = RexNetPeerId::from_peer(&peer);
        }
        Event::InviteReceived { peer, title_id, session_id } => {
            out.kind = RexNetEventKind::InviteReceived;
            out.peer = RexNetPeerId::from_peer(&peer);
            out.virtual_ip = title_id; // repurposed: title id
            set_data(out, &session_id);
        }
        Event::InviteReplied { peer, reply } => {
            out.kind = RexNetEventKind::InviteReplied;
            out.peer = RexNetPeerId::from_peer(&peer);
            out.flag = reply;
        }
        Event::PunchResult { peer, ok } => {
            out.kind = RexNetEventKind::PunchResult;
            out.peer = RexNetPeerId::from_peer(&peer);
            out.flag = ok as u8;
        }
        Event::SessionFound { host, session_id, slots_total, slots_open, requires_invite } => {
            out.kind = RexNetEventKind::SessionFound;
            out.peer = RexNetPeerId::from_peer(&host);
            out.port = u16::from(slots_total);
            out.src_port = u16::from(slots_open);
            out.flag = requires_invite as u8;
            set_data(out, &session_id);
        }
        Event::Datagram { virtual_ip, src_port, dst_port, data } => {
            out.kind = RexNetEventKind::Datagram;
            out.virtual_ip = virtual_ip;
            out.port = dst_port;
            out.src_port = src_port;
            set_data(out, &data);
        }
        Event::Degraded { peer } => {
            out.kind = RexNetEventKind::Degraded;
            out.peer = RexNetPeerId::from_peer(&peer);
        }
        Event::Error { message } => {
            out.kind = RexNetEventKind::Error;
            set_data(out, message.as_bytes());
        }
        Event::ExternalAddress { addr } => {
            out.kind = RexNetEventKind::ExternalAddress;
            set_data(out, addr.as_bytes());
        }
    }
}

/// # Safety
/// `cfg` must point to a valid [`RexNetConfig`] with valid, NUL-terminated
/// strings. Returns null on failure.
#[no_mangle]
pub unsafe extern "C" fn rexnet_init(cfg: *const RexNetConfig) -> *mut RexNetHandle {
    catch_unwind(AssertUnwindSafe(|| {
        if cfg.is_null() {
            return std::ptr::null_mut();
        }
        let cfg = &*cfg;

        let data_dir = PathBuf::from(cstr_owned(cfg.data_dir));
        let display_name = cstr_owned(cfg.display_name);
        let mut bootstrap = Vec::new();
        if !cfg.bootstrap.is_null() {
            for i in 0..cfg.bootstrap_len as usize {
                bootstrap.push(cstr_owned(*cfg.bootstrap.add(i)));
            }
        }
        if cfg.use_default_bootstrap {
            bootstrap.extend(crate::AMINO_BOOTSTRAP.iter().map(|s| s.to_string()));
        }
        let mut relays = Vec::new();
        if !cfg.relays.is_null() {
            for i in 0..cfg.relays_len as usize {
                relays.push(cstr_owned(*cfg.relays.add(i)));
            }
        }

        let keypair = match identity::load_or_generate(&data_dir) {
            Ok(k) => k,
            Err(err) => {
                tracing::error!(%err, "rexnet identity load failed");
                return std::ptr::null_mut();
            }
        };

        let runtime = match tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .thread_name("rexnet")
            .enable_all()
            .build()
        {
            Ok(rt) => rt,
            Err(err) => {
                tracing::error!(%err, "rexnet runtime start failed");
                return std::ptr::null_mut();
            }
        };

        let handles = match runtime.block_on(engine::spawn(
            keypair,
            engine::EngineConfig {
                data_dir,
                title_id: cfg.title_id,
                display_name,
                bootstrap,
                listen_port: cfg.listen_port,
                game_port: cfg.game_port,
                relays,
                force_tunnel: cfg.force_tunnel,
            },
        )) {
            Ok(handles) => handles,
            Err(err) => {
                tracing::error!(%err, "rexnet engine start failed");
                return std::ptr::null_mut();
            }
        };

        Box::into_raw(Box::new(RexNetHandle {
            _runtime: runtime,
            cmd_tx: handles.cmd_tx,
            evt_rx: Mutex::new(handles.evt_rx),
            peer_id: handles.peer_id,
        }))
    }))
    .unwrap_or(std::ptr::null_mut())
}

/// # Safety
/// `handle` must come from [`rexnet_init`] and not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn rexnet_shutdown(handle: *mut RexNetHandle) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        if handle.is_null() {
            return;
        }
        let handle = Box::from_raw(handle);
        let _ = handle.cmd_tx.send(Command::Shutdown);
        // Dropping the runtime joins its worker threads.
    }));
}

unsafe fn with_handle(handle: *mut RexNetHandle, f: impl FnOnce(&RexNetHandle)) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        if let Some(handle) = handle.as_ref() {
            f(handle);
        }
    }));
}

/// # Safety
/// `handle` from [`rexnet_init`]; `out` must point to a [`RexNetEvent`].
/// Returns true while events remain; drain once per frame.
#[no_mangle]
pub unsafe extern "C" fn rexnet_poll_event(
    handle: *mut RexNetHandle,
    out: *mut RexNetEvent,
) -> bool {
    catch_unwind(AssertUnwindSafe(|| {
        let Some(handle) = handle.as_ref() else { return false };
        if out.is_null() {
            return false;
        }
        let Ok(mut rx) = handle.evt_rx.lock() else { return false };
        match rx.try_recv() {
            Ok(event) => {
                fill_event(&mut *out, event);
                true
            }
            Err(_) => false,
        }
    }))
    .unwrap_or(false)
}

/// Render a peer id as its canonical base58 string into `out` (NUL
/// terminated, truncated to `cap`). Returns the number of bytes written
/// (excluding NUL), or 0 on failure.
///
/// # Safety
/// `peer` points to a valid [`RexNetPeerId`]; `out` has room for `cap` bytes.
#[no_mangle]
pub unsafe extern "C" fn rexnet_peer_id_string(
    peer: *const RexNetPeerId,
    out: *mut c_char,
    cap: u32,
) -> u32 {
    catch_unwind(AssertUnwindSafe(|| {
        let Some(peer) = peer.as_ref().and_then(RexNetPeerId::to_peer) else { return 0 };
        if out.is_null() || cap == 0 {
            return 0;
        }
        let s = peer.to_base58();
        let bytes = s.as_bytes();
        let n = bytes.len().min(cap as usize - 1);
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), out as *mut u8, n);
        *out.add(n) = 0;
        n as u32
    }))
    .unwrap_or(0)
}

/// Parse a peer identity into `out`: accepts either the canonical base58
/// peer-id string or a `REXN-…` friend code (case-insensitive, dashes
/// optional). Returns false on malformed input (or a multihash exceeding
/// the fixed 63-byte capacity — never the case for standard ed25519 ids).
///
/// # Safety
/// `s` is a NUL-terminated string; `out` points to a [`RexNetPeerId`].
#[no_mangle]
pub unsafe extern "C" fn rexnet_peer_id_parse(
    s: *const c_char,
    out: *mut RexNetPeerId,
) -> bool {
    catch_unwind(AssertUnwindSafe(|| {
        if s.is_null() || out.is_null() {
            return false;
        }
        let text = CStr::from_ptr(s).to_string_lossy();
        let trimmed = text.trim();
        let peer = match trimmed.parse::<PeerId>() {
            Ok(peer) => peer,
            Err(_) => match identity::parse_friend_code(trimmed) {
                Some((peer, _name)) => peer,
                None => return false,
            },
        };
        if peer.to_bytes().len() > 63 {
            return false;
        }
        *out = RexNetPeerId::from_peer(&peer);
        true
    }))
    .unwrap_or(false)
}

/// Render a peer id (+ optional display name, NUL-terminated UTF-8, ≤24
/// bytes used) as a `REXN-…` friend code into `out` (NUL-terminated).
/// Returns bytes written (excluding NUL), or 0 if the peer id is not an
/// ed25519 identity (or `cap` is too small — codes are ≤119 chars + NUL).
///
/// # Safety
/// `peer` points to a valid [`RexNetPeerId`]; `name` is NULL or a
/// NUL-terminated string; `out` has room for `cap` bytes.
#[no_mangle]
pub unsafe extern "C" fn rexnet_friend_code(
    peer: *const RexNetPeerId,
    name: *const c_char,
    out: *mut c_char,
    cap: u32,
) -> u32 {
    catch_unwind(AssertUnwindSafe(|| {
        let Some(peer) = peer.as_ref().and_then(RexNetPeerId::to_peer) else { return 0 };
        let name = if name.is_null() {
            String::new()
        } else {
            CStr::from_ptr(name).to_string_lossy().into_owned()
        };
        let Some(code) = identity::render_friend_code(&peer, &name) else { return 0 };
        if out.is_null() || (cap as usize) < code.len() + 1 {
            return 0;
        }
        let bytes = code.as_bytes();
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), out as *mut u8, bytes.len());
        *out.add(bytes.len()) = 0;
        bytes.len() as u32
    }))
    .unwrap_or(0)
}

/// Parse a friend code, returning the peer id and the embedded display
/// name ("" for nameless codes) — `name_out` may be NULL to skip the name.
/// Returns false on malformed input (checksum mismatches included).
///
/// # Safety
/// `s` is NUL-terminated; `out_peer` points to a [`RexNetPeerId`];
/// `name_out` is NULL or has room for `name_cap` bytes.
#[no_mangle]
pub unsafe extern "C" fn rexnet_friend_code_parse(
    s: *const c_char,
    out_peer: *mut RexNetPeerId,
    name_out: *mut c_char,
    name_cap: u32,
) -> bool {
    catch_unwind(AssertUnwindSafe(|| {
        if s.is_null() || out_peer.is_null() {
            return false;
        }
        let text = CStr::from_ptr(s).to_string_lossy();
        let Some((peer, name)) = identity::parse_friend_code(text.trim()) else {
            return false;
        };
        if peer.to_bytes().len() > 63 {
            return false;
        }
        *out_peer = RexNetPeerId::from_peer(&peer);
        if !name_out.is_null() && name_cap > 0 {
            let bytes = name.as_bytes();
            let n = bytes.len().min(name_cap as usize - 1);
            std::ptr::copy_nonoverlapping(bytes.as_ptr(), name_out as *mut u8, n);
            *name_out.add(n) = 0;
        }
        true
    }))
    .unwrap_or(false)
}

/// Change the self-asserted display name at runtime; presence is re-pushed
/// to friends immediately. The name is sanitized (control chars stripped,
/// truncated to 24 bytes on a char boundary).
///
/// # Safety
/// `handle` from [`rexnet_init`]; `name` is a NUL-terminated string.
#[no_mangle]
pub unsafe extern "C" fn rexnet_set_display_name(
    handle: *mut RexNetHandle,
    name: *const c_char,
) {
    let name = cstr_owned(name);
    with_handle(handle, |h| {
        let _ = h.cmd_tx.send(Command::SetDisplayName { name: name.clone() });
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `out` must point to a [`RexNetPeerId`].
#[no_mangle]
pub unsafe extern "C" fn rexnet_local_peer_id(
    handle: *mut RexNetHandle,
    out: *mut RexNetPeerId,
) {
    with_handle(handle, |h| {
        if !out.is_null() {
            *out = RexNetPeerId::from_peer(&h.peer_id);
        }
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `presence`/`presence_len` describe a
/// postcard-encoded presence payload (§8.1) owned by the caller.
#[no_mangle]
pub unsafe extern "C" fn rexnet_set_presence(
    handle: *mut RexNetHandle,
    title_id: u32,
    state: u8,
    presence: *const u8,
    presence_len: u32,
) {
    with_handle(handle, |h| {
        let rich = slice_owned(presence, presence_len);
        let _ = h.cmd_tx.send(Command::SetPresence { title_id, state, rich });
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `peer` must point to a valid [`RexNetPeerId`].
/// Fire-and-forget game-plane punch (§8.4); the outcome arrives as a
/// `PunchResult` event.
#[no_mangle]
pub unsafe extern "C" fn rexnet_punch_peer(
    handle: *mut RexNetHandle,
    peer: *const RexNetPeerId,
) {
    with_handle(handle, |h| {
        if let Some(peer) = peer.as_ref().and_then(RexNetPeerId::to_peer) {
            let _ = h.cmd_tx.send(Command::Punch { peer, reply: None });
        }
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `peer` must point to a valid [`RexNetPeerId`].
#[no_mangle]
pub unsafe extern "C" fn rexnet_connect_peer(
    handle: *mut RexNetHandle,
    peer: *const RexNetPeerId,
) {
    with_handle(handle, |h| {
        if let Some(peer) = peer.as_ref().and_then(RexNetPeerId::to_peer) {
            let _ = h.cmd_tx.send(Command::ConnectPeer { peer });
        }
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `multiaddr` is a NUL-terminated string
/// (manual connect string, the preservation floor — §4).
#[no_mangle]
pub unsafe extern "C" fn rexnet_connect_manual(
    handle: *mut RexNetHandle,
    multiaddr: *const c_char,
) {
    with_handle(handle, |h| {
        let _ = h.cmd_tx.send(Command::ConnectManual { multiaddr: cstr_owned(multiaddr) });
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `peer` valid. Invites `peer` to the
/// current local session (§8.3); the reply arrives as an InviteReplied
/// event, and an accepting invitee fetches our session descriptor.
#[no_mangle]
pub unsafe extern "C" fn rexnet_send_invite(
    handle: *mut RexNetHandle,
    peer: *const RexNetPeerId,
) {
    with_handle(handle, |h| {
        if let Some(peer) = peer.as_ref().and_then(RexNetPeerId::to_peer) {
            let _ = h.cmd_tx.send(Command::SendInvite { peer });
        }
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `peer` valid. Accepting also fetches the
/// host's session descriptor (arrives as a SessionFound event).
#[no_mangle]
pub unsafe extern "C" fn rexnet_invite_reply(
    handle: *mut RexNetHandle,
    peer: *const RexNetPeerId,
    accept: bool,
) {
    with_handle(handle, |h| {
        if let Some(peer) = peer.as_ref().and_then(RexNetPeerId::to_peer) {
            let _ = h.cmd_tx.send(Command::InviteReply { peer, accept });
        }
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `peer` valid; `note` NUL-terminated.
#[no_mangle]
pub unsafe extern "C" fn rexnet_friend_request(
    handle: *mut RexNetHandle,
    peer: *const RexNetPeerId,
    note: *const c_char,
) {
    with_handle(handle, |h| {
        if let Some(peer) = peer.as_ref().and_then(RexNetPeerId::to_peer) {
            let _ = h.cmd_tx.send(Command::FriendRequest { peer, note: cstr_owned(note) });
        }
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `peer` valid.
#[no_mangle]
pub unsafe extern "C" fn rexnet_friend_accept(
    handle: *mut RexNetHandle,
    peer: *const RexNetPeerId,
) {
    with_handle(handle, |h| {
        if let Some(peer) = peer.as_ref().and_then(RexNetPeerId::to_peer) {
            let _ = h.cmd_tx.send(Command::FriendAccept { peer });
        }
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `peer` valid.
#[no_mangle]
pub unsafe extern "C" fn rexnet_friend_remove(
    handle: *mut RexNetHandle,
    peer: *const RexNetPeerId,
) {
    with_handle(handle, |h| {
        if let Some(peer) = peer.as_ref().and_then(RexNetPeerId::to_peer) {
            let _ = h.cmd_tx.send(Command::FriendRemove { peer });
        }
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `session_id` must point to 16 bytes
/// (allocated by the shim; surfaced to the guest as XNKID+XNKEY).
#[no_mangle]
pub unsafe extern "C" fn rexnet_session_create(
    handle: *mut RexNetHandle,
    session_id: *const u8,
    slots_total: u8,
    slots_open: u8,
    is_public: bool,
) {
    with_handle(handle, |h| {
        if session_id.is_null() {
            return;
        }
        let mut id = [0u8; 16];
        id.copy_from_slice(std::slice::from_raw_parts(session_id, 16));
        let _ = h.cmd_tx.send(Command::SessionCreate { id, slots_total, slots_open, is_public });
    });
}

/// # Safety
/// `handle` from [`rexnet_init`].
#[no_mangle]
pub unsafe extern "C" fn rexnet_session_delete(handle: *mut RexNetHandle) {
    with_handle(handle, |h| {
        let _ = h.cmd_tx.send(Command::SessionDelete);
    });
}

/// Guest TCP connect to a peer's virtual IP (§18). Result arrives as a
/// StreamOpened or StreamConnectFailed event.
///
/// # Safety
/// `handle` from [`rexnet_init`].
#[no_mangle]
pub unsafe extern "C" fn rexnet_stream_connect(
    handle: *mut RexNetHandle,
    virtual_ip: u32,
    src_port: u16,
    dst_port: u16,
) {
    with_handle(handle, |h| {
        let _ = h.cmd_tx.send(Command::StreamConnect { virtual_ip, src_port, dst_port });
    });
}

/// Write to a guest TCP connection.
///
/// # Safety
/// `handle` from [`rexnet_init`]; `data` must be readable for `len` bytes.
#[no_mangle]
pub unsafe extern "C" fn rexnet_stream_send(
    handle: *mut RexNetHandle,
    stream_id: u64,
    data: *const u8,
    len: u32,
) {
    if data.is_null() || len == 0 {
        return;
    }
    let bytes = std::slice::from_raw_parts(data, len as usize).to_vec();
    with_handle(handle, |h| {
        let _ = h.cmd_tx.send(Command::StreamSend { stream_id, data: bytes.clone() });
    });
}

/// Close a guest TCP connection.
///
/// # Safety
/// `handle` from [`rexnet_init`].
#[no_mangle]
pub unsafe extern "C" fn rexnet_stream_close(handle: *mut RexNetHandle, stream_id: u64) {
    with_handle(handle, |h| {
        let _ = h.cmd_tx.send(Command::StreamClose { stream_id });
    });
}

/// Opt in to the ambient title shard (§17.3). Idempotent; `cap` of 0 takes
/// the spec default of 255. Titles opt in via `rexnet.toml` (§17.5) -- this
/// is never implied, so a game that has no use for ambient presence pays no
/// traffic for it.
///
/// # Safety
/// `handle` from [`rexnet_init`].
#[no_mangle]
pub unsafe extern "C" fn rexnet_shard_enable(handle: *mut RexNetHandle, cap: u16) {
    with_handle(handle, |h| {
        let _ = h.cmd_tx.send(Command::ShardEnable { cap });
    });
}

/// Leave any shard and stop publishing/scanning.
///
/// # Safety
/// `handle` from [`rexnet_init`].
#[no_mangle]
pub unsafe extern "C" fn rexnet_shard_disable(handle: *mut RexNetHandle) {
    with_handle(handle, |h| {
        let _ = h.cmd_tx.send(Command::ShardDisable);
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]. Results arrive as SessionFound events
/// (peer = host, data = 16-byte session id, port/src_port = total/open
/// slots, flag = requires_invite). Filtering on game-defined attributes is
/// applied client-side by the shim per the game config.
#[no_mangle]
pub unsafe extern "C" fn rexnet_session_search(handle: *mut RexNetHandle) {
    with_handle(handle, |h| {
        let _ = h.cmd_tx.send(Command::SessionSearch);
    });
}

/// # Safety
/// `handle` from [`rexnet_init`]; `data`/`len` owned by the caller.
///
/// `src_port`/`dst_port` are the guest ports (the game socket multiplexes
/// all of them). `reliable` exists for module-internal traffic only. Guest
/// datagrams MUST pass false when the game config sets
/// `game_channel = "unreliable"` (own-reliability titles like XRNM — §6).
#[no_mangle]
pub unsafe extern "C" fn rexnet_send_datagram(
    handle: *mut RexNetHandle,
    virtual_ip: u32,
    src_port: u16,
    dst_port: u16,
    data: *const u8,
    len: u32,
    reliable: bool,
) {
    with_handle(handle, |h| {
        let data = slice_owned(data, len);
        let _ = h.cmd_tx.send(Command::SendDatagram {
            virtual_ip,
            src_port,
            dst_port,
            data,
            reliable,
        });
    });
}

unsafe fn slice_owned(ptr: *const u8, len: u32) -> Vec<u8> {
    if ptr.is_null() || len == 0 {
        Vec::new()
    } else {
        std::slice::from_raw_parts(ptr, len as usize).to_vec()
    }
}
