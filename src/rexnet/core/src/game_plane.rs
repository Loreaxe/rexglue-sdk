// @file        rexnet/core/src/game_plane.rs
// @brief       Game-plane hot path, shared off the swarm loop.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! The encrypted UDP hot path (spec §6), factored out of the engine so game
//! traffic never queues behind DHT, gossip or identify work.
//!
//! The engine still owns negotiation: it punches endpoints, agrees keys and
//! allocates virtual IPs, and records the results here. Once a peer is
//! *ready* (endpoint mapped and keys installed) the traffic itself bypasses
//! the engine loop in both directions: the UDP reader task decrypts inbound
//! frames and emits them straight to the shim, and the FFI send call seals
//! and transmits on the caller's thread. Anything not ready — a peer still
//! punching, a tunnelled peer, a broadcast, a probe — falls through to the
//! engine exactly as before.
//!
//! Every table here is mutated only by the engine; the fast paths take the
//! lock briefly to look up and to advance the per-peer nonce counter or
//! replay window.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex, MutexGuard};

use libp2p::PeerId;
use tokio::net::UdpSocket;
use tokio::sync::{mpsc, Notify};

use crate::crypto::{self, SessionKeys};
use crate::engine::{Event, FRAME_DATA};

/// Event sink shared by the engine and the fast paths. Every emit pings the
/// notify so a blocked `rexnet_wait_event` wakes at once instead of at the
/// next pump tick.
#[derive(Clone)]
pub struct Emitter {
    tx: mpsc::UnboundedSender<Event>,
    notify: Arc<Notify>,
}

impl Emitter {
    pub fn new(tx: mpsc::UnboundedSender<Event>, notify: Arc<Notify>) -> Self {
        Self { tx, notify }
    }

    pub fn emit(&self, event: Event) {
        let _ = self.tx.send(event);
        self.notify.notify_one();
    }
}

#[derive(Default)]
struct Tables {
    keys: HashMap<PeerId, SessionKeys>,
    endpoint_by_peer: HashMap<PeerId, SocketAddr>,
    peer_by_endpoint: HashMap<SocketAddr, PeerId>,
    vip_by_peer: HashMap<PeerId, u32>,
    peer_by_vip: HashMap<u32, PeerId>,
}

pub struct GamePlane {
    socket: Arc<UdpSocket>,
    emitter: Emitter,
    tables: Mutex<Tables>,
}

impl GamePlane {
    pub fn new(socket: Arc<UdpSocket>, emitter: Emitter) -> Self {
        Self {
            socket,
            emitter,
            tables: Mutex::new(Tables::default()),
        }
    }

    fn lock(&self) -> MutexGuard<'_, Tables> {
        // A panic while holding the lock leaves the tables consistent (every
        // mutation is a single insert/remove), so poisoning carries no signal.
        self.tables.lock().unwrap_or_else(|e| e.into_inner())
    }

    // --- Engine-side bookkeeping ------------------------------------------

    pub fn map_endpoint(&self, peer: PeerId, addr: SocketAddr) {
        let mut t = self.lock();
        t.endpoint_by_peer.insert(peer, addr);
        t.peer_by_endpoint.insert(addr, peer);
    }

    pub fn unmap_endpoint(&self, peer: &PeerId) -> Option<SocketAddr> {
        let mut t = self.lock();
        let addr = t.endpoint_by_peer.remove(peer)?;
        t.peer_by_endpoint.remove(&addr);
        Some(addr)
    }

    pub fn endpoint(&self, peer: &PeerId) -> Option<SocketAddr> {
        self.lock().endpoint_by_peer.get(peer).copied()
    }

    pub fn peer_at(&self, addr: &SocketAddr) -> Option<PeerId> {
        self.lock().peer_by_endpoint.get(addr).copied()
    }

    /// False if the peer already had keys; an established session is left
    /// alone so its counter and replay window are never rewound.
    pub fn install_keys(&self, peer: PeerId, keys: SessionKeys) -> bool {
        let mut t = self.lock();
        if t.keys.contains_key(&peer) {
            return false;
        }
        t.keys.insert(peer, keys);
        true
    }

    pub fn remove_keys(&self, peer: &PeerId) {
        self.lock().keys.remove(peer);
    }

    pub fn has_keys(&self, peer: &PeerId) -> bool {
        self.lock().keys.contains_key(peer)
    }

    /// Endpoint mapped and keys installed: traffic can flow both ways.
    pub fn ready(&self, peer: &PeerId) -> bool {
        let t = self.lock();
        t.endpoint_by_peer.contains_key(peer) && t.keys.contains_key(peer)
    }

    pub fn set_vip(&self, peer: PeerId, vip: u32) {
        let mut t = self.lock();
        if let Some(old) = t.vip_by_peer.insert(peer, vip) {
            if old != vip {
                t.peer_by_vip.remove(&old);
            }
        }
        t.peer_by_vip.insert(vip, peer);
    }

    pub fn clear_vip(&self, peer: &PeerId) {
        let mut t = self.lock();
        if let Some(vip) = t.vip_by_peer.remove(peer) {
            t.peer_by_vip.remove(&vip);
        }
    }

    // --- Sealing, used by the engine for queued and broadcast traffic ------

    /// `None` when there are no keys — the caller drops it. No plaintext
    /// fallback: that would be a downgrade an attacker could force.
    pub fn seal(&self, peer: PeerId, src_port: u16, dst_port: u16, data: &[u8]) -> Option<Vec<u8>> {
        let mut t = self.lock();
        let Some(keys) = t.keys.get_mut(&peer) else {
            tracing::warn!(%peer, "no game-plane keys; datagram dropped rather than sent in clear");
            return None;
        };
        seal_with(keys, peer, src_port, dst_port, data)
    }

    /// Authenticate and decrypt an inbound game datagram.
    pub fn open(&self, peer: PeerId, frame: &[u8]) -> Option<(u16, u16, Vec<u8>)> {
        let mut t = self.lock();
        let keys = t.keys.get_mut(&peer)?;
        open_with(keys, peer, frame)
    }

    // --- Fast paths --------------------------------------------------------

    /// Seal and transmit on the caller's thread if the peer is ready. False
    /// means the caller must hand the datagram to the engine: unknown or
    /// broadcast address, punch still in flight, or a tunnelled peer.
    pub fn try_send(&self, virtual_ip: u32, src_port: u16, dst_port: u16, data: &[u8]) -> bool {
        let mut t = self.lock();
        let Some(peer) = t.peer_by_vip.get(&virtual_ip).copied() else {
            return false;
        };
        let Some(endpoint) = t.endpoint_by_peer.get(&peer).copied() else {
            return false;
        };
        let Some(keys) = t.keys.get_mut(&peer) else {
            return false;
        };
        if let Some(frame) = seal_with(keys, peer, src_port, dst_port, data) {
            // Non-blocking on purpose: the guest thinks this is UDP, and a
            // full socket buffer should drop rather than stall a game thread.
            let _ = self.socket.try_send_to(&frame, endpoint);
        }
        true
    }

    /// Decrypt and deliver an inbound frame if its source is a ready peer.
    /// False means the engine must take it: a probe, or a data frame from an
    /// endpoint that has not finished punching (held there, not dropped).
    pub fn try_receive(&self, addr: SocketAddr, frame: &[u8]) -> bool {
        if frame.first() != Some(&FRAME_DATA) || frame.len() < 1 + crypto::CRYPTO_OVERHEAD {
            return false;
        }
        let event = {
            let mut t = self.lock();
            let Some(peer) = t.peer_by_endpoint.get(&addr).copied() else {
                return false;
            };
            let Some(vip) = t.vip_by_peer.get(&peer).copied() else {
                return false;
            };
            let Some(keys) = t.keys.get_mut(&peer) else {
                return false;
            };
            // Authentic-but-rejected frames (replays, short records) are
            // consumed here: the engine could do nothing more with them.
            open_with(keys, peer, frame).map(|(src_port, dst_port, data)| Event::Datagram {
                virtual_ip: vip,
                src_port,
                dst_port,
                data,
            })
        };
        if let Some(event) = event {
            self.emitter.emit(event);
        }
        true
    }
}

fn seal_with(
    keys: &mut SessionKeys,
    peer: PeerId,
    src_port: u16,
    dst_port: u16,
    data: &[u8],
) -> Option<Vec<u8>> {
    let mut plaintext = Vec::with_capacity(4 + data.len());
    plaintext.extend_from_slice(&src_port.to_be_bytes());
    plaintext.extend_from_slice(&dst_port.to_be_bytes());
    plaintext.extend_from_slice(data);
    match keys.sealer.seal(&[FRAME_DATA], &plaintext) {
        Ok(record) => {
            let mut frame = Vec::with_capacity(1 + record.len());
            frame.push(FRAME_DATA);
            frame.extend_from_slice(&record);
            Some(frame)
        }
        Err(err) => {
            tracing::warn!(%peer, ?err, "sealing a game datagram failed; dropped");
            None
        }
    }
}

fn open_with(keys: &mut SessionKeys, peer: PeerId, frame: &[u8]) -> Option<(u16, u16, Vec<u8>)> {
    match keys.opener.open(&[FRAME_DATA], &frame[1..]) {
        Ok(plain) if plain.len() >= 4 => Some((
            u16::from_be_bytes([plain[0], plain[1]]),
            u16::from_be_bytes([plain[2], plain[3]]),
            plain[4..].to_vec(),
        )),
        Ok(_) => {
            tracing::debug!(%peer, "authentic game datagram was too short to carry ports");
            None
        }
        Err(crypto::CryptoError::Replay) => {
            // Expected with retransmitting titles; not an attack signal.
            tracing::trace!(%peer, "replayed game datagram dropped");
            None
        }
        Err(err) => {
            tracing::debug!(%peer, ?err, "game datagram failed authentication");
            None
        }
    }
}
