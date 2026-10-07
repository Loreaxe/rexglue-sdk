// @file        rexnet/core/src/friends.rs
// @brief       Local friend list (design spec §8.2).
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Local friend list (design spec §8.2).
//!
//! Friendship is mutual-consent: a request/accept exchange is required
//! before presence flows in either direction, enforced at the protocol
//! layer (engine), not UI. The list is stored locally only — one base58
//! PeerId per line in `friends.txt`. Inbound pending requests are in-memory.
//!
//! Pending requests persist too, both ways. OUTBOUND in `friends_pending.txt`:
//! an Accept only counts while its peer is in `pending_out`, and the accept
//! usually arrives in a later session than the request -- with the set in
//! memory the acceptor saved the friendship and the requester threw the
//! Accept away, leaving a one-sided friend list. INBOUND in
//! `friends_inbound.txt` (peer, display name, note): a request shown in one
//! session is still waiting in the next, so a restart no longer loses it.

use std::collections::{HashMap, HashSet, VecDeque};
use std::path::{Path, PathBuf};
use std::str::FromStr;

use libp2p::PeerId;

use crate::identity::sanitize_display_name;

/// Inbound requests held at once. A stranger can send one per peer id for
/// free, so the set is bounded; past this the oldest unanswered one goes.
pub const PENDING_IN_MAX: usize = 64;

/// Longest note kept from an inbound request, in bytes.
pub const NOTE_MAX: usize = 64;

/// Text a stranger sent, made safe to show and to store: control characters
/// (newlines included -- the store is line-based) stripped, cut at
/// `max_bytes` on a character boundary, trimmed.
pub fn sanitize_text(text: &str, max_bytes: usize) -> String {
    let mut out = String::with_capacity(text.len().min(max_bytes));
    for ch in text.chars() {
        if ch.is_control() {
            continue;
        }
        if out.len() + ch.len_utf8() > max_bytes {
            break;
        }
        out.push(ch);
    }
    out.trim().to_string()
}

pub struct FriendStore {
    path: PathBuf,
    pending_path: PathBuf,
    inbound_path: PathBuf,
    friends: HashSet<PeerId>,
    pub pending_in: HashSet<PeerId>,
    pub pending_out: HashSet<PeerId>,
    /// Display name and note of each inbound request, for re-showing it.
    inbound: HashMap<PeerId, (String, String)>,
    /// Arrival order of `inbound`, oldest first, so the cap evicts the oldest.
    inbound_order: VecDeque<PeerId>,
}

fn read_inbound(path: &Path) -> (HashMap<PeerId, (String, String)>, VecDeque<PeerId>) {
    let mut inbound = HashMap::new();
    let mut order = VecDeque::new();
    if let Ok(text) = std::fs::read_to_string(path) {
        for line in text.lines() {
            let line = line.trim_end();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let mut parts = line.splitn(3, '\t');
            let peer = parts.next().unwrap_or("");
            let name = sanitize_display_name(parts.next().unwrap_or(""));
            let note = sanitize_text(parts.next().unwrap_or(""), NOTE_MAX);
            match PeerId::from_str(peer) {
                Ok(peer) => {
                    if inbound.insert(peer, (name, note)).is_none() {
                        order.push_back(peer);
                    }
                }
                Err(err) => tracing::warn!(line, %err, "bad friends_inbound.txt entry skipped"),
            }
        }
    }
    while order.len() > PENDING_IN_MAX {
        if let Some(oldest) = order.pop_front() {
            inbound.remove(&oldest);
        }
    }
    (inbound, order)
}

fn read_peers(path: &Path, what: &str) -> HashSet<PeerId> {
    let mut peers = HashSet::new();
    if let Ok(text) = std::fs::read_to_string(path) {
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            match PeerId::from_str(line) {
                Ok(peer) => {
                    peers.insert(peer);
                }
                Err(err) => tracing::warn!(line, %err, "bad {what} entry skipped"),
            }
        }
    }
    peers
}

fn write_peers(path: &Path, header: &str, peers: &HashSet<PeerId>, what: &str) {
    let mut text = String::from(header);
    for peer in peers {
        text.push_str(&peer.to_base58());
        text.push('\n');
    }
    if let Some(parent) = path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    if let Err(err) = std::fs::write(path, text) {
        tracing::warn!(%err, "failed to persist {what}");
    }
}

impl FriendStore {
    pub fn load(data_dir: &Path) -> Self {
        let path = data_dir.join("friends.txt");
        let pending_path = data_dir.join("friends_pending.txt");
        let inbound_path = data_dir.join("friends_inbound.txt");
        let friends = read_peers(&path, "friends.txt");
        let mut pending_out = read_peers(&pending_path, "friends_pending.txt");
        pending_out.retain(|peer| !friends.contains(peer));
        let (mut inbound, mut inbound_order) = read_inbound(&inbound_path);
        inbound.retain(|peer, _| !friends.contains(peer));
        inbound_order.retain(|peer| inbound.contains_key(peer));
        let pending_in = inbound.keys().copied().collect();
        Self {
            path,
            pending_path,
            inbound_path,
            friends,
            pending_in,
            pending_out,
            inbound,
            inbound_order,
        }
    }

    fn save_inbound(&self) {
        let mut text =
            String::from("# rexnet friend requests received and not yet answered — peer id, name, note\n");
        for peer in &self.inbound_order {
            let Some((name, note)) = self.inbound.get(peer) else { continue };
            text.push_str(&peer.to_base58());
            text.push('\t');
            text.push_str(&name.replace(['\t', '\n', '\r'], " "));
            text.push('\t');
            text.push_str(&note.replace(['\t', '\n', '\r'], " "));
            text.push('\n');
        }
        if let Some(parent) = self.inbound_path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        if let Err(err) = std::fs::write(&self.inbound_path, text) {
            tracing::warn!(%err, "failed to persist inbound friend requests");
        }
    }

    /// Drop the inbound request from `peer`. True when there was one; the
    /// file is rewritten only then.
    fn drop_inbound(&mut self, peer: &PeerId) -> bool {
        let had = self.pending_in.remove(peer);
        if self.inbound.remove(peer).is_some() {
            self.inbound_order.retain(|p| p != peer);
            self.save_inbound();
        }
        had
    }

    /// A request came in. True when it is new, i.e. worth showing. The text
    /// is a stranger's: it is sanitised and capped here, before it is stored
    /// or shown. A repeat of a request already held changes nothing and
    /// touches no file -- the sender re-sends after every restart.
    pub fn request_received(&mut self, peer: PeerId, display_name: &str, note: &str) -> bool {
        if !self.pending_in.insert(peer) {
            return false;
        }
        let display_name = sanitize_display_name(display_name);
        let note = sanitize_text(note, NOTE_MAX);
        while self.inbound_order.len() >= PENDING_IN_MAX {
            match self.inbound_order.pop_front() {
                Some(oldest) => {
                    self.pending_in.remove(&oldest);
                    self.inbound.remove(&oldest);
                    tracing::info!(peer = %oldest, "oldest unanswered friend request dropped: too many held");
                }
                None => break,
            }
        }
        self.inbound.insert(peer, (display_name, note));
        self.inbound_order.push_back(peer);
        self.save_inbound();
        true
    }

    /// The display name and note held for an inbound request, as sanitised.
    pub fn inbound_text(&self, peer: &PeerId) -> Option<(String, String)> {
        self.inbound.get(peer).cloned()
    }

    /// The inbound requests still waiting, oldest first, for re-showing
    /// after a restart.
    pub fn inbound(&self) -> impl Iterator<Item = (PeerId, String, String)> + '_ {
        self.inbound_order.iter().filter_map(|peer| {
            self.inbound.get(peer).map(|(name, note)| (*peer, name.clone(), note.clone()))
        })
    }

    /// The player dismissed a request. True when there was one to dismiss, which
    /// is when the requester is told.
    pub fn dismiss(&mut self, peer: &PeerId) -> bool {
        self.drop_inbound(peer)
    }

    /// The other side declined a request we sent. True when we were waiting on it.
    pub fn request_declined(&mut self, peer: &PeerId) -> bool {
        if self.pending_out.remove(peer) {
            self.save_pending();
            return true;
        }
        false
    }

    fn save(&self) {
        write_peers(
            &self.path,
            "# rexnet friend list — one peer id per line\n",
            &self.friends,
            "friend list",
        );
    }

    fn save_pending(&self) {
        write_peers(
            &self.pending_path,
            "# rexnet friend requests sent and not yet answered — one peer id per line\n",
            &self.pending_out,
            "pending friend requests",
        );
    }

    /// A request went out (or is queued to go out): remembered across runs.
    pub fn request_sent(&mut self, peer: PeerId) {
        if self.pending_out.insert(peer) {
            self.save_pending();
        }
    }

    pub fn is_friend(&self, peer: &PeerId) -> bool {
        self.friends.contains(peer)
    }

    pub fn friends(&self) -> impl Iterator<Item = &PeerId> {
        self.friends.iter()
    }

    pub fn add(&mut self, peer: PeerId) {
        self.drop_inbound(&peer);
        if self.pending_out.remove(&peer) {
            self.save_pending();
        }
        if self.friends.insert(peer) {
            self.save();
        }
    }

    pub fn remove(&mut self, peer: &PeerId) {
        self.drop_inbound(peer);
        if self.pending_out.remove(peer) {
            self.save_pending();
        }
        if self.friends.remove(peer) {
            self.save();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sanitize_strips_control_and_caps_on_a_char_boundary() {
        assert_eq!(sanitize_text("hi\nthere\t\r\u{7}!", 64), "hithere!");
        // 'é' is two bytes; a cap of 3 keeps "aé" and not a torn char.
        assert_eq!(sanitize_text("aéb", 3), "aé");
        assert_eq!(sanitize_text("   padded   ", 64), "padded");
    }

    #[test]
    fn inbound_requests_are_capped_oldest_first() {
        let dir = std::env::temp_dir().join(format!("rexnet-friends-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut store = FriendStore::load(&dir);
        let peers: Vec<PeerId> = (0..PENDING_IN_MAX + 3).map(|_| PeerId::random()).collect();
        for peer in &peers {
            assert!(store.request_received(*peer, "name\n", "note"));
        }
        assert_eq!(store.pending_in.len(), PENDING_IN_MAX);
        for old in &peers[..3] {
            assert!(!store.pending_in.contains(old), "oldest request survived the cap");
        }
        assert!(store.pending_in.contains(peers.last().unwrap()));
        assert_eq!(store.inbound_text(peers.last().unwrap()).unwrap().0, "name");
        // A repeat is not new.
        assert!(!store.request_received(*peers.last().unwrap(), "x", "y"));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
