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

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::str::FromStr;

use libp2p::PeerId;

pub struct FriendStore {
    path: PathBuf,
    pending_path: PathBuf,
    inbound_path: PathBuf,
    friends: HashSet<PeerId>,
    pub pending_in: HashSet<PeerId>,
    pub pending_out: HashSet<PeerId>,
    /// Display name and note of each inbound request, for re-showing it.
    inbound: HashMap<PeerId, (String, String)>,
}

fn read_inbound(path: &Path) -> HashMap<PeerId, (String, String)> {
    let mut inbound = HashMap::new();
    if let Ok(text) = std::fs::read_to_string(path) {
        for line in text.lines() {
            let line = line.trim_end();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let mut parts = line.splitn(3, '\t');
            let peer = parts.next().unwrap_or("");
            let name = parts.next().unwrap_or("").to_string();
            let note = parts.next().unwrap_or("").to_string();
            match PeerId::from_str(peer) {
                Ok(peer) => {
                    inbound.insert(peer, (name, note));
                }
                Err(err) => tracing::warn!(line, %err, "bad friends_inbound.txt entry skipped"),
            }
        }
    }
    inbound
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
        let mut inbound = read_inbound(&inbound_path);
        inbound.retain(|peer, _| !friends.contains(peer));
        let pending_in = inbound.keys().copied().collect();
        Self { path, pending_path, inbound_path, friends, pending_in, pending_out, inbound }
    }

    fn save_inbound(&self) {
        let mut text =
            String::from("# rexnet friend requests received and not yet answered — peer id, name, note\n");
        for (peer, (name, note)) in &self.inbound {
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

    /// A request came in. True when it is new, i.e. worth showing.
    pub fn request_received(&mut self, peer: PeerId, display_name: &str, note: &str) -> bool {
        let new = self.pending_in.insert(peer);
        if new {
            self.inbound.insert(peer, (display_name.to_string(), note.to_string()));
            self.save_inbound();
        }
        new
    }

    /// The inbound requests still waiting, for re-showing after a restart.
    pub fn inbound(&self) -> impl Iterator<Item = (PeerId, String, String)> + '_ {
        self.inbound.iter().map(|(peer, (name, note))| (*peer, name.clone(), note.clone()))
    }

    /// The player dismissed a request. True when there was one to dismiss, which
    /// is when the requester is told.
    pub fn dismiss(&mut self, peer: &PeerId) -> bool {
        let had = self.pending_in.remove(peer);
        if self.inbound.remove(peer).is_some() {
            self.save_inbound();
        }
        had
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
        self.pending_in.remove(&peer);
        if self.inbound.remove(&peer).is_some() {
            self.save_inbound();
        }
        if self.pending_out.remove(&peer) {
            self.save_pending();
        }
        if self.friends.insert(peer) {
            self.save();
        }
    }

    pub fn remove(&mut self, peer: &PeerId) {
        self.pending_in.remove(peer);
        if self.inbound.remove(peer).is_some() {
            self.save_inbound();
        }
        if self.pending_out.remove(peer) {
            self.save_pending();
        }
        if self.friends.remove(peer) {
            self.save();
        }
    }
}
