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
//! PeerId per line in `friends.txt`. Pending requests are in-memory.

use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::str::FromStr;

use libp2p::PeerId;

pub struct FriendStore {
    path: PathBuf,
    friends: HashSet<PeerId>,
    pub pending_in: HashSet<PeerId>,
    pub pending_out: HashSet<PeerId>,
}

impl FriendStore {
    pub fn load(data_dir: &Path) -> Self {
        let path = data_dir.join("friends.txt");
        let mut friends = HashSet::new();
        if let Ok(text) = std::fs::read_to_string(&path) {
            for line in text.lines() {
                let line = line.trim();
                if line.is_empty() || line.starts_with('#') {
                    continue;
                }
                match PeerId::from_str(line) {
                    Ok(peer) => {
                        friends.insert(peer);
                    }
                    Err(err) => tracing::warn!(line, %err, "bad friends.txt entry skipped"),
                }
            }
        }
        Self { path, friends, pending_in: HashSet::new(), pending_out: HashSet::new() }
    }

    fn save(&self) {
        let mut text = String::from("# rexnet friend list — one peer id per line\n");
        for peer in &self.friends {
            text.push_str(&peer.to_base58());
            text.push('\n');
        }
        if let Some(parent) = self.path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        if let Err(err) = std::fs::write(&self.path, text) {
            tracing::warn!(%err, "failed to persist friend list");
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
        self.pending_out.remove(&peer);
        if self.friends.insert(peer) {
            self.save();
        }
    }

    pub fn remove(&mut self, peer: &PeerId) {
        self.pending_in.remove(peer);
        self.pending_out.remove(peer);
        if self.friends.remove(peer) {
            self.save();
        }
    }
}
