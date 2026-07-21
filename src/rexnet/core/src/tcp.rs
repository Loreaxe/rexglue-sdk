// @file        rexnet/core/src/tcp.rs
// @brief       Guest TCP over libp2p streams.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Guest TCP over libp2p streams — docs/rexnet-design-spec.md §18.1.
//!
//! System Link titles are not UDP-only, and the libp2p connection is already
//! reliable and ordered, so a guest SOCK_STREAM maps onto a stream. A stream
//! names a peer but not a listener, hence the port header below.

use std::collections::HashMap;

use libp2p::PeerId;

/// Allocated locally and never sent: the two ends number the same connection
/// differently.
pub type StreamId = u64;

/// Opening header on a `/rexnet/tcp/1.0.0` stream. Big-endian: the shim reads
/// these straight into guest structures.
///
/// ```text
///   0        1        2        3        4
/// +--------+--------+--------+--------+--------+
/// | schema |    dst_port     |    src_port     |
/// +--------+--------+--------+--------+--------+
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreamHeader {
    pub schema: u8,
    /// Guest port being connected *to* — which listener this belongs to.
    pub dst_port: u16,
    /// Guest port the initiator bound, reported to the acceptor as the peer
    /// port so `accept()` can fill in a sockaddr.
    pub src_port: u16,
}

pub const STREAM_HEADER_LEN: usize = 5;
pub const STREAM_SCHEMA: u8 = 1;

impl StreamHeader {
    pub fn encode(&self) -> [u8; STREAM_HEADER_LEN] {
        let mut out = [0u8; STREAM_HEADER_LEN];
        out[0] = self.schema;
        out[1..3].copy_from_slice(&self.dst_port.to_be_bytes());
        out[3..5].copy_from_slice(&self.src_port.to_be_bytes());
        out
    }

    /// `None` rather than guessing: a stream opened by something that is not
    /// us must not be mistaken for a guest connection.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < STREAM_HEADER_LEN || bytes[0] != STREAM_SCHEMA {
            return None;
        }
        Some(Self {
            schema: bytes[0],
            dst_port: u16::from_be_bytes([bytes[1], bytes[2]]),
            src_port: u16::from_be_bytes([bytes[3], bytes[4]]),
        })
    }
}

/// What the engine knows about one live guest connection.
#[derive(Debug, Clone)]
pub struct StreamInfo {
    pub peer: PeerId,
    /// Local guest port: the bound listener for an accepted connection, the
    /// initiator's own port for an outbound one.
    pub local_port: u16,
    /// The far side's guest port.
    pub remote_port: u16,
    /// True when we opened it, false when we accepted it. Only used for
    /// diagnostics -- the two are otherwise symmetric.
    pub outbound: bool,
}

/// The engine owns the futures that move bytes; this only answers "what is
/// stream N" and "which streams belong to a peer that went away".
#[derive(Debug, Default)]
pub struct StreamTable {
    next_id: StreamId,
    streams: HashMap<StreamId, StreamInfo>,
}

impl StreamTable {
    pub fn new() -> Self {
        // Start at 1: zero reads as "no stream" across the FFI, where a
        // zeroed event field is otherwise indistinguishable from a real id.
        Self { next_id: 1, streams: HashMap::new() }
    }

    pub fn insert(&mut self, info: StreamInfo) -> StreamId {
        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1).max(1);
        self.streams.insert(id, info);
        id
    }

    pub fn get(&self, id: StreamId) -> Option<&StreamInfo> {
        self.streams.get(&id)
    }

    pub fn remove(&mut self, id: StreamId) -> Option<StreamInfo> {
        self.streams.remove(&id)
    }

    pub fn len(&self) -> usize {
        self.streams.len()
    }

    pub fn is_empty(&self) -> bool {
        self.streams.is_empty()
    }

    /// A dropped connection takes its guest sockets with it; the shim must be
    /// told, or the guest sees a socket that silently stops.
    pub fn streams_for_peer(&self, peer: &PeerId) -> Vec<StreamId> {
        self.streams
            .iter()
            .filter(|(_, info)| info.peer == *peer)
            .map(|(id, _)| *id)
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_round_trips() {
        let header = StreamHeader { schema: STREAM_SCHEMA, dst_port: 1001, src_port: 49152 };
        let encoded = header.encode();
        assert_eq!(encoded.len(), STREAM_HEADER_LEN);
        assert_eq!(StreamHeader::decode(&encoded), Some(header));
    }

    #[test]
    fn header_is_big_endian() {
        // The shim reads these into guest structures, which are big-endian.
        let header = StreamHeader { schema: STREAM_SCHEMA, dst_port: 0x03E9, src_port: 0x1234 };
        let encoded = header.encode();
        assert_eq!(&encoded[1..3], &[0x03, 0xE9]);
        assert_eq!(&encoded[3..5], &[0x12, 0x34]);
    }

    #[test]
    fn truncated_header_is_rejected() {
        let header = StreamHeader { schema: STREAM_SCHEMA, dst_port: 1001, src_port: 1 };
        let encoded = header.encode();
        for n in 0..STREAM_HEADER_LEN {
            assert_eq!(StreamHeader::decode(&encoded[..n]), None, "accepted {n} bytes");
        }
    }

    #[test]
    fn unknown_schema_is_rejected() {
        // A stream opened by a future or foreign implementation must not be
        // read as a guest connection.
        let mut encoded = StreamHeader { schema: STREAM_SCHEMA, dst_port: 1, src_port: 2 }.encode();
        encoded[0] = 0xFF;
        assert_eq!(StreamHeader::decode(&encoded), None);
    }

    #[test]
    fn ids_start_at_one_and_are_unique() {
        let mut table = StreamTable::new();
        let peer = PeerId::random();
        let a = table.insert(StreamInfo { peer, local_port: 1, remote_port: 2, outbound: true });
        let b = table.insert(StreamInfo { peer, local_port: 3, remote_port: 4, outbound: false });
        assert_ne!(a, 0, "0 must stay reserved for 'no stream'");
        assert_ne!(a, b);
        assert_eq!(table.len(), 2);
    }

    #[test]
    fn remove_forgets_the_stream() {
        let mut table = StreamTable::new();
        let peer = PeerId::random();
        let id = table.insert(StreamInfo { peer, local_port: 1, remote_port: 2, outbound: true });
        assert!(table.get(id).is_some());
        assert!(table.remove(id).is_some());
        assert!(table.get(id).is_none());
        assert!(table.remove(id).is_none(), "double close must be harmless");
    }

    #[test]
    fn streams_for_peer_finds_only_that_peer() {
        let mut table = StreamTable::new();
        let mine = PeerId::random();
        let other = PeerId::random();
        let a = table.insert(StreamInfo { peer: mine, local_port: 1, remote_port: 2, outbound: true });
        let b = table.insert(StreamInfo { peer: mine, local_port: 3, remote_port: 4, outbound: true });
        table.insert(StreamInfo { peer: other, local_port: 5, remote_port: 6, outbound: false });

        let mut found = table.streams_for_peer(&mine);
        found.sort_unstable();
        let mut expected = vec![a, b];
        expected.sort_unstable();
        assert_eq!(found, expected);
        assert_eq!(table.streams_for_peer(&PeerId::random()).len(), 0);
    }
}
