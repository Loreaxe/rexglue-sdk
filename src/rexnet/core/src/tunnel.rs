// @file        rexnet/core/src/tunnel.rs
// @brief       Degraded game plane.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Degraded game plane: one long-lived stream per peer per direction when the
//! punch fails. See docs/rexnet-design-spec.md §5, §14.
//!
//! Frames are not acknowledged and may be dropped — the guest thinks it is
//! sending UDP, and adding reliability would change the timing the title was
//! written against.

/// `[u16 payload_len][u16 src_port][u16 dst_port]`, big-endian.
pub const TUNNEL_HEADER_LEN: usize = 6;

/// Matches the game socket's receive buffer: both paths must cap alike, or a
/// datagram would survive a punch and vanish on fallback. Also bounds what a
/// peer can make us allocate from one header.
pub const MAX_TUNNEL_PAYLOAD: usize = 2048;

/// `None` if oversized; callers drop rather than truncate.
pub fn encode(src_port: u16, dst_port: u16, data: &[u8]) -> Option<Vec<u8>> {
    if data.len() > MAX_TUNNEL_PAYLOAD {
        return None;
    }
    let mut out = Vec::with_capacity(TUNNEL_HEADER_LEN + data.len());
    out.extend_from_slice(&(data.len() as u16).to_be_bytes());
    out.extend_from_slice(&src_port.to_be_bytes());
    out.extend_from_slice(&dst_port.to_be_bytes());
    out.extend_from_slice(data);
    Some(out)
}

/// `None` means the stream is unusable, not just this frame: a byte stream
/// cannot resynchronise after a bad length, so the caller must close it.
pub fn decode_header(bytes: &[u8; TUNNEL_HEADER_LEN]) -> Option<(u16, u16, usize)> {
    let len = u16::from_be_bytes([bytes[0], bytes[1]]) as usize;
    if len > MAX_TUNNEL_PAYLOAD {
        return None;
    }
    let src_port = u16::from_be_bytes([bytes[2], bytes[3]]);
    let dst_port = u16::from_be_bytes([bytes[4], bytes[5]]);
    Some((src_port, dst_port, len))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header_of(frame: &[u8]) -> [u8; TUNNEL_HEADER_LEN] {
        let mut header = [0u8; TUNNEL_HEADER_LEN];
        header.copy_from_slice(&frame[..TUNNEL_HEADER_LEN]);
        header
    }

    #[test]
    fn frame_round_trips() {
        let payload = b"XRNM-ish payload".to_vec();
        let frame = encode(1000, 1001, &payload).expect("within cap");
        let (src, dst, len) = decode_header(&header_of(&frame)).expect("valid header");
        assert_eq!(src, 1000);
        assert_eq!(dst, 1001);
        assert_eq!(len, payload.len());
        assert_eq!(&frame[TUNNEL_HEADER_LEN..], payload.as_slice());
    }

    #[test]
    fn empty_payload_is_legal() {
        // A zero-length UDP datagram is meaningful (some titles use it as a
        // keepalive), so it must survive the tunnel rather than be dropped.
        let frame = encode(7, 8, &[]).expect("empty is fine");
        assert_eq!(frame.len(), TUNNEL_HEADER_LEN);
        assert_eq!(decode_header(&header_of(&frame)), Some((7, 8, 0)));
    }

    #[test]
    fn header_is_big_endian() {
        let frame = encode(0x1234, 0x03E9, &[0xAA]).expect("within cap");
        assert_eq!(&frame[..2], &[0x00, 0x01]); // length
        assert_eq!(&frame[2..4], &[0x12, 0x34]); // src
        assert_eq!(&frame[4..6], &[0x03, 0xE9]); // dst
    }

    #[test]
    fn oversized_payload_is_refused() {
        let big = vec![0u8; MAX_TUNNEL_PAYLOAD + 1];
        assert!(encode(1, 2, &big).is_none());
        assert!(encode(1, 2, &big[..MAX_TUNNEL_PAYLOAD]).is_some(), "the cap itself must fit");
    }

    #[test]
    fn oversized_length_is_refused() {
        // A hostile or desynchronised peer must not be able to make us
        // allocate an arbitrary buffer from six bytes.
        let mut header = [0u8; TUNNEL_HEADER_LEN];
        header[..2].copy_from_slice(&((MAX_TUNNEL_PAYLOAD + 1) as u16).to_be_bytes());
        assert_eq!(decode_header(&header), None);
    }

    #[test]
    fn ports_survive_the_full_range() {
        // Ephemeral guest ports live above 32768; a signed slip would fold
        // them onto low ports and cross-deliver to the wrong socket.
        let frame = encode(65535, 49152, b"x").expect("within cap");
        assert_eq!(decode_header(&header_of(&frame)), Some((65535, 49152, 1)));
    }
}
