//! Degraded game plane — design spec §5, §14.
//!
//! When the punch fails (CGNAT, symmetric NAT) game traffic has to ride the
//! libp2p control connection instead. The first implementation did that with
//! request-response: one CBOR request per datagram, each acknowledged.
//!
//! That is the wrong shape for a game plane. A title sending 30 datagrams a
//! second opened 30 substreams a second *per peer*, and libp2p caps inbound
//! substreams per connection — we hit "maximum number of inbound substreams
//! exceeded" in practice, which turns a degraded link into a dead one under
//! exactly the load it exists to carry. Every frame also paid for CBOR map
//! keys and a round-trip ack that UDP semantics never asked for.
//!
//! So the tunnel is now one long-lived stream per peer carrying length-
//! prefixed frames. Opening it costs one substream for the life of the link
//! rather than one per datagram, and the wire cost per frame drops to a
//! 6-byte header.
//!
//! Frames are **not** acknowledged and may be dropped when the link backs up.
//! That is deliberate: the guest thinks it is sending UDP, and a tunnel that
//! silently added reliability would change the timing characteristics the
//! title was written against.

/// `[u16 payload_len][u16 src_port][u16 dst_port]`, big-endian to match guest
/// byte order.
pub const TUNNEL_HEADER_LEN: usize = 6;

/// Largest guest datagram the tunnel will carry.
///
/// Matched to the game socket's 2048-byte receive buffer, which is the cap on
/// the punched path already — the two paths must have the same limits or a
/// datagram would survive a punch and vanish on fallback. It also bounds what
/// a peer can make us allocate from a single frame header.
pub const MAX_TUNNEL_PAYLOAD: usize = 2048;

/// Serialise a datagram for the tunnel stream. Returns `None` if the payload
/// exceeds [`MAX_TUNNEL_PAYLOAD`], which the caller should drop rather than
/// truncate: half a datagram is worse than none.
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

/// Read a frame header: `(src_port, dst_port, payload_len)`.
///
/// `None` means the stream is unusable, not that this frame is bad — a byte
/// stream cannot be resynchronised after a length that makes no sense, so the
/// caller must close it rather than skip ahead and misframe everything after.
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
