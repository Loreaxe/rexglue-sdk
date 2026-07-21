// @file        rexnet/core/src/identity.rs
// @brief       Persistent ed25519 identity (design spec §3).
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Persistent ed25519 identity (design spec §3).
//!
//! One keypair per install; the derived PeerId is the player's permanent
//! identity across every recomp. Losing the key means a new identity, so the
//! file is precious — document export/import for users.

use std::io::Write;
use std::path::Path;

use libp2p::identity::Keypair;
use libp2p::PeerId;

pub const IDENTITY_FILE: &str = "identity.key";

#[derive(Debug, thiserror::Error)]
pub enum IdentityError {
    #[error("io error accessing identity key: {0}")]
    Io(#[from] std::io::Error),
    #[error("identity key file is corrupt: {0}")]
    Decode(#[from] libp2p::identity::DecodingError),
}

/// Load `identity.key` from `data_dir`, generating (and persisting, mode
/// 0600) a fresh ed25519 keypair on first run.
pub fn load_or_generate(data_dir: &Path) -> Result<Keypair, IdentityError> {
    let path = data_dir.join(IDENTITY_FILE);
    if path.exists() {
        let bytes = std::fs::read(&path)?;
        return Ok(Keypair::from_protobuf_encoding(&bytes)?);
    }

    let keypair = Keypair::generate_ed25519();
    let bytes = keypair
        .to_protobuf_encoding()
        .expect("ed25519 keys are always encodable");

    std::fs::create_dir_all(data_dir)?;
    let mut options = std::fs::OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    options.open(&path)?.write_all(&bytes)?;

    Ok(keypair)
}

/// Human-exchangeable friend code (§3): `REXN-` + dash-grouped base32 of
/// `[32-byte ed25519 key][display name UTF-8, 0–24 bytes]` plus a 4-char
/// (20-bit, truncated CRC-32) checksum group. Case-insensitive on input;
/// dashes/whitespace optional; 0/1 accepted as O/I. The embedded name lets
/// the receiving side label the friend before any presence arrives.
///
/// Every identity we mint is an ed25519 identity multihash, whose PeerId
/// bytes are the fixed prefix `00 24 08 01 12 20` + the 32-byte key, so the
/// code round-trips exactly and stays far shorter than base58ing the whole
/// multihash.
const FRIEND_CODE_PREFIX: &str = "REXN";
pub const FRIEND_CODE_NAME_MAX: usize = 24;
const ED25519_PEER_PREFIX: [u8; 6] = [0x00, 0x24, 0x08, 0x01, 0x12, 0x20];
const B32_ALPHABET: &[u8; 32] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";

fn b32_encode(data: &[u8]) -> String {
    let mut out = String::new();
    let mut acc: u32 = 0;
    let mut bits = 0u32;
    for &byte in data {
        acc = (acc << 8) | u32::from(byte);
        bits += 8;
        while bits >= 5 {
            bits -= 5;
            out.push(B32_ALPHABET[((acc >> bits) & 0x1F) as usize] as char);
        }
    }
    if bits > 0 {
        out.push(B32_ALPHABET[((acc << (5 - bits)) & 0x1F) as usize] as char);
    }
    out
}

fn b32_decode(s: &str) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(s.len() * 5 / 8);
    let mut acc: u32 = 0;
    let mut bits = 0u32;
    for ch in s.bytes() {
        let value = B32_ALPHABET.iter().position(|&a| a == ch)? as u32;
        acc = (acc << 5) | value;
        bits += 5;
        if bits >= 8 {
            bits -= 8;
            out.push(((acc >> bits) & 0xFF) as u8);
        }
    }
    // Trailing bits are encoder padding and must be zero.
    if (acc & ((1 << bits) - 1)) != 0 {
        return None;
    }
    Some(out)
}

/// Keep an embedded name codeable: UTF-8, no control chars, ≤24 bytes
/// (truncated on a char boundary).
pub fn sanitize_display_name(name: &str) -> String {
    let mut out = String::with_capacity(FRIEND_CODE_NAME_MAX);
    for ch in name.chars() {
        if ch.is_control() {
            continue;
        }
        if out.len() + ch.len_utf8() > FRIEND_CODE_NAME_MAX {
            break;
        }
        out.push(ch);
    }
    out.trim().to_string()
}

/// CRC-32 (IEEE, bitwise) truncated to 20 bits = one base32 quad.
fn crc20(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xEDB8_8320 & (0u32.wrapping_sub(crc & 1)));
        }
    }
    !crc & 0xF_FFFF
}

/// Render a peer id (+ optional display name) as a friend code. `None`
/// when the peer id is not an ed25519 identity multihash (never the case
/// for identities we generate).
pub fn render_friend_code(peer: &PeerId, name: &str) -> Option<String> {
    let bytes = peer.to_bytes();
    let key = bytes.strip_prefix(&ED25519_PEER_PREFIX)?;
    if key.len() != 32 {
        return None;
    }
    let mut payload = key.to_vec();
    payload.extend_from_slice(sanitize_display_name(name).as_bytes());
    let mut chars = b32_encode(&payload);
    let check = crc20(&payload);
    for shift in [15u32, 10, 5, 0] {
        chars.push(B32_ALPHABET[((check >> shift) & 0x1F) as usize] as char);
    }
    let mut code = String::with_capacity(4 + chars.len() + chars.len() / 4);
    code.push_str(FRIEND_CODE_PREFIX);
    for (i, ch) in chars.chars().enumerate() {
        if i % 4 == 0 {
            code.push('-');
        }
        code.push(ch);
    }
    Some(code)
}

/// Parse a friend code back into a PeerId + the embedded display name
/// (empty for nameless codes). Tolerates lowercase, missing or extra
/// dashes/whitespace, and the 0→O / 1→I lookalikes; rejects on any
/// checksum mismatch.
pub fn parse_friend_code(s: &str) -> Option<(PeerId, String)> {
    let mut chars = String::with_capacity(96);
    for ch in s.chars() {
        match ch {
            '-' | ' ' | '\t' | '\r' | '\n' => {}
            '0' => chars.push('O'),
            '1' => chars.push('I'),
            _ => chars.push(ch.to_ascii_uppercase()),
        }
    }
    let body = chars.strip_prefix(FRIEND_CODE_PREFIX).unwrap_or(&chars);
    if body.len() < 56 {
        return None;
    }
    let (data, check) = body.split_at(body.len() - 4);
    let payload = b32_decode(data)?;
    if payload.len() < 32 || payload.len() > 32 + FRIEND_CODE_NAME_MAX {
        return None;
    }
    let mut want: u32 = 0;
    for ch in check.bytes() {
        want = (want << 5) | B32_ALPHABET.iter().position(|&a| a == ch)? as u32;
    }
    if want != crc20(&payload) {
        return None;
    }
    let (key, name_bytes) = payload.split_at(32);
    let name = sanitize_display_name(&String::from_utf8_lossy(name_bytes));
    let mut peer_bytes = Vec::with_capacity(38);
    peer_bytes.extend_from_slice(&ED25519_PEER_PREFIX);
    peer_bytes.extend_from_slice(key);
    PeerId::from_bytes(&peer_bytes).ok().map(|peer| (peer, name))
}

/// Friend code for the local keypair (always ed25519, so always renders).
pub fn friend_code(keypair: &Keypair) -> String {
    let peer = keypair.public().to_peer_id();
    render_friend_code(&peer, "").unwrap_or_else(|| peer.to_base58())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn friend_code_round_trips() {
        let keypair = Keypair::generate_ed25519();
        let peer = keypair.public().to_peer_id();
        let code = friend_code(&keypair);
        assert!(code.starts_with("REXN-"), "{code}");
        assert_eq!(code.len(), 4 + 14 * 5, "{code}");
        assert_eq!(parse_friend_code(&code), Some((peer, String::new())));
    }

    #[test]
    fn friend_code_carries_a_name() {
        let keypair = Keypair::generate_ed25519();
        let peer = keypair.public().to_peer_id();
        let code = render_friend_code(&peer, "Loreaxe").unwrap();
        assert_eq!(parse_friend_code(&code), Some((peer, "Loreaxe".to_string())));
        // Max-length and multibyte names survive (truncated on a char
        // boundary at 24 bytes).
        let code = render_friend_code(&peer, "ABCDEFGHIJKLMNOPQRSTUVWXYZ").unwrap();
        assert_eq!(parse_friend_code(&code),
                   Some((peer, "ABCDEFGHIJKLMNOPQRSTUVWX".to_string())));
        let code = render_friend_code(&peer, "Løreæxé\u{7}").unwrap();
        assert_eq!(parse_friend_code(&code), Some((peer, "Løreæxé".to_string())));
    }

    #[test]
    fn parse_is_forgiving_about_formatting() {
        let keypair = Keypair::generate_ed25519();
        let peer = keypair.public().to_peer_id();
        let code = render_friend_code(&peer, "Loreaxe").unwrap();
        let expected = Some((peer, "Loreaxe".to_string()));
        let lowercase_no_dashes: String =
            code.chars().filter(|c| *c != '-').collect::<String>().to_lowercase();
        assert_eq!(parse_friend_code(&lowercase_no_dashes), expected);
        assert_eq!(parse_friend_code(&format!("  {code}\n")), expected);
        // 0/1 lookalikes map to O/I.
        let confused = code.replace('O', "0").replace('I', "1");
        assert_eq!(parse_friend_code(&confused), expected);
    }

    #[test]
    fn corruption_is_rejected() {
        let code = friend_code(&Keypair::generate_ed25519());
        // Flip one data character (guaranteed in-alphabet change).
        let mut bytes = code.into_bytes();
        let i = 6; // inside the first data group
        bytes[i] = if bytes[i] == b'A' { b'B' } else { b'A' };
        let corrupted = String::from_utf8(bytes).unwrap();
        assert_eq!(parse_friend_code(&corrupted), None);
        assert_eq!(parse_friend_code("REXN-TOO-SHORT"), None);
        assert_eq!(parse_friend_code(""), None);
    }
}
