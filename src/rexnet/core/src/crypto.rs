// @file        rexnet/core/src/crypto.rs
// @brief       Game-plane encryption for the punched UDP socket.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Game-plane encryption for the punched UDP socket. Full rationale and wire
//! format in docs/rexnet-design-spec.md §6.1.
//!
//! X25519 per peer session -> HKDF-SHA256 -> two directional ChaCha20-Poly1305
//! keys. Record is `[counter u64BE][ciphertext||tag]`; the frame type is AAD.
//!
//! Defends against an on-path observer, not against the peer you are playing
//! with — they hold the key. Anti-cheat is a non-goal.

use chacha20poly1305::aead::{Aead, KeyInit, Payload};
use chacha20poly1305::{ChaCha20Poly1305, Key, Nonce};
use hkdf::Hkdf;
use sha2::Sha256;
use x25519_dalek::{PublicKey, StaticSecret};

/// 8-byte counter + 16-byte Poly1305 tag.
pub const CRYPTO_OVERHEAD: usize = 8 + 16;

/// Changing this changes the keys: effectively the crypto version.
const HKDF_INFO: &[u8] = b"rexnet-game-plane-v1";

/// Reorder tolerance. UDP reorders, so rejecting out-of-order would drop good
/// traffic; 64 is the bitmap width and matches DTLS/WireGuard practice.
pub const REPLAY_WINDOW: u64 = 64;

#[derive(Debug, PartialEq, Eq)]
pub enum CryptoError {
    /// Decryption or authentication failed: wrong key, or tampered bytes.
    NotAuthentic,
    /// Well-formed and authentic, but already seen or too old to judge.
    Replay,
    /// Shorter than a counter plus a tag; cannot be a real record.
    Truncated,
    /// Payload exceeds what the game socket will carry.
    TooLarge,
}

/// Fresh per peer session, reused across every punch attempt to that peer —
/// see `derive`.
pub struct LocalKeyAgreement {
    secret: StaticSecret,
    public: [u8; 32],
}

impl LocalKeyAgreement {
    pub fn new() -> Self {
        let secret = StaticSecret::random_from_rng(rand::rngs::OsRng);
        let public = PublicKey::from(&secret).to_bytes();
        Self { secret, public }
    }

    /// The public key to put in a punch offer or answer.
    pub fn public_key(&self) -> [u8; 32] {
        self.public
    }

    /// Idempotent: simultaneous punches mean the peer's key can arrive twice,
    /// and both ends must land on the same result. Direction comes from
    /// comparing public keys rather than a caller-passed initiator flag, which
    /// the two ends could disagree about.
    pub fn derive(&self, peer_public: &[u8; 32]) -> SessionKeys {
        let shared = self.secret.diffie_hellman(&PublicKey::from(*peer_public));

        // Order both sides compute identically.
        let (low, high) = if self.public <= *peer_public {
            (self.public, *peer_public)
        } else {
            (*peer_public, self.public)
        };
        let mut info = Vec::with_capacity(HKDF_INFO.len() + 64);
        info.extend_from_slice(HKDF_INFO);
        info.extend_from_slice(&low);
        info.extend_from_slice(&high);

        let hk = Hkdf::<Sha256>::new(None, shared.as_bytes());
        let mut okm = [0u8; 64];
        hk.expand(&info, &mut okm)
            .expect("64 bytes is a valid HKDF-SHA256 output length");

        let (first, second) = okm.split_at(32);
        let we_are_low = self.public <= *peer_public;
        let (send, recv) = if we_are_low { (first, second) } else { (second, first) };

        SessionKeys {
            sealer: Sealer::new(send),
            opener: Opener::new(recv),
        }
    }
}

impl Default for LocalKeyAgreement {
    fn default() -> Self {
        Self::new()
    }
}

/// Both halves of one peer's game-plane session.
pub struct SessionKeys {
    pub sealer: Sealer,
    pub opener: Opener,
}

/// Outbound half: encrypts and numbers datagrams.
pub struct Sealer {
    cipher: ChaCha20Poly1305,
    counter: u64,
}

impl Sealer {
    fn new(key: &[u8]) -> Self {
        Self {
            cipher: ChaCha20Poly1305::new(Key::from_slice(key)),
            counter: 0,
        }
    }

    /// `aad` is the frame type, so a record cannot be reinterpreted as
    /// another kind.
    pub fn seal(&mut self, aad: &[u8], plaintext: &[u8]) -> Result<Vec<u8>, CryptoError> {
        let counter = self.counter;
        // Wrapping would reuse a nonce under the same key. Unreachable in
        // practice, checked anyway.
        self.counter = self.counter.checked_add(1).expect("game-plane nonce counter exhausted");

        let nonce = nonce_for(counter);
        let ciphertext = self
            .cipher
            .encrypt(&nonce, Payload { msg: plaintext, aad })
            .map_err(|_| CryptoError::TooLarge)?;

        let mut out = Vec::with_capacity(8 + ciphertext.len());
        out.extend_from_slice(&counter.to_be_bytes());
        out.extend_from_slice(&ciphertext);
        Ok(out)
    }

    #[cfg(test)]
    pub fn counter(&self) -> u64 {
        self.counter
    }
}

/// Inbound half: decrypts, and refuses anything it has already accepted.
pub struct Opener {
    cipher: ChaCha20Poly1305,
    /// Highest counter accepted so far.
    highest: u64,
    /// Bit *i* means `highest - 1 - i` has been seen.
    seen: u64,
    /// Without this, counter 0 is indistinguishable from "nothing yet".
    started: bool,
}

impl Opener {
    fn new(key: &[u8]) -> Self {
        Self {
            cipher: ChaCha20Poly1305::new(Key::from_slice(key)),
            highest: 0,
            seen: 0,
            started: false,
        }
    }

    /// The window only advances after authentication, so a forged packet with
    /// a high counter cannot lock out the real peer.
    pub fn open(&mut self, aad: &[u8], record: &[u8]) -> Result<Vec<u8>, CryptoError> {
        if record.len() < 8 + 16 {
            return Err(CryptoError::Truncated);
        }
        let mut counter_bytes = [0u8; 8];
        counter_bytes.copy_from_slice(&record[..8]);
        let counter = u64::from_be_bytes(counter_bytes);

        if self.is_replay(counter) {
            return Err(CryptoError::Replay);
        }

        let nonce = nonce_for(counter);
        let plaintext = self
            .cipher
            .decrypt(&nonce, Payload { msg: &record[8..], aad })
            .map_err(|_| CryptoError::NotAuthentic)?;

        self.accept(counter);
        Ok(plaintext)
    }

    fn is_replay(&self, counter: u64) -> bool {
        if !self.started {
            return false;
        }
        if counter > self.highest {
            return false;
        }
        let behind = self.highest - counter;
        if behind >= REPLAY_WINDOW {
            return true; // too old to prove it is not a replay
        }
        if behind == 0 {
            return true; // exactly the highest, already accepted
        }
        self.seen & (1u64 << (behind - 1)) != 0
    }

    fn accept(&mut self, counter: u64) {
        if !self.started {
            self.started = true;
            self.highest = counter;
            self.seen = 0;
            return;
        }
        if counter > self.highest {
            let advance = counter - self.highest;
            if advance >= 64 {
                self.seen = 0;
            } else {
                self.seen = (self.seen << advance) | (1u64 << (advance - 1));
            }
            self.highest = counter;
        } else {
            let behind = self.highest - counter;
            if behind >= 1 && behind <= REPLAY_WINDOW {
                self.seen |= 1u64 << (behind - 1);
            }
        }
    }
}

fn nonce_for(counter: u64) -> Nonce {
    let mut bytes = [0u8; 12];
    bytes[4..].copy_from_slice(&counter.to_be_bytes());
    *Nonce::from_slice(&bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pair() -> (SessionKeys, SessionKeys) {
        let a = LocalKeyAgreement::new();
        let b = LocalKeyAgreement::new();
        (a.derive(&b.public_key()), b.derive(&a.public_key()))
    }

    #[test]
    fn round_trips_between_peers() {
        let (mut a, mut b) = pair();
        let msg = b"XRNM-ish game datagram";
        let sealed = a.sealer.seal(&[0x01], msg).unwrap();
        assert_eq!(b.opener.open(&[0x01], &sealed).unwrap(), msg);
    }

    #[test]
    fn both_directions_work() {
        // Directional keys are the point: if send/recv were swapped on one
        // side, one direction would silently fail to authenticate.
        let (mut a, mut b) = pair();
        let to_b = a.sealer.seal(&[0x01], b"a->b").unwrap();
        assert_eq!(b.opener.open(&[0x01], &to_b).unwrap(), b"a->b");
        let to_a = b.sealer.seal(&[0x01], b"b->a").unwrap();
        assert_eq!(a.opener.open(&[0x01], &to_a).unwrap(), b"b->a");
    }

    #[test]
    fn ciphertext_does_not_contain_the_plaintext() {
        // The whole point of the change. A game datagram must not be readable
        // by anyone watching the punched socket.
        let (mut a, _b) = pair();
        let secret = b"PLAYER-POSITION-SECRET";
        let sealed = a.sealer.seal(&[0x01], secret).unwrap();
        assert!(
            !sealed.windows(secret.len()).any(|w| w == secret),
            "plaintext survived into the record"
        );
    }

    #[test]
    fn tampering_is_detected() {
        let (mut a, mut b) = pair();
        let mut sealed = a.sealer.seal(&[0x01], b"payload").unwrap();
        let last = sealed.len() - 1;
        sealed[last] ^= 0x01;
        assert_eq!(b.opener.open(&[0x01], &sealed), Err(CryptoError::NotAuthentic));
    }

    #[test]
    fn associated_data_is_bound() {
        // A record must not be replayable as a different frame kind.
        let (mut a, mut b) = pair();
        let sealed = a.sealer.seal(&[0x01], b"payload").unwrap();
        assert_eq!(b.opener.open(&[0x02], &sealed), Err(CryptoError::NotAuthentic));
    }

    #[test]
    fn a_third_party_cannot_read_or_forge() {
        let (mut a, _b) = pair();
        let (_c, mut d) = pair();
        let sealed = a.sealer.seal(&[0x01], b"payload").unwrap();
        assert_eq!(d.opener.open(&[0x01], &sealed), Err(CryptoError::NotAuthentic));
    }

    #[test]
    fn a_new_session_derives_different_keys() {
        // Keypairs are fresh per peer session, so a later session between the
        // same two players must not reuse keys.
        let (mut old_a, _old_b) = pair();
        let (_new_a, mut new_b) = pair();
        let sealed = old_a.sealer.seal(&[0x01], b"payload").unwrap();
        assert_eq!(new_b.opener.open(&[0x01], &sealed), Err(CryptoError::NotAuthentic));
    }

    #[test]
    fn derivation_is_stable_and_order_independent() {
        // The heart of the simultaneous-punch fix: whichever message carried
        // the public key, and whichever punch won, both ends must land on the
        // same keys -- and deriving twice must not produce something new.
        let a = LocalKeyAgreement::new();
        let b = LocalKeyAgreement::new();

        let mut a1 = a.derive(&b.public_key());
        let mut b1 = b.derive(&a.public_key());
        let mut a2 = a.derive(&b.public_key());
        let mut b2 = b.derive(&a.public_key());

        let from_a1 = a1.sealer.seal(&[0x01], b"payload").unwrap();
        assert!(b2.opener.open(&[0x01], &from_a1).is_ok(), "re-derived key differs");
        let from_b1 = b1.sealer.seal(&[0x01], b"payload").unwrap();
        assert!(a2.opener.open(&[0x01], &from_b1).is_ok(), "re-derived key differs");
    }

    #[test]
    fn the_two_sides_never_share_a_send_key() {
        // If both ends sealed under the same key they would also share a
        // nonce sequence, which is the one failure ChaCha20-Poly1305 does not
        // survive. Direction is chosen by public-key order precisely to make
        // that impossible.
        let a = LocalKeyAgreement::new();
        let b = LocalKeyAgreement::new();
        let mut ka = a.derive(&b.public_key());
        let mut kb = b.derive(&a.public_key());
        let sa = ka.sealer.seal(&[0x01], b"same plaintext").unwrap();
        let sb = kb.sealer.seal(&[0x01], b"same plaintext").unwrap();
        assert_ne!(sa, sb, "both directions sealed identically -- shared key and nonce");
        // And neither can open its own record.
        assert_eq!(ka.opener.open(&[0x01], &sa), Err(CryptoError::NotAuthentic));
    }

    #[test]
    fn replayed_datagram_is_rejected() {
        let (mut a, mut b) = pair();
        let sealed = a.sealer.seal(&[0x01], b"payload").unwrap();
        assert!(b.opener.open(&[0x01], &sealed).is_ok());
        assert_eq!(b.opener.open(&[0x01], &sealed), Err(CryptoError::Replay));
    }

    #[test]
    fn out_of_order_within_the_window_is_accepted() {
        // UDP reorders; rejecting that would drop legitimate game traffic.
        let (mut a, mut b) = pair();
        let first = a.sealer.seal(&[0x01], b"one").unwrap();
        let second = a.sealer.seal(&[0x01], b"two").unwrap();
        let third = a.sealer.seal(&[0x01], b"three").unwrap();
        assert!(b.opener.open(&[0x01], &third).is_ok());
        assert!(b.opener.open(&[0x01], &first).is_ok(), "reordered packet was dropped");
        assert!(b.opener.open(&[0x01], &second).is_ok(), "reordered packet was dropped");
        // ...but each still only once.
        assert_eq!(b.opener.open(&[0x01], &first), Err(CryptoError::Replay));
    }

    #[test]
    fn very_old_datagrams_fall_out_of_the_window() {
        let (mut a, mut b) = pair();
        let ancient = a.sealer.seal(&[0x01], b"ancient").unwrap();
        for _ in 0..REPLAY_WINDOW + 8 {
            let f = a.sealer.seal(&[0x01], b"filler").unwrap();
            assert!(b.opener.open(&[0x01], &f).is_ok());
        }
        assert_eq!(
            b.opener.open(&[0x01], &ancient),
            Err(CryptoError::Replay),
            "a datagram older than the window must be refused, not guessed at"
        );
    }

    #[test]
    fn forged_high_counter_cannot_lock_out_the_peer() {
        // Authentication runs before the window advances. Otherwise an
        // attacker could send a garbage record with counter u64::MAX and
        // every genuine datagram afterwards would look like a replay.
        let (mut a, mut b) = pair();
        let mut forged = a.sealer.seal(&[0x01], b"payload").unwrap();
        forged[..8].copy_from_slice(&u64::MAX.to_be_bytes());
        assert_eq!(b.opener.open(&[0x01], &forged), Err(CryptoError::NotAuthentic));

        let genuine = a.sealer.seal(&[0x01], b"still fine").unwrap();
        assert!(
            b.opener.open(&[0x01], &genuine).is_ok(),
            "a forged packet poisoned the replay window"
        );
    }

    #[test]
    fn counter_starts_at_zero_and_advances() {
        let (mut a, mut b) = pair();
        assert_eq!(a.sealer.counter(), 0);
        let first = a.sealer.seal(&[0x01], b"x").unwrap();
        assert_eq!(&first[..8], &0u64.to_be_bytes());
        assert_eq!(a.sealer.counter(), 1);
        // Counter 0 must be accepted; it is a real value, not "unset".
        assert!(b.opener.open(&[0x01], &first).is_ok());
    }

    #[test]
    fn truncated_records_are_rejected() {
        let (mut a, mut b) = pair();
        let sealed = a.sealer.seal(&[0x01], b"payload").unwrap();
        for n in 0..8 + 16 {
            assert_eq!(
                b.opener.open(&[0x01], &sealed[..n]),
                Err(CryptoError::Truncated),
                "accepted a {n}-byte record"
            );
        }
    }

    #[test]
    fn empty_payload_survives() {
        // A zero-length datagram is meaningful to some titles (keepalives).
        let (mut a, mut b) = pair();
        let sealed = a.sealer.seal(&[0x01], b"").unwrap();
        assert_eq!(sealed.len(), 8 + 16);
        assert_eq!(b.opener.open(&[0x01], &sealed).unwrap(), b"");
    }

    #[test]
    fn overhead_constant_matches_reality() {
        let (mut a, _b) = pair();
        let sealed = a.sealer.seal(&[0x01], b"1234567890").unwrap();
        assert_eq!(sealed.len(), 10 + CRYPTO_OVERHEAD);
    }
}
