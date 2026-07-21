//! rexnet-core — serverless P2P control plane for RexNet.
//!
//! Layer L1 of the RexNet design: identity, Kademlia peer routing, relay
//! client/service, DCUtR hole punching, mDNS, and the request-response
//! protocols (presence / friend / invite / session / punch / tunnel).
//!
//! This crate contains **no Xbox concepts** beyond an opaque namespace tag
//! (the title id passed through [`ffi::RexNetConfig`]). All XDK semantics
//! live in the C++ XAM shim (Layer L3, `src/rexnet`).
//!
//! The public surface is the C ABI in [`ffi`]; everything else is internal.

pub mod crypto;
pub mod engine;
pub mod ffi;
pub mod friends;
pub mod identity;
pub mod shard;
pub mod subnet;
pub mod tcp;
pub mod tunnel;

/// Wire-protocol version prefixes (design spec §7). Bump majors deliberately.
pub mod protocol {
    pub const PRESENCE: &str = "/rexnet/presence/1.0.0";
    pub const FRIEND: &str = "/rexnet/friend/1.0.0";
    pub const INVITE: &str = "/rexnet/invite/1.0.0";
    pub const SESSION: &str = "/rexnet/session/1.0.0";
    /// Punch signalling, and the X25519 key agreement for the game plane
    /// (§6). 2.0.0 because the offer/answer now carry public keys: a peer
    /// speaking 1.0.0 would punch successfully and then be unable to decrypt
    /// anything, so this must fail negotiation instead.
    pub const PUNCH: &str = "/rexnet/punch/2.0.0";

    /// Degraded game plane (§5, §14).
    ///
    /// 2.0.0 because the carrier changed shape, not just the payload: 1.0.0
    /// was request-response with one substream and one ack per datagram,
    /// which exhausted the inbound substream allowance under game load. This
    /// is a single long-lived stream of length-prefixed frames. The two are
    /// not interoperable, and a version bump makes that a clean negotiation
    /// failure instead of a peer that connects and then misframes everything.
    pub const TUNNEL: &str = "/rexnet/tunnel/2.0.0";

    /// Ambient title-shard descriptor query (§17.3.1).
    pub const SHARD: &str = "/rexnet/shard/1.0.0";

    /// Guest TCP carried as a libp2p stream (§18).
    ///
    /// System Link titles are not UDP-only: SoulCalibur IV runs a TCP server
    /// on port 1001 and does its session work over streams. Reimplementing
    /// TCP over the punched UDP path would be rebuilding what the libp2p
    /// connection already provides -- reliable, ordered, multiplexed -- so a
    /// guest stream socket maps onto a stream here.
    pub const TCP: &str = "/rexnet/tcp/1.0.0";

    /// DHT provider-record namespace for public sessions (§10).
    pub fn session_key(title_id: u32) -> String {
        format!("rexnet/v1/title/{title_id:08x}/sessions")
    }

    /// DHT provider-record namespace for ambient title shards (§17.3.1).
    ///
    /// Deliberately a *different* key from [`session_key`]. An ambient shard
    /// is not an XSession: if the two shared a namespace, a title's session
    /// search would find a 255-member shard and try to join it as a game.
    pub fn shard_key(title_id: u32) -> String {
        format!("rexnet/v1/title/{title_id:08x}/shards")
    }

    /// Gossipsub topic carrying one shard's ambient presence (§17.3.4).
    pub fn shard_topic(shard_id: &[u8; 16]) -> String {
        let mut hex = String::with_capacity(32);
        for byte in shard_id {
            hex.push_str(&format!("{byte:02x}"));
        }
        format!("rexnet/v1/shard/{hex}")
    }

    /// DHT provider-record namespace for opt-in mesh relays (§9).
    pub const RELAYS_KEY: &str = "rexnet/v1/relays";

    /// Milestone-1 scratch protocol (spec §15): request-response echo used to
    /// prove connectivity end to end. Not part of the published §7 surface.
    pub const ECHO: &str = "/rexnet/echo/1.0.0";
}

/// Standard Amino (IPFS) DHT bootstrap set (spec §4). The engine never
/// implies these: callers opt in (the CLI does by default for `find`; the
/// shim passes whatever the user configured, and an empty list is valid —
/// LAN mDNS, manual strings, and direct v6 keep working without it).
pub const AMINO_BOOTSTRAP: &[&str] = &[
    "/dnsaddr/bootstrap.libp2p.io/p2p/QmNnooDu7bfjPFoTZYxMNLWUQJyrVwtbZg5gBMjTezGAJN",
    "/dnsaddr/bootstrap.libp2p.io/p2p/QmQCU2EcMqAqQPR2i9bChDtGNJchTbq5TbXJJ16u19uLTa",
    "/dnsaddr/bootstrap.libp2p.io/p2p/QmbLHAnMoJPWSCR5Zhtx6BHJX9KiKNN6tpvbUcqanj75Nb",
    "/dnsaddr/bootstrap.libp2p.io/p2p/QmcZf59bWwK5XFi76CZX8cbJ4BhTzzA3gU1ZjYZcYW3dwt",
];
