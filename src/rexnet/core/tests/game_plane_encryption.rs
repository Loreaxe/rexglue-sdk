// @file        rexnet/core/tests/game_plane_encryption.rs
// @brief       The game plane carries encrypted traffic end to end.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Proves the engine actually uses the crypto, which is a different claim
//! from `crypto.rs` proving the cipher works.
//!
//! The round-trip half is the discriminating one: miss a send site and the
//! datagram never arrives. That is how `broadcast_datagram` was caught still
//! emitting plaintext.
//!
//! The injection half is defence in depth only — a frame from an unmapped
//! address was already dropped before any of this existed, so it would have
//! passed then too. Spoofing the punched peer's source address is what would
//! test the receive path, and that is not reachable from loopback.

use std::time::Duration;

use libp2p::Multiaddr;
use rexnet_core::engine::{self, Command, EngineConfig, EngineHandles, Event};
use tokio::net::UdpSocket;
use tokio::sync::oneshot;

const TITLE: u32 = 0x4D53_0916;
const TIMEOUT: Duration = Duration::from_secs(30);
/// Fixed so the test can address the game socket directly. High and unusual to
/// avoid colliding with the other integration tests running in parallel.
const B_GAME_PORT: u16 = 45771;
const FRAME_DATA: u8 = 0x01;

async fn spawn_node(dir: &std::path::Path, game_port: u16) -> EngineHandles {
    engine::spawn(
        libp2p::identity::Keypair::generate_ed25519(),
        EngineConfig {
            data_dir: dir.to_path_buf(),
            title_id: TITLE,
            display_name: "crypto".into(),
            bootstrap: vec![],
            listen_port: 0,
            game_port,
            relay: None,
            force_tunnel: false,
        },
    )
    .await
    .expect("engine start")
}

fn tempdir() -> std::path::PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "rexnet-crypto-test-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&path).expect("temp dir");
    path
}

#[tokio::test(flavor = "multi_thread")]
async fn game_traffic_round_trips_encrypted_and_injection_is_refused() {
    let dir_a = tempdir();
    let dir_b = tempdir();
    let mut a = spawn_node(&dir_a, 0).await;
    let mut b = spawn_node(&dir_b, B_GAME_PORT).await;

    // Bring the control plane up and punch, so B has a real, working session.
    tokio::time::sleep(Duration::from_millis(500)).await;
    let (tx, rx) = oneshot::channel();
    let _ = a.cmd_tx.send(Command::LocalAddrs { reply: tx });
    let addrs: Vec<Multiaddr> = rx.await.unwrap_or_default();
    assert!(!addrs.is_empty(), "node A never reported a listen address");
    for addr in addrs {
        let with_peer = addr.with(libp2p::multiaddr::Protocol::P2p(a.peer_id));
        let _ = b.cmd_tx.send(Command::ConnectManual { multiaddr: with_peer.to_string() });
    }

    // Punch first: a peer is only announced once something makes it relevant,
    // so waiting for PeerConnected before asking for anything would wait
    // forever.
    tokio::time::sleep(Duration::from_secs(2)).await;
    let _ = a.cmd_tx.send(Command::Punch { peer: b.peer_id, reply: None });

    let b_vip = tokio::time::timeout(TIMEOUT, async {
        loop {
            match a.evt_rx.recv().await {
                Some(Event::PeerConnected { peer, virtual_ip }) if peer == b.peer_id => {
                    return Some(virtual_ip)
                }
                Some(_) => continue,
                None => return None,
            }
        }
    })
    .await
    .ok()
    .flatten()
    .expect("A never learned B's virtual IP");

    // Genuine traffic must arrive: without this the test could pass simply
    // because nothing works at all.
    let genuine = b"genuine-encrypted-datagram".to_vec();
    let delivered = tokio::time::timeout(TIMEOUT, async {
        loop {
            let _ = a.cmd_tx.send(Command::SendDatagram {
                virtual_ip: b_vip,
                src_port: 1000,
                dst_port: 1000,
                data: genuine.clone(),
                reliable: false,
            });
            let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
            while tokio::time::Instant::now() < deadline {
                match tokio::time::timeout(Duration::from_millis(250), b.evt_rx.recv()).await {
                    Ok(Some(Event::Datagram { data, .. })) if data == genuine => return true,
                    Ok(Some(_)) => continue,
                    Ok(None) => return false,
                    Err(_) => continue,
                }
            }
        }
    })
    .await
    .unwrap_or(false);
    assert!(
        delivered,
        "encrypted game traffic did not round-trip through the engine. This is the \
         assertion that actually proves the wiring: a send site that still emits \
         plaintext, or a seal/open mismatch, lands here."
    );

    // Now forge. This is precisely the old wire format — [type][src][dst][payload]
    // — which is what an attacker replaying a capture of the previous protocol,
    // or simply guessing the obvious framing, would send.
    let attacker = UdpSocket::bind(("127.0.0.1", 0)).await.expect("attacker socket");
    let forged_payload = b"INJECTED-BY-AN-ATTACKER".to_vec();
    let mut forged = Vec::new();
    forged.push(FRAME_DATA);
    forged.extend_from_slice(&1000u16.to_be_bytes());
    forged.extend_from_slice(&1000u16.to_be_bytes());
    forged.extend_from_slice(&forged_payload);

    // Also try a frame padded past the crypto overhead, so the rejection
    // cannot be dismissed as a mere length check.
    let mut forged_long = Vec::new();
    forged_long.push(FRAME_DATA);
    forged_long.extend_from_slice(&[0u8; 8]); // plausible counter
    forged_long.extend_from_slice(&forged_payload);
    forged_long.extend_from_slice(&[0u8; 16]); // plausible tag

    for _ in 0..5 {
        let _ = attacker.send_to(&forged, ("127.0.0.1", B_GAME_PORT)).await;
        let _ = attacker.send_to(&forged_long, ("127.0.0.1", B_GAME_PORT)).await;
    }

    // Give B every chance to surface it, then confirm it never did.
    let leaked = tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            match b.evt_rx.recv().await {
                Some(Event::Datagram { data, .. }) if data == forged_payload => return true,
                Some(_) => continue,
                None => return false,
            }
        }
    })
    .await
    .unwrap_or(false);

    assert!(
        !leaked,
        "a forged plaintext datagram reached the guest — the game plane accepts \
         unauthenticated traffic, so anyone on the path can inject into a session"
    );
}
