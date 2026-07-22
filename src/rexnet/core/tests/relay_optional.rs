// @file        rexnet/core/tests/relay_optional.rs
// @brief       Configured relays are an accelerant, never a dependency.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Hosted relays must never become a dependency — design spec §9.
//!
//! The whole argument for using someone else's infrastructure is that losing
//! it costs speed and not capability. That is easy to say and easy to break:
//! an unreachable relay that aborts startup, or a reservation attempt that
//! blocks discovery, turns an accelerant into a single point of failure and
//! nobody notices until the day it goes away.
//!
//! So: point two nodes at relays that do not exist, and require them to find
//! each other and carry traffic anyway.

use std::time::Duration;

use libp2p::Multiaddr;
use rexnet_core::engine::{self, Command, EngineConfig, EngineHandles, Event};
use tokio::sync::oneshot;

const TITLE: u32 = 0x4D53_0918;
const TIMEOUT: Duration = Duration::from_secs(30);

/// Syntactically valid, resolvable, and answered by nothing. Port 1 on the
/// documentation-range address is as dead as an address gets.
fn dead_relays() -> Vec<String> {
    vec![
        "/ip4/192.0.2.1/tcp/1/p2p/12D3KooWDpJ7As7BWAwRMfu1VU2WCqNjvq387JEYKDBj4kx6nXTN".into(),
        "/ip4/198.51.100.7/udp/1/quic-v1/p2p/12D3KooWSyH2eqBHRJQZBBHRJQZBBHRJQZBBHRJQZBBHRJQZ"
            .into(),
        "not even a multiaddr".into(),
    ]
}

async fn spawn_node(dir: &std::path::Path) -> EngineHandles {
    engine::spawn(
        libp2p::identity::Keypair::generate_ed25519(),
        EngineConfig {
            data_dir: dir.to_path_buf(),
            title_id: TITLE,
            display_name: "relay".into(),
            bootstrap: vec![],
            listen_port: 0,
            game_port: 0,
            relays: dead_relays(),
            force_tunnel: false,
        },
    )
    .await
    .expect("a dead relay must not stop the engine starting")
}

fn tempdir() -> std::path::PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "rexnet-relay-test-{}-{}",
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
async fn nodes_connect_and_play_with_every_relay_dead() {
    let dir_a = tempdir();
    let dir_b = tempdir();

    // Reaching here at all is the first assertion: `spawn` must not propagate
    // a relay failure. One unparseable entry is in the list on purpose.
    let mut a = spawn_node(&dir_a).await;
    let mut b = spawn_node(&dir_b).await;

    tokio::time::sleep(Duration::from_millis(500)).await;
    let (tx, rx) = oneshot::channel();
    let _ = a.cmd_tx.send(Command::LocalAddrs { reply: tx });
    let addrs: Vec<Multiaddr> = rx.await.unwrap_or_default();
    assert!(
        !addrs.is_empty(),
        "node A reported no listen address; a dead relay must not suppress the \
         direct listeners"
    );
    for addr in addrs {
        let with_peer = addr.with(libp2p::multiaddr::Protocol::P2p(a.peer_id));
        let _ = b.cmd_tx.send(Command::ConnectManual { multiaddr: with_peer.to_string() });
    }

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
    .expect("peers never connected with relays configured but unreachable");

    // Discovery surviving is not enough: the game plane has to work too.
    let payload = b"relays-are-not-load-bearing".to_vec();
    let delivered = tokio::time::timeout(TIMEOUT, async {
        loop {
            let _ = a.cmd_tx.send(Command::SendDatagram {
                virtual_ip: b_vip,
                src_port: 1000,
                dst_port: 1000,
                data: payload.clone(),
                reliable: false,
            });
            let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
            while tokio::time::Instant::now() < deadline {
                match tokio::time::timeout(Duration::from_millis(250), b.evt_rx.recv()).await {
                    Ok(Some(Event::Datagram { data, .. })) if data == payload => return true,
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
        "peers connected but no game traffic flowed with dead relays configured — \
         hosted infrastructure has become load-bearing"
    );
}
