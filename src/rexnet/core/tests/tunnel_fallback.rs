// @file        rexnet/core/tests/tunnel_fallback.rs
// @brief       Game traffic still flows when the punch fails.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Game traffic still flows when the punch fails (§5, §14).
//!
//! On a loopback machine every punch succeeds, so the fallback would never
//! run — hence `force_tunnel`, which is also how a user reproduces a CGNAT
//! player's experience locally.

use std::time::Duration;

use libp2p::Multiaddr;
use rexnet_core::engine::{self, Command, EngineConfig, EngineHandles, Event};
use tokio::sync::oneshot;

/// Per-test title ids: tests run in parallel and mDNS would otherwise let one
/// test's nodes discover another's, which has contaminated results before.
const TITLE_FALLBACK: u32 = 0x4D53_0913;
const TITLE_SUSTAINED: u32 = 0x4D53_0915;
/// Punch timeout is 6 s; allow for the watchdog tick and a loaded machine.
const FALLBACK_TIMEOUT: Duration = Duration::from_secs(30);

async fn spawn_node(dir: &std::path::Path, title_id: u32, force_tunnel: bool) -> EngineHandles {
    engine::spawn(
        libp2p::identity::Keypair::generate_ed25519(),
        EngineConfig {
            data_dir: dir.to_path_buf(),
            title_id,
            display_name: "tunnel".into(),
            bootstrap: vec![],
            listen_port: 0,
            game_port: 0,
            relay: None,
            force_tunnel,
        },
    )
    .await
    .expect("engine start")
}

fn tempdir() -> std::path::PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "rexnet-tunnel-test-{}-{}",
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
async fn traffic_falls_back_to_the_tunnel_when_the_punch_fails() {
    let dir_a = tempdir();
    let dir_b = tempdir();
    // A cannot punch; B is a normal node, which is the realistic shape --
    // one player behind CGNAT talking to one who is not.
    let mut a = spawn_node(&dir_a, TITLE_FALLBACK, true).await;
    let mut b = spawn_node(&dir_b, TITLE_FALLBACK, false).await;

    // Control plane: connect them normally. This is what the tunnel rides.
    tokio::time::sleep(Duration::from_millis(500)).await;
    let (tx, rx) = oneshot::channel();
    let _ = a.cmd_tx.send(Command::LocalAddrs { reply: tx });
    let addrs: Vec<Multiaddr> = rx.await.unwrap_or_default();
    assert!(!addrs.is_empty(), "node A never reported a listen address");
    for addr in addrs {
        let with_peer = addr.with(libp2p::multiaddr::Protocol::P2p(a.peer_id));
        let _ = b.cmd_tx.send(Command::ConnectManual { multiaddr: with_peer.to_string() });
    }
    tokio::time::sleep(Duration::from_secs(2)).await;

    // Ask A to reach B on the game plane. With force_tunnel it never punches.
    let _ = a.cmd_tx.send(Command::Punch { peer: b.peer_id, reply: None });

    // A should give up and report the link degraded rather than stalling.
    // B's virtual IP is learned from PeerConnected on the way past -- it is
    // how the guest would address B, and the tunnel has to accept the same.
    let mut b_vip: Option<u32> = None;
    let degraded = tokio::time::timeout(FALLBACK_TIMEOUT, async {
        loop {
            match a.evt_rx.recv().await {
                Some(Event::PeerConnected { peer, virtual_ip }) if peer == b.peer_id => {
                    b_vip = Some(virtual_ip);
                }
                Some(Event::Degraded { peer }) if peer == b.peer_id => return true,
                Some(_) => continue,
                None => return false,
            }
        }
    })
    .await
    .unwrap_or(false);
    assert!(
        degraded,
        "punch failed but no Degraded event -- the fallback never engaged, so a \
         CGNAT player would just silently fail"
    );
    let b_vip = b_vip.expect("A never allocated a virtual IP for B");

    // The point of degrading: traffic still gets through. B's virtual IP for A
    // is picked up on the way past -- the reply leg below needs it.
    let payload = b"tunneled-datagram".to_vec();
    let mut a_vip: Option<u32> = None;
    let delivered = tokio::time::timeout(FALLBACK_TIMEOUT, async {
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
                    Ok(Some(Event::PeerConnected { peer, virtual_ip })) if peer == a.peer_id => {
                        a_vip = Some(virtual_ip);
                    }
                    Ok(Some(Event::Datagram { data, src_port, dst_port, virtual_ip })) => {
                        if data == payload {
                            // Ports must survive the tunnel: the shim routes
                            // on dst_port, so a frame that arrives on the
                            // wrong one reaches the wrong guest socket.
                            assert_eq!(src_port, 1000, "src port mangled by the tunnel");
                            assert_eq!(dst_port, 1000, "dst port mangled by the tunnel");
                            a_vip = Some(virtual_ip);
                            return true;
                        }
                    }
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
        "Degraded was reported but no datagram arrived over the tunnel -- the \
         fallback exists in name only"
    );
    let a_vip = a_vip.expect("B never learned A's virtual IP");

    // The reply leg. B never punched and never degraded on its own, so it only
    // knows to use the tunnel because a tunneled frame arrived from A. Without
    // that inference B's replies would sit in the punch queue until they aged
    // out, and a session would be one-way -- which is no session at all.
    let reply = b"tunneled-reply".to_vec();
    let replied = tokio::time::timeout(FALLBACK_TIMEOUT, async {
        loop {
            let _ = b.cmd_tx.send(Command::SendDatagram {
                virtual_ip: a_vip,
                src_port: 1000,
                dst_port: 1000,
                data: reply.clone(),
                reliable: false,
            });
            let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
            while tokio::time::Instant::now() < deadline {
                match tokio::time::timeout(Duration::from_millis(250), a.evt_rx.recv()).await {
                    Ok(Some(Event::Datagram { data, .. })) if data == reply => return true,
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
        replied,
        "A's tunneled datagram reached B, but B's reply never came back -- the \
         tunnel carries traffic in one direction only"
    );
}

/// 200 datagrams at a game-like rate, each arriving exactly once.
///
/// A regression guard, not a reproduction of the substream exhaustion that
/// motivated the rewrite — on loopback the old carrier would likely have
/// passed this too. What it could not pass is the arithmetic: 200 substreams
/// against a per-connection cap, where this opens one.
#[tokio::test(flavor = "multi_thread")]
async fn sustained_tunnel_traffic_survives() {
    const COUNT: u32 = 200;

    let dir_a = tempdir();
    let dir_b = tempdir();
    let mut a = spawn_node(&dir_a, TITLE_SUSTAINED, true).await;
    let mut b = spawn_node(&dir_b, TITLE_SUSTAINED, false).await;

    tokio::time::sleep(Duration::from_millis(500)).await;
    let (tx, rx) = oneshot::channel();
    let _ = a.cmd_tx.send(Command::LocalAddrs { reply: tx });
    let addrs: Vec<Multiaddr> = rx.await.unwrap_or_default();
    assert!(!addrs.is_empty(), "node A never reported a listen address");
    for addr in addrs {
        let with_peer = addr.with(libp2p::multiaddr::Protocol::P2p(a.peer_id));
        let _ = b.cmd_tx.send(Command::ConnectManual { multiaddr: with_peer.to_string() });
    }
    tokio::time::sleep(Duration::from_secs(2)).await;

    let _ = a.cmd_tx.send(Command::Punch { peer: b.peer_id, reply: None });
    let b_vip = tokio::time::timeout(FALLBACK_TIMEOUT, async {
        let mut vip = None;
        loop {
            match a.evt_rx.recv().await {
                Some(Event::PeerConnected { peer, virtual_ip }) if peer == b.peer_id => {
                    vip = Some(virtual_ip);
                }
                Some(Event::Degraded { peer }) if peer == b.peer_id => return vip,
                Some(_) => continue,
                None => return None,
            }
        }
    })
    .await
    .ok()
    .flatten()
    .expect("A never degraded onto the tunnel");

    // Paced rather than dumped: a burst larger than the writer queue is
    // legitimately allowed to drop (the guest thinks this is UDP), so firing
    // everything at once would test the drop policy, not the carrier.
    tokio::spawn(async move {
        for seq in 0..COUNT {
            let _ = a.cmd_tx.send(Command::SendDatagram {
                virtual_ip: b_vip,
                src_port: 1000,
                dst_port: 1000,
                data: seq.to_be_bytes().to_vec(),
                reliable: false,
            });
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        // Keep the handle alive until every datagram is sent; dropping cmd_tx
        // would shut the engine down mid-run.
        tokio::time::sleep(FALLBACK_TIMEOUT).await;
    });

    let mut seen = vec![false; COUNT as usize];
    let mut count = 0usize;
    let all = tokio::time::timeout(FALLBACK_TIMEOUT, async {
        loop {
            match b.evt_rx.recv().await {
                Some(Event::Datagram { data, .. }) if data.len() == 4 => {
                    let seq = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
                    assert!(seq < COUNT, "datagram {seq} was never sent");
                    assert!(!seen[seq as usize], "datagram {seq} arrived twice");
                    seen[seq as usize] = true;
                    count += 1;
                    if count == COUNT as usize {
                        return true;
                    }
                }
                Some(_) => continue,
                None => return false,
            }
        }
    })
    .await
    .unwrap_or(false);

    assert!(
        all,
        "only {count}/{COUNT} datagrams survived the tunnel -- sustained traffic \
         is still being lost, which is what the per-datagram substream did"
    );
}
