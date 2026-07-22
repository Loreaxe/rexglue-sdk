// @file        rexnet/core/tests/guest_tcp.rs
// @brief       Guest TCP over RexNet.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Guest TCP over RexNet (§18.1): a connection to a peer's virtual IP arrives
//! addressed to the right guest port, and bytes travel both ways.
//!
//! Does not prove any title is satisfied; that needs a recompiled System Link
//! game.

use std::time::Duration;

use libp2p::Multiaddr;
use rexnet_core::engine::{self, Command, EngineConfig, EngineHandles, Event};
use tokio::sync::oneshot;

const TITLE: u32 = 0x4D53_0914;
const TIMEOUT: Duration = Duration::from_secs(30);
/// The port SoulCalibur IV actually listens on, for fidelity.
const GUEST_LISTEN_PORT: u16 = 1001;
const GUEST_CLIENT_PORT: u16 = 49152;

async fn spawn_node(dir: &std::path::Path) -> EngineHandles {
    engine::spawn(
        libp2p::identity::Keypair::generate_ed25519(),
        EngineConfig {
            data_dir: dir.to_path_buf(),
            title_id: TITLE,
            display_name: "tcp".into(),
            bootstrap: vec![],
            listen_port: 0,
            game_port: 0,
            relays: Vec::new(),
            force_tunnel: false,
        },
    )
    .await
    .expect("engine start")
}

fn tempdir() -> std::path::PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "rexnet-tcp-test-{}-{}",
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
async fn guest_tcp_connects_and_carries_bytes_both_ways() {
    let dir_a = tempdir();
    let dir_b = tempdir();
    let mut a = spawn_node(&dir_a).await;
    let mut b = spawn_node(&dir_b).await;

    // Connect the control plane; the stream rides it.
    tokio::time::sleep(Duration::from_millis(500)).await;
    let (tx, rx) = oneshot::channel();
    let _ = a.cmd_tx.send(Command::LocalAddrs { reply: tx });
    let addrs: Vec<Multiaddr> = rx.await.unwrap_or_default();
    assert!(!addrs.is_empty(), "node A never reported a listen address");
    for addr in addrs {
        let with_peer = addr.with(libp2p::multiaddr::Protocol::P2p(a.peer_id));
        let _ = b.cmd_tx.send(Command::ConnectManual { multiaddr: with_peer.to_string() });
    }

    // B learns A's virtual IP from PeerConnected -- the same address a guest
    // would have resolved through XNetXnAddrToInAddr.
    let a_vip = tokio::time::timeout(TIMEOUT, async {
        loop {
            match b.evt_rx.recv().await {
                Some(Event::PeerConnected { peer, virtual_ip }) if peer == a.peer_id => {
                    return Some(virtual_ip);
                }
                Some(_) => continue,
                None => return None,
            }
        }
    })
    .await
    .ok()
    .flatten()
    .expect("B never learned A's virtual IP");

    // B connects to A's "listening" guest port.
    let _ = b.cmd_tx.send(Command::StreamConnect {
        virtual_ip: a_vip,
        src_port: GUEST_CLIENT_PORT,
        dst_port: GUEST_LISTEN_PORT,
    });

    // A should see an inbound connection addressed to the port it listens on.
    let (a_stream, a_local, a_remote) = tokio::time::timeout(TIMEOUT, async {
        loop {
            match a.evt_rx.recv().await {
                Some(Event::StreamOpened { stream_id, local_port, remote_port, outbound, .. }) => {
                    assert!(!outbound, "A accepted, so this must not be outbound");
                    return Some((stream_id, local_port, remote_port));
                }
                Some(_) => continue,
                None => return None,
            }
        }
    })
    .await
    .ok()
    .flatten()
    .expect("A never accepted the guest TCP connection");

    // The header has to survive: without it a stream says which peer but never
    // which listener, and the shim could not match it to a socket.
    assert_eq!(a_local, GUEST_LISTEN_PORT, "accepted on the wrong guest port");
    assert_eq!(a_remote, GUEST_CLIENT_PORT, "wrong peer port reported");

    let b_stream = tokio::time::timeout(TIMEOUT, async {
        loop {
            match b.evt_rx.recv().await {
                Some(Event::StreamOpened { stream_id, outbound, .. }) => {
                    assert!(outbound, "B opened this one");
                    return Some(stream_id);
                }
                Some(_) => continue,
                None => return None,
            }
        }
    })
    .await
    .ok()
    .flatten()
    .expect("B never saw its own connection open");

    // B -> A
    let to_a = b"hello-from-the-client".to_vec();
    let _ = b.cmd_tx.send(Command::StreamSend { stream_id: b_stream, data: to_a.clone() });
    let got = tokio::time::timeout(TIMEOUT, async {
        loop {
            match a.evt_rx.recv().await {
                Some(Event::StreamData { stream_id, data }) if stream_id == a_stream => {
                    return Some(data);
                }
                Some(_) => continue,
                None => return None,
            }
        }
    })
    .await
    .ok()
    .flatten();
    assert_eq!(got.as_deref(), Some(to_a.as_slice()), "client -> server bytes lost");

    // A -> B, proving the stream is genuinely bidirectional rather than a
    // one-way pipe that happened to work in the first direction.
    let to_b = b"hello-from-the-server".to_vec();
    let _ = a.cmd_tx.send(Command::StreamSend { stream_id: a_stream, data: to_b.clone() });
    let got = tokio::time::timeout(TIMEOUT, async {
        loop {
            match b.evt_rx.recv().await {
                Some(Event::StreamData { stream_id, data }) if stream_id == b_stream => {
                    return Some(data);
                }
                Some(_) => continue,
                None => return None,
            }
        }
    })
    .await
    .ok()
    .flatten();
    assert_eq!(got.as_deref(), Some(to_b.as_slice()), "server -> client bytes lost");

    // Closing one end must surface on the other, or a guest socket would wait
    // forever on a connection that is already gone.
    let _ = b.cmd_tx.send(Command::StreamClose { stream_id: b_stream });
    let closed = tokio::time::timeout(TIMEOUT, async {
        loop {
            match a.evt_rx.recv().await {
                Some(Event::StreamClosed { stream_id }) if stream_id == a_stream => return true,
                Some(_) => continue,
                None => return false,
            }
        }
    })
    .await
    .unwrap_or(false);
    assert!(closed, "peer close never reached the other end");
}
