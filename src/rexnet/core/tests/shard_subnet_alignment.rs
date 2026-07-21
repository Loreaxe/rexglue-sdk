// @file        rexnet/core/tests/shard_subnet_alignment.rs
// @brief       Shard members share the shard's /24 — design spec §17.3.5.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Being in a shard has to mean being on its subnet.
//!
//! Addresses were previously assigned once and never revised, while a node's
//! *own* address was derived fresh on every read. A pair that met before
//! either had a shard therefore ended up split: the local address followed the
//! shard onto its /24 while the peer stayed behind on the reserved range. Both
//! nodes still talked, so nothing looked wrong — but subnet broadcast, the
//! whole point of a shard, can only reach the /24 the sender is on, so System
//! Link discovery had nobody to find.
//!
//! The order matters here: the nodes meet *before* enabling shards, which is
//! what a friend invite does and what leaves a stale address to correct.

use std::time::Duration;

use libp2p::Multiaddr;
use rexnet_core::engine::{self, Command, EngineConfig, EngineHandles, Event, VIP_NETWORK_BASE};
use rexnet_core::subnet;
use tokio::sync::{mpsc, oneshot};

/// Own title id so nodes from other tests running in parallel cannot join
/// this shard (see the note in shard_convergence.rs).
const TITLE: u32 = 0x4D53_0914;
const SETTLE_TIMEOUT: Duration = Duration::from_secs(30);
const ADDR_TIMEOUT: Duration = Duration::from_secs(30);

async fn spawn_node(dir: &std::path::Path) -> EngineHandles {
    engine::spawn(
        libp2p::identity::Keypair::generate_ed25519(),
        EngineConfig {
            data_dir: dir.to_path_buf(),
            title_id: TITLE,
            display_name: "align".into(),
            bootstrap: vec![],
            listen_port: 0,
            game_port: 0,
            relay: None,
            force_tunnel: false,
        },
    )
    .await
    .expect("engine start")
}

async fn shard_of(cmd_tx: &mpsc::UnboundedSender<Command>) -> Option<[u8; 16]> {
    let (tx, rx) = oneshot::channel();
    let _ = cmd_tx.send(Command::ShardStatus { reply: tx });
    rx.await.unwrap_or(None)
}

async fn await_shard(cmd_tx: &mpsc::UnboundedSender<Command>) -> Option<[u8; 16]> {
    tokio::time::timeout(SETTLE_TIMEOUT, async {
        loop {
            if let Some(id) = shard_of(cmd_tx).await {
                return id;
            }
            tokio::time::sleep(Duration::from_millis(200)).await;
        }
    })
    .await
    .ok()
}

/// Drain events, keeping the newest address this node holds for itself and for
/// `other`. Both are re-emitted when a shard change moves them, so the last
/// value seen is the one that counts.
async fn latest_addresses(
    node: &mut EngineHandles,
    other: libp2p::PeerId,
    settle: Duration,
) -> (Option<u32>, Option<u32>) {
    let mut mine = None;
    let mut theirs = None;
    let _ = tokio::time::timeout(settle, async {
        loop {
            match node.evt_rx.recv().await {
                Some(Event::LocalAddress { virtual_ip }) => mine = Some(virtual_ip),
                Some(Event::PeerConnected { peer, virtual_ip }) if peer == other => {
                    theirs = Some(virtual_ip);
                }
                Some(_) => {}
                None => break,
            }
        }
    })
    .await;
    (mine, theirs)
}

fn subnet_octet_of(vip: u32) -> u8 {
    ((vip >> 8) & 0xFF) as u8
}

fn tempdir() -> std::path::PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "rexnet-align-test-{}-{}",
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
async fn joining_a_shard_moves_peers_onto_its_subnet() {
    let dir_a = tempdir();
    let dir_b = tempdir();
    let mut a = spawn_node(&dir_a).await;
    let mut b = spawn_node(&dir_b).await;
    let peer_a = a.peer_id;
    let peer_b = b.peer_id;

    // Meet with no shard on either side: this is the friend-invite path, and
    // it is what leaves an address that later has to be corrected.
    tokio::time::sleep(Duration::from_millis(500)).await;
    let (tx, rx) = oneshot::channel();
    let _ = a.cmd_tx.send(Command::LocalAddrs { reply: tx });
    let addrs: Vec<Multiaddr> = rx.await.unwrap_or_default();
    assert!(!addrs.is_empty(), "node A never reported a listen address");
    for addr in addrs {
        let with_peer = addr.with(libp2p::multiaddr::Protocol::P2p(peer_a));
        let _ = b.cmd_tx.send(Command::ConnectManual { multiaddr: with_peer.to_string() });
    }
    tokio::time::sleep(Duration::from_secs(2)).await;

    let _ = a.cmd_tx.send(Command::ShardEnable { cap: 0 });
    let shard = await_shard(&a.cmd_tx).await.expect("A never settled into a shard");
    let _ = b.cmd_tx.send(Command::ShardEnable { cap: 0 });
    let shard_b = await_shard(&b.cmd_tx).await.expect("B never settled into a shard");
    assert_eq!(shard, shard_b, "nodes are not in the same shard");

    // Membership travels by heartbeat, so the readdress lands a beat later.
    let (a_mine, a_theirs) = latest_addresses(&mut a, peer_b, ADDR_TIMEOUT).await;
    let (b_mine, b_theirs) = latest_addresses(&mut b, peer_a, ADDR_TIMEOUT).await;

    let a_mine = a_mine.expect("A never reported its own address");
    let b_mine = b_mine.expect("B never reported its own address");
    let a_theirs = a_theirs.expect("A never reported an address for B");
    let b_theirs = b_theirs.expect("B never reported an address for A");

    let expected = subnet::subnet_octet(&shard);
    let reserved = subnet::RESERVED_SUBNET;

    // The bug: local address followed the shard, the peer stayed on 255.
    assert_eq!(
        subnet_octet_of(a_theirs),
        expected,
        "A still addresses B on {}.x (reserved={}) while the shard is {}.x",
        subnet_octet_of(a_theirs),
        reserved,
        expected
    );
    assert_eq!(
        subnet_octet_of(b_theirs),
        expected,
        "B still addresses A on {}.x while the shard is {}.x",
        subnet_octet_of(b_theirs),
        expected
    );
    assert_eq!(subnet_octet_of(a_mine), expected, "A's own address is off the shard subnet");
    assert_eq!(subnet_octet_of(b_mine), expected, "B's own address is off the shard subnet");

    // Same /24 is necessary but not sufficient: each side must also name the
    // other the way that side names itself, or broadcast lands on a stranger.
    assert_eq!(a_theirs, b_mine, "A and B disagree about B's address");
    assert_eq!(b_theirs, a_mine, "A and B disagree about A's address");
    assert_ne!(a_mine, b_mine, "both nodes claim the same address");

    // And the broadcast address is then reachable from where they actually are.
    let broadcast = subnet::broadcast_address(VIP_NETWORK_BASE, &shard);
    assert_eq!(subnet_octet_of(broadcast), expected);
    assert_eq!(broadcast & 0xFF, 255);
}
