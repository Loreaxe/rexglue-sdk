// @file        rexnet/core/tests/address_agreement.rs
// @brief       Two nodes agree on each other's virtual IPs off-shard.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Off-shard addressing must be derived, not allocated.
//!
//! Addresses used to come from a local counter, with every shardless node
//! calling itself 10.77.255.254. Two nodes then disagreed about who was who,
//! and both believed they were the same address — invisible on one machine
//! where each instance only ever consults its own table, and broken the moment
//! a title puts its own address in a packet.
//!
//! This is a derivation property, so loopback proves it.

use std::time::Duration;

use libp2p::Multiaddr;
use rexnet_core::engine::{self, Command, EngineConfig, EngineHandles, Event};
use tokio::sync::oneshot;

const TITLE: u32 = 0x4D53_0917;
const TIMEOUT: Duration = Duration::from_secs(30);

async fn spawn_node(dir: &std::path::Path) -> EngineHandles {
    engine::spawn(
        libp2p::identity::Keypair::generate_ed25519(),
        EngineConfig {
            data_dir: dir.to_path_buf(),
            title_id: TITLE,
            display_name: "addr".into(),
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
        "rexnet-addr-test-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&path).expect("temp dir");
    path
}

/// Collect this node's own address and the address it gives `other`.
async fn addresses(
    node: &mut EngineHandles,
    other: libp2p::PeerId,
) -> (Option<u32>, Option<u32>) {
    let mut mine = None;
    let mut theirs = None;
    let _ = tokio::time::timeout(TIMEOUT, async {
        loop {
            match node.evt_rx.recv().await {
                Some(Event::LocalAddress { virtual_ip }) => mine = Some(virtual_ip),
                Some(Event::PeerConnected { peer, virtual_ip }) if peer == other => {
                    theirs = Some(virtual_ip)
                }
                Some(_) => continue,
                None => return,
            }
            if mine.is_some() && theirs.is_some() {
                return;
            }
        }
    })
    .await;
    (mine, theirs)
}

#[tokio::test(flavor = "multi_thread")]
async fn two_nodes_agree_on_each_others_addresses() {
    let dir_a = tempdir();
    let dir_b = tempdir();
    let mut a = spawn_node(&dir_a).await;
    let mut b = spawn_node(&dir_b).await;

    tokio::time::sleep(Duration::from_millis(500)).await;
    let (tx, rx) = oneshot::channel();
    let _ = a.cmd_tx.send(Command::LocalAddrs { reply: tx });
    let addrs: Vec<Multiaddr> = rx.await.unwrap_or_default();
    assert!(!addrs.is_empty(), "node A never reported a listen address");
    for addr in addrs {
        let with_peer = addr.with(libp2p::multiaddr::Protocol::P2p(a.peer_id));
        let _ = b.cmd_tx.send(Command::ConnectManual { multiaddr: with_peer.to_string() });
    }

    // Punching is what makes each side announce the other.
    tokio::time::sleep(Duration::from_secs(2)).await;
    let b_peer = b.peer_id;
    let a_peer = a.peer_id;
    let _ = a.cmd_tx.send(Command::Punch { peer: b_peer, reply: None });
    let _ = b.cmd_tx.send(Command::Punch { peer: a_peer, reply: None });

    let (a_self, a_view_of_b) = addresses(&mut a, b_peer).await;
    let (b_self, b_view_of_a) = addresses(&mut b, a_peer).await;

    let a_self = a_self.expect("A never reported its own address");
    let b_self = b_self.expect("B never reported its own address");
    let a_view_of_b = a_view_of_b.expect("A never addressed B");
    let b_view_of_a = b_view_of_a.expect("B never addressed A");

    assert_ne!(
        a_self, b_self,
        "both nodes claim the same address ({a_self:#x}) — every shardless node \
         used to be 10.77.255.254, so no title could tell them apart"
    );
    assert_eq!(
        a_view_of_b, b_self,
        "A calls B {a_view_of_b:#x} but B calls itself {b_self:#x}; an address B \
         puts in a packet would not resolve back to B"
    );
    assert_eq!(
        b_view_of_a, a_self,
        "B calls A {b_view_of_a:#x} but A calls itself {a_self:#x}"
    );
}
