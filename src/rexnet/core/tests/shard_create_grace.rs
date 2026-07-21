// @file        rexnet/core/tests/shard_create_grace.rs
// @brief       A node does not found a shard before discovery has answered.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! Don't found a shard in the gap before discovery answers.
//!
//! A node that enables shards starts looking immediately, and on an empty
//! routing table that lookup comes back at once. If an incumbent is already
//! out there but nothing has introduced the two yet, the newcomer founds a
//! shard and abandons it moments later. Convergence handles the collision
//! correctly, so nothing looks broken — but the shard id determines the
//! subnet, so the whole membership is re-addressed for a shard that lived
//! for one rescan.
//!
//! Seen on LAN as a founded shard abandoned 7 s later for one that was
//! already 133 s old.
//!
//! ## Why the test is shaped like this
//!
//! The ordering is the entire test, and two earlier attempts got it wrong in
//! ways that still passed:
//!
//! - Connecting the nodes before enabling shards lets the newcomer query the
//!   incumbent directly and join. Green with or without the grace.
//! - Sleeping before enabling shards achieves nothing either: these nodes
//!   share a host, so mDNS introduces them within a second whether or not
//!   anything calls `ConnectManual`.
//!
//! So B is brought up and left to found its shard first, and A is spawned and
//! set looking in the same breath — before discovery has had time to mention
//! B to it. That is the window the grace covers.

use std::collections::BTreeSet;
use std::time::Duration;

use rexnet_core::engine::{self, Command, EngineConfig, EngineHandles};
use tokio::sync::{mpsc, oneshot};

/// Own title id so nodes from other tests running in parallel cannot join
/// this shard (see the note in shard_convergence.rs).
const TITLE: u32 = 0x4D53_0915;
const CONVERGE_TIMEOUT: Duration = Duration::from_secs(60);
/// Long enough to cover the grace and the rescan that follows it, so a shard
/// founded and abandoned is caught either side of the transition.
const WATCH_WINDOW: Duration = Duration::from_secs(35);

async fn spawn_node(dir: &std::path::Path, name: &str) -> EngineHandles {
    engine::spawn(
        libp2p::identity::Keypair::generate_ed25519(),
        EngineConfig {
            data_dir: dir.to_path_buf(),
            title_id: TITLE,
            display_name: name.into(),
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

async fn await_shard(
    cmd_tx: &mpsc::UnboundedSender<Command>,
    timeout: Duration,
) -> Option<[u8; 16]> {
    tokio::time::timeout(timeout, async {
        loop {
            if let Some(id) = shard_of(cmd_tx).await {
                return id;
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    })
    .await
    .ok()
}

/// Wait for this node to be in `target` specifically.
///
/// Not "wait for any shard": without the grace the newcomer founds its own
/// within milliseconds and only migrates later, so a bare await would trip the
/// convergence check on timing and report the wrong fault. Convergence is not
/// what this test is about — how many shards it took to get there is.
async fn await_shard_eq(
    cmd_tx: &mpsc::UnboundedSender<Command>,
    target: [u8; 16],
    timeout: Duration,
) -> bool {
    tokio::time::timeout(timeout, async {
        loop {
            if shard_of(cmd_tx).await == Some(target) {
                return;
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    })
    .await
    .is_ok()
}

fn tempdir() -> std::path::PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "rexnet-grace-test-{}-{}",
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
async fn a_shard_is_not_founded_before_discovery_answers() {
    // The incumbent, alone, founds a shard and holds it.
    let dir_b = tempdir();
    let b = spawn_node(&dir_b, "grace-b").await;
    let _ = b.cmd_tx.send(Command::ShardEnable { cap: 0 });
    let b_shard = await_shard(&b.cmd_tx, CONVERGE_TIMEOUT)
        .await
        .expect("B never settled into a shard");

    // The newcomer starts looking the moment it exists, before anything has
    // introduced it to B.
    let dir_a = tempdir();
    let a = spawn_node(&dir_a, "grace-a").await;
    let _ = a.cmd_tx.send(Command::ShardEnable { cap: 0 });

    // Sample from the first instant, so a shard founded and dropped between
    // polls is still caught.
    let watch = {
        let cmd_tx = a.cmd_tx.clone();
        tokio::spawn(async move {
            let mut seen: BTreeSet<[u8; 16]> = BTreeSet::new();
            let deadline = tokio::time::Instant::now() + WATCH_WINDOW;
            while tokio::time::Instant::now() < deadline {
                if let Some(id) = shard_of(&cmd_tx).await {
                    seen.insert(id);
                }
                tokio::time::sleep(Duration::from_millis(25)).await;
            }
            seen
        })
    };

    assert!(
        await_shard_eq(&a.cmd_tx, b_shard, CONVERGE_TIMEOUT).await,
        "A never converged into B's shard"
    );

    let seen = watch.await.expect("watcher panicked");
    assert!(seen.contains(&b_shard), "watcher never observed A in B's shard");
    assert_eq!(
        seen.len(),
        1,
        "A held {} shards before settling -- it founded one while discovery \
         was still in flight rather than waiting, and every member was \
         re-addressed on the way through it (first bytes: {:02x?})",
        seen.len(),
        seen.iter().map(|id| id[0]).collect::<Vec<_>>()
    );
}
