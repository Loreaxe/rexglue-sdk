//! Two live nodes must end up in the same ambient shard — design spec §17.3.
//!
//! `shard.rs` unit-tests the placement and convergence *rules* over synthetic
//! candidates. It cannot see the swarm, and every bug hit while wiring this
//! feature lived exactly there: provider results being dispatched to the wrong
//! pending query, and a shard being created before discovery had answered.
//! Both produced a permanently fragmented population while every unit test
//! stayed green. This exercises the real path: DHT provider records, the
//! descriptor request-response, and the settle ordering.
//!
//! Loopback only, no bootstrap — with an empty bootstrap list the engine runs
//! kad in server mode, so provider records work peer-to-peer.

use std::time::Duration;

use libp2p::Multiaddr;
use rexnet_core::engine::{self, Command, EngineConfig, EngineHandles};
use tokio::sync::{mpsc, oneshot};

/// Each test uses its **own** title id. These nodes run on one machine with
/// mDNS on, so tests executing in parallel discover each other's nodes.
/// Shard keys, placement and the descriptor query are all title-scoped, so a
/// distinct id per test makes them hermetic -- and incidentally exercises
/// that filtering. Sharing one id here makes the suite pass alone and fail
/// under `cargo test`.
const TITLE_SEQUENTIAL: u32 = 0x4D53_0910;
const TITLE_SIMULTANEOUS: u32 = 0x4D53_0911;
/// Generous: a DHT sweep plus a descriptor round trip on a loaded CI box.
const SETTLE_TIMEOUT: Duration = Duration::from_secs(30);

async fn spawn_node(dir: &std::path::Path, title_id: u32) -> EngineHandles {
    let keypair = libp2p::identity::Keypair::generate_ed25519();
    engine::spawn(
        keypair,
        EngineConfig {
            data_dir: dir.to_path_buf(),
            title_id,
            display_name: "test".into(),
            // No bootstrap: stay off the public DHT entirely.
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

async fn local_addrs(cmd_tx: &mpsc::UnboundedSender<Command>) -> Vec<Multiaddr> {
    let (tx, rx) = oneshot::channel();
    let _ = cmd_tx.send(Command::LocalAddrs { reply: tx });
    rx.await.unwrap_or_default()
}

async fn shard_of(cmd_tx: &mpsc::UnboundedSender<Command>) -> Option<[u8; 16]> {
    let (tx, rx) = oneshot::channel();
    let _ = cmd_tx.send(Command::ShardStatus { reply: tx });
    rx.await.unwrap_or(None)
}

/// Poll until the node reports a shard, or give up.
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

#[tokio::test(flavor = "multi_thread")]
async fn two_nodes_converge_on_one_shard() {
    let dir_a = tempdir();
    let dir_b = tempdir();
    let a = spawn_node(&dir_a, TITLE_SEQUENTIAL).await;
    let b = spawn_node(&dir_b, TITLE_SEQUENTIAL).await;

    // Let listeners come up, then wire B to A directly. mDNS would usually do
    // this, but an explicit dial keeps the test independent of the sandbox's
    // multicast behaviour.
    tokio::time::sleep(Duration::from_millis(500)).await;
    let addrs = local_addrs(&a.cmd_tx).await;
    assert!(!addrs.is_empty(), "node A never reported a listen address");
    for addr in addrs {
        let with_peer = addr.with(libp2p::multiaddr::Protocol::P2p(a.peer_id));
        let _ = b.cmd_tx.send(Command::ConnectManual { multiaddr: with_peer.to_string() });
    }
    tokio::time::sleep(Duration::from_secs(2)).await;

    // A opts in first and, finding nothing, founds a shard.
    let _ = a.cmd_tx.send(Command::ShardEnable { cap: 0 });
    let shard_a = await_shard(&a.cmd_tx).await.expect("A never settled into a shard");

    // B opts in second and must *find* A's shard rather than founding its own.
    let _ = b.cmd_tx.send(Command::ShardEnable { cap: 0 });
    let shard_b = await_shard(&b.cmd_tx).await.expect("B never settled into a shard");

    assert_eq!(
        shard_a, shard_b,
        "nodes landed in different shards -- the population is fragmented \
         (A={shard_a:02x?} B={shard_b:02x?})"
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn simultaneous_enable_still_converges() {
    // The split-brain case, for real: both nodes look at once, both see
    // nothing, both create. They must not stay split. This is the scenario
    // §17.3.3 exists for, and the one the size-guard bug broke.
    let dir_a = tempdir();
    let dir_b = tempdir();
    let a = spawn_node(&dir_a, TITLE_SIMULTANEOUS).await;
    let b = spawn_node(&dir_b, TITLE_SIMULTANEOUS).await;

    tokio::time::sleep(Duration::from_millis(500)).await;
    let addrs = local_addrs(&a.cmd_tx).await;
    assert!(!addrs.is_empty(), "node A never reported a listen address");
    for addr in addrs {
        let with_peer = addr.with(libp2p::multiaddr::Protocol::P2p(a.peer_id));
        let _ = b.cmd_tx.send(Command::ConnectManual { multiaddr: with_peer.to_string() });
    }
    tokio::time::sleep(Duration::from_secs(2)).await;

    let _ = a.cmd_tx.send(Command::ShardEnable { cap: 0 });
    let _ = b.cmd_tx.send(Command::ShardEnable { cap: 0 });

    let shard_a = await_shard(&a.cmd_tx).await.expect("A never settled");
    let shard_b = await_shard(&b.cmd_tx).await.expect("B never settled");

    if shard_a == shard_b {
        return; // Converged immediately; nothing to reconcile.
    }

    // They split, which is legal. Convergence is then the rescan's job, and
    // migration is probabilistic (§17.3.3 jitter), so allow several rounds.
    let deadline = tokio::time::Instant::now() + Duration::from_secs(240);
    loop {
        tokio::time::sleep(Duration::from_secs(5)).await;
        let (now_a, now_b) = (shard_of(&a.cmd_tx).await, shard_of(&b.cmd_tx).await);
        if now_a.is_some() && now_a == now_b {
            return;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "split never healed: A={now_a:02x?} B={now_b:02x?}"
        );
    }
}

/// Minimal scratch dir; avoids a dev-dependency just for this.
fn tempdir() -> std::path::PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "rexnet-shard-test-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&path).expect("temp dir");
    path
}
