//! Subnet broadcast reaches every shard member — design spec §17.3.6.
//!
//! This is the mechanism System Link rests on: a title discovers peers by
//! shouting at the subnet rather than naming a host. No System Link title is
//! available to test against, and a real one would exercise a good deal more
//! XDK surface than this, so the point here is narrower and worth stating:
//! it proves the **fan-out** works — that a datagram addressed to the
//! broadcast address is delivered to other members — not that any particular
//! game's System Link stack would be satisfied.
//!
//! Without this, broadcast delivery would be entirely unexercised code.

use std::time::Duration;

use libp2p::Multiaddr;
use rexnet_core::engine::{self, Command, EngineConfig, EngineHandles, Event, VIP_NETWORK_BASE};
use rexnet_core::subnet;
use tokio::sync::{mpsc, oneshot};

/// Own title id, so nodes from other tests running in parallel cannot join
/// this shard (see the note in shard_convergence.rs).
const TITLE: u32 = 0x4D53_0912;
const SETTLE_TIMEOUT: Duration = Duration::from_secs(30);
const DELIVERY_TIMEOUT: Duration = Duration::from_secs(45);

async fn spawn_node(dir: &std::path::Path) -> EngineHandles {
    engine::spawn(
        libp2p::identity::Keypair::generate_ed25519(),
        EngineConfig {
            data_dir: dir.to_path_buf(),
            title_id: TITLE,
            display_name: "bcast".into(),
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

fn tempdir() -> std::path::PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "rexnet-bcast-test-{}-{}",
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
async fn broadcast_reaches_the_other_shard_member() {
    let dir_a = tempdir();
    let dir_b = tempdir();
    let a = spawn_node(&dir_a).await;
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
    tokio::time::sleep(Duration::from_secs(2)).await;

    let _ = a.cmd_tx.send(Command::ShardEnable { cap: 0 });
    let shard = await_shard(&a.cmd_tx).await.expect("A never settled into a shard");
    let _ = b.cmd_tx.send(Command::ShardEnable { cap: 0 });
    let shard_b = await_shard(&b.cmd_tx).await.expect("B never settled into a shard");
    assert_eq!(shard, shard_b, "nodes are not in the same shard");

    // Members learn each other from heartbeats, and the first broadcast only
    // starts the punch it needs, so send repeatedly the way a discovering
    // title would rather than once.
    let broadcast = subnet::broadcast_address(VIP_NETWORK_BASE, &shard);
    let payload = b"rexnet-syslink-probe".to_vec();

    let received = tokio::time::timeout(DELIVERY_TIMEOUT, async {
        loop {
            let _ = a.cmd_tx.send(Command::SendDatagram {
                virtual_ip: broadcast,
                src_port: 3074,
                dst_port: 3074,
                data: payload.clone(),
                reliable: false,
            });
            // Drain anything B has for us before sending again.
            let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
            while tokio::time::Instant::now() < deadline {
                match tokio::time::timeout(Duration::from_millis(250), b.evt_rx.recv()).await {
                    Ok(Some(Event::Datagram { data, dst_port, .. })) if data == payload => {
                        return dst_port;
                    }
                    Ok(Some(_)) => continue,
                    Ok(None) => return 0,
                    Err(_) => continue,
                }
            }
        }
    })
    .await;

    let dst_port = received.expect(
        "broadcast never reached the other member -- fan-out or the lazy punch it \
         depends on is not working",
    );
    assert_eq!(dst_port, 3074, "delivered to the wrong port");
}
