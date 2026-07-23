//! rexnet-cli — small operator tool for the RexNet control plane.
//!
//! Minimal today: prints this build's version and the local identity (peer id
//! and friend code) for a data directory, so an operator can read the id off
//! without launching a title. This exists mainly so the `[[bin]]` target has a
//! real source file — a missing one breaks every `cargo build`/`cargo test` on
//! the crate. Richer subcommands (e.g. a DHT `find`) can grow here.

use std::path::PathBuf;

fn main() {
    let version = env!("CARGO_PKG_VERSION");
    let mut args = std::env::args().skip(1);
    match args.next().as_deref() {
        Some("id") => {
            let data_dir = args
                .next()
                .map(PathBuf::from)
                .unwrap_or_else(|| PathBuf::from("."));
            match rexnet_core::identity::load_or_generate(&data_dir) {
                Ok(keypair) => {
                    println!("peer id:     {}", keypair.public().to_peer_id());
                    println!("friend code: {}", rexnet_core::identity::friend_code(&keypair));
                }
                Err(err) => {
                    eprintln!("failed to load identity from {}: {err}", data_dir.display());
                    std::process::exit(1);
                }
            }
        }
        Some("version") | Some("--version") => println!("rexnet-cli {version}"),
        _ => {
            eprintln!("rexnet-cli {version}");
            eprintln!("usage:");
            eprintln!("  rexnet-cli id [data-dir]   print the local peer id and friend code");
            eprintln!("  rexnet-cli version         print the version");
            std::process::exit(2);
        }
    }
}
