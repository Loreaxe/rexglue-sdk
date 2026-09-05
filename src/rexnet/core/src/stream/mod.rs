// @file        rexnet/core/src/stream/mod.rs
// @brief       Generic stream-oriented libp2p behaviour (vendored).
//
// Vendored from libp2p-stream 0.4.0-alpha, Copyright (c) 2024 Protocol Labs,
// MIT licensed (https://github.com/libp2p/rust-libp2p). Paths rewritten to
// the `libp2p` facade crate and one deprecated `try_next` swapped for
// `try_recv`; behaviour unchanged.
//
// Why vendored rather than depended on: the crate has only ever shipped as an
// alpha, and it carries guest TCP (spec §18.1) and the degraded tunnel (§5).
// A preservation project cannot rest on a pre-release that may be yanked or
// change shape; 660 lines pinned in-tree will still build in ten years.

mod behaviour;
mod control;
mod handler;
mod shared;
mod upgrade;

pub use behaviour::{AlreadyRegistered, Behaviour};
pub use control::{Control, IncomingStreams, OpenStreamError};
