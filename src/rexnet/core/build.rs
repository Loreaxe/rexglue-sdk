// @file        rexnet/core/build.rs
// @brief       Regenerates the C header from src/ffi.rs on every build.
//
// @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
//              All rights reserved.
//
// @license     BSD 3-Clause License
//              See LICENSE file in the project root for full license text.

//! The checked-in `include/rex/net/rexnet_ffi.h` is generated output. It is
//! rewritten here whenever the ABI in `src/ffi.rs` changes, so a C++ build
//! that follows a cargo build always sees the current ABI, and a stale
//! header shows up as a diff in `git status` (and fails CI) rather than as a
//! silent layout mismatch at runtime.

use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=cbindgen.toml");
    println!("cargo:rerun-if-changed=src/ffi.rs");

    let crate_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let header = crate_dir.join("../../../include/rex/net/rexnet_ffi.h");
    let config = cbindgen::Config::from_file(crate_dir.join("cbindgen.toml"))
        .expect("cbindgen.toml is well-formed");

    let bindings = match cbindgen::Builder::new()
        // Only ffi.rs is the ABI; parsing the whole crate would export every
        // pub const in every module.
        .with_src(crate_dir.join("src/ffi.rs"))
        .with_config(config)
        .generate()
    {
        Ok(bindings) => bindings,
        Err(err) => {
            // A parse failure must not take the whole build down: the last
            // good header still describes the ABI unless ffi.rs changed, and
            // CI's drift check catches that case.
            println!("cargo:warning=cbindgen failed; rexnet_ffi.h left as-is: {err}");
            return;
        }
    };

    let mut generated = Vec::new();
    bindings.write(&mut generated);
    let current = std::fs::read(&header).unwrap_or_default();
    if generated != current {
        std::fs::write(&header, &generated).expect("write include/rex/net/rexnet_ffi.h");
        println!("cargo:warning=rexnet_ffi.h regenerated from src/ffi.rs; commit the updated header");
    }
}
