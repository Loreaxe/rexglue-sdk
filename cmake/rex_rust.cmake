# Rust toolchain gate for the RexNet netplay module.
#
# RexNet's control plane is a Rust crate (src/rexnet/core) built through
# corrosion. Rust is *not* a general requirement of the SDK — it is needed
# only when REXGLUE_ENABLE_REXNET is ON, which is why this is a gate rather
# than a top-level dependency.
#
# Without this check, a missing toolchain surfaces as a corrosion failure
# deep in its own configure step that never names Rust as the cause, and a
# too-old toolchain surfaces even later as a wall of compile errors inside
# somebody else's crate. Both are the same user mistake and deserve to be
# reported once, at configure time, in terms of what to actually do.

# Minimum usable Rust, derived from the dependency tree rather than guessed:
# `time` 0.3.53 declares rust-version 1.88.0 and is the highest in our
# lockfile. Re-derive after a dependency bump — do not raise it on a hunch.
set(REXGLUE_REXNET_MIN_RUST "1.88.0" CACHE STRING
    "Minimum Rust toolchain version required to build the RexNet module")
mark_as_advanced(REXGLUE_REXNET_MIN_RUST)

function(rexglue_require_rust_toolchain)
    find_program(REXGLUE_CARGO_EXECUTABLE cargo)
    find_program(REXGLUE_RUSTC_EXECUTABLE rustc)

    if(NOT REXGLUE_CARGO_EXECUTABLE OR NOT REXGLUE_RUSTC_EXECUTABLE)
        message(FATAL_ERROR
            "REXGLUE_ENABLE_REXNET is ON, but no Rust toolchain was found.\n"
            "  cargo: ${REXGLUE_CARGO_EXECUTABLE}\n"
            "  rustc: ${REXGLUE_RUSTC_EXECUTABLE}\n"
            "\n"
            "RexNet's control plane is written in Rust. Either install a "
            "toolchain (>= ${REXGLUE_REXNET_MIN_RUST}):\n"
            "    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh\n"
            "\n"
            "or build without netplay:\n"
            "    cmake -DREXGLUE_ENABLE_REXNET=OFF ...\n"
            "\n"
            "The SDK builds and runs fully without RexNet; the netplay module "
            "is compiled out and its XAM entry points fall back to their "
            "offline behaviour."
        )
    endif()

    execute_process(
        COMMAND "${REXGLUE_RUSTC_EXECUTABLE}" --version
        OUTPUT_VARIABLE _rustc_version_output
        ERROR_VARIABLE _rustc_version_error
        RESULT_VARIABLE _rustc_version_result
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if(NOT _rustc_version_result EQUAL 0)
        message(FATAL_ERROR
            "Found rustc at ${REXGLUE_RUSTC_EXECUTABLE}, but running it failed:\n"
            "  ${_rustc_version_error}\n"
            "A rustup shim on PATH with no toolchain installed behaves this "
            "way; `rustup default stable` usually fixes it."
        )
    endif()

    # "rustc 1.91.1 (ed61e7d7e 2025-11-07)" -> 1.91.1
    if(NOT _rustc_version_output MATCHES "rustc ([0-9]+\\.[0-9]+\\.[0-9]+)")
        # Nightly and custom builds can carry suffixes we do not model. Warn
        # rather than fail: refusing to build on an unparseable-but-working
        # toolchain would be worse than trying and letting cargo object.
        message(WARNING
            "Could not parse a version from '${_rustc_version_output}'; "
            "skipping the Rust ${REXGLUE_REXNET_MIN_RUST} minimum check."
        )
        return()
    endif()
    set(_rustc_version "${CMAKE_MATCH_1}")

    if(_rustc_version VERSION_LESS REXGLUE_REXNET_MIN_RUST)
        message(FATAL_ERROR
            "Rust ${_rustc_version} is too old to build RexNet "
            "(need >= ${REXGLUE_REXNET_MIN_RUST}).\n"
            "  rustc: ${REXGLUE_RUSTC_EXECUTABLE}\n"
            "\n"
            "Update it:\n"
            "    rustup update stable\n"
            "\n"
            "or build without netplay:\n"
            "    cmake -DREXGLUE_ENABLE_REXNET=OFF ..."
        )
    endif()

    set(REXGLUE_RUST_VERSION "${_rustc_version}" PARENT_SCOPE)
    message(STATUS "RexNet: Rust ${_rustc_version} (${REXGLUE_CARGO_EXECUTABLE})")
endfunction()
