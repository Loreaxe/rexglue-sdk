# rexnet - Serverless Xbox Live replacement module (netplay)
# L3 XAM shim (C++) over rexnet-core (Rust, libp2p) — see
# docs/rexnet-design-spec.md. The Rust crate is built via corrosion
# (imported in thirdparty/CMakeLists.txt when REXGLUE_ENABLE_REXNET is ON).

corrosion_import_crate(
    MANIFEST_PATH "${CMAKE_CURRENT_SOURCE_DIR}/core/Cargo.toml"
    CRATE_TYPES staticlib
)

# Keep cargo from oversubscribing the machine.
#
# Ninja sees the whole cargo invocation as *one* edge, so it happily runs its
# full parallel width of C++ compiles alongside it — while cargo, told
# nothing, spawns one rustc per core inside that single edge. On a 16-core
# host that is ~18 C++ jobs plus 16 rustc jobs at once. Ninja gained a GNU
# jobserver in 1.13 that would solve this properly; until the floor moves
# there, capping cargo is the portable fix. rustc is memory-hungry, so the
# failure this prevents is an OOM during a full build, not merely slow.
#
# 0 means "do not pass a limit" (cargo's own default: one job per core).
if(NOT DEFINED REXGLUE_REXNET_CARGO_JOBS)
    cmake_host_system_information(RESULT _rexnet_host_cores QUERY NUMBER_OF_LOGICAL_CORES)
    if(_rexnet_host_cores GREATER 1)
        math(EXPR _rexnet_default_jobs "${_rexnet_host_cores} / 2")
    else()
        set(_rexnet_default_jobs 1)
    endif()
    set(REXGLUE_REXNET_CARGO_JOBS "${_rexnet_default_jobs}" CACHE STRING
        "Parallel cargo jobs when building rexnet-core (0 = cargo's default)")
endif()

if(REXGLUE_REXNET_CARGO_JOBS GREATER 0)
    corrosion_set_env_vars(rexnet_core "CARGO_BUILD_JOBS=${REXGLUE_REXNET_CARGO_JOBS}")
    message(STATUS "RexNet: cargo limited to ${REXGLUE_REXNET_CARGO_JOBS} parallel jobs")
endif()

add_library(rexnet OBJECT
    rexnet.cpp
    pseudonym.cpp
    virtual_ip.cpp
)
add_library(rex::rexnet ALIAS rexnet)

target_include_directories(rexnet PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Corrosion names the imported target after the crate's [lib] name.
target_link_libraries(rexnet
    PUBLIC rexcore
    PRIVATE rexnet_core tomlplusplus
)

# The Rust staticlib pulls in libc/pthread/dl on Linux; corrosion handles
# most of it, but Windows needs the usual socket/crypt system libs.
if(WIN32)
    target_link_libraries(rexnet PRIVATE ws2_32 bcrypt ntdll userenv)
endif()
