# rexnet - Serverless Xbox Live replacement module (netplay)
# L3 XAM shim (C++) over rexnet-core (Rust, libp2p) — see
# docs/rexnet-design-spec.md. The Rust crate is built via corrosion
# (imported in thirdparty/CMakeLists.txt when REXGLUE_ENABLE_REXNET is ON).

corrosion_import_crate(
    MANIFEST_PATH "${CMAKE_CURRENT_SOURCE_DIR}/core/Cargo.toml"
    CRATE_TYPES staticlib
)

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
