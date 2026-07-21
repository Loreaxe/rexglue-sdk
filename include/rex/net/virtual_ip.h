/**
 * @file        net/virtual_ip.h
 * @brief       Virtual-IP table for XNet address mapping (design spec §11).
 *
 * @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
 *              All rights reserved.
 *
 * @license     BSD 3-Clause License
 *              See LICENSE file in the project root for full license text.
 *
 * Each remote peer has an address in 10.77.0.0/16; XNetXnAddrToInAddr and
 * InAddrToXnAddr resolve through this table, and sendto/recvfrom on a
 * virtual IP routes to that peer's punched game socket. Allocation is owned
 * by rexnet-core (PeerConnected events carry the assigned address); this
 * table mirrors it for guest-side lookups.
 */
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rex/net/rexnet_ffi.h"

namespace rex::net {

class VirtualIpTable {
 public:
    /// 10.77.0.0/16, host byte order.
    static constexpr uint32_t kNetworkBase = (10u << 24) | (77u << 16);

    /// Record a core-assigned mapping (idempotent). Host byte order.
    void Insert(const RexNetPeerId& peer, uint32_t virtual_ip);

    std::optional<RexNetPeerId> Lookup(uint32_t virtual_ip) const;
    std::optional<uint32_t> Find(const RexNetPeerId& peer) const;
    /// All (virtual_ip, peer) mappings, for diagnostics/enumeration.
    std::vector<std::pair<uint32_t, RexNetPeerId>> Entries() const;
    /// Resolve a 20-byte XNADDR abOnline key (peer multihash prefix).
    std::optional<uint32_t> FindByOnlineKey(const uint8_t key[20]) const;

    void Remove(uint32_t virtual_ip);

 private:
    // Key: peer multihash bytes packed into a string (cheap, hashable).
    static std::string Key(const RexNetPeerId& peer);

    std::unordered_map<std::string, uint32_t> by_peer_;
    std::unordered_map<uint32_t, RexNetPeerId> by_ip_;
    // abOnline (20-byte multihash prefix) -> virtual IP
    std::unordered_map<std::string, uint32_t> by_online_key_;
};

}  // namespace rex::net
