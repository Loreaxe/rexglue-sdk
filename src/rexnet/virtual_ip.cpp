/**
 * @file        virtual_ip.cpp
 * @brief       Virtual-IP table: peer <-> 10.77.0.0/16 address mapping.
 *
 * @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
 *              All rights reserved.
 *
 * @license     BSD 3-Clause License
 *              See LICENSE file in the project root for full license text.
 */
#include "rex/net/virtual_ip.h"

namespace rex::net {

std::string VirtualIpTable::Key(const RexNetPeerId& peer) {
  return std::string(reinterpret_cast<const char*>(peer.bytes), peer.len);
}

void VirtualIpTable::Insert(const RexNetPeerId& peer, uint32_t virtual_ip) {
  const std::string key = Key(peer);
  if (auto it = by_peer_.find(key); it != by_peer_.end() && it->second == virtual_ip) {
    return;
  }
  by_peer_[key] = virtual_ip;
  by_ip_[virtual_ip] = peer;
  by_online_key_[key.substr(0, 20)] = virtual_ip;
}

std::vector<std::pair<uint32_t, RexNetPeerId>> VirtualIpTable::Entries() const {
  std::vector<std::pair<uint32_t, RexNetPeerId>> out;
  out.reserve(by_ip_.size());
  for (const auto& [vip, peer] : by_ip_) {
    out.emplace_back(vip, peer);
  }
  return out;
}

std::optional<uint32_t> VirtualIpTable::FindByOnlineKey(const uint8_t key[20]) const {
  const std::string k(reinterpret_cast<const char*>(key), 20);
  if (auto it = by_online_key_.find(k); it != by_online_key_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::optional<RexNetPeerId> VirtualIpTable::Lookup(uint32_t virtual_ip) const {
  if (auto it = by_ip_.find(virtual_ip); it != by_ip_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::optional<uint32_t> VirtualIpTable::Find(const RexNetPeerId& peer) const {
  if (auto it = by_peer_.find(Key(peer)); it != by_peer_.end()) {
    return it->second;
  }
  return std::nullopt;
}

void VirtualIpTable::Remove(uint32_t virtual_ip) {
  if (auto it = by_ip_.find(virtual_ip); it != by_ip_.end()) {
    const std::string key = Key(it->second);
    by_peer_.erase(key);
    by_online_key_.erase(key.substr(0, 20));
    by_ip_.erase(it);
  }
}

}  // namespace rex::net
