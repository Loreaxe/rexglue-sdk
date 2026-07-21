/**
 * @file        net/virtual_ip_test.cpp
 * @brief       Virtual-IP table tests — peer <-> address mapping consistency.
 *
 * @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
 *              All rights reserved.
 *
 * @license     BSD 3-Clause License
 *              See LICENSE file in the project root for full license text.
 *
 * @remarks     A peer's address is not fixed: it is derived from shard
 *              membership, so entering or leaving a shard readdresses it. The
 *              table therefore has to survive the same peer arriving under a
 *              new address without leaving the old one behind — consumers
 *              enumerate it to count who is present, and a duplicate reads as
 *              an extra occupant.
 */

#include <catch2/catch_test_macros.hpp>

#include <cstdint>

#include <rex/net/virtual_ip.h>

using rex::net::VirtualIpTable;

namespace {

RexNetPeerId Peer(std::uint8_t tag) {
  RexNetPeerId peer{};
  peer.len = 32;
  for (std::uint8_t i = 0; i < peer.len; ++i) {
    peer.bytes[i] = static_cast<std::uint8_t>(tag + i);
  }
  return peer;
}

constexpr std::uint32_t kBase = VirtualIpTable::kNetworkBase;

}  // namespace

TEST_CASE("readdressing a peer leaves no duplicate", "[net][virtual_ip]") {
  VirtualIpTable table;
  const auto peer = Peer(1);

  table.Insert(peer, kBase | 0xFF00u | 10u);
  REQUIRE(table.Entries().size() == 1);

  // Same peer, new address -- what a shard change produces.
  table.Insert(peer, kBase | 0x1D00u | 10u);

  // The old address must not linger: Entries() is how callers count peers.
  CHECK(table.Entries().size() == 1);
  CHECK(table.Find(peer) == (kBase | 0x1D00u | 10u));
  CHECK_FALSE(table.Lookup(kBase | 0xFF00u | 10u).has_value());
  CHECK(table.Lookup(kBase | 0x1D00u | 10u).has_value());
}

TEST_CASE("repeated identical inserts stay idempotent", "[net][virtual_ip]") {
  VirtualIpTable table;
  const auto peer = Peer(7);
  const std::uint32_t vip = kBase | 0x2A00u | 3u;

  table.Insert(peer, vip);
  table.Insert(peer, vip);
  table.Insert(peer, vip);

  CHECK(table.Entries().size() == 1);
  CHECK(table.Find(peer) == vip);
}

TEST_CASE("an address handed to a new peer drops the old owner", "[net][virtual_ip]") {
  VirtualIpTable table;
  const auto first = Peer(1);
  const auto second = Peer(100);
  const std::uint32_t vip = kBase | 0x0500u | 9u;

  table.Insert(first, vip);
  table.Insert(second, vip);

  // One address, one owner -- the loser must not still resolve to it.
  CHECK(table.Entries().size() == 1);
  CHECK(table.Lookup(vip).has_value());
  CHECK(table.Find(second) == vip);
  CHECK_FALSE(table.Find(first).has_value());
}

TEST_CASE("online-key lookup follows the current address", "[net][virtual_ip]") {
  VirtualIpTable table;
  const auto peer = Peer(3);
  const auto id = Peer(3);

  table.Insert(peer, kBase | 0xFF00u | 4u);
  table.Insert(peer, kBase | 0x0900u | 4u);

  CHECK(table.FindByOnlineKey(id.bytes) == (kBase | 0x0900u | 4u));
}

TEST_CASE("remove clears every index", "[net][virtual_ip]") {
  VirtualIpTable table;
  const auto peer = Peer(5);
  const std::uint32_t vip = kBase | 0x1100u | 2u;

  table.Insert(peer, vip);
  table.Remove(vip);

  CHECK(table.Entries().empty());
  CHECK_FALSE(table.Find(peer).has_value());
  CHECK_FALSE(table.Lookup(vip).has_value());
  CHECK_FALSE(table.FindByOnlineKey(peer.bytes).has_value());
}
