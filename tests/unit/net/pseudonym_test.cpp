/**
 * @file        net/pseudonym_test.cpp
 * @brief       Pseudonym derivation tests — design spec §17.3.7.
 *
 * @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
 *              All rights reserved.
 *
 * @license     BSD 3-Clause License
 *              See LICENSE file in the project root for full license text.
 *
 * @remarks     These names are shown to every peer who is not your friend, so
 *              the properties that matter are: identical on every observer,
 *              stable across sessions, and evenly spread so a shard does not
 *              fill with duplicates.
 */

#include <catch2/catch_test_macros.hpp>

#include <map>
#include <set>
#include <string>

#include <rex/net/pseudonym.h>

using rex::net::Pseudonym;
using rex::net::PseudonymHash;
using rex::net::PseudonymWithDiscriminator;

namespace {

/// Peer id shaped like the real thing: multihash prefix then digest bytes.
RexNetPeerId MakePeer(uint8_t seed, uint8_t len = 34) {
  RexNetPeerId peer{};
  peer.len = len;
  peer.bytes[0] = 0x00;  // multihash code
  peer.bytes[1] = 0x24;  // length
  for (uint8_t i = 2; i < len; ++i) {
    // Cheap spread so distinct seeds differ in every byte, not just one.
    peer.bytes[i] = static_cast<uint8_t>(seed * 31u + i * 17u);
  }
  return peer;
}

}  // namespace

TEST_CASE("pseudonym is deterministic", "[net][pseudonym]") {
  const auto peer = MakePeer(7);
  REQUIRE(Pseudonym(peer) == Pseudonym(peer));

  // Same identity rebuilt from scratch must name identically -- this is what
  // makes every observer agree.
  REQUIRE(Pseudonym(MakePeer(7)) == Pseudonym(peer));
}

TEST_CASE("pseudonym reads as 'Adjective Noun'", "[net][pseudonym]") {
  const auto name = Pseudonym(MakePeer(1));
  const auto space = name.find(' ');
  REQUIRE(space != std::string::npos);
  REQUIRE(space > 0);
  REQUIRE(space + 1 < name.size());
  // Exactly two words.
  REQUIRE(name.find(' ', space + 1) == std::string::npos);
}

TEST_CASE("distinct peers usually get distinct names", "[net][pseudonym]") {
  // A full shard is 255 members (§17.3). With 4096 combinations a handful of
  // birthday collisions is expected and handled by the discriminator; what
  // would be a bug is systemic clustering.
  std::set<std::string> names;
  for (int i = 0; i < 255; ++i) {
    names.insert(Pseudonym(MakePeer(static_cast<uint8_t>(i))));
  }
  REQUIRE(names.size() > 230);
}

TEST_CASE("hash spreads across both wordlists", "[net][pseudonym]") {
  // Guards against a derivation that varies only the noun (or only the
  // adjective) -- which would still pass a naive uniqueness check.
  std::set<std::string> adjectives;
  std::set<std::string> nouns;
  for (int i = 0; i < 255; ++i) {
    const auto name = Pseudonym(MakePeer(static_cast<uint8_t>(i)));
    const auto space = name.find(' ');
    adjectives.insert(name.substr(0, space));
    nouns.insert(name.substr(space + 1));
  }
  REQUIRE(adjectives.size() > 30);
  REQUIRE(nouns.size() > 30);
}

TEST_CASE("length is part of the identity", "[net][pseudonym]") {
  // A shorter id that is a prefix of a longer one must not share its name.
  const auto shorter = MakePeer(3, 20);
  auto longer = MakePeer(3, 34);
  for (uint8_t i = 0; i < 20; ++i) {
    longer.bytes[i] = shorter.bytes[i];
  }
  REQUIRE(PseudonymHash(shorter) != PseudonymHash(longer));
}

TEST_CASE("discriminator leaves the first holder undecorated", "[net][pseudonym]") {
  const auto peer = MakePeer(11);
  REQUIRE(PseudonymWithDiscriminator(peer, 0) == Pseudonym(peer));
  // Humans count from one, so the second holder reads as "... 2".
  REQUIRE(PseudonymWithDiscriminator(peer, 1) == Pseudonym(peer) + " 2");
  REQUIRE(PseudonymWithDiscriminator(peer, 2) == Pseudonym(peer) + " 3");
}

TEST_CASE("empty peer id does not crash", "[net][pseudonym]") {
  RexNetPeerId peer{};
  peer.len = 0;
  REQUIRE_FALSE(Pseudonym(peer).empty());
}

TEST_CASE("oversized length is clamped to the buffer", "[net][pseudonym]") {
  // len is attacker-controllable over the wire; it must never read past
  // bytes[].
  RexNetPeerId peer{};
  peer.len = 255;  // > sizeof(bytes)
  REQUIRE_FALSE(Pseudonym(peer).empty());
}
