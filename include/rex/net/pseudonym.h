/**
 * @file        net/pseudonym.h
 * @brief       Stable inoffensive display names for non-friend peers —
 *              design spec §17.3.7.
 *
 * Peers you are not friends with are never shown their self-asserted display
 * name. They are shown a generated one ("Curious Cat", "Lucky Dog") derived
 * deterministically from the peer id, so:
 *
 *   - every observer names a given peer identically, which is what makes
 *     verbal coordination work ("follow Curious Cat"), and
 *   - the name is stable across sessions.
 *
 * The derivation is deliberately simple and fully specified (§17.3.7) so an
 * independent implementation — the Rust CLI, a non-recomp adopter — produces
 * byte-identical names. It is NOT a security primitive: the peer id is
 * already a cryptographic digest, so this only has to distribute evenly.
 *
 * Deliberately free of any rexnet-core dependency: pure function of its
 * input, unit-testable without the Rust staticlib or a live mesh.
 */
#pragma once

#include <cstdint>
#include <string>

#include <rex/net/rexnet_ffi.h>

namespace rex::net {

/// Wordlist sizes. Both lists are curated so that *no* pairing is offensive;
/// safety is a property of the lists, not of a downstream filter.
inline constexpr uint32_t kPseudonymAdjectives = 64;
inline constexpr uint32_t kPseudonymNouns = 64;
/// 4096 distinct names. In a full 255-member shard (§17.3) expect ~8 pairs to
/// collide by birthday; callers resolve that with PseudonymWithDiscriminator.
inline constexpr uint32_t kPseudonymCombinations =
    kPseudonymAdjectives * kPseudonymNouns;

/// Domain-separation prefix. Mixed in so a pseudonym never correlates with
/// the other values derived from these same bytes (OnlineKey, EnetAddr).
inline constexpr const char* kPseudonymDomain = "rexnet-pseudonym-v1";

/// FNV-1a 64 over (kPseudonymDomain || peer bytes). Exposed for tests and
/// for reimplementation parity checks.
uint64_t PseudonymHash(const RexNetPeerId& peer);

/// "Curious Cat". Stable for a given peer id, identical on every observer.
std::string Pseudonym(const RexNetPeerId& peer);

/// As Pseudonym(), plus a numeric discriminator ("Curious Cat 2") when two
/// peers in view collide. `index` is the collision ordinal: 0 yields the bare
/// name, so the first holder is never decorated.
std::string PseudonymWithDiscriminator(const RexNetPeerId& peer, uint32_t index);

}  // namespace rex::net
