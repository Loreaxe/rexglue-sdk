/**
 * @file        rexnet/pseudonym.cpp
 * @brief       Deterministic inoffensive peer names — design spec §17.3.7.
 */

#include <rex/net/pseudonym.h>

#include <cstring>

namespace rex::net {
namespace {

// Curated so that every adjective/noun pairing reads as harmless. Keep both
// lists free of anything that could combine badly: no body parts, no slurs or
// near-homophones, no words with a crude second meaning. Adding a word is
// adding it to 64 pairings at once -- check them all.
//
// Order is part of the wire contract: these are indexed by hash, so
// REORDERING OR REMOVING AN ENTRY RENAMES EVERY AFFECTED PEER. Append-only,
// and only alongside a §17.3.7 version bump (kPseudonymDomain).
constexpr const char* kAdjectives[kPseudonymAdjectives] = {
    "Curious",  "Lucky",    "Brave",    "Clever",   "Gentle",   "Swift",
    "Quiet",    "Cheerful", "Bright",   "Calm",     "Daring",   "Eager",
    "Fearless", "Friendly", "Graceful", "Happy",    "Honest",   "Humble",
    "Jolly",    "Keen",     "Kindly",   "Lively",   "Loyal",    "Merry",
    "Mighty",   "Nimble",   "Noble",    "Patient",  "Peaceful", "Playful",
    "Polite",   "Proud",    "Quick",    "Radiant",  "Ready",    "Restless",
    "Rugged",   "Sincere",  "Sleepy",   "Smiling",  "Sunny",    "Spirited",
    "Steady",   "Sturdy",   "Thankful", "Thoughtful", "Tidy",   "Tireless",
    "Trusty",   "Valiant",  "Vivid",    "Wandering", "Watchful", "Whimsical",
    "Wise",     "Witty",    "Zealous",  "Amber",    "Autumn",   "Golden",
    "Silver",   "Crimson",  "Emerald",  "Sapphire",
};

constexpr const char* kNouns[kPseudonymNouns] = {
    "Cat",      "Dog",      "Fox",      "Owl",      "Bear",     "Wolf",
    "Otter",    "Badger",   "Hare",     "Deer",     "Elk",      "Lynx",
    "Heron",    "Falcon",   "Raven",    "Robin",    "Sparrow",  "Swallow",
    "Finch",    "Magpie",   "Puffin",   "Pelican",  "Swan",     "Crane",
    "Turtle",   "Tortoise", "Gecko",    "Newt",     "Frog",     "Toad",
    "Salmon",   "Trout",    "Perch",    "Marlin",   "Dolphin",  "Whale",
    "Seal",     "Walrus",   "Penguin",  "Puma",     "Tiger",    "Panther",
    "Leopard",  "Jaguar",   "Bison",    "Moose",    "Ibex",     "Antelope",
    "Gazelle",  "Zebra",    "Camel",    "Llama",    "Alpaca",   "Donkey",
    "Pony",     "Stallion", "Beetle",   "Cricket",  "Firefly",  "Dragonfly",
    "Hedgehog", "Squirrel", "Marmot",   "Meerkat",
};

}  // namespace

uint64_t PseudonymHash(const RexNetPeerId& peer) {
    // FNV-1a 64. Chosen over a cryptographic hash deliberately: the peer id is
    // already a digest, so this only needs even distribution, and a trivially
    // reimplementable function keeps cross-implementation parity cheap.
    constexpr uint64_t kOffsetBasis = 1469598103934665603ull;
    constexpr uint64_t kPrime = 1099511628211ull;

    uint64_t hash = kOffsetBasis;
    for (const char* p = kPseudonymDomain; *p; ++p) {
        hash ^= static_cast<uint8_t>(*p);
        hash *= kPrime;
    }
    // Hash the full length-prefixed identity. The length is included so two
    // peer ids where one is a prefix of the other cannot collide.
    const uint32_t len = peer.len <= sizeof(peer.bytes) ? peer.len : sizeof(peer.bytes);
    hash ^= static_cast<uint8_t>(len);
    hash *= kPrime;
    for (uint32_t i = 0; i < len; ++i) {
        hash ^= peer.bytes[i];
        hash *= kPrime;
    }

    // Avalanche (Murmur3 fmix64). Required, not decorative: FNV-1a's carries
    // only propagate toward high bits, so its low bits barely mix. Indexing a
    // 64-entry list with the raw low bits yielded 2 distinct nouns across 255
    // peers. Finalizing makes every bit usable, so the two halves below can be
    // taken as independent.
    hash ^= hash >> 33;
    hash *= 0xff51afd7ed558ccdull;
    hash ^= hash >> 33;
    hash *= 0xc4ceb9fe1a85ec53ull;
    hash ^= hash >> 33;
    return hash;
}

std::string Pseudonym(const RexNetPeerId& peer) {
    const uint64_t hash = PseudonymHash(peer);
    // Independent halves, so adjective and noun do not co-vary.
    const char* adjective = kAdjectives[(hash >> 32) % kPseudonymAdjectives];
    const char* noun = kNouns[(hash & 0xFFFFFFFFull) % kPseudonymNouns];

    std::string out;
    out.reserve(std::strlen(adjective) + 1 + std::strlen(noun));
    out += adjective;
    out += ' ';
    out += noun;
    return out;
}

std::string PseudonymWithDiscriminator(const RexNetPeerId& peer, uint32_t index) {
    std::string out = Pseudonym(peer);
    if (index) {
        // Humans count from 1, and the first holder stays undecorated, so the
        // second peer to collide reads as "... 2".
        out += ' ';
        out += std::to_string(index + 1);
    }
    return out;
}

}  // namespace rex::net
