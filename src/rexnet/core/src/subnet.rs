//! Shard subnet addressing — a shard as a virtual /24 (design spec §17.3.7).
//!
//! A shard is capped at a /24 worth of hosts because that is what it *is*: a
//! temporary LAN. For that to mean anything, every member must see the same
//! address for a given peer — otherwise "10.77.5.9" names a different player
//! depending on who you ask, and subnet broadcast (the basis of System Link
//! discovery) has nothing to address.
//!
//! Assignment is therefore derived, not negotiated: every node computes the
//! same table from the same membership, with no coordinator to elect or fail
//! over.
//!
//! Pure functions over a membership snapshot, so the interesting parts —
//! determinism regardless of observation order, collision handling, a full
//! subnet — are testable without a swarm.

use std::collections::BTreeMap;

use libp2p::PeerId;

/// Usable host range of the shard's /24. `.0` is the network address and
/// `.255` the broadcast address, so a shard holds at most 254 members —
/// the `cap` of 255 in §17.3 is one too many and should be read as "a /24".
pub const HOST_MIN: u8 = 1;
pub const HOST_MAX: u8 = 254;
pub const USABLE_HOSTS: usize = (HOST_MAX - HOST_MIN) as usize + 1;

/// FNV-1a 64 with the Murmur3 finalizer, as in the pseudonym derivation:
/// FNV's low bits barely avalanche, and we index small ranges with them.
fn hash(domain: &str, bytes: &[u8]) -> u64 {
    const OFFSET: u64 = 1469598103934665603;
    const PRIME: u64 = 1099511628211;
    let mut h = OFFSET;
    for byte in domain.as_bytes().iter().chain(bytes) {
        h ^= u64::from(*byte);
        h = h.wrapping_mul(PRIME);
    }
    h ^= h >> 33;
    h = h.wrapping_mul(0xff51afd7ed558ccd);
    h ^= h >> 33;
    h = h.wrapping_mul(0xc4ceb9fe1a85ec53);
    h ^= h >> 33;
    h
}

/// `10.77.255.0/24`, reserved for peers that are not in our shard — friends
/// playing something else, an invite from a stranger, a session host we found
/// through the DHT. Kept out of the shard range so the two allocation schemes
/// can never hand out the same address.
pub const RESERVED_SUBNET: u8 = 255;

/// Which /24 inside 10.77.0.0/16 this shard occupies. Never
/// [`RESERVED_SUBNET`], so 255 shard subnets are available.
///
/// Two shards can collide on a subnet. That is harmless: addresses are only
/// ever resolved *within* a shard, and a node belongs to one shard at a time.
pub fn subnet_octet(shard_id: &[u8; 16]) -> u8 {
    (hash("rexnet-subnet-v1", shard_id) % u64::from(RESERVED_SUBNET)) as u8
}

/// The host octet a peer would prefer, before collisions are resolved.
fn preferred_host(peer: &PeerId) -> u8 {
    let raw = hash("rexnet-host-v1", &peer.to_bytes());
    HOST_MIN + (raw % USABLE_HOSTS as u64) as u8
}

/// Assign every member a host octet in `1..=254`.
///
/// Each peer prefers `preferred_host`; collisions are resolved by linear
/// probing, with members processed in peer-id order so that every observer
/// derives an identical table from identical membership. Members beyond a
/// full subnet get no address.
///
/// Caveat: a new arrival can displace a *later* peer that had probed into the
/// slot it wants, so addresses are not perfectly stable as membership grows.
/// Below roughly 50 members collisions are rare enough that this is
/// negligible; a stability guarantee would need a coordinator, which is what
/// deriving the table is meant to avoid.
pub fn assign(members: &[PeerId]) -> BTreeMap<PeerId, u8> {
    // Peer-id order, not observation order -- this is what makes two nodes
    // that learned the membership differently still agree.
    let mut ordered: Vec<&PeerId> = members.iter().collect();
    ordered.sort_by_key(|peer| peer.to_bytes());
    ordered.dedup_by_key(|peer| peer.to_bytes());

    let mut taken = [false; 256];
    let mut out = BTreeMap::new();
    for peer in ordered {
        let start = preferred_host(peer);
        let mut host = None;
        for step in 0..USABLE_HOSTS {
            let candidate =
                HOST_MIN + (((start - HOST_MIN) as usize + step) % USABLE_HOSTS) as u8;
            if !taken[candidate as usize] {
                host = Some(candidate);
                break;
            }
        }
        // Subnet full: the remaining members simply have no address here.
        // Placement (§17.3.2) caps a shard, so this should not be reachable.
        let Some(host) = host else { break };
        taken[host as usize] = true;
        out.insert(*peer, host);
    }
    out
}

/// Full virtual IP for a member: `10.77.<subnet>.<host>`, host byte order.
pub fn address(network_base: u32, shard_id: &[u8; 16], host: u8) -> u32 {
    network_base | (u32::from(subnet_octet(shard_id)) << 8) | u32::from(host)
}

/// Subnet broadcast address, `10.77.<subnet>.255`. Delivering to this means
/// delivering to every shard member — the basis for System Link discovery,
/// where titles broadcast to find each other rather than naming a host.
pub fn broadcast_address(network_base: u32, shard_id: &[u8; 16]) -> u32 {
    network_base | (u32::from(subnet_octet(shard_id)) << 8) | 255
}

#[cfg(test)]
mod tests {
    use super::*;

    fn peers(n: usize) -> Vec<PeerId> {
        (0..n).map(|_| PeerId::random()).collect()
    }

    #[test]
    fn addresses_are_in_the_usable_host_range() {
        let members = peers(50);
        for (_, host) in assign(&members) {
            assert!((HOST_MIN..=HOST_MAX).contains(&host), "host {host} out of range");
        }
    }

    #[test]
    fn addresses_are_unique() {
        let members = peers(200);
        let table = assign(&members);
        let mut seen = std::collections::HashSet::new();
        for (_, host) in &table {
            assert!(seen.insert(*host), "duplicate host octet {host}");
        }
        assert_eq!(table.len(), 200);
    }

    #[test]
    fn observation_order_does_not_change_the_table() {
        // The whole point: two nodes that met the shard in different orders
        // must still name every peer identically.
        let members = peers(80);
        let forward = assign(&members);
        let mut shuffled = members.clone();
        shuffled.reverse();
        let backward = assign(&shuffled);
        assert_eq!(forward, backward);
    }

    #[test]
    fn duplicate_membership_entries_are_ignored() {
        let mut members = peers(10);
        let clone = members[3];
        members.push(clone);
        let table = assign(&members);
        assert_eq!(table.len(), 10);
    }

    #[test]
    fn a_full_subnet_uses_every_slot() {
        let members = peers(USABLE_HOSTS);
        let table = assign(&members);
        assert_eq!(table.len(), USABLE_HOSTS);
        let mut hosts: Vec<u8> = table.values().copied().collect();
        hosts.sort_unstable();
        assert_eq!(hosts.first(), Some(&HOST_MIN));
        assert_eq!(hosts.last(), Some(&HOST_MAX));
    }

    #[test]
    fn overflow_members_get_no_address_rather_than_a_bad_one() {
        // 255 members into 254 slots. Better to be unaddressed than to
        // collide with someone else's address.
        let members = peers(USABLE_HOSTS + 1);
        let table = assign(&members);
        assert_eq!(table.len(), USABLE_HOSTS);
    }

    #[test]
    fn shard_subnets_never_take_the_reserved_range() {
        // A shard landing on 10.77.255.0/24 would collide with the addresses
        // handed to peers outside any shard.
        for seed in 0u8..=255 {
            assert_ne!(subnet_octet(&[seed; 16]), RESERVED_SUBNET);
        }
    }

    #[test]
    fn subnet_and_broadcast_line_up() {
        let base = (10u32 << 24) | (77u32 << 16);
        let shard = [7u8; 16];
        let octet = subnet_octet(&shard);
        let host = address(base, &shard, 9);
        assert_eq!(host, base | (u32::from(octet) << 8) | 9);
        // Broadcast is the same /24, host 255.
        assert_eq!(broadcast_address(base, &shard), (host & !0xFF) | 255);
    }

    #[test]
    fn preferred_hosts_spread_across_the_subnet() {
        // A derivation that clustered would make collisions the norm and
        // linear probing degenerate.
        let members = peers(200);
        let distinct: std::collections::HashSet<u8> =
            members.iter().map(preferred_host).collect();
        assert!(distinct.len() > 120, "only {} distinct preferred hosts", distinct.len());
    }
}
