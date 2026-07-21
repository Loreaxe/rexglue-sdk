/**
 * @file        xam/xam_friends.cpp
 * @brief       Friends enumerator backed by the RexNet friend mirror.
 *
 * Marshals the module's friend list (design spec §8.2) into guest
 * XONLINE_FRIEND records. Reached two ways: the XamCreateEnumerator-style
 * export below, and XLiveBase message 0x58020 (xlivebase_app.cpp) — the
 * XDK's title-side XFriendsCreateEnumerator stub is statically linked into
 * titles (verified in Fable 2 at 0x82CFFAB0) and marshals its arguments
 * through XMsgInProcessCall(0xFC, 0x58020, ...), never this export.
 *
 * Record layout/stride verified against Fable 2's walker
 * (NLivePresence::ConnectToFriend, 0x822FA988): stride 0xC4, state at
 * 0x18 (checks ONLINE|JOINABLE before connecting), session XNKID at 0x1C,
 * title id at 0x24 — the early-360 XDK layout, not the late one that moves
 * rich presence before the session fields.
 */

// Disable warnings about unused parameters for kernel functions
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <cstring>

#include <rex/kernel/xam/private.h>
#include <rex/logging.h>
#include <rex/hook.h>
#include <rex/types.h>
#include <rex/system/kernel_state.h>
#include <rex/system/xenumerator.h>
#include <rex/system/xtypes.h>

#if REXGLUE_ENABLE_REXNET
#include <rex/net/rexnet.h>
#endif

namespace rex {
namespace kernel {
namespace xam {
using namespace rex::system;
using namespace rex::system::xam;

// Early-360 XDK xonline.h XONLINE_FRIEND, guest (big) endian. The XUID is
// split into dwords so the struct keeps 4-byte alignment: the guest stride
// is exactly 0xC4 and an 8-aligned host struct would pad to 0xC8.
struct X_ONLINE_FRIEND {
  be<uint32_t> xuid_hi;                  // 0x00
  be<uint32_t> xuid_lo;                  // 0x04
  char gamertag[16];                     // 0x08 NUL-terminated ASCII
  be<uint32_t> friend_state;             // 0x18 XONLINE_FRIENDSTATE_*
  uint8_t session_id[8];                 // 0x1C XNKID
  be<uint32_t> title_id;                 // 0x24
  be<uint16_t> rich_presence[64];        // 0x28 UTF-16
  be<uint32_t> rich_presence_len;        // 0xA8 in WCHARs
  be<uint32_t> user_time[2];             // 0xAC FILETIME (lo, hi)
  uint8_t invite_session_id[8];          // 0xB4 XNKID
  be<uint32_t> invite_received_time[2];  // 0xBC FILETIME
};
static_assert_size(X_ONLINE_FRIEND, 0xC4);
// Offsets Fable 2's walker reads (0x822FA988); do not let padding move them.
static_assert(offsetof(X_ONLINE_FRIEND, gamertag) == 0x08);
static_assert(offsetof(X_ONLINE_FRIEND, friend_state) == 0x18);
static_assert(offsetof(X_ONLINE_FRIEND, session_id) == 0x1C);
static_assert(offsetof(X_ONLINE_FRIEND, title_id) == 0x24);

constexpr uint32_t kFriendStateFlagOnline = 0x00000001;
constexpr uint32_t kFriendStateFlagPlaying = 0x00000002;
constexpr uint32_t kFriendStateFlagJoinable = 0x00000010;
constexpr uint32_t kMaxFriends = 100;

X_RESULT xeXFriendsCreateEnumerator(uint32_t user_index, uint32_t starting_index,
                                    uint32_t friends_to_return, uint32_t* buffer_size_out,
                                    uint32_t* handle_out) {
  if (!handle_out || !friends_to_return || friends_to_return > kMaxFriends ||
      starting_index >= kMaxFriends) {
    return X_ERROR_INVALID_PARAMETER;
  }

  if (buffer_size_out) {
    *buffer_size_out = static_cast<uint32_t>(sizeof(X_ONLINE_FRIEND)) * friends_to_return;
  }

  auto e = make_object<XStaticEnumerator<X_ONLINE_FRIEND>>(REX_KERNEL_STATE(),
                                                           friends_to_return);
  auto result = e->Initialize(user_index, 0xFA, 0x58021, 0x58022, 0);
  if (XFAILED(result)) {
    return result;
  }

#if REXGLUE_ENABLE_REXNET
  if (auto* rexnet = net::RexNet::shared()) {
    auto friends = rexnet->SnapshotFriends();
    for (size_t i = starting_index;
         i < friends.size() && e->item_count() < friends_to_return; ++i) {
      const auto& info = friends[i];
      auto* item = e->AppendItem();
      std::memset(item, 0, sizeof(*item));
      const uint64_t xuid = net::RexNet::XuidFromPeer(info.peer);
      item->xuid_hi = static_cast<uint32_t>(xuid >> 32);
      item->xuid_lo = static_cast<uint32_t>(xuid);
      // Presence display name when seen; otherwise a peer-id prefix so the
      // entry is still recognizable against the F6 overlay.
      std::string name = info.display_name.empty()
                             ? net::RexNet::PeerIdString(info.peer)
                             : info.display_name;
      std::strncpy(item->gamertag, name.c_str(), sizeof(item->gamertag) - 1);
      uint32_t state = 0;
      if (info.online) {
        state |= kFriendStateFlagOnline;
      }
      if (info.title_id) {
        state |= kFriendStateFlagPlaying;
        item->title_id = info.title_id;
      }
      if (info.has_session) {
        // Friend is hosting: JOINABLE + the session XNKID feed the title's
        // join-in-progress UI (Fable 2 orbs -> XSessionSearchByID).
        state |= kFriendStateFlagJoinable;
        std::memcpy(item->session_id, info.session_id.data(), 8);
      }
      item->friend_state = state;
      REXKRNL_INFO("  friend[{}]: '{}' online={} playing={} joinable={} title={:08X}",
                   i, name, info.online, info.title_id != 0, info.has_session,
                   info.title_id);
    }
    // Ambient shard members (§17.3). Fable 2 builds its passive orbs from
    // this enumerator, so without appending them here a shard is invisible
    // in game no matter how well placement works. Opt-in per title: these
    // are not friends, and the XDK calls this list "friends".
    // Friends are appended first and shard members only fill what is left,
    // so a populated shard can never push real friends out of a capped list.
    // At two players this is moot; at 254 it decides whether your friends are
    // visible at all.
    size_t shard_added = 0;
    if (rexnet->game_config().shard_surface_as_friends) {
        const uint32_t title_id = rexnet->game_config().advertised_title_id;
        for (const auto& member : rexnet->SnapshotShardMembers()) {
            if (e->item_count() >= friends_to_return) {
                break;
            }
            // A shard member who is also a friend is already listed above;
            // listing them twice would double their orb.
            if (member.is_friend) {
                continue;
            }
            auto* item = e->AppendItem();
            std::memset(item, 0, sizeof(*item));
            const uint64_t xuid = net::RexNet::XuidFromPeer(member.peer);
            item->xuid_hi = static_cast<uint32_t>(xuid >> 32);
            item->xuid_lo = static_cast<uint32_t>(xuid);
            // Already §17.2-resolved: a pseudonym, never their own name.
            std::strncpy(item->gamertag, member.display_name.c_str(),
                         sizeof(item->gamertag) - 1);
            uint32_t state = kFriendStateFlagOnline;
            // Everyone in a shard is in this title by construction.
            state |= kFriendStateFlagPlaying;
            item->title_id = title_id;
            if (member.has_session) {
                state |= kFriendStateFlagJoinable;
                std::memcpy(item->session_id, member.session_id.data(), 8);
            }
            item->friend_state = state;
            ++shard_added;
            REXKRNL_INFO("  shard[{}]: '{}' state={} joinable={}", shard_added,
                         member.display_name, member.state, member.has_session);
        }
    }
    REXKRNL_INFO(
        "XFriendsCreateEnumerator: {} entries ({} friends of {}, {} shard) (start {}, max {})",
        e->item_count(), e->item_count() - shard_added, friends.size(), shard_added,
        starting_index, friends_to_return);
  } else {
    REXKRNL_DEBUG("XFriendsCreateEnumerator: RexNet inactive; empty list");
  }
#else
  REXKRNL_DEBUG("XFriendsCreateEnumerator: built without RexNet; empty list");
#endif  // REXGLUE_ENABLE_REXNET

  *handle_out = e->handle();
  return X_ERROR_SUCCESS;
}

u32 XFriendsCreateEnumerator_entry(u32 user_index, u32 starting_index, u32 friends_to_return,
                                   mapped_u32 buffer_size_ptr, mapped_u32 handle_ptr) {
  uint32_t buffer_size = 0;
  uint32_t handle = 0;
  auto result = xeXFriendsCreateEnumerator(user_index, starting_index, friends_to_return,
                                           &buffer_size, &handle);
  if (buffer_size_ptr) {
    *buffer_size_ptr = buffer_size;
  }
  if (XFAILED(result)) {
    return result;
  }
  *handle_ptr = handle;
  return X_ERROR_SUCCESS;
}

}  // namespace xam
}  // namespace kernel
}  // namespace rex

REX_EXPORT(__imp__XFriendsCreateEnumerator, rex::kernel::xam::XFriendsCreateEnumerator_entry)
