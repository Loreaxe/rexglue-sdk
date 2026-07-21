/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2021 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 *
 * @modified    Tom Clay, 2026 - Adapted for ReXGlue runtime
 *
 * @modified    Ryan Fisher, 2026 - RexNet netplay integration
 */

#include <rex/kernel/xam/apps/xlivebase_app.h>
#include <rex/kernel/xam/private.h>
#include <rex/logging.h>
#include <rex/thread.h>

#if REXGLUE_ENABLE_REXNET
#include <cstring>

#include <rex/net/rexnet.h>
#endif

namespace rex {
namespace kernel {
namespace xam {
using namespace rex::system;
using namespace rex::system::xam;
namespace apps {
using namespace rex::system;

XLiveBaseApp::XLiveBaseApp(KernelState* kernel_state) : App(kernel_state, 0xFC) {}

// http://mb.mirage.org/bugzilla/xliveless/main.c

X_HRESULT XLiveBaseApp::DispatchMessageSync(uint32_t message, uint32_t buffer_ptr,
                                            uint32_t buffer_length) {
  // NOTE: buffer_length may be zero or valid.
  auto buffer = memory_->TranslateVirtual(buffer_ptr);
  switch (message) {
    case 0x00058004: {
      // Called on startup, seems to just return a bool in the buffer.
      assert_true(!buffer_length || buffer_length == 4);
      REXKRNL_DEBUG("XLiveBaseGetLogonId({:08X})", buffer_ptr);
      memory::store_and_swap<uint32_t>(buffer + 0, 1);  // ?
      return X_E_SUCCESS;
    }
    case 0x00058006: {
      assert_true(!buffer_length || buffer_length == 4);
      REXKRNL_DEBUG("XLiveBaseGetNatType({:08X})", buffer_ptr);
      memory::store_and_swap<uint32_t>(buffer + 0, 1);  // XONLINE_NAT_OPEN
      return X_E_SUCCESS;
    }
    case 0x0005800E: {
      // XUserMuteListQuery: "is this remote talker on my mute list?" Polled
      // during multiplayer voice setup. We keep no mute list, and the guest
      // stub pre-zeroes its result (= not muted), so succeeding reports
      // "nobody muted" — the right generic answer for any title.
      REXKRNL_DEBUG("XUserMuteListQuery({:08X}, {:08X}) -> not muted", buffer_ptr,
                    buffer_length);
      return X_E_SUCCESS;
    }
    case 0x00058007: {
      // Occurs if title calls XOnlineGetServiceInfo, expects dwServiceId
      // and pServiceInfo. pServiceInfo should contain pointer to
      // XONLINE_SERVICE_INFO structure.
      REXKRNL_DEBUG("CXLiveLogon::GetServiceInfo({:08X}, {:08X})", buffer_ptr, buffer_length);
      return 0x80151802;  // ERROR_CONNECTION_INVALID
    }
    case 0x00058020: {
      // CXLiveFriends::Enumerate. Titles reach this through the XDK's
      // statically linked XFriendsCreateEnumerator stub (Fable 2:
      // 0x82CFFAB0), which marshals (dwUserIndex, dwStartingIndex,
      // dwFriendsToReturn, pcbBuffer, ph) into a CArgumentList and calls
      // XMsgInProcessCall(0xFC, 0x58020, marshal_block, arg_list).
      // buffer_ptr is the marshal block (schema id 518; nothing we need),
      // buffer_length is the guest arg list: 16-byte entries
      // { u32 type (4 = pointer), pad, u64 value (low dword = guest ptr,
      // high dword uninitialized) }, entry count at +512.
      const uint32_t list_ptr = buffer_length;
      if (!list_ptr || !memory_->LookupHeap(list_ptr)) {
        REXKRNL_WARN("CXLiveFriends::Enumerate: bad argument list {:08X}", list_ptr);
        return X_E_INVALIDARG;
      }
      const uint8_t* list = memory_->TranslateVirtual(list_ptr);
      const uint32_t arg_count = memory::load_and_swap<uint32_t>(list + 512);
      uint32_t args[5] = {};
      if (arg_count < 5) {
        REXKRNL_WARN("CXLiveFriends::Enumerate: unexpected arg count {}", arg_count);
        return X_E_INVALIDARG;
      }
      for (uint32_t i = 0; i < 5; ++i) {
        const uint32_t type = memory::load_and_swap<uint32_t>(list + 16 * i);
        if (type != 4) {
          REXKRNL_WARN("CXLiveFriends::Enumerate: arg {} type {} != pointer", i, type);
          return X_E_INVALIDARG;
        }
        args[i] = memory::load_and_swap<uint32_t>(list + 16 * i + 12);
        if (!args[i] || !memory_->LookupHeap(args[i])) {
          REXKRNL_WARN("CXLiveFriends::Enumerate: arg {} bad ptr {:08X}", i, args[i]);
          return X_E_INVALIDARG;
        }
      }
      const uint32_t user_index =
          memory::load_and_swap<uint32_t>(memory_->TranslateVirtual(args[0]));
      const uint32_t starting_index =
          memory::load_and_swap<uint32_t>(memory_->TranslateVirtual(args[1]));
      const uint32_t friends_to_return =
          memory::load_and_swap<uint32_t>(memory_->TranslateVirtual(args[2]));
      uint32_t buffer_size = 0;
      uint32_t handle = 0;
      auto result = xeXFriendsCreateEnumerator(user_index, starting_index, friends_to_return,
                                               &buffer_size, &handle);
      memory::store_and_swap<uint32_t>(memory_->TranslateVirtual(args[3]), buffer_size);
      if (XFAILED(result)) {
        return X_HRESULT_FROM_WIN32(result);
      }
      memory::store_and_swap<uint32_t>(memory_->TranslateVirtual(args[4]), handle);
      REXKRNL_INFO("CXLiveFriends::Enumerate(user {}, start {}, max {}): {} bytes",
                   user_index, starting_index, friends_to_return, buffer_size);
      return X_E_SUCCESS;
    }
    case 0x00058023: {
      REXKRNL_DEBUG("CXLiveMessaging::XMessageGameInviteGetAcceptedInfo({:08X}, {:08X})",
                    buffer_ptr, buffer_length);
#if REXGLUE_ENABLE_REXNET
      if (auto* rexnet = net::RexNet::shared()) {
        auto accepted = rexnet->accepted_invite();
        if (!accepted) {
          return X_E_FAIL;
        }
        // Reached via the XDK's XInviteGetAcceptedInfo stub (Fable 2:
        // 0x82CFFC10), which marshals (dwUserIndex, pInfo) into a
        // CArgumentList and calls XMsgInProcessCall(0xFC, 0x58023, block,
        // arg_list) — the same mechanism as the friends enumerate (0x58020).
        // buffer_length is the guest CArgumentList: 16-byte entries
        // { u32 type(4=ptr), pad, u64 value(low dword = guest ptr) }, count
        // at +512. arg[1] is pInfo (the real XINVITE_INFO). The earlier code
        // treated buffer_ptr as the info directly and wrote to the wrong
        // place, leaving the guest's XINVITE_INFO all zero.
        const uint32_t list_ptr = buffer_length;
        uint32_t info_ptr = 0;
        if (list_ptr && memory_->LookupHeap(list_ptr)) {
          const uint8_t* list = memory_->TranslateVirtual(list_ptr);
          const uint32_t arg_count = memory::load_and_swap<uint32_t>(list + 512);
          if (arg_count >= 2 &&
              memory::load_and_swap<uint32_t>(list + 16 * 1) == 4) {
            info_ptr = memory::load_and_swap<uint32_t>(list + 16 * 1 + 12);
          }
        }
        if (!info_ptr || !memory_->LookupHeap(info_ptr)) {
          REXKRNL_WARN("XMessageGameInviteGetAcceptedInfo: bad arg list {:08X} / info {:08X}",
                       list_ptr, info_ptr);
          return X_E_FAIL;
        }
        // XINVITE_INFO (Fable 2 / early XDK layout, verified against the
        // guest's accept handler sub_822CAD38 + join sub_822EA848):
        //   +0x00 XUID xuidInviter
        //   +0x08 XUID xuidInvitee
        //   +0x10 DWORD dwTitleID   (guest checks == its own title id, else
        //                            it refuses the join and nothing happens)
        //   +0x14 XSESSION_INFO hostInfo (60 bytes: XNKID + XNADDR + XNKEY)
        //   +0x50 BOOL fFromGameInvite
        // The earlier layout put hostInfo at +8 with no title id, so the
        // guest's +0x10 title check read session bytes and always failed.
        uint8_t* info = memory_->TranslateVirtual(info_ptr);
        memory::store_and_swap<uint64_t>(info + 0x00, net::RexNet::XuidFromPeer(accepted->host));
        memory::store_and_swap<uint64_t>(info + 0x08,
                                         kernel_state_->user_profile()->xuid());
        memory::store_and_swap<uint32_t>(info + 0x10, kernel_state_->title_id());
        net::RexNet::FillGuestSessionInfo(info + 0x14, accepted->session_id,
                                          accepted->host_vip, accepted->host);
        memory::store_and_swap<uint32_t>(info + 0x50, 1);  // fFromGameInvite
        REXKRNL_INFO("XMessageGameInviteGetAcceptedInfo: filled XINVITE_INFO "
                     "titleID={:08X} host_vip=10.77.{}.{} (guest checks +0x10 "
                     "== its title id)",
                     kernel_state_->title_id(), (accepted->host_vip >> 8) & 0xFF,
                     accepted->host_vip & 0xFF);
        return X_E_SUCCESS;
      }
#endif
      return X_E_FAIL;
    }
    // Serialized XOnline service RPCs (the XDK's CMarshaller layer): the
    // message id is 0x50000 | low word of the per-call marshal cookie, so
    // the whole 0x0005xxxx (x < 0x8000) range is one mechanism, carried
    // via XMsgStartIORequest with a 0x28-byte descriptor block. Fable 2
    // drives its news/gifting storage suite through this (verified:
    // XStorageDownloadToMemory at 0x82CFFFA0 sends (u16)cookie|0x50000) —
    // XStorageEnumerate/Download of Presence / LiveDate / items.dat /
    // gold.dat. Emulating the marshalled schemas is a standalone project;
    // fail cleanly so the title's retry pump keeps the feature dormant.
    // Handled below after the explicit cases (the direct XLiveBase
    // messages all live at 0x00058xxx).
    case 0x00058035: {
      // XStorageBuildServerPath. The XDK in-process stub marshals its
      // arguments as (offsets in the guest block, BE):
      //   +0x00 u32 user_index          +0x08 u64 xuid (0 = current user)
      //   +0x10 u32 storage facility (1 game-clip / 2 per-title /
      //                               3 per-user-title)
      //   +0x14 u32 facility info ptr   +0x18 u32 facility info size
      //   +0x1C u32 item name ptr (UTF-16, null-terminated)
      //   +0x20 u32 server path out ptr (WCHAR)
      //   +0x24 u32 in/out length ptr (WCHARs incl. null)
      // Titles only echo the path back into XStorage* calls, so any
      // consistent shape works; sizes/limits from the observed callers.
      uint64_t xuid = memory::load_and_swap<uint64_t>(buffer + 0x08);
      uint32_t facility = memory::load_and_swap<uint32_t>(buffer + 0x10);
      uint32_t item_name_ptr = memory::load_and_swap<uint32_t>(buffer + 0x1C);
      uint32_t path_ptr = memory::load_and_swap<uint32_t>(buffer + 0x20);
      uint32_t length_ptr = memory::load_and_swap<uint32_t>(buffer + 0x24);
      if (!path_ptr || !length_ptr) {
        return X_E_INVALIDARG;
      }
      std::string item_name;
      if (item_name_ptr) {
        const uint8_t* wide = memory_->TranslateVirtual(item_name_ptr);
        for (uint32_t i = 0; i < 255; ++i) {
          uint16_t ch = memory::load_and_swap<uint16_t>(wide + i * 2);
          if (!ch) {
            break;
          }
          item_name.push_back(ch < 0x80 ? char(ch) : '_');
        }
      }
      if (!xuid) {
        xuid = kernel_state_->user_profile()->xuid();
      }
      const uint32_t title_id = kernel_state_->title_id();
      std::string path;
      switch (facility) {
        case 1:  // XSTORAGE_FACILITY_GAME_CLIP
          path = fmt::format("title/{:08X}/storage/clips/{}", title_id, item_name);
          break;
        case 3:  // XSTORAGE_FACILITY_PER_USER_TITLE
          path = fmt::format("user/{:016X}/title/{:08X}/storage/{}", xuid, title_id, item_name);
          break;
        default:  // XSTORAGE_FACILITY_PER_TITLE and anything unknown
          path = fmt::format("title/{:08X}/storage/{}", title_id, item_name);
          break;
      }
      uint8_t* length_mem = memory_->TranslateVirtual(length_ptr);
      const uint32_t capacity = memory::load_and_swap<uint32_t>(length_mem);
      const uint32_t needed = static_cast<uint32_t>(path.size()) + 1;
      memory::store_and_swap<uint32_t>(length_mem, needed);
      if (capacity < needed) {
        return X_HRESULT_FROM_WIN32(X_ERROR_INSUFFICIENT_BUFFER);
      }
      uint8_t* out = memory_->TranslateVirtual(path_ptr);
      for (uint32_t i = 0; i < path.size(); ++i) {
        memory::store_and_swap<uint16_t>(out + i * 2, uint16_t(path[i]));
      }
      memory::store_and_swap<uint16_t>(out + path.size() * 2, 0);
      REXKRNL_DEBUG("XStorageBuildServerPath(facility={}) -> {}", facility, path);
      return X_E_SUCCESS;
    }
    case 0x00058046: {
      // Required to be successful for 4D530910 to detect signed-in profile
      // Doesn't seem to set anything in the given buffer, probably only takes
      // input
      REXKRNL_DEBUG("XLiveBaseUnk58046({:08X}, {:08X}) unimplemented", buffer_ptr, buffer_length);
      return X_E_SUCCESS;
    }
  }
  if (message >= 0x00050000 && message < 0x00058000) {
    // Serialized XOnline service RPC (see comment above): decline cleanly
    // and log the descriptor for visibility into which RPC was wanted.
    if (buffer_length >= 4) {
      uint32_t block_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      if (block_ptr && memory_->LookupHeap(block_ptr)) {
        const uint8_t* block = memory_->TranslateVirtual(block_ptr);
        REXKRNL_DEBUG("XOnlineServiceRpc(msg {:08X}, {:08X}): hdr {:08X} {:08X} {:08X} {:08X}",
                      message, block_ptr, memory::load_and_swap<uint32_t>(block + 0),
                      memory::load_and_swap<uint32_t>(block + 4),
                      memory::load_and_swap<uint32_t>(block + 8),
                      memory::load_and_swap<uint32_t>(block + 12));
      }
    }
    return X_E_FAIL;
  }
  REXKRNL_ERROR(
      "Unimplemented XLIVEBASE message app={:08X}, msg={:08X}, arg1={:08X}, "
      "arg2={:08X}",
      app_id(), message, buffer_ptr, buffer_length);
  return X_E_FAIL;
}

}  // namespace apps
}  // namespace xam
}  // namespace kernel
}  // namespace rex
