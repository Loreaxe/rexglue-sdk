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

#include <rex/kernel/xam/apps/xgi_app.h>
#include <rex/logging.h>
#include <rex/thread.h>

#if REXGLUE_ENABLE_REXNET
#include <algorithm>
#include <array>
#include <cstring>
#include <random>

#include <rex/net/rexnet.h>
#endif

namespace rex {
namespace kernel {
namespace xam {
using namespace rex::system;
using namespace rex::system::xam;
namespace apps {
using namespace rex::system;

XgiApp::XgiApp(KernelState* kernel_state) : App(kernel_state, 0xFB) {}

// XSessionCreate flags (XDK).
[[maybe_unused]] constexpr uint32_t kSessionCreateHost = 0x00000001;
[[maybe_unused]] constexpr uint32_t kSessionCreateUsesMatchmaking = 0x00000008;

#if REXGLUE_ENABLE_REXNET
// Shared XSESSION_INFO synthesis lives on RexNet (also used by the
// XLiveBase invite-info path).
static void FillSessionInfo(uint8_t* info, const std::array<uint8_t, 16>& session_id,
                            uint32_t host_vip, const RexNetPeerId& host_peer) {
  net::RexNet::FillGuestSessionInfo(info, session_id, host_vip, host_peer);
}
#endif

// http://mb.mirage.org/bugzilla/xliveless/main.c

#if REXGLUE_ENABLE_REXNET
// Write an XSESSION_SEARCHRESULT_HEADER + results array into the guest
// buffer. Layout (XDK): header { dwSearchResults u32, pResults ptr } then
// XSESSION_SEARCHRESULT[] { XSESSION_INFO(60), open/filled slot counts
// (4 u32), cProperties, cContexts, pProperties, pContexts }. Returns bytes
// used.
static uint32_t MarshalSearchResults(memory::Memory* memory, uint32_t results_ptr,
                                     uint32_t buffer_size, uint32_t max_wanted,
                                     const std::vector<net::RexNet::SessionResult>& results) {
  constexpr uint32_t kHeaderSize = 8;
  constexpr uint32_t kResultSize = 92;
  if (!results_ptr || buffer_size < kHeaderSize) {
    return 0;
  }
  auto* rexnet = net::RexNet::shared();
  const uint32_t capacity = (buffer_size - kHeaderSize) / kResultSize;
  const uint32_t count =
      std::min<uint32_t>({static_cast<uint32_t>(results.size()), max_wanted, capacity});

  uint8_t* base = memory->TranslateVirtual(results_ptr);
  memory::store_and_swap<uint32_t>(base + 0, count);
  memory::store_and_swap<uint32_t>(base + 4, count ? results_ptr + kHeaderSize : 0);

  for (uint32_t i = 0; i < count; i++) {
    const auto& result = results[i];
    uint8_t* entry = base + kHeaderSize + i * kResultSize;

    uint8_t key[20];
    net::RexNet::OnlineKey(result.host, key);
    const uint32_t host_vip = rexnet->VipFromOnlineKey(key).value_or(0);
    FillSessionInfo(entry, result.session_id, host_vip, result.host);

    const uint32_t filled = result.slots_total - result.slots_open;
    memory::store_and_swap<uint32_t>(entry + 60, result.slots_open);  // open public
    memory::store_and_swap<uint32_t>(entry + 64, 0);                  // open private
    memory::store_and_swap<uint32_t>(entry + 68, filled);             // filled public
    memory::store_and_swap<uint32_t>(entry + 72, 0);                  // filled private
    memory::store_and_swap<uint32_t>(entry + 76, 0);                  // cProperties
    memory::store_and_swap<uint32_t>(entry + 80, 0);                  // cContexts
    memory::store_and_swap<uint32_t>(entry + 84, 0);                  // pProperties
    memory::store_and_swap<uint32_t>(entry + 88, 0);                  // pContexts
  }
  return kHeaderSize + count * kResultSize;
}
#endif  // REXGLUE_ENABLE_REXNET

X_HRESULT XgiApp::DispatchMessageAsync(uint32_t message, uint32_t buffer_ptr,
                                       uint32_t buffer_length, uint32_t overlapped_ptr,
                                       bool* out_deferred) {
#if REXGLUE_ENABLE_REXNET
  // 0x000B0016 = XSessionSearch, 0x000B001C = XSessionSearchEx (same layout
  // + trailing num_users).
  if ((message == 0x000B0016 || message == 0x000B001C) && overlapped_ptr && net::RexNet::shared()) {
    auto buffer = memory_->TranslateVirtual(buffer_ptr);
    uint32_t num_results = memory::load_and_swap<uint32_t>(buffer + 8);
    uint32_t results_buffer_size = memory::load_and_swap<uint32_t>(buffer + 24);
    uint32_t search_results_ptr = memory::load_and_swap<uint32_t>(buffer + 28);
    REXKRNL_DEBUG("XSessionSearch(async): want {} results into {:08X} ({} bytes)", num_results,
                  search_results_ptr, results_buffer_size);

    auto* rexnet = net::RexNet::shared();
    rexnet->SessionSearch();

    if (out_deferred) {
      *out_deferred = true;
    }
    kernel_state_->CompleteOverlappedDeferredEx(
        [this, num_results, results_buffer_size, search_results_ptr](uint32_t& extended_error,
                                                                     uint32_t& length) -> X_RESULT {
          // Dispatch thread: give the DHT lookup + descriptor queries a
          // bounded window, finishing early once we have enough.
          auto* rexnet = net::RexNet::shared();
          std::vector<net::RexNet::SessionResult> results;
          for (int i = 0; i < 25 && rexnet; i++) {
            rex::thread::Sleep(std::chrono::milliseconds(100));
            auto batch = rexnet->TakeSessionResults();
            results.insert(results.end(), batch.begin(), batch.end());
            if (results.size() >= num_results) {
              break;
            }
          }
          length = MarshalSearchResults(memory_, search_results_ptr, results_buffer_size,
                                        num_results, results);
          REXKRNL_INFO("XSessionSearch: {} result(s), {} bytes", results.size(), length);
          extended_error = X_ERROR_SUCCESS;
          return X_ERROR_SUCCESS;
        },
        overlapped_ptr);
    return X_ERROR_IO_PENDING;
  }
#endif
  if (out_deferred) {
    *out_deferred = false;
  }
  return DispatchMessageSync(message, buffer_ptr, buffer_length);
}

X_HRESULT XgiApp::DispatchMessageSync(uint32_t message, uint32_t buffer_ptr,
                                      uint32_t buffer_length) {
  // NOTE: buffer_length may be zero or valid.
  auto buffer = memory_->TranslateVirtual(buffer_ptr);
  switch (message) {
    case 0x000B0006: {
      if (buffer_length && buffer_length != 24) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 24 (netplay bring-up: tolerating)",
            message, buffer_length);
      }
      // dword r3 user index
      // dword (unwritten?)
      // qword 0
      // dword r4 context enum
      // dword r5 value
      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t context_id = memory::load_and_swap<uint32_t>(buffer + 16);
      uint32_t context_value = memory::load_and_swap<uint32_t>(buffer + 20);
      // INFO while the Fable 2 presence mapping is unknown: these are the
      // context ids the title publishes about itself, and rexnet.toml has to
      // map the right one into a presence rich KV for remote players to learn
      // it. Guessing the id would be worse than reading it off a live run.
      REXKRNL_INFO("XGIUserSetContextEx(user={:08X}, context={:08X}, value={:08X})", user_index,
                   context_id, context_value);
#if REXGLUE_ENABLE_REXNET
      if (auto* rexnet = net::RexNet::shared()) {
        // Per-game config maps context ids into presence rich KVs (§11).
        rexnet->OnGuestContext(context_id, context_value);
      }
#endif
      return X_E_SUCCESS;
    }
    case 0x000B0007: {
      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t property_id = memory::load_and_swap<uint32_t>(buffer + 16);
      uint32_t value_size = memory::load_and_swap<uint32_t>(buffer + 20);
      uint32_t value_ptr = memory::load_and_swap<uint32_t>(buffer + 24);
      REXKRNL_INFO("XGIUserSetPropertyEx(user={:08X}, property={:08X}, size={}, ptr={:08X})",
                   user_index, property_id, value_size, value_ptr);
#if REXGLUE_ENABLE_REXNET
      if (auto* rexnet = net::RexNet::shared(); rexnet && value_ptr && value_size) {
        rexnet->OnGuestProperty(property_id, memory_->TranslateVirtual(value_ptr), value_size);
      }
#endif
      return X_E_SUCCESS;
    }
    case 0x000B0008: {
      // Raw dump so we can confirm the actual buffer layout the game sends.
      uint32_t raw0 = buffer_length >= 4 ? memory::load_and_swap<uint32_t>(buffer + 0) : 0;
      uint32_t raw4 = buffer_length >= 8 ? memory::load_and_swap<uint32_t>(buffer + 4) : 0;
      REXKRNL_INFO("XGIUserWriteAchievements called: buf_len={} raw[0]={:08X} raw[4]={:08X}",
                   buffer_length, raw0, raw4);

      if (buffer_length && buffer_length != 8) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 8 (netplay bring-up: tolerating)",
            message, buffer_length);
      }
      uint32_t achievement_count = raw0;
      uint32_t achievements_ptr = raw4;

      // Empirically confirmed from log: each entry is {u32 padding/user_index, u32 id, ...}.
      // The achievement ID sits at offset 4, not 0. Stride 8 covers the observed fields.
      constexpr uint32_t kEntryIdOffset = 4;
      constexpr uint32_t kEntryStride = 8;
      constexpr uint32_t kMaxAchievements = 1000;

      if (achievements_ptr && achievement_count > 0) {
        if (achievement_count > kMaxAchievements) {
          REXKRNL_WARN("XGIUserWriteAchievements: count={} unreasonable, ignoring",
                       achievement_count);
          return X_E_FAIL;
        }
        uint32_t span_end = achievements_ptr + achievement_count * kEntryStride - 1;
        if (!memory_->LookupHeap(achievements_ptr) || !memory_->LookupHeap(span_end)) {
          REXKRNL_WARN("XGIUserWriteAchievements: ptr {:08X} OOB", achievements_ptr);
          return X_E_FAIL;
        }
        auto* base = memory_->TranslateVirtual(achievements_ptr);
        for (uint32_t i = 0; i < achievement_count; ++i) {
          uint32_t id = memory::load_and_swap<uint32_t>(base + i * kEntryStride + kEntryIdOffset);
          REXKRNL_INFO("XGIUserWriteAchievements: id={} ({})", id, i);
          kernel_state_->UnlockAchievement(id);
        }
      } else {
        REXKRNL_INFO("XGIUserWriteAchievements: skipped (count={} ptr={:08X})", achievement_count,
                     achievements_ptr);
      }
      return X_E_SUCCESS;
    }
    case 0x000B0010: {
      if (buffer_length && buffer_length != 28) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 28 (netplay bring-up: tolerating)",
            message, buffer_length);
      }
      // Sequence:
      // - XamSessionCreateHandle
      // - XamSessionRefObjByHandle
      // - [this]
      // - CloseHandle
      uint32_t session_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t flags = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t num_slots_public = memory::load_and_swap<uint32_t>(buffer + 8);
      uint32_t num_slots_private = memory::load_and_swap<uint32_t>(buffer + 12);
      uint32_t user_xuid = memory::load_and_swap<uint32_t>(buffer + 16);
      uint32_t session_info_ptr = memory::load_and_swap<uint32_t>(buffer + 20);
      uint32_t nonce_ptr = memory::load_and_swap<uint32_t>(buffer + 24);

      REXKRNL_DEBUG(
          "XGISessionCreateImpl({:08X}, {:08X}, {}, {}, {:08X}, {:08X}, "
          "{:08X})",
          session_ptr, flags, num_slots_public, num_slots_private, user_xuid, session_info_ptr,
          nonce_ptr);

#if REXGLUE_ENABLE_REXNET
      if (auto* rexnet = net::RexNet::shared()) {
        const bool is_host = (flags & kSessionCreateHost) != 0;
        const bool is_public = (flags & kSessionCreateUsesMatchmaking) != 0;
        const uint32_t slots_total = num_slots_public + num_slots_private;

        if (is_host && session_info_ptr) {
          auto session_id =
              rexnet->SessionCreate(is_public, static_cast<uint8_t>(std::min(slots_total, 255u)),
                                    static_cast<uint8_t>(std::min(slots_total, 255u)));
          FillSessionInfo(memory_->TranslateVirtual(session_info_ptr), session_id,
                          net::RexNet::kLocalVip, rexnet->local_peer_id());
        }
        // Joiners already carry XSESSION_INFO from search/invite material;
        // nothing to fill.
        if (nonce_ptr) {
          std::random_device rd;
          uint64_t nonce = (uint64_t(rd()) << 32) | rd();
          memory::store_and_swap<uint64_t>(memory_->TranslateVirtual(nonce_ptr), nonce);
        }
      }
#endif
      return X_E_SUCCESS;
    }
    case 0x000B0011: {
      if (buffer_length && buffer_length != 16) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 16 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t obj_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t flags = memory::load_and_swap<uint32_t>(buffer + 4);
      uint64_t session_nonce = memory::load_and_swap<uint64_t>(buffer + 8);

      REXKRNL_DEBUG("XGISessionDelete({:08X}, {:08X}, {:016X})", obj_ptr, flags, session_nonce);

#if REXGLUE_ENABLE_REXNET
      if (auto* rexnet = net::RexNet::shared()) {
        rexnet->SessionDelete();
      }
#endif
      return X_E_SUCCESS;
    }
    case 0x000B0012: {
      if (buffer_length && buffer_length != 20) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 20 (netplay bring-up: tolerating)",
            message, buffer_length);
      }
      uint32_t session_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t user_count = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t unk_0 = memory::load_and_swap<uint32_t>(buffer + 8);
      uint32_t user_index_array = memory::load_and_swap<uint32_t>(buffer + 12);
      uint32_t private_slots_array = memory::load_and_swap<uint32_t>(buffer + 16);

      if (unk_0) {
        REXKRNL_WARN("XGISessionJoinLocal: unk_0={} (nonzero in co-op join; tolerating)", unk_0);
      }
      REXKRNL_DEBUG("XGISessionJoinLocal({:08X}, {}, {}, {:08X}, {:08X})", session_ptr, user_count,
                    unk_0, user_index_array, private_slots_array);
      return X_E_SUCCESS;
    }
    case 0x000B0014: {
      if (buffer_length && buffer_length != 16) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 16 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t obj_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t flags = memory::load_and_swap<uint32_t>(buffer + 4);
      uint64_t session_nonce = memory::load_and_swap<uint64_t>(buffer + 8);

      REXKRNL_DEBUG("XSessionStart({:08X}, {:08X}, {:016X})", obj_ptr, flags, session_nonce);

      return X_STATUS_SUCCESS;
    }
    case 0x000B0015: {
      // send high scores?
      if (buffer_length && buffer_length != 16) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 16 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t obj_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t flags = memory::load_and_swap<uint32_t>(buffer + 4);
      uint64_t session_nonce = memory::load_and_swap<uint64_t>(buffer + 8);

      REXKRNL_DEBUG("XSessionEnd({:08X}, {:08X}, {:016X})", obj_ptr, flags, session_nonce);

      return X_E_SUCCESS;
    }
    case 0x000B0016: {
      if (buffer_length && buffer_length != 32) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 32 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t proc_index = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t num_results = memory::load_and_swap<uint32_t>(buffer + 8);
      uint16_t num_props = memory::load_and_swap<uint16_t>(buffer + 12);
      uint16_t num_ctx = memory::load_and_swap<uint16_t>(buffer + 14);
      uint32_t props_ptr = memory::load_and_swap<uint32_t>(buffer + 16);
      uint32_t ctx_ptr = memory::load_and_swap<uint32_t>(buffer + 20);
      uint32_t results_buffer_size = memory::load_and_swap<uint32_t>(buffer + 24);
      uint32_t search_results_ptr = memory::load_and_swap<uint32_t>(buffer + 28);

      REXKRNL_DEBUG("XSessionSearch({}, {}, {}, {}, {}, {:08X}, {:08X}, {}, {:08X})", proc_index,
                    user_index, num_results, num_props, num_ctx, props_ptr, ctx_ptr,
                    results_buffer_size, search_results_ptr);
      return X_E_SUCCESS;
    }
    case 0x000B0018: {
      if (buffer_length && buffer_length != 16) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 16 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t obj_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t flags = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t maxPublicSlots = memory::load_and_swap<uint32_t>(buffer + 8);
      uint16_t maxPrivateSlots = memory::load_and_swap<uint16_t>(buffer + 12);

      REXKRNL_DEBUG("XSessionModify({:08X}, {:08X}, {:08X}, {:08X})", obj_ptr, flags,
                    maxPublicSlots, maxPrivateSlots);

      return X_E_SUCCESS;
    }
    case 0x000B001C: {
      if (buffer_length && buffer_length != 36) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 36 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      // session_search
      uint32_t proc_index = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t num_results = memory::load_and_swap<uint32_t>(buffer + 8);
      uint16_t num_props = memory::load_and_swap<uint16_t>(buffer + 12);
      uint16_t num_ctx = memory::load_and_swap<uint16_t>(buffer + 14);
      uint32_t props_ptr = memory::load_and_swap<uint32_t>(buffer + 16);
      uint32_t ctx_ptr = memory::load_and_swap<uint32_t>(buffer + 20);
      uint32_t results_buffer_size = memory::load_and_swap<uint32_t>(buffer + 24);
      uint32_t search_results_ptr = memory::load_and_swap<uint32_t>(buffer + 28);
      //
      uint32_t num_users = memory::load_and_swap<uint32_t>(buffer + 32);

      REXKRNL_DEBUG("XSessionSearchEx({}, {}, {}, {}, {}, {:08X}, {:08X}, {}, {:08X}, {})",
                    proc_index, user_index, num_results, num_props, num_ctx, props_ptr, ctx_ptr,
                    results_buffer_size, search_results_ptr, num_users);

      return X_E_SUCCESS;
    }
    case 0x000B001D: {
      if (buffer_length && buffer_length != 24) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 24 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t obj_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t details_buffer_size = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t session_details_ptr = memory::load_and_swap<uint32_t>(buffer + 8);
      uint32_t reserved1 = memory::load_and_swap<uint32_t>(buffer + 12);
      uint32_t reserved2 = memory::load_and_swap<uint32_t>(buffer + 16);
      uint32_t reserved3 = memory::load_and_swap<uint32_t>(buffer + 20);

      REXKRNL_DEBUG("XSessionGetDetails({:08X}, {}, {:08X}, {}, {}, {})", obj_ptr,
                    details_buffer_size, session_details_ptr, reserved1, reserved2, reserved3);

      return X_E_SUCCESS;
    }
    case 0x000B001E: {
      if (buffer_length && buffer_length != 24) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 24 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t obj_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t session_info_ptr = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 8);
      uint32_t reserved1 = memory::load_and_swap<uint32_t>(buffer + 12);
      uint32_t reserved2 = memory::load_and_swap<uint32_t>(buffer + 16);
      uint32_t reserved3 = memory::load_and_swap<uint32_t>(buffer + 20);

      REXKRNL_DEBUG("XSessionMigrateHost({:08X}, {:08X}, {}, {}, {}, {})", obj_ptr,
                    session_info_ptr, user_index, reserved1, reserved2, reserved3);

      return X_E_SUCCESS;
    }
    case 0x000B0019: {
      if (buffer_length && buffer_length != 8) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 8 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t session_info_ptr = memory::load_and_swap<uint32_t>(buffer + 4);

      REXKRNL_DEBUG("XSessionGetInvitationData - unimplemented({}, {:08X})", user_index,
                    session_info_ptr);

      return X_E_SUCCESS;
    }
    case 0x000B001A: {
      if (buffer_length && buffer_length != 28) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 28 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t obj_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t flags = memory::load_and_swap<uint32_t>(buffer + 4);
      uint64_t session_nonce = memory::load_and_swap<uint64_t>(buffer + 8);
      uint32_t session_duration_sec = memory::load_and_swap<uint32_t>(buffer + 16);  // 300
      uint32_t results_buffer_size = memory::load_and_swap<uint32_t>(buffer + 20);
      uint32_t results_ptr = memory::load_and_swap<uint32_t>(buffer + 24);

      REXKRNL_DEBUG("XSessionArbitrationRegister({:08X}, {:08X}, {:016X}, {:08X}, {:08X}, {:08X})",
                    obj_ptr, flags, session_nonce, session_duration_sec, results_buffer_size,
                    results_ptr);

      return X_E_SUCCESS;
    }
    case 0x000B001B: {
      // XSessionSearchByID. The title-side stub (Fable 2: 0x82D01238)
      // answers the size probe itself (needs 0x536 bytes / error 122); XAM
      // only sees the fill call, a 0x14-byte block:
      //   +0x00 user_index
      //   +0x04 u64: XNKID guest pointer in the low dword (std of r3)
      //   +0x0C *pcbResultsBuffer (byte count, already >= 0x536)
      //   +0x10 XSESSION_SEARCHRESULT_HEADER out ptr
      // Fable 2 reaches this from the friends-list/orb join
      // (NLivePresence::FindSessionFromId) with the XNKID it read from our
      // XONLINE_FRIEND record.
      if (buffer_length && buffer_length != 20) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 20 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t session_id_ptr = memory::load_and_swap<uint32_t>(buffer + 8);
      uint32_t results_buffer_size = memory::load_and_swap<uint32_t>(buffer + 12);
      uint32_t search_results_ptr = memory::load_and_swap<uint32_t>(buffer + 16);

      REXKRNL_INFO("XSessionSearchByID(user {}, xnkid @{:08X}, {} bytes into {:08X})", user_index,
                   session_id_ptr, results_buffer_size, search_results_ptr);

#if REXGLUE_ENABLE_REXNET
      if (auto* rexnet = net::RexNet::shared(); rexnet && session_id_ptr && search_results_ptr) {
        const uint8_t* xnkid = memory_->TranslateVirtual(session_id_ptr);
        std::vector<net::RexNet::SessionResult> results;
        if (auto found = rexnet->FindFriendSessionByXnkid(xnkid)) {
          net::RexNet::SessionResult result;
          result.host = found->first;
          result.session_id = found->second;
          result.slots_total = 2;  // Fable 2 co-op: host + one guest
          result.slots_open = 1;
          results.push_back(result);
        } else if (auto acc = rexnet->accepted_invite();
                   acc && !std::memcmp(acc->session_id.data(), xnkid, 8)) {
          // The user was invited to this exact session — presence may now
          // advertise a different session id for the host, but the invite
          // carries the one the title is searching for. Resolve via the
          // recorded invite host + join material.
          net::RexNet::SessionResult result;
          result.host = acc->host;
          result.session_id = acc->session_id;
          result.slots_total = 2;
          result.slots_open = 1;
          results.push_back(result);
        } else if (auto own = rexnet->current_session_id();
                   own && !std::memcmp(own->data(), xnkid, 8)) {
          // Searching for our own session (host-side sanity path).
          net::RexNet::SessionResult result;
          result.host = rexnet->local_peer_id();
          result.session_id = *own;
          result.slots_total = 2;
          result.slots_open = 1;
          results.push_back(result);
        }
        MarshalSearchResults(memory_, search_results_ptr, results_buffer_size, 1, results);
        REXKRNL_INFO("XSessionSearchByID: {} match(es)", results.size());
        return X_E_SUCCESS;
      }
#endif
      // No netplay module: report zero results.
      if (search_results_ptr && results_buffer_size >= 8) {
        uint8_t* header = memory_->TranslateVirtual(search_results_ptr);
        memory::store_and_swap<uint32_t>(header + 0, 0);
        memory::store_and_swap<uint32_t>(header + 4, 0);
      }
      return X_E_SUCCESS;
    }
    case 0x000B001F: {
      if (buffer_length && buffer_length != 24) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 24 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t obj_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t array_count = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t xuid_array_ptr = memory::load_and_swap<uint32_t>(buffer + 8);
      uint32_t reserved1 = memory::load_and_swap<uint32_t>(buffer + 12);
      uint32_t reserved2 = memory::load_and_swap<uint32_t>(buffer + 16);
      uint32_t reserved3 = memory::load_and_swap<uint32_t>(buffer + 20);

      REXKRNL_DEBUG("XSessionModifySkill({:08X}, {}, {:08X}, {}, {}, {})", obj_ptr, array_count,
                    xuid_array_ptr, reserved1, reserved2, reserved3);

      return X_E_SUCCESS;
    }
    case 0x000B0020: {
      if (buffer_length && buffer_length != 8) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 8 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t view_id = memory::load_and_swap<uint32_t>(buffer + 4);

      REXKRNL_DEBUG("XUserResetStatsView({:08X}, {})", user_index, view_id);

      return X_E_SUCCESS;
    }
    case 0x000B0021: {
      if (buffer_length && buffer_length != 28) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 28 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t title_id = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t xuids_count = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t xuids_ptr = memory::load_and_swap<uint32_t>(buffer + 8);
      uint32_t specs_count = memory::load_and_swap<uint32_t>(buffer + 12);
      uint32_t specs_ptr = memory::load_and_swap<uint32_t>(buffer + 16);
      uint32_t results_size = memory::load_and_swap<uint32_t>(buffer + 20);
      uint32_t results_ptr = memory::load_and_swap<uint32_t>(buffer + 24);

      REXKRNL_DEBUG("XUserReadStats({}, {}, {:08X}, {}, {:08X}, {}, {:08X})", title_id, xuids_count,
                    xuids_ptr, specs_count, specs_ptr, results_size, results_ptr);

      return X_E_SUCCESS;
    }
    case 0x000B0025: {
      if (buffer_length && buffer_length != 20) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 20 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t obj_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint64_t xuid = memory::load_and_swap<uint64_t>(buffer + 4);
      uint32_t num_views = memory::load_and_swap<uint32_t>(buffer + 12);
      uint32_t views_ptr = memory::load_and_swap<uint32_t>(buffer + 16);

      REXKRNL_DEBUG("XSessionWriteStats({:08X}, {:016X}, {:08X}, {:08X})", obj_ptr, xuid, num_views,
                    views_ptr);

      return X_E_SUCCESS;
    }
    case 0x000B0026: {
      if (buffer_length && buffer_length != 20) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 20 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t obj_ptr = memory::load_and_swap<uint32_t>(buffer + 0);
      uint64_t xuid = memory::load_and_swap<uint64_t>(buffer + 4);
      uint32_t num_views = memory::load_and_swap<uint32_t>(buffer + 12);
      uint32_t views_ptr = memory::load_and_swap<uint32_t>(buffer + 16);

      REXKRNL_DEBUG("XSessionFlushStats({:08X}, {:016X}, {:08X}, {:08X})", obj_ptr, xuid, num_views,
                    views_ptr);

      return X_E_SUCCESS;
    }
    case 0x000B0036: {
      // Called after opening xbox live arcade and clicking on xbox live v5759
      // to 5787 and called after clicking xbox live in the game library from
      // v6683 to v6717
      // Does not get sent a buffer
      REXKRNL_DEBUG("XInvalidateGamerTileCache, unimplemented");
      return X_E_FAIL;
    }
    case 0x000B003D: {
      if (buffer_length && buffer_length != 16) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 16 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t AnId_buffer_size = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t AnId_buffer_ptr = memory::load_and_swap<uint32_t>(buffer + 8);
      uint32_t block = memory::load_and_swap<uint32_t>(buffer + 12);

      REXKRNL_DEBUG("XUserGetANID({:08X}, {:08X}, {:08X}, {:08X})", user_index, AnId_buffer_size,
                    AnId_buffer_ptr, block);

      return X_E_SUCCESS;
    }
    case 0x000B0041: {
      if (buffer_length && buffer_length != 32) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 32 (netplay bring-up: tolerating)",
            message, buffer_length);
      }
      // 00000000 2789fecc 00000000 00000000 200491e0 00000000 200491f0 20049340
      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t context_ptr = memory::load_and_swap<uint32_t>(buffer + 16);
      auto context = context_ptr ? memory_->TranslateVirtual(context_ptr) : nullptr;
      uint32_t context_id = context ? memory::load_and_swap<uint32_t>(context + 0) : 0;
      REXKRNL_DEBUG("XGIUserGetContext({:08X}, {:08X}, {:08X}))", user_index, context_ptr,
                    context_id);
      uint32_t value = 0;
      if (context) {
        memory::store_and_swap<uint32_t>(context + 4, value);
      }
      return X_E_FAIL;
    }
    case 0x000B0060: {
      if (buffer_length && buffer_length != 32) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 32 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t num_session_ids = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t session_ids_ptr = memory::load_and_swap<uint32_t>(buffer + 8);
      uint32_t results_buffer_size = memory::load_and_swap<uint32_t>(buffer + 12);
      uint32_t search_results_ptr = memory::load_and_swap<uint32_t>(buffer + 16);
      uint32_t reserved1 = memory::load_and_swap<uint32_t>(buffer + 20);
      uint32_t reserved2 = memory::load_and_swap<uint32_t>(buffer + 24);
      uint32_t reserved3 = memory::load_and_swap<uint32_t>(buffer + 28);

      REXKRNL_DEBUG("XSessionSearchByIds({:08X}, {:08X}, {:08X}, {:08X}, {:08X}, {}, {}, {})",
                    user_index, num_session_ids, session_ids_ptr, results_buffer_size,
                    search_results_ptr, reserved1, reserved2, reserved3);

      return X_E_SUCCESS;
    }
    case 0x000B0065: {
      if (buffer_length && buffer_length != 52) {
        REXKRNL_WARN(
            "XGI msg {:08X}: buffer_length {} != expected 52 (netplay bring-up: tolerating)",
            message, buffer_length);
      }

      uint32_t proc_index = memory::load_and_swap<uint32_t>(buffer + 0);
      uint32_t user_index = memory::load_and_swap<uint32_t>(buffer + 4);
      uint32_t num_results = memory::load_and_swap<uint32_t>(buffer + 8);
      uint16_t num_weighted_properties = memory::load_and_swap<uint16_t>(buffer + 12);
      uint16_t num_weighted_contexts = memory::load_and_swap<uint16_t>(buffer + 14);
      uint32_t weighted_search_properties_ptr = memory::load_and_swap<uint32_t>(buffer + 16);
      uint32_t weighted_search_contexts_ptr = memory::load_and_swap<uint32_t>(buffer + 20);
      uint16_t num_props = memory::load_and_swap<uint16_t>(buffer + 24);
      uint16_t num_ctx = memory::load_and_swap<uint16_t>(buffer + 26);
      uint32_t non_weighted_search_properties_ptr = memory::load_and_swap<uint32_t>(buffer + 28);
      uint32_t non_weighted_search_contexts_ptr = memory::load_and_swap<uint32_t>(buffer + 32);
      uint32_t results_buffer_size = memory::load_and_swap<uint32_t>(buffer + 36);
      uint32_t search_results_ptr = memory::load_and_swap<uint32_t>(buffer + 40);
      uint32_t num_users = memory::load_and_swap<uint32_t>(buffer + 44);
      uint32_t weighted_search = memory::load_and_swap<uint32_t>(buffer + 48);

      REXKRNL_DEBUG(
          "XSessionSearchWeighted({:08X}, {:08X}, {:08X}, {}, {}, {:08X}, {:08X}, {}, {}, {:08X}, "
          "{:08X}, {:08X}, {:08X}, {:08X}, {:08X})",
          proc_index, user_index, num_results, num_weighted_properties, num_weighted_contexts,
          weighted_search_properties_ptr, weighted_search_contexts_ptr, num_props, num_ctx,
          non_weighted_search_properties_ptr, non_weighted_search_contexts_ptr, results_buffer_size,
          search_results_ptr, num_users, weighted_search);

      return X_E_SUCCESS;
    }
    case 0x000B0071: {
      REXKRNL_DEBUG("XGI 0x000B0071, unimplemented");
      return X_E_SUCCESS;
    }
  }
  REXKRNL_ERROR(
      "Unimplemented XGI message app={:08X}, msg={:08X}, arg1={:08X}, "
      "arg2={:08X}",
      app_id(), message, buffer_ptr, buffer_length);
  return X_E_FAIL;
}

}  // namespace apps
}  // namespace xam
}  // namespace kernel
}  // namespace rex
