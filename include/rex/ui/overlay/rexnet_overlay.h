/**
 * @file        rex/ui/overlay/rexnet_overlay.h
 * @brief       Basic ImGui overlay for the RexNet netplay module (F6).
 *
 * @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
 *              All rights reserved.
 *
 * @license     BSD 3-Clause License
 *              See LICENSE file in the project root for full license text.
 *
 * The UI layer sits below the kernel shim that owns the RexNet instance, so
 * this overlay reads a snapshot through a registered provider and issues
 * actions through registered callbacks (the same observer pattern used for
 * the log sink / achievement callbacks). The rexnet module installs both at
 * startup; when nothing is registered the overlay shows an inactive state.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <rex/ui/imgui_dialog.h>

namespace rex::ui {

struct RexNetOverlayStatus {
  bool active = false;
  std::string peer_id;      // base58 identity
  std::string friend_code;  // REXN-… rendering of the same identity
  uint32_t friend_count = 0;
  bool session_active = false;
  std::string session_kind;  // "public" / "private" / ""
  struct Peer {
    std::string virtual_ip;  // "10.77.x.y"
    std::string status;      // "connected" / "pending" / "lost" / "idle"
    // Safe to render as-is (§17.2): a friend's own name, else a pseudonym.
    std::string name;
    bool is_friend = false;
  };
  std::vector<Peer> peers;

  /// Ambient shard roster (§17.3). Empty when the title has not opted in.
  struct ShardMember {
    std::string name;  // §17.2-resolved: friend name or pseudonym
    bool is_friend = false;
    uint8_t state = 0;  // 0 offline, 1 online, 2 in-game, 3 joinable
    bool has_session = false;
  };
  std::vector<ShardMember> shard_members;

  std::string display_name;  // local self-asserted name (may be empty)

  // All action callbacks are keyed by the base58 peer id.
  struct Friend {
    std::string peer_id;
    std::string name;  // presence display name; empty until seen
    bool online = false;
    uint32_t title_id = 0;   // last presence, 0 = unknown
    std::string title_name;  // advertised game name, may be empty
  };
  std::vector<Friend> friends;
  struct FriendRequest {
    std::string peer_id;
    std::string name;  // requester's self-asserted name
    std::string note;
  };
  std::vector<FriendRequest> friend_requests;
  struct Invite {
    std::string peer_id;
    std::string name;  // inviter presence name if known
    uint32_t title_id = 0;
  };
  std::vector<Invite> invites;
};

using RexNetStatusProvider = std::function<RexNetOverlayStatus()>;

struct RexNetOverlayActions {
  std::function<void(const std::string& multiaddr)> connect_manual;
  // Set the local display name (applied live, persisted, re-renders the
  // friend code).
  std::function<void(const std::string& name)> set_display_name;
  // Returns false if the peer id fails to parse (overlay shows an error).
  std::function<bool(const std::string& peer_id)> send_friend_request;
  std::function<void(const std::string& peer_id)> accept_friend_request;
  std::function<void(const std::string& peer_id)> dismiss_friend_request;
  std::function<void(const std::string& peer_id)> remove_friend;
  // Invite a friend to the current session (no-op when none is active).
  std::function<void(const std::string& peer_id)> invite_friend;
  std::function<void(const std::string& peer_id)> accept_invite;
  std::function<void(const std::string& peer_id)> decline_invite;
};

// Installed by the rexnet module (no-op to clear). Thread-safe to call once
// at startup; the overlay reads on the UI thread.
void SetRexNetStatusProvider(RexNetStatusProvider provider);
void SetRexNetOverlayActions(RexNetOverlayActions actions);

// Snapshot via the registered provider (inactive status if none). Usable by
// the overlay, console commands, or tests.
RexNetOverlayStatus QueryRexNetStatus();

class RexNetOverlayDialog : public ImGuiDialog {
 public:
  explicit RexNetOverlayDialog(ImGuiDrawer* imgui_drawer);
  ~RexNetOverlayDialog() override;

 protected:
  void OnDraw(ImGuiIO& io) override;

 private:
  char connect_buf_[256] = {};
  char add_friend_buf_[128] = {};  // fits a named REXN code or base58 id
  bool add_friend_error_ = false;
  char name_buf_[25] = {};  // 24 chars + NUL
  bool name_seeded_ = false;
};

}  // namespace rex::ui
