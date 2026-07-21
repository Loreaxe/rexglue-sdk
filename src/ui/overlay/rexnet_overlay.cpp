/**
 * @file        ui/overlay/rexnet_overlay.cpp
 * @brief       RexNet overlay implementation. See rexnet_overlay.h.
 *
 * @copyright   Copyright (c) 2026 Ryan Fisher <ryanfisher099@gmail.com>
 *              All rights reserved.
 *
 * @license     BSD 3-Clause License
 *              See LICENSE file in the project root for full license text.
 */
#include <rex/ui/overlay/rexnet_overlay.h>

#include <cstring>
#include <mutex>

#include <imgui.h>

namespace rex::ui {

namespace {
std::mutex g_mutex;
RexNetStatusProvider g_provider;
RexNetOverlayActions g_actions;
}  // namespace

void SetRexNetStatusProvider(RexNetStatusProvider provider) {
  std::lock_guard<std::mutex> lock(g_mutex);
  g_provider = std::move(provider);
}

void SetRexNetOverlayActions(RexNetOverlayActions actions) {
  std::lock_guard<std::mutex> lock(g_mutex);
  g_actions = std::move(actions);
}

RexNetOverlayStatus QueryRexNetStatus() {
  RexNetStatusProvider provider;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    provider = g_provider;
  }
  return provider ? provider() : RexNetOverlayStatus{};
}

RexNetOverlayDialog::RexNetOverlayDialog(ImGuiDrawer* imgui_drawer) : ImGuiDialog(imgui_drawer) {}
RexNetOverlayDialog::~RexNetOverlayDialog() = default;

void RexNetOverlayDialog::OnDraw(ImGuiIO& /*io*/) {
  RexNetStatusProvider provider;
  RexNetOverlayActions actions;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    provider = g_provider;
    actions = g_actions;
  }

  ImGui::SetNextWindowSize(ImVec2(460, 560), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowBgAlpha(0.85f);
  if (!ImGui::Begin("RexNet##rexnet", nullptr, ImGuiWindowFlags_NoCollapse)) {
    ImGui::End();
    return;
  }

  RexNetOverlayStatus status = provider ? provider() : RexNetOverlayStatus{};

  if (!status.active) {
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "RexNet is not active.");
    ImGui::TextWrapped(
        "The module starts on the first XNetStartup from guest code (or when "
        "a title initializes networking). Identity and peers appear here once "
        "it is up.");
    ImGui::End();
    return;
  }

  ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Active");
  ImGui::Separator();

  // Local display name: seed the box once from the module state, then let
  // the user edit; Save applies live and re-renders the friend code.
  if (!name_seeded_) {
    std::strncpy(name_buf_, status.display_name.c_str(), sizeof(name_buf_) - 1);
    name_seeded_ = true;
  }
  ImGui::TextDisabled("Display name (shown to friends, max 24)");
  ImGui::SetNextItemWidth(-70.0f);
  const bool name_submit = ImGui::InputText("##displayname", name_buf_, sizeof(name_buf_),
                                            ImGuiInputTextFlags_EnterReturnsTrue);
  ImGui::SameLine();
  const bool can_save_name = actions.set_display_name != nullptr;
  if (!can_save_name) {
    ImGui::BeginDisabled();
  }
  if ((ImGui::Button("Save##name") || name_submit) && can_save_name) {
    actions.set_display_name(name_buf_);
  }
  if (!can_save_name) {
    ImGui::EndDisabled();
  }

  if (!status.friend_code.empty()) {
    ImGui::Text("Friend code:");
    ImGui::TextWrapped("%s", status.friend_code.c_str());
    if (ImGui::SmallButton("Copy##code")) {
      ImGui::SetClipboardText(status.friend_code.c_str());
    }
    ImGui::SameLine();
  } else {
    ImGui::Text("Peer ID:");
    ImGui::SameLine();
    ImGui::TextWrapped("%s", status.peer_id.c_str());
  }
  if (ImGui::SmallButton("Copy peer id##peer")) {
    ImGui::SetClipboardText(status.peer_id.c_str());
  }

  if (status.session_active) {
    ImGui::Text("Session: active (%s)", status.session_kind.c_str());
  } else {
    ImGui::TextDisabled("Session: none");
  }

  // Inbound requests/invites first: they are actionable and time-sensitive.
  if (!status.friend_requests.empty()) {
    ImGui::Separator();
    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f),
                       "Friend requests (%zu):", status.friend_requests.size());
    for (const auto& request : status.friend_requests) {
      ImGui::PushID(request.peer_id.c_str());
      if (!request.name.empty()) {
        ImGui::Text("%s", request.name.c_str());
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip("%s", request.peer_id.c_str());
        }
      } else {
        ImGui::TextWrapped("%s", request.peer_id.c_str());
      }
      if (!request.note.empty()) {
        ImGui::TextDisabled("\"%s\"", request.note.c_str());
      }
      if (ImGui::SmallButton("Accept") && actions.accept_friend_request) {
        actions.accept_friend_request(request.peer_id);
      }
      ImGui::SameLine();
      if (ImGui::SmallButton("Dismiss") && actions.dismiss_friend_request) {
        actions.dismiss_friend_request(request.peer_id);
      }
      ImGui::PopID();
    }
  }

  if (!status.invites.empty()) {
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Invites (%zu):", status.invites.size());
    for (const auto& invite : status.invites) {
      ImGui::PushID(invite.peer_id.c_str());
      if (!invite.name.empty()) {
        ImGui::Text("%s", invite.name.c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("(title %08X)", invite.title_id);
      } else {
        ImGui::TextWrapped("%s (title %08X)", invite.peer_id.c_str(), invite.title_id);
      }
      if (ImGui::SmallButton("Accept") && actions.accept_invite) {
        actions.accept_invite(invite.peer_id);
      }
      ImGui::SameLine();
      if (ImGui::SmallButton("Decline") && actions.decline_invite) {
        actions.decline_invite(invite.peer_id);
      }
      ImGui::PopID();
    }
  }

  ImGui::Separator();
  ImGui::Text("Friends (%u):", status.friend_count);
  if (!status.friends.empty() &&
      ImGui::BeginTable("##friends", 3,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                        ImVec2(0, 110))) {
    ImGui::TableSetupColumn("Name", ImGuiTableFlags_None);
    ImGui::TableSetupColumn("Status");
    ImGui::TableSetupColumn("##actions");
    ImGui::TableHeadersRow();
    for (const auto& friend_info : status.friends) {
      ImGui::TableNextRow();
      ImGui::PushID(friend_info.peer_id.c_str());
      ImGui::TableNextColumn();
      const char* name =
          !friend_info.name.empty() ? friend_info.name.c_str() : friend_info.peer_id.c_str();
      ImGui::TextUnformatted(name);
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", friend_info.peer_id.c_str());
      }
      ImGui::TableNextColumn();
      if (!friend_info.title_name.empty()) {
        ImGui::TextUnformatted(friend_info.title_name.c_str());
      } else if (friend_info.title_id) {
        ImGui::Text("in game %08X", friend_info.title_id);
      } else {
        ImGui::TextUnformatted(friend_info.online ? "online" : "offline");
      }
      ImGui::TableNextColumn();
      const bool can_invite = status.session_active && friend_info.online && actions.invite_friend;
      if (!can_invite) {
        ImGui::BeginDisabled();
      }
      if (ImGui::SmallButton("Invite") && can_invite) {
        actions.invite_friend(friend_info.peer_id);
      }
      if (!can_invite) {
        ImGui::EndDisabled();
      }
      ImGui::SameLine();
      if (ImGui::SmallButton("Remove") && actions.remove_friend) {
        actions.remove_friend(friend_info.peer_id);
      }
      ImGui::PopID();
    }
    ImGui::EndTable();
  }

  ImGui::TextDisabled("Add friend (friend code or peer id)");
  ImGui::SetNextItemWidth(-70.0f);
  const bool add_submit = ImGui::InputText("##addfriend", add_friend_buf_, sizeof(add_friend_buf_),
                                           ImGuiInputTextFlags_EnterReturnsTrue);
  ImGui::SameLine();
  const bool can_add = actions.send_friend_request && add_friend_buf_[0] != '\0';
  if (!can_add) {
    ImGui::BeginDisabled();
  }
  if ((ImGui::Button("Add") || add_submit) && can_add) {
    add_friend_error_ = !actions.send_friend_request(add_friend_buf_);
    if (!add_friend_error_) {
      add_friend_buf_[0] = '\0';
    }
  }
  if (!can_add) {
    ImGui::EndDisabled();
  }
  if (add_friend_error_) {
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Not a valid friend code or peer id.");
  }

  ImGui::Separator();
  ImGui::Text("Shard (%zu):", status.shard_members.size());
  if (status.shard_members.empty()) {
    ImGui::TextDisabled("not in a shard (opt in via rexnet.toml [shard])");
  } else if (ImGui::BeginTable(
                 "##shard", 3,
                 ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                 ImVec2(0, 120))) {
    ImGui::TableSetupColumn("Name");
    ImGui::TableSetupColumn("State");
    ImGui::TableSetupColumn("Joinable");
    ImGui::TableHeadersRow();
    for (const auto& member : status.shard_members) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      if (member.is_friend) {
        ImGui::TextUnformatted(member.name.c_str());
      } else {
        // Dimmed: a generated name, not one they chose.
        ImGui::TextDisabled("%s", member.name.c_str());
      }
      ImGui::TableNextColumn();
      static const char* kStates[] = {"offline", "online", "in game", "joinable"};
      ImGui::TextUnformatted(member.state < 4 ? kStates[member.state] : "?");
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(member.has_session ? "yes" : "");
    }
    ImGui::EndTable();
  }

  ImGui::Separator();
  ImGui::Text("Peers (%zu):", status.peers.size());
  if (ImGui::BeginTable("##peers", 3,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                        ImVec2(0, 140))) {
    ImGui::TableSetupColumn("Virtual IP");
    ImGui::TableSetupColumn("Status");
    ImGui::TableSetupColumn("Name");
    ImGui::TableHeadersRow();
    for (const auto& peer : status.peers) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(peer.virtual_ip.c_str());
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(peer.status.c_str());
      ImGui::TableNextColumn();
      if (peer.is_friend) {
        ImGui::TextUnformatted(peer.name.c_str());
      } else {
        // Dimmed so a generated name is never mistaken for a real one.
        ImGui::TextDisabled("%s", peer.name.c_str());
      }
    }
    ImGui::EndTable();
  }

  ImGui::Separator();
  ImGui::TextDisabled("Manual connect (multiaddr)");
  ImGui::SetNextItemWidth(-70.0f);
  const bool submit = ImGui::InputText("##multiaddr", connect_buf_, sizeof(connect_buf_),
                                       ImGuiInputTextFlags_EnterReturnsTrue);
  ImGui::SameLine();
  const bool can_connect = actions.connect_manual && connect_buf_[0] != '\0';
  if (!can_connect) {
    ImGui::BeginDisabled();
  }
  if ((ImGui::Button("Connect") || submit) && can_connect) {
    actions.connect_manual(connect_buf_);
    connect_buf_[0] = '\0';
  }
  if (!can_connect) {
    ImGui::EndDisabled();
  }

  ImGui::End();
}

}  // namespace rex::ui
