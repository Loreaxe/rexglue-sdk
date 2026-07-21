/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2020 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 *
 * @modified    Tom Clay, 2026 - Adapted for ReXGlue runtime
 *
 * @modified    Ryan Fisher, 2026 - RexNet netplay integration
 */

#include <rex/cvar.h>
#include <rex/logging.h>
#include <rex/system/kernel_state.h>
#include <rex/system/xam/app_manager.h>

REXCVAR_DEFINE_BOOL(xam_trace_msgs, false, "XAM",
                    "Log every XMsg app dispatch (app id, message id, result)");

namespace rex {
namespace system {
namespace xam {

App::App(KernelState* kernel_state, uint32_t app_id)
    : kernel_state_(kernel_state), memory_(kernel_state->memory()), app_id_(app_id) {}

void AppManager::RegisterApp(std::unique_ptr<App> app) {
  assert_zero(app_lookup_.count(app->app_id()));
  app_lookup_.insert({app->app_id(), app.get()});
  apps_.push_back(std::move(app));
}

X_HRESULT AppManager::DispatchMessageSync(uint32_t app_id, uint32_t message, uint32_t buffer_ptr,
                                          uint32_t buffer_length) {
  App* app;
  {
    auto it = app_lookup_.find(app_id);
    if (it == app_lookup_.end()) {
      return X_E_NOTFOUND;
    }
    app = it->second;
  }
  X_HRESULT result = app->DispatchMessageSync(message, buffer_ptr, buffer_length);
  if (REXCVAR_GET(xam_trace_msgs)) {
    REXSYS_INFO("XMsg sync app={:08X} msg={:08X} len={} -> {:08X}", app_id, message, buffer_length,
                static_cast<uint32_t>(result));
  }
  return result;
}

X_HRESULT AppManager::DispatchMessageAsync(uint32_t app_id, uint32_t message, uint32_t buffer_ptr,
                                           uint32_t buffer_length, uint32_t overlapped_ptr,
                                           bool* out_deferred) {
  App* app;
  {
    auto it = app_lookup_.find(app_id);
    if (it == app_lookup_.end()) {
      if (out_deferred) {
        *out_deferred = false;
      }
      return X_E_NOTFOUND;
    }
    app = it->second;
  }
  X_HRESULT result =
      app->DispatchMessageAsync(message, buffer_ptr, buffer_length, overlapped_ptr, out_deferred);
  if (REXCVAR_GET(xam_trace_msgs)) {
    REXSYS_INFO("XMsg async app={:08X} msg={:08X} len={} deferred={} -> {:08X}", app_id, message,
                buffer_length, out_deferred && *out_deferred,
                static_cast<uint32_t>(result));
  }
  return result;
}

}  // namespace xam
}  // namespace system
}  // namespace rex
