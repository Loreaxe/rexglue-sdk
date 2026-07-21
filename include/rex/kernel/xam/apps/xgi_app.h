#pragma once
/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2015 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 *
 * @modified    Tom Clay, 2026 - Adapted for ReXGlue runtime
 * @modified    Ryan Fisher, 2026 - RexNet netplay integration
 */

#include <rex/system/kernel_state.h>
#include <rex/system/xam/app_manager.h>

namespace rex {
namespace kernel {
namespace xam {
namespace apps {

class XgiApp : public system::xam::App {
 public:
  explicit XgiApp(system::KernelState* kernel_state);

  X_HRESULT DispatchMessageSync(uint32_t message, uint32_t buffer_ptr,
                                uint32_t buffer_length) override;

  // XSessionSearch/-Ex complete asynchronously (DHT lookup + per-host
  // descriptor queries), deferring the overlapped to the dispatch thread.
  X_HRESULT DispatchMessageAsync(uint32_t message, uint32_t buffer_ptr, uint32_t buffer_length,
                                 uint32_t overlapped_ptr, bool* out_deferred) override;
};

}  // namespace apps
}  // namespace xam
}  // namespace kernel
}  // namespace rex
