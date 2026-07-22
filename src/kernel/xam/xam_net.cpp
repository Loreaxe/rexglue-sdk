/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 *
 * @modified    Tom Clay, 2026 - Adapted for ReXGlue runtime
 *
 * @modified    Ryan Fisher, 2026 - RexNet netplay integration
 */

// Disable warnings about unused parameters for kernel functions
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <random>
#include <ctime>
#include <fstream>

#include <rex/thread.h>

#include <rex/chrono/clock.h>
#include <rex/kernel/xam/module.h>
#include <rex/kernel/xam/private.h>
#include <rex/kernel/xboxkrnl/error.h>
#include <rex/kernel/xboxkrnl/threading.h>
#include <rex/logging.h>
#include <rex/hook.h>
#include <rex/types.h>
#include <rex/string.h>
#include <rex/system/kernel_state.h>
#include <rex/system/xevent.h>
#include <rex/system/xsocket.h>
#include <rex/system/xthread.h>
#include <rex/system/xtypes.h>

#if REXGLUE_ENABLE_REXNET
#include <rex/runtime.h>
#include <rex/net/rexnet.h>
#endif

#if REX_PLATFORM_WIN32
// NOTE: must be included last as it expects windows.h to already be included.
#define _WINSOCK_DEPRECATED_NO_WARNINGS  // inet_addr
#include <winsock2.h>                    // NOLINT(build/include_order)
#elif REX_PLATFORM_LINUX
#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#endif

namespace rex {

namespace {
// The XNKID's top byte is a session-type tag the guest reads back, not
// opaque bytes: 0x00 system link, 0xAE online, 0xC0 server. A title asking
// "is this a system-link session" gets a wrong answer if we mint noise.
static constexpr uint8_t kXnkidSystemLink = 0x00;
static constexpr uint8_t kXnkidOnline = 0xAE;

// The session the guest last registered, so XNetInAddrToXnAddr can report a
// real XNKID rather than zeros.
static std::mutex g_session_key_mutex;
static uint8_t g_session_xnkid[8] = {};
static bool g_session_registered = false;

// Set by XNetSetSystemLinkPort; a non-zero value means the title is doing
// System Link, which is what decides the XNKID tag above.
static std::atomic<uint32_t> g_system_link_port{0};
}  // namespace
namespace kernel {
namespace xam {
using namespace rex::system;
using namespace rex::system::xam;

#if !REX_PLATFORM_WIN32
// The title branches on specific WSA error codes (the XRNM pump and the
// download manager treat anything other than WSAEWOULDBLOCK as a dead
// socket), so host errno values must arrive as their Winsock equivalents.
static uint32_t HostSocketErrorToWSA(int err) {
  switch (err) {
    case EAGAIN:
#if EWOULDBLOCK != EAGAIN
    case EWOULDBLOCK:
#endif
      return 0x2733;  // WSAEWOULDBLOCK
    case EINTR:
      return 0x2714;  // WSAEINTR
    case EACCES:
      return 0x271D;  // WSAEACCES
    case EFAULT:
      return 0x271E;  // WSAEFAULT
    case EINPROGRESS:
      return 0x2734;  // WSAEINPROGRESS
    case EMSGSIZE:
      return 0x2738;  // WSAEMSGSIZE
    case ENETUNREACH:
      return 0x2743;  // WSAENETUNREACH
    case ECONNABORTED:
      return 0x2745;  // WSAECONNABORTED
    case ECONNRESET:
      return 0x2746;  // WSAECONNRESET
    case ENOTCONN:
      return 0x2749;  // WSAENOTCONN
    case ETIMEDOUT:
      return 0x274C;  // WSAETIMEDOUT
    case ECONNREFUSED:
      return 0x274D;  // WSAECONNREFUSED
    case EHOSTUNREACH:
      return 0x2751;  // WSAEHOSTUNREACH
    default:
      return 0x2726;  // WSAEINVAL
  }
}
#endif

// https://github.com/G91/TitanOffLine/blob/1e692d9bb9dfac386d08045ccdadf4ae3227bb5e/xkelib/xam/xamNet.h
enum {
  XNCALLER_INVALID = 0x0,
  XNCALLER_TITLE = 0x1,
  XNCALLER_SYSAPP = 0x2,
  XNCALLER_XBDM = 0x3,
  XNCALLER_TEST = 0x4,
  NUM_XNCALLER_TYPES = 0x4,
};

// https://github.com/pmrowla/hl2sdk-csgo/blob/master/common/xbox/xboxstubs.h
typedef struct {
  // FYI: IN_ADDR should be in network-byte order.
  in_addr ina;                    // IP address (zero if not static/DHCP)
  in_addr inaOnline;              // Online IP address (zero if not online)
  rex::be<uint16_t> wPortOnline;  // Online port
  uint8_t abEnet[6];              // Ethernet MAC address
  uint8_t abOnline[20];           // Online identification
} XNADDR;

typedef struct {
  rex::be<int32_t> status;
  rex::be<uint32_t> cina;
  in_addr aina[8];
} XNDNS;

typedef struct {
  uint8_t flags;
  uint8_t reserved;
  rex::be<uint16_t> probes_xmit;
  rex::be<uint16_t> probes_recv;
  rex::be<uint16_t> data_len;
  rex::be<uint32_t> data_ptr;
  rex::be<uint16_t> rtt_min_in_msecs;
  rex::be<uint16_t> rtt_med_in_msecs;
  rex::be<uint32_t> up_bits_per_sec;
  rex::be<uint32_t> down_bits_per_sec;
} XNQOSINFO;

typedef struct {
  rex::be<uint32_t> count;
  rex::be<uint32_t> count_pending;
  XNQOSINFO info[1];
} XNQOS;

struct Xsockaddr_t {
  rex::be<uint16_t> sa_family;
  char sa_data[14];
};

struct X_WSADATA {
  rex::be<uint16_t> version;
  rex::be<uint16_t> version_high;
  char description[256 + 1];
  char system_status[128 + 1];
  rex::be<uint16_t> max_sockets;
  rex::be<uint16_t> max_udpdg;
  rex::be<uint32_t> vendor_info_ptr;
};

struct XWSABUF {
  rex::be<uint32_t> len;
  rex::be<uint32_t> buf_ptr;
};

struct XWSAOVERLAPPED {
  rex::be<uint32_t> internal;
  rex::be<uint32_t> internal_high;
  union {
    struct {
      rex::be<uint32_t> low;
      rex::be<uint32_t> high;
    } offset;  // must be named to avoid GCC error
    rex::be<uint32_t> pointer;
  };
  rex::be<uint32_t> event_handle;
};

void LoadSockaddr(const uint8_t* ptr, sockaddr* out_addr) {
  out_addr->sa_family = memory::load_and_swap<uint16_t>(ptr + 0);
  switch (out_addr->sa_family) {
    case AF_INET: {
      auto in_addr = reinterpret_cast<sockaddr_in*>(out_addr);
      in_addr->sin_port = memory::load_and_swap<uint16_t>(ptr + 2);
      // Maybe? Depends on type.
      in_addr->sin_addr.s_addr = *(uint32_t*)(ptr + 4);
      break;
    }
    default:
      assert_unhandled_case(out_addr->sa_family);
      break;
  }
}

void StoreSockaddr(const sockaddr& addr, uint8_t* ptr) {
  switch (addr.sa_family) {
    case AF_UNSPEC:
      std::memset(ptr, 0, sizeof(addr));
      break;
    case AF_INET: {
      auto& in_addr = reinterpret_cast<const sockaddr_in&>(addr);
      memory::store_and_swap<uint16_t>(ptr + 0, in_addr.sin_family);
      memory::store_and_swap<uint16_t>(ptr + 2, in_addr.sin_port);
      // Maybe? Depends on type.
      memory::store_and_swap<uint32_t>(ptr + 4, in_addr.sin_addr.s_addr);
      break;
    }
    default:
      assert_unhandled_case(addr.sa_family);
      break;
  }
}

// https://github.com/joolswills/mameox/blob/master/MAMEoX/Sources/xbox_Network.cpp#L136
struct XNetStartupParams {
  uint8_t cfgSizeOfStruct;
  uint8_t cfgFlags;
  uint8_t cfgSockMaxDgramSockets;
  uint8_t cfgSockMaxStreamSockets;
  uint8_t cfgSockDefaultRecvBufsizeInK;
  uint8_t cfgSockDefaultSendBufsizeInK;
  uint8_t cfgKeyRegMax;
  uint8_t cfgSecRegMax;
  uint8_t cfgQosDataLimitDiv4;
  uint8_t cfgQosProbeTimeoutInSeconds;
  uint8_t cfgQosProbeRetries;
  uint8_t cfgQosSrvMaxSimultaneousResponses;
  uint8_t cfgQosPairWaitTimeInSeconds;
};

XNetStartupParams xnet_startup_params = {};

#if REXGLUE_ENABLE_REXNET
// XNetStartup/XNetCleanup refcount for the shared RexNet instance (design
// spec §11: init/teardown module handle). Guest code always calls these from
// kernel export dispatch, so no locking is needed beyond the guest's own.
int rexnet_startup_count = 0;

void RexNetStartup() {
  if (!REXCVAR_GET(rexnet)) {
    return;
  }
  if (!net::RexNet::shared()) {
    auto* kernel_state = REX_KERNEL_STATE();

    net::RexNetOptions options;
    options.force_tunnel = REXCVAR_GET(rexnet_force_tunnel);
    options.title_id = kernel_state->title_id();
    options.display_name = REXCVAR_GET(rexnet_display_name);

    auto user_root = kernel_state->emulator()->user_data_root();
    options.data_dir = (user_root.empty() ? std::filesystem::path(".") : user_root) / "rexnet";

    // A name set from the F6 overlay (display_name.txt) outlives the
    // session and wins over the cvar default.
    if (std::ifstream name_file(options.data_dir / "display_name.txt"); name_file.good()) {
      std::string saved((std::istreambuf_iterator<char>(name_file)),
                        std::istreambuf_iterator<char>());
      while (!saved.empty() && (saved.back() == '\n' || saved.back() == '\r')) {
        saved.pop_back();
      }
      if (!saved.empty()) {
        options.display_name = saved;
      }
    }

    const std::string bootstrap = REXCVAR_GET(rexnet_bootstrap);
    if (bootstrap == "amino") {
      options.use_default_bootstrap = true;
    } else if (bootstrap == "none") {
      options.use_default_bootstrap = false;
    } else {
      options.use_default_bootstrap = false;
      for (size_t pos = 0; pos < bootstrap.size();) {
        size_t comma = bootstrap.find(',', pos);
        if (comma == std::string::npos)
          comma = bootstrap.size();
        if (comma > pos) {
          options.bootstrap.emplace_back(bootstrap.substr(pos, comma - pos));
        }
        pos = comma + 1;
      }
    }

    // Per-game configuration (§11.1) is read *before* the engine starts:
    // its [network] lists are engine construction parameters, and loading it
    // afterwards would leave them silently inert.
    std::optional<net::GameConfig> game_config;
    auto config_path = kernel_state->emulator()->FindMetadataPath("rexnet.toml");
    if (config_path) {
      game_config = net::RexNet::LoadGameConfig(*config_path);
    }
    if (game_config) {
      options.relays = game_config->relays;
      options.bootstrap.insert(options.bootstrap.end(), game_config->bootstrap.begin(),
                               game_config->bootstrap.end());
      if (!game_config->bootstrap.empty()) {
        // Project-supplied entry points are additional, not a replacement:
        // the public set stays unless the user turned it off.
        REXKRNL_INFO("rexnet.toml: {} extra bootstrap peer(s)", game_config->bootstrap.size());
      }
    }

    auto* rexnet = net::RexNet::InitializeShared(options);
    if (!rexnet) {
      REXKRNL_ERROR(
          "XNetStartup: RexNet initialization failed; "
          "networking exports will behave as offline");
      return;
    }

    // Inbound game datagrams -> the guest UDP socket bound to that port.
    // Guest TCP (§18): route stream events onto the sockets that own them.
    rexnet->SetStreamSinks(
        // Accept: find the listening socket for this guest port.
        [](uint64_t stream_id, uint32_t peer_vip, uint16_t local_port,
           uint16_t remote_port) -> bool {
          auto listener = XSocket::FindStreamListener(local_port);
          if (!listener) {
            return false;
          }
          listener->QueueAcceptedStream(stream_id, peer_vip, remote_port);
          REXKRNL_INFO("guest TCP: accepted connection on port {} from 10.77.{}.{}:{}", local_port,
                       (peer_vip >> 8) & 0xFF, peer_vip & 0xFF, remote_port);
          return true;
        },
        [](uint64_t stream_id, const uint8_t* data, uint32_t len) {
          if (auto socket = XSocket::FindStream(stream_id)) {
            socket->QueueStreamData(data, len);
          }
        },
        [](uint64_t stream_id) {
          if (auto socket = XSocket::FindStream(stream_id)) {
            socket->OnStreamClosed();
          }
        },
        [](uint32_t peer_vip, uint16_t dst_port, uint64_t stream_id) {
          // Hand the finished connect to the socket that asked for it.
          auto socket = XSocket::ClaimPendingConnect(peer_vip, dst_port);
          if (!socket) {
            // Nobody waiting: the guest closed the socket mid-connect. Do not
            // strand the stream on the peer.
            if (stream_id) {
              if (auto* rexnet = net::RexNet::shared()) {
                rexnet->StreamClose(stream_id);
              }
            }
            return;
          }
          if (stream_id) {
            socket->AdoptStream(stream_id, peer_vip, dst_port);
          } else {
            socket->OnStreamConnectFailed();
          }
          REXKRNL_INFO("guest TCP: connect to 10.77.{}.{}:{} {}", (peer_vip >> 8) & 0xFF,
                       peer_vip & 0xFF, dst_port, stream_id ? "established" : "failed");
        });

    rexnet->SetDatagramSink([](uint32_t src_vip, uint16_t src_port, uint16_t dst_port,
                               const uint8_t* data, uint32_t len) {
      auto socket = XSocket::FindBoundUdp(dst_port);
      // TEMP(refii netplay bring-up): trace the co-op data path. Port 1000 =
      // XRNM link, 1001 = msgJoinSession handshake, 1002/1003 = presence.
      // Dump the head of 1001 messages: JOINRESPONSE code is at byte +3
      // (0=APPROVED 1=FULL 2=DIFFVER 3=NOTHOSTING 4=DIFFVER/DLC).
      char head[40] = {};
      if (dst_port == 1000 || dst_port == 1001) {
        const uint32_t n = len < 12 ? len : 12;
        for (uint32_t i = 0; i < n; ++i) {
          std::snprintf(head + i * 3, 4, "%02X ", data[i]);
        }
      }
      REXKRNL_INFO("RexNet IN: 10.77.{}.{}:{} -> :{} ({} bytes) socket={} head[{}]",
                   (src_vip >> 8) & 0xFF, src_vip & 0xFF, src_port, dst_port, len,
                   socket ? "bound" : "NO-BOUND-SOCKET", head);
      if (!socket) {
        return false;
      }
      return socket->QueuePacket(src_vip, src_port, data, len);
    });

    // Module events -> guest XNotify queue.
    rexnet->SetNotifySink([](uint32_t id, uint32_t param) {
      REX_KERNEL_STATE()->BroadcastNotification(static_cast<XNotificationID>(id), param);
    });

    // Advertise the game's display name to friends (reserved presence rich
    // KV): their overlay shows "Fable II", not a title id.
    if (auto xdbf = kernel_state->title_xdbf(); xdbf.is_valid()) {
      const std::string title_name = xdbf.title();
      REXKRNL_INFO("XNetStartup: advertising game title '{}'", title_name);
      rexnet->SetGameTitleName(title_name);
    } else {
      REXKRNL_INFO("XNetStartup: no title XDBF; game name not advertised");
    }

    // Per-game configuration (design spec §11.1): session model, channel
    // mode, presence context mapping, quirks. Found via the runtime's
    // per-title metadata discovery.
    if (config_path) {
      if (game_config) {
        rexnet->SetGameConfig(std::move(*game_config));
      }
    } else {
      // WARN, not INFO: for a title that ships one this means every
      // per-title behaviour (shard opt-in, advertised_title_id, presence
      // mappings) is silently off. Set log level to debug to see the paths
      // searched (Runtime::FindMetadataPath).
      REXKRNL_WARN(
          "XNetStartup: no rexnet.toml found for this title; per-title config "
          "(shard, advertised_title_id, presence mappings) is INACTIVE");
    }

    // Local profile from identity + display name (design spec §11): a
    // stable offline-format XUID derived from the keypair, so the same
    // install presents the same identity to every title.
    if (auto* profile = kernel_state->user_profile()) {
      profile->set_identity(net::RexNet::XuidFromPeer(rexnet->local_peer_id()),
                            options.display_name);
      // The profile just went from signed-in-locally to signed-in-to-LIVE.
      // Titles cache XamUserGetSigninState and only re-scan on
      // XN_SYS_SIGNINCHANGED (Fable 2 keeps NLivePresence in
      // LIVE_PRESENCE_MODE_DISABLED without it), so tell them — and follow
      // with XN_LIVE_CONNECTIONCHANGED so logon-watchers see the connection
      // come up.
      kernel_state->BroadcastNotification(net::kXNotifySystemSignInChanged, 1);
      kernel_state->BroadcastNotification(net::kXNotifyLiveConnectionChanged,
                                          net::kXOnlineLogonConnectionEstablished);
      REXKRNL_INFO(
          "XNetStartup: profile signed in to Live (xuid {:016X}); "
          "signin-changed + connection-established broadcast",
          profile->xuid());
    }
  }
  ++rexnet_startup_count;
}
#endif  // REXGLUE_ENABLE_REXNET

u32 NetDll_XNetStartup_entry(u32 caller, ppc_ptr_t<XNetStartupParams> params) {
  if (params) {
    assert_true(params->cfgSizeOfStruct == sizeof(XNetStartupParams));
    std::memcpy(&xnet_startup_params, params, sizeof(XNetStartupParams));
  }

#if REXGLUE_ENABLE_REXNET
  RexNetStartup();
#endif

  return 0;
}

u32 NetDll_XNetCleanup_entry(u32 caller, mapped_void params) {
#if REXGLUE_ENABLE_REXNET
  if (rexnet_startup_count > 0 && --rexnet_startup_count == 0) {
    net::RexNet::DestroyShared();
  }
#endif

  return 0;
}

u32 NetDll_XNetGetOpt_entry(u32 one, u32 option_id, mapped_void buffer_ptr,
                            mapped_u32 buffer_size) {
  assert_true(one == 1);
  switch (option_id) {
    case 1:
      if (*buffer_size < sizeof(XNetStartupParams)) {
        *buffer_size = sizeof(XNetStartupParams);
        return 0x2738;  // WSAEMSGSIZE
      }
      std::memcpy(buffer_ptr, &xnet_startup_params, sizeof(XNetStartupParams));
      return 0;
    default:
      REXKRNL_ERROR("NetDll_XNetGetOpt: option {} unimplemented", option_id);
      return 0x2726;  // WSAEINVAL
  }
}

u32 NetDll_XNetRandom_entry(u32 caller, mapped_void buffer_ptr, u32 length) {
  // For now, constant values.
  // This makes replicating things easier.
  std::memset(buffer_ptr, 0xBB, length);

  return 0;
}

u32 NetDll_WSAStartup_entry(u32 caller, u16 version, ppc_ptr_t<X_WSADATA> data_ptr) {
// TODO(benvanik): abstraction layer needed.
#if REX_PLATFORM_WIN32
  WSADATA wsaData;
  ZeroMemory(&wsaData, sizeof(WSADATA));
  int ret = WSAStartup(version, &wsaData);

  auto data_out = REX_KERNEL_MEMORY()->TranslateVirtual(data_ptr.guest_address());

  if (data_ptr) {
    data_ptr->version = wsaData.wVersion;
    data_ptr->version_high = wsaData.wHighVersion;
    std::memcpy(&data_ptr->description, wsaData.szDescription, 0x100);
    std::memcpy(&data_ptr->system_status, wsaData.szSystemStatus, 0x80);
    data_ptr->max_sockets = wsaData.iMaxSockets;
    data_ptr->max_udpdg = wsaData.iMaxUdpDg;

    // Some games (5841099F) want this value round-tripped - they'll compare if
    // it changes and bugcheck if it does.
    uint32_t vendor_ptr = memory::load_and_swap<uint32_t>(data_out + 0x190);
    memory::store_and_swap<uint32_t>(data_out + 0x190, vendor_ptr);
  }
#else
  int ret = 0;
  if (data_ptr) {
    // Guess these values!
    data_ptr->version = version;
    data_ptr->description[0] = '\0';
    data_ptr->system_status[0] = '\0';
    data_ptr->max_sockets = 100;
    data_ptr->max_udpdg = 1024;
  }
#endif

  // DEBUG
  /*
  auto xam = REX_KERNEL_STATE()->GetKernelModule<XamModule>("xam.xex");
  if (!xam->xnet()) {
    auto xnet = new XNet(REX_KERNEL_STATE());
    xnet->Initialize();

    xam->set_xnet(xnet);
  }
  */

  return ret;
}

u32 NetDll_WSACleanup_entry(u32 caller) {
  // This does nothing. Xenia needs WSA running.
  return 0;
}

u32 NetDll_WSAGetLastError_entry() {
  return XThread::GetLastError();
}

// Winsock signature (9 args):
//   WSARecvFrom(s, lpBuffers, dwBufferCount, lpNumberOfBytesRecvd, lpFlags,
//               lpFrom, lpFromlen, lpOverlapped, lpCompletionRoutine)
// lpFromlen sits between lpFrom and lpOverlapped and must not be omitted --
// dropping it shifts every later argument by one register, so lpOverlapped is
// read as lpFromlen and lpCompletionRoutine as lpOverlapped. That silently
// breaks every overlapped receive: the completion is written into the caller's
// from-length instead of its OVERLAPPED, so the guest never learns a datagram
// arrived (XRNM's CXrnmEndpoint_QueueRecvToSocket waits on exactly this).
u32 NetDll_WSARecvFrom_entry(u32 caller, u32 socket_handle, ppc_ptr_t<XWSABUF> buffers_ptr,
                             u32 buffer_count, mapped_u32 num_bytes_recv, mapped_u32 flags_ptr,
                             ppc_ptr_t<XSOCKADDR_IN> from_addr, mapped_u32 fromlen_ptr,
                             ppc_ptr_t<XWSAOVERLAPPED> overlapped_ptr,
                             mapped_void completion_routine_ptr) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    XThread::SetLastError(0x2736);  // WSAENOTSOCK
    return -1;
  }
  if (completion_routine_ptr) {
    // Winsock alertable I/O (LPWSAOVERLAPPED_COMPLETION_ROUTINE). This never
    // completes inline: the routine is dispatched by an APC when the issuing
    // thread next enters an alertable wait, and it both consumes the datagram
    // and re-posts the receive. XRNM-style netcode (port-1000 reliable links)
    // is driven entirely this way, so a completion routine MUST be honored --
    // otherwise a host that receives a peer's connect probe never processes it
    // and never sends its half of the handshake.
    if (!overlapped_ptr) {
      XThread::SetLastError(0x271E);  // WSAEFAULT
      return -1;
    }
    std::vector<XSocket::WsaRecvBuffer> buffers;
    buffers.reserve(buffer_count);
    for (uint32_t i = 0; i < buffer_count; i++) {
      buffers.push_back({buffers_ptr[i].buf_ptr, buffers_ptr[i].len});
    }
    if (!socket->SetPendingWsaRecv(std::move(buffers), overlapped_ptr.guest_address(),
                                   from_addr ? from_addr.guest_address() : 0,
                                   fromlen_ptr ? fromlen_ptr.guest_address() : 0,
                                   completion_routine_ptr.guest_address(),
                                   XThread::GetCurrentThreadHandle())) {
      XThread::SetLastError(0x2734);  // WSAEINPROGRESS
      return -1;
    }
    // If a datagram is already queued, complete now (the APC is still deferred
    // to the next alertable wait, per Winsock semantics).
    socket->PumpPendingWsaRecv();
    XThread::SetLastError(997);  // WSA_IO_PENDING
    return -1;
  }

  // Immediate attempt: RexNet queue or host socket.
  uint8_t tmp[2048];
  N_XSOCKADDR_IN native_from{};
  int received = socket->TryRecvFrom(tmp, sizeof(tmp), &native_from);
  if (received >= 0) {
    uint32_t copied = 0;
    for (uint32_t i = 0; i < buffer_count && copied < (uint32_t)received; i++) {
      uint32_t chunk = std::min<uint32_t>(buffers_ptr[i].len, (uint32_t)received - copied);
      std::memcpy(REX_KERNEL_MEMORY()->TranslateVirtual(buffers_ptr[i].buf_ptr), tmp + copied,
                  chunk);
      copied += chunk;
    }
    if (from_addr) {
      from_addr->sin_family = native_from.sin_family;
      from_addr->sin_port = native_from.sin_port;
      from_addr->sin_addr = native_from.sin_addr;
      std::memset(from_addr->x_sin_zero, 0, sizeof(from_addr->x_sin_zero));
      if (fromlen_ptr) {
        *fromlen_ptr = sizeof(XSOCKADDR_IN);
      }
    }
    if (num_bytes_recv) {
      *num_bytes_recv = copied;
    }
    if (overlapped_ptr) {
      overlapped_ptr->internal = 0;  // success
      overlapped_ptr->internal_high = copied;
      if (overlapped_ptr->event_handle) {
        auto ev = REX_KERNEL_OBJECTS()->LookupObject<XEvent>(overlapped_ptr->event_handle);
        if (ev) {
          ev->Set(0, false);
        }
      }
    }
    return 0;
  }

  if (overlapped_ptr) {
    // Register the pending receive; completion happens when a RexNet packet
    // is queued or the socket poller sees host data (XRNM-style flow:
    // WSA_IO_PENDING -> wait on event -> WSAGetOverlappedResult).
    std::vector<XSocket::WsaRecvBuffer> buffers;
    buffers.reserve(buffer_count);
    for (uint32_t i = 0; i < buffer_count; i++) {
      buffers.push_back({buffers_ptr[i].buf_ptr, buffers_ptr[i].len});
    }
    if (!socket->SetPendingWsaRecv(std::move(buffers), overlapped_ptr.guest_address(),
                                   from_addr ? from_addr.guest_address() : 0,
                                   fromlen_ptr ? fromlen_ptr.guest_address() : 0)) {
      XThread::SetLastError(0x2734);  // WSAEINPROGRESS
      return -1;
    }
    XThread::SetLastError(997);  // WSA_IO_PENDING
    return -1;
  }

  // No overlapped: plain blocking receive into the guest buffers.
  uint32_t from_len = sizeof(N_XSOCKADDR_IN);
  received = socket->RecvFrom(tmp, sizeof(tmp), 0, &native_from, &from_len);
  if (received < 0) {
    XThread::SetLastError(0x2733);  // WSAEWOULDBLOCK
    return -1;
  }
  uint32_t copied = 0;
  for (uint32_t i = 0; i < buffer_count && copied < (uint32_t)received; i++) {
    uint32_t chunk = std::min<uint32_t>(buffers_ptr[i].len, (uint32_t)received - copied);
    std::memcpy(REX_KERNEL_MEMORY()->TranslateVirtual(buffers_ptr[i].buf_ptr), tmp + copied, chunk);
    copied += chunk;
  }
  if (from_addr) {
    from_addr->sin_family = native_from.sin_family;
    from_addr->sin_port = native_from.sin_port;
    from_addr->sin_addr = native_from.sin_addr;
    std::memset(from_addr->x_sin_zero, 0, sizeof(from_addr->x_sin_zero));
    if (fromlen_ptr) {
      *fromlen_ptr = sizeof(XSOCKADDR_IN);
    }
  }
  if (num_bytes_recv) {
    *num_bytes_recv = copied;
  }
  return 0;
}

u32 NetDll_WSAGetOverlappedResult_entry(u32 caller, u32 socket_handle,
                                        ppc_ptr_t<XWSAOVERLAPPED> overlapped_ptr,
                                        mapped_u32 bytes_ptr, u32 wait, mapped_u32 flags_ptr) {
  if (!overlapped_ptr) {
    XThread::SetLastError(0x271E);  // WSAEFAULT
    return 0;
  }
  uint32_t status = overlapped_ptr->internal;
  if (status == 0x103 /* STATUS_PENDING */) {
    if (!wait) {
      XThread::SetLastError(996);  // WSA_IO_INCOMPLETE
      return 0;
    }
    if (overlapped_ptr->event_handle) {
      auto ev = REX_KERNEL_OBJECTS()->LookupObject<XEvent>(overlapped_ptr->event_handle);
      if (ev) {
        ev->Wait(0, 0, true, nullptr);
      }
    } else {
      while ((status = overlapped_ptr->internal) == 0x103) {
        rex::thread::Sleep(std::chrono::milliseconds(1));
      }
    }
    status = overlapped_ptr->internal;
  }
  if (status == 0) {
    if (bytes_ptr) {
      *bytes_ptr = static_cast<uint32_t>(overlapped_ptr->internal_high);
    }
    if (flags_ptr) {
      *flags_ptr = 0;
    }
    return 1;
  }
  XThread::SetLastError(status);
  return 0;
}

// If the socket is a VDP socket, buffer 0 is the game data length, and buffer 1
// is the unencrypted game data.
u32 NetDll_WSASendTo_entry(u32 caller, u32 socket_handle, ppc_ptr_t<XWSABUF> buffers,
                           u32 num_buffers, mapped_u32 num_bytes_sent, u32 flags,
                           ppc_ptr_t<XSOCKADDR_IN> to_ptr, u32 to_len,
                           ppc_ptr_t<XWSAOVERLAPPED> overlapped, mapped_void completion_routine) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  // Our sockets implementation doesn't support multiple buffers, so we need
  // to combine the buffers the game has given us!
  std::vector<uint8_t> combined_buffer_mem;
  uint32_t combined_buffer_size = 0;
  uint32_t combined_buffer_offset = 0;
  for (uint32_t i = 0; i < num_buffers; i++) {
    combined_buffer_size += buffers[i].len;
    combined_buffer_mem.resize(combined_buffer_size);
    uint8_t* combined_buffer = combined_buffer_mem.data();

    std::memcpy(combined_buffer + combined_buffer_offset,
                REX_KERNEL_MEMORY()->TranslateVirtual(buffers[i].buf_ptr), buffers[i].len);
    combined_buffer_offset += buffers[i].len;
  }

  N_XSOCKADDR_IN native_to(to_ptr);
  socket->SendTo(combined_buffer_mem.data(), combined_buffer_size, flags, &native_to, to_len);

  if (num_bytes_sent) {
    *num_bytes_sent = combined_buffer_size;
  }
  // The send is synchronous for us, so an overlapped send completes
  // immediately (XRNM issues overlapped WSASendTo on the port-1000 link).
  // Report success now and signal the completion event/handle so the
  // guest's CNwmIo wait wakes up. A completion routine, if provided, is
  // dispatched by the guest's own alertable wait against this event.
  if (overlapped) {
    overlapped->internal = 0;  // success
    overlapped->internal_high = combined_buffer_size;
    if (overlapped->event_handle) {
      if (auto ev = REX_KERNEL_OBJECTS()->LookupObject<XEvent>(overlapped->event_handle)) {
        ev->Set(0, false);
      }
    }
  }

  return 0;
}

u32 NetDll_WSAWaitForMultipleEvents_entry(u32 num_events, mapped_u32 events, u32 wait_all,
                                          u32 timeout, u32 alertable) {
  if (num_events > 64) {
    XThread::SetLastError(87);  // ERROR_INVALID_PARAMETER
    return ~0u;
  }

  uint64_t timeout_wait = (uint64_t)timeout;

  X_STATUS result = 0;
  do {
    result = xboxkrnl::xeNtWaitForMultipleObjectsEx(num_events, events, wait_all, 1, alertable,
                                                    timeout != -1 ? &timeout_wait : nullptr);
  } while (result == X_STATUS_ALERTED);

  // An alertable wait that ran a user APC (a Winsock completion routine) must
  // report WSA_WAIT_IO_COMPLETION so the caller loops back into its wait rather
  // than mistaking it for event[0] being signaled. This drives XRNM's recv
  // completion-routine dispatch on the port-1000 reliable link.
  if (result == X_STATUS_USER_APC) {
    return 0x000000C0;  // WSA_WAIT_IO_COMPLETION / WAIT_IO_COMPLETION
  }

  if (XFAILED(result)) {
    uint32_t error = xboxkrnl::xeRtlNtStatusToDosError(result);
    XThread::SetLastError(error);
    return ~0u;
  }
  return 0;
}

// Associate a WSAEVENT with a socket so the guest can wait for readability
// instead of polling. Armored Core 4 drives its socket loop this way; with
// this stubbed the event never fires and the title waits forever.
// A title asking for its own bound address. Report the RexNet identity, not
// the host socket's: the guest's world is the virtual network, and a title
// that puts this address in a packet for peers must give them one they can
// actually reach.
// Queried by titles before deciding buffer sizes or reporting errors. The
// value has to be plausible rather than zero: a title that reads SO_ERROR as
// 0 concludes the socket is healthy, and one that reads a zero buffer size
// may refuse to send at all.
u32 NetDll_getsockopt_entry(u32 caller, u32 socket_handle, u32 level, u32 optname,
                            mapped_void optval, mapped_u32 optlen) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket || !optval || !optlen) {
    XThread::SetLastError(0x2736);  // WSAENOTSOCK
    return -1;
  }
  if (*optlen < sizeof(uint32_t)) {
    XThread::SetLastError(0x2726);  // WSAEFAULT
    return -1;
  }
  constexpr uint32_t kSolSocket = 0xFFFF;
  uint32_t value = 0;
  if (level == kSolSocket) {
    switch (optname) {
      case 0x1001:  // SO_SNDBUF
      case 0x1002:  // SO_RCVBUF
        // The RexNet carrier buffers for us; report a conventional size.
        value = 64 * 1024;
        break;
      case 0x1007:  // SO_ERROR
        value = 0;  // no pending error
        break;
      case 0x0020:  // SO_BROADCAST
        value = 1;
        break;
      default:
        value = 0;
        break;
    }
  }
  *reinterpret_cast<rex::be<uint32_t>*>(static_cast<void*>(optval)) = value;
  *optlen = sizeof(uint32_t);
  return 0;
}

// Drops the local mapping a title made with XNetXnAddrToInAddr. RexNet keeps
// virtual IPs stable for the life of the peer -- the guest may still hold the
// address in a live socket -- so this releases nothing and simply succeeds.
u32 NetDll_XNetUnregisterInAddr_entry(u32 caller, u32 in_addr) {
  return 0;
}

u32 NetDll_getsockname_entry(u32 caller, u32 socket_handle, ppc_ptr_t<XSOCKADDR_IN> name,
                             mapped_u32 name_len) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket || !name) {
    XThread::SetLastError(0x2736);  // WSAENOTSOCK
    return -1;
  }
  if (name_len && *name_len < sizeof(XSOCKADDR_IN)) {
    XThread::SetLastError(0x2726);  // WSAEFAULT
    return -1;
  }
  name->sin_family = 2;  // AF_INET
  name->sin_port = socket->bound_port();
#if REXGLUE_ENABLE_REXNET
  auto* rexnet = net::RexNet::shared();
  name->sin_addr = rexnet ? rexnet->local_vip() : htonl(INADDR_LOOPBACK);
#else
  name->sin_addr = htonl(INADDR_LOOPBACK);
#endif
  std::memset(name->x_sin_zero, 0, sizeof(name->x_sin_zero));
  if (name_len) {
    *name_len = sizeof(XSOCKADDR_IN);
  }
  return 0;
}

u32 NetDll_WSAEventSelect_entry(u32 caller, u32 socket_handle, u32 event_handle, u32 event_mask) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    XThread::SetLastError(0x2736);  // WSAENOTSOCK
    return -1;
  }
  socket->SetEventSelect(event_handle, event_mask);
  REXKRNL_DEBUG("WSAEventSelect(socket={:08X}, event={:08X}, mask={:08X})", socket_handle,
                event_handle, event_mask);
  return 0;
}

u32 NetDll_WSACreateEvent_entry() {
  XEvent* ev = new XEvent(REX_KERNEL_STATE());
  ev->Initialize(true, false);
  return ev->handle();
}

u32 NetDll_WSACloseEvent_entry(u32 event_handle) {
  X_STATUS result = REX_KERNEL_OBJECTS()->ReleaseHandle(event_handle);
  if (XFAILED(result)) {
    uint32_t error = xboxkrnl::xeRtlNtStatusToDosError(result);
    XThread::SetLastError(error);
    return 0;
  }
  return 1;
}

u32 NetDll_WSAResetEvent_entry(u32 event_handle) {
  X_STATUS result = xboxkrnl::xeNtClearEvent(event_handle);
  if (XFAILED(result)) {
    uint32_t error = xboxkrnl::xeRtlNtStatusToDosError(result);
    XThread::SetLastError(error);
    return 0;
  }
  return 1;
}

u32 NetDll_WSASetEvent_entry(u32 event_handle) {
  X_STATUS result = xboxkrnl::xeNtSetEvent(event_handle, nullptr);
  if (XFAILED(result)) {
    uint32_t error = xboxkrnl::xeRtlNtStatusToDosError(result);
    XThread::SetLastError(error);
    return 0;
  }
  return 1;
}

struct XnAddrStatus {
  // Address acquisition is not yet complete
  static const uint32_t XNET_GET_XNADDR_PENDING = 0x00000000;
  // XNet is uninitialized or no debugger found
  static const uint32_t XNET_GET_XNADDR_NONE = 0x00000001;
  // Host has ethernet address (no IP address)
  static const uint32_t XNET_GET_XNADDR_ETHERNET = 0x00000002;
  // Host has statically assigned IP address
  static const uint32_t XNET_GET_XNADDR_STATIC = 0x00000004;
  // Host has DHCP assigned IP address
  static const uint32_t XNET_GET_XNADDR_DHCP = 0x00000008;
  // Host has PPPoE assigned IP address
  static const uint32_t XNET_GET_XNADDR_PPPOE = 0x00000010;
  // Host has one or more gateways configured
  static const uint32_t XNET_GET_XNADDR_GATEWAY = 0x00000020;
  // Host has one or more DNS servers configured
  static const uint32_t XNET_GET_XNADDR_DNS = 0x00000040;
  // Host is currently connected to online service
  static const uint32_t XNET_GET_XNADDR_ONLINE = 0x00000080;
  // Network configuration requires troubleshooting
  static const uint32_t XNET_GET_XNADDR_TROUBLESHOOT = 0x00008000;
};

#if REXGLUE_ENABLE_REXNET
// Synthesize the XNADDR for a peer (or the local node): virtual IP in the
// address fields, stable pseudo-MAC in abEnet, peer-id key in abOnline
// (design spec §11).
void FillRexNetXnAddr(XNADDR* addr, uint32_t virtual_ip, const RexNetPeerId& peer) {
  addr->ina.s_addr = htonl(virtual_ip);
  addr->inaOnline.s_addr = htonl(virtual_ip);
  addr->wPortOnline = 3074;
  net::RexNet::EnetAddr(peer, addr->abEnet);
  net::RexNet::OnlineKey(peer, addr->abOnline);
}
#endif  // REXGLUE_ENABLE_REXNET

#if REXGLUE_ENABLE_REXNET
// Is this address us? Our own address moves when we join or leave a shard
// (§17.3.5), so accept the current one *and* the pre-shard default: the guest
// may still be holding an address it obtained earlier, and mistaking our own
// address for a peer's would send us punching at ourselves.
static bool IsLocalVip(net::RexNet* rexnet, uint32_t in_addr) {
  return in_addr == net::RexNet::kLocalVip || (rexnet && in_addr == rexnet->local_vip());
}
#endif

u32 NetDll_XNetGetTitleXnAddr_entry(u32 caller, ppc_ptr_t<XNADDR> addr_ptr) {
#if REXGLUE_ENABLE_REXNET
  if (auto* rexnet = net::RexNet::shared()) {
    // Our address on the virtual network -- our host on the shard's /24 when
    // we are on one, else the reserved-range default.
    FillRexNetXnAddr(addr_ptr, rexnet->local_vip(), rexnet->local_peer_id());
    // ETHERNET included: a title checking for a physical link finds none
    // without it, and the virtual adapter is always "plugged in".
    return XnAddrStatus::XNET_GET_XNADDR_ETHERNET | XnAddrStatus::XNET_GET_XNADDR_STATIC |
           XnAddrStatus::XNET_GET_XNADDR_GATEWAY | XnAddrStatus::XNET_GET_XNADDR_DNS |
           XnAddrStatus::XNET_GET_XNADDR_ONLINE;
  }
#endif

  // Just return a loopback address atm.
  addr_ptr->ina.s_addr = htonl(INADDR_LOOPBACK);
  addr_ptr->inaOnline.s_addr = 0;
  addr_ptr->wPortOnline = 0;

  // TODO(gibbed): A proper mac address.
  // RakNet's 360 version appears to depend on abEnet to create "random" 64-bit
  // numbers. A zero value will cause RakPeer::Startup to fail. This causes
  // 58411436 to crash on startup.
  // The 360-specific code is scrubbed from the RakNet repo, but there's still
  // traces of what it's doing which match the game code.
  // https://github.com/facebookarchive/RakNet/blob/master/Source/RakPeer.cpp#L382
  // https://github.com/facebookarchive/RakNet/blob/master/Source/RakPeer.cpp#L4527
  // https://github.com/facebookarchive/RakNet/blob/master/Source/RakPeer.cpp#L4467
  // "Mac address is a poor solution because you can't have multiple connections
  // from the same system"
  std::memset(addr_ptr->abEnet, 0xCC, 6);

  std::memset(addr_ptr->abOnline, 0, 20);

  return XnAddrStatus::XNET_GET_XNADDR_STATIC;
}

u32 NetDll_XNetGetDebugXnAddr_entry(u32 caller, ppc_ptr_t<XNADDR> addr_ptr) {
  addr_ptr.Zero();

  // XNET_GET_XNADDR_NONE causes caller to gracefully return.
  return XnAddrStatus::XNET_GET_XNADDR_NONE;
}

u32 NetDll_XNetXnAddrToMachineId_entry(u32 caller, ppc_ptr_t<XNADDR> addr_ptr, mapped_u64 id_ptr) {
  // Derive a stable 64-bit machine id from the XNADDR's online identity so
  // sessions that key peers by machine id (Fable 2's NLivePresence raises a
  // fatal EError 7 if this fails) get a consistent, unique value. Real
  // console ids carry the 0xFA marker byte in the high position; mirror
  // that. Returning failure here (the old "act offline" stub) aborts the
  // whole online path once presence is enabled.
#if REXGLUE_ENABLE_REXNET
  if (addr_ptr && id_ptr && net::RexNet::shared()) {
    const XNADDR* addr = addr_ptr;
    uint64_t id = 0;
    // Fold the 20-byte abOnline key into 56 bits; abOnline is the stable
    // per-identity multihash prefix (same source as XUID derivation).
    for (int i = 0; i < 20; ++i) {
      id = (id * 131) + addr->abOnline[i];
    }
    id = 0xFA00000000000000ull | (id & 0x00FFFFFFFFFFFFFFull);
    *id_ptr = id;
    return 0;
  }
#endif
  return 1;
}

void NetDll_XNetInAddrToString_entry(u32 caller, u32 in_addr, mapped_string string_out,
                                     u32 string_size) {
  rex::string::copy_truncating(string_out, "666.666.666.666", string_size);
}

// This converts a XNet address to an IN_ADDR. The IN_ADDR is used for
// subsequent socket calls (like a handle to a XNet address)
u32 NetDll_XNetXnAddrToInAddr_entry(u32 caller, ppc_ptr_t<XNADDR> xn_addr, mapped_void xid,
                                    mapped_void in_addr) {
#if REXGLUE_ENABLE_REXNET
  auto* rexnet = net::RexNet::shared();
  if (rexnet && xn_addr && in_addr) {
    uint8_t local_key[20];
    net::RexNet::OnlineKey(rexnet->local_peer_id(), local_key);

    uint32_t virtual_ip = 0;
    if (!std::memcmp(xn_addr->abOnline, local_key, 20)) {
      virtual_ip = rexnet->local_vip();
    } else if (auto found = rexnet->VipFromOnlineKey(xn_addr->abOnline)) {
      virtual_ip = *found;
    }

    if (virtual_ip) {
      rex::be<uint32_t> value = virtual_ip;
      std::memcpy(in_addr, &value, sizeof(value));
      // Resolving a peer's XNADDR to an in_addr is the game announcing it is
      // about to talk to that peer. Fable 2's XRNM co-op path builds its
      // link straight from this in_addr and never calls XNetConnect, so kick
      // the game-plane punch here (idempotent) — otherwise the XRNM
      // datagrams the title sends next have no punched path to route over.
      if (!IsLocalVip(rexnet, virtual_ip)) {
        rexnet->Connect(virtual_ip);
      }
      REXKRNL_INFO(
          "XNetXnAddrToInAddr -> 10.77.{}.{} (peer connect starting, "
          "punching game plane)",
          (virtual_ip >> 8) & 0xFF, virtual_ip & 0xFF);
      return 0;
    }
    REXKRNL_WARN("XNetXnAddrToInAddr: unknown XNADDR (peer not yet registered)");
  }
#endif
  return 1;
}

// Does the reverse of the above. The IN_ADDR is passed by value.
u32 NetDll_XNetInAddrToXnAddr_entry(u32 caller, u32 in_addr, ppc_ptr_t<XNADDR> xn_addr,
                                    mapped_void xid) {
#if REXGLUE_ENABLE_REXNET
  auto* rexnet = net::RexNet::shared();
  if (rexnet && xn_addr) {
    if (xid) {
      // Report the session the guest registered; zeros would tell a title its
      // own session does not exist.
      std::lock_guard<std::mutex> lock(g_session_key_mutex);
      if (g_session_registered) {
        std::memcpy(static_cast<void*>(xid), g_session_xnkid, sizeof(g_session_xnkid));
      } else {
        std::memset(xid, 0, 8);
      }
    }
    // System Link discovery resolves the broadcast address before it knows any
    // peer. Failing here leaves the title with nothing to address, so answer
    // with our own identity on the broadcast address.
    const bool is_broadcast = in_addr == 0xFFFFFFFFu || (in_addr & 0xFFu) == 0xFFu;
    if (is_broadcast) {
      FillRexNetXnAddr(xn_addr, in_addr, rexnet->local_peer_id());
      return 0;
    }
    if (IsLocalVip(rexnet, in_addr)) {
      FillRexNetXnAddr(xn_addr, in_addr, rexnet->local_peer_id());
      return 0;
    }
    if (auto peer = rexnet->PeerFromVip(in_addr)) {
      FillRexNetXnAddr(xn_addr, in_addr, *peer);
      return 0;
    }
  }
#endif
  return 1;
}

// XNetConnect/XNetGetConnectStatus are load-bearing for XRNM titles: the
// link state machine calls XNetConnect itself and gates every send on
// XNET_CONNECT_STATUS_CONNECTED (design spec §11 / Fable 2 IDB decode).
u32 NetDll_XNetConnect_entry(u32 caller, u32 in_addr) {
#if REXGLUE_ENABLE_REXNET
  auto* rexnet = net::RexNet::shared();
  if (rexnet) {
    if (IsLocalVip(rexnet, in_addr) || in_addr == htonl(INADDR_LOOPBACK) ||
        in_addr == INADDR_LOOPBACK) {
      return 0;  // self-connect is trivially up
    }
    const bool ok = rexnet->Connect(in_addr);
    REXKRNL_INFO("XNetConnect(10.77.{}.{}) -> {} (punching game plane)", (in_addr >> 8) & 0xFF,
                 in_addr & 0xFF, ok ? "started" : "unknown-vip");
    return ok ? 0 : 1;
  }
#endif
  return 1;
}

enum XNetConnectStatus : u32 {
  XNET_CONNECT_STATUS_IDLE = 0,
  XNET_CONNECT_STATUS_PENDING = 1,
  XNET_CONNECT_STATUS_CONNECTED = 2,
  XNET_CONNECT_STATUS_LOST = 3,
};

u32 NetDll_XNetGetConnectStatus_entry(u32 caller, u32 in_addr) {
#if REXGLUE_ENABLE_REXNET
  auto* rexnet = net::RexNet::shared();
  if (rexnet) {
    if (IsLocalVip(rexnet, in_addr) || in_addr == htonl(INADDR_LOOPBACK) ||
        in_addr == INADDR_LOOPBACK) {
      return XNET_CONNECT_STATUS_CONNECTED;
    }
    auto status = rexnet->GetConnectStatus(in_addr);
    // XRNM gates every send on this. The host side accepts an inbound XRNM
    // link but never calls XNetConnect for the joiner, so its status for
    // that peer sits at IDLE and it refuses to send its half of the
    // handshake. Kick the (idempotent) punch here for any known peer the
    // title is polling — it progresses IDLE -> PENDING -> CONNECTED, and the
    // datagram plane both ways is already up.
    if (status == net::ConnectStatus::kIdle) {
      if (rexnet->Connect(in_addr)) {
        status = rexnet->GetConnectStatus(in_addr);
      }
    }
    // Log only on transition (this is polled every frame) so we can see the
    // game plane come up (PENDING -> CONNECTED) without spamming.
    static std::unordered_map<uint32_t, net::ConnectStatus> last;
    if (auto it = last.find(in_addr); it == last.end() || it->second != status) {
      last[in_addr] = status;
      const char* name = status == net::ConnectStatus::kConnected ? "CONNECTED"
                         : status == net::ConnectStatus::kPending ? "PENDING"
                         : status == net::ConnectStatus::kLost    ? "LOST"
                                                                  : "IDLE";
      REXKRNL_INFO("XNetGetConnectStatus(10.77.{}.{}) = {}", (in_addr >> 8) & 0xFF, in_addr & 0xFF,
                   name);
    }
    return static_cast<u32>(status);
  }
#endif
  return XNET_CONNECT_STATUS_LOST;
}

// Key registration is a no-op: the RexNet transport is already encrypted
// end-to-end, and XNKID/XNKEY only survive as opaque session identifiers.
u32 NetDll_XNetCreateKey_entry(u32 caller, mapped_void xnkid, mapped_void xnkey) {
  // A title that declared a system-link port is running System Link; anything
  // else is treated as online, which is what RexNet presents itself as.
  // Derived from what the guest did rather than configured per title (§11.0).
  const uint8_t tag =
      g_system_link_port.load(std::memory_order_relaxed) ? kXnkidSystemLink : kXnkidOnline;

  std::random_device rd;
  uint8_t kid[8];
  kid[0] = tag;
  for (int i = 1; i < 8; i++) {
    kid[i] = static_cast<uint8_t>(rd());
  }
  if (xnkid) {
    std::memcpy(static_cast<void*>(xnkid), kid, sizeof(kid));
  }
  if (xnkey) {
    uint8_t key[16];
    for (auto& b : key) {
      b = static_cast<uint8_t>(rd());
    }
    std::memcpy(static_cast<void*>(xnkey), key, sizeof(key));
  }
  REXKRNL_INFO("XNetCreateKey -> {} session", tag == kXnkidSystemLink ? "system-link" : "online");
  return 0;
}

u32 NetDll_XNetRegisterKey_entry(u32 caller, mapped_void xnkid, mapped_void xnkey) {
  if (xnkid) {
    std::lock_guard<std::mutex> lock(g_session_key_mutex);
    std::memcpy(g_session_xnkid, static_cast<const void*>(xnkid), sizeof(g_session_xnkid));
    g_session_registered = true;
  }
  return 0;
}

u32 NetDll_XNetUnregisterKey_entry(u32 caller, mapped_void xnkid) {
  return 0;
}

// https://www.google.com/patents/WO2008112448A1?cl=en
// Reserves the port a title uses for System Link discovery. Titles broadcast
// on it to find each other rather than naming a host, so a shard's subnet
// broadcast (§17.3.6) is what carries them.
//
// The port is stored verbatim and returned verbatim by XNetGetSystemLinkPort,
// so a title that sets and reads it back sees exactly what it wrote,
// whichever byte order it chose.

u32 NetDll_XNetSetSystemLinkPort_entry(u32 caller, u32 port) {
  g_system_link_port.store(port, std::memory_order_relaxed);
  REXKRNL_INFO("XNetSetSystemLinkPort({})", port);
  // 0 = success. This previously returned 1, which a title reads as failure
  // and takes as "System Link unavailable" -- it would give up before ever
  // broadcasting.
  return 0;
}

u32 NetDll_XNetGetSystemLinkPort_entry(u32 caller) {
  return g_system_link_port.load(std::memory_order_relaxed);
}

// Whether a peer's broadcast is version-compatible with ours. Everyone on a
// shard runs the same title as far as we can tell, so report compatible
// rather than leaving the title to interpret a stub's zero.
u32 NetDll_XNetGetBroadcastVersionStatus_entry(u32 caller, u32 reset) {
  return 0;  // XNET_BROADCAST_VERSION_OK
}

// https://github.com/ILOVEPIE/Cxbx-Reloaded/blob/master/src/CxbxKrnl/EmuXOnline.h#L39
struct XEthernetStatus {
  static const uint32_t XNET_ETHERNET_LINK_ACTIVE = 0x01;
  static const uint32_t XNET_ETHERNET_LINK_100MBPS = 0x02;
  static const uint32_t XNET_ETHERNET_LINK_10MBPS = 0x04;
  static const uint32_t XNET_ETHERNET_LINK_FULL_DUPLEX = 0x08;
  static const uint32_t XNET_ETHERNET_LINK_HALF_DUPLEX = 0x10;
};

u32 NetDll_XNetGetEthernetLinkStatus_entry(u32 caller) {
  // Zero reads as "cable unplugged", which contradicts the ETHERNET bit
  // XNetGetTitleXnAddr reports and stops link-checking titles before they
  // start. The virtual adapter is always up.
  return XEthernetStatus::XNET_ETHERNET_LINK_ACTIVE | XEthernetStatus::XNET_ETHERNET_LINK_100MBPS |
         XEthernetStatus::XNET_ETHERNET_LINK_FULL_DUPLEX;
}

u32 NetDll_XNetDnsLookup_entry(u32 caller, mapped_string host, u32 event_handle, mapped_u32 pdns) {
  // TODO(gibbed): actually implement this
  if (pdns) {
    auto dns_guest = REX_KERNEL_MEMORY()->SystemHeapAlloc(sizeof(XNDNS));
    auto dns = REX_KERNEL_MEMORY()->TranslateVirtual<XNDNS*>(dns_guest);
    dns->status = 1;  // non-zero = error
    *pdns = dns_guest;
  }
  if (event_handle) {
    auto ev = REX_KERNEL_OBJECTS()->LookupObject<XEvent>(event_handle);
    assert_not_null(ev);
    ev->Set(0, false);
  }
  return 0;
}

u32 NetDll_XNetDnsRelease_entry(u32 caller, ppc_ptr_t<XNDNS> dns) {
  if (!dns) {
    return X_STATUS_INVALID_PARAMETER;
  }
  REX_KERNEL_MEMORY()->SystemHeapFree(dns.guest_address());
  return 0;
}

u32 NetDll_XNetQosServiceLookup_entry(u32 caller, u32 flags, u32 event_handle, mapped_u32 pqos) {
  // Set pqos as some games will try accessing it despite non-successful result
  if (pqos) {
    auto qos_guest = REX_KERNEL_MEMORY()->SystemHeapAlloc(sizeof(XNQOS));
    auto qos = REX_KERNEL_MEMORY()->TranslateVirtual<XNQOS*>(qos_guest);
    qos->count = qos->count_pending = 0;
    *pqos = qos_guest;
  }
  if (event_handle) {
    auto ev = REX_KERNEL_OBJECTS()->LookupObject<XEvent>(event_handle);
    assert_not_null(ev);
    ev->Set(0, false);
  }
  return 0;
}

u32 NetDll_XNetQosRelease_entry(u32 caller, ppc_ptr_t<XNQOS> qos) {
  if (!qos) {
    return X_STATUS_INVALID_PARAMETER;
  }
  REX_KERNEL_MEMORY()->SystemHeapFree(qos.guest_address());
  return 0;
}

u32 NetDll_XNetQosListen_entry(u32 caller, mapped_void id, mapped_void data, u32 data_size, u32 r7,
                               u32 flags) {
#if REXGLUE_ENABLE_REXNET
  if (net::RexNet::shared()) {
    // Success no-op: hosts call Listen on session create (design spec §11).
    return 0;
  }
#endif
  return X_ERROR_FUNCTION_FAILED;
}

// QoS results (design spec §11): every target reports as contacted so join
// screens proceed. Round trip is the punch engine's measured value where a
// probe has come back, and a plausible default until then — a title showing
// ping would otherwise rank every peer identically.
u32 NetDll_XNetQosLookup_entry(u32 caller, u32 cxna, mapped_void apxna, mapped_void apxnkid,
                               mapped_void apxnkey, u32 cina, mapped_void aina,
                               mapped_void adwServiceId, u32 probe_count, u32 bits_per_sec,
                               u32 flags, u32 event_handle, mapped_u32 pqos) {
#if REXGLUE_ENABLE_REXNET
  if (net::RexNet::shared()) {
    const uint32_t count = cxna + cina;
    if (pqos) {
      const uint32_t size = sizeof(XNQOS) + (count > 1 ? (count - 1) * sizeof(XNQOSINFO) : 0);
      auto qos_guest = REX_KERNEL_MEMORY()->SystemHeapAlloc(size);
      auto qos = REX_KERNEL_MEMORY()->TranslateVirtual<XNQOS*>(qos_guest);
      std::memset(qos, 0, size);
      qos->count = count;
      qos->count_pending = 0;
      // Targets arrive as XNADDRs first, then raw in_addrs; both carry the
      // virtual IP we key measurements by.
      auto* xnaddrs = static_cast<const XNADDR*>(static_cast<void*>(apxna));
      auto* inaddrs = static_cast<const rex::be<uint32_t>*>(static_cast<void*>(aina));
      for (uint32_t i = 0; i < count; i++) {
        uint32_t target_vip = 0;
        if (i < cxna) {
          if (xnaddrs) {
            target_vip = ntohl(xnaddrs[i].ina.s_addr);
          }
        } else if (inaddrs) {
          target_vip = ntohl(inaddrs[i - cxna]);
        }
        const uint32_t measured = target_vip ? net::RexNet::shared()->PeerRttMs(target_vip) : 0;

        auto& info = qos->info[i];
        info.flags = 0x01 | 0x02;  // XNET_XNQOSINFO_COMPLETE | _TARGET_CONTACTED
        info.probes_xmit = static_cast<uint16_t>(probe_count ? probe_count : 8);
        info.probes_recv = info.probes_xmit;
        // Until a probe has come back there is nothing honest to report, so
        // fall back to a plausible figure rather than claim a 0 ms link.
        info.rtt_min_in_msecs = static_cast<uint32_t>(measured ? measured : 30);
        info.rtt_med_in_msecs = static_cast<uint32_t>(measured ? measured : 50);
        info.up_bits_per_sec = bits_per_sec ? bits_per_sec : 1024 * 1024;
        info.down_bits_per_sec = bits_per_sec ? bits_per_sec : 1024 * 1024;
      }
      *pqos = qos_guest;
    }
    if (event_handle) {
      auto ev = REX_KERNEL_OBJECTS()->LookupObject<XEvent>(event_handle);
      if (ev) {
        ev->Set(0, false);
      }
    }
    return 0;
  }
#endif
  return X_ERROR_FUNCTION_FAILED;
}

u32 NetDll_inet_addr_entry(mapped_string addr_ptr) {
  if (!addr_ptr) {
    return -1;
  }

  uint32_t addr = inet_addr(addr_ptr);
  // https://docs.microsoft.com/en-us/windows/win32/api/winsock2/nf-winsock2-inet_addr#return-value
  // Based on console research it seems like x360 uses old version of inet_addr
  // In case of empty string it return 0 instead of -1
  if (addr == -1 && !addr_ptr.value().length()) {
    return 0;
  }

  return rex::byte_swap(addr);
}

u32 NetDll_socket_entry(u32 caller, u32 af, u32 type, u32 protocol) {
  XSocket* socket = new XSocket(REX_KERNEL_STATE());
  X_STATUS result =
      socket->Initialize(XSocket::AddressFamily((uint32_t)af), XSocket::Type((uint32_t)type),
                         XSocket::Protocol((uint32_t)protocol));

  if (XFAILED(result)) {
    socket->Release();

    uint32_t error = xboxkrnl::xeRtlNtStatusToDosError(result);
    XThread::SetLastError(error);
    return -1;
  }

  return socket->handle();
}

u32 NetDll_closesocket_entry(u32 caller, u32 socket_handle) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  // TODO: Absolutely delete this object. It is no longer valid after calling
  // closesocket.
  socket->Close();
  socket->ReleaseHandle();
  return 0;
}

i32 NetDll_shutdown_entry(u32 caller, u32 socket_handle, i32 how) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  auto ret = socket->Shutdown(how);
  if (ret == -1) {
#if REX_PLATFORM_WIN32
    uint32_t error_code = WSAGetLastError();
    XThread::SetLastError(error_code);
#else
    XThread::SetLastError(0x0);
#endif
  }
  return ret;
}

u32 NetDll_setsockopt_entry(u32 caller, u32 socket_handle, u32 level, u32 optname,
                            mapped_void optval_ptr, u32 optlen) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  X_STATUS status = socket->SetOption(level, optname, optval_ptr, optlen);
  return XSUCCEEDED(status) ? 0 : -1;
}

u32 NetDll_ioctlsocket_entry(u32 caller, u32 socket_handle, u32 cmd, mapped_void arg_ptr) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  X_STATUS status = socket->IOControl(cmd, arg_ptr);
  if (XFAILED(status)) {
    XThread::SetLastError(xboxkrnl::xeRtlNtStatusToDosError(status));
    return -1;
  }

  // TODO
  return 0;
}

u32 NetDll_bind_entry(u32 caller, u32 socket_handle, ppc_ptr_t<XSOCKADDR_IN> name, u32 namelen) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  N_XSOCKADDR_IN native_name(name);
  X_STATUS status = socket->Bind(&native_name, namelen);
  if (XFAILED(status)) {
    XThread::SetLastError(xboxkrnl::xeRtlNtStatusToDosError(status));
    return -1;
  }

  return 0;
}

u32 NetDll_connect_entry(u32 caller, u32 socket_handle, ppc_ptr_t<XSOCKADDR> name, u32 namelen) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  N_XSOCKADDR native_name(name);
  X_STATUS status = socket->Connect(&native_name, namelen);
  if (XFAILED(status)) {
    XThread::SetLastError(xboxkrnl::xeRtlNtStatusToDosError(status));
    return -1;
  }

  return 0;
}

u32 NetDll_listen_entry(u32 caller, u32 socket_handle, i32 backlog) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  X_STATUS status = socket->Listen(backlog);
  if (XFAILED(status)) {
    XThread::SetLastError(xboxkrnl::xeRtlNtStatusToDosError(status));
    return -1;
  }

  return 0;
}

u32 NetDll_accept_entry(u32 caller, u32 socket_handle, ppc_ptr_t<XSOCKADDR> addr_ptr,
                        mapped_u32 addrlen_ptr) {
  if (!addr_ptr) {
    // WSAEFAULT
    XThread::SetLastError(0x271E);
    return -1;
  }

  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  N_XSOCKADDR native_addr(addr_ptr);
  int native_len = *addrlen_ptr;
  auto new_socket = socket->Accept(&native_addr, &native_len);
  if (new_socket) {
    addr_ptr->address_family = native_addr.address_family;
    std::memcpy(addr_ptr->sa_data, native_addr.sa_data, *addrlen_ptr - 2);
    *addrlen_ptr = native_len;

    return new_socket->handle();
  } else {
    return -1;
  }
}

struct x_fd_set {
  rex::be<uint32_t> fd_count;
  rex::be<uint32_t> fd_array[64];
};

struct host_set {
  uint32_t count;
  object_ref<XSocket> sockets[64];

  void Load(const x_fd_set* guest_set) {
    assert_true(guest_set->fd_count < 64);
    this->count = guest_set->fd_count;
    for (uint32_t i = 0; i < this->count; ++i) {
      auto socket_handle = static_cast<X_HANDLE>(guest_set->fd_array[i]);
      if (socket_handle == -1) {
        this->count = i;
        break;
      }
      // Convert from Xenia -> native
      auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
      assert_not_null(socket);
      this->sockets[i] = socket;
    }
  }

  void Store(x_fd_set* guest_set) {
    guest_set->fd_count = 0;
    for (uint32_t i = 0; i < this->count; ++i) {
      auto socket = this->sockets[i];
      guest_set->fd_array[guest_set->fd_count++] = socket->handle();
    }
  }

  void Store(fd_set* native_set) {
    FD_ZERO(native_set);
    for (uint32_t i = 0; i < this->count; ++i) {
      FD_SET(this->sockets[i]->native_handle(), native_set);
    }
  }

  void UpdateFrom(fd_set* native_set) {
    uint32_t new_count = 0;
    for (uint32_t i = 0; i < this->count; ++i) {
      auto socket = this->sockets[i];
      if (FD_ISSET(socket->native_handle(), native_set)) {
        this->sockets[new_count++] = socket;
      }
    }
    this->count = new_count;
  }
};

i32 NetDll_select_entry(i32 caller, i32 nfds, ppc_ptr_t<x_fd_set> readfds,
                        ppc_ptr_t<x_fd_set> writefds, ppc_ptr_t<x_fd_set> exceptfds,
                        mapped_void timeout_ptr) {
  host_set host_readfds = {};
  fd_set native_readfds = {};
  bool has_queued = false;
  host_set original_readfds = {};
  if (readfds) {
    host_readfds.Load(readfds);
    host_readfds.Store(&native_readfds);
#if REXGLUE_ENABLE_REXNET
    // RexNet packets queued on a socket make it readable even though the
    // host socket is idle; don't block in host select if any are waiting.
    original_readfds = host_readfds;
    for (uint32_t i = 0; i < host_readfds.count; ++i) {
      // A RexNet stream socket is readable when it has bytes, a pending
      // accept, has been closed, or its connect failed -- none of which the
      // host socket knows.
      if (host_readfds.sockets[i]->stream_readable() ||
          host_readfds.sockets[i]->stream_connect_failed() ||
          host_readfds.sockets[i]->HasQueuedPackets()) {
        has_queued = true;
        break;
      }
    }
#endif
  }
  host_set host_writefds = {};
  fd_set native_writefds = {};
  if (writefds) {
    host_writefds.Load(writefds);
    host_writefds.Store(&native_writefds);
  }
  host_set host_exceptfds = {};
  fd_set native_exceptfds = {};
  if (exceptfds) {
    host_exceptfds.Load(exceptfds);
    host_exceptfds.Store(&native_exceptfds);
  }
  timeval* timeout_in = nullptr;
  timeval timeout;
  if (timeout_ptr) {
    timeout = {static_cast<int32_t>(timeout_ptr.as_array<int32_t>()[0]),
               static_cast<int32_t>(timeout_ptr.as_array<int32_t>()[1])};
    chrono::Clock::ScaleGuestDurationTimeval(reinterpret_cast<int32_t*>(&timeout.tv_sec),
                                             reinterpret_cast<int32_t*>(&timeout.tv_usec));
    timeout_in = &timeout;
  }
  if (has_queued) {
    // Poll only; queued RexNet packets already satisfy the wait.
    timeout = {0, 0};
    timeout_in = &timeout;
  }
  int ret = select(nfds, readfds ? &native_readfds : nullptr, writefds ? &native_writefds : nullptr,
                   exceptfds ? &native_exceptfds : nullptr, timeout_in);
  if (readfds) {
    host_readfds.UpdateFrom(&native_readfds);
#if REXGLUE_ENABLE_REXNET
    if (has_queued) {
      // Union in sockets readable only because of queued RexNet packets.
      for (uint32_t i = 0; i < original_readfds.count; ++i) {
        auto& socket = original_readfds.sockets[i];
        if (!socket->HasQueuedPackets()) {
          continue;
        }
        bool present = false;
        for (uint32_t j = 0; j < host_readfds.count; ++j) {
          if (host_readfds.sockets[j].get() == socket.get()) {
            present = true;
            break;
          }
        }
        if (!present && host_readfds.count < 64) {
          host_readfds.sockets[host_readfds.count++] = socket;
        }
      }
    }
#endif
    host_readfds.Store(readfds);
  }
  if (writefds) {
    host_writefds.UpdateFrom(&native_writefds);
    host_writefds.Store(writefds);
  }
  if (exceptfds) {
    host_exceptfds.UpdateFrom(&native_exceptfds);
    host_exceptfds.Store(exceptfds);
  }

#if REXGLUE_ENABLE_REXNET
  if (has_queued) {
    // All three sets are final now; report the true ready count.
    ret = static_cast<int>(host_readfds.count + host_writefds.count + host_exceptfds.count);
  }
#endif

  // TODO(gibbed): modify ret to be what's actually copied to the guest fd_sets?
  return ret;
}

u32 NetDll_recv_entry(u32 caller, u32 socket_handle, mapped_void buf_ptr, u32 buf_len, u32 flags) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  int ret = socket->Recv(buf_ptr, buf_len, flags);
  if (ret < 0) {
#if REX_PLATFORM_WIN32
    XThread::SetLastError(WSAGetLastError());
#else
    XThread::SetLastError(HostSocketErrorToWSA(errno));
#endif
  }
  return ret;
}

u32 NetDll_recvfrom_entry(u32 caller, u32 socket_handle, mapped_void buf_ptr, u32 buf_len,
                          u32 flags, ppc_ptr_t<XSOCKADDR_IN> from_ptr, mapped_u32 fromlen_ptr) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  N_XSOCKADDR_IN native_from;
  if (from_ptr) {
    native_from = *from_ptr;
  }
  uint32_t native_fromlen = fromlen_ptr ? fromlen_ptr.value() : 0;
  int ret =
      socket->RecvFrom(buf_ptr, buf_len, flags, &native_from, fromlen_ptr ? &native_fromlen : 0);

  if (from_ptr) {
    from_ptr->sin_family = native_from.sin_family;
    from_ptr->sin_port = native_from.sin_port;
    from_ptr->sin_addr = native_from.sin_addr;
    std::memset(from_ptr->x_sin_zero, 0, sizeof(from_ptr->x_sin_zero));
  }
  if (fromlen_ptr) {
    *fromlen_ptr = native_fromlen;
  }

  if (ret == -1) {
#if REX_PLATFORM_WIN32
    uint32_t error_code = WSAGetLastError();
    XThread::SetLastError(error_code);
#else
    XThread::SetLastError(HostSocketErrorToWSA(errno));
#endif
  }

  return ret;
}

u32 NetDll_send_entry(u32 caller, u32 socket_handle, mapped_void buf_ptr, u32 buf_len, u32 flags) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  int ret = socket->Send(buf_ptr, buf_len, flags);
  if (ret < 0) {
#if REX_PLATFORM_WIN32
    XThread::SetLastError(WSAGetLastError());
#else
    XThread::SetLastError(HostSocketErrorToWSA(errno));
#endif
  }
  return ret;
}

u32 NetDll_sendto_entry(u32 caller, u32 socket_handle, mapped_void buf_ptr, u32 buf_len, u32 flags,
                        ppc_ptr_t<XSOCKADDR_IN> to_ptr, u32 to_len) {
  auto socket = REX_KERNEL_OBJECTS()->LookupObject<XSocket>(socket_handle);
  if (!socket) {
    // WSAENOTSOCK
    XThread::SetLastError(0x2736);
    return -1;
  }

  N_XSOCKADDR_IN native_to(to_ptr);
  int ret = socket->SendTo(buf_ptr, buf_len, flags, &native_to, to_len);
  if (ret < 0) {
#if REX_PLATFORM_WIN32
    XThread::SetLastError(WSAGetLastError());
#else
    XThread::SetLastError(HostSocketErrorToWSA(errno));
#endif
  }
  return ret;
}

u32 NetDll___WSAFDIsSet_entry(u32 socket_handle, ppc_ptr_t<x_fd_set> fd_set) {
  const uint8_t max_fd_count = std::min((uint32_t)fd_set->fd_count, uint32_t(64));
  for (uint8_t i = 0; i < max_fd_count; i++) {
    if (fd_set->fd_array[i] == socket_handle) {
      return 1;
    }
  }
  return 0;
}

void NetDll_WSASetLastError_entry(u32 error_code) {
  XThread::SetLastError(error_code);
}

}  // namespace xam
}  // namespace kernel
}  // namespace rex

REX_EXPORT(__imp__NetDll_XNetStartup, rex::kernel::xam::NetDll_XNetStartup_entry)
REX_EXPORT(__imp__NetDll_XNetCleanup, rex::kernel::xam::NetDll_XNetCleanup_entry)
REX_EXPORT(__imp__NetDll_XNetGetOpt, rex::kernel::xam::NetDll_XNetGetOpt_entry)
REX_EXPORT(__imp__NetDll_XNetRandom, rex::kernel::xam::NetDll_XNetRandom_entry)
REX_EXPORT(__imp__NetDll_WSAStartup, rex::kernel::xam::NetDll_WSAStartup_entry)
REX_EXPORT(__imp__NetDll_WSACleanup, rex::kernel::xam::NetDll_WSACleanup_entry)
REX_EXPORT(__imp__NetDll_WSAGetLastError, rex::kernel::xam::NetDll_WSAGetLastError_entry)
REX_EXPORT(__imp__NetDll_WSARecvFrom, rex::kernel::xam::NetDll_WSARecvFrom_entry)
REX_EXPORT(__imp__NetDll_WSAGetOverlappedResult,
           rex::kernel::xam::NetDll_WSAGetOverlappedResult_entry)
REX_EXPORT(__imp__NetDll_WSASendTo, rex::kernel::xam::NetDll_WSASendTo_entry)
REX_EXPORT(__imp__NetDll_WSAWaitForMultipleEvents,
           rex::kernel::xam::NetDll_WSAWaitForMultipleEvents_entry)
REX_EXPORT(__imp__NetDll_WSACreateEvent, rex::kernel::xam::NetDll_WSACreateEvent_entry)
REX_EXPORT(__imp__NetDll_WSACloseEvent, rex::kernel::xam::NetDll_WSACloseEvent_entry)
REX_EXPORT(__imp__NetDll_WSAResetEvent, rex::kernel::xam::NetDll_WSAResetEvent_entry)
REX_EXPORT(__imp__NetDll_WSASetEvent, rex::kernel::xam::NetDll_WSASetEvent_entry)
REX_EXPORT(__imp__NetDll_XNetGetTitleXnAddr, rex::kernel::xam::NetDll_XNetGetTitleXnAddr_entry)
REX_EXPORT(__imp__NetDll_XNetGetDebugXnAddr, rex::kernel::xam::NetDll_XNetGetDebugXnAddr_entry)
REX_EXPORT(__imp__NetDll_XNetXnAddrToMachineId,
           rex::kernel::xam::NetDll_XNetXnAddrToMachineId_entry)
REX_EXPORT(__imp__NetDll_XNetInAddrToString, rex::kernel::xam::NetDll_XNetInAddrToString_entry)
REX_EXPORT(__imp__NetDll_XNetXnAddrToInAddr, rex::kernel::xam::NetDll_XNetXnAddrToInAddr_entry)
REX_EXPORT(__imp__NetDll_XNetInAddrToXnAddr, rex::kernel::xam::NetDll_XNetInAddrToXnAddr_entry)
REX_EXPORT(__imp__NetDll_XNetConnect, rex::kernel::xam::NetDll_XNetConnect_entry)
REX_EXPORT(__imp__NetDll_XNetGetConnectStatus, rex::kernel::xam::NetDll_XNetGetConnectStatus_entry)
REX_EXPORT(__imp__NetDll_XNetCreateKey, rex::kernel::xam::NetDll_XNetCreateKey_entry)
REX_EXPORT(__imp__NetDll_XNetRegisterKey, rex::kernel::xam::NetDll_XNetRegisterKey_entry)
REX_EXPORT(__imp__NetDll_XNetUnregisterKey, rex::kernel::xam::NetDll_XNetUnregisterKey_entry)
REX_EXPORT(__imp__NetDll_XNetQosLookup, rex::kernel::xam::NetDll_XNetQosLookup_entry)
REX_EXPORT(__imp__NetDll_XNetSetSystemLinkPort,
           rex::kernel::xam::NetDll_XNetSetSystemLinkPort_entry)
REX_EXPORT(__imp__NetDll_XNetGetEthernetLinkStatus,
           rex::kernel::xam::NetDll_XNetGetEthernetLinkStatus_entry)
REX_EXPORT(__imp__NetDll_XNetDnsLookup, rex::kernel::xam::NetDll_XNetDnsLookup_entry)
REX_EXPORT(__imp__NetDll_XNetDnsRelease, rex::kernel::xam::NetDll_XNetDnsRelease_entry)
REX_EXPORT(__imp__NetDll_XNetQosServiceLookup, rex::kernel::xam::NetDll_XNetQosServiceLookup_entry)
REX_EXPORT(__imp__NetDll_XNetQosRelease, rex::kernel::xam::NetDll_XNetQosRelease_entry)
REX_EXPORT(__imp__NetDll_XNetQosListen, rex::kernel::xam::NetDll_XNetQosListen_entry)
REX_EXPORT(__imp__NetDll_inet_addr, rex::kernel::xam::NetDll_inet_addr_entry)
REX_EXPORT(__imp__NetDll_socket, rex::kernel::xam::NetDll_socket_entry)
REX_EXPORT(__imp__NetDll_closesocket, rex::kernel::xam::NetDll_closesocket_entry)
REX_EXPORT(__imp__NetDll_shutdown, rex::kernel::xam::NetDll_shutdown_entry)
REX_EXPORT(__imp__NetDll_setsockopt, rex::kernel::xam::NetDll_setsockopt_entry)
REX_EXPORT(__imp__NetDll_ioctlsocket, rex::kernel::xam::NetDll_ioctlsocket_entry)
REX_EXPORT(__imp__NetDll_bind, rex::kernel::xam::NetDll_bind_entry)
REX_EXPORT(__imp__NetDll_connect, rex::kernel::xam::NetDll_connect_entry)
REX_EXPORT(__imp__NetDll_listen, rex::kernel::xam::NetDll_listen_entry)
REX_EXPORT(__imp__NetDll_accept, rex::kernel::xam::NetDll_accept_entry)
REX_EXPORT(__imp__NetDll_select, rex::kernel::xam::NetDll_select_entry)
REX_EXPORT(__imp__NetDll_recv, rex::kernel::xam::NetDll_recv_entry)
REX_EXPORT(__imp__NetDll_recvfrom, rex::kernel::xam::NetDll_recvfrom_entry)
REX_EXPORT(__imp__NetDll_send, rex::kernel::xam::NetDll_send_entry)
REX_EXPORT(__imp__NetDll_sendto, rex::kernel::xam::NetDll_sendto_entry)
REX_EXPORT(__imp__NetDll___WSAFDIsSet, rex::kernel::xam::NetDll___WSAFDIsSet_entry)
REX_EXPORT(__imp__NetDll_WSASetLastError, rex::kernel::xam::NetDll_WSASetLastError_entry)

REX_EXPORT_STUB(__imp__NetDll_UpnpActionCalculateWorkBufferSize);
REX_EXPORT_STUB(__imp__NetDll_UpnpActionCreate);
REX_EXPORT_STUB(__imp__NetDll_UpnpActionGetResults);
REX_EXPORT_STUB(__imp__NetDll_UpnpCleanup);
REX_EXPORT_STUB(__imp__NetDll_UpnpCloseHandle);
REX_EXPORT_STUB(__imp__NetDll_UpnpDescribeCreate);
REX_EXPORT_STUB(__imp__NetDll_UpnpDescribeGetResults);
REX_EXPORT_STUB(__imp__NetDll_UpnpDoWork);
REX_EXPORT_STUB(__imp__NetDll_UpnpEventCreate);
REX_EXPORT_STUB(__imp__NetDll_UpnpEventGetCurrentState);
REX_EXPORT_STUB(__imp__NetDll_UpnpEventUnsubscribe);
REX_EXPORT_STUB(__imp__NetDll_UpnpSearchCreate);
REX_EXPORT_STUB(__imp__NetDll_UpnpSearchGetDevices);
REX_EXPORT_STUB(__imp__NetDll_UpnpStartup);
REX_EXPORT_STUB(__imp__NetDll_WSACancelOverlappedIO);
REX_EXPORT(__imp__NetDll_WSAEventSelect, rex::kernel::xam::NetDll_WSAEventSelect_entry)

REX_EXPORT_STUB(__imp__NetDll_WSARecv);
REX_EXPORT_STUB(__imp__NetDll_WSASend);
REX_EXPORT_STUB(__imp__NetDll_WSAStartupEx);
REX_EXPORT_STUB(__imp__NetDll_XHttpCloseHandle);
REX_EXPORT_STUB(__imp__NetDll_XHttpConnect);
REX_EXPORT_STUB(__imp__NetDll_XHttpCrackUrl);
REX_EXPORT_STUB(__imp__NetDll_XHttpCrackUrlW);
REX_EXPORT_STUB(__imp__NetDll_XHttpCreateUrl);
REX_EXPORT_STUB(__imp__NetDll_XHttpCreateUrlW);
REX_EXPORT_STUB(__imp__NetDll_XHttpDoWork);
REX_EXPORT_STUB(__imp__NetDll_XHttpGetPerfCounters);
REX_EXPORT_STUB(__imp__NetDll_XHttpOpen);
REX_EXPORT_STUB(__imp__NetDll_XHttpOpenRequest);
REX_EXPORT_STUB(__imp__NetDll_XHttpOpenRequestUsingMemory);
REX_EXPORT_STUB(__imp__NetDll_XHttpQueryAuthSchemes);
REX_EXPORT_STUB(__imp__NetDll_XHttpQueryHeaders);
REX_EXPORT_STUB(__imp__NetDll_XHttpQueryOption);
REX_EXPORT_STUB(__imp__NetDll_XHttpReadData);
REX_EXPORT_STUB(__imp__NetDll_XHttpReceiveResponse);
REX_EXPORT_STUB(__imp__NetDll_XHttpResetPerfCounters);
REX_EXPORT_STUB(__imp__NetDll_XHttpSendRequest);
REX_EXPORT_STUB(__imp__NetDll_XHttpSetCredentials);
REX_EXPORT_STUB(__imp__NetDll_XHttpSetOption);
REX_EXPORT_STUB(__imp__NetDll_XHttpSetStatusCallback);
REX_EXPORT_STUB(__imp__NetDll_XHttpShutdown);
REX_EXPORT_STUB(__imp__NetDll_XHttpStartup);
REX_EXPORT_STUB(__imp__NetDll_XHttpWriteData);
REX_EXPORT_STUB(__imp__NetDll_XNetDnsReverseLookup);
REX_EXPORT_STUB(__imp__NetDll_XNetDnsReverseRelease);
REX_EXPORT(__imp__NetDll_XNetGetBroadcastVersionStatus,
           rex::kernel::xam::NetDll_XNetGetBroadcastVersionStatus_entry)
REX_EXPORT(__imp__NetDll_XNetGetSystemLinkPort,
           rex::kernel::xam::NetDll_XNetGetSystemLinkPort_entry)
REX_EXPORT_STUB(__imp__NetDll_XNetGetXnAddrPlatform);
REX_EXPORT_STUB(__imp__NetDll_XNetInAddrToServer);
REX_EXPORT_STUB(__imp__NetDll_XNetQosGetListenStats);
REX_EXPORT_STUB(__imp__NetDll_XNetReplaceKey);
REX_EXPORT_STUB(__imp__NetDll_XNetServerToInAddr);
REX_EXPORT_STUB(__imp__NetDll_XNetSetOpt);
REX_EXPORT_STUB(__imp__NetDll_XNetStartupEx);
REX_EXPORT_STUB(__imp__NetDll_XNetTsAddrToInAddr);
REX_EXPORT(__imp__NetDll_XNetUnregisterInAddr, rex::kernel::xam::NetDll_XNetUnregisterInAddr_entry)
REX_EXPORT_STUB(__imp__NetDll_XmlDownloadContinue);
REX_EXPORT_STUB(__imp__NetDll_XmlDownloadGetParseTime);
REX_EXPORT_STUB(__imp__NetDll_XmlDownloadGetReceivedDataSize);
REX_EXPORT_STUB(__imp__NetDll_XmlDownloadStart);
REX_EXPORT_STUB(__imp__NetDll_XmlDownloadStop);
REX_EXPORT_STUB(__imp__NetDll_XnpCapture);
REX_EXPORT_STUB(__imp__NetDll_XnpConfig);
REX_EXPORT_STUB(__imp__NetDll_XnpConfigUPnP);
REX_EXPORT_STUB(__imp__NetDll_XnpConfigUPnPPortAndExternalAddr);
REX_EXPORT_STUB(__imp__NetDll_XnpEthernetInterceptRecv);
REX_EXPORT_STUB(__imp__NetDll_XnpEthernetInterceptSetCallbacks);
REX_EXPORT_STUB(__imp__NetDll_XnpEthernetInterceptSetExtendedReceiveCallback);
REX_EXPORT_STUB(__imp__NetDll_XnpEthernetInterceptXmit);
REX_EXPORT_STUB(__imp__NetDll_XnpEthernetInterceptXmitAsIp);
REX_EXPORT_STUB(__imp__NetDll_XnpGetActiveSocketList);
REX_EXPORT_STUB(__imp__NetDll_XnpGetConfigStatus);
REX_EXPORT_STUB(__imp__NetDll_XnpGetKeyList);
REX_EXPORT_STUB(__imp__NetDll_XnpGetQosLookupList);
REX_EXPORT_STUB(__imp__NetDll_XnpGetSecAssocList);
REX_EXPORT_STUB(__imp__NetDll_XnpGetVlanXboxName);
REX_EXPORT_STUB(__imp__NetDll_XnpLoadConfigParams);
REX_EXPORT_STUB(__imp__NetDll_XnpLoadMachineAccount);
REX_EXPORT_STUB(__imp__NetDll_XnpLogonClearChallenge);
REX_EXPORT_STUB(__imp__NetDll_XnpLogonClearQEvent);
REX_EXPORT_STUB(__imp__NetDll_XnpLogonGetChallenge);
REX_EXPORT_STUB(__imp__NetDll_XnpLogonGetQFlags);
REX_EXPORT_STUB(__imp__NetDll_XnpLogonGetQVals);
REX_EXPORT_STUB(__imp__NetDll_XnpLogonGetStatus);
REX_EXPORT_STUB(__imp__NetDll_XnpLogonSetChallengeResponse);
REX_EXPORT_STUB(__imp__NetDll_XnpLogonSetPState);
REX_EXPORT_STUB(__imp__NetDll_XnpLogonSetQEvent);
REX_EXPORT_STUB(__imp__NetDll_XnpLogonSetQFlags);
REX_EXPORT_STUB(__imp__NetDll_XnpLogonSetQVals);
REX_EXPORT_STUB(__imp__NetDll_XnpNoteSystemTime);
REX_EXPORT_STUB(__imp__NetDll_XnpPersistTitleState);
REX_EXPORT_STUB(__imp__NetDll_XnpQosHistoryGetAggregateMeasurement);
REX_EXPORT_STUB(__imp__NetDll_XnpQosHistoryGetEntries);
REX_EXPORT_STUB(__imp__NetDll_XnpQosHistoryLoad);
REX_EXPORT_STUB(__imp__NetDll_XnpQosHistorySaveMeasurements);
REX_EXPORT_STUB(__imp__NetDll_XnpRegisterKeyForCallerType);
REX_EXPORT_STUB(__imp__NetDll_XnpReplaceKeyForCallerType);
REX_EXPORT_STUB(__imp__NetDll_XnpSaveConfigParams);
REX_EXPORT_STUB(__imp__NetDll_XnpSaveMachineAccount);
REX_EXPORT_STUB(__imp__NetDll_XnpSetVlanXboxName);
REX_EXPORT_STUB(__imp__NetDll_XnpToolIpProxyInject);
REX_EXPORT_STUB(__imp__NetDll_XnpToolSetCallbacks);
REX_EXPORT_STUB(__imp__NetDll_XnpUnregisterKeyForCallerType);
REX_EXPORT_STUB(__imp__NetDll_XnpUpdateConfigParams);
REX_EXPORT_STUB(__imp__NetDll_getpeername);
REX_EXPORT(__imp__NetDll_getsockname, rex::kernel::xam::NetDll_getsockname_entry)
REX_EXPORT(__imp__NetDll_getsockopt, rex::kernel::xam::NetDll_getsockopt_entry)
