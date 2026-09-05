/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2013 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 *
 * @modified    Tom Clay, 2026 - Adapted for ReXGlue runtime
 *
 * @modified    Ryan Fisher, 2026 - RexNet netplay integration
 */

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <rex/kernel/xam/module.h>
#include <rex/system/xevent.h>
#include <rex/platform.h>
#include <rex/system/kernel_state.h>
#include <rex/system/xsocket.h>
#include <rex/system/xthread.h>
// #include <rex/system/xnet.h>

#include <rex/net/socket.h>

// Standard socket types used by Xbox API emulation
#if REX_PLATFORM_WIN32
#include <WinSock2.h>

#include <WS2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#endif

#if REXGLUE_ENABLE_REXNET
#include <rex/net/rexnet.h>
#endif

namespace rex::system {

// Registry of bound UDP sockets by guest port, so RexNet can route inbound
// virtual-IP datagrams to the right socket's packet queue.
static std::mutex bound_udp_mutex;
static std::unordered_map<uint16_t, XSocket*> bound_udp_sockets;

// Sockets with a pending overlapped receive are polled by one background
// thread (host-socket data has no completion callback; RexNet queue inserts
// complete inline via QueuePacket).
static std::mutex polled_mutex;
static std::vector<object_ref<XSocket>> polled_sockets;
static std::atomic<bool> poller_running{false};

static void AddPolledSocket(XSocket* socket) {
  {
    std::lock_guard<std::mutex> lock(polled_mutex);
    for (auto& existing : polled_sockets) {
      if (existing.get() == socket) {
        return;
      }
    }
    polled_sockets.push_back(retain_object(socket));
  }
  bool expected = false;
  if (poller_running.compare_exchange_strong(expected, true)) {
    std::thread([] {
      for (;;) {
        std::vector<object_ref<XSocket>> snapshot;
        {
          std::lock_guard<std::mutex> lock(polled_mutex);
          snapshot = polled_sockets;
        }
        for (auto& socket : snapshot) {
          socket->PumpPendingWsaRecv();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }).detach();
  }
}

static void RemovePolledSocket(XSocket* socket) {
  std::lock_guard<std::mutex> lock(polled_mutex);
  for (auto it = polled_sockets.begin(); it != polled_sockets.end(); ++it) {
    if (it->get() == socket) {
      polled_sockets.erase(it);
      return;
    }
  }
}

// Guest TCP (§18): listeners by bound guest port, and connections by stream
// id. Separate from bound_udp_sockets -- a title may well use the same port
// number for both a UDP and a TCP socket, as SoulCalibur IV does around 1001.
static std::mutex stream_registry_mutex;
static std::unordered_map<uint16_t, XSocket*> stream_listeners;
static std::unordered_map<uint64_t, XSocket*> stream_sockets;

// Sockets waiting on an outbound guest TCP connect, keyed by where they are
// connecting to. A FIFO per key: two sockets may legitimately dial the same
// address, and completions must be handed out in the order they were asked
// for rather than to whichever happens to be found first.
static std::mutex pending_connect_mutex;
static std::map<std::pair<uint32_t, uint16_t>, std::vector<XSocket*>> pending_connects;

void XSocket::BeginPendingConnect(uint32_t peer_vip, uint16_t dst_port) {
  rexnet_connect_failed_.store(false, std::memory_order_release);
  std::lock_guard<std::mutex> lock(pending_connect_mutex);
  pending_connects[{peer_vip, dst_port}].push_back(this);
}

object_ref<XSocket> XSocket::ClaimPendingConnect(uint32_t peer_vip, uint16_t dst_port) {
  std::lock_guard<std::mutex> lock(pending_connect_mutex);
  auto it = pending_connects.find({peer_vip, dst_port});
  if (it == pending_connects.end() || it->second.empty()) {
    return object_ref<XSocket>();
  }
  XSocket* socket = it->second.front();
  it->second.erase(it->second.begin());
  if (it->second.empty()) {
    pending_connects.erase(it);
  }
  return retain_object(socket);
}

void XSocket::OnStreamConnectFailed() {
  rexnet_connect_failed_.store(true, std::memory_order_release);
  // Winsock reports a failed non-blocking connect through the exception set,
  // but titles overwhelmingly poll readability; make it observable either way.
  SignalWsaEvent();
}

bool XSocket::stream_writable() {
  // A connected stream is always writable: the carrier buffers below us, so
  // there is no send-window to run out of from the guest's point of view.
  return rexnet_stream_id_ != 0;
}

void XSocket::SetEventSelect(uint32_t event_handle, uint32_t mask) {
  wsa_event_handle_.store(event_handle, std::memory_order_release);
  wsa_event_mask_.store(mask, std::memory_order_release);
  // Winsock levels the event on association: if the socket is already
  // readable the guest must not miss the edge it never saw.
  if (event_handle && (HasQueuedPackets() || stream_readable())) {
    SignalWsaEvent();
  }
}

void XSocket::SignalWsaEvent() {
  const uint32_t handle = wsa_event_handle_.load(std::memory_order_acquire);
  if (!handle) {
    return;
  }
  if (auto ev = kernel_state_->object_table()->LookupObject<XEvent>(handle)) {
    ev->Set(0, false);
  }
}

object_ref<XSocket> XSocket::FindStreamListener(uint16_t port) {
  std::lock_guard<std::mutex> lock(stream_registry_mutex);
  auto it = stream_listeners.find(port);
  return it == stream_listeners.end() ? object_ref<XSocket>() : retain_object(it->second);
}

object_ref<XSocket> XSocket::FindStream(uint64_t stream_id) {
  std::lock_guard<std::mutex> lock(stream_registry_mutex);
  auto it = stream_sockets.find(stream_id);
  return it == stream_sockets.end() ? object_ref<XSocket>() : retain_object(it->second);
}

void XSocket::BecomeStreamListener() {
  rexnet_listening_ = true;
  std::lock_guard<std::mutex> lock(stream_registry_mutex);
  stream_listeners[bound_port_] = this;
}

void XSocket::AdoptStream(uint64_t stream_id, uint32_t peer_vip, uint16_t remote_port) {
  rexnet_stream_id_ = stream_id;
  rexnet_peer_vip_ = peer_vip;
  rexnet_remote_port_ = remote_port;
  {
    std::lock_guard<std::mutex> lock(stream_registry_mutex);
    stream_sockets[stream_id] = this;
  }
  // A completed connect is an event a waiting title needs to see.
  SignalWsaEvent();
}

void XSocket::QueueAcceptedStream(uint64_t stream_id, uint32_t peer_vip, uint16_t remote_port) {
  {
    std::lock_guard<std::mutex> lock(stream_mutex_);
    pending_accepts_.push_back({stream_id, peer_vip, remote_port});
  }
  // Wake anyone blocked in select() or waiting on a WSAEVENT.
  SignalWsaEvent();
}

void XSocket::QueueStreamData(const uint8_t* data, uint32_t len) {
  {
    std::lock_guard<std::mutex> lock(stream_mutex_);
    stream_rx_.insert(stream_rx_.end(), data, data + len);
  }
  SignalWsaEvent();
}

void XSocket::OnStreamClosed() {
  {
    std::lock_guard<std::mutex> lock(stream_mutex_);
    // Buffered bytes still belong to the guest: report EOF only once it has
    // read them, exactly as a real TCP close does.
    rexnet_stream_eof_ = true;
  }
  SignalWsaEvent();
}

bool XSocket::stream_readable() {
  std::lock_guard<std::mutex> lock(stream_mutex_);
  // A closed connection is "readable" so the guest can observe the EOF.
  return !stream_rx_.empty() || !pending_accepts_.empty() || rexnet_stream_eof_;
}

// Guest XWSAOVERLAPPED offsets (see xam_net.cpp): internal +0 (status),
// internal_high +4 (bytes), event handle +16.
constexpr uint32_t kWsaOverlappedInternal = 0;
constexpr uint32_t kWsaOverlappedInternalHigh = 4;
constexpr uint32_t kWsaOverlappedEvent = 16;
// Status while an overlapped operation is outstanding (STATUS_PENDING).
constexpr uint32_t kWsaStatusPending = 0x103;

XSocket::XSocket(KernelState* kernel_state) : XObject(kernel_state, kObjectType) {}

XSocket::XSocket(KernelState* kernel_state, uint64_t native_handle)
    : XObject(kernel_state, kObjectType), native_handle_(native_handle) {}

XSocket::~XSocket() {
  Close();
}

X_STATUS XSocket::Initialize(AddressFamily af, Type type, Protocol proto) {
  af_ = af;
  type_ = type;
  proto_ = proto;

  if (proto == Protocol::X_IPPROTO_VDP) {
    // VDP is a layer on top of UDP.
    proto = Protocol::X_IPPROTO_UDP;
  }

  native_handle_ = socket(af, type, proto);
  if (native_handle_ == -1) {
    return X_STATUS_UNSUCCESSFUL;
  }

  return X_STATUS_SUCCESS;
}

X_STATUS XSocket::Close() {
  if (bound_ && type_ == Type::X_SOCK_DGRAM) {
    std::lock_guard<std::mutex> lock(bound_udp_mutex);
    if (auto it = bound_udp_sockets.find(bound_port_);
        it != bound_udp_sockets.end() && it->second == this) {
      bound_udp_sockets.erase(it);
    }
  }
  bound_ = false;

  // Abort any outstanding overlapped receive so waiters wake up.
  {
    std::lock_guard<std::mutex> lock(pending_recv_mutex_);
    if (pending_recv_active_) {
      CompleteWsaOverlapped(pending_recv_overlapped_, 995 /* WSA_OPERATION_ABORTED */, 0);
      pending_recv_active_ = false;
      pending_recv_buffers_.clear();
      pending_recv_completion_routine_ = 0;
      pending_recv_apc_thread_ = 0;
    }
  }
  RemovePolledSocket(this);

  // Guest TCP: drop registry entries and tell the peer we are gone.
  {
    std::lock_guard<std::mutex> lock(stream_registry_mutex);
    if (rexnet_listening_) {
      auto it = stream_listeners.find(bound_port_);
      if (it != stream_listeners.end() && it->second == this) {
        stream_listeners.erase(it);
      }
      rexnet_listening_ = false;
    }
    if (rexnet_stream_id_) {
      stream_sockets.erase(rexnet_stream_id_);
    }
  }
#if REXGLUE_ENABLE_REXNET
  if (rexnet_stream_id_) {
    if (auto* rexnet = rex::net::RexNet::shared()) {
      rexnet->StreamClose(rexnet_stream_id_);
    }
    rexnet_stream_id_ = 0;
  }
#endif

  int ret = rex::net::socket_close(native_handle_);
  if (ret != 0) {
    return X_STATUS_UNSUCCESSFUL;
  }

  return X_STATUS_SUCCESS;
}

X_STATUS XSocket::SetOption(uint32_t level, uint32_t optname, void* optval_ptr, uint32_t optlen) {
  if (level == 0xFFFF && (optname == 0x5801 || optname == 0x5802)) {
    // Disable socket encryption
    secure_ = false;
    return X_STATUS_SUCCESS;
  }

  int ret = setsockopt(native_handle_, level, optname, (char*)optval_ptr, optlen);
  if (ret < 0) {
    // TODO: WSAGetLastError()
    return X_STATUS_UNSUCCESSFUL;
  }

  // SO_BROADCAST
  if (level == 0xFFFF && optname == 0x0020) {
    broadcast_socket_ = true;
  }

  return X_STATUS_SUCCESS;
}

X_STATUS XSocket::IOControl(uint32_t cmd, uint8_t* arg_ptr) {
  // The title issues Winsock ioctl command codes with big-endian arguments
  // (XRNM sets FIONBIO right after bind; the download manager polls FIONREAD).
  // Winsock code values only coincide with the host's on Win32, so the codes
  // the guest ABI defines must be translated here rather than handed to the
  // host ioctl.
  constexpr uint32_t kX_FIONBIO = 0x8004667E;
  constexpr uint32_t kX_FIONREAD = 0x4004667F;
  switch (cmd) {
    case kX_FIONBIO: {
      const uint32_t enable = arg_ptr ? memory::load_and_swap<uint32_t>(arg_ptr) : 1;
#if REX_PLATFORM_WIN32
      u_long value = enable ? 1 : 0;
      if (ioctlsocket(native_handle_, FIONBIO, &value) != 0) {
        return X_STATUS_UNSUCCESSFUL;
      }
#else
      const int flags = fcntl(static_cast<int>(native_handle_), F_GETFL, 0);
      if (flags < 0) {
        return X_STATUS_UNSUCCESSFUL;
      }
      const int updated = enable ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
      if (fcntl(static_cast<int>(native_handle_), F_SETFL, updated) != 0) {
        return X_STATUS_UNSUCCESSFUL;
      }
#endif
      return X_STATUS_SUCCESS;
    }
    case kX_FIONREAD: {
      uint32_t available = 0;
      {
        std::lock_guard<std::mutex> lock(incoming_packet_mutex_);
        if (!incoming_packets_.empty()) {
          available = ((packet*)incoming_packets_.front())->data_len;
        }
      }
      if (available == 0) {
#if REX_PLATFORM_WIN32
        u_long value = 0;
        if (ioctlsocket(native_handle_, FIONREAD, &value) != 0) {
          return X_STATUS_UNSUCCESSFUL;
        }
        available = static_cast<uint32_t>(value);
#else
        int value = 0;
        if (ioctl(static_cast<int>(native_handle_), FIONREAD, &value) != 0) {
          return X_STATUS_UNSUCCESSFUL;
        }
        available = static_cast<uint32_t>(value);
#endif
      }
      if (arg_ptr) {
        memory::store_and_swap<uint32_t>(arg_ptr, available);
      }
      return X_STATUS_SUCCESS;
    }
    default:
      break;
  }

  int ret = rex::net::socket_ioctl(native_handle_, cmd, arg_ptr);
  if (ret < 0) {
    // TODO: Get last error
    return X_STATUS_UNSUCCESSFUL;
  }

  return X_STATUS_SUCCESS;
}

X_STATUS XSocket::Connect(N_XSOCKADDR* name, int name_len) {
#if REXGLUE_ENABLE_REXNET
  // A stream socket aimed at a virtual IP is carried by RexNet: the host
  // stack has no route to 10.77.0.0/16, so handing it over would just fail.
  if (type_ == Type::X_SOCK_STREAM && name && name_len >= 8) {
    const auto* addr = reinterpret_cast<const N_XSOCKADDR_IN*>(name);
    const uint32_t dst_ip = addr->sin_addr;
    if ((dst_ip & 0xFFFF0000u) == rex::net::VirtualIpTable::kNetworkBase) {
      auto* rexnet = rex::net::RexNet::shared();
      if (!rexnet) {
        return X_STATUS_UNSUCCESSFUL;
      }
      const uint16_t dst_port = addr->sin_port;
      // A title polling a non-blocking connect calls connect() again; Winsock
      // answers WSAEISCONN once it is up, which is how it learns to stop.
      if (rexnet_stream_id_) {
        XThread::SetLastError(0x2748);  // WSAEISCONN
        return X_STATUS_SUCCESS;
      }
      if (rexnet_connect_failed_.load(std::memory_order_acquire)) {
        XThread::SetLastError(0x274D);  // WSAECONNREFUSED
        return X_STATUS_UNSUCCESSFUL;
      }
      BeginPendingConnect(dst_ip, dst_port);
      rexnet->StreamConnect(dst_ip, bound_port_, dst_port);
      // Connect is asynchronous here. A non-blocking guest socket -- which is
      // what System Link titles use -- expects exactly this and polls with
      // select(), so report "in progress" rather than pretending it is done.
      XThread::SetLastError(0x2733);  // WSAEWOULDBLOCK
      return X_STATUS_PENDING;
    }
  }
#endif
  int ret = connect(native_handle_, (sockaddr*)name, name_len);
  if (ret < 0) {
    return X_STATUS_UNSUCCESSFUL;
  }

  return X_STATUS_SUCCESS;
}

X_STATUS XSocket::Bind(N_XSOCKADDR_IN* name, int name_len) {
#if REXGLUE_ENABLE_REXNET
  // With RexNet active, guest UDP traffic is fully virtualized: outbound to
  // 10.77.0.0/16 routes through the punched socket (SendTo), inbound arrives
  // via QueuePacket, and both are keyed on the guest port. The native socket
  // carries none of that traffic, so the host port it binds is irrelevant —
  // and binding the guest's real port is actively harmful: titles use the
  // Xbox's low well-known ports (Fable 2: XRNM 1000, msgJoinSession 1001,
  // presence 1002/1003), all < 1024, which Linux refuses to bind from a
  // non-root process (EACCES). Bind the native socket to an ephemeral
  // unprivileged port instead; keep the guest port for RexNet routing. This
  // also removes any same-host two-instance collision.
  if (rex::net::RexNet::shared() && type_ == Type::X_SOCK_DGRAM) {
    N_XSOCKADDR_IN ephemeral = *name;
    ephemeral.sin_port = 0;  // OS picks an unprivileged ephemeral port
    ephemeral.sin_addr = 0;  // INADDR_ANY
    if (bind(native_handle_, (sockaddr*)&ephemeral, name_len) < 0) {
      REXSYS_WARN(
          "XSocket::Bind: ephemeral native bind failed (errno {}); "
          "guest port {} still routed via RexNet",
          errno, static_cast<uint16_t>(name->sin_port));
    }
    bound_ = true;
    bound_port_ = name->sin_port;  // guest (virtual) port for RexNet routing
    {
      std::lock_guard<std::mutex> lock(bound_udp_mutex);
      bound_udp_sockets[bound_port_] = this;
    }
    return X_STATUS_SUCCESS;
  }
#endif

  int ret = bind(native_handle_, (sockaddr*)name, name_len);
  if (ret < 0) {
    return X_STATUS_UNSUCCESSFUL;
  }

  bound_ = true;
  bound_port_ = name->sin_port;

  if (type_ == Type::X_SOCK_DGRAM) {
    std::lock_guard<std::mutex> lock(bound_udp_mutex);
    bound_udp_sockets[bound_port_] = this;
  }

  return X_STATUS_SUCCESS;
}

object_ref<XSocket> XSocket::FindBoundUdp(uint16_t port) {
  std::lock_guard<std::mutex> lock(bound_udp_mutex);
  if (auto it = bound_udp_sockets.find(port); it != bound_udp_sockets.end()) {
    return retain_object(it->second);
  }
  return nullptr;
}

bool XSocket::HasQueuedPackets() {
  std::lock_guard<std::mutex> lock(incoming_packet_mutex_);
  return !incoming_packets_.empty();
}

X_STATUS XSocket::Listen(int backlog) {
#if REXGLUE_ENABLE_REXNET
  // Listen on both planes: a title cannot know whether the peer that answers
  // will arrive over RexNet or the real network, and registering here costs
  // nothing when nobody does.
  if (type_ == Type::X_SOCK_STREAM && rex::net::RexNet::shared()) {
    BecomeStreamListener();
  }
#endif
  int ret = listen(native_handle_, backlog);
  if (ret < 0) {
    return X_STATUS_UNSUCCESSFUL;
  }

  return X_STATUS_SUCCESS;
}

object_ref<XSocket> XSocket::Accept(N_XSOCKADDR* name, int* name_len) {
#if REXGLUE_ENABLE_REXNET
  // A RexNet connection waiting to be accepted takes priority: it arrived
  // through our own path and the host socket knows nothing about it.
  if (rexnet_listening_) {
    PendingAccept pending{};
    bool have = false;
    {
      std::lock_guard<std::mutex> lock(stream_mutex_);
      if (!pending_accepts_.empty()) {
        pending = pending_accepts_.front();
        pending_accepts_.erase(pending_accepts_.begin());
        have = true;
      }
    }
    if (have) {
      // The accepted socket has no host descriptor -- it exists only as a
      // RexNet stream -- so it is created without one.
      auto socket = object_ref<XSocket>(new XSocket(kernel_state_, uint64_t(-1)));
      socket->af_ = af_;
      socket->type_ = type_;
      socket->proto_ = proto_;
      socket->bound_port_ = bound_port_;
      socket->AdoptStream(pending.stream_id, pending.peer_vip, pending.remote_port);
      if (name && name_len && *name_len >= int(sizeof(N_XSOCKADDR_IN))) {
        auto* out = reinterpret_cast<N_XSOCKADDR_IN*>(name);
        out->sin_family = 2;  // AF_INET
        out->sin_port = pending.remote_port;
        out->sin_addr = pending.peer_vip;
        std::memset(out->x_sin_zero, 0, sizeof(out->x_sin_zero));
        *name_len = sizeof(N_XSOCKADDR_IN);
      }
      return socket;
    }
  }
#endif
  sockaddr n_sockaddr;
  socklen_t n_name_len = sizeof(sockaddr);
  uintptr_t ret = accept(native_handle_, &n_sockaddr, &n_name_len);
  if (ret == -1) {
    std::memset(name, 0, *name_len);
    *name_len = 0;
    return nullptr;
  }

  std::memcpy(name, &n_sockaddr, n_name_len);
  *name_len = n_name_len;

  // Create a kernel object to represent the new socket, and copy parameters
  // over.
  auto socket = object_ref<XSocket>(new XSocket(kernel_state_, ret));
  socket->af_ = af_;
  socket->type_ = type_;
  socket->proto_ = proto_;

  return socket;
}

int XSocket::Shutdown(int how) {
  return shutdown(native_handle_, how);
}

int XSocket::Recv(uint8_t* buf, uint32_t buf_len, uint32_t flags) {
#if REXGLUE_ENABLE_REXNET
  if (rexnet_stream_id_) {
    std::lock_guard<std::mutex> lock(stream_mutex_);
    if (!stream_rx_.empty()) {
      const uint32_t take = std::min<uint32_t>(buf_len, uint32_t(stream_rx_.size()));
      std::memcpy(buf, stream_rx_.data(), take);
      stream_rx_.erase(stream_rx_.begin(), stream_rx_.begin() + take);
      return int(take);
    }
    // Buffer empty: EOF only once the far side has actually closed, which is
    // what distinguishes "nothing yet" from "never again".
    if (rexnet_stream_eof_) {
      return 0;
    }
    XThread::SetLastError(0x2733);  // WSAEWOULDBLOCK
    return -1;
  }
#endif

  return recv(native_handle_, reinterpret_cast<char*>(buf), buf_len, flags);
}

int XSocket::RecvFrom(uint8_t* buf, uint32_t buf_len, uint32_t flags, N_XSOCKADDR_IN* from,
                      uint32_t* from_len) {
  // Virtual-network (RexNet) packets first: queued by the datagram sink in
  // xam_net.cpp with a 10.77.0.0/16 source.
  {
    std::lock_guard<std::mutex> lock(incoming_packet_mutex_);
    if (!incoming_packets_.empty()) {
      packet* pkt = (packet*)incoming_packets_.front();
      uint32_t copied = std::min((uint32_t)pkt->data_len, buf_len);
      std::memcpy(buf, pkt->data, copied);

      if (from) {
        from->sin_family = 2;
        from->sin_addr = pkt->src_ip;  // BE <- BE
        from->sin_port = pkt->src_port;
        std::memset(from->x_sin_zero, 0, sizeof(from->x_sin_zero));
      }
      if (from_len) {
        *from_len = sizeof(N_XSOCKADDR_IN);
      }

      incoming_packets_.pop();
      delete[] (uint8_t*)pkt;

      return (int)copied;
    }
  }

  sockaddr_in nfrom;
  socklen_t nfromlen = sizeof(sockaddr_in);
  int ret = recvfrom(native_handle_, reinterpret_cast<char*>(buf), buf_len, flags,
                     (sockaddr*)&nfrom, &nfromlen);
  if (from) {
    from->sin_family = nfrom.sin_family;
    from->sin_addr = ntohl(nfrom.sin_addr.s_addr);  // BE <- BE
    from->sin_port = nfrom.sin_port;
    std::memset(from->x_sin_zero, 0, sizeof(from->x_sin_zero));
  }

  if (from_len) {
    *from_len = nfromlen;
  }

  return ret;
}

int XSocket::Send(const uint8_t* buf, uint32_t buf_len, uint32_t flags) {
#if REXGLUE_ENABLE_REXNET
  if (rexnet_stream_id_) {
    auto* rexnet = rex::net::RexNet::shared();
    if (!rexnet) {
      return -1;
    }
    rexnet->StreamSend(rexnet_stream_id_, buf, buf_len);
    // The stream is reliable and buffered below us, so a partial write is not
    // a case the guest ever has to handle here.
    return int(buf_len);
  }
#endif

  return send(native_handle_, reinterpret_cast<const char*>(buf), buf_len, flags);
}

int XSocket::SendTo(uint8_t* buf, uint32_t buf_len, uint32_t flags, N_XSOCKADDR_IN* to,
                    uint32_t to_len) {
#if REXGLUE_ENABLE_REXNET
  // Datagrams to a virtual IP (10.77.0.0/16) route through the punched
  // RexNet game socket instead of the host network (design spec §11).
  if (to && type_ == Type::X_SOCK_DGRAM) {
    const uint32_t dst_ip = to->sin_addr;  // logical host-order value
    const bool is_virtual = (dst_ip & 0xFFFF0000u) == rex::net::VirtualIpTable::kNetworkBase;
    // SO_BROADCAST is required of the caller, as real Winsock requires.
    const bool limited_broadcast = dst_ip == 0xFFFFFFFFu && broadcast_socket_;
    // Our own address sits in 10.77/16, so a title deriving a subnet
    // broadcast from it aims inside the virtual network. Same intent, other
    // spelling.
    const bool directed_broadcast = is_virtual && (dst_ip & 0xFFu) == 0xFFu;
    const bool is_broadcast = limited_broadcast || directed_broadcast;

    // System Link is a *local* facility: a title broadcasts to its LAN and
    // plays with whoever answers. Carrying that over RexNet turns a LAN-only
    // game into an internet-capable one -- a deliberate choice, not a default,
    // because redirecting unconditionally would silently break ordinary
    // same-house play where two instances should just use the real network.
    const bool syslink_over_rexnet = REXCVAR_GET(rexnet_syslink);

    // Unicast to a peer's virtual address always belongs to RexNet; a
    // broadcast only does when the switch is on.
    const bool route_to_rexnet =
        (is_virtual && !is_broadcast) || (is_broadcast && syslink_over_rexnet);
    if (route_to_rexnet) {
      auto* rexnet = rex::net::RexNet::shared();
      if (!rexnet) {
        return -1;
      }
      rexnet->SendDatagram(dst_ip, bound_port_, to->sin_port, buf, buf_len);
      return (int)buf_len;
    }
  }
#endif

  sockaddr_in nto;
  if (to) {
    nto.sin_addr.s_addr = to->sin_addr;
    nto.sin_family = to->sin_family;
    nto.sin_port = to->sin_port;
  }

  return sendto(native_handle_, reinterpret_cast<char*>(buf), buf_len, flags,
                to ? (sockaddr*)&nto : nullptr, to_len);
}

bool XSocket::QueuePacket(uint32_t src_ip, uint16_t src_port, const uint8_t* buf, size_t len) {
  {
    // A real UDP socket discards arrivals once its receive buffer is full, so
    // drop the newcomer rather than evicting data the guest has not read yet.
    // Unbounded, a peer that sends faster than the title reads grows this
    // without limit -- every other RexNet queue is capped, this was the one
    // that was not.
    std::lock_guard<std::mutex> lock(incoming_packet_mutex_);
    if (incoming_packets_.size() >= kMaxQueuedPackets) {
      // Loud once, then rare: a full queue means the title is not draining,
      // which is worth knowing but repeats every datagram once it starts.
      if (++dropped_packets_ == 1 || dropped_packets_ % 1000 == 0) {
        REXLOG_WARN("socket :{} receive queue full ({}); {} datagrams dropped", bound_port_,
                    kMaxQueuedPackets, dropped_packets_);
      }
      return false;
    }
  }

  packet* pkt = reinterpret_cast<packet*>(new uint8_t[sizeof(packet) + len]);
  pkt->src_ip = src_ip;
  pkt->src_port = src_port;

  pkt->data_len = (uint16_t)len;
  std::memcpy(pkt->data, buf, len);

  {
    std::lock_guard<std::mutex> lock(incoming_packet_mutex_);
    incoming_packets_.push((uint8_t*)pkt);
  }

  // An overlapped receive may be waiting on exactly this.
  PumpPendingWsaRecv();
  // Event-driven titles (Armored Core 4) wait on a WSAEVENT rather than
  // polling, so an arrival has to signal it or they never wake.
  SignalWsaEvent();
  return true;
}

int XSocket::TryRecvFrom(uint8_t* buf, uint32_t buf_len, N_XSOCKADDR_IN* from) {
  // Virtual-network queue first.
  {
    std::lock_guard<std::mutex> lock(incoming_packet_mutex_);
    if (!incoming_packets_.empty()) {
      packet* pkt = (packet*)incoming_packets_.front();
      uint32_t copied = std::min((uint32_t)pkt->data_len, buf_len);
      std::memcpy(buf, pkt->data, copied);
      if (from) {
        from->sin_family = 2;
        from->sin_addr = pkt->src_ip;  // BE <- BE
        from->sin_port = pkt->src_port;
        std::memset(from->x_sin_zero, 0, sizeof(from->x_sin_zero));
      }
      incoming_packets_.pop();
      delete[] (uint8_t*)pkt;
      return (int)copied;
    }
  }

  // Host socket: poll readability, then receive (portable non-blocking).
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(native_handle_, &rfds);
  timeval tv{0, 0};
  int sel = select((int)native_handle_ + 1, &rfds, nullptr, nullptr, &tv);
  if (sel <= 0) {
    return -1;
  }
  sockaddr_in nfrom;
  socklen_t nfromlen = sizeof(nfrom);
  int ret = recvfrom(native_handle_, reinterpret_cast<char*>(buf), buf_len, 0, (sockaddr*)&nfrom,
                     &nfromlen);
  if (ret < 0) {
    return -1;
  }
  if (from) {
    from->sin_family = nfrom.sin_family;
    from->sin_addr = ntohl(nfrom.sin_addr.s_addr);  // BE <- BE
    from->sin_port = nfrom.sin_port;
    std::memset(from->x_sin_zero, 0, sizeof(from->x_sin_zero));
  }
  return ret;
}

void XSocket::CompleteWsaOverlapped(uint32_t overlapped_ptr, uint32_t status, uint32_t bytes) {
  if (!overlapped_ptr) {
    return;
  }
  auto* memory = kernel_state_->memory();
  uint8_t* ov = memory->TranslateVirtual(overlapped_ptr);
  memory::store_and_swap<uint32_t>(ov + kWsaOverlappedInternal, status);
  memory::store_and_swap<uint32_t>(ov + kWsaOverlappedInternalHigh, bytes);
  uint32_t event_handle = memory::load_and_swap<uint32_t>(ov + kWsaOverlappedEvent);
  if (event_handle) {
    auto ev = kernel_state_->object_table()->LookupObject<XEvent>(event_handle);
    if (ev) {
      ev->Set(0, false);
    }
  }

  // If this receive was posted with a Winsock completion routine, deliver it
  // via an APC on the issuing thread (alertable I/O). XRNM-style netcode posts
  // one WSARecvFrom with a completion routine, then waits alertably; the
  // routine both consumes the datagram and re-posts the receive. Without this
  // the routine never fires (its overlapped has no event), so a host that
  // receives a connect probe never processes it and never replies.
  uint32_t routine = pending_recv_completion_routine_;
  uint32_t thread_handle = pending_recv_apc_thread_;
  if (routine && thread_handle) {
    // We may be on a pure host thread (RexNet datagram / socket poller) with no
    // guest PPC context, where EnqueueApc cannot safely queue cross-thread.
    // Marshal onto the kernel dispatch thread (guest context) to queue the APC.
    KernelState* ks = kernel_state_;
    ks->PostToDispatchThread([ks, thread_handle, routine, status, bytes, overlapped_ptr]() {
      auto thread = ks->object_table()->LookupObject<XThread>(thread_handle);
      if (thread) {
        // Guest routine: void(dwError, cbTransferred, lpOverlapped, dwFlags).
        // EnqueueApc passes (normal_context, arg1, arg2) -> r3/r4/r5.
        thread->EnqueueApc(routine, status, bytes, overlapped_ptr);
      }
    });
  }
}

bool XSocket::SetPendingWsaRecv(std::vector<WsaRecvBuffer> buffers, uint32_t overlapped_ptr,
                                uint32_t from_ptr, uint32_t fromlen_ptr,
                                uint32_t completion_routine, uint32_t apc_thread_handle) {
  {
    std::lock_guard<std::mutex> lock(pending_recv_mutex_);
    if (pending_recv_active_) {
      return false;
    }
    pending_recv_active_ = true;
    pending_recv_buffers_ = std::move(buffers);
    pending_recv_overlapped_ = overlapped_ptr;
    pending_recv_from_ptr_ = from_ptr;
    pending_recv_fromlen_ptr_ = fromlen_ptr;
    pending_recv_completion_routine_ = completion_routine;
    pending_recv_apc_thread_ = apc_thread_handle;
    // Mark the guest overlapped as in flight.
    auto* memory = kernel_state_->memory();
    uint8_t* ov = memory->TranslateVirtual(overlapped_ptr);
    memory::store_and_swap<uint32_t>(ov + kWsaOverlappedInternal, kWsaStatusPending);
    memory::store_and_swap<uint32_t>(ov + kWsaOverlappedInternalHigh, 0);
  }
  AddPolledSocket(this);
  return true;
}

bool XSocket::PumpPendingWsaRecv() {
  std::lock_guard<std::mutex> lock(pending_recv_mutex_);
  if (!pending_recv_active_) {
    return false;
  }

  uint8_t tmp[2048];
  N_XSOCKADDR_IN from{};
  int received = TryRecvFrom(tmp, sizeof(tmp), &from);
  if (received < 0) {
    return false;
  }

  auto* memory = kernel_state_->memory();
  uint32_t copied = 0;
  for (const auto& buffer : pending_recv_buffers_) {
    if (copied >= (uint32_t)received) {
      break;
    }
    uint32_t chunk = std::min(buffer.len, (uint32_t)received - copied);
    std::memcpy(memory->TranslateVirtual(buffer.guest_ptr), tmp + copied, chunk);
    copied += chunk;
  }

  if (pending_recv_from_ptr_) {
    auto* guest_from = memory->TranslateVirtual<XSOCKADDR_IN*>(pending_recv_from_ptr_);
    guest_from->sin_family = from.sin_family;
    guest_from->sin_port = from.sin_port;
    guest_from->sin_addr = from.sin_addr;
    std::memset(guest_from->x_sin_zero, 0, sizeof(guest_from->x_sin_zero));
    if (pending_recv_fromlen_ptr_) {
      memory::store_and_swap<uint32_t>(memory->TranslateVirtual(pending_recv_fromlen_ptr_),
                                       sizeof(XSOCKADDR_IN));
    }
  }

  CompleteWsaOverlapped(pending_recv_overlapped_, 0, copied);
  pending_recv_active_ = false;
  pending_recv_buffers_.clear();
  pending_recv_completion_routine_ = 0;
  pending_recv_apc_thread_ = 0;
  RemovePolledSocket(this);
  return true;
}

}  // namespace rex::system
