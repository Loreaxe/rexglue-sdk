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

#include <atomic>
#include <cstring>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include <rex/math.h>
#include <rex/system/xobject.h>
#include <rex/types.h>

namespace rex::system {

struct XSOCKADDR {
  rex::be<uint16_t> address_family;
  char sa_data[14];
};

struct N_XSOCKADDR {
  N_XSOCKADDR() {}
  N_XSOCKADDR(const XSOCKADDR* other) { *this = *other; }
  N_XSOCKADDR& operator=(const XSOCKADDR& other) {
    address_family = other.address_family;
    std::memcpy(sa_data, other.sa_data, rex::countof(sa_data));
    return *this;
  }

  uint16_t address_family;
  char sa_data[14];
};

struct XSOCKADDR_IN {
  rex::be<uint16_t> sin_family;

  // Always big-endian!
  rex::be<uint16_t> sin_port;
  rex::be<uint32_t> sin_addr;
  // sin_zero is defined as __pad on Android, so prefixed here.
  char x_sin_zero[8];
};

// Xenia native sockaddr_in
struct N_XSOCKADDR_IN {
  N_XSOCKADDR_IN() {}
  N_XSOCKADDR_IN(const XSOCKADDR_IN* other) { *this = *other; }
  N_XSOCKADDR_IN& operator=(const XSOCKADDR_IN& other) {
    sin_family = other.sin_family;
    sin_port = other.sin_port;
    sin_addr = other.sin_addr;
    std::memset(x_sin_zero, 0, sizeof(x_sin_zero));

    return *this;
  }

  uint16_t sin_family;
  rex::be<uint16_t> sin_port;
  rex::be<uint32_t> sin_addr;
  // sin_zero is defined as __pad on Android, so prefixed here.
  char x_sin_zero[8];
};

class XSocket : public XObject {
 public:
  static const XObject::Type kObjectType = XObject::Type::Socket;

  enum AddressFamily {
    AF_INET = 2,
  };

  enum Type {
    SOCK_STREAM = 1,
    SOCK_DGRAM = 2,
  };

  enum Protocol {
    IPPROTO_TCP = 6,
    IPPROTO_UDP = 17,

    // LIVE Voice and Data Protocol
    // https://blog.csdn.net/baozi3026/article/details/4277227
    // Format: [cbGameData][GameData(encrypted)][VoiceData(unencrypted)]
    IPPROTO_VDP = 254,
  };

  XSocket(KernelState* kernel_state);
  ~XSocket();

  uint64_t native_handle() const { return native_handle_; }
  uint16_t bound_port() const { return bound_port_; }

  X_STATUS Initialize(AddressFamily af, Type type, Protocol proto);
  X_STATUS Close();

  X_STATUS SetOption(uint32_t level, uint32_t optname, void* optval_ptr, uint32_t optlen);
  X_STATUS IOControl(uint32_t cmd, uint8_t* arg_ptr);

  X_STATUS Connect(N_XSOCKADDR* name, int name_len);
  X_STATUS Bind(N_XSOCKADDR_IN* name, int name_len);
  X_STATUS Listen(int backlog);
  object_ref<XSocket> Accept(N_XSOCKADDR* name, int* name_len);
  int Shutdown(int how);

  int Recv(uint8_t* buf, uint32_t buf_len, uint32_t flags);
  int Send(const uint8_t* buf, uint32_t buf_len, uint32_t flags);

  int RecvFrom(uint8_t* buf, uint32_t buf_len, uint32_t flags, N_XSOCKADDR_IN* from,
               uint32_t* from_len);
  int SendTo(uint8_t* buf, uint32_t buf_len, uint32_t flags, N_XSOCKADDR_IN* to, uint32_t to_len);

  struct packet {
    // These values are in network byte order.
    rex::be<uint16_t> src_port;
    rex::be<uint32_t> src_ip;

    uint16_t data_len;
    uint8_t data[1];
  };

  // Queue a packet into our internal buffer (src values host byte order).
  // Also completes a pending overlapped receive, if any.
  bool QueuePacket(uint32_t src_ip, uint16_t src_port, const uint8_t* buf, size_t len);
  bool HasQueuedPackets();

  // Find the UDP socket bound to a guest port (RexNet inbound routing).
  static object_ref<XSocket> FindBoundUdp(uint16_t port);

  // --- Overlapped receive (WSARecvFrom) ---------------------------------
  // One pending receive per socket (the WSA model XRNM-style netcode uses:
  // post, wait on the event, WSAGetOverlappedResult, repeat).

  struct WsaRecvBuffer {
    uint32_t guest_ptr;
    uint32_t len;
  };

  // Non-blocking receive: RexNet queue first, then host socket.
  // Returns -1 when nothing is available.
  int TryRecvFrom(uint8_t* buf, uint32_t buf_len, N_XSOCKADDR_IN* from);

  // --- Guest TCP over RexNet (design spec §18) ---------------------------
  //
  // A SOCK_STREAM socket whose peer is a virtual IP is carried by a libp2p
  // stream rather than the host stack, which has no route to 10.77.0.0/16.
  // System Link titles need this: SoulCalibur IV's session runs on TCP 1001.

  /// Look up the listening stream socket bound to a guest port, if any.
  static object_ref<XSocket> FindStreamListener(uint16_t port);
  /// Look up the stream socket owning a connection.
  static object_ref<XSocket> FindStream(uint64_t stream_id);

  /// Mark this socket as a RexNet stream listener (from listen()).
  void BecomeStreamListener();
  /// Queue an accepted connection for accept() to hand out.
  void QueueAcceptedStream(uint64_t stream_id, uint32_t peer_vip, uint16_t remote_port);
  /// Deliver received bytes to a connected stream socket.
  void QueueStreamData(const uint8_t* data, uint32_t len);
  /// The far side (or the peer connection) went away.
  void OnStreamClosed();
  /// Bind this socket to an established connection (accept/connect result).
  void AdoptStream(uint64_t stream_id, uint32_t peer_vip, uint16_t remote_port);

  /// Claim the socket that is awaiting an outbound connect to this address,
  /// so the resulting stream can be handed to it. Returns null if none is
  /// waiting (a connect that was abandoned, or a duplicate completion).
  static object_ref<XSocket> ClaimPendingConnect(uint32_t peer_vip, uint16_t dst_port);
  /// Record that this socket is awaiting an outbound connect.
  void BeginPendingConnect(uint32_t peer_vip, uint16_t dst_port);
  /// The outbound connect failed; a later connect()/select() must see that.
  void OnStreamConnectFailed();

  /// Winsock signals a completed non-blocking connect by making the socket
  /// writable, which is how a title that set FIONBIO learns it may send.
  bool stream_writable();
  bool stream_connect_failed() const {
    return rexnet_connect_failed_.load(std::memory_order_acquire);
  }

  bool is_rexnet_stream() const { return rexnet_stream_id_ != 0; }
  bool stream_readable();

  // --- WSAEventSelect ----------------------------------------------------
  //
  // Event-driven socket notification: the guest associates a WSAEVENT with a
  // socket, waits on it, then reads. Armored Core 4 drives its whole socket
  // loop this way (WSACreateEvent + WSAEventSelect +
  // WSAWaitForMultipleEvents), so a socket that never signals leaves the
  // title waiting forever -- the same shape of failure as an overlapped
  // receive whose completion never fires.

  /// Associate a guest WSAEVENT with this socket. `mask` is FD_READ etc.;
  /// handle 0 clears the association.
  void SetEventSelect(uint32_t event_handle, uint32_t mask);
  /// Signal the associated guest event, if any. Called whenever the socket
  /// becomes readable.
  void SignalWsaEvent();

  // Register an overlapped receive (guest addresses; completed from the
  // packet queue or the socket poller thread). False if one is pending.
  // When completion_routine is non-zero the receive was posted with a Winsock
  // completion routine (alertable I/O, hEvent typically NULL): on completion an
  // APC calling completion_routine(dwError, cbTransferred, lpOverlapped, ...) is
  // queued to apc_thread_handle. This is how XRNM-style netcode is driven.
  bool SetPendingWsaRecv(std::vector<WsaRecvBuffer> buffers, uint32_t overlapped_ptr,
                         uint32_t from_ptr, uint32_t fromlen_ptr = 0,
                         uint32_t completion_routine = 0, uint32_t apc_thread_handle = 0);
  // Try to satisfy the pending receive; returns true if it completed.
  bool PumpPendingWsaRecv();

 private:
  XSocket(KernelState* kernel_state, uint64_t native_handle);

  // Write status/bytes into a guest XWSAOVERLAPPED and set its event.
  void CompleteWsaOverlapped(uint32_t overlapped_ptr, uint32_t status, uint32_t bytes);

  uint64_t native_handle_ = -1;

  AddressFamily af_;    // Address family
  Type type_;           // Type (DGRAM/Stream/etc)
  Protocol proto_;      // Protocol (TCP/UDP/etc)
  bool secure_ = true;  // Secure socket (encryption enabled)

  bool bound_ = false;  // Explicitly bound to an IP address?
  uint16_t bound_port_ = 0;

  bool broadcast_socket_ = false;

  std::unique_ptr<rex::thread::Event> event_;
  /// Guest WSAEVENT associated by WSAEventSelect, and the event mask it
  /// asked for. Read from the pump thread, written from guest threads.
  std::atomic<uint32_t> wsa_event_handle_{0};
  std::atomic<uint32_t> wsa_event_mask_{0};
  std::mutex incoming_packet_mutex_;
  std::queue<uint8_t*> incoming_packets_;

  // --- Guest TCP state (§18) ---
  /// Non-zero once this socket owns a RexNet stream.
  uint64_t rexnet_stream_id_ = 0;
  uint32_t rexnet_peer_vip_ = 0;
  uint16_t rexnet_remote_port_ = 0;
  /// True between listen() and close() for a RexNet stream listener.
  bool rexnet_listening_ = false;
  /// Set when an outbound connect resolved as failed.
  std::atomic<bool> rexnet_connect_failed_{false};
  /// Set when the far side closed, so recv() can report EOF once the buffer
  /// drains rather than the moment the close arrives.
  bool rexnet_stream_eof_ = false;
  std::mutex stream_mutex_;
  /// Received bytes awaiting recv(). A stream is a byte sequence, so this is
  /// deliberately one buffer rather than a queue of messages.
  std::vector<uint8_t> stream_rx_;
  struct PendingAccept {
    uint64_t stream_id;
    uint32_t peer_vip;
    uint16_t remote_port;
  };
  std::vector<PendingAccept> pending_accepts_;

  std::mutex pending_recv_mutex_;
  bool pending_recv_active_ = false;
  std::vector<WsaRecvBuffer> pending_recv_buffers_;
  uint32_t pending_recv_overlapped_ = 0;
  uint32_t pending_recv_from_ptr_ = 0;
  uint32_t pending_recv_fromlen_ptr_ = 0;
  // Winsock completion-routine (alertable) delivery for the pending receive.
  uint32_t pending_recv_completion_routine_ = 0;
  uint32_t pending_recv_apc_thread_ = 0;
};

}  // namespace rex::system