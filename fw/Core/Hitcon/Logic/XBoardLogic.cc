#include <Logic/XBoardLogic.h>

#include <cstring>

using namespace hitcon::service::sched;
using namespace hitcon::service::xboard;

namespace hitcon {
namespace service {
namespace xboard {

XBoardLogic g_xboard_logic;

constexpr size_t PKT_DATA_LEN_MAX = 256;
static uint8_t packet_data[PKT_DATA_LEN_MAX];

static callback_t packet_cb = nullptr;
static void *packet_cb_self = nullptr;
// static PacketCallbackArg *packet_cb_arg;

struct Frame {
  uint64_t preamble;  // 0xD555555555555555
  uint16_t id;
  uint8_t len;
  uint8_t type;  // 8: ping
  uint16_t checksum;
};
constexpr size_t HEADER_SZ = sizeof(Frame);

uint8_t tx_buf[] = "Hello World";
uint8_t rx_buf[RX_BUF_SZ] = {0};
static int prod_head = 0;
static int cons_head = 0;
static bool recv_ping = false;
static uint8_t no_ping_count = 0;

uint8_t alive_message[] = "alive_message";
void send_ping() {
  uint8_t pkt[sizeof(Frame)] = {0};
  *(Frame *)pkt = Frame{0xD555555555555555, 0, 0, 8, 0};
  // for (int i = 0; i < sizeof(Frame); i++) {
  //   pkt[i] = (0x11+i)&0x0FF;
  //   pkt[i] = 0;
  // }
  g_xboard_service.QueueDataForTx(pkt, sizeof(pkt));
}

inline uint16_t inc_head(size_t head, size_t offset) {
  return (head + offset) % RX_BUF_SZ;
}

// read `size` bytes from `rx_buf` to `dst`
// if `head_offset` > 0, start reading from cons_head + head_offset
// return false if no enough bytes to read
bool try_read_bytes(uint8_t *dst, size_t size, uint16_t head_offset = 0) {
  uint16_t _cons_head = ((uint16_t)cons_head + head_offset) % RX_BUF_SZ;
  if (cons_head == prod_head) {
    if (_cons_head != cons_head) return false;
  } else if (cons_head < prod_head) {
    // O: c <= _c <= p
    if (_cons_head > prod_head) return false;
    if (_cons_head < cons_head) return false;
  } else {
    // O: p < c <= _c
    // O: _c <= p < c
    if (prod_head < _cons_head && _cons_head < cons_head) return false;
  }
  unsigned int remain_size =
      (_cons_head > prod_head ? 0 : RX_BUF_SZ) - prod_head + _cons_head;
  if (remain_size < size) {
    return false;
  }
  uint16_t next_cons_head = inc_head(_cons_head, size);
  if (next_cons_head < _cons_head) {
    uint16_t sz1 = RX_BUF_SZ - _cons_head;
    memcpy(dst, rx_buf + _cons_head, sz1);
    memcpy(dst + sz1, rx_buf, size - sz1);
  } else {
    memcpy(dst, rx_buf + _cons_head, size);
  }
  return true;
}

void XBoardLogic::ParsePacket() {
  recv_ping = false;
  while (cons_head != prod_head) {
    if (rx_buf[cons_head] != 0x55) {
      cons_head = (cons_head + 1) % RX_BUF_SZ;
      continue;
    }
    unsigned int in_buf_size =
        (prod_head > cons_head ? 0 : RX_BUF_SZ) + prod_head - cons_head;
    if ((unsigned int)in_buf_size < sizeof(Frame)) {
      break;
    }

    Frame header;
    try_read_bytes((uint8_t *)&header, HEADER_SZ);
    if (header.preamble != 0xD555555555555555) {
      cons_head = inc_head(cons_head, 1);
      continue;
    }
    if (!try_read_bytes((uint8_t *)&packet_data, header.len, sizeof(Frame))) {
      // no enough bytes to read, wait more bytes in
      return;
    }
    cons_head = inc_head(cons_head, HEADER_SZ + header.len);
    if (header.type == 8) {
      recv_ping = true;
      continue;
    }
    if (packet_arrive_handler != nullptr) {
      PacketCallbackArg packet_cb_arg;
      memcpy(packet_cb_arg.data, &packet_data, header.len);
      packet_cb_arg.len = header.len;
      packet_cb(packet_cb_self, &packet_cb_arg);
    }
  }
}

XBoardLogic::XBoardLogic()
    : _routine_task(490, (task_callback_t)&XBoardLogic::Routine, this, 200) {}

void XBoardLogic::Init() {
  scheduler.Queue(&_routine_task, nullptr);
  scheduler.EnablePeriodic(&_routine_task);
  g_xboard_service.SetOnByteRx((callback_t)&XBoardLogic::OnByteArrive, this);
}

void XBoardLogic::QueuePacketForTx(uint8_t *packet, size_t packet_len) {
  // TODO
}

void XBoardLogic::SetOnConnectCallback(callback_t callback, void *self) {
  connect_handler = callback;
  connect_handler_self = self;
}

void XBoardLogic::SetOnDisconnectCallback(callback_t callback, void *self) {
  disconnect_handler = callback;
  disconnect_handler_self = self;
}

void XBoardLogic::SetOnPacketCallback(callback_t callback, void *self/*,
                                      PacketCallbackArg *callback_arg*/) {
  packet_arrive_handler = callback;
  packet_arrive_handler_self = self;
}

void XBoardLogic::Routine(void *) {
  send_ping();
  ParsePacket();
  CheckPing();
}

void XBoardLogic::OnByteArrive(void *arg1) {
  uint8_t b = static_cast<uint8_t>(reinterpret_cast<size_t>(arg1));
  uint16_t next_prod_head = inc_head(prod_head, 1);
  if (next_prod_head == cons_head) {
    // drop the data
    AssertOverflow();
    return;
  }
  rx_buf[prod_head] = b;
  prod_head = next_prod_head;
}

void XBoardLogic::CheckPing() {
  if (!recv_ping) {
    if (no_ping_count < 3) {
      ++no_ping_count;
    }
  } else {
    no_ping_count = 0;
  }
  UsartConnectState next_state = no_ping_count >= 3 ? Disconnect : Connect;
  if (next_state != connect_state) {
    if (next_state == Disconnect && disconnect_handler != nullptr) {
      disconnect_handler(disconnect_handler_self, nullptr);
    } else if (next_state == Connect && connect_handler != nullptr) {
      connect_handler(connect_handler_self, nullptr);
    }
  }
  connect_state = next_state;
}

}  // namespace xboard
}  // namespace service
}  // namespace hitcon
