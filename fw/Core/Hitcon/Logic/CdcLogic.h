#ifndef HITCON_LOGIC_CDC_SERVICE_H_
#define HITCON_LOGIC_CDC_SERVICE_H_
#include <utility>
#include <Util/callback.h>
#include "Service/Sched/Scheduler.h"
#include "CdcLogicFn.h"

namespace hitcon {
namespace logic {
namespace cdc {

constexpr size_t BUF_CAPACITY = 128 + 1;
constexpr size_t PKT_PAYLOAD_LEN_MAX = 32;

struct PacketCallbackArg {
  uint8_t *data;
  uint8_t len;
};

class CdcLogic {
 public:
  CdcLogic();
  void Init();
  void SetOnPacketArrive(callback_t callback, void *self, FnId handler_id);

 private:
  uint16_t prod_head = 0;
  uint16_t cons_head = 0;
  uint8_t buf[BUF_CAPACITY] = {0};
  std::pair<callback_t, void *> packet_arrive_cbs[FnId::MAX] = {};
  hitcon::service::sched::PeriodicTask _parse_routine;

  bool IsBufferFull();
  uint16_t BufferLen();
  uint16_t BufferSpace();
  void OnDataReceived(uint8_t* data, size_t len);
  void ParsePacket();
  bool TryReadBytes(uint8_t *dst, size_t size, uint16_t head_offset = 0);
  void ParseRoutine(void *);
};
extern CdcLogic g_cdc_logic;
}  // namespace cdc
}  // namespace logic
}  // namespace hitcon
#endif  // HITCON_LOGIC_CDC_SERVICE_H_
