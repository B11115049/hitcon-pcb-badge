#include <Logic/BaseStnHub.h>
#include <Logic/IrController.h>

#include <cstring>
using namespace hitcon::service::sched;
using namespace hitcon::ir;
using namespace hitcon::logic::cdc;

namespace hitcon {
namespace basestn {

BaseStationHub g_basestn_hub;

BaseStationHub::BaseStationHub()
    : _routine_task(490, (task_callback_t)&BaseStationHub::Routine, this, 20) {}

void BaseStationHub::Init() {
  g_cdc_logic.SetOnPacketArrive((callback_t)&BaseStationHub::QueueTxHandler,
                                this, FnId::QueueStationTX);
  scheduler.Queue(&_routine_task, nullptr);
  scheduler.EnablePeriodic(&_routine_task);
}

bool BaseStationHub::WriteBuffer(BufferType buffer_type, uint8_t* data,
                                 size_t cnt) {
  if (cnt > kBufferSize - 1) {
    return false;
  }
  auto& buffer = buffer_type == BufferType::RX ? rx_buffer : tx_buffer;
  for (uint8_t i = 0; i < kBufferCount; i++) {
    auto& status = reinterpret_cast<BufferMeta&>(buffer[i * kBufferSize]);
    if (status.locked) continue;
    status.locked = 1;
    status.length = cnt;
    memcpy(&buffer[i * kBufferSize + 1], data, cnt);
    return true;
  }
  return false;
}

bool BaseStationHub::ReadBuffer(BufferType buffer_type, uint8_t* data,
                                size_t& cnt) {
  auto& buffer = buffer_type == BufferType::RX ? rx_buffer : tx_buffer;
  for (uint8_t i = 0; i < kBufferCount; i++) {
    auto& status = reinterpret_cast<BufferMeta&>(buffer[i * kBufferSize]);
    if (!status.locked) continue;
    if (status.length > cnt) continue;
    status.locked = 0;
    memcpy(data, &buffer[i * kBufferSize + 1], status.length);
    cnt = status.length;
    return true;
  }
  return false;
}

void BaseStationHub::OnIrPacketRecv(uint8_t* data, size_t cnt) {
  WriteBuffer(BufferType::RX, data, cnt);
}

void BaseStationHub::QueueTxHandler(PacketCallbackArg* arg) {
  bool success = WriteBuffer(BufferType::TX, arg->data, arg->len);
  uint8_t buffer[HEADER_SZ + 1] = {0};
  auto header = reinterpret_cast<PktHdr*>(buffer);
  auto payload = buffer + HEADER_SZ;
  header->id = arg->id;
  header->type = 0x81;
  header->len = 1;
  payload[0] = success ? 2 : 1;
  g_cdc_logic.SendPacket(buffer);
}

void BaseStationHub::Routine(void*) {
  // try to send ir
  for (uint8_t i = 0; i < kBufferCount; i++) {
    auto& buffer = tx_buffer;
    auto& status = reinterpret_cast<BufferMeta&>(buffer[i * kBufferSize]);
    if (!status.locked) continue;
    IrData irdata = {
        .ttl = 0,
    };
    memcpy(&irdata.type, &buffer[i * kBufferSize], status.length);
    bool success =
        irLogic.SendPacket(reinterpret_cast<uint8_t*>(&irdata), status.length);
    // release if sent successfully
    if (success) {
      status.locked = 0;
    }
    // sent at most one buffer each routine
    break;
  }
}

}  // namespace basestn
}  // namespace hitcon
