/*
 * Hitcon.cpp
 *
 *  Created on: Jun 30, 2024
 *      Author: aoaaceai
 */

#include <Hitcon.h>
#include <Service/IrService.h>
#include <Service/Sched/Scheduler.h>
#include <stdint.h>

using namespace hitcon;
using namespace hitcon::service::sched;

void cb(void *unused1, void *unused2) {
  uint8_t data[]{0xa, 0xb, 0xc, 0xd, 0xe, 0xf};
  size_t len = 6;
  hitcon::ir::irService.SendBuffer(data, len);
}

Task InitTask(1, (task_callback_t)&cb, nullptr);

void hitcon_run() {
  hitcon::ir::irService.Init();
  scheduler.Queue(&InitTask, nullptr);
  scheduler.Run();
}
