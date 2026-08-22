#include "deferred_logger.h"

#include <FreeRTOS.h>
#include <queue.h>
#include <type_traits>

#include "callback_log_event.h"
#include "logger.h"

namespace deferred_logger {

namespace {

using callback_log_event::Event;
static_assert(std::is_trivially_copyable<Event>::value,
              "FreeRTOS queues copy events byte-for-byte");

static constexpr size_t kQueueDepth = 16;
static StaticQueue_t s_queue_control;
alignas(portBYTE_ALIGNMENT)
static uint8_t s_queue_storage[kQueueDepth * sizeof(Event)];
static QueueHandle_t s_queue = nullptr;
static volatile uint32_t s_dropped = 0;

void note_drop() {
  taskENTER_CRITICAL();
  ++s_dropped;
  taskEXIT_CRITICAL();
}

void post(const Event& event) {
  if (!s_queue || xQueueSend(s_queue, &event, 0) != pdPASS) note_drop();
}

uint32_t take_drop_count() {
  taskENTER_CRITICAL();
  uint32_t dropped = s_dropped;
  s_dropped = 0;
  taskEXIT_CRITICAL();
  return dropped;
}

}  // namespace

bool begin() {
  if (s_queue) return true;
  s_queue = xQueueCreateStatic(kQueueDepth, sizeof(Event), s_queue_storage,
                              &s_queue_control);
  return s_queue != nullptr;
}

void post_ble_connected(uint16_t conn_handle) {
  post(callback_log_event::ble_connected(conn_handle));
}

void post_ble_disconnected(uint8_t reason) {
  post(callback_log_event::ble_disconnected(reason));
}

void post_scan_rejected(const uint8_t addr[6], int8_t rssi,
                        const char* name, const char* reason) {
  post(callback_log_event::scan_rejected(addr, rssi, name, reason));
}

void drain() {
  if (s_queue) {
    Event event;
    char message[112];
    // Bound each service pass even if a noisy scan keeps the producer busy.
    // Remaining events stay queued for the next Arduino-task poll.
    for (size_t i = 0; i < kQueueDepth; ++i) {
      if (xQueueReceive(s_queue, &event, 0) != pdPASS) break;
      if (callback_log_event::format(event, message, sizeof(message))) {
        logger::log("%s", message);
      }
    }
  }

  uint32_t dropped = take_drop_count();
  if (dropped != 0) {
    logger::log("callback-log: queue overflow (%lu dropped)",
                (unsigned long)dropped);
  }
}

}  // namespace deferred_logger
