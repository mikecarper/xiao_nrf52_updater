#pragma once

#include <stdint.h>

namespace deferred_logger {

// Create the static callback-to-main queue. Call once from setup() before BLE
// callbacks can run.
bool begin();

// Callback-task entry points. These only copy fixed-size data into a FreeRTOS
// queue; they never touch Serial, SdFat, or QSPI.
void post_ble_connected(uint16_t conn_handle);
void post_ble_disconnected(uint8_t reason);
void post_scan_rejected(const uint8_t addr[6], int8_t rssi,
                        const char* name, const char* reason);

// Drain and format queued events through logger::log(). This must be called
// only by the Arduino task.
void drain();

}  // namespace deferred_logger
