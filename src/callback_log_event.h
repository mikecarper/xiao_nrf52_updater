#pragma once

#include <stddef.h>
#include <stdint.h>

namespace callback_log_event {

enum class Kind : uint8_t {
  kBleConnected,
  kBleDisconnected,
  kScanRejected,
};

// Plain, fixed-size data copied out of Bluefruit's callback task. Formatting
// and all logger/filesystem work happen later on the Arduino task.
struct Event {
  Kind kind;
  uint16_t conn_handle;
  uint8_t disconnect_reason;
  uint8_t addr[6];
  int8_t rssi;
  char name[24];
  char detail[8];
};

Event ble_connected(uint16_t conn_handle);
Event ble_disconnected(uint8_t reason);
Event scan_rejected(const uint8_t addr[6], int8_t rssi,
                    const char* name, const char* reason);

// Render one event without a timestamp or line ending. Returns false for an
// invalid output buffer or unknown event kind.
bool format(const Event& event, char* out, size_t out_len);

}  // namespace callback_log_event
