#include "callback_log_event.h"

#include <stdio.h>
#include <string.h>

namespace callback_log_event {

namespace {

Event empty_event(Kind kind) {
  Event event = {};
  event.kind = kind;
  return event;
}

void copy_text(char* out, size_t out_len, const char* text) {
  if (!out || out_len == 0) return;
  snprintf(out, out_len, "%s", text ? text : "");
}

}  // namespace

Event ble_connected(uint16_t conn_handle) {
  Event event = empty_event(Kind::kBleConnected);
  event.conn_handle = conn_handle;
  return event;
}

Event ble_disconnected(uint8_t reason) {
  Event event = empty_event(Kind::kBleDisconnected);
  event.disconnect_reason = reason;
  return event;
}

Event scan_rejected(const uint8_t addr[6], int8_t rssi,
                    const char* name, const char* reason) {
  Event event = empty_event(Kind::kScanRejected);
  if (addr) memcpy(event.addr, addr, sizeof(event.addr));
  event.rssi = rssi;
  copy_text(event.name, sizeof(event.name), name);
  copy_text(event.detail, sizeof(event.detail), reason);
  return event;
}

bool format(const Event& event, char* out, size_t out_len) {
  if (!out || out_len == 0) return false;

  int written = -1;
  switch (event.kind) {
    case Kind::kBleConnected:
      written = snprintf(out, out_len, "dfu: connected (conn=%u)",
                         static_cast<unsigned>(event.conn_handle));
      break;
    case Kind::kBleDisconnected:
      written = snprintf(out, out_len, "dfu: disconnected reason=0x%02x",
                         static_cast<unsigned>(event.disconnect_reason));
      break;
    case Kind::kScanRejected:
      written = snprintf(
          out, out_len,
          "scan: %s %02X:%02X:%02X:%02X:%02X:%02X rssi=%d name='%s'",
          event.detail, static_cast<unsigned>(event.addr[5]),
          static_cast<unsigned>(event.addr[4]),
          static_cast<unsigned>(event.addr[3]),
          static_cast<unsigned>(event.addr[2]),
          static_cast<unsigned>(event.addr[1]),
          static_cast<unsigned>(event.addr[0]), event.rssi,
          event.name);
      break;
    default:
      out[0] = '\0';
      return false;
  }

  if (written < 0) {
    out[0] = '\0';
    return false;
  }
  out[out_len - 1] = '\0';
  return true;
}

}  // namespace callback_log_event
