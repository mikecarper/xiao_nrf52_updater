#pragma once

#include <stddef.h>
#include <stdint.h>

namespace dfu_protocol {

static constexpr uint8_t kResponseCode = 0x10;
static constexpr uint8_t kPacketReceiptNotification = 0x11;

enum class NotificationKind : uint8_t {
  kUnknown,
  kResponse,
  kPacketReceipt,
};

inline NotificationKind classify(const uint8_t* data, size_t len) {
  if (!data || len == 0) return NotificationKind::kUnknown;
  if (data[0] == kResponseCode) return NotificationKind::kResponse;
  if (data[0] == kPacketReceiptNotification) {
    return NotificationKind::kPacketReceipt;
  }
  return NotificationKind::kUnknown;
}

inline bool decode_response(const uint8_t* data, size_t len,
                            uint8_t* op, uint8_t* status) {
  if (classify(data, len) != NotificationKind::kResponse || len < 3 ||
      !op || !status) {
    return false;
  }
  *op = data[1];
  *status = data[2];
  return true;
}

inline bool decode_prn(const uint8_t* data, size_t len, uint32_t* received) {
  if (classify(data, len) != NotificationKind::kPacketReceipt || len < 5 ||
      !received) {
    return false;
  }
  *received = static_cast<uint32_t>(data[1]) |
              (static_cast<uint32_t>(data[2]) << 8) |
              (static_cast<uint32_t>(data[3]) << 16) |
              (static_cast<uint32_t>(data[4]) << 24);
  return true;
}

}  // namespace dfu_protocol
