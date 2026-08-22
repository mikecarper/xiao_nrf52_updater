#pragma once

#include <Arduino.h>

namespace v4_config {

struct Config {
  char ble_name[32];
  bool ble_mac_set;
  uint8_t ble_mac[6];  // BLE-native, least-significant octet first.
  uint16_t prn;
  bool high_mtu;
  uint8_t retries;
  int8_t min_rssi;
  uint16_t retry_cooldown;
  uint16_t wedge_cooldown;
  uint16_t scan_timeout;
  bool scan_debug;
};

void begin();
const Config& current();

// A target is deliberately mandatory.  The V4 never falls back to the first
// arbitrary Legacy DFU advertiser because this build can transmit firmware.
bool set_target_name(const char* name);
bool set_target_mac(const char* text);
void clear_target();
bool has_target();
void describe_target(char* out, size_t out_len);

}  // namespace v4_config
