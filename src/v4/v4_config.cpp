#include "v4_config.h"

#include "config_parse.h"

namespace v4_config {

static Config s_config;

void begin() {
  memset(&s_config, 0, sizeof(s_config));
  s_config.prn = 8;
  s_config.high_mtu = true;
  s_config.retries = 3;
  s_config.min_rssi = -90;
  s_config.retry_cooldown = 5;
  s_config.wedge_cooldown = 60;
  s_config.scan_timeout = 120;
  s_config.scan_debug = false;
}

const Config& current() { return s_config; }

bool set_target_name(const char* name) {
  if (!name) return false;
  while (*name == ' ' || *name == '\t') ++name;
  size_t len = strlen(name);
  while (len && (name[len - 1] == ' ' || name[len - 1] == '\t')) --len;
  if (len == 0 || len >= sizeof(s_config.ble_name)) return false;
  memcpy(s_config.ble_name, name, len);
  s_config.ble_name[len] = '\0';
  s_config.ble_mac_set = false;
  memset(s_config.ble_mac, 0, sizeof(s_config.ble_mac));
  return true;
}

bool set_target_mac(const char* text) {
  uint8_t parsed[6];
  if (!config_parse::ble_mac(text, parsed)) return false;
  memcpy(s_config.ble_mac, parsed, sizeof(parsed));
  s_config.ble_mac_set = true;
  s_config.ble_name[0] = '\0';
  return true;
}

void clear_target() {
  s_config.ble_name[0] = '\0';
  s_config.ble_mac_set = false;
  memset(s_config.ble_mac, 0, sizeof(s_config.ble_mac));
}

bool has_target() {
  return s_config.ble_name[0] != '\0' || s_config.ble_mac_set;
}

void describe_target(char* out, size_t out_len) {
  if (!out || out_len == 0) return;
  if (s_config.ble_name[0] != '\0') {
    snprintf(out, out_len, "name:%s", s_config.ble_name);
  } else if (s_config.ble_mac_set) {
    snprintf(out, out_len, "mac:%02X:%02X:%02X:%02X:%02X:%02X",
             s_config.ble_mac[5], s_config.ble_mac[4],
             s_config.ble_mac[3], s_config.ble_mac[2],
             s_config.ble_mac[1], s_config.ble_mac[0]);
  } else {
    snprintf(out, out_len, "<unset>");
  }
}

}  // namespace v4_config
