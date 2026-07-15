#include "config.h"

#include "logger.h"
#include "storage.h"
#include <stdio.h>

namespace config {

static Config s_current;

static void apply_defaults(Config* c) {
  c->ble_name[0]      = '\0';
  c->ble_mac_set = false;
  memset(c->ble_mac, 0, sizeof(c->ble_mac));
  c->prn              = 8;
  c->high_mtu         = true;
  c->retries          = 3;
  c->min_rssi         = -90;  // minimal viable signal
  c->retry_cooldown = 5;
  c->wedge_cooldown = 60;
  c->tx_power       = 4;
  c->scan_timeout   = 0;     // 0 = infinite, scan forever
  c->scan_debug     = false;
}

static void trim(char* s) {
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t' ||
                   s[n - 1] == '\r' || s[n - 1] == '\n')) {
    s[--n] = '\0';
  }
  size_t lead = 0;
  while (s[lead] == ' ' || s[lead] == '\t') lead++;
  if (lead) memmove(s, s + lead, n - lead + 1);
}

static bool parse_bool(const char* v) {
  return v[0] == '1' || v[0] == 't' || v[0] == 'T' || v[0] == 'y' || v[0] == 'Y';
}

static bool parse_ble_mac(const char* s, uint8_t out[6]) {
  unsigned int b0, b1, b2, b3, b4, b5;

  int n = sscanf(s, "%x:%x:%x:%x:%x:%x", &b0, &b1, &b2, &b3, &b4, &b5);
  if (n != 6) {
    n = sscanf(s, "%x-%x-%x-%x-%x-%x", &b0, &b1, &b2, &b3, &b4, &b5);
  }

  if (n != 6) return false;

  if (b0 > 0xFF || b1 > 0xFF || b2 > 0xFF ||
      b3 > 0xFF || b4 > 0xFF || b5 > 0xFF) {
    return false;
  }

  // Nordic/SoftDevice stores BLE address bytes little-endian.
  // Logs print addr[5]..addr[0], so store AA:BB:CC:DD:EE:FF as:
  // out[5]=AA ... out[0]=FF
  out[5] = (uint8_t)b0;
  out[4] = (uint8_t)b1;
  out[3] = (uint8_t)b2;
  out[2] = (uint8_t)b3;
  out[1] = (uint8_t)b4;
  out[0] = (uint8_t)b5;

  return true;
}

static void apply_kv(Config* c, const char* key, const char* val) {
  if      (!strcmp(key, "ble_name")) {
    snprintf(c->ble_name, sizeof(c->ble_name), "%s", val);
  } else if (!strcmp(key, "ble_mac")) {
  c->ble_mac_set = false;
  memset(c->ble_mac, 0, sizeof(c->ble_mac));

  if (val[0] != '\0') {
    if (parse_ble_mac(val, c->ble_mac)) {
      c->ble_mac_set = true;
    } else {
      logger::log("cfg: invalid ble_mac '%s'", val);
    }
  }
  } else if (!strcmp(key, "prn")) {
    int n = atoi(val);
    if (n >= 0 && n <= 65535) c->prn = (uint16_t)n;
  } else if (!strcmp(key, "high_mtu")) {
    c->high_mtu = parse_bool(val);
  } else if (!strcmp(key, "retries")) {
    int n = atoi(val);
    if (n >= 1 && n <= 255) c->retries = (uint8_t)n;
  } else if (!strcmp(key, "min_rssi")) {
    int n = atoi(val);
    if (n >= -127 && n <= 0) c->min_rssi = (int8_t)n;
  } else if (!strcmp(key, "retry_cooldown")) {
    int n = atoi(val);
    if (n >= 0 && n <= 600) c->retry_cooldown = (uint16_t)n;
  } else if (!strcmp(key, "wedge_cooldown")) {
    int n = atoi(val);
    if (n >= 0 && n <= 600) c->wedge_cooldown = (uint16_t)n;
  } else if (!strcmp(key, "tx_power")) {
    int n = atoi(val);
    // We don't enforce the allowed-values list here; Bluefruit::setTxPower
    // will silently reject invalid values when we apply it.
    if (n >= -40 && n <= 8) c->tx_power = (int8_t)n;
  } else if (!strcmp(key, "scan_timeout")) {
    int n = atoi(val);
    if (n >= 0 && n <= 65535) c->scan_timeout = (uint16_t)n;
  } else if (!strcmp(key, "scan_debug")) {
    c->scan_debug = parse_bool(val);
  }
}

bool load() {
  apply_defaults(&s_current);

  File f = storage::fs().open("CONFIG.TXT", O_RDONLY);
  if (!f) return false;

  char line[80];
  while (true) {
    int n = f.fgets(line, sizeof(line));
    if (n <= 0) break;
    trim(line);
    if (line[0] == '\0' || line[0] == '#' || line[0] == ';') continue;

    char* eq = strchr(line, '=');
    if (!eq) continue;
    *eq = '\0';
    char* key = line;
    char* val = eq + 1;
    trim(key);
    trim(val);
    apply_kv(&s_current, key, val);
  }
  f.close();
  return true;
}

const Config& current() { return s_current; }

}  // namespace config
