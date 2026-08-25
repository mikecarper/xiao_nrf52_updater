#include "ble_scanner.h"

#include "logger.h"

namespace ble_scanner {

// Nordic Legacy DFU service UUID (LSB first because BLEUuid takes a 16-byte
// LE array): 00001530-1212-EFDE-1523-785FEABCD123
static const uint8_t kLegacyDfuUuid128[16] = {
  0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
  0xDE, 0xEF, 0x12, 0x12, 0x30, 0x15, 0x00, 0x00,
};

static volatile bool         s_found       = false;
static Target                s_match;
static const char*           s_name_filter = nullptr;   // nullptr or "" = accept any
static int8_t                s_min_rssi    = -127;      // -127 = no minimum
static const ble_gap_addr_t* s_prefer_mac  = nullptr;   // optional MAC / MAC+1 fast-path
static const ble_gap_addr_t* s_target_mac  = nullptr;   // strict CONFIG.TXT ble_mac target
static bool                  s_debug       = false;     // verbose per-ad log

void set_debug(bool on) { s_debug = on; }

void set_tx_power(int8_t tx_power_dbm) {
  if (!Bluefruit.setTxPower(tx_power_dbm)) {
    Bluefruit.setTxPower(0);
  }
}

// Pipe-delimited substring match: returns true if `name` contains any of
// the '|'-separated tokens in `filter` (after trimming each). Empty tokens
// are ignored. An empty/null filter returns false here; the caller treats
// "no filter" as a separate code path.
static bool name_matches(const char* name, const char* filter) {
  if (!name || !filter || !*filter) return false;

  char buf[24];
  size_t n = strnlen(filter, sizeof(buf) - 1);
  memcpy(buf, filter, n);
  buf[n] = '\0';

  char* save = nullptr;
  for (char* tok = strtok_r(buf, "|", &save); tok; tok = strtok_r(nullptr, "|", &save)) {
    while (*tok == ' ' || *tok == '\t') tok++;
    char* end = tok + strlen(tok);
    while (end > tok && (end[-1] == ' ' || end[-1] == '\t')) *--end = '\0';
    if (*tok && strstr(name, tok)) return true;
  }
  return false;
}

static bool mac_equal(const ble_gap_addr_t& addr, const ble_gap_addr_t* ref) {
  return ref && memcmp(addr.addr, ref->addr, 6) == 0;
}

// True if `addr` equals `*ref + 1` (with carry across all 6 bytes). Nordic
// Legacy DFU bootloaders commonly increment the application address.
static bool mac_is_plus_one(const ble_gap_addr_t& addr, const ble_gap_addr_t* ref) {
  if (!ref) return false;
  ble_gap_addr_t plus_one = *ref;
  for (int i = 0; i < 6; i++) {
    plus_one.addr[i]++;
    if (plus_one.addr[i] != 0) break;   // no further carry needed
  }
  return memcmp(addr.addr, plus_one.addr, 6) == 0;
}

static bool mac_match_or_plus_one(const ble_gap_addr_t& addr,
                                  const ble_gap_addr_t* ref) {
  return mac_equal(addr, ref) || mac_is_plus_one(addr, ref);
}

// One-line summary of an ad. Caller passes the reason; we de-dupe per MAC so
// dense BLE environments don't flood the log. Gated on `s_debug`.
static void log_seen(const ble_gap_evt_adv_report_t* report, const char* name,
                     const char* reason) {
  if (!s_debug) return;

  static uint8_t  seen_macs[8][6];
  static char     seen_reasons[8][8];
  static uint8_t  seen_count = 0;

  // Skip if we've already logged this MAC with the same reason.
  for (uint8_t i = 0; i < seen_count; i++) {
    if (memcmp(seen_macs[i], report->peer_addr.addr, 6) == 0 &&
        strncmp(seen_reasons[i], reason, sizeof(seen_reasons[0])) == 0) {
      return;
    }
  }
  if (seen_count < 8) {
    memcpy(seen_macs[seen_count], report->peer_addr.addr, 6);
    snprintf(seen_reasons[seen_count], sizeof(seen_reasons[0]), "%s", reason);
    seen_count++;
  }

  logger::log("scan: %s %02X:%02X:%02X:%02X:%02X:%02X rssi=%d name='%s'",
              reason,
              report->peer_addr.addr[5], report->peer_addr.addr[4],
              report->peer_addr.addr[3], report->peer_addr.addr[2],
              report->peer_addr.addr[1], report->peer_addr.addr[0],
              report->rssi, name);
}

static void scan_cb(ble_gap_evt_adv_report_t* report) {
  if (s_found) {
    Bluefruit.Scanner.resume();
    return;
  }

  // Always parse the name first so every rejection log line is informative.
  Target candidate{};
  candidate.addr = report->peer_addr;
  candidate.rssi = report->rssi;
  Bluefruit.Scanner.parseReportByType(
      report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
      (uint8_t*)candidate.name, sizeof(candidate.name) - 1);
  if (candidate.name[0] == '\0') {
    Bluefruit.Scanner.parseReportByType(
        report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
        (uint8_t*)candidate.name, sizeof(candidate.name) - 1);
  }
  candidate.name[sizeof(candidate.name) - 1] = '\0';

  // Mode evidence carried into the connection phase. OTAFIX bootloaders put
  // the Legacy DFU service UUID in their advertising data. MeshCore's
  // application-side BLEDfu service intentionally advertises only its name,
  // even though the same service becomes visible after connection.
  BLEUuid legacy_dfu_uuid(kLegacyDfuUuid128);
  candidate.advertises_legacy_dfu =
    Bluefruit.Scanner.checkReportForUuid(report, legacy_dfu_uuid);

  // RSSI threshold.
  if (report->rssi < s_min_rssi) {
    log_seen(report, candidate.name, "weak");
    Bluefruit.Scanner.resume();
    return;
  }

  // Filter strategy. Accept the ad if any of these holds:
  //
  // 1. Strict CONFIG.TXT ble_mac mode:
  //    When no ble_name is configured and target_mac is set, accept only
  //    the configured MAC or MAC+1. Do not fall back to random DFU UUID devices.
  //
  // 2. Existing prefer_mac fast-path:
  //    Used after buttonless DFU trigger, accepts app MAC or MAC+1.
  //
  // 3. Existing ble_name filter.
  //
  // 4. Existing fallback:
  //    When neither ble_name nor ble_mac is configured, accept Legacy DFU UUID.

  bool has_name = (s_name_filter && s_name_filter[0] != '\0');

  // Strict MAC mode from CONFIG.TXT.
  // This is intentionally before the old UUID fallback.
  if (!has_name && s_target_mac) {
    bool const target_mac_ok = mac_match_or_plus_one(report->peer_addr, s_target_mac);
    bool const prefer_mac_ok = mac_match_or_plus_one(report->peer_addr, s_prefer_mac);

    if (!(target_mac_ok || prefer_mac_ok)) {
      log_seen(report, candidate.name, "mac?");
      Bluefruit.Scanner.resume();
      return;
    }

    candidate.mac_plus_one = mac_is_plus_one(report->peer_addr, s_target_mac) ||
                             mac_is_plus_one(report->peer_addr, s_prefer_mac);
    s_match = candidate;
    s_found = true;
    Bluefruit.Scanner.stop();
    return;
  }

  // Original behavior follows.
  bool mac_ok = mac_match_or_plus_one(report->peer_addr, s_prefer_mac);
  bool name_ok = has_name && name_matches(candidate.name, s_name_filter);
  bool need_uuid = !mac_ok && !has_name;
  bool uuid_ok = need_uuid && candidate.advertises_legacy_dfu;

  if (!(mac_ok || name_ok || uuid_ok)) {
    const char* reason = mac_ok ? "mac" : (has_name ? "name?" : "uuid?");
    log_seen(report, candidate.name, reason);
    Bluefruit.Scanner.resume();
    return;
  }

  candidate.mac_plus_one = mac_is_plus_one(report->peer_addr, s_prefer_mac);
  s_match  = candidate;
  s_found  = true;
  Bluefruit.Scanner.stop();
}

void begin(int8_t tx_power_dbm) {
  // The SoftDevice's ATT buffer cap is fixed at Bluefruit.begin() time.
  // Default is 23 (MTU=23). To make later MTU-exchange requests effective,
  // we have to pre-allocate buffer space for the maximum we ever want:
  //   mtu_max=247        : allow up to 247 B per ATT packet
  //   event_len=6        : 6 LL units per connection event (more bandwidth)
  //   hvn_qsize=2        : modest notification queue
  //   wrcmd_qsize=4      : 4 outstanding WRITE_CMD packets (streaming throughput)
  // These are applied to the central role; we don't run as peripheral.
  Bluefruit.configCentralConn(247, 6, 2, 4);

  // 0 peripheral, 1 central - we're a pure DFU client.
  Bluefruit.begin(0, 1);
  Bluefruit.setName("XIAO DFU updater");

  // TX power. Allowed values on nRF52840: -40, -20, -16, -12, -8, -4, 0,
  // 2, 3, 4, 5, 6, 7, 8 (max). setTxPower() returns false and leaves the
  // previous (default 0 dBm) value if the requested level isn't in the list.
  set_tx_power(tx_power_dbm);

  Bluefruit.Scanner.setRxCallback(scan_cb);
  Bluefruit.Scanner.restartOnDisconnect(false);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.setInterval(160, 80);  // 100 ms window / 200 ms interval
}

bool find_first(Target* out, uint32_t timeout_ms, const char* name_filter,
                int8_t min_rssi, const ble_gap_addr_t* prefer_mac, const ble_gap_addr_t* target_mac) {                    
  s_found       = false;
  s_name_filter = name_filter;
  s_min_rssi    = min_rssi;
  s_prefer_mac  = prefer_mac;
  s_target_mac = target_mac;
  memset(&s_match, 0, sizeof(s_match));

  Bluefruit.Scanner.start(0);  // 0 = scan until told to stop

  // timeout_ms == 0 means "scan forever" (drone use). Otherwise we honour
  // the deadline.
  uint32_t deadline = millis() + timeout_ms;
  while (!s_found) {
    if (timeout_ms != 0 && (int32_t)(deadline - millis()) <= 0) break;
    delay(50);
  }

  Bluefruit.Scanner.stop();

  if (!s_found) return false;
  *out = s_match;
  return true;
}

}  // namespace ble_scanner
