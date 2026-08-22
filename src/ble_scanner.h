#pragma once

#include <Arduino.h>
#include <bluefruit.h>

namespace ble_scanner {

struct Target {
  ble_gap_addr_t addr;        // 48-bit MAC + type, ready to pass to Central.connect()
  int8_t         rssi;
  char           name[24];    // empty string if device didn't advertise a name
};

// Initialize the BLE central stack. Must be called once before scan() and the
// later DFU client work. Safe to call after USB is up.
//
// `tx_power_dbm` is applied via Bluefruit.setTxPower(); invalid values
// (outside the nRF52840's allowed list) are silently rejected by Bluefruit
// and the default (0 dBm) stays in effect.
void begin(int8_t tx_power_dbm = 0);

// Reapply settings that CONFIG.TXT may change during the host-copy/eject
// workflow. Safe after begin() and before scanning.
void apply_runtime_settings(int8_t tx_power_dbm, bool debug);

// When true, every rejected advertisement is logged (with reason and MAC).
// Off by default — turn on to diagnose "target isn't being picked up".
void set_debug(bool on);

// Scan for any device advertising the Nordic Legacy DFU service UUID
// (00001530-1212-EFDE-1523-785FEABCD123). Blocks until either a match is
// found or `timeout_ms` elapses (0 = no timeout). Returns true on match.
//
// `name_filter`, when non-empty, is matched as a substring against the
// advertised Complete / Short Local Name. The filter can hold multiple
// substrings separated by '|' (with optional surrounding whitespace), and
// any one matching accepts the ad — useful when an app and its bootloader
// advertise under different names.
//
// `min_rssi` rejects ads weaker than the threshold (in dBm, negative). Pass
// -127 for no signal-strength filter.
//
// `prefer_mac`, when non-null, makes the scanner additionally accept ads
// whose MAC equals `*prefer_mac` or `*prefer_mac + 1` (Nordic convention
// for the app→bootloader transition). The name filter still applies on top
// — matching either side accepts the ad. Used right after a buttonless
// trigger so we don't lose the target when the bootloader advertises under
// a different name.
bool find_first(Target* out, uint32_t timeout_ms,
                const char* name_filter = nullptr,
                int8_t min_rssi = -127,
                const ble_gap_addr_t* prefer_mac = nullptr,
                const ble_gap_addr_t* target_mac = nullptr
);

}  // namespace ble_scanner
