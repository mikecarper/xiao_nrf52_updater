#pragma once

#include <Arduino.h>

namespace config {

struct Config {
  // Substring filter for advertised BLE name. Empty = accept any peer that
  // exposes the Legacy DFU service.
  char     ble_name[24];

  // Optional fixed BLE MAC target from CONFIG.TXT.
  // Used only when ble_name is empty.
  bool ble_mac_set;
  uint8_t ble_mac[6];

  // Packet Receipt Notification cadence (writes between PRN callbacks).
  // 10 is safe for SDK 6.0 bootloaders; modern ones tolerate up to ~32.
  // 0 disables PRNs entirely (faster but no flow control — risky).
  uint16_t prn;

  // Negotiate ATT MTU up to 247 B after connect. When false, falls back to
  // the default MTU of 23 (20 B payload per write). Older bootloaders may
  // not honour MTU exchange — set to false if a target stalls.
  bool     high_mtu;

  // Number of DFU attempts before giving up.
  uint8_t  retries;

  // Minimum RSSI (dBm, negative) we'll accept from a target. Ads weaker than
  // this are ignored during the scan phase. Default -127 = no minimum.
  // Useful when this rig is mounted on a drone and we want to refuse flashing
  // a peer that's too far / signal too weak to reliably stream to.
  int8_t   min_rssi;

  // Cooldown between failed attempts, in seconds. The DFU bootloader needs
  // a moment to settle after a reset before it will accept another START_DFU,
  // and slamming retries returns INVALID_STATE.
  uint16_t retry_cooldown;

  // Extended cooldown used after a bootloader protocol failure (response
  // timeout, remote error, or link drop after START_DFU). SDK 11-era
  // bootloaders that wedge mid-DFU only unstick when their internal inactivity
  // watchdog fires — usually 60–120 s. Discovery, classification, and
  // buttonless-transition failures still use the short `retry_cooldown`.
  uint16_t wedge_cooldown;

  // BLE transmit power in dBm. nRF52840 valid values:
  //   -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8
  // Default 0. Crank to 8 for max range (drone use); SoftDevice will reject
  // anything not in the allowed list and we fall back to 0.
  int8_t   tx_power;

  // Per-scan timeout in seconds. 0 = scan forever (never give up) — the
  // default, intended for drone use where the target might take minutes
  // to come into range. Set non-zero to cap the wait.
  uint16_t scan_timeout;

  // If true, log every rejected advertisement (weak signal / name mismatch /
  // UUID mismatch). Useful when diagnosing why a target isn't being picked up.
  // Off by default to keep the field log quiet.
  bool     scan_debug;
};

// Set the working config from defaults, then overlay anything found in
// /CONFIG.TXT on the drive. Always succeeds; missing file = defaults only.
// Returns true if CONFIG.TXT was loaded, false if defaults were used.
bool load();

const Config& current();

}  // namespace config
