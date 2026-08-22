#include "v4_scanner.h"

#include <NimBLEDevice.h>

#include "v4_log.h"
#include "v4_scan_policy.h"

namespace v4_scanner {

namespace {

portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
volatile bool s_found = false;
Target s_match = {};
const v4_config::Config* s_config = nullptr;
const Target* s_preferred = nullptr;
volatile uint32_t s_rejected_weak = 0;
volatile uint32_t s_rejected_target = 0;

bool name_matches(const char* name, const char* filter) {
  if (!name || !filter || !*filter) return false;
  char copy[32];
  snprintf(copy, sizeof(copy), "%s", filter);
  char* save = nullptr;
  for (char* token = strtok_r(copy, "|", &save); token;
       token = strtok_r(nullptr, "|", &save)) {
    while (*token == ' ' || *token == '\t') ++token;
    char* end = token + strlen(token);
    while (end > token && (end[-1] == ' ' || end[-1] == '\t')) *--end = '\0';
    if (*token && strstr(name, token)) return true;
  }
  return false;
}

class ScanCallbacks final : public NimBLEAdvertisedDeviceCallbacks {
 public:
  void onResult(NimBLEAdvertisedDevice* advertised) override {
    if (!advertised || s_found || !s_config) return;
    int rssi = advertised->getRSSI();
    if (rssi < s_config->min_rssi) {
      ++s_rejected_weak;
      return;
    }

    Target candidate = {};
    NimBLEAddress address = advertised->getAddress();
    memcpy(candidate.address, address.getNative(), sizeof(candidate.address));
    candidate.address_type = address.getType();
    candidate.rssi = static_cast<int8_t>(rssi);
    if (advertised->haveName()) {
      std::string name = advertised->getName();
      snprintf(candidate.name, sizeof(candidate.name), "%s", name.c_str());
    }

    bool configured_match = false;
    if (s_config->ble_name[0] != '\0') {
      configured_match = name_matches(candidate.name, s_config->ble_name);
    } else if (s_config->ble_mac_set) {
      configured_match = v4_scan_policy::address_matches_or_plus_one(
          candidate.address, s_config->ble_mac);
    }
    bool accepted = v4_scan_policy::accept_candidate(
        candidate.address, s_preferred ? s_preferred->address : nullptr,
        configured_match);
    if (!accepted) {
      ++s_rejected_target;
      return;
    }

    portENTER_CRITICAL(&s_lock);
    if (!s_found) {
      s_match = candidate;
      s_found = true;
    }
    portEXIT_CRITICAL(&s_lock);
  }
};

ScanCallbacks s_callbacks;

}  // namespace

bool begin() {
  NimBLEDevice::init("Heltec V4 Legacy DFU");
  if (NimBLEDevice::setMTU(247) != 0) return false;
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEScan* scan = NimBLEDevice::getScan();
  if (!scan) return false;
  // Do not let the controller's finite duplicate cache hide a target which
  // changes its advertisement payload or reappears after app->boot reboot.
  // The callback deletes every stored result immediately (maxResults=0).
  scan->setAdvertisedDeviceCallbacks(&s_callbacks, true);
  scan->setActiveScan(true);
  scan->setInterval(200);
  scan->setWindow(100);
  scan->setMaxResults(0);
  return true;
}

bool find_first(Target* out, const v4_config::Config& config,
                const Target* preferred) {
  if (!out || (config.ble_name[0] == '\0' && !config.ble_mac_set)) return false;
  NimBLEScan* scan = NimBLEDevice::getScan();
  if (!scan) return false;

  portENTER_CRITICAL(&s_lock);
  s_found = false;
  memset(&s_match, 0, sizeof(s_match));
  portEXIT_CRITICAL(&s_lock);
  s_config = &config;
  s_preferred = preferred;
  s_rejected_weak = 0;
  s_rejected_target = 0;
  uint32_t started_at = millis();
  uint32_t timeout_ms = static_cast<uint32_t>(config.scan_timeout) * 1000u;
  while (!s_found &&
         (timeout_ms == 0 ||
          static_cast<uint32_t>(millis() - started_at) < timeout_ms)) {
    // A short finite scan matters for active scanning: NimBLE defers an
    // ADV_IND callback while waiting for a scan response. At scan completion
    // it flushes advertisers that never sent one. An infinite scan would hide
    // such a bootloader forever. Use the blocking overload and always let the
    // finite scan complete: stop()/clearResults() while onResult is returning
    // races NimBLE 1.4.x's callback-owned advertised-device object.
    scan->start(2, false);
    delay(10);
  }

  bool found = false;
  portENTER_CRITICAL(&s_lock);
  if (s_found) {
    *out = s_match;
    found = true;
  }
  portEXIT_CRITICAL(&s_lock);
  s_config = nullptr;
  s_preferred = nullptr;

  if (!found && config.scan_debug) {
    v4_log::line("scan: timeout weak=%lu target-mismatch=%lu",
                 static_cast<unsigned long>(s_rejected_weak),
                 static_cast<unsigned long>(s_rejected_target));
  }
  return found;
}

void format_address(const Target& target, char out[18]) {
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
           target.address[5], target.address[4], target.address[3],
           target.address[2], target.address[1], target.address[0]);
}

}  // namespace v4_scanner
