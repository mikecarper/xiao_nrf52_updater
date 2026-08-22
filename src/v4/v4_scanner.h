#pragma once

#include <Arduino.h>

#include "v4_config.h"

namespace v4_scanner {

struct Target {
  uint8_t address[6];  // BLE-native / NimBLE order (LSB first)
  uint8_t address_type;
  int8_t rssi;
  char name[32];
};

bool begin();

// `preferred` is the immutable application address remembered across a
// buttonless reboot. When present, only its exact value or value+1 is accepted;
// configured name/MAC matching cannot broaden the second-leg scan.
bool find_first(Target* out, const v4_config::Config& config,
                const Target* preferred = nullptr);

void format_address(const Target& target, char out[18]);

}  // namespace v4_scanner
