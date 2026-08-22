#pragma once

#include <Arduino.h>

#include "ble_scanner.h"
#include "config.h"
#include "dfu_types.h"
#include "firmware_zip.h"

namespace dfu_legacy {

// Invoked from inside run() during the firmware stream. `pct` is 0..100.
// Called frequently — keep the body fast and avoid blocking I/O.
typedef void (*ProgressCb)(uint8_t pct);

void set_progress_callback(ProgressCb cb);

// Run the legacy DFU flow against `target`, sourcing the firmware from the
// already-parsed `bundle`. Blocking, intended to be called from loop() after
// the zip + scan steps. `cleanup` tells retry policy whether the peer reset or
// may still hold partial legacy-DFU state.
RunResult run(const ble_scanner::Target& target,
              const firmware_zip::Parsed& bundle,
              const config::Config& cfg);

}  // namespace dfu_legacy
