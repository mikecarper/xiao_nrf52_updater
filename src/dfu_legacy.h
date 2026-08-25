#pragma once

#include <Arduino.h>

#include "ble_scanner.h"
#include "config.h"
#include "firmware_zip.h"

namespace dfu_legacy {

// Invoked from inside run() during the firmware stream. `pct` is 0..100.
// Called frequently - keep the body fast and avoid blocking I/O.
typedef void (*ProgressCb)(uint8_t pct);

void set_progress_callback(ProgressCb cb);

enum class Result {
  kOk,
  kButtonlessTriggered,   // Peer was in app mode; we kicked it into bootloader.
                          // Caller should wait ~2 s and rescan.
  kConnectFailed,
  kServiceMissing,
  kCharMissing,
  kRevisionUnreadable,
  kDisconnectedEarly,
  kTimeout,
  kRemoteError,
  kFsError,
};

// Run the legacy DFU flow against `target`, sourcing the firmware from the
// already-parsed `bundle`. Blocking, intended to be called from loop() after
// the zip + scan steps. Returns Result::kOk on full success.
Result run(const ble_scanner::Target& target,
           const firmware_zip::Parsed& bundle,
           const config::Config& cfg);

}  // namespace dfu_legacy
