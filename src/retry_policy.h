#pragma once

#include <stdint.h>

#include "dfu_types.h"

namespace retry_policy {

enum class CooldownClass : uint8_t {
  kStop,
  kShort,
  kWedge,
};

inline CooldownClass classify(const dfu_legacy::RunResult& run) {
  using dfu_legacy::CleanupOutcome;
  using dfu_legacy::Result;

  // A local media failure cannot improve by reconnecting to the target.
  if (run.result == Result::kFsError ||
      run.result == Result::kButtonlessLimit) {
    return CooldownClass::kStop;
  }

  // These identify an incompatible/wrong advertisement, not a wedged target.
  // Rescanning is useful, but waiting for a DFU watchdog is not.
  if (run.result == Result::kServiceMissing ||
      run.result == Result::kCharMissing) {
    return CooldownClass::kShort;
  }

  if (run.cleanup == CleanupOutcome::kLinkAlreadyLost ||
      run.cleanup == CleanupOutcome::kForcedDisconnect) {
    return CooldownClass::kWedge;
  }

  return CooldownClass::kShort;
}

}  // namespace retry_policy
