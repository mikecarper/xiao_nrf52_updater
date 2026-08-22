#pragma once

#include <stdint.h>

namespace dfu_legacy {

enum class Result : uint8_t {
  kOk,
  kButtonlessTriggered,  // App was kicked to bootloader; caller should rescan.
  kConnectFailed,
  kServiceMissing,
  kCharMissing,
  kDisconnectedEarly,
  kTimeout,
  kRemoteError,
  kFsError,
  kButtonlessLimit,  // App still present after the one permitted transition.
};

// Describes how a failed connected session was left. Only a confirmed peer
// reset permits an immediate retry; a lost/forced link may leave legacy DFU
// state alive until the bootloader's inactivity watchdog expires.
enum class CleanupOutcome : uint8_t {
  kNotNeeded,
  kResetObserved,
  kLinkAlreadyLost,
  kForcedDisconnect,
};

struct RunResult {
  Result result;
  CleanupOutcome cleanup;
};

}  // namespace dfu_legacy
