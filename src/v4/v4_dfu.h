#pragma once

#include <Arduino.h>

#include "dfu_types.h"
#include "v4_bundle.h"
#include "v4_config.h"
#include "v4_scanner.h"

namespace v4_dfu {

using dfu_legacy::RunResult;

typedef void (*ProgressCallback)(uint8_t percent);

void set_progress_callback(ProgressCallback callback);
RunResult run(const v4_scanner::Target& target,
              const v4_bundle::Parsed& bundle,
              const v4_config::Config& config,
              bool allow_buttonless = true);

}  // namespace v4_dfu
