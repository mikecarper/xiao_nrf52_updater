#pragma once

#include <Arduino.h>

#include "v4_zip.h"

namespace v4_bundle {

struct Parsed {
  uint8_t type;
  v4_zip::Entry bin;
  v4_zip::Entry dat;
  uint32_t sd_size;
  uint32_t bl_size;
  uint32_t app_size;
};

// On success the ZIP remains open for streaming.  Call v4_zip::close().
bool parse(const char* path, Parsed* out, char* err, size_t err_len);

}  // namespace v4_bundle
