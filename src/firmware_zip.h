#pragma once

#include <Arduino.h>

#include "dfu_image_layout.h"
#include "zip_reader.h"

namespace firmware_zip {

// Bit flags that exactly match the Nordic Legacy DFU "Start" opcode's mode
// byte (see LegacyDfuImpl.java). When more than one is set, the firmware is
// a combined image.
enum FwType : uint8_t {
  TYPE_SOFTDEVICE = dfu_image_layout::kTypeSoftdevice,
  TYPE_BOOTLOADER = dfu_image_layout::kTypeBootloader,
  TYPE_APPLICATION = dfu_image_layout::kTypeApplication,
};

struct Parsed {
  uint8_t            type;        // bitmask of FwType
  zip_reader::Entry  bin;         // firmware image (concatenated for SD+BL)
  zip_reader::Entry  dat;         // init packet
  uint32_t           sd_size;     // Legacy DFU size tuple, SoftDevice field
  uint32_t           bl_size;     // Legacy DFU size tuple, Bootloader field
  uint32_t           app_size;    // Legacy DFU size tuple, Application field
};

// Open the zip at `zip_path`, parse manifest.json, and resolve the bin/dat
// entries described inside it. On success the zip stays open via zip_reader;
// the caller is responsible for zip_reader::close() once it's done streaming
// firmware bytes.
bool parse(const char* zip_path, Parsed* out, char* err, size_t err_len);

}  // namespace firmware_zip
