#pragma once

#include <Arduino.h>

#include "zip_reader.h"

namespace firmware_zip {

// Bit flags that exactly match the Nordic Legacy DFU "Start" opcode's mode
// byte (see LegacyDfuImpl.java). When more than one is set, the firmware is
// a combined image.
enum FwType : uint8_t {
  TYPE_SOFTDEVICE = 0x01,
  TYPE_BOOTLOADER = 0x02,
  TYPE_APPLICATION = 0x04,
};

struct Parsed {
  uint8_t            type;        // bitmask of FwType
  zip_reader::Entry  bin;         // firmware image (concatenated for SD+BL)
  zip_reader::Entry  dat;         // init packet
  uint32_t           sd_size;     // set for SD-only and SD+BL bundles
  uint32_t           bl_size;     // set for BL-only and SD+BL bundles
};

// Open the zip at `zip_path`, parse manifest.json, and resolve the bin/dat
// entries described inside it. On success the zip stays open via zip_reader;
// the caller is responsible for zip_reader::close() once it's done streaming
// firmware bytes.
bool parse(const char* zip_path, Parsed* out, char* err, size_t err_len);

}  // namespace firmware_zip
