#pragma once

#include <Adafruit_SPIFlash.h>
#include <SdFat.h>

namespace storage {

bool begin();
bool ready();

Adafruit_SPIFlash& flash();
FatVolume&         fs();

// Scan the FAT root directory for exactly one `*.zip` file (case-insensitive).
// On success copies the 8.3 / long filename into `out` and returns the count
// of zips found. The caller should treat a return value other than 1 as an
// error (none or ambiguous).
int find_single_zip(char* out, size_t out_len);

// Delete the named file from the root. Used to clear the firmware bundle
// after a successful update.
bool delete_file(const char* name);

}  // namespace storage
