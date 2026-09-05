#pragma once

#include <Adafruit_SPIFlash.h>
#include <SdFat.h>

namespace storage {

bool begin();

Adafruit_SPIFlash& flash();
FatVolume&         fs();

// Flush firmware-side filesystem state before a USB host is allowed to access
// the same flash as a raw block device.
bool prepare_for_host_access();

// Finish raw MSC writes and invalidate SdFat's cached FAT/root-directory
// sectors before firmware-side file access. The USB host and SdFat use the
// same flash through different paths, so an eject alone does not make an old
// in-RAM directory cache coherent.
bool refresh_after_host_write();

// Scan the FAT root directory for exactly one `*.zip` file (case-insensitive).
// On success copies the 8.3 / long filename into `out` and returns the count
// of zips found. The caller should treat a return value other than 1 as an
// error (none or ambiguous).
int find_single_zip(char* out, size_t out_len);

// Delete the named file from the root. Used to clear the firmware bundle
// after a successful update.
bool delete_file(const char* name);

}  // namespace storage
