#pragma once

#include <stdint.h>

namespace usb_msc {

// Start exposing QSPI as a USB disk. `host_present` must reflect VBUS at
// startup so firmware-side filesystem writes are blocked before the host can
// issue its first SCSI command.
void begin(bool host_present);

// Has the host ever mounted the drive this session?
bool was_ever_mounted();

// Is the host currently mounted?
bool is_mounted();

// True while the USB host owns the raw block device. Firmware-side SdFat
// writes must be suppressed in this interval; sharing the FAT through both
// APIs can otherwise restore stale directory metadata over a host copy.
bool host_owns_media();

// True after the host has issued a SCSI Start-Stop-Unit with load_eject=1.
// This is what macOS / Finder send on Eject (the USB device stays plugged
// in, so tud_umount_cb does NOT fire). Once latched, stays true.
bool was_ejected();

// Atomically stop accepting raw MSC I/O, wait for any callback that already
// entered to finish, flush the flash write cache, and invalidate SdFat's stale
// view. On success firmware owns the medium and may safely open files.
bool take_media_for_firmware(uint32_t timeout_ms = 1000);

}  // namespace usb_msc
