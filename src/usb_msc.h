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

// The stock TinyUSB 1200-baud callback resets the MCU immediately from the
// USB task. With a composite CDC+MSC device that can strand a host SCSI URB.
// This project overrides the weak callback and defers the reset to loop(),
// where the medium can be quiesced first.
bool serial_dfu_requested();

// Internal handoff used by the strong TinyUSB callback override in
// cdc_reset_hook.cpp. Public only because the callback must live in a
// translation unit that has not inherited TinyUSB's weak declaration.
void note_cdc_dtr_drop();

// Stop new MSC I/O, drain and flush any callback already in progress, then
// electrically detach the complete USB device. Returns false without
// detaching if the medium cannot be made safe before the timeout.
bool detach_for_serial_dfu(uint32_t timeout_ms = 2000);

// Atomically stop accepting raw MSC I/O, wait for any callback that already
// entered to finish, flush the flash write cache, and invalidate SdFat's stale
// view. On success firmware owns the medium and may safely open files.
bool take_media_for_firmware(uint32_t timeout_ms = 1000);

}  // namespace usb_msc
