#pragma once

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

// Disable / re-enable the MSC LUN. We disable while running DFU so the host
// can't write to the flash mid-operation.
void set_ready(bool ready);

}  // namespace usb_msc
