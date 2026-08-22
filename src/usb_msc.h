#pragma once

namespace usb_msc {

// Hold the TinyUSB pull-up down while optional interfaces are registered, then
// reattach with one stable descriptor set. If storage failed, begin() is not
// called and the reattached device contains CDC only.
bool suspend_for_descriptor_update();
void resume_after_descriptor_update(bool was_suspended);

// Register the storage-backed MSC interface. Returns false if descriptor
// registration failed; callers should leave USB in CDC-only mode.
bool begin();

// True while the composite USB device is configured by a host. This is a USB
// bus state, not proof that the FAT volume is mounted by the operating system.
bool usb_configured();

// Latched by SCSI START_STOP_UNIT(load_eject=1, start=0).
bool was_ejected();

// Permanently make the LUN not-ready, drain block I/O, sync the flash, clear
// firmware FAT caches, and transfer this boot's media ownership to firmware.
bool claim_after_eject();

// Logger/filesystem code may write only when this and storage::ready() are
// both true.
bool firmware_owns_media();

}  // namespace usb_msc
