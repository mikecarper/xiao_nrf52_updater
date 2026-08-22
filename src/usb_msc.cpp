#include "usb_msc.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>

#include "storage.h"

namespace {

enum class MediaOwner : uint8_t {
  kNone,
  kHost,
  kQuiescing,
  kFirmware,
};

Adafruit_USBD_MSC s_msc;
StaticSemaphore_t s_media_mutex_storage;
SemaphoreHandle_t s_media_mutex = nullptr;

volatile bool       s_usb_configured = false;
volatile bool       s_msc_enabled = false;
volatile bool       s_ready = false;
volatile bool       s_ejected = false;
volatile MediaOwner s_owner = MediaOwner::kNone;

void ensure_media_mutex() {
  if (!s_media_mutex) {
    s_media_mutex = xSemaphoreCreateMutexStatic(&s_media_mutex_storage);
  }
}

bool lock_media(TickType_t timeout) {
  return s_media_mutex && xSemaphoreTake(s_media_mutex, timeout) == pdTRUE;
}

void unlock_media() {
  xSemaphoreGive(s_media_mutex);
}

bool host_io_allowed() {
  return s_msc_enabled && s_ready && s_owner == MediaOwner::kHost;
}

int32_t on_read(uint32_t lba, void* buffer, uint32_t bufsize) {
  if (!lock_media(portMAX_DELAY)) return -1;
  bool ok = host_io_allowed() &&
            storage::flash().readBlocks(lba, static_cast<uint8_t*>(buffer),
                                        bufsize / 512);
  unlock_media();
  return ok ? static_cast<int32_t>(bufsize) : -1;
}

int32_t on_write(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  if (!lock_media(portMAX_DELAY)) return -1;
  bool ok = host_io_allowed() &&
            storage::flash().writeBlocks(lba, buffer, bufsize / 512);
  unlock_media();
  return ok ? static_cast<int32_t>(bufsize) : -1;
}

void on_flush() {
  if (!lock_media(portMAX_DELAY)) return;
  if (host_io_allowed()) {
    storage::flash().syncBlocks();
    // Drop the firmware-side FAT cache so the later ownership handoff sees
    // every directory/FAT block written by the host.
    storage::fs().cacheClear();
  }
  unlock_media();
}

bool on_start_stop(uint8_t power_condition, bool start, bool load_eject) {
  (void)power_condition;
  if (load_eject && !start) {
    if (lock_media(portMAX_DELAY)) {
      s_ejected = true;
      s_ready = false;
      if (s_owner == MediaOwner::kHost) s_owner = MediaOwner::kQuiescing;
      unlock_media();
    }

    // Make the one-shot ownership boundary visible during the eject command
    // itself. The host cannot remount while loop() drains caches and claims FAT.
    s_msc.setUnitReady(false);
  }
  return true;
}

}  // namespace

namespace usb_msc {

bool suspend_for_descriptor_update() {
  // The core creates TinyUSB before setup(), but the device task starts
  // asynchronously. Wait briefly so detach cannot race first enumeration.
  uint32_t started = millis();
  while (!TinyUSBDevice.isInitialized() &&
         static_cast<uint32_t>(millis() - started) < 100) {
    delay(1);
  }
  if (!TinyUSBDevice.isInitialized()) return false;

  TinyUSBDevice.detach();
  delay(20);
  return true;
}

void resume_after_descriptor_update(bool was_suspended) {
  if (!was_suspended) return;
  delay(20);
  TinyUSBDevice.attach();
}

bool begin() {
  ensure_media_mutex();
  if (!s_media_mutex) return false;

#if defined(BOARD_RAK4631)
  s_msc.setID("RAK", "DFU Updater", "1.0");
#else
  s_msc.setID("XIAO", "DFU Updater", "1.0");
#endif
  s_msc.setReadWriteCallback(on_read, on_write, on_flush);
  s_msc.setStartStopCallback(on_start_stop);

  uint32_t blocks = storage::flash().size() / 512;
  Serial.print("usb_msc: capacity = ");
  Serial.print(blocks);
  Serial.println(" * 512 B");

  s_msc.setCapacity(blocks, 512);
  s_msc.setUnitReady(true);

  if (lock_media(portMAX_DELAY)) {
    s_owner = MediaOwner::kHost;
    s_ready = true;
    s_ejected = false;
    unlock_media();
  }

  bool ok = s_msc.begin();
  s_msc_enabled = ok;
  if (!ok) {
    s_msc.setUnitReady(false);
    if (lock_media(portMAX_DELAY)) {
      s_owner = MediaOwner::kNone;
      s_ready = false;
      unlock_media();
    }
  }

  Serial.print("usb_msc: s_msc.begin() -> ");
  Serial.println(ok ? "ok" : "FAIL (CDC only)");
  return ok;
}

bool usb_configured() { return s_usb_configured; }
bool was_ejected() { return s_ejected; }

bool claim_after_eject() {
  if (!s_msc_enabled || !s_ejected) return false;

  // Unit readiness was already cleared synchronously in START_STOP_UNIT. Do
  // it again defensively before waiting for any in-flight callback to leave
  // the mutex.
  s_msc.setUnitReady(false);

  if (!lock_media(pdMS_TO_TICKS(2000))) return false;
  s_ready = false;
  if (s_owner == MediaOwner::kFirmware) {
    unlock_media();
    return true;
  }
  s_owner = MediaOwner::kQuiescing;

  bool synced = storage::flash().syncBlocks();
  storage::fs().cacheClear();
  if (synced) s_owner = MediaOwner::kFirmware;
  unlock_media();
  return synced;
}

bool firmware_owns_media() {
  // With no MSC interface (battery-triggered one-shot or CDC-only fallback),
  // there is no host block owner to contend with.
  if (!s_msc_enabled) return true;
  if (!lock_media(0)) return false;
  bool owns_media = s_owner == MediaOwner::kFirmware;
  unlock_media();
  return owns_media;
}

}  // namespace usb_msc

// TinyUSB calls these for USB SET_CONFIGURATION / bus removal. They describe
// the composite USB device, not whether an operating system mounted the FAT
// filesystem.
extern "C" {

void tud_mount_cb(void) {
  s_usb_configured = true;
}

void tud_umount_cb(void) {
  s_usb_configured = false;
}

}  // extern "C"
