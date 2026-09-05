#include "usb_msc.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <atomic>
#include <nrf_power.h>

#include "storage.h"

// File-static state. Accessed both from the namespace API and from the C-linkage
// TinyUSB callbacks below, so it lives at file scope rather than inside the namespace.
static Adafruit_USBD_MSC    s_msc;
static std::atomic<bool>     s_mounted(false);
static std::atomic<bool>     s_ever_mounted(false);
static std::atomic<bool>     s_host_owns_media(false);
static std::atomic<bool>     s_accept_io(true);
static std::atomic<uint32_t> s_active_io(0);
static std::atomic<bool>     s_ejected(false);
static std::atomic<bool>     s_cdc_dtr_dropped(false);

// Closing the gate and then waiting for this counter to reach zero prevents
// firmware-side SdFat access from racing an MSC callback that passed its first
// readiness check just before the eject was observed.
static bool enter_media_io() {
  if (!s_accept_io.load(std::memory_order_acquire)) return false;

  s_active_io.fetch_add(1, std::memory_order_acq_rel);
  if (s_accept_io.load(std::memory_order_acquire)) return true;

  s_active_io.fetch_sub(1, std::memory_order_release);
  return false;
}

static void leave_media_io() {
  s_active_io.fetch_sub(1, std::memory_order_release);
}

static int32_t on_read(uint32_t lba, void* buffer, uint32_t bufsize) {
  if (!enter_media_io()) return -1;
  bool const ok = storage::flash().readBlocks(lba, (uint8_t*)buffer, bufsize / 512);
  leave_media_io();
  return ok ? (int32_t)bufsize : -1;
}

static int32_t on_write(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  if (!enter_media_io()) return -1;
  bool const ok = storage::flash().writeBlocks(lba, buffer, bufsize / 512);
  leave_media_io();
  return ok ? (int32_t)bufsize : -1;
}

static void on_flush() {
  if (!enter_media_io()) return;
  // SdFat is deliberately untouched while the host owns the medium. Only the
  // raw flash library's erase cache belongs to this callback.
  storage::flash().syncBlocks();
  leave_media_io();
}

static bool on_start_stop(uint8_t power_condition, bool start, bool load_eject) {
  (void)power_condition;
  if (load_eject && !start) {
    // Do not touch QSPI or relinquish media ownership from this TinyUSB
    // callback. loop() first closes the I/O gate and drains callbacks, then
    // performs the filesystem handoff outside the USB command path.
    s_ejected.store(true, std::memory_order_release);
  }
  return true;
}

namespace usb_msc {

void begin(bool host_present) {
  // Claim ownership before enumeration. At boot there is a short window
  // between MSC begin() and tud_mount_cb(); treating VBUS as ownership closes
  // that window and prevents LOG.TXT from dirtying SdFat's cache while the
  // host starts reading or writing the same flash.
  s_host_owns_media.store(host_present, std::memory_order_release);
  s_accept_io.store(true, std::memory_order_release);

  s_msc.setID("XIAO", "DFU Updater", "1.0");
  s_msc.setReadWriteCallback(on_read, on_write, on_flush);
  // macOS / Linux / Windows all send SCSI START_STOP_UNIT(load_eject=1, start=0)
  // when the user ejects. The USB cable stays connected, so we have to hook
  // this rather than tud_umount_cb.
  s_msc.setStartStopCallback(on_start_stop);

  uint32_t blocks = storage::flash().size() / 512;
  Serial.print("usb_msc: capacity = ");
  Serial.print(blocks);
  Serial.println(" * 512 B");

  s_msc.setCapacity(blocks, 512);
  s_msc.setUnitReady(true);

  bool ok = s_msc.begin();
  Serial.print("usb_msc: s_msc.begin() -> ");
  Serial.println(ok ? "ok" : "FAIL");
}

bool is_mounted()       { return s_mounted.load(std::memory_order_acquire); }
bool was_ever_mounted() { return s_ever_mounted.load(std::memory_order_acquire); }
bool was_ejected()      { return s_ejected.load(std::memory_order_acquire); }
bool host_owns_media()  { return s_host_owns_media.load(std::memory_order_acquire); }

bool serial_dfu_requested() {
  if (!s_cdc_dtr_dropped.exchange(false, std::memory_order_acq_rel)) {
    return false;
  }
  return Serial.baud() == 1200;
}

void note_cdc_dtr_drop() {
  s_cdc_dtr_dropped.store(true, std::memory_order_release);
}

bool detach_for_serial_dfu(uint32_t timeout_ms) {
  // A CDC 1200-baud touch normally resets immediately from TinyUSB's USB
  // task. On a composite CDC+MSC device Linux may still have a SCSI URB in
  // flight, and the legacy Raspberry Pi dwc_otg host driver can wedge while
  // dequeuing it. Move the teardown into loop(): make the LUN unavailable,
  // stop new raw-flash callbacks, and wait for the one already running.
  s_msc.setUnitReady(false);
  s_accept_io.store(false, std::memory_order_release);

  uint32_t const started = millis();
  while (s_active_io.load(std::memory_order_acquire) != 0) {
    if ((uint32_t)(millis() - started) >= timeout_ms) {
      s_accept_io.store(true, std::memory_order_release);
      s_msc.setUnitReady(true);
      return false;
    }
    delay(1);
  }

  if (!storage::flash().syncBlocks()) {
    s_accept_io.store(true, std::memory_order_release);
    s_msc.setUnitReady(true);
    return false;
  }

  // Drop the device pull-up before resetting. This gives the host a normal
  // disconnect event with no MSC callback outstanding instead of making all
  // endpoints disappear in the middle of a transaction.
  if (!TinyUSBDevice.detach()) {
    s_accept_io.store(true, std::memory_order_release);
    s_msc.setUnitReady(true);
    return false;
  }
  s_mounted.store(false, std::memory_order_release);
  s_host_owns_media.store(false, std::memory_order_release);
  return true;
}

bool take_media_for_firmware(uint32_t timeout_ms) {
  // Close our callback gate before changing TinyUSB's reported LUN state. A
  // callback that raced the close either observes false or is counted below.
  s_accept_io.store(false, std::memory_order_release);
  s_msc.setUnitReady(false);

  uint32_t const started = millis();
  while (s_active_io.load(std::memory_order_acquire) != 0) {
    if ((uint32_t)(millis() - started) >= timeout_ms) return false;
    delay(1);
  }

  if (!storage::refresh_after_host_write()) return false;

  s_mounted.store(false, std::memory_order_release);
  s_host_owns_media.store(false, std::memory_order_release);
  return true;
}

}  // namespace usb_msc

// TinyUSB device-state callbacks (C linkage, weak in the SDK).
extern "C" {

void tud_mount_cb(void) {
  s_mounted.store(true, std::memory_order_release);
  s_ever_mounted.store(true, std::memory_order_release);
  s_host_owns_media.store(true, std::memory_order_release);
}

void tud_umount_cb(void) {
  s_mounted.store(false, std::memory_order_release);
  // A USB bus reset also produces an unmount callback while VBUS and host
  // ownership remain. Only a real cable removal makes firmware access safe.
  if ((NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) == 0) {
    s_host_owns_media.store(false, std::memory_order_release);
  }
}

}  // extern "C"
