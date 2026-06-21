#include <Arduino.h>
// IMPORTANT: must be in the same TU as setup()/Serial so the BSP swaps Serial
// to TinyUSB CDC. Without this, USB-MSC and CDC silently fail to enumerate.
#include <Adafruit_TinyUSB.h>
#include <nrf_power.h>

#include "ble_scanner.h"
#include "config.h"
#include "dfu_legacy.h"
#include "firmware_zip.h"
#include "logger.h"
#include "storage.h"
#include "usb_msc.h"
#include "zip_reader.h"

// ---------------------------------------------------------------------------
// LEDs
// ---------------------------------------------------------------------------
// Indicator scheme:
//   BLUE  slow blink     = idle, waiting for the host
//   BLUE  solid          = host has the drive mounted
//   GREEN fast blink     = DFU running
//   GREEN solid          = DFU succeeded
//   RED   solid          = DFU failed (XIAO; has 3 LEDs)
//   GREEN+BLUE alternate = DFU failed (RAK4631; no red LED)

static void led_set(uint8_t pin, bool on) { digitalWrite(pin, on ? LED_STATE_ON : !LED_STATE_ON); }

static void leds_off() {
#if !defined(BOARD_RAK4631)
  pinMode(LED_RED,   OUTPUT); led_set(LED_RED,   false);
#endif
  pinMode(LED_GREEN, OUTPUT); led_set(LED_GREEN, false);
  pinMode(LED_BLUE,  OUTPUT); led_set(LED_BLUE,  false);
}

// ---------------------------------------------------------------------------
// Boot conditions
// ---------------------------------------------------------------------------
// VBUS detect — bit set when USB is supplying power. If absent, the board is
// running off the BAT+/BAT- pads and we should treat any zip on the drive as
// "ready to flash immediately", matching the requirements' physical-unplug
// trigger.
static bool vbus_present() {
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum class State : uint8_t {
  kIdle,          // waiting for trigger
  kRunning,       // DFU sequence in progress (we block in here)
  kDoneOk,        // success — green solid
  kDoneFail,      // failed after retries — red solid
};

static State                  s_state     = State::kIdle;
static bool                   s_storage_ok = false;
static bool                   s_armed_boot = false;  // VBUS-absent trigger fired
static firmware_zip::Parsed   s_bundle;
static volatile uint8_t       s_progress_pct = 0;     // 0..100, updated from dfu_legacy

static void leds_tick(uint32_t now);  // forward decl

static void on_dfu_progress(uint8_t pct) {
  s_progress_pct = pct;
  leds_tick(millis());
}

// Scan for a target advertising the Legacy DFU service (or matching the
// configured ble_name). Returns true once found, false only when an explicit
// scan_timeout was configured and exceeded. With the default cfg.scan_timeout=0
// this blocks indefinitely — appropriate for drone use where the target may
// take minutes to come into range.
//
// `prefer_mac` is non-null right after a buttonless trigger: it makes the
// scanner also accept ads from the same MAC or MAC+1, so we don't lose the
// peer when the bootloader advertises under a different name.
static bool scan_for_target(ble_scanner::Target* out,
                            const ble_gap_addr_t* prefer_mac = nullptr) {
  const config::Config& cfg = config::current();
  uint32_t timeout_ms = (uint32_t)cfg.scan_timeout * 1000;
  if (cfg.scan_timeout == 0) {
    logger::log("scan: looking for DFU target (no timeout)%s...",
                prefer_mac ? ", MAC+1 fallback armed" : "");
  } else {
    logger::log("scan: looking for DFU target (%u s timeout)%s...",
                cfg.scan_timeout, prefer_mac ? ", MAC+1 fallback armed" : "");
  }
  if (!ble_scanner::find_first(out, timeout_ms, cfg.ble_name, cfg.min_rssi, prefer_mac)) {
    logger::log("scan: timed out, no target found");
    return false;
  }
  logger::log("scan: found %02X:%02X:%02X:%02X:%02X:%02X  rssi=%d  '%s'",
              out->addr.addr[5], out->addr.addr[4], out->addr.addr[3],
              out->addr.addr[2], out->addr.addr[1], out->addr.addr[0],
              out->rssi, out->name);
  return true;
}

// Full sequence: parse zip, scan (forever by default), then up to `retries`
// DFU attempts against the discovered target. Scan failures do NOT consume
// a retry — only post-scan DFU failures do. Buttonless triggers also don't
// consume retries: we just rescan after the peer reboots.
static void run_dfu_sequence() {
  char zip_name[64];
  int n = storage::find_single_zip(zip_name, sizeof(zip_name));
  if (n <= 0) {
    logger::log("dfu: trigger but no zip on drive (count=%d)", n);
    s_state = State::kDoneFail;
    return;
  }
  if (n > 1) {
    logger::log("dfu: %d zips on drive, expected exactly 1", n);
    s_state = State::kDoneFail;
    return;
  }

  char err[80];
  if (!firmware_zip::parse(zip_name, &s_bundle, err, sizeof(err))) {
    logger::log("dfu: zip parse failed: %s", err);
    s_state = State::kDoneFail;
    return;
  }
  logger::log("dfu: bundle %s  type=0x%02x bin=%lu B dat=%lu B",
              zip_name, s_bundle.type,
              (unsigned long)s_bundle.bin.size,
              (unsigned long)s_bundle.dat.size);

  const config::Config& cfg = config::current();
  const uint8_t  retries        = cfg.retries;
  const uint16_t cooldown       = cfg.retry_cooldown;
  const uint16_t wedge_cooldown = cfg.wedge_cooldown;
  uint8_t        dfu_attempt    = 0;

  // After a buttonless trigger we remember the app-mode MAC and pass it into
  // the next scan so the scanner also accepts MAC / MAC+1 hits — covers
  // bootloaders that re-advertise under a different name (e.g. RAK4631_OTA
  // → 4631_DFU). Cleared once we get a non-buttonless result.
  ble_gap_addr_t pending_app_mac      = {};
  bool           have_pending_app_mac = false;

  while (dfu_attempt < retries) {
    ble_scanner::Target t;
    if (!scan_for_target(&t, have_pending_app_mac ? &pending_app_mac : nullptr)) {
      // Only reachable when scan_timeout > 0. Treat as a soft failure.
      s_state = State::kDoneFail;
      zip_reader::close();
      return;
    }

    dfu_legacy::Result r = dfu_legacy::run(t, s_bundle, cfg);

    if (r == dfu_legacy::Result::kButtonlessTriggered) {
      logger::log("dfu: buttonless triggered, waiting 2 s for peer reboot...");
      pending_app_mac      = t.addr;
      have_pending_app_mac = true;
      delay(2000);
      continue;   // rescan; doesn't count as a DFU retry
    }

    // Anything other than buttonless means the next scan should go back to
    // the normal name/UUID matching.
    have_pending_app_mac = false;

    if (r == dfu_legacy::Result::kOk) {
      logger::log("dfu: SUCCESS, deleting %s", zip_name);
      if (!storage::delete_file(zip_name)) {
        logger::log("dfu: WARNING: failed to delete %s", zip_name);
      }
      zip_reader::close();
      s_state = State::kDoneOk;
      return;
    }

    dfu_attempt++;
    logger::log("dfu: attempt %u/%u failed with result=%d",
                dfu_attempt, retries, (int)r);
    if (dfu_attempt < retries) {
      // Classify the failure to pick a cooldown. Pre-connect failures
      // (link never came up, dropped during setup) — short cooldown, the
      // peer is fine, we just need to rescan. Post-connect failures
      // (service/char missing, response timeout, protocol error, mid-stream
      // drop) suggest the bootloader's DFU state machine is wedged from a
      // half-finished session; only its internal inactivity watchdog
      // unsticks it, so we wait the longer `wedge_cooldown`.
      bool wedge = (r != dfu_legacy::Result::kConnectFailed);
      uint16_t wait_s = wedge ? wedge_cooldown : cooldown;
      if (wait_s > 0) {
        logger::log("dfu: %s cooldown %u s before next attempt",
                    wedge ? "wedge" : "retry", wait_s);
        delay((uint32_t)wait_s * 1000);
      }
    }
  }

  logger::log("dfu: FAILED after %u attempts", retries);
  zip_reader::close();
  s_state = State::kDoneFail;
}

// ---------------------------------------------------------------------------
// LED tick
// ---------------------------------------------------------------------------
static void leds_tick(uint32_t now) {
  static uint32_t last  = 0;
  static bool     phase = false;

  // Helper to clear the red LED on boards that have one. On RAK4631 LED_RED
  // is aliased to LED_GREEN, so calling led_set(LED_RED, ...) would clobber
  // the green channel — guard it instead.
#if defined(BOARD_RAK4631)
  #define LED_RED_SET(on) ((void)0)
#else
  #define LED_RED_SET(on) led_set(LED_RED, (on))
#endif

  switch (s_state) {
    case State::kIdle:
      if (usb_msc::is_mounted()) {
        LED_RED_SET(false);
        led_set(LED_GREEN, false);
        led_set(LED_BLUE,  true);
      } else {
        if (now - last >= 500) { last = now; phase = !phase; }
        LED_RED_SET(false);
        led_set(LED_GREEN, false);
        led_set(LED_BLUE,  phase);
      }
      break;
    case State::kRunning: {
      // Green blink whose period shortens as progress climbs. At 0% the
      // half-period is 600 ms (slow flash); by ~95% it's near 30 ms so the
      // LED looks solid to the eye. 100% pins it on.
      uint8_t  pct       = s_progress_pct;
      uint32_t half_ms;
      if (pct >= 100) {
        LED_RED_SET(false);
        led_set(LED_GREEN, true);
        led_set(LED_BLUE, false);
        break;
      }
      half_ms = 600 - ((uint32_t)pct * (600 - 30)) / 100;  // 600..30 ms
      if (now - last >= half_ms) { last = now; phase = !phase; }
      LED_RED_SET(false);
      led_set(LED_GREEN, phase);
      led_set(LED_BLUE,  false);
      break;
    }
    case State::kDoneOk:
      LED_RED_SET(false);
      led_set(LED_GREEN, true);
      led_set(LED_BLUE,  false);
      break;
    case State::kDoneFail:
#if defined(BOARD_RAK4631)
      // No red LED — alternate green/blue at ~4 Hz so the failure state is
      // unmistakable.
      if (now - last >= 125) { last = now; phase = !phase; }
      led_set(LED_GREEN, phase);
      led_set(LED_BLUE,  !phase);
#else
      led_set(LED_RED,   true);
      led_set(LED_GREEN, false);
      led_set(LED_BLUE,  false);
#endif
      break;
  }
#undef LED_RED_SET
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  leds_off();
  Serial.begin(115200);

  bool vbus = vbus_present();

  s_storage_ok = storage::begin();
  // MSC is only useful while USB is connected; expose it unconditionally so
  // the rare "plugged in after boot" case still works.
  usb_msc::begin();
  dfu_legacy::set_progress_callback(on_dfu_progress);

  bool cfg_loaded = false;
  if (s_storage_ok) cfg_loaded = config::load();
  const config::Config& cfg = config::current();

  // BLE init has to come AFTER config so we can apply tx_power before
  // anything would transmit.
  ble_scanner::begin(cfg.tx_power);
  ble_scanner::set_debug(cfg.scan_debug);

  logger::log("boot: storage=%s vbus=%d cfg=%s",
              s_storage_ok ? "ok" : "fail", (int)vbus,
              cfg_loaded ? "CONFIG.TXT" : "defaults");
  logger::log("cfg:  ble_name='%s' prn=%u high_mtu=%d retries=%u min_rssi=%d retry_cooldown=%u wedge_cooldown=%u tx_power=%d scan_timeout=%u scan_debug=%d",
              cfg.ble_name, cfg.prn, (int)cfg.high_mtu, cfg.retries,
              (int)cfg.min_rssi, cfg.retry_cooldown, cfg.wedge_cooldown,
              (int)cfg.tx_power, cfg.scan_timeout, (int)cfg.scan_debug);

  // Boot-without-USB-power trigger: if we came up on battery and there's
  // already a zip on the drive, jump straight into DFU. This matches the
  // requirements' "physical unplug → board flashes target" workflow when
  // the XIAO has a battery wired to BAT+/BAT-.
  if (!vbus && s_storage_ok) {
    char zip_name[64];
    if (storage::find_single_zip(zip_name, sizeof(zip_name)) == 1) {
      logger::log("boot: no VBUS + zip present, arming DFU");
      s_armed_boot = true;
    }
  }
}

void loop() {
  uint32_t now = millis();
  leds_tick(now);

  if (s_state == State::kIdle) {
    bool eject_trig = s_storage_ok && usb_msc::was_ejected();
    if (eject_trig || s_armed_boot) {
      logger::log("dfu: trigger %s", eject_trig ? "eject" : "boot-no-vbus");
      s_armed_boot = false;
      s_state      = State::kRunning;
      run_dfu_sequence();   // blocking, sets s_state to kDoneOk / kDoneFail
    }
  }
}
