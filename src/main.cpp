#include <Arduino.h>
// IMPORTANT: must be in the same TU as setup()/Serial so the BSP swaps Serial
// to TinyUSB CDC. Without this, USB-MSC and CDC silently fail to enumerate.
#include <Adafruit_TinyUSB.h>
#include <nrf_power.h>

#include "ble_scanner.h"
#include "config.h"
#include "deferred_logger.h"
#include "dfu_legacy.h"
#include "firmware_zip.h"
#include "logger.h"
#include "retry_policy.h"
#include "storage.h"
#include "usb_msc.h"
#include "zip_reader.h"

// ---------------------------------------------------------------------------
// LEDs
// ---------------------------------------------------------------------------
// Indicator scheme:
//   BLUE  slow blink     = idle, waiting for the host
//   BLUE  solid          = USB configured by a host
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

static void log_effective_config(const char* source) {
  const config::Config& cfg = config::current();
  logger::log("cfg: source=%s ble_name='%s' prn=%u high_mtu=%d retries=%u min_rssi=%d retry_cooldown=%u wedge_cooldown=%u tx_power=%d scan_timeout=%u scan_debug=%d",
              source, cfg.ble_name, cfg.prn, (int)cfg.high_mtu, cfg.retries,
              (int)cfg.min_rssi, cfg.retry_cooldown, cfg.wedge_cooldown,
              (int)cfg.tx_power, cfg.scan_timeout, (int)cfg.scan_debug);
  if (cfg.ble_mac_set) {
    logger::log("cfg: ble_mac=%02X:%02X:%02X:%02X:%02X:%02X",
                cfg.ble_mac[5], cfg.ble_mac[4], cfg.ble_mac[3],
                cfg.ble_mac[2], cfg.ble_mac[1], cfg.ble_mac[0]);
  } else {
    logger::log("cfg: ble_mac=<none>");
  }
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
  ble_gap_addr_t target_mac_addr = {};
  const ble_gap_addr_t* target_mac = nullptr;
  bool has_name_filter = (cfg.ble_name[0] != '\0');

  if (!has_name_filter && cfg.ble_mac_set) {
    memcpy(target_mac_addr.addr, cfg.ble_mac, 6);
    target_mac = &target_mac_addr;

    logger::log(
      "scan: fixed MAC target %02X:%02X:%02X:%02X:%02X:%02X",
      cfg.ble_mac[5], cfg.ble_mac[4], cfg.ble_mac[3],
      cfg.ble_mac[2], cfg.ble_mac[1], cfg.ble_mac[0]
    );
  }
  if (cfg.scan_timeout == 0) {
    logger::log("scan: looking for DFU target (no timeout)%s...",
                prefer_mac ? ", MAC+1 fallback armed" : "");
  } else {
    logger::log("scan: looking for DFU target (%u s timeout)%s...",
                cfg.scan_timeout, prefer_mac ? ", MAC+1 fallback armed" : "");
  }
  if (!ble_scanner::find_first(out, timeout_ms, cfg.ble_name, cfg.min_rssi, prefer_mac, target_mac)) {
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

    dfu_legacy::RunResult run = dfu_legacy::run(t, s_bundle, cfg);
    dfu_legacy::Result r = run.result;

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
    logger::log("dfu: attempt %u/%u failed result=%d cleanup=%d",
                dfu_attempt, retries, (int)r, (int)run.cleanup);

    retry_policy::CooldownClass cooldown_class = retry_policy::classify(run);
    if (cooldown_class == retry_policy::CooldownClass::kStop) {
      logger::log("dfu: local fatal error; retries stopped");
      break;
    }

    if (dfu_attempt < retries) {
      // A confirmed RESET gets the short retry delay. Only a session whose
      // link vanished or had to be forced down waits for the bootloader's
      // inactivity watchdog.
      bool wedge = cooldown_class == retry_policy::CooldownClass::kWedge;
      uint16_t wait_s = wedge ? wedge_cooldown : cooldown;
      if (wait_s > 0) {
        logger::log("dfu: %s cooldown %u s before next attempt",
                    wedge ? "wedge" : "retry", wait_s);
        delay((uint32_t)wait_s * 1000);
      }
    }
  }

  logger::log("dfu: FAILED after %u attempt(s)", dfu_attempt);
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
      if (usb_msc::usb_configured()) {
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
  bool deferred_log_ok = deferred_logger::begin();

  bool vbus = vbus_present();

  // TinyUSB starts before setup(). Hold its pull-up down while storage is
  // probed and the final composite descriptor is assembled. This prevents a
  // slow first-time flash format from enumerating CDC before MSC is added.
  bool usb_suspended = usb_msc::suspend_for_descriptor_update();

  s_storage_ok = storage::begin();
  dfu_legacy::set_progress_callback(on_dfu_progress);

  // load() always establishes defaults, even if storage failed.
  bool cfg_loaded = config::load();

  // Inspect battery-trigger state before exposing the block device. If a zip
  // is already staged, this boot belongs to firmware and USB remains CDC-only.
  if (!vbus && s_storage_ok) {
    char zip_name[64];
    if (storage::find_single_zip(zip_name, sizeof(zip_name)) == 1) {
      s_armed_boot = true;
    }
  }

  bool msc_ok = false;
  if (s_storage_ok && !s_armed_boot) msc_ok = usb_msc::begin();
  usb_msc::resume_after_descriptor_update(usb_suspended);

  const config::Config& cfg = config::current();

  // BLE init has to come AFTER config so we can apply tx_power before
  // anything would transmit.
  ble_scanner::begin(cfg.tx_power);
  ble_scanner::set_debug(cfg.scan_debug);

  logger::log("boot: storage=%s msc=%s vbus=%d cfg=%s",
              s_storage_ok ? "ok" : "fail",
              msc_ok ? "ready" : "cdc-only", (int)vbus,
              cfg_loaded ? "CONFIG.TXT" : "defaults");
  if (!deferred_log_ok) {
    logger::log("boot: callback event queue unavailable");
  }
  log_effective_config(cfg_loaded ? "CONFIG.TXT" : "defaults");

  // Boot-without-USB-power trigger: if we came up on battery and there's
  // already a zip on the drive, jump straight into DFU. This matches the
  // requirements' "physical unplug → board flashes target" workflow when
  // the XIAO has a battery wired to BAT+/BAT-.
  if (s_armed_boot) {
    logger::log("boot: no VBUS + zip present, arming DFU");
  }
}

void loop() {
  // Bluefruit callbacks only enqueue fixed-size events. All formatting and
  // Serial/SdFat/QSPI logging is confined to this Arduino task.
  deferred_logger::drain();

  uint32_t now = millis();
  leds_tick(now);

  if (s_state == State::kIdle) {
    bool eject_trig = s_storage_ok && usb_msc::was_ejected();
    if (eject_trig || s_armed_boot) {
      if (eject_trig) {
        // START_STOP_UNIT already made the LUN not-ready. Complete the
        // exclusive handoff before any SdFat read, then reload the files the
        // host just wrote during this same boot.
        if (!usb_msc::claim_after_eject()) return;
        bool cfg_loaded = config::load();
        const config::Config& cfg = config::current();
        ble_scanner::apply_runtime_settings(cfg.tx_power, cfg.scan_debug);
        log_effective_config(cfg_loaded ? "CONFIG.TXT after eject" :
                                         "defaults after eject");
      }
      logger::log("dfu: trigger %s", eject_trig ? "eject" : "boot-no-vbus");
      s_armed_boot = false;
      s_state      = State::kRunning;
      run_dfu_sequence();   // blocking, sets s_state to kDoneOk / kDoneFail
    }
  }
}
