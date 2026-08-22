#include <Arduino.h>

#include "retry_policy.h"
#include "v4_bundle.h"
#include "v4_config.h"
#include "v4_dfu.h"
#include "v4_log.h"
#include "v4_scanner.h"
#include "v4_storage.h"
#include "v4_zip.h"

namespace {

constexpr uint8_t kLedPin = 35;
constexpr uint32_t kMaxUploadBytes = 2u * 1024u * 1024u;
constexpr size_t kSerialRxBufferBytes = 8192;
constexpr uint8_t kMaxButtonlessTransitions = 1;
constexpr uint16_t kBootloaderScanTimeoutSeconds = 45;

enum class LedState : uint8_t {
  kIdle,
  kRunning,
  kSuccess,
  kFailure,
};

LedState s_led_state = LedState::kIdle;
volatile uint8_t s_progress = 0;
bool s_serial_quarantined = false;
bool s_serial_rx_ready = false;

void update_led() {
  static uint32_t last_change = 0;
  static bool phase = false;
  uint32_t now = millis();
  switch (s_led_state) {
    case LedState::kIdle:
      if (now - last_change >= 500) {
        last_change = now;
        phase = !phase;
      }
      digitalWrite(kLedPin, phase);
      break;
    case LedState::kRunning: {
      uint32_t half_period = 500u -
          static_cast<uint32_t>(s_progress) * 470u / 100u;
      if (now - last_change >= half_period) {
        last_change = now;
        phase = !phase;
      }
      digitalWrite(kLedPin, phase);
      break;
    }
    case LedState::kSuccess:
      digitalWrite(kLedPin, HIGH);
      break;
    case LedState::kFailure:
      if (now - last_change >= 100) {
        last_change = now;
        phase = !phase;
      }
      digitalWrite(kLedPin, phase);
      break;
  }
}

void on_progress(uint8_t percent) {
  s_progress = percent;
  update_led();
}

bool valid_sha_text(const char* text) {
  if (!text || strlen(text) != 64) return false;
  for (size_t i = 0; i < 64; ++i) {
    char c = text[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
          (c >= 'A' && c <= 'F'))) {
      return false;
    }
  }
  return true;
}

void print_help() {
  Serial.println("Commands:");
  Serial.println("  STATUS");
  Serial.println("  TARGET NAME <substring[|substring]>");
  Serial.println("  TARGET MAC <AA:BB:CC:DD:EE:FF>");
  Serial.println("  TARGET CLEAR");
  Serial.println("  UPLOAD <byte-count> <sha256>  (wait for READY 512; ACK each chunk)");
  Serial.println("  START");
  Serial.println("  ERASE");
  Serial.println("No update runs automatically at boot or after UPLOAD.");
}

void print_status() {
  char target[48];
  v4_config::describe_target(target, sizeof(target));
  Serial.printf("STATUS storage=%s staged=%s bytes=%lu target=%s\r\n",
                v4_storage::ready() ? "ready" : "failed",
                v4_storage::has_staged() ? "yes" : "no",
                static_cast<unsigned long>(v4_storage::staged_size()), target);
}

void perform_dfu() {
  if (!v4_config::has_target()) {
    v4_log::line("start refused: set TARGET NAME or TARGET MAC first");
    s_led_state = LedState::kFailure;
    return;
  }

  char error[96] = {};
  if (!v4_storage::verify_staged(error, sizeof(error))) {
    v4_log::line("start refused: %s", error);
    s_led_state = LedState::kFailure;
    return;
  }

  v4_bundle::Parsed bundle;
  if (!v4_bundle::parse(v4_storage::kStagedPath, &bundle,
                        error, sizeof(error))) {
    v4_log::line("start refused: ZIP invalid: %s", error);
    s_led_state = LedState::kFailure;
    return;
  }
  v4_log::line("bundle: type=%02X bin=%lu dat=%lu sd=%lu bl=%lu app=%lu",
               bundle.type, static_cast<unsigned long>(bundle.bin.size),
               static_cast<unsigned long>(bundle.dat.size),
               static_cast<unsigned long>(bundle.sd_size),
               static_cast<unsigned long>(bundle.bl_size),
               static_cast<unsigned long>(bundle.app_size));

  const v4_config::Config& config = v4_config::current();
  s_led_state = LedState::kRunning;
  s_progress = 0;
  uint8_t attempts = 0;
  uint8_t buttonless_transitions = 0;
  v4_scanner::Target preferred = {};
  bool have_preferred = false;

  while (attempts < config.retries) {
    v4_log::line("scan: waiting for explicit target%s",
                 have_preferred ? " (app MAC/+1 preferred)" : "");
    v4_scanner::Target target;
    v4_config::Config scan_config = config;
    if (have_preferred &&
        (scan_config.scan_timeout == 0 ||
         scan_config.scan_timeout > kBootloaderScanTimeoutSeconds)) {
      scan_config.scan_timeout = kBootloaderScanTimeoutSeconds;
    }
    if (!v4_scanner::find_first(&target, scan_config,
                                have_preferred ? &preferred : nullptr)) {
      v4_log::line("scan: target not found before timeout");
      v4_zip::close();
      s_led_state = LedState::kFailure;
      return;
    }
    char address[18];
    v4_scanner::format_address(target, address);
    v4_log::line("scan: found %s type=%u rssi=%d name='%s'", address,
                 target.address_type, target.rssi, target.name);

    dfu_legacy::RunResult run = v4_dfu::run(
        target, bundle, config,
        buttonless_transitions < kMaxButtonlessTransitions);
    if (run.result == dfu_legacy::Result::kButtonlessTriggered) {
      if (!have_preferred) {
        preferred = target;  // immutable app address for all later scans
        have_preferred = true;
      }
      ++buttonless_transitions;
      v4_log::line(
          "dfu: buttonless transition %u/%u; strict MAC/MAC+1 scan in 2 s "
          "(timeout %u s)",
          buttonless_transitions, kMaxButtonlessTransitions,
          kBootloaderScanTimeoutSeconds);
      delay(2000);
      continue;
    }
    if (run.result == dfu_legacy::Result::kOk) {
      v4_zip::close();
      v4_config::clear_target();
      bool erased = v4_storage::erase_staged();
      if (!erased) {
        v4_log::line(
            "dfu: FAILED staging cleanup after target activation; serial "
            "quarantined until reboot");
        s_serial_quarantined = true;
        s_led_state = LedState::kFailure;
      } else {
        v4_log::line("dfu: SUCCESS staged-delete=1");
        s_led_state = LedState::kSuccess;
      }
      return;
    }

    ++attempts;
    v4_log::line("dfu: attempt %u/%u failed result=%d cleanup=%d",
                 attempts, config.retries, static_cast<int>(run.result),
                 static_cast<int>(run.cleanup));
    retry_policy::CooldownClass cooldown = retry_policy::classify(run);
    if (cooldown == retry_policy::CooldownClass::kStop) break;
    if (attempts < config.retries) {
      uint16_t wait_seconds =
          cooldown == retry_policy::CooldownClass::kWedge
              ? config.wedge_cooldown
              : config.retry_cooldown;
      if (wait_seconds) {
        v4_log::line("dfu: cooldown %u s", wait_seconds);
        delay(static_cast<uint32_t>(wait_seconds) * 1000u);
      }
    }
  }

  v4_zip::close();
  v4_log::line("dfu: FAILED after %u attempt(s); staged ZIP retained", attempts);
  s_led_state = LedState::kFailure;
}

void handle_upload(const char* arguments) {
  // A new raw-frame transaction invalidates any prior target authorization.
  // The target must be set again only after the new bundle is safely staged.
  v4_config::clear_target();
  if (!s_serial_rx_ready) {
    Serial.println("ERR serial RX buffer unavailable; reboot required");
    return;
  }
  unsigned long size = 0;
  char sha[65] = {};
  char extra = '\0';
  if (!arguments ||
      sscanf(arguments, "%lu %64s %c", &size, sha, &extra) != 2 ||
      size == 0 || size > kMaxUploadBytes || !valid_sha_text(sha)) {
    Serial.println("ERR usage: UPLOAD <1..2097152> <64-hex-sha256>");
    return;
  }

  char error[96] = {};
  if (!v4_storage::prepare_incoming(static_cast<uint32_t>(size), sha,
                                    error, sizeof(error))) {
    Serial.printf("ERR prepare: %s\r\n", error);
    return;
  }

  // READY is emitted only after the file and SHA context exist.
  Serial.println("READY 512");
  Serial.flush();
  v4_storage::ReceiveResult receive =
      v4_storage::receive_prepared(error, sizeof(error));
  if (receive != v4_storage::ReceiveResult::kOk) {
    if (receive == v4_storage::ReceiveResult::kPartialFailure) {
      s_serial_quarantined = true;
      Serial.printf("ERR upload: %s; serial quarantined until reboot\r\n",
                    error);
      return;
    }
    Serial.printf("ERR upload: %s\r\n", error);
    return;
  }

  v4_bundle::Parsed parsed;
  if (!v4_bundle::parse(v4_storage::kStagedPath, &parsed,
                        error, sizeof(error))) {
    v4_storage::discard_received();
    Serial.printf("ERR ZIP: %s\r\n", error);
    return;
  }
  v4_zip::close();
  if (!v4_storage::commit_received(error, sizeof(error))) {
    Serial.printf("ERR commit: %s\r\n", error);
    return;
  }
  Serial.printf("OK staged bytes=%lu type=%02X bin=%lu dat=%lu\r\n",
                size, parsed.type,
                static_cast<unsigned long>(parsed.bin.size),
                static_cast<unsigned long>(parsed.dat.size));
}

void handle_command(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  size_t length = strlen(line);
  while (length && (line[length - 1] == ' ' || line[length - 1] == '\t' ||
                    line[length - 1] == '\r')) {
    line[--length] = '\0';
  }
  if (length == 0) return;

  if (strcasecmp(line, "HELP") == 0) {
    print_help();
  } else if (strcasecmp(line, "STATUS") == 0) {
    print_status();
  } else if (strncasecmp(line, "TARGET NAME ", 12) == 0) {
    if (v4_config::set_target_name(line + 12)) {
      Serial.println("OK target name set");
    } else {
      Serial.println("ERR target name must be 1..31 characters");
    }
  } else if (strncasecmp(line, "TARGET MAC ", 11) == 0) {
    if (v4_config::set_target_mac(line + 11)) {
      Serial.println("OK target MAC set");
    } else {
      Serial.println("ERR MAC must be six two-digit octets");
    }
  } else if (strcasecmp(line, "TARGET CLEAR") == 0) {
    v4_config::clear_target();
    Serial.println("OK target cleared");
  } else if (strncasecmp(line, "UPLOAD ", 7) == 0) {
    handle_upload(line + 7);
  } else if (strcasecmp(line, "START") == 0) {
    perform_dfu();
  } else if (strcasecmp(line, "ERASE") == 0) {
    v4_config::clear_target();
    Serial.println(v4_storage::erase_staged() ? "OK erased" : "ERR erase failed");
  } else {
    Serial.println("ERR unknown command; use HELP");
  }
}

void poll_serial() {
  static char line[160];
  static size_t used = 0;
  static bool discard_until_newline = false;
  if (s_serial_quarantined) {
    // Never reinterpret unconsumed raw-frame bytes as newline commands. Drain
    // only to avoid filling the CDC receive buffer; reboot is the sole exit.
    while (Serial.available()) Serial.read();
    return;
  }
  while (!s_serial_quarantined && Serial.available()) {
    int value = Serial.read();
    if (value < 0) return;
    if (value == '\n') {
      if (discard_until_newline) {
        discard_until_newline = false;
        used = 0;
        continue;
      }
      line[used] = '\0';
      handle_command(line);
      used = 0;
      if (s_serial_quarantined) {
        while (Serial.available()) Serial.read();
        return;
      }
    } else if (discard_until_newline) {
      continue;
    } else if (used + 1 < sizeof(line)) {
      line[used++] = static_cast<char>(value);
    } else {
      used = 0;
      discard_until_newline = true;
      Serial.println("ERR command too long");
    }
  }
}

}  // namespace

void setup() {
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, LOW);
  size_t rx_buffer_bytes = Serial.setRxBufferSize(kSerialRxBufferBytes);
  s_serial_rx_ready = rx_buffer_bytes == kSerialRxBufferBytes;
  Serial.begin(115200);
  uint32_t started_at = millis();
  while (!Serial && millis() - started_at < 3000) delay(10);

  v4_config::begin();
  bool storage_ok = v4_storage::begin();
  bool ble_ok = v4_scanner::begin();
  v4_dfu::set_progress_callback(on_progress);
  v4_log::line("boot: Heltec V4 Legacy DFU central storage=%d ble=%d",
               storage_ok, ble_ok);
  v4_log::line("boot: serial RX buffer=%u/%u bytes",
               static_cast<unsigned>(rx_buffer_bytes),
               static_cast<unsigned>(kSerialRxBufferBytes));
  print_status();
  print_help();
  if (!storage_ok || !ble_ok || rx_buffer_bytes != kSerialRxBufferBytes)
    s_led_state = LedState::kFailure;
}

void loop() {
  update_led();
  poll_serial();
  delay(1);
}
