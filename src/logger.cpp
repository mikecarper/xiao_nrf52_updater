#include "logger.h"

#include <stdarg.h>
#include <stdio.h>

#include "storage.h"
#include "usb_msc.h"

namespace logger {

static constexpr const char* kLogPath = "LOG.TXT";

void log(const char* fmt, ...) {
  // Format once into a stack buffer; reuse for Serial and the file write.
  char body[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(body, sizeof(body), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n >= (int)sizeof(body)) n = sizeof(body) - 1;

  // Prefix with a boot-relative hh:mm:ss timestamp (no RTC available).
  uint32_t total_s = millis() / 1000;
  uint32_t hh      = total_s / 3600;
  uint32_t mm      = (total_s / 60) % 60;
  uint32_t ss      = total_s % 60;

  char line[192];
  int total = snprintf(line, sizeof(line), "[%02lu:%02lu:%02lu] %s\r\n",
                       (unsigned long)hh, (unsigned long)mm, (unsigned long)ss, body);
  if (total < 0) return;
  if (total >= (int)sizeof(line)) total = sizeof(line) - 1;

  // Always mirror to Serial — even when the FS write fails.
  Serial.write((const uint8_t*)line, total);

  // A block host and SdFat must never share the volume. File logging starts
  // only after the one-shot eject handoff (or on a battery-triggered boot
  // where MSC was never exposed).
  if (!storage::ready() || !usb_msc::firmware_owns_media()) return;

  File f = storage::fs().open(kLogPath, O_WRONLY | O_CREAT | O_APPEND);
  if (!f) return;
  f.write((const uint8_t*)line, total);
  f.close();
}

}  // namespace logger
