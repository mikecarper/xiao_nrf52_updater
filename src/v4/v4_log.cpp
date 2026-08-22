#include "v4_log.h"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

namespace v4_log {

void line(const char* fmt, ...) {
  char body[192];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(body, sizeof(body), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  body[sizeof(body) - 1] = '\0';

  uint32_t total_s = millis() / 1000;
  char prefix[24];
  int p = snprintf(prefix, sizeof(prefix), "[%02lu:%02lu:%02lu] ",
                   (unsigned long)(total_s / 3600),
                   (unsigned long)((total_s / 60) % 60),
                   (unsigned long)(total_s % 60));
  if (p > 0) Serial.write(reinterpret_cast<const uint8_t*>(prefix), p);
  Serial.println(body);
}

}  // namespace v4_log
