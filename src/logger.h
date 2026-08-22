#pragma once

#include <Arduino.h>

namespace logger {

// Mirror a line to Serial and, when firmware exclusively owns the mounted
// volume, append it to LOG.TXT. Timestamp is boot-relative hh:mm:ss.
void log(const char* fmt, ...);

}  // namespace logger
