#pragma once

namespace v4_log {

// Serial-only by design.  BLE callbacks never call this function; they only
// enqueue fixed-size events for the Arduino task to consume.
void line(const char* fmt, ...);

}  // namespace v4_log
