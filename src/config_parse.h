#pragma once

#include <stddef.h>
#include <stdint.h>

namespace config_parse {

// Parse exactly six two-digit hexadecimal octets separated consistently by
// ':' or '-'. The caller trims surrounding whitespace before calling this.
// Bytes are returned in SoftDevice order (least-significant octet first).
bool ble_mac(const char* text, uint8_t out[6]);

}  // namespace config_parse
