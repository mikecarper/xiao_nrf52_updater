#include "config_parse.h"

namespace config_parse {

static int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool ble_mac(const char* text, uint8_t out[6]) {
  if (!text || !out) return false;

  uint8_t parsed[6];
  char separator = '\0';
  size_t pos = 0;

  for (size_t i = 0; i < 6; ++i) {
    // Check each byte before looking one character ahead. Besides enforcing
    // exactly two hex digits per octet, this keeps malformed short strings
    // from being read past their terminating NUL.
    if (text[pos] == '\0') return false;
    int high = hex_value(text[pos]);
    if (high < 0 || text[pos + 1] == '\0') return false;
    int low = hex_value(text[pos + 1]);
    if (low < 0) return false;
    parsed[i] = static_cast<uint8_t>((high << 4) | low);
    pos += 2;

    if (i == 5) break;
    char current = text[pos++];
    if (current != ':' && current != '-') return false;
    if (separator == '\0') separator = current;
    if (current != separator) return false;
  }

  if (text[pos] != '\0') return false;

  // Nordic/SoftDevice stores BLE addresses least-significant octet first.
  for (size_t i = 0; i < 6; ++i) out[5 - i] = parsed[i];
  return true;
}

}  // namespace config_parse
