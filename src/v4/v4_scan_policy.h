#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace v4_scan_policy {

inline bool address_matches(const uint8_t candidate[6],
                            const uint8_t reference[6]) {
  return candidate && reference && memcmp(candidate, reference, 6) == 0;
}

// NimBLEAddress::getNative() is least-significant octet first, so increment
// from index zero. This is the address convention used by Nordic bootloaders
// which advertise the application address plus one after buttonless reboot.
inline bool address_matches_or_plus_one(const uint8_t candidate[6],
                                        const uint8_t reference[6]) {
  if (address_matches(candidate, reference)) return true;
  if (!candidate || !reference) return false;
  uint8_t plus_one[6];
  memcpy(plus_one, reference, sizeof(plus_one));
  // OTAFIX's SDK11 bootloader increments only addr[0] (uint8_t wrap), not a
  // 48-bit integer with carry.
  ++plus_one[0];
  return address_matches(candidate, plus_one);
}

// Once the application has been triggered, its immutable address is the only
// identity anchor. A configured name match must never broaden that second-leg
// scan to another same-name device.
inline bool accept_candidate(const uint8_t candidate[6],
                             const uint8_t* preferred,
                             bool configured_target_matches) {
  if (preferred) {
    return address_matches_or_plus_one(candidate, preferred);
  }
  return configured_target_matches;
}

}  // namespace v4_scan_policy
