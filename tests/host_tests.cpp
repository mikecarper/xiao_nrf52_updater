#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "callback_log_event.h"
#include "config_parse.h"
#include "dfu_client_lifecycle.h"
#include "dfu_image_layout.h"
#include "dfu_protocol.h"
#include "retry_policy.h"
#include "v4/v4_scan_policy.h"

static void test_mac_parser() {
  uint8_t mac[6] = {};
  assert(config_parse::ble_mac("AA:bb:01:02:FE:ff", mac));
  const uint8_t expected[] = {0xFF, 0xFE, 0x02, 0x01, 0xBB, 0xAA};
  assert(memcmp(mac, expected, sizeof(mac)) == 0);
  assert(config_parse::ble_mac("00-11-22-33-44-55", mac));

  assert(!config_parse::ble_mac("", mac));
  assert(!config_parse::ble_mac("A", mac));
  assert(!config_parse::ble_mac("A:B:C:D:E:F", mac));
  assert(!config_parse::ble_mac("00:11:22:33:44", mac));
  assert(!config_parse::ble_mac("00:11:22:33:44:55junk", mac));
  assert(!config_parse::ble_mac("00:11-22:33:44:55", mac));
  assert(!config_parse::ble_mac("GG:11:22:33:44:55", mac));
  assert(!config_parse::ble_mac(nullptr, mac));
  assert(!config_parse::ble_mac("00:11:22:33:44:55", nullptr));
}

static void test_notification_parser() {
  const uint8_t response[] = {0x10, 0x03, 0x01};
  uint8_t op = 0;
  uint8_t status = 0;
  assert(dfu_protocol::classify(response, sizeof(response)) ==
         dfu_protocol::NotificationKind::kResponse);
  assert(dfu_protocol::decode_response(response, sizeof(response), &op, &status));
  assert(op == 0x03 && status == 0x01);

  const uint8_t prn[] = {0x11, 0x78, 0x56, 0x34, 0x12};
  uint32_t received = 0;
  assert(dfu_protocol::classify(prn, sizeof(prn)) ==
         dfu_protocol::NotificationKind::kPacketReceipt);
  assert(dfu_protocol::decode_prn(prn, sizeof(prn), &received));
  assert(received == 0x12345678u);

  const uint8_t unknown[] = {0x99};
  assert(dfu_protocol::classify(unknown, sizeof(unknown)) ==
         dfu_protocol::NotificationKind::kUnknown);
  assert(!dfu_protocol::decode_response(response, 2, &op, &status));
  assert(!dfu_protocol::decode_prn(prn, 4, &received));
}

static void test_dfu_client_registration_is_one_shot() {
  dfu_client_lifecycle::RegistrationState registration;
  unsigned begin_count = 0;

  // A normal update enters run() first against the application/buttonless
  // service and then again against the bootloader service. Both legs reuse the
  // same static Bluefruit client objects and must produce one registration.
  for (unsigned leg = 0; leg < 2; ++leg) {
    if (!registration.initialized()) {
      ++begin_count;
      registration.mark_initialized();
    }
  }
  assert(begin_count == 1);

  // Later retry legs must not grow Bluefruit's append-only dispatch lists.
  for (unsigned retry = 0; retry < 4; ++retry) {
    if (!registration.initialized()) ++begin_count;
  }
  assert(begin_count == 1);
}

static void assert_image_layout(
    uint8_t manifest_section, uint32_t bin_size, uint32_t metadata_sd_size,
    uint32_t metadata_bl_size, uint8_t expected_type,
    uint32_t expected_sd_size, uint32_t expected_bl_size,
    uint32_t expected_app_size, const uint8_t expected_tuple[12]) {
  using namespace dfu_image_layout;

  ManifestSection selected = static_cast<ManifestSection>(0);
  assert(select_section(manifest_section, &selected) == Status::kOk);
  assert(static_cast<uint8_t>(selected) == manifest_section);

  Layout layout;
  assert(make_layout(selected, bin_size, metadata_sd_size, metadata_bl_size,
                     &layout) == Status::kOk);
  assert(layout.type == expected_type);
  assert(layout.sd_size == expected_sd_size);
  assert(layout.bl_size == expected_bl_size);
  assert(layout.app_size == expected_app_size);

  uint8_t tuple[13];
  memset(tuple, 0xA5, sizeof(tuple));
  assert(encode_size_tuple(layout, tuple, 12));
  assert(memcmp(tuple, expected_tuple, 12) == 0);
  assert(tuple[12] == 0xA5);
}

static void test_manifest_sections_and_size_tuples() {
  using namespace dfu_image_layout;

  static const uint8_t app_tuple[12] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0x78, 0x56, 0x34, 0x12};
  assert_image_layout(kApplication, 0x12345678u, 0, 0, kTypeApplication,
                      0, 0, 0x12345678u, app_tuple);

  static const uint8_t bootloader_tuple[12] = {
      0, 0, 0, 0, 0x78, 0x56, 0x34, 0x12, 0, 0, 0, 0};
  assert_image_layout(kBootloader, 0x12345678u, 0, 0, kTypeBootloader,
                      0, 0x12345678u, 0, bootloader_tuple);

  static const uint8_t softdevice_tuple[12] = {
      0x78, 0x56, 0x34, 0x12, 0, 0, 0, 0, 0, 0, 0, 0};
  assert_image_layout(kSoftdevice, 0x12345678u, 0, 0, kTypeSoftdevice,
                      0x12345678u, 0, 0, softdevice_tuple);

  static const uint8_t system_tuple[12] = {
      0x04, 0x03, 0x02, 0x01, 0x14, 0x13, 0x12, 0x11, 0, 0, 0, 0};
  assert_image_layout(kSoftdeviceBootloader, 0x12141618u, 0x01020304u,
                      0x11121314u, kTypeSoftdevice | kTypeBootloader,
                      0x01020304u, 0x11121314u, 0, system_tuple);

  // Every Secure-DFU-only combined application section must be rejected,
  // including when a valid-looking Legacy section is also present.
  const uint8_t secure_sections[] = {
      kBootloaderApplication,
      kSoftdeviceApplication,
      kSoftdeviceBootloaderApplication,
  };
  for (size_t i = 0; i < sizeof(secure_sections); ++i) {
    ManifestSection selected = static_cast<ManifestSection>(0);
    assert(select_section(secure_sections[i], &selected) ==
           Status::kSecureDfuRequired);
    assert(select_section(secure_sections[i] | kApplication, &selected) ==
           Status::kSecureDfuRequired);
  }

  // A Legacy manifest may contain system + application entries for a
  // two-session update. This updater must reject all multi-entry combinations
  // instead of flashing one entry and deleting the package as if complete.
  const uint8_t legacy_sections[] = {
      kApplication, kBootloader, kSoftdevice, kSoftdeviceBootloader};
  for (size_t i = 0; i < sizeof(legacy_sections); ++i) {
    for (size_t j = i + 1; j < sizeof(legacy_sections); ++j) {
      ManifestSection selected = static_cast<ManifestSection>(0);
      assert(select_section(legacy_sections[i] | legacy_sections[j],
                            &selected) == Status::kMultipleLegacySections);
    }
  }

  ManifestSection selected = static_cast<ManifestSection>(0);
  assert(select_section(0, &selected) == Status::kNoFirmwareSection);
  assert(select_section(0x80, &selected) == Status::kInvalidArgument);
  assert(select_section(kApplication, nullptr) == Status::kInvalidArgument);

  Layout layout;
  assert(make_layout(kApplication, 0, 0, 0, &layout) ==
         Status::kEmptyImage);
  assert(make_layout(kSoftdeviceBootloader, 100, 0, 100, &layout) ==
         Status::kCombinedSizesMissing);
  assert(make_layout(kSoftdeviceBootloader, 100, 40, 0, &layout) ==
         Status::kCombinedSizesMissing);
  assert(make_layout(kSoftdeviceBootloader, 100, 40, 59, &layout) ==
         Status::kCombinedSizeMismatch);
  assert(make_layout(kSoftdeviceBootloader, 100, 101, UINT32_MAX, &layout) ==
         Status::kCombinedSizeMismatch);
  assert(make_layout(kBootloaderApplication, 100, 0, 0, &layout) ==
         Status::kInvalidArgument);
  assert(make_layout(kApplication, 100, 0, 0, nullptr) ==
         Status::kInvalidArgument);

  uint8_t short_tuple[12] = {};
  assert(!encode_size_tuple(layout, nullptr, sizeof(short_tuple)));
  assert(!encode_size_tuple(layout, short_tuple, sizeof(short_tuple) - 1));
  const Layout invalid_layouts[] = {
      {0, 0, 0, 0},
      {kTypeApplication, 0, 0, 0},
      {kTypeApplication, 1, 0, 1},
      {kTypeBootloader, 0, 0, 0},
      {kTypeSoftdevice, 0, 1, 0},
      {static_cast<uint8_t>(kTypeSoftdevice | kTypeBootloader), 1, 0, 0},
      {static_cast<uint8_t>(kTypeSoftdevice | kTypeBootloader), 1, 1, 1},
      {static_cast<uint8_t>(kTypeApplication | kTypeBootloader), 0, 1, 1},
  };
  for (size_t i = 0; i < sizeof(invalid_layouts) / sizeof(invalid_layouts[0]);
       ++i) {
    assert(!encode_size_tuple(invalid_layouts[i], short_tuple,
                              sizeof(short_tuple)));
  }
}

static void test_callback_log_events_are_owned_and_deferred() {
  char rendered[112];

  callback_log_event::Event connected = callback_log_event::ble_connected(7);
  assert(callback_log_event::format(connected, rendered, sizeof(rendered)));
  assert(strcmp(rendered, "dfu: connected (conn=7)") == 0);

  callback_log_event::Event disconnected =
      callback_log_event::ble_disconnected(0x13);
  assert(callback_log_event::format(disconnected, rendered, sizeof(rendered)));
  assert(strcmp(rendered, "dfu: disconnected reason=0x13") == 0);

  uint8_t addr[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x02};
  char name[] = "RAK3401_OTA";
  char reason[] = "mac?";
  callback_log_event::Event scan =
      callback_log_event::scan_rejected(addr, -57, name, reason);

  // The callback's advertisement buffer is owned by Bluefruit and becomes
  // invalid when it returns. Verify the deferred event owns every byte it
  // needs instead of retaining any caller pointer.
  memset(addr, 0, sizeof(addr));
  memset(name, 'X', sizeof(name) - 1);
  memset(reason, 'Y', sizeof(reason) - 1);
  assert(callback_log_event::format(scan, rendered, sizeof(rendered)));
  assert(strcmp(rendered,
                "scan: mac? 02:00:00:00:00:01 rssi=-57 name='RAK3401_OTA'") ==
         0);

  char tiny[4];
  assert(callback_log_event::format(scan, tiny, sizeof(tiny)));
  assert(tiny[sizeof(tiny) - 1] == '\0');
  assert(!callback_log_event::format(scan, nullptr, sizeof(rendered)));
  assert(!callback_log_event::format(scan, rendered, 0));

  callback_log_event::Event invalid = scan;
  invalid.kind = static_cast<callback_log_event::Kind>(0xFF);
  assert(!callback_log_event::format(invalid, rendered, sizeof(rendered)));
  assert(rendered[0] == '\0');
}

static void test_retry_policy() {
  using dfu_legacy::CleanupOutcome;
  using dfu_legacy::Result;
  using retry_policy::CooldownClass;

  assert(retry_policy::classify({Result::kFsError,
                                 CleanupOutcome::kResetObserved}) ==
         CooldownClass::kStop);
  assert(retry_policy::classify({Result::kRemoteError,
                                 CleanupOutcome::kResetObserved}) ==
         CooldownClass::kShort);
  assert(retry_policy::classify({Result::kRemoteError,
                                 CleanupOutcome::kForcedDisconnect}) ==
         CooldownClass::kWedge);
  assert(retry_policy::classify({Result::kDisconnectedEarly,
                                 CleanupOutcome::kLinkAlreadyLost}) ==
         CooldownClass::kWedge);
  assert(retry_policy::classify({Result::kConnectFailed,
                                 CleanupOutcome::kNotNeeded}) ==
         CooldownClass::kShort);
  assert(retry_policy::classify({Result::kServiceMissing,
                                 CleanupOutcome::kForcedDisconnect}) ==
         CooldownClass::kShort);
  assert(retry_policy::classify({Result::kButtonlessLimit,
                                 CleanupOutcome::kNotNeeded}) ==
         CooldownClass::kStop);
}

static void test_v4_strict_transition_address() {
  const uint8_t anchor[] = {0xFE, 0x10, 0x20, 0x30, 0x40, 0x50};
  const uint8_t plus_one[] = {0xFF, 0x10, 0x20, 0x30, 0x40, 0x50};
  const uint8_t same_name_wrong_mac[] = {0x01, 0x99, 0x20, 0x30, 0x40, 0x50};
  assert(v4_scan_policy::address_matches_or_plus_one(anchor, anchor));
  assert(v4_scan_policy::address_matches_or_plus_one(plus_one, anchor));
  assert(!v4_scan_policy::accept_candidate(same_name_wrong_mac, anchor, true));
  assert(v4_scan_policy::accept_candidate(same_name_wrong_mac, nullptr, true));

  // OTAFIX increments only native addr[0], wrapping without carry.
  const uint8_t wrap_anchor[] = {0xFF, 0x10, 0x20, 0x30, 0x40, 0x50};
  const uint8_t wrapped[] = {0x00, 0x10, 0x20, 0x30, 0x40, 0x50};
  const uint8_t carried[] = {0x00, 0x11, 0x20, 0x30, 0x40, 0x50};
  assert(v4_scan_policy::address_matches_or_plus_one(wrapped, wrap_anchor));
  assert(!v4_scan_policy::address_matches_or_plus_one(carried, wrap_anchor));
}

int main() {
  test_mac_parser();
  test_notification_parser();
  test_dfu_client_registration_is_one_shot();
  test_manifest_sections_and_size_tuples();
  test_callback_log_events_are_owned_and_deferred();
  test_retry_policy();
  test_v4_strict_transition_address();
  puts("host tests: PASS");
  return 0;
}
