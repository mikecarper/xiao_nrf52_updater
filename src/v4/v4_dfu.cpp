#include "v4_dfu.h"

#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "dfu_image_layout.h"
#include "dfu_protocol.h"
#include "v4_log.h"
#include "v4_zip.h"

namespace v4_dfu {

namespace {

using dfu_legacy::CleanupOutcome;
using dfu_legacy::Result;

constexpr const char* kServiceUuid = "00001530-1212-efde-1523-785feabcd123";
constexpr const char* kControlUuid = "00001531-1212-efde-1523-785feabcd123";
constexpr const char* kPacketUuid = "00001532-1212-efde-1523-785feabcd123";
constexpr const char* kVersionUuid = "00001534-1212-efde-1523-785feabcd123";

constexpr uint8_t kOpStart = 0x01;
constexpr uint8_t kOpInit = 0x02;
constexpr uint8_t kOpReceive = 0x03;
constexpr uint8_t kOpValidate = 0x04;
constexpr uint8_t kOpActivate = 0x05;
constexpr uint8_t kOpReset = 0x06;
constexpr uint8_t kOpPrn = 0x08;
constexpr uint8_t kStatusSuccess = 0x01;

constexpr uint16_t kPrepInterval = 160;      // 200 ms, in 1.25 ms units
constexpr uint16_t kTransferInterval = 16;  // 20 ms
constexpr uint16_t kLatency = 0;
constexpr uint16_t kSupervisionTimeout = 3200;  // 32 s, in 10 ms units
constexpr uint32_t kStartSettleMs = 600;
constexpr uint32_t kLinkSettleMs = 600;
constexpr uint32_t kParamUpdateDeadlineMs = 5000;
constexpr uint32_t kParamApplyWindowMs = 1500;
constexpr uint32_t kParamBusyRetryMs = 150;

struct Notification {
  uint8_t length;
  uint8_t data[20];
};

constexpr size_t kResponseQueueDepth = 4;
constexpr size_t kPrnQueueDepth = 8;
StaticQueue_t s_response_queue_state;
StaticQueue_t s_prn_queue_state;
uint8_t s_response_queue_storage[kResponseQueueDepth * sizeof(Notification)];
uint8_t s_prn_queue_storage[kPrnQueueDepth * sizeof(Notification)];
QueueHandle_t s_response_queue = nullptr;
QueueHandle_t s_prn_queue = nullptr;
volatile uint32_t s_notification_drops = 0;
volatile uint32_t s_unknown_notifications = 0;

NimBLEClient* s_client = nullptr;
NimBLERemoteCharacteristic* s_control = nullptr;
NimBLERemoteCharacteristic* s_packet = nullptr;
NimBLERemoteCharacteristic* s_version = nullptr;
volatile bool s_connected = false;
bool s_control_usable = false;
ProgressCallback s_progress_callback = nullptr;

class ClientCallbacks final : public NimBLEClientCallbacks {
 public:
  void onConnect(NimBLEClient*) override { s_connected = true; }
  void onDisconnect(NimBLEClient*) override { s_connected = false; }
};

ClientCallbacks s_client_callbacks;

void on_control_notification(NimBLERemoteCharacteristic*, uint8_t* data,
                             size_t length, bool) {
  QueueHandle_t destination = nullptr;
  switch (dfu_protocol::classify(data, length)) {
    case dfu_protocol::NotificationKind::kResponse:
      destination = s_response_queue;
      break;
    case dfu_protocol::NotificationKind::kPacketReceipt:
      destination = s_prn_queue;
      break;
    case dfu_protocol::NotificationKind::kUnknown:
      ++s_unknown_notifications;
      return;
  }
  Notification notification = {};
  if (length > sizeof(notification.data)) length = sizeof(notification.data);
  notification.length = static_cast<uint8_t>(length);
  memcpy(notification.data, data, length);
  if (!destination || xQueueSend(destination, &notification, 0) != pdPASS) {
    ++s_notification_drops;
  }
}

bool reset_queues() {
  if (!s_response_queue) {
    s_response_queue = xQueueCreateStatic(
        kResponseQueueDepth, sizeof(Notification), s_response_queue_storage,
        &s_response_queue_state);
  }
  if (!s_prn_queue) {
    s_prn_queue = xQueueCreateStatic(
        kPrnQueueDepth, sizeof(Notification), s_prn_queue_storage,
        &s_prn_queue_state);
  }
  if (!s_response_queue || !s_prn_queue) return false;
  xQueueReset(s_response_queue);
  xQueueReset(s_prn_queue);
  s_notification_drops = 0;
  s_unknown_notifications = 0;
  return true;
}

bool wait_disconnected(uint32_t timeout_ms) {
  uint32_t started_at = millis();
  while (s_connected &&
         static_cast<uint32_t>(millis() - started_at) < timeout_ms) {
    delay(20);
  }
  return !s_connected;
}

bool release_client() {
  s_control = nullptr;
  s_packet = nullptr;
  s_version = nullptr;
  s_control_usable = false;
  if (!s_client) {
    s_connected = false;
    return true;
  }
  if (s_client->isConnected()) {
    s_client->disconnect();
    wait_disconnected(2000);
  }
  if (!NimBLEDevice::deleteClient(s_client)) {
    v4_log::line("dfu: client deletion failed; refusing a replacement client");
    s_connected = s_client->isConnected();
    return false;
  }
  s_client = nullptr;
  s_connected = false;
  return true;
}

RunResult finish(Result result,
                 CleanupOutcome cleanup = CleanupOutcome::kNotNeeded) {
  release_client();
  return {result, cleanup};
}

bool subscribe_control() {
  if (!s_control || !s_control->canNotify() ||
      !s_control->getDescriptor(NimBLEUUID(static_cast<uint16_t>(0x2902)))) {
    v4_log::line("dfu: notification CCCD missing");
    return false;
  }
  return s_control->subscribe(true, on_control_notification, true);
}

bool link_parameters_match(uint16_t interval) {
  if (!s_client || !s_client->isConnected()) return false;
  NimBLEConnInfo info = s_client->getConnInfo();
  return info.getConnInterval() == interval &&
         info.getConnLatency() == kLatency &&
         info.getConnTimeout() == kSupervisionTimeout;
}

bool request_link_parameters(uint16_t interval, const char* phase) {
  if (!s_client || !s_client->isConnected()) return false;
  NimBLEConnInfo before = s_client->getConnInfo();
  v4_log::line("dfu: %s params current interval=%u latency=%u timeout=%u ms",
               phase, before.getConnInterval(), before.getConnLatency(),
               before.getConnTimeout() * 10u);
  if (link_parameters_match(interval)) return true;

  // Keep NimBLEClient's preferred tuple in sync for a peer-initiated update,
  // but submit directly so NimBLE 1.4.x cannot hide EALREADY/EBUSY/errors
  // behind its void updateConnParams() API.
  s_client->setConnectionParams(interval, interval, kLatency,
                                kSupervisionTimeout);
  ble_gap_upd_params params = {};
  params.itvl_min = interval;
  params.itvl_max = interval;
  params.latency = kLatency;
  params.supervision_timeout = kSupervisionTimeout;
  params.min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN;
  params.max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN;

  uint32_t started_at = millis();
  uint32_t next_submit_at = started_at;
  while (s_connected &&
         static_cast<uint32_t>(millis() - started_at) <
             kParamUpdateDeadlineMs) {
    if (link_parameters_match(interval)) {
      NimBLEConnInfo after = s_client->getConnInfo();
      v4_log::line("dfu: %s params applied interval=%u latency=%u timeout=%u ms",
                   phase, after.getConnInterval(), after.getConnLatency(),
                   after.getConnTimeout() * 10u);
      return true;
    }

    uint32_t now = millis();
    if (static_cast<int32_t>(now - next_submit_at) >= 0) {
      int rc = ble_gap_update_params(s_client->getConnId(), &params);
      if (rc == 0) {
        v4_log::line("dfu: %s parameter request submitted", phase);
        next_submit_at = now + kParamApplyWindowMs;
      } else if (rc == BLE_HS_EALREADY || rc == BLE_HS_EBUSY) {
        next_submit_at = now + kParamBusyRetryMs;
      } else {
        v4_log::line("dfu: %s parameter request failed rc=%d", phase, rc);
        return false;
      }
    }
    delay(20);
  }
  v4_log::line("dfu: %s parameter update not observed", phase);
  return false;
}

uint8_t consume_response(uint8_t expected_op, uint32_t timeout_ms = 15000) {
  uint32_t started_at = millis();
  while (static_cast<uint32_t>(millis() - started_at) < timeout_ms) {
    if (s_notification_drops != 0) {
      v4_log::line("dfu: notification overflow (%lu dropped)",
                   static_cast<unsigned long>(s_notification_drops));
      return 0xff;
    }
    Notification notification;
    if (xQueueReceive(s_response_queue, &notification, 0) == pdPASS) {
      uint8_t operation = 0;
      uint8_t status = 0;
      if (!dfu_protocol::decode_response(notification.data,
                                         notification.length,
                                         &operation, &status) ||
          operation != expected_op) {
        v4_log::line("dfu: unexpected response op=%02X expected=%02X",
                     operation, expected_op);
        return 0xff;
      }
      return status;
    }
    if (!s_connected) break;
    delay(10);
  }
  v4_log::line("dfu: response timeout op=%02X unknown=%lu", expected_op,
               static_cast<unsigned long>(s_unknown_notifications));
  return 0xff;
}

enum class PrnWait : uint8_t {
  kReceived,
  kFinalResponseReady,
  kTimeout,
  kInvalid,
};

PrnWait wait_for_prn(uint32_t expected_bytes, bool final_packet,
                     uint32_t timeout_ms) {
  uint32_t started_at = millis();
  while (static_cast<uint32_t>(millis() - started_at) < timeout_ms) {
    if (s_notification_drops != 0) return PrnWait::kInvalid;
    Notification notification;
    if (xQueueReceive(s_prn_queue, &notification, 0) == pdPASS) {
      uint32_t received = 0;
      if (!dfu_protocol::decode_prn(notification.data, notification.length,
                                    &received) ||
          received != expected_bytes) {
        v4_log::line("dfu: PRN mismatch sent=%lu peer=%lu",
                     static_cast<unsigned long>(expected_bytes),
                     static_cast<unsigned long>(received));
        return PrnWait::kInvalid;
      }
      return PrnWait::kReceived;
    }
    if (xQueuePeek(s_response_queue, &notification, 0) == pdPASS) {
      return final_packet ? PrnWait::kFinalResponseReady : PrnWait::kInvalid;
    }
    if (!s_connected) break;
    delay(10);
  }
  return PrnWait::kTimeout;
}

RunResult fail(Result result) {
  if (!s_connected) return finish(result, CleanupOutcome::kLinkAlreadyLost);
  if (!s_control_usable || !s_control) {
    s_client->disconnect();
    wait_disconnected(2000);
    return finish(result, CleanupOutcome::kForcedDisconnect);
  }

  bool restored = request_link_parameters(kTransferInterval, "recovery");
  if (!restored) {
    v4_log::line("dfu: recovery params not observed; RESET at current tuple");
  }
  if (!s_connected) return finish(result, CleanupOutcome::kLinkAlreadyLost);

  // NimBLE's acknowledged write blocks for completion (or disconnect), so the
  // Bluefruit backend's 100 ms write restriction does not apply here. RESET is
  // always attempted once while the control point and link are usable.
  uint8_t reset = kOpReset;
  bool wrote = s_control->writeValue(&reset, 1, true);
  v4_log::line("dfu: RESET write=%d", static_cast<int>(wrote));
  if (wait_disconnected(3000)) {
    return finish(result, CleanupOutcome::kResetObserved);
  }
  v4_log::line("dfu: RESET did not drop link; forcing disconnect");
  s_client->disconnect();
  wait_disconnected(2000);
  return finish(result, CleanupOutcome::kForcedDisconnect);
}

bool write_control(const uint8_t* data, size_t length, const char* label) {
  if (!s_control || !s_connected ||
      !s_control->writeValue(data, length, true)) {
    v4_log::line("dfu: %s control write failed", label);
    return false;
  }
  return true;
}

bool write_packet(const uint8_t* data, size_t length) {
  return s_packet && s_connected && s_packet->writeValue(data, length, false);
}

}  // namespace

void set_progress_callback(ProgressCallback callback) {
  s_progress_callback = callback;
}

RunResult run(const v4_scanner::Target& target,
              const v4_bundle::Parsed& bundle,
              const v4_config::Config& config,
              bool allow_buttonless) {
  if (!release_client()) {
    return {Result::kRemoteError, CleanupOutcome::kForcedDisconnect};
  }
  if (!reset_queues()) return finish(Result::kRemoteError);

  char address_text[18];
  v4_scanner::format_address(target, address_text);
  NimBLEAddress address(address_text, target.address_type);
  s_client = NimBLEDevice::createClient(address);
  if (!s_client) return finish(Result::kConnectFailed);
  s_client->setClientCallbacks(&s_client_callbacks, false);
  s_client->setConnectTimeout(10);
  // Keep the initial app/boot connection conventional. The destructive
  // phases below explicitly request and verify the 32 s supervision timeout.
  s_client->setConnectionParams(kTransferInterval, kTransferInterval,
                                kLatency, 400);
  s_connected = false;
  s_control_usable = false;

  v4_log::line("dfu: connecting %s type=%u", address_text,
               target.address_type);
  // deleteAttributes=true forces fresh service/characteristic discovery on
  // each app/bootloader leg; no GATT handles cross the reboot boundary.
  if (!s_client->connect(address, true)) return finish(Result::kConnectFailed);
  s_connected = s_client->isConnected();
  delay(300);
  if (!s_connected || !s_client->isConnected()) {
    return finish(Result::kConnectFailed);
  }

  uint16_t mtu = s_client->getMTU();
  uint16_t payload = config.high_mtu && mtu > 23 ? mtu - 3 : 20;
  if (payload > 244) payload = 244;
  if (payload < 20) payload = 20;
  v4_log::line("dfu: MTU=%u packet-payload=%u", mtu, payload);

  NimBLERemoteService* service = s_client->getService(kServiceUuid);
  if (!service) {
    v4_log::line("dfu: Legacy service missing");
    return fail(Result::kServiceMissing);
  }
  s_control = service->getCharacteristic(kControlUuid);
  s_packet = service->getCharacteristic(kPacketUuid);
  s_version = service->getCharacteristic(kVersionUuid);
  s_control_usable = s_control != nullptr;
  v4_log::line("dfu: chars control=%d packet=%d version=%d",
               s_control != nullptr, s_packet != nullptr,
               s_version != nullptr);
  if (!s_control) return fail(Result::kCharMissing);

  uint16_t version = 0;
  if (s_version && s_version->canRead()) {
    NimBLEAttValue value = s_version->readValue();
    if (value.size() >= 2) {
      version = static_cast<uint16_t>(value[0]) |
                (static_cast<uint16_t>(value[1]) << 8);
      v4_log::line("dfu: version=%u.%u raw=%04X", version >> 8,
                   version & 0xff, version);
    }
  }

  bool app_mode = (s_version && version == 0x0001) || !s_packet;
  if (app_mode) {
    if (!allow_buttonless) {
      v4_log::line("dfu: app still present after buttonless transition limit");
      return finish(Result::kButtonlessLimit);
    }
    v4_log::line("dfu: application mode; sending buttonless trigger");
    if (!subscribe_control()) {
      return fail(Result::kCharMissing);
    }
    delay(200);
    const uint8_t enter_bootloader[] = {0x01, 0x04};
    // A false return is allowed when the peer resets before acknowledging.
    s_control->writeValue(enter_bootloader, sizeof(enter_bootloader), true);
    if (!wait_disconnected(5000)) {
      v4_log::line("dfu: buttonless peer did not disconnect");
      s_client->disconnect();
      wait_disconnected(2000);
      return finish(Result::kConnectFailed,
                    CleanupOutcome::kForcedDisconnect);
    }
    return finish(Result::kButtonlessTriggered);
  }

  if (!subscribe_control()) {
    v4_log::line("dfu: CCCD subscription failed");
    return fail(Result::kCharMissing);
  }
  delay(200);

  const uint8_t start[] = {kOpStart, bundle.type};
  if (!write_control(start, sizeof(start), "START")) {
    return fail(Result::kDisconnectedEarly);
  }
  v4_log::line("dfu: START type=%02X", bundle.type);
  if (!request_link_parameters(kPrepInterval, "flash-prep")) {
    return fail(Result::kConnectFailed);
  }
  delay(kStartSettleMs);

  uint8_t sizes[12] = {};
  dfu_image_layout::Layout layout = {
      bundle.type, bundle.sd_size, bundle.bl_size, bundle.app_size};
  if (!dfu_image_layout::encode_size_tuple(layout, sizes, sizeof(sizes))) {
    return fail(Result::kFsError);
  }
  if (!write_packet(sizes, sizeof(sizes))) {
    v4_log::line("dfu: size tuple write failed");
    return fail(Result::kDisconnectedEarly);
  }
  v4_log::line("dfu: sizes sd=%lu bl=%lu app=%lu",
               static_cast<unsigned long>(bundle.sd_size),
               static_cast<unsigned long>(bundle.bl_size),
               static_cast<unsigned long>(bundle.app_size));
  uint8_t status = consume_response(kOpStart, 60000);
  v4_log::line("dfu: START response=%02X", status);
  if (status != kStatusSuccess) return fail(Result::kRemoteError);
  delay(600);

  if (!request_link_parameters(kTransferInterval, "transfer")) {
    return fail(Result::kConnectFailed);
  }
  delay(kLinkSettleMs);

  const uint8_t init_start[] = {kOpInit, 0x00};
  const uint8_t init_complete[] = {kOpInit, 0x01};
  if (!write_control(init_start, sizeof(init_start), "INIT start")) {
    return fail(Result::kDisconnectedEarly);
  }
  uint8_t init_chunk[20];
  uint32_t init_offset = 0;
  while (init_offset < bundle.dat.size) {
    uint32_t want = bundle.dat.size - init_offset;
    if (want > sizeof(init_chunk)) want = sizeof(init_chunk);
    int n = v4_zip::read(bundle.dat, init_offset, init_chunk, want);
    if (n != static_cast<int>(want)) return fail(Result::kFsError);
    if (!write_packet(init_chunk, n)) {
      v4_log::line("dfu: init packet write failed at %lu",
                   static_cast<unsigned long>(init_offset));
      return fail(Result::kDisconnectedEarly);
    }
    init_offset += static_cast<uint32_t>(n);
  }
  v4_log::line("dfu: init packet sent (%lu B)",
               static_cast<unsigned long>(bundle.dat.size));
  delay(50);
  if (!write_control(init_complete, sizeof(init_complete), "INIT complete")) {
    return fail(Result::kDisconnectedEarly);
  }
  status = consume_response(kOpInit);
  v4_log::line("dfu: INIT response=%02X", status);
  if (status != kStatusSuccess) return fail(Result::kRemoteError);

  const uint16_t prn = config.prn;
  const uint8_t set_prn[] = {
      kOpPrn, static_cast<uint8_t>(prn), static_cast<uint8_t>(prn >> 8)};
  if (!write_control(set_prn, sizeof(set_prn), "PRN")) {
    return fail(Result::kDisconnectedEarly);
  }
  v4_log::line("dfu: PRN=%u", prn);

  const uint8_t receive[] = {kOpReceive};
  if (!write_control(receive, sizeof(receive), "RECEIVE")) {
    return fail(Result::kDisconnectedEarly);
  }
  if (!request_link_parameters(kPrepInterval, "stream")) {
    return fail(Result::kConnectFailed);
  }

  uint8_t firmware_chunk[244];
  uint32_t sent = 0;
  uint16_t packets_since_prn = 0;
  uint32_t next_log_percent = 5;
  uint32_t stream_started_at = millis();
  if (s_progress_callback) s_progress_callback(0);
  v4_log::line("dfu: streaming %lu B",
               static_cast<unsigned long>(bundle.bin.size));
  while (sent < bundle.bin.size) {
    uint32_t want = bundle.bin.size - sent;
    if (want > payload) want = payload;
    int n = v4_zip::read(bundle.bin, sent, firmware_chunk, want);
    if (n != static_cast<int>(want)) return fail(Result::kFsError);

    uint16_t attempts = 0;
    while (!write_packet(firmware_chunk, n)) {
      if (!s_connected || ++attempts > 200) {
        v4_log::line("dfu: packet stalled at %lu",
                     static_cast<unsigned long>(sent));
        return fail(Result::kDisconnectedEarly);
      }
      delay(5);
    }
    sent += static_cast<uint32_t>(n);
    ++packets_since_prn;

    if (prn > 0 && packets_since_prn >= prn) {
      packets_since_prn = 0;
      PrnWait prn_result =
          wait_for_prn(sent, sent == bundle.bin.size, 5000);
      if (prn_result == PrnWait::kTimeout) return fail(Result::kTimeout);
      if (prn_result == PrnWait::kInvalid) return fail(Result::kRemoteError);
    }

    uint32_t percent = static_cast<uint32_t>(
        static_cast<uint64_t>(sent) * 100u / bundle.bin.size);
    if (s_progress_callback)
      s_progress_callback(static_cast<uint8_t>(percent));
    if (percent >= next_log_percent) {
      v4_log::line("dfu: progress %lu%% (%lu/%lu)",
                   static_cast<unsigned long>(percent),
                   static_cast<unsigned long>(sent),
                   static_cast<unsigned long>(bundle.bin.size));
      next_log_percent = percent + 5;
    }
  }
  if (s_progress_callback) s_progress_callback(100);
  uint32_t elapsed = millis() - stream_started_at;
  v4_log::line("dfu: stream complete %lu ms (%lu B/s)",
               static_cast<unsigned long>(elapsed),
               elapsed ? static_cast<unsigned long>(
                             static_cast<uint64_t>(bundle.bin.size) * 1000u /
                             elapsed)
                       : 0ul);

  status = consume_response(kOpReceive);
  v4_log::line("dfu: RECEIVE response=%02X", status);
  if (status != kStatusSuccess) return fail(Result::kRemoteError);

  if (!request_link_parameters(kTransferInterval, "finalize")) {
    return fail(Result::kConnectFailed);
  }
  delay(kLinkSettleMs);
  const uint8_t validate[] = {kOpValidate};
  if (!write_control(validate, sizeof(validate), "VALIDATE")) {
    return fail(Result::kDisconnectedEarly);
  }
  status = consume_response(kOpValidate);
  v4_log::line("dfu: VALIDATE response=%02X", status);
  if (status != kStatusSuccess) return fail(Result::kRemoteError);

  const uint8_t activate[] = {kOpActivate};
  s_control->writeValue(activate, sizeof(activate), true);
  v4_log::line("dfu: ACTIVATE sent; waiting for reset");
  if (!wait_disconnected(120000)) {
    v4_log::line("dfu: ACTIVATE timeout; forcing disconnect");
    s_client->disconnect();
    wait_disconnected(3000);
    return finish(Result::kTimeout, CleanupOutcome::kForcedDisconnect);
  }
  v4_log::line("dfu: DONE");
  return finish(Result::kOk);
}

}  // namespace v4_dfu
