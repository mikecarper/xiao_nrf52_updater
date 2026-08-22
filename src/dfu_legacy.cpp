#include "dfu_legacy.h"

#include <FreeRTOS.h>
#include <bluefruit.h>
#include <queue.h>

#include "dfu_client_lifecycle.h"
#include "dfu_protocol.h"
#include "deferred_logger.h"
#include "logger.h"
#include "zip_reader.h"

namespace dfu_legacy {

// ---------- Nordic Legacy DFU UUIDs ----------
//   Service        00001530-1212-EFDE-1523-785FEABCD123
//   Control Point  00001531-1212-EFDE-1523-785FEABCD123
//   Packet         00001532-1212-EFDE-1523-785FEABCD123
//   Version        00001534-1212-EFDE-1523-785FEABCD123
static const uint8_t kSvcUuid[16]  = { 0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,0xDE,0xEF,0x12,0x12,0x30,0x15,0x00,0x00 };
static const uint8_t kCtrlUuid[16] = { 0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,0xDE,0xEF,0x12,0x12,0x31,0x15,0x00,0x00 };
static const uint8_t kPktUuid[16]  = { 0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,0xDE,0xEF,0x12,0x12,0x32,0x15,0x00,0x00 };
static const uint8_t kVerUuid[16]  = { 0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,0xDE,0xEF,0x12,0x12,0x34,0x15,0x00,0x00 };

// ---------- Opcodes (mirrors LegacyDfuImpl.java) ----------
static constexpr uint8_t OP_START_DFU                = 0x01;
static constexpr uint8_t OP_INIT_DFU_PARAMS          = 0x02;
static constexpr uint8_t OP_RECEIVE_FW               = 0x03;
static constexpr uint8_t OP_VALIDATE                 = 0x04;
static constexpr uint8_t OP_ACTIVATE_AND_RESET       = 0x05;
static constexpr uint8_t OP_RESET                    = 0x06;
static constexpr uint8_t OP_PKT_RECEIPT_NOTIF_REQ    = 0x08;
static constexpr uint8_t STATUS_SUCCESS              = 0x01;

// Bluefruit Central defaults to a 20 ms interval and 2 s supervision timeout.
// A Nordic page erase needs roughly 85--90 ms of uninterrupted flash time.
// Use 20 ms for acknowledged control writes (Bluefruit waits only 100 ms for
// their ATT response) and 200 ms for destructive/lazy-erase phases, including
// firmware streaming. This avoids relying on a target-local latency hint whose
// actual value is not observable by the central.
static constexpr uint16_t DFU_PREP_INTERVAL_UNITS       = 160;  // 200 ms
static constexpr uint16_t DFU_TRANSFER_INTERVAL_UNITS   = 16;   // 20 ms
static constexpr uint16_t DFU_SLAVE_LATENCY             = 0;
static constexpr uint16_t DFU_SUPERVISION_TIMEOUT_10MS = 3200;  // 32 s (BLE maximum)

// recrof/nrf_dfu_py uses a configurable compatibility delay (0.4 s by
// default) between the acknowledged START write and the 12-byte size tuple.
// Keep a slightly larger margin for embedded central/legacy-target pairs.
static constexpr uint32_t START_TO_SIZES_SETTLE_MS = 600;

// A live run saw the first GATT WRITE_REQ fail immediately after the new GAP
// parameters became visible to the central.  Give the link several connection
// events to settle before the first INIT control write.  This is an empirical
// compatibility guard: Bluefruit's write_resp() does not distinguish an
// immediate submission error from its fixed 100 ms response timeout.
static constexpr uint32_t LINK_UPDATE_TO_GATT_SETTLE_MS = 600;

// ---------- Globals ----------
static BLEUuid                  s_svc_uuid (kSvcUuid);
static BLEUuid                  s_ctrl_uuid(kCtrlUuid);
static BLEUuid                  s_pkt_uuid (kPktUuid);
static BLEUuid                  s_ver_uuid (kVerUuid);
static BLEClientService         s_svc (s_svc_uuid);
static BLEClientCharacteristic  s_ctrl(s_ctrl_uuid);
static BLEClientCharacteristic  s_pkt (s_pkt_uuid);
static BLEClientCharacteristic  s_ver (s_ver_uuid);
static dfu_client_lifecycle::RegistrationState s_client_registration;

static volatile bool     s_connected   = false;
static volatile uint16_t s_conn_handle = BLE_CONN_HANDLE_INVALID;
static bool              s_ctrl_usable = false;

struct Notification {
  uint8_t len;
  uint8_t data[20];
};

static constexpr size_t kResponseQueueDepth = 4;
static constexpr size_t kPrnQueueDepth = 8;
static StaticQueue_t s_response_queue_control;
static StaticQueue_t s_prn_queue_control;
static uint8_t s_response_queue_storage[kResponseQueueDepth * sizeof(Notification)];
static uint8_t s_prn_queue_storage[kPrnQueueDepth * sizeof(Notification)];
static QueueHandle_t s_response_queue = nullptr;
static QueueHandle_t s_prn_queue = nullptr;
static volatile uint32_t s_notification_drops = 0;
static volatile uint32_t s_unknown_notifications = 0;

static ProgressCb        s_progress_cb = nullptr;
void set_progress_callback(ProgressCb cb) { s_progress_cb = cb; }

// ---------- Callbacks ----------
static void on_connect(uint16_t conn_handle) {
  s_conn_handle = conn_handle;
  deferred_logger::post_ble_connected(conn_handle);
  // Publish the state only after the event is queued, so the Arduino task can
  // always drain the corresponding log before moving past wait_connected().
  s_connected   = true;
}

static void on_disconnect(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  deferred_logger::post_ble_disconnected(reason);
  s_conn_handle = BLE_CONN_HANDLE_INVALID;
  // Publish the state last for the same ordering guarantee as on_connect().
  s_connected   = false;
}

static void on_ctrl_notify(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)chr;
  QueueHandle_t destination = nullptr;
  switch (dfu_protocol::classify(data, len)) {
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
  if (len > sizeof(notification.data)) len = sizeof(notification.data);
  notification.len = static_cast<uint8_t>(len);
  memcpy(notification.data, data, len);

  // Bluefruit normally dispatches this through its callback task. A static
  // FreeRTOS queue makes producer/consumer synchronization explicit and keeps
  // back-to-back PRN + final-response notifications distinct.
  if (!destination || xQueueSend(destination, &notification, 0) != pdPASS) {
    ++s_notification_drops;
  }
}

// ---------- Helpers ----------
static RunResult make_result(Result result,
                             CleanupOutcome cleanup = CleanupOutcome::kNotNeeded) {
  deferred_logger::drain();
  return {result, cleanup};
}

static bool reset_notification_queues() {
  if (!s_response_queue) {
    s_response_queue = xQueueCreateStatic(
        kResponseQueueDepth, sizeof(Notification), s_response_queue_storage,
        &s_response_queue_control);
  }
  if (!s_prn_queue) {
    s_prn_queue = xQueueCreateStatic(
        kPrnQueueDepth, sizeof(Notification), s_prn_queue_storage,
        &s_prn_queue_control);
  }
  if (!s_response_queue || !s_prn_queue) return false;
  xQueueReset(s_response_queue);
  xQueueReset(s_prn_queue);
  s_notification_drops = 0;
  s_unknown_notifications = 0;
  return true;
}

static bool initialize_clients_once() {
  if (s_client_registration.initialized()) return true;

  // Bluefruit's client begin() calls append-only _addService() and
  // _addCharacteristic(). run() is entered once in application mode and then
  // again in bootloader mode; registering here on every leg makes each HVX
  // event dispatch repeatedly and also grows the fixed GATT client lists.
  if (!s_svc.begin()) return false;
  s_ctrl.begin(&s_svc);
  s_pkt.begin(&s_svc);
  s_ver.begin(&s_svc);
  s_client_registration.mark_initialized();
  return true;
}

static bool wait_connected(uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  while (!s_connected && (int32_t)(deadline - millis()) > 0) {
    deferred_logger::drain();
    delay(20);
  }
  deferred_logger::drain();
  return s_connected;
}

static bool wait_disconnected(uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  while (s_connected && (int32_t)(deadline - millis()) > 0) {
    deferred_logger::drain();
    delay(20);
  }
  deferred_logger::drain();
  return !s_connected;
}

static bool request_link_parameters(uint16_t interval_units, const char* phase,
                                    bool require_long_supervision = true) {
  BLEConnection* conn = Bluefruit.Connection(s_conn_handle);
  if (!conn) return false;
  logger::log("dfu: %s params current interval_units=%u latency=%u supervision=%u ms",
              phase,
              conn->getConnectionInterval(), conn->getSlaveLatency(),
              conn->getSupervisionTimeout() * 10u);
  auto parameters_match = [interval_units, require_long_supervision](BLEConnection* current) {
    return current && current->getConnectionInterval() == interval_units &&
           current->getSlaveLatency() == DFU_SLAVE_LATENCY &&
           (!require_long_supervision ||
            current->getSupervisionTimeout() >= DFU_SUPERVISION_TIMEOUT_10MS);
  };
  if (parameters_match(conn)) {
    return true;
  }

  // The wrapper collapses transient SoftDevice errors such as NRF_ERROR_BUSY
  // to false. Retry submission for a bounded interval instead of aborting on
  // one busy connection event.
  bool submitted = false;
  uint32_t submit_deadline = millis() + 1000;
  while (s_connected && (int32_t)(submit_deadline - millis()) > 0) {
    deferred_logger::drain();
    BLEConnection* current = Bluefruit.Connection(s_conn_handle);
    if (current && current->requestConnectionParameter(
                       interval_units, DFU_SLAVE_LATENCY,
                       DFU_SUPERVISION_TIMEOUT_10MS)) {
      submitted = true;
      break;
    }
    delay(50);
  }
  if (!submitted) {
    logger::log("dfu: %s parameter update submission failed", phase);
    return false;
  }

  uint32_t deadline = millis() + 4000;
  while (s_connected && (int32_t)(deadline - millis()) > 0) {
    deferred_logger::drain();
    // A disconnect can retire the Bluefruit connection object. Reacquire it
    // after every delay instead of retaining a possibly stale pointer.
    BLEConnection* current = Bluefruit.Connection(s_conn_handle);
    if (parameters_match(current)) {
      logger::log("dfu: %s params updated interval_units=%u latency=%u supervision=%u ms",
                  phase,
                  current->getConnectionInterval(),
                  current->getSlaveLatency(),
                  current->getSupervisionTimeout() * 10u);
      return true;
    }
    delay(20);
  }
  logger::log("dfu: %s parameter update did not take effect", phase);
  return false;
}

// Validate a control-point response notification. Expected layout: [0x10, <op>, <status>].
// Returns the status byte, or 0xFF on protocol error.
static uint8_t consume_response(uint8_t expected_op, uint32_t timeout_ms = 15000) {
  uint32_t deadline = millis() + timeout_ms;
  while ((int32_t)(deadline - millis()) > 0) {
    deferred_logger::drain();
    if (s_notification_drops != 0) {
      logger::log("dfu: notification queue overflow (%lu dropped)",
                  (unsigned long)s_notification_drops);
      return 0xFF;
    }

    Notification notification;
    if (xQueueReceive(s_response_queue, &notification, 0) == pdPASS) {
      uint8_t op = 0;
      uint8_t status = 0;
      if (!dfu_protocol::decode_response(notification.data, notification.len,
                                         &op, &status) || op != expected_op) {
        logger::log("dfu: unexpected response op=0x%02x len=%u expected=0x%02x",
                    op, notification.len, expected_op);
        return 0xFF;
      }
      return status;
    }
    if (!s_connected) break;
    delay(20);
  }
  logger::log("dfu: response timeout expected=0x%02x unknown=%lu",
              expected_op, (unsigned long)s_unknown_notifications);
  return 0xFF;
}

enum class PrnWaitResult : uint8_t {
  kReceived,
  kFinalResponseReady,
  kTimeout,
  kInvalid,
};

static PrnWaitResult wait_for_prn(uint32_t expected_bytes,
                                  bool final_packet,
                                  uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  while ((int32_t)(deadline - millis()) > 0) {
    deferred_logger::drain();
    if (s_notification_drops != 0) return PrnWaitResult::kInvalid;

    Notification notification;
    if (xQueueReceive(s_prn_queue, &notification, 0) == pdPASS) {
      uint32_t peer_received = 0;
      if (!dfu_protocol::decode_prn(notification.data, notification.len,
                                    &peer_received)) {
        return PrnWaitResult::kInvalid;
      }
      if (peer_received != expected_bytes) {
        logger::log("dfu: PRN mismatch sent=%lu peer=%lu",
                    (unsigned long)expected_bytes,
                    (unsigned long)peer_received);
        return PrnWaitResult::kInvalid;
      }
      return PrnWaitResult::kReceived;
    }

    // Some legacy targets emit the final RECEIVE response without a final PRN
    // when both would coincide. Keep the response queued for consume_response.
    if (xQueuePeek(s_response_queue, &notification, 0) == pdPASS) {
      return final_packet ? PrnWaitResult::kFinalResponseReady
                          : PrnWaitResult::kInvalid;
    }
    if (!s_connected) break;
    delay(20);
  }
  return PrnWaitResult::kTimeout;
}

// Send RESET (0x06) and let the bootloader tear down the link itself. This
// is the safe exit on any error: it tells the bootloader to throw away
// whatever DFU state it accumulated so the *next* attempt starts clean
// instead of getting INVALID_STATE back on its very first START_DFU.
//
// We do NOT initiate the disconnect ourselves — RESET is processed
// asynchronously and the bootloader needs time to act on it before it
// reboots. If we disconnect first the bootloader may just terminate the
// link without ever running its RESET handler, leaving the half-finished
// DFU state in place and the next attempt will hit the same wall.
static RunResult fail(Result r) {
  if (!s_connected) return make_result(r, CleanupOutcome::kLinkAlreadyLost);

  // Service/characteristic discovery may fail before RESET is addressable.
  if (!s_ctrl_usable) {
    Bluefruit.disconnect(s_conn_handle);
    wait_disconnected(2000);
    return make_result(r, CleanupOutcome::kForcedDisconnect);
  }

  // Bluefruit write_resp() has a fixed 100 ms wait. Never issue RESET at the
  // 200 ms erase interval: first restore 20 ms and let the updated link settle.
  if (!request_link_parameters(DFU_TRANSFER_INTERVAL_UNITS, "recovery", false)) {
    logger::log("dfu: cannot restore 20 ms link; RESET not sent");
    if (s_connected) {
      Bluefruit.disconnect(s_conn_handle);
      wait_disconnected(2000);
    }
    return make_result(r, CleanupOutcome::kForcedDisconnect);
  }
  delay(LINK_UPDATE_TO_GATT_SETTLE_MS);
  if (!s_connected) return make_result(r, CleanupOutcome::kLinkAlreadyLost);

  // Re-read the live parameters after the settle window. This also catches a
  // late completion from an older 200 ms request which raced the recovery
  // request; RESET must never rely only on a stale cached 20 ms value.
  BLEConnection* recovery_conn = Bluefruit.Connection(s_conn_handle);
  if (!recovery_conn ||
      recovery_conn->getConnectionInterval() != DFU_TRANSFER_INTERVAL_UNITS ||
      recovery_conn->getSlaveLatency() != DFU_SLAVE_LATENCY) {
    logger::log("dfu: recovery link drifted; retrying 20 ms request");
    if (!request_link_parameters(DFU_TRANSFER_INTERVAL_UNITS, "recovery-retry",
                                 false)) {
      if (s_connected) {
        Bluefruit.disconnect(s_conn_handle);
        wait_disconnected(2000);
      }
      return make_result(r, CleanupOutcome::kForcedDisconnect);
    }
    delay(LINK_UPDATE_TO_GATT_SETTLE_MS);
    recovery_conn = Bluefruit.Connection(s_conn_handle);
    if (!s_connected || !recovery_conn ||
        recovery_conn->getConnectionInterval() != DFU_TRANSFER_INTERVAL_UNITS ||
        recovery_conn->getSlaveLatency() != DFU_SLAVE_LATENCY) {
      if (s_connected) {
        Bluefruit.disconnect(s_conn_handle);
        wait_disconnected(2000);
      }
      return make_result(r, CleanupOutcome::kForcedDisconnect);
    }
  }

  uint8_t reset_cmd[1] = { OP_RESET };
  int reset_write = s_ctrl.write_resp(reset_cmd, sizeof(reset_cmd));
  logger::log("dfu: RESET write result=%d", reset_write);

  // Even a local 100 ms timeout can mean the request reached a peer which
  // reset before ATT acknowledged it. The observed peer disconnect is the
  // authoritative cleanup signal.
  if (wait_disconnected(3000)) {
    return make_result(r, CleanupOutcome::kResetObserved);
  }

  logger::log("dfu: peer did not drop link after RESET, forcing");
  Bluefruit.disconnect(s_conn_handle);
  wait_disconnected(2000);
  return make_result(r, CleanupOutcome::kForcedDisconnect);
}

// ---------- DFU run ----------
RunResult run(const ble_scanner::Target& target,
              const firmware_zip::Parsed& bundle,
              const config::Config& cfg) {
  Bluefruit.Central.setConnectCallback(on_connect);
  Bluefruit.Central.setDisconnectCallback(on_disconnect);

  s_ctrl.setNotifyCallback(on_ctrl_notify);
  if (!initialize_clients_once()) {
    logger::log("dfu: client registration failed");
    return make_result(Result::kRemoteError);
  }

  s_connected   = false;
  s_conn_handle = BLE_CONN_HANDLE_INVALID;
  s_ctrl_usable = false;
  if (!reset_notification_queues()) {
    logger::log("dfu: notification queue initialization failed");
    return make_result(Result::kRemoteError);
  }

  ble_gap_addr_t addr = target.addr;
  logger::log("dfu: connecting to %02X:%02X:%02X:%02X:%02X:%02X",
              addr.addr[5], addr.addr[4], addr.addr[3],
              addr.addr[2], addr.addr[1], addr.addr[0]);

  if (!Bluefruit.Central.connect(&addr)) return make_result(Result::kConnectFailed);
  if (!wait_connected(10000))            return make_result(Result::kConnectFailed);

  // Connection-stability gate. At low RSSI the LL handshake can complete just
  // long enough to fire on_connect, then immediately fail with reason 0x3E
  // (CONN_FAILED_TO_BE_ESTABLISHED). Without this short settle window we
  // march on through MTU exchange and service discovery on a dead link, and
  // misreport the failure as "DFU service not present" instead of "couldn't
  // hold a connection". 300 ms is well past the timing window in which the
  // LL aborts its own setup.
  delay(300);
  if (!s_connected) {
    logger::log("dfu: link dropped immediately after connect (weak signal?)");
    return make_result(Result::kConnectFailed);
  }

  // Optional MTU negotiation. nRF52840 SoftDevice supports up to 247.
  // Many Nordic Legacy DFU bootloaders honour the exchange and let us write
  // (mtu-3)-byte payloads to the Packet characteristic — typically 5–10x
  // faster end-to-end than the default 20 B writes.
  uint16_t payload = 20;
  if (cfg.high_mtu) {
    BLEConnection* conn = Bluefruit.Connection(s_conn_handle);
    if (conn) {
      conn->requestMtuExchange(247);
      delay(200);  // let the exchange complete
      if (!s_connected) {
        logger::log("dfu: link dropped during MTU exchange");
        return make_result(Result::kConnectFailed);
      }
      uint16_t mtu = conn->getMtu();
      payload = mtu > 3 ? mtu - 3 : 20;
      logger::log("dfu: MTU negotiated = %u (payload=%u)", mtu, payload);
    }
  }
  if (payload > 244) payload = 244;

  if (!s_svc.discover(s_conn_handle)) {
    logger::log("dfu: DFU service not present on peer");
    return fail(Result::kServiceMissing);
  }
  bool ctrl_ok = s_ctrl.discover();
  bool pkt_ok  = s_pkt.discover();
  bool ver_ok  = s_ver.discover();
  logger::log("dfu: chars present  ctrl=%d  packet=%d  version=%d",
              (int)ctrl_ok, (int)pkt_ok, (int)ver_ok);
  if (!ctrl_ok) {
    return fail(Result::kCharMissing);
  }
  s_ctrl_usable = true;

  // Read the DFU Version characteristic (little-endian uint16). Format is
  // hi-byte = major, lo-byte = minor:
  //   0x0001 = 0.1 = app with buttonless support — peer needs trigger
  //   0x0005+ = 0.5+ = real bootloader — proceed with full DFU
  uint16_t version = 0;
  if (ver_ok) {
    uint8_t verbuf[2] = {0, 0};
    s_ver.read(verbuf, 2);
    version = (uint16_t)verbuf[0] | ((uint16_t)verbuf[1] << 8);
    logger::log("dfu: peer DFU version = %u.%u  (raw 0x%04X)",
                version >> 8, version & 0xFF, version);
  }

  // Buttonless detection: prefer the version byte when present. Fall back
  // to "Packet characteristic missing" for older firmwares that don't
  // expose the Version characteristic at all.
  bool is_app_mode = (ver_ok && version == 0x0001) || !pkt_ok;
  if (is_app_mode) {
    logger::log("dfu: peer in app mode, sending buttonless trigger");
    if (!s_ctrl.enableNotify()) {
      Bluefruit.disconnect(s_conn_handle);
      wait_disconnected(2000);
      return make_result(Result::kCharMissing,
                         CleanupOutcome::kForcedDisconnect);
    }
    // Bluefruit implements CCCD enable as an asynchronous WRITE_CMD. Let it
    // reach the peer before the control write whose authorization checks it.
    delay(200);
    uint8_t enter_bl[2] = { 0x01, 0x04 };
    // write_resp may fail because the peer disconnects before sending the
    // ATT response; that's expected and not an error.
    s_ctrl.write_resp(enter_bl, sizeof(enter_bl));
    if (!wait_disconnected(5000)) {
      logger::log("dfu: buttonless peer did not close link, forcing disconnect");
      Bluefruit.disconnect(s_conn_handle);
      if (!wait_disconnected(2000)) {
        return make_result(Result::kConnectFailed,
                           CleanupOutcome::kForcedDisconnect);
      }
      return make_result(Result::kConnectFailed);
    }
    return make_result(Result::kButtonlessTriggered);
  }

  // Enable CCCD on the Control Point so the target's status notifications
  // come back as on_ctrl_notify() callbacks.
  if (!s_ctrl.enableNotify()) {
    logger::log("dfu: enableNotify() failed");
    return fail(Result::kCharMissing);
  }
  logger::log("dfu: notifications enabled");
  // See the app-mode note above: START_DFU is rejected when the CCCD write
  // has not reached the bootloader yet.
  delay(200);

  // -------------------- Start DFU + image sizes --------------------
  uint8_t start_cmd[2] = { OP_START_DFU, bundle.type };
  if (s_ctrl.write_resp(start_cmd, sizeof(start_cmd)) <= 0) {
    logger::log("dfu: Start DFU write failed");
    return fail(Result::kDisconnectedEarly);
  }
  logger::log("dfu: sent START_DFU type=0x%02x", bundle.type);

  // START itself needs Bluefruit's normal short interval because write_resp()
  // has a fixed 100 ms host timeout. Once the peer has acknowledged START,
  // create a real flash window before sending the destructive size tuple.
  if (!request_link_parameters(DFU_PREP_INTERVAL_UNITS, "flash-prep")) {
    return fail(Result::kConnectFailed);
  }

  delay(START_TO_SIZES_SETTLE_MS);

  // 3 × uint32 LE: SD size, BL size, App size. Per LegacyDfuImpl, even when a
  // field is unused it must still be present and zero.
  uint8_t sizes[12] = {0};
  const dfu_image_layout::Layout image_layout = {
      bundle.type, bundle.sd_size, bundle.bl_size, bundle.app_size};
  if (!dfu_image_layout::encode_size_tuple(image_layout, sizes,
                                           sizeof(sizes))) {
    logger::log("dfu: image size tuple encoding failed");
    return fail(Result::kFsError);
  }
  if (s_pkt.write(sizes, sizeof(sizes)) <= 0) {
    logger::log("dfu: image size write failed");
    return fail(Result::kDisconnectedEarly);
  }
  logger::log("dfu: sent sizes  sd=%lu bl=%lu app=%lu",
              (unsigned long)bundle.sd_size, (unsigned long)bundle.bl_size,
              (unsigned long)bundle.app_size);

  uint8_t status = consume_response(OP_START_DFU, 60000);
  logger::log("dfu: START_DFU response status=0x%02x", status);
  if (status != STATUS_SUCCESS) {
    return fail(Result::kRemoteError);
  }

  // Some legacy targets acknowledge START while follow-on flash/settings work
  // is still queued. Keep the erase-friendly interval briefly before changing
  // link timing again.
  delay(600);

  // Return to a short interval for the remaining acknowledged control writes.
  if (!request_link_parameters(DFU_TRANSFER_INTERVAL_UNITS, "transfer")) {
    return fail(Result::kConnectFailed);
  }
  delay(LINK_UPDATE_TO_GATT_SETTLE_MS);
  logger::log("dfu: transfer link settled for %lu ms",
              (unsigned long)LINK_UPDATE_TO_GATT_SETTLE_MS);

  // -------------------- Init packet (.dat) --------------------
  uint8_t init_start[2]    = { OP_INIT_DFU_PARAMS, 0x00 };
  uint8_t init_complete[2] = { OP_INIT_DFU_PARAMS, 0x01 };

  uint32_t init_start_at = millis();
  int init_start_result = s_ctrl.write_resp(init_start, sizeof(init_start));
  logger::log("dfu: INIT_DFU_PARAMS start write result=%d elapsed=%lu ms",
              init_start_result, (unsigned long)(millis() - init_start_at));
  if (init_start_result <= 0) {
    return fail(Result::kDisconnectedEarly);
  }

  // Stream the .dat in 20-byte chunks (default ATT MTU). The Wio Tracker bundle
  // has dat=14 bytes which fits in a single write, but loop the general case.
  uint8_t chunk[20];
  uint32_t off = 0;
  while (off < bundle.dat.size) {
    uint32_t want = bundle.dat.size - off;
    if (want > sizeof(chunk)) want = sizeof(chunk);
    int n = zip_reader::read(bundle.dat, off, chunk, want);
    if (n != (int)want) {
      logger::log("dfu: dat read short  off=%lu n=%d", (unsigned long)off, n);
      return fail(Result::kFsError);
    }
    if (s_pkt.write(chunk, n) <= 0) {
      logger::log("dfu: dat packet write failed at off=%lu", (unsigned long)off);
      return fail(Result::kDisconnectedEarly);
    }
    off += n;
  }
  logger::log("dfu: sent init packet (%lu B)", (unsigned long)bundle.dat.size);

  // Let the previous WRITE-without-response packets drain before we issue
  // another WRITE-with-response on the same characteristic. Without this the
  // SoftDevice sometimes rejects the queued WRITE_REQ.
  delay(50);

  int r = s_ctrl.write_resp(init_complete, sizeof(init_complete));
  if (r <= 0) {
    logger::log("dfu: INIT_DFU_PARAMS complete write_resp -> %d", r);
    return fail(Result::kDisconnectedEarly);
  }

  status = consume_response(OP_INIT_DFU_PARAMS);
  logger::log("dfu: INIT_DFU_PARAMS response status=0x%02x", status);
  if (status != STATUS_SUCCESS) {
    return fail(Result::kRemoteError);
  }

  // -------------------- PRN setup --------------------
  const uint16_t prn = cfg.prn;
  uint8_t prn_cmd[3] = { OP_PKT_RECEIPT_NOTIF_REQ,
                         (uint8_t)(prn & 0xFF),
                         (uint8_t)(prn >> 8) };
  if (s_ctrl.write_resp(prn_cmd, sizeof(prn_cmd)) <= 0) {
    logger::log("dfu: PRN set write failed");
    return fail(Result::kDisconnectedEarly);
  }
  logger::log("dfu: PRN set to %u", prn);

  // -------------------- Receive Firmware Image --------------------
  uint8_t recv_cmd[1] = { OP_RECEIVE_FW };
  if (s_ctrl.write_resp(recv_cmd, sizeof(recv_cmd)) <= 0) {
    logger::log("dfu: RECEIVE_FW write failed");
    return fail(Result::kDisconnectedEarly);
  }

  // Lazy erase prepares each later application page on demand. Give every
  // page the same physical flash window as START; PRNs provide flow control,
  // so the lower throughput is preferable to a link that can never schedule
  // an 85--90 ms page erase.
  if (!request_link_parameters(DFU_PREP_INTERVAL_UNITS, "stream")) {
    return fail(Result::kConnectFailed);
  }
  logger::log("dfu: streaming %lu B...", (unsigned long)bundle.bin.size);

  // -------------------- Stream firmware bytes --------------------
  // Chunk size = ATT payload (= mtu - 3). 20 B is the default MTU-23 case.
  // 244 B is the max with negotiated MTU 247.
  uint8_t  fw_chunk[244];
  uint32_t sent              = 0;
  uint16_t packets_in_burst  = 0;
  uint32_t next_log_at       = 5;    // log progress at every 5% boundary
  uint32_t t_start           = millis();
  uint8_t  last_progress_pct = 0xFF;
  if (s_progress_cb) s_progress_cb(0);

  while (sent < bundle.bin.size) {
    deferred_logger::drain();
    uint32_t want = bundle.bin.size - sent;
    if (want > payload) want = payload;

    int n = zip_reader::read(bundle.bin, sent, fw_chunk, want);
    if (n != (int)want) {
      logger::log("dfu: bin read short at sent=%lu n=%d", (unsigned long)sent, n);
      return fail(Result::kFsError);
    }

    // SoftDevice write queue can fill up; retry with a short backoff. Bail if
    // it stays full long enough that the link is clearly dead.
    int tries = 0;
    while (true) {
      int w = s_pkt.write(fw_chunk, n);
      if (w == n) break;
      if (++tries > 200) {
        logger::log("dfu: packet write stalled at sent=%lu", (unsigned long)sent);
        return fail(Result::kDisconnectedEarly);
      }
      delay(5);
      if (!s_connected) {
        logger::log("dfu: link dropped mid-stream at sent=%lu", (unsigned long)sent);
        return fail(Result::kDisconnectedEarly);
      }
    }

    sent             += n;
    packets_in_burst += 1;

    // Every `prn` packets the peer fires a PRN notification with the running
    // byte count it has received. We block here so the SoftDevice queue can
    // drain and we don't sprint past what the peer can flash. With prn=0 PRNs
    // are disabled so we never hit this branch.
    if (prn > 0 && packets_in_burst >= prn) {
      packets_in_burst = 0;
      PrnWaitResult prn_result =
          wait_for_prn(sent, sent == bundle.bin.size, 5000);
      if (prn_result == PrnWaitResult::kTimeout) {
        logger::log("dfu: PRN timeout at sent=%lu", (unsigned long)sent);
        return fail(Result::kTimeout);
      }
      if (prn_result == PrnWaitResult::kInvalid) {
        logger::log("dfu: invalid/premature notification at sent=%lu",
                    (unsigned long)sent);
        return fail(Result::kRemoteError);
      }
    }

    uint32_t pct = (uint32_t)((uint64_t)sent * 100 / bundle.bin.size);
    last_progress_pct = (uint8_t)pct;

    // Call the callback on EVERY packet so it can also drive the LED tick;
    // the callback is responsible for being cheap.
    if (s_progress_cb) s_progress_cb(last_progress_pct);

    // Coarser log at every 5% boundary.
    if (pct >= next_log_at) {
      logger::log("dfu: progress %lu%%  (%lu / %lu B)",
                  (unsigned long)pct, (unsigned long)sent,
                  (unsigned long)bundle.bin.size);
      next_log_at = pct + 5;
    }
  }
  if (s_progress_cb) s_progress_cb(100);

  uint32_t elapsed = millis() - t_start;
  logger::log("dfu: stream done in %lu ms (%lu B/s)",
              (unsigned long)elapsed,
              elapsed ? (unsigned long)((uint64_t)bundle.bin.size * 1000 / elapsed) : 0);

  // -------------------- Final receive response --------------------
  uint8_t st = consume_response(OP_RECEIVE_FW);
  logger::log("dfu: RECEIVE_FW final status=0x%02x", st);
  if (st != STATUS_SUCCESS) return fail(Result::kRemoteError);

  if (!request_link_parameters(DFU_TRANSFER_INTERVAL_UNITS, "finalize")) {
    return fail(Result::kConnectFailed);
  }
  delay(LINK_UPDATE_TO_GATT_SETTLE_MS);
  logger::log("dfu: finalize link settled for %lu ms",
              (unsigned long)LINK_UPDATE_TO_GATT_SETTLE_MS);

  // -------------------- Validate --------------------
  uint8_t validate_cmd[1] = { OP_VALIDATE };
  if (s_ctrl.write_resp(validate_cmd, sizeof(validate_cmd)) <= 0) {
    logger::log("dfu: VALIDATE write failed");
    return fail(Result::kDisconnectedEarly);
  }
  st = consume_response(OP_VALIDATE);
  logger::log("dfu: VALIDATE status=0x%02x", st);
  if (st != STATUS_SUCCESS) return fail(Result::kRemoteError);

  // -------------------- Activate and reset --------------------
  // ACTIVATE_AND_RESET (0x05) is sent as WRITE_REQ. Depending on the legacy
  // bootloader, it may finalize settings immediately or schedule image
  // activation for the next boot. In either case, let the peer own the reset
  // boundary instead of truncating the control command with a local disconnect.
  //
  // WRITE_REQ rather than WRITE_CMD because most Legacy DFU bootloaders'
  // Control Point chars don't list the WriteWithoutResponse property — the
  // SoftDevice silently drops WRITE_CMDs to those chars, so an ACTIVATE
  // sent as WRITE_CMD never reaches the peer.
  uint8_t activate_cmd[1] = { OP_ACTIVATE_AND_RESET };
  s_ctrl.write_resp(activate_cmd, sizeof(activate_cmd));

  logger::log("dfu: ACTIVATE sent, waiting for peer to reset...");
  // Up to 2 minutes — SD+BL combo bundles do a lot of flash erase+copy work
  // before they're ready to reboot, and large MTU streams arrive faster than
  // the flash can be cleared, so the post-stream phase can be long.
  if (!wait_disconnected(120000)) {
    logger::log("dfu: peer did not disconnect within 120 s, forcing");
    Bluefruit.disconnect(s_conn_handle);
    wait_disconnected(3000);
    return make_result(Result::kTimeout,
                       CleanupOutcome::kForcedDisconnect);
  }

  logger::log("dfu: DONE");
  return make_result(Result::kOk);
}

}  // namespace dfu_legacy
