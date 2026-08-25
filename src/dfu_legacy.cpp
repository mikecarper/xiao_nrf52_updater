#include "dfu_legacy.h"

#include <bluefruit.h>

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
static constexpr uint8_t OP_RESPONSE_CODE            = 0x10;
static constexpr uint8_t OP_PKT_RECEIPT_NOTIF        = 0x11;

static constexpr uint8_t STATUS_SUCCESS              = 0x01;

// ---------- Globals ----------
static BLEUuid                  s_svc_uuid (kSvcUuid);
static BLEUuid                  s_ctrl_uuid(kCtrlUuid);
static BLEUuid                  s_pkt_uuid (kPktUuid);
static BLEUuid                  s_ver_uuid (kVerUuid);
static BLEClientService         s_svc (s_svc_uuid);
static BLEClientCharacteristic  s_ctrl(s_ctrl_uuid);
static BLEClientCharacteristic  s_pkt (s_pkt_uuid);
static BLEClientCharacteristic  s_ver (s_ver_uuid);

static volatile bool     s_connected   = false;
static volatile uint16_t s_conn_handle = BLE_CONN_HANDLE_INVALID;

static volatile bool     s_notif_ready = false;
static uint8_t           s_notif_buf[20];
static uint8_t           s_notif_len   = 0;

static ProgressCb        s_progress_cb = nullptr;
void set_progress_callback(ProgressCb cb) { s_progress_cb = cb; }

// ---------- Callbacks ----------
static void on_connect(uint16_t conn_handle) {
  s_conn_handle = conn_handle;
  s_connected   = true;
  logger::log("dfu: connected (conn=%u)", conn_handle);
}

static void on_disconnect(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  s_connected   = false;
  s_conn_handle = BLE_CONN_HANDLE_INVALID;
  logger::log("dfu: disconnected reason=0x%02x", reason);
}

static void on_ctrl_notify(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)chr;
  if (len > sizeof(s_notif_buf)) len = sizeof(s_notif_buf);
  memcpy((uint8_t*)s_notif_buf, data, len);
  s_notif_len   = (uint8_t)len;
  s_notif_ready = true;
}

// ---------- Helpers ----------
static bool wait_connected(uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  while (!s_connected && (int32_t)(deadline - millis()) > 0) delay(20);
  return s_connected;
}

static bool wait_disconnected(uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  while (s_connected && (int32_t)(deadline - millis()) > 0) delay(20);
  return !s_connected;
}

static bool wait_notification(uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  while (!s_notif_ready && s_connected && (int32_t)(deadline - millis()) > 0) {
    delay(20);
  }
  return s_notif_ready;
}

// Validate a control-point response notification. Expected layout: [0x10, <op>, <status>].
// Returns the status byte, or 0xFF on protocol error.
static uint8_t consume_response(uint8_t expected_op) {
  if (!wait_notification(15000)) {
    logger::log("dfu: response timeout (expected op=0x%02x)", expected_op);
    return 0xFF;
  }
  s_notif_ready = false;

  if (s_notif_len < 3 || s_notif_buf[0] != OP_RESPONSE_CODE || s_notif_buf[1] != expected_op) {
    logger::log("dfu: unexpected response  op_class=0x%02x op=0x%02x len=%u",
                s_notif_buf[0], s_notif_buf[1], s_notif_len);
    return 0xFF;
  }
  return s_notif_buf[2];
}

static void put_u32le(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v);
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

// Send RESET (0x06) and let the bootloader tear down the link itself. This
// is the safe exit on any error: it tells the bootloader to throw away
// whatever DFU state it accumulated so the *next* attempt starts clean
// instead of getting INVALID_STATE back on its very first START_DFU.
//
// We do NOT initiate the disconnect ourselves - RESET is processed
// asynchronously and the bootloader needs time to act on it before it
// reboots. If we disconnect first the bootloader may just terminate the
// link without ever running its RESET handler, leaving the half-finished
// DFU state in place and the next attempt will hit the same wall.
static Result fail(Result r) {
  if (s_connected) {
    uint8_t reset_cmd[1] = { OP_RESET };
    // Use WRITE_REQ (write_resp). The Control Point characteristic on most
    // Nordic Legacy DFU bootloaders only advertises Write (no WriteWithout-
    // Response), and the SoftDevice silently drops WRITE_CMDs to chars that
    // don't list that property. write_resp blocks until either the peer ACKs
    // or it times out.
    //
    // The return value is intentionally ignored: even if our local write
    // queue refuses (because the link is already half-dead), the disconnect
    // path below still cleans up.
    s_ctrl.write_resp(reset_cmd, sizeof(reset_cmd));
    // Wait for the bootloader to drop the link as a side-effect of RESET.
    // 3 s covers the typical "process command + reboot" round trip. If it
    // doesn't happen we force a local disconnect so we don't stall here.
    if (!wait_disconnected(3000)) {
      logger::log("dfu: peer did not drop link after RESET, forcing");
      Bluefruit.disconnect(s_conn_handle);
      wait_disconnected(2000);
    }
  }
  return r;
}

// Disconnect without writing a DFU opcode. Use this when the peer's mode is
// ambiguous: sending RESET to an application or START_DFU to an unknown peer
// would turn a read/classification failure into a state-changing operation.
static Result disconnect_only(Result r) {
  if (s_connected) {
    uint16_t const conn_handle = s_conn_handle;
    Bluefruit.disconnect(conn_handle);
    wait_disconnected(2000);
  }
  return r;
}

// BLEClientCharacteristic::read() uses the framework's 100 ms generic GATT
// timeout. That is shorter than a complete request/response on peers that are
// still using a slow application-mode connection interval, and a timeout is
// reported as a zero-length read. Read-by-UUID has a 500 ms discovery timeout;
// retry it a bounded number of times and require exactly the two bytes defined
// by the Nordic Legacy DFU revision characteristic.
static bool read_revision(uint16_t* revision) {
  ble_gattc_handle_range_t const range = s_svc.getHandleRange();

  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    uint8_t bytes[2] = {0xA5, 0x5A};
    uint16_t const len = Bluefruit.Gatt.readCharByUuid(
      s_conn_handle, s_ver_uuid, bytes, sizeof(bytes),
      range.start_handle, range.end_handle
    );

    logger::log(
      "dfu: revision read %u/3 len=%u raw=%02x%02x",
      attempt, len, bytes[0], bytes[1]
    );

    if (!s_connected) return false;
    if (len == sizeof(bytes)) {
      *revision = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
      return true;
    }

    delay(150);
  }

  return false;
}

// Bluefruit implements enableNotify() as a queued CCCD WRITE_CMD. There is no
// ATT response to wait on, so returning true only means the command entered
// the local SoftDevice queue. Give the peer enough connection events to apply
// it before sending a control-point WRITE_REQ. The application-side BLEDfu
// service explicitly rejects control writes until the CCCD is active.
static bool enable_control_notifications() {
  if (!s_ctrl.enableNotify()) return false;
  delay(250);
  s_notif_ready = false;
  s_notif_len = 0;
  return s_connected;
}

// ---------- DFU run ----------
Result run(const ble_scanner::Target& target,
           const firmware_zip::Parsed& bundle,
           const config::Config& cfg) {
  Bluefruit.Central.setConnectCallback(on_connect);
  Bluefruit.Central.setDisconnectCallback(on_disconnect);

  s_svc.begin();
  s_ctrl.setNotifyCallback(on_ctrl_notify);
  s_ctrl.begin();
  s_pkt.begin();
  s_ver.begin();

  s_connected   = false;
  s_conn_handle = BLE_CONN_HANDLE_INVALID;
  s_notif_ready = false;
  s_notif_len   = 0;

  ble_gap_addr_t addr = target.addr;
  logger::log("dfu: connecting to %02X:%02X:%02X:%02X:%02X:%02X",
              addr.addr[5], addr.addr[4], addr.addr[3],
              addr.addr[2], addr.addr[1], addr.addr[0]);

  if (!Bluefruit.Central.connect(&addr)) return Result::kConnectFailed;
  if (!wait_connected(10000))            return Result::kConnectFailed;

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
    return Result::kConnectFailed;
  }

  // Optional MTU negotiation. nRF52840 SoftDevice supports up to 247.
  // Many Nordic Legacy DFU bootloaders honour the exchange and let us write
  // (mtu-3)-byte payloads to the Packet characteristic - typically 5-10x
  // faster end-to-end than the default 20 B writes.
  uint16_t payload = 20;
  if (cfg.high_mtu) {
    BLEConnection* conn = Bluefruit.Connection(s_conn_handle);
    if (conn) {
      conn->requestMtuExchange(247);
      delay(200);  // let the exchange complete
      if (!s_connected) {
        logger::log("dfu: link dropped during MTU exchange");
        return Result::kConnectFailed;
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

  // OTAFIX bootloaders advertise the Legacy DFU UUID. MeshCore application
  // mode exposes the same GATT service after connection but does not put its
  // UUID in the advertisement. This is more reliable than using a failed
  // revision read as value 0, which previously misclassified the app as a
  // bootloader and sent START_DFU to the wrong state machine.
  // Older builds with an overlong DEVICE_NAME could run out of the 31-byte
  // advertising payload before adding the UUID. MAC+1 remains independent
  // bootloader evidence for recovering those deployed builds.
  bool is_app_mode = !target.advertises_legacy_dfu && !target.mac_plus_one;

  // Read the DFU Version characteristic (little-endian uint16) as an optional
  // bootloader-side consistency check. The revision is 0.1 in app mode and
  // 0.5 or newer in a modern Legacy DFU bootloader. Some Bluefruit central
  // builds cannot complete this read, so the advertised UUID remains the
  // primary mode signal for supported OTAFIX targets.
  if (!is_app_mode && !pkt_ok) {
    logger::log("dfu: bootloader advertisement without Packet characteristic");
    return fail(Result::kCharMissing);
  }

  uint16_t version = 0;
  if (!is_app_mode && ver_ok && read_revision(&version)) {
    logger::log("dfu: peer DFU version = %u.%u  (raw 0x%04X)",
                version >> 8, version & 0xFF, version);

    if (version <= 0x0001) {
      logger::log("dfu: revision contradicts bootloader advertisement; refusing update");
      return disconnect_only(Result::kRevisionUnreadable);
    }
  } else if (!is_app_mode && ver_ok) {
    logger::log("dfu: revision unreadable; using scan-time bootloader evidence");
  }

  if (is_app_mode) {
    logger::log("dfu: advertisement omitted DFU UUID; selected peer is app mode");
    logger::log("dfu: peer in app mode, sending buttonless trigger");
    if (!enable_control_notifications()) {
      logger::log("dfu: buttonless notification setup failed");
      return disconnect_only(Result::kCharMissing);
    }
    uint8_t enter_bl[2] = { 0x01, 0x04 };
    // write_resp may fail because the peer disconnects before sending the
    // ATT response; that's expected and not an error.
    int const written = s_ctrl.write_resp(enter_bl, sizeof(enter_bl));
    if (written <= 0) {
      logger::log("dfu: buttonless trigger had no ATT response; waiting for reboot");
    }
    if (!wait_disconnected(5000)) {
      logger::log("dfu: buttonless peer did not reboot");
      return disconnect_only(Result::kTimeout);
    }
    return Result::kButtonlessTriggered;
  }

  // Enable CCCD on the Control Point so the target's status notifications
  // come back as on_ctrl_notify() callbacks.
  if (!enable_control_notifications()) {
    logger::log("dfu: enableNotify() failed");
    return fail(Result::kCharMissing);
  }
  logger::log("dfu: notifications enabled");

  // -------------------- Start DFU + image sizes --------------------
  uint8_t start_cmd[2] = { OP_START_DFU, bundle.type };
  if (s_ctrl.write_resp(start_cmd, sizeof(start_cmd)) <= 0) {
    logger::log("dfu: Start DFU write failed");
    return fail(Result::kDisconnectedEarly);
  }
  logger::log("dfu: sent START_DFU type=0x%02x", bundle.type);

  // 3 x uint32 LE: SD size, BL size, App size. Per LegacyDfuImpl, even when a
  // field is unused it must still be present and zero.
  uint8_t sizes[12] = {0};
  put_u32le(sizes + 0, bundle.sd_size);
  put_u32le(sizes + 4, bundle.bl_size);
  put_u32le(sizes + 8, (bundle.type & firmware_zip::TYPE_APPLICATION) ? bundle.bin.size : 0);
  if (s_pkt.write(sizes, sizeof(sizes)) <= 0) {
    logger::log("dfu: image size write failed");
    return fail(Result::kDisconnectedEarly);
  }
  logger::log("dfu: sent sizes  sd=%lu bl=%lu app=%lu",
              (unsigned long)bundle.sd_size, (unsigned long)bundle.bl_size,
              (unsigned long)((bundle.type & firmware_zip::TYPE_APPLICATION) ? bundle.bin.size : 0));

  uint8_t status = consume_response(OP_START_DFU);
  logger::log("dfu: START_DFU response status=0x%02x", status);
  if (status != STATUS_SUCCESS) {
    return fail(Result::kRemoteError);
  }

  // -------------------- Init packet (.dat) --------------------
  uint8_t init_start[2]    = { OP_INIT_DFU_PARAMS, 0x00 };
  uint8_t init_complete[2] = { OP_INIT_DFU_PARAMS, 0x01 };

  if (s_ctrl.write_resp(init_start, sizeof(init_start)) <= 0) {
    logger::log("dfu: INIT_DFU_PARAMS start write failed");
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
      if (!wait_notification(5000)) {
        logger::log("dfu: PRN timeout at sent=%lu", (unsigned long)sent);
        return fail(Result::kTimeout);
      }
      if (sent == bundle.bin.size && s_notif_len >= 3 &&
          s_notif_buf[0] == OP_RESPONSE_CODE &&
          s_notif_buf[1] == OP_RECEIVE_FW) {
        // The peer may enqueue its final RECEIVE_FW response before the
        // central has handled the last local write completion. Preserve it
        // for consume_response() below instead of mistaking it for a PRN and
        // waiting forever for a notification that already arrived.
        logger::log("dfu: final response arrived with last PRN burst");
      } else if (s_notif_len >= 5 && s_notif_buf[0] == OP_PKT_RECEIPT_NOTIF) {
        s_notif_ready = false;
        uint32_t peer_recv = (uint32_t)s_notif_buf[1] |
                             ((uint32_t)s_notif_buf[2] << 8) |
                             ((uint32_t)s_notif_buf[3] << 16) |
                             ((uint32_t)s_notif_buf[4] << 24);
        // The peer's count must agree with ours. If it doesn't, the link or
        // bootloader is desynced and continuing is pointless.
        if (peer_recv != sent) {
          logger::log("dfu: PRN mismatch  sent=%lu peer=%lu",
                      (unsigned long)sent, (unsigned long)peer_recv);
          return fail(Result::kRemoteError);
        }
      } else {
        s_notif_ready = false;
        // Could be the final 0x10/0x03 already - we'll handle that after the
        // stream loop only when it accompanies the actual final burst.
        logger::log("dfu: unexpected notif during stream  op=0x%02x len=%u",
                    s_notif_buf[0], s_notif_len);
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
  // ACTIVATE_AND_RESET (0x05) is sent as WRITE_REQ. The peer:
  //   1. ATT-acks the request (or just disconnects mid-request - both fine)
  //   2. copies the staged image into its final region (flash erase + write,
  //      hundreds of ms for SD+BL bundles)
  //   3. resets - which is what tears down the BLE link.
  //
  // We must NOT call disconnect() ourselves: doing so before step 2 finishes
  // leaves the peer with a partially-applied image and it falls back to the
  // old firmware. The Java reference client just waits for the peer to drop
  // the link, and we mirror that.
  //
  // WRITE_REQ rather than WRITE_CMD because most Legacy DFU bootloaders'
  // Control Point chars don't list the WriteWithoutResponse property - the
  // SoftDevice silently drops WRITE_CMDs to those chars, so an ACTIVATE
  // sent as WRITE_CMD never reaches the peer.
  uint8_t activate_cmd[1] = { OP_ACTIVATE_AND_RESET };
  s_ctrl.write_resp(activate_cmd, sizeof(activate_cmd));

  logger::log("dfu: ACTIVATE sent, waiting for peer to reset...");
  // Up to 2 minutes - SD+BL combo bundles do a lot of flash erase+copy work
  // before they're ready to reboot, and large MTU streams arrive faster than
  // the flash can be cleared, so the post-stream phase can be long.
  if (!wait_disconnected(120000)) {
    logger::log("dfu: peer did not disconnect within 120 s, forcing");
    Bluefruit.disconnect(s_conn_handle);
    wait_disconnected(3000);
    return Result::kTimeout;
  }

  logger::log("dfu: DONE");
  return Result::kOk;
}

}  // namespace dfu_legacy
