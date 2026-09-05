#!/usr/bin/env python3
"""Source-level regression tests for field-critical DFU state transitions."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
DFU = (ROOT / "src" / "dfu_legacy.cpp").read_text()
MAIN = (ROOT / "src" / "main.cpp").read_text()
HEADER = (ROOT / "src" / "dfu_legacy.h").read_text()
USB_MSC = (ROOT / "src" / "usb_msc.cpp").read_text()
CDC_RESET_HOOK = (ROOT / "src" / "cdc_reset_hook.cpp").read_text()


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for offset, char in enumerate(source[brace:], start=brace):
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[start : offset + 1]
    raise AssertionError(f"unterminated function: {signature}")


class DfuGattSafetyTests(unittest.TestCase):
    def test_cccd_is_discovered_and_acknowledged(self):
        body = function_body(DFU, "static bool enable_control_notifications()")
        self.assertIn("sd_ble_gattc_descriptors_discover", body)
        self.assertIn("BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG", DFU)
        self.assertIn("write_request(s_cccd_handle", body)
        self.assertNotRegex(body, r"\bs_ctrl\.enableNotify\s*\(")

    def test_control_writes_retain_real_att_status(self):
        body = function_body(DFU, "static GattWriteResult write_request(")
        self.assertIn("BLE_GATT_OP_WRITE_REQ", body)
        self.assertIn("BLE_GATTC_EVT_WRITE_RSP", DFU)
        self.assertIn("gattc->gatt_status", DFU)
        self.assertIn("s_write_status != BLE_GATT_STATUS_SUCCESS", body)
        self.assertNotRegex(DFU, r"\bs_ctrl\.write_resp\s*\(")

    def test_client_objects_are_registered_only_once(self):
        body = function_body(DFU, "Result run(")
        self.assertIn("static bool clients_begun = false", body)
        self.assertRegex(
            body,
            re.compile(
                r"if \(!clients_begun\).*s_svc\.begin\(\).*clients_begun = true",
                re.DOTALL,
            ),
        )

    def test_preflight_failures_do_not_send_reset(self):
        body = function_body(DFU, "Result run(")
        self.assertIn("return disconnect_only(Result::kServiceMissing)", body)
        self.assertGreaterEqual(
            body.count("return disconnect_only(Result::kCharMissing)"), 3
        )

    def test_buttonless_failure_is_bounded_and_not_a_wedge(self):
        self.assertIn("kButtonlessFailed", HEADER)
        self.assertIn("if (r == dfu_legacy::Result::kButtonlessFailed)", MAIN)
        self.assertIn("dfu_attempt++", MAIN)
        self.assertIn("have_pending_app_mac = true", MAIN)

        wedge = re.search(r"bool wedge = \((.*?)\);", MAIN, re.DOTALL)
        self.assertIsNotNone(wedge)
        self.assertNotIn("kButtonlessFailed", wedge.group(1))

    def test_live_revision_overrides_advertising_mode(self):
        body = function_body(DFU, "Result run(")
        read = body.index("if (ver_ok && read_revision(&version))")
        app = body.index("if (version == 0x0001)", read)
        bootloader = body.index("else if (version >= 0x0005)", app)
        packet_gate = body.index("if (!is_app_mode && !pkt_ok)", bootloader)
        self.assertLess(read, app)
        self.assertLess(app, bootloader)
        self.assertLess(bootloader, packet_gate)

    def test_activate_requires_acceptance_or_disconnect(self):
        body = function_body(DFU, "Result run(")
        activate = body[body.index("uint8_t activate_cmd") :]
        self.assertIn("activate != GattWriteResult::kSuccess", activate)
        self.assertIn("activate != GattWriteResult::kDisconnected", activate)
        self.assertIn("return fail(Result::kRemoteError)", activate)

    def test_pre_submit_disconnect_is_not_command_acceptance(self):
        body = function_body(DFU, "static GattWriteResult write_request(")
        pre_submit = body[
            body.index("if (err != NRF_SUCCESS) {") :
            body.index("while (!s_write_done")
        ]
        self.assertIn("return GattWriteResult::kLocalError", pre_submit)
        self.assertNotIn("return GattWriteResult::kDisconnected", pre_submit)


class StartupTriggerSafetyTests(unittest.TestCase):
    def test_usb_power_has_stabilization_and_enumeration_grace(self):
        self.assertIn("vbus_poll_deadline = millis() + 500", MAIN)
        self.assertIn("s_vbus_grace_deadline = millis() + 5000", MAIN)
        self.assertIn("vbus_present() || usb_msc::is_mounted()", MAIN)
        self.assertIn("no VBUS after 5 s + zip present", MAIN)

    def test_battery_backed_usb_unplug_is_a_real_trigger(self):
        self.assertIn("usb_msc::was_ever_mounted()", MAIN)
        self.assertIn("!usb_msc::is_mounted() && !vbus_present()", MAIN)
        self.assertIn('unplug_trig ? "USB-unplug"', MAIN)
        self.assertIn("(uint32_t)(now - s_unplug_started) >= 2000", MAIN)

    def test_1200_baud_reset_is_deferred_out_of_usb_callback(self):
        callback = function_body(CDC_RESET_HOOK, "tud_cdc_line_state_cb(")
        self.assertIn("usb_msc::note_cdc_dtr_drop()", callback)
        self.assertNotIn("enterSerialDfu", callback)
        self.assertNotIn("Adafruit_TinyUSB", CDC_RESET_HOOK)
        self.assertIn("Serial.baud() == 1200", USB_MSC)
        self.assertIn("usb_msc::serial_dfu_requested()", MAIN)

    def test_serial_dfu_quiesces_msc_before_detach_and_reset(self):
        body = function_body(USB_MSC, "bool detach_for_serial_dfu(")
        not_ready = body.index("s_msc.setUnitReady(false)")
        close_gate = body.index("s_accept_io.store(false", not_ready)
        drain = body.index("s_active_io.load", close_gate)
        flush = body.index("storage::flash().syncBlocks()", drain)
        detach = body.index("TinyUSBDevice.detach()", flush)
        self.assertLess(not_ready, close_gate)
        self.assertLess(close_gate, drain)
        self.assertLess(drain, flush)
        self.assertLess(flush, detach)

        request = MAIN.index("usb_msc::serial_dfu_requested()")
        safe_detach = MAIN.index("usb_msc::detach_for_serial_dfu()", request)
        reset = MAIN.index("enterSerialDfu()", safe_detach)
        self.assertLess(request, safe_detach)
        self.assertLess(safe_detach, reset)


class PackageSafetyTests(unittest.TestCase):
    def test_multi_image_packages_are_rejected_before_transfer(self):
        source = (ROOT / "src" / "firmware_zip.cpp").read_text()
        self.assertIn('m["softdevice_bootloader_application"]', source)
        self.assertIn("section_count != 1", source)
        self.assertIn("sizes do not match bin", source)


if __name__ == "__main__":
    unittest.main()
