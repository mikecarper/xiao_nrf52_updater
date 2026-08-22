#!/usr/bin/env python3

import hashlib
import importlib.util
import json
import pathlib
import tempfile
import unittest
import warnings
import zipfile


ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "v4_serial_upload", ROOT / "tools" / "v4_serial_upload.py")
UPLOADER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(UPLOADER)


def write_bundle(path, manifest, compression=zipfile.ZIP_STORED,
                 init_packet=b"init"):
    with zipfile.ZipFile(path, "w", compression=compression) as archive:
        archive.writestr("firmware.bin", b"firmware bytes")
        archive.writestr("firmware.dat", init_packet)
        archive.writestr("manifest.json", json.dumps({"manifest": manifest}))


class FakePort:
    def __init__(self, replies):
        self.replies = [reply.encode() + b"\n" for reply in replies]
        self.writes = []

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)

    def flush(self):
        pass

    def readline(self):
        return self.replies.pop(0) if self.replies else b""


class PreflightTests(unittest.TestCase):
    def test_accepts_single_stored_application(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "valid.zip"
            write_bundle(path, {"application": {
                "bin_file": "firmware.bin", "dat_file": "firmware.dat"}})
            size, digest, section = UPLOADER.preflight_zip(path)
            self.assertEqual(size, path.stat().st_size)
            self.assertEqual(digest, hashlib.sha256(path.read_bytes()).hexdigest())
            self.assertEqual(section, "application")

    def test_rejects_compressed_member(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "compressed.zip"
            write_bundle(path, {"application": {
                "bin_file": "firmware.bin", "dat_file": "firmware.dat"}},
                zipfile.ZIP_DEFLATED)
            with self.assertRaisesRegex(ValueError, "STORED"):
                UPLOADER.preflight_zip(path)

    def test_rejects_multiple_legacy_sections(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "multiple.zip"
            node = {"bin_file": "firmware.bin", "dat_file": "firmware.dat"}
            write_bundle(path, {"application": node, "bootloader": node})
            with self.assertRaisesRegex(ValueError, "exactly one"):
                UPLOADER.preflight_zip(path)

    def test_rejects_secure_combined_application(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "secure.zip"
            node = {"bin_file": "firmware.bin", "dat_file": "firmware.dat"}
            write_bundle(path, {"bootloader_application": node})
            with self.assertRaisesRegex(ValueError, "Secure DFU"):
                UPLOADER.preflight_zip(path)

    def test_rejects_invalid_softdevice_bootloader_sizes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "bad-sizes.zip"
            write_bundle(path, {"softdevice_bootloader": {
                "bin_file": "firmware.bin", "dat_file": "firmware.dat",
                "sd_size": 1, "bl_size": 1}})
            with self.assertRaisesRegex(ValueError, "do not match"):
                UPLOADER.preflight_zip(path)

    def test_rejects_init_packet_over_legacy_buffer(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "large-init.zip"
            write_bundle(path, {"application": {
                "bin_file": "firmware.bin", "dat_file": "firmware.dat"}},
                init_packet=b"x" * 129)
            with self.assertRaisesRegex(ValueError, "1..128"):
                UPLOADER.preflight_zip(path)

    def test_rejects_duplicate_member_name(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "duplicate.zip"
            node = {"bin_file": "firmware.bin", "dat_file": "firmware.dat"}
            write_bundle(path, {"application": node})
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", UserWarning)
                with zipfile.ZipFile(path, "a") as archive:
                    archive.writestr("firmware.dat", b"second")
            with self.assertRaisesRegex(ValueError, "duplicate"):
                UPLOADER.preflight_zip(path)

    def test_rejects_crc_corruption(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "crc.zip"
            node = {"bin_file": "firmware.bin", "dat_file": "firmware.dat"}
            write_bundle(path, {"application": node})
            payload = bytearray(path.read_bytes())
            position = payload.index(b"firmware bytes")
            payload[position] ^= 0x01
            with self.assertRaises((ValueError, zipfile.BadZipFile)):
                UPLOADER.preflight_payload(bytes(payload))

    def test_rejects_non_object_manifest_section(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "bad-manifest.zip"
            write_bundle(path, {"application": "not-an-object"})
            with self.assertRaisesRegex(ValueError, "must be an object"):
                UPLOADER.preflight_zip(path)

    def test_rejects_member_name_too_long_for_device(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "long-name.zip"
            long_name = "x" * 96
            with zipfile.ZipFile(path, "w") as archive:
                archive.writestr(long_name, b"unused")
                archive.writestr("firmware.bin", b"firmware")
                archive.writestr("firmware.dat", b"init")
                archive.writestr("manifest.json", json.dumps({"manifest": {
                    "application": {"bin_file": "firmware.bin",
                                    "dat_file": "firmware.dat"}}}))
            with self.assertRaisesRegex(ValueError, "1..95"):
                UPLOADER.preflight_zip(path)


class SerialFlowTests(unittest.TestCase):
    def make_payload(self, directory):
        path = pathlib.Path(directory) / "valid.zip"
        write_bundle(path, {"application": {
            "bin_file": "firmware.bin", "dat_file": "firmware.dat"}})
        return path, UPLOADER.load_bundle(path)

    def test_chunks_are_acked_before_target_and_start(self):
        with tempfile.TemporaryDirectory() as directory:
            path, loaded = self.make_payload(directory)
            payload, size, digest, _ = loaded
            # Mutating the source after load must not affect transmitted bytes.
            path.write_bytes(path.read_bytes() + b"grew-after-preflight")
            offsets = list(range(64, size, 64)) + [size]
            replies = (["STATUS storage=ready staged=no bytes=0 target=<unset>",
                        "READY 64"] +
                       [f"ACK {offset}" for offset in offsets] +
                       ["OK staged bytes=1", "OK target MAC set",
                        "[00:00:01] dfu: SUCCESS staged-delete=1"])
            port = FakePort(replies)
            result = UPLOADER.stage_and_maybe_start(
                port, payload, size, digest,
                "TARGET MAC AA:BB:CC:DD:EE:FF", True, 5)
            self.assertEqual(result, 0)
            self.assertEqual(port.writes[0], b"STATUS\n")
            self.assertTrue(port.writes[1].startswith(b"UPLOAD "))
            raw_writes = port.writes[2:2 + len(offsets)]
            self.assertEqual(b"".join(raw_writes), payload)
            self.assertEqual(port.writes[2 + len(offsets)],
                             b"TARGET MAC AA:BB:CC:DD:EE:FF\n")
            self.assertEqual(port.writes[3 + len(offsets)], b"START\n")

    def test_upload_failure_never_authorizes_target_or_start(self):
        with tempfile.TemporaryDirectory() as directory:
            _, loaded = self.make_payload(directory)
            payload, size, digest, _ = loaded
            port = FakePort([
                "STATUS storage=ready staged=yes bytes=99 target=name:old",
                "READY 512",
                "ERR upload: write failure; serial quarantined until reboot",
            ])
            with self.assertRaisesRegex(RuntimeError, "ERR upload"):
                UPLOADER.stage_and_maybe_start(
                    port, payload, size, digest,
                    "TARGET NAME XIAO", True, 5)
            commands = b"".join(port.writes)
            self.assertNotIn(b"TARGET ", commands)
            self.assertNotIn(b"START\n", commands)

    def test_ack_offset_mismatch_stops_flow(self):
        with tempfile.TemporaryDirectory() as directory:
            _, loaded = self.make_payload(directory)
            payload, size, digest, _ = loaded
            port = FakePort([
                "STATUS storage=ready staged=no bytes=0 target=<unset>",
                "READY 512", "ACK 1",
            ])
            with self.assertRaisesRegex(RuntimeError, "acknowledgement mismatch"):
                UPLOADER.stage_and_maybe_start(
                    port, payload, size, digest,
                    "TARGET NAME XIAO", True, 5)

    def test_payload_mismatch_sends_nothing(self):
        port = FakePort([])
        with self.assertRaisesRegex(RuntimeError, "framing"):
            UPLOADER.stage_and_maybe_start(
                port, b"changed", 6, hashlib.sha256(b"other").hexdigest(),
                "TARGET NAME XIAO", True, 5)
        self.assertEqual(port.writes, [])


if __name__ == "__main__":
    unittest.main()
