#!/usr/bin/env python3
"""Safely stage a Nordic Legacy DFU ZIP on a Heltec V4 updater.

The serial port and target are always explicit.  Staging is the default;
passing --start is required to transmit firmware over BLE.
"""

import argparse
import hashlib
import io
import json
import pathlib
import re
import sys
import time
import zipfile

try:
    import serial
except ImportError as exc:  # pragma: no cover - depends on host packaging
    raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc


MAX_ZIP_BYTES = 2 * 1024 * 1024
MAX_INIT_PACKET_BYTES = 128
LEGACY_SECTIONS = {
    "application",
    "bootloader",
    "softdevice",
    "softdevice_bootloader",
}
SECURE_SECTIONS = {
    "bootloader_application",
    "softdevice_application",
    "softdevice_bootloader_application",
}


def preflight_payload(payload: bytes) -> tuple[int, str, str]:
    size = len(payload)
    if size <= 0 or size > MAX_ZIP_BYTES:
        raise ValueError(f"ZIP size must be 1..{MAX_ZIP_BYTES} bytes")
    digest = hashlib.sha256(payload).hexdigest()

    with zipfile.ZipFile(io.BytesIO(payload), "r") as archive:
        infos = archive.infolist()
        if not infos or any(info.compress_type != zipfile.ZIP_STORED for info in infos):
            raise ValueError("all ZIP members must use STORED compression")
        if any(info.flag_bits & (0x0001 | 0x0008) for info in infos):
            raise ValueError("encrypted/data-descriptor ZIP members are unsupported")
        member_names = [info.filename for info in infos]
        if len(member_names) != len(set(member_names)):
            raise ValueError("duplicate ZIP member names are unsupported")
        try:
            encoded_names = [name.encode("ascii") for name in member_names]
        except UnicodeEncodeError as exc:
            raise ValueError("ZIP member names must be ASCII") from exc
        if any(not 1 <= len(name) <= 95 for name in encoded_names):
            raise ValueError("ZIP member names must be 1..95 bytes")
        bad_member = archive.testzip()
        if bad_member is not None:
            raise ValueError(f"CRC check failed for {bad_member}")
        names = set(member_names)
        if "manifest.json" not in names:
            raise ValueError("manifest.json is missing")
        manifest_bytes = archive.read("manifest.json")
        if not 0 < len(manifest_bytes) <= 2048:
            raise ValueError("manifest.json size is out of range")
        root = json.loads(manifest_bytes)
        if not isinstance(root, dict) or not isinstance(root.get("manifest"), dict):
            raise ValueError("top-level manifest must be an object")
        manifest = root["manifest"]

        present_legacy = sorted(LEGACY_SECTIONS.intersection(manifest))
        present_secure = sorted(SECURE_SECTIONS.intersection(manifest))
        if present_secure:
            raise ValueError("combined application package requires Secure DFU")
        if len(present_legacy) != 1:
            raise ValueError("ZIP must contain exactly one Legacy DFU section")
        section_name = present_legacy[0]
        section = manifest[section_name]
        if not isinstance(section, dict):
            raise ValueError("selected manifest section must be an object")
        bin_name = section.get("bin_file")
        dat_name = section.get("dat_file")
        if (not isinstance(bin_name, str) or not isinstance(dat_name, str) or
                not bin_name or not dat_name or bin_name not in names or
                dat_name not in names):
            raise ValueError("manifest bin_file/dat_file is missing from ZIP")
        bin_size = archive.getinfo(bin_name).file_size
        dat_size = archive.getinfo(dat_name).file_size
        if bin_size == 0 or not 1 <= dat_size <= MAX_INIT_PACKET_BYTES:
            raise ValueError(
                f"firmware must be nonempty and init packet must be 1..{MAX_INIT_PACKET_BYTES} bytes")
        if section_name == "softdevice_bootloader":
            metadata = section.get("info_read_only_metadata", {})
            if not isinstance(metadata, dict):
                raise ValueError("info_read_only_metadata must be an object")
            sd_size = section.get("sd_size", 0) or metadata.get("sd_size", 0)
            bl_size = section.get("bl_size", 0) or metadata.get("bl_size", 0)
            if (not isinstance(sd_size, int) or isinstance(sd_size, bool) or
                    not isinstance(bl_size, int) or isinstance(bl_size, bool) or
                    sd_size <= 0 or bl_size <= 0 or
                    sd_size > 0xFFFFFFFF or bl_size > 0xFFFFFFFF or
                    sd_size + bl_size != bin_size):
                raise ValueError("softdevice_bootloader sizes do not match its binary")

    return size, digest, section_name


def load_bundle(path: pathlib.Path) -> tuple[bytes, int, str, str]:
    # The <=2 MiB immutable snapshot is both preflighted and transmitted. This
    # avoids reopening a path which may have changed or grown after hashing.
    payload = path.read_bytes()
    size, digest, section = preflight_payload(payload)
    return payload, size, digest, section


def preflight_zip(path: pathlib.Path) -> tuple[int, str, str]:
    _, size, digest, section = load_bundle(path)
    return size, digest, section


def read_until(port, prefixes: tuple[str, ...], timeout: float) -> str:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        print(line, flush=True)
        if line.startswith(prefixes):
            return line
    raise TimeoutError(f"timed out waiting for {prefixes}")


def send_line(port, line: str) -> None:
    port.write(line.encode("utf-8") + b"\n")
    port.flush()


def stage_and_maybe_start(port, payload: bytes, size: int,
                          digest: str, target_command: str, start: bool,
                          wait_seconds: int) -> int:
    if len(payload) != size or hashlib.sha256(payload).hexdigest() != digest:
        raise RuntimeError("immutable upload payload no longer matches framing")
    send_line(port, "STATUS")
    status = read_until(port, ("STATUS ",), 5)
    if "storage=ready" not in status:
        raise RuntimeError("V4 storage is not ready")

    # Stage first. The device clears any previous target as soon as it accepts
    # UPLOAD, binding later authorization to this completed raw frame.
    send_line(port, f"UPLOAD {size} {digest}")
    ready = read_until(port, ("READY ", "ERR "), 5)
    if not ready.startswith("READY "):
        raise RuntimeError(ready)
    try:
        chunk_size = int(ready.split()[1])
    except (IndexError, ValueError) as exc:
        raise RuntimeError(f"invalid READY reply: {ready}") from exc
    if not 1 <= chunk_size <= 4096:
        raise RuntimeError(f"unsafe upload chunk size: {chunk_size}")

    offset = 0
    while offset < size:
        chunk = payload[offset:offset + chunk_size]
        if not chunk:
            raise RuntimeError("immutable payload shorter than declared size")
        written = port.write(chunk)
        if written != len(chunk):
            raise RuntimeError(
                f"short serial write at {offset}: {written}/{len(chunk)}")
        port.flush()
        offset += len(chunk)
        ack = read_until(port, ("ACK ", "ERR "), 15)
        if ack != f"ACK {offset}":
            raise RuntimeError(f"upload acknowledgement mismatch: {ack}")
    upload_reply = read_until(port, ("OK staged", "ERR "), 120)
    if upload_reply.startswith("ERR "):
        raise RuntimeError(upload_reply)

    send_line(port, target_command)
    target_reply = read_until(port, ("OK target", "ERR "), 5)
    if target_reply.startswith("ERR "):
        raise RuntimeError(target_reply)

    if not start:
        print("staged only; pass --start explicitly to run BLE DFU")
        return 0

    send_line(port, "START")
    deadline = time.monotonic() + wait_seconds
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        print(line, flush=True)
        if "dfu: SUCCESS" in line:
            return 0
        if ("dfu: FAILED" in line or "start refused:" in line or
                "scan: target not found" in line):
            return 2
    raise TimeoutError("timed out waiting for final DFU result")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True,
                        help="exact V4 CDC device, e.g. /dev/ttyACM0")
    parser.add_argument("--zip", required=True, type=pathlib.Path,
                        help="Nordic Legacy DFU ZIP to stage")
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--target-name",
                        help="explicit advertised-name substring(s), separated by |")
    target.add_argument("--target-mac",
                        help="explicit AA:BB:CC:DD:EE:FF target address")
    parser.add_argument("--start", action="store_true",
                        help="start BLE DFU after staging (default: stage only)")
    parser.add_argument("--wait-seconds", type=int, default=1200,
                        help="maximum log wait after --start (default: 1200)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.zip.is_file():
        raise SystemExit(f"ZIP does not exist: {args.zip}")
    if args.wait_seconds <= 0:
        raise SystemExit("--wait-seconds must be positive")
    if args.target_name and ("\r" in args.target_name or "\n" in args.target_name):
        raise SystemExit("target name cannot contain a newline")
    if args.target_name and not 1 <= len(args.target_name.strip()) <= 31:
        raise SystemExit("target name must be 1..31 characters")
    if args.target_mac and not re.fullmatch(
            r"(?i)[0-9a-f]{2}(?::[0-9a-f]{2}){5}", args.target_mac):
        raise SystemExit("target MAC must be AA:BB:CC:DD:EE:FF")

    try:
        payload, size, digest, section = load_bundle(args.zip)
    except (KeyError, json.JSONDecodeError, OSError, ValueError,
            zipfile.BadZipFile) as exc:
        raise SystemExit(f"ZIP preflight failed: {exc}") from exc
    print(f"preflight: {section}, {size} bytes, sha256={digest}")

    # Open only the exact requested path and suppress modem-control reset
    # pulses. There is deliberately no auto-discovery or first-port fallback.
    port = serial.Serial(port=None, baudrate=115200, timeout=0.5,
                         write_timeout=10, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = args.port
    port.open()
    try:
        time.sleep(0.5)
        port.reset_input_buffer()
        target_command = (f"TARGET NAME {args.target_name}" if args.target_name
                          else f"TARGET MAC {args.target_mac}")
        return stage_and_maybe_start(
            port, payload, size, digest, target_command, args.start,
            args.wait_seconds)
    except (OSError, RuntimeError, TimeoutError, serial.SerialException) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
