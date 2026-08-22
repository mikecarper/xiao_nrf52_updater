# xiao_nrf52_updater

A standalone BLE DFU client that runs on a **Seeed XIAO nRF52840**, **RAK4631** (with RAK15001 QSPI flash), or **Heltec WiFi LoRa 32 V4** and flashes Nordic-format firmware bundles to *other* nRF52 devices over Bluetooth. The nRF52 updater variants use USB MSC; the Heltec V4 uses an explicit, SHA-256 checked serial staging protocol and never starts DFU automatically.

<p align="center">
  <img src="img/drone.jpg" width="400"><br>
  <sup>DJI Neo 2 with Seeed Xiao nRF52 and 600mA battery: 10g payload</sup>
</p>

Intended use: a drone-mounted OTA flasher that updates a hard to reach nRF52 repeater in the field.

## What's in the box

| Component | Role |
|---|---|
| **USB MSC** | Exposes a 2 MB QSPI flash as a FAT12 USB drive (label `XIAO DFU` on XIAO, `RAK DFU` on RAK4631). The host drops the target firmware `.zip` and `CONFIG.TXT` here. |
| **`CONFIG.TXT`** | `key=value` config (BLE name filter, optional fixed BLE MAC target, PRN, MTU, retries, min RSSI, retry cooldown). |
| **`LOG.TXT`** | Append-only log written after firmware has exclusive ownership of the drive. |
| **BLE central** | Bluefruit central; scans for the Nordic Legacy DFU service UUID, advertised BLE names, or an optional fixed BLE MAC target. |
| **DFU client** | Implements the Nordic Legacy DFU protocol (mirrors the Android `LegacyDfuImpl.java`), including the buttonless trigger for app-mode targets. |

## Workflow

1. Plug the XIAO into a host. The `XIAO DFU` drive appears.
2. Drop a firmware bundle (`*.zip` produced by `nrfutil pkg generate`) into the drive root.
3. Copy a `CONFIG.TXT` from the repo and either set `ble_name` to match the advertised BLE name you want to update, for example `RAK4631_OTA`, or leave `ble_name` empty and set `ble_mac` to target a known BLE MAC address.
4. **Eject the drive** (or unplug if the XIAO is battery-powered). Eject makes the LUN not-ready and transfers exclusive FAT ownership to the updater for the rest of this boot.
5. The updater reloads the just-copied `CONFIG.TXT`, scans for a target advertising the Legacy DFU service, matching the configured BLE name, or matching the configured BLE MAC address. It optionally sends the buttonless trigger to kick the target from app mode into bootloader, then runs the full DFU sequence.
6. On success the `.zip` is deleted from the drive; the `LOG.TXT` keeps the history.

Reset or power-cycle the updater before staging another bundle. The MSC LUN is intentionally never re-enabled after an eject/DFU trigger, which prevents the host and firmware from sharing FAT caches.

If QSPI flash initialization fails, USB deliberately re-enumerates as CDC-only instead of advertising a zero-capacity disk; the serial log then contains the storage error.

## Triggers

Two ways to start a DFU sequence:

| Trigger | When | How it's detected |
|---|---|---|
| **Eject** | Host ejects the drive (USB still connected) | SCSI Start-Stop Unit (`load_eject=1, start=0`) callback; LUN becomes not-ready before firmware reads FAT |
| **Boot off battery** | XIAO powers up with no VBUS and a `.zip` already on the drive | `NRF_POWER->USBREGSTATUS` read at boot |

Only one runs per boot; after success or final failure the firmware sits idle until reboot.

## Heltec V4 serial updater

The `heltec_v4` environment turns the ESP32-S3 V4 into a Nordic Legacy DFU
central. It deliberately does **not** expose USB MSC. A bundle is written into
the existing 3.375 MiB SPIFFS partition only after the host supplies an
immutable payload, its exact length, and SHA-256. Upload uses 512-byte credits:
the host must receive the exact cumulative `ACK` before sending another chunk.
The checksum sidecar is committed only after the full hash and Legacy manifest
pass, and the bundle is re-hashed immediately before DFU. Rebooting clears the
target selection; a staged ZIP alone can never trigger an update.

`UPLOAD` deliberately clears the prior target and removes any prior staged
bundle before `READY 512`. A failed or power-interrupted upload therefore
cannot fall back to old firmware. If a raw frame ends early, the command parser
is quarantined until reboot so leftover ZIP bytes cannot become commands. The
updater mounts SPIFFS with auto-format disabled; a mount failure is fail-closed
and never formats MeshCore's persisted identity/preferences.

Build it with pinned Arduino-ESP32 2.0.17 / NimBLE-Arduino 1.4.2:

```bash
pio run -e heltec_v4
```

Before installing the V4 updater, make a coherent full 16 MiB recovery image.
Use the exact V4 port; do not substitute an auto-discovered first port:

```bash
esptool.py --chip esp32s3 --port /dev/ttyACM_V4 get_security_info
esptool.py --chip esp32s3 --port /dev/ttyACM_V4 flash_id

esptool.py --chip esp32s3 --port /dev/ttyACM_V4 \
  --after no_reset read_flash \
  0x000000 0x1000000 heltec-v4-before-dfu.bin
test "$(stat -c %s heltec-v4-before-dfu.bin)" = 16777216
sha256sum heltec-v4-before-dfu.bin | tee heltec-v4-before-dfu.bin.sha256

# Verify the same stopped device before allowing it to boot, then derive the
# live partition table and OTA selector from that one coherent image.
esptool.py --chip esp32s3 --port /dev/ttyACM_V4 \
  --before no_reset --after hard_reset --no-stub verify_flash --diff no \
  --flash_mode keep --flash_freq keep --flash_size 16MB \
  0x000000 heltec-v4-before-dfu.bin
dd if=heltec-v4-before-dfu.bin of=heltec-v4-partitions-before.bin \
  bs=1 skip=$((0x8000)) count=$((0x1000)) status=none
dd if=heltec-v4-before-dfu.bin of=heltec-v4-otadata-before.bin \
  bs=1 skip=$((0xe000)) count=$((0x2000)) status=none
```

esptool 4.5 buffers a requested read and creates the destination only after
the complete range arrives. Use stable power and exclusive serial ownership;
an interrupted read is not a recovery image. Some virtualized native
USB-JTAG links cannot sustain the stub's bulk-read handshake. If a full read
stalls, stop and use a persistent ROM-mode reader or external programmer,
then require a whole-device digest match before relying on the backup.

Stop before any write if secure boot or flash encryption is enabled, if the
reported flash is not the expected 16 MiB device, or if the live table/readback
cannot be preserved and checked. Those states require a board-specific signed
or encrypted recovery path which this updater does not provide.

Do not use `pio -t upload` for the first test: it also writes bootloader,
partition-table, and OTA metadata images. First compare the live partition
table with `.pio/build/heltec_v4/partitions.bin` and independently confirm that
the live OTA selector boots `ota_0` (`app0` at `0x10000`). Only under those two
conditions, write the app image alone:

```bash
esptool.py --chip esp32s3 --port /dev/ttyACM_V4 --no-stub write_flash \
  --flash_mode keep --flash_freq keep --flash_size 16MB \
  --no-compress --verify \
  0x10000 .pio/build/heltec_v4/firmware.bin
```

If either condition is unknown, stop and use a recovery-aware OTA method; do
not write any generated bootloader, table, or `otadata` artifact.

The helper snapshots at most 2 MiB once, performs ZIP/CRC/manifest preflight on
those exact bytes, requires the exact serial port and caller-supplied target,
and stages only by default. It stages and receives all chunk ACKs before it
authorizes `TARGET`; `START` is sent only when requested:

```bash
# Safe default: stage, validate, and stop without sending BLE firmware.
python3 tools/v4_serial_upload.py \
  --port /dev/ttyACM_V4 \
  --zip firmware.zip \
  --target-mac AA:BB:CC:DD:EE:FF

# Transmit only when --start is present explicitly.
python3 tools/v4_serial_upload.py \
  --port /dev/ttyACM_V4 \
  --zip firmware.zip \
  --target-mac AA:BB:CC:DD:EE:FF \
  --start
```

`--target-name 'Xiao_nrf52|XIAO_DFU'` can replace `--target-mac`, but a known
MAC is safer for the first hardware test. The updater accepts that MAC and the
Nordic bootloader convention MAC+1 across the buttonless reboot. The helper
requires Python 3.9+ and `pyserial`.

If the test build, partition table, USB CDC, or BLE port misbehaves, put the V4
in its ROM download mode and restore the sealed full-flash image:

```bash
sha256sum -c heltec-v4-before-dfu.bin.sha256
esptool.py --chip esp32s3 --port /dev/ttyACM_V4 --no-stub write_flash \
  --flash_mode keep --flash_freq keep --flash_size 16MB \
  --no-compress --verify \
  0x000000 heltec-v4-before-dfu.bin
esptool.py --chip esp32s3 --port /dev/ttyACM_V4 --no-stub verify_flash \
  --diff no --flash_mode keep --flash_freq keep --flash_size 16MB \
  0x000000 heltec-v4-before-dfu.bin
```

The V4 serial commands are also usable manually at 115200 baud: `STATUS`,
`UPLOAD <bytes> <sha256>` followed only after `READY 512` by at most 512 raw
bytes, waiting for the exact cumulative `ACK <offset>` after every chunk; then
`TARGET NAME ...` or `TARGET MAC ...`, `START`, and `ERASE`. There is no boot,
upload-complete, GPIO, or scan-only trigger.

## CONFIG.TXT

`key=value` per line, `#` or `;` start a comment, Missing keys use defaults.

```ini
# Substring filter for advertised BLE name.
# When set, name matching has priority over ble_mac.
# Multiple names can be OR'd with '|', useful when an app and its bootloader
# advertise under different names, like oltaco's OTAFIX bootloader does.
ble_name=RAK4631 | 4631_DFU

# Optional fixed BLE MAC target.
# Used only when ble_name is empty.
# Format: AA:BB:CC:DD:EE:FF
# The scanner accepts the configured MAC or MAC+1, which matches Nordic's
# common bootloader address convention after buttonless DFU.
# Leave empty to use the original UUID fallback behavior.
ble_mac=

# Packet Receipt Notification cadence (writes per ACK from the peer).
# 10 is the safe Nordic default. Higher = faster but risk overflowing the
# peer's RX queue.
prn=10

# Negotiate MTU 247 after connect (5–10x faster stream).
# Some older bootloaders ignore the request and we fall back to 20 B writes.
high_mtu=1

# Number of DFU attempts before giving up.
retries=3

# Seconds to wait between failed attempts. The bootloader needs time to
# settle after a reset before it'll accept another START_DFU.
retry_cooldown=5

# Seconds to wait only when RESET could not be completed and the link was
# already lost or had to be forced down. Legacy bootloaders may then retain
# partial DFU state until their inactivity watchdog fires. A confirmed RESET
# and pre-connect/discovery failures use retry_cooldown instead.
wedge_cooldown=60

# Minimum RSSI (dBm, negative). Ads weaker than this are rejected during
# scan. Default -90; -127 = no minimum. Useful on a drone to refuse flashing when
# the signal isn't strong enough to reliably stream.
min_rssi=-60

# BLE TX power in dBm. nRF52840 allowed values:
#   -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8
# Default 4. Crank to 8 for max range; an unsupported value falls back to
# 0 dBm.
tx_power=8

# Scan timeout in seconds. 0 = scan forever (the default; intended for
# drone use where the target may take minutes to come into range).
# Non-zero caps the wait; on expiry the sequence gives up without
# consuming any DFU retry slot.
scan_timeout=0

# When set, every rejected advertisement is logged with reason + MAC.
# Useful for diagnosing "my target isn't being picked up". Off in the field.
scan_debug=0
```

Notes:

- A configured `ble_name` switches the scanner from UUID matching to **name-substring** matching. Most Nordic app-mode firmwares expose the Legacy DFU service in their GATT database but do not advertise the UUID; matching on the device name is often the only way to find them before connecting.
- If `ble_name` is empty and `ble_mac` is configured, the scanner uses **strict MAC matching** instead. Only the configured BLE MAC address or MAC+1 is accepted. This is useful when the target name is unknown, missing, shortened, or changes between app mode and bootloader mode.
- If both `ble_name` and `ble_mac` are empty, the original fallback behavior is used: only peers that explicitly advertise the Legacy DFU service UUID are considered, which is typical of bootloader-mode targets.
- `ble_name` accepts multiple substrings joined by `|`. Example: `RAK4631 | 4631_DFU` matches the RAK app *and* its bootloader, which advertise under different names.
- **MAC+1 fallback after buttonless**: after we send the buttonless trigger, the next scan automatically also accepts the same MAC or MAC+1 (Nordic's bootloader convention). This works both for the existing buttonless flow and for the optional `ble_mac` target mode.

`retries` only counts post-scan DFU attempts. Scan failures (with `scan_timeout=0`, impossible) and buttonless triggers don't consume retries.

## LED indicators

XIAO has 3 LEDs (R/G/B, active-low). RAK4631 has 2 (green + blue, active-high) and uses an alternating pattern to signal failure instead of a red LED.

| State | XIAO | RAK4631 |
|---|---|---|
| Idle, USB not configured | BLUE slow blink (~1 Hz) | BLUE slow blink (~1 Hz) |
| USB configured by host | BLUE solid | BLUE solid |
| DFU running, streaming | GREEN blink, period shrinks 0%→95% | same |
| DFU succeeded | GREEN solid | GREEN solid |
| DFU failed (after retries) | RED solid | GREEN+BLUE alternating ~4 Hz |

The progress LED is driven from inside the DFU stream loop (the main loop is blocked during DFU), so it animates in real time.

## LOG.TXT

Lines are `[hh:mm:ss] message`. Timestamps are boot-relative - there's no RTC.

The log is mirrored to both Serial (USB CDC) and `LOG.TXT`. File writes are skipped while the host owns the block device. A USB `SET_CONFIGURATION` callback only says the composite device is connected—it does not prove that an operating system mounted FAT—so the updater uses an explicit ownership state instead. Eject first makes the LUN not-ready, drains block I/O, syncs flash, clears the firmware FAT cache, and only then enables file logging. Bluefruit callback tasks only copy fixed-size events into a queue; the Arduino task drains that queue and performs all Serial/SdFat/QSPI logging, so callback logging cannot race bundle reads. Serial output is always live.

| Drive state | Serial | LOG.TXT |
|---|---|---|
| Before eject / host owns media | ✓ | skipped |
| Ejected (USB still plugged, LUN not-ready) | ✓ | ✓ |
| Battery-triggered boot | nothing reads it | ✓ |

Useful scan/debug lines when `ble_mac` is configured:

```text
cfg: ble_mac=AA:BB:CC:DD:EE:FF
scan: fixed MAC target AA:BB:CC:DD:EE:FF
```

When `scan_debug=1`, advertisements rejected because they do not match the configured MAC target are logged with reason `mac?`.

## Supported DFU bundles

| Manifest section | Supported | Notes |
|---|---|---|
| `application` | yes | App-only. Sends size tuple `[0, 0, bin_size]`. |
| `bootloader` | yes | Bootloader-only. Sends `[0, bin_size, 0]`. |
| `softdevice` | yes | SoftDevice-only. Sends `[bin_size, 0, 0]`. |
| `softdevice_bootloader` | yes | SD+BL combined. Sends `[sd_size, bl_size, 0]`; both sizes are required and their sum must equal the combined `.bin` size. Top-level and `info_read_only_metadata` size fields are accepted. |
| `bootloader_application` | no | Combined-bin Secure DFU package; not valid for this Legacy DFU client. |
| `softdevice_application` | no | Combined-bin Secure DFU package; not valid for this Legacy DFU client. |
| `softdevice_bootloader_application` | no | Combined-bin Secure DFU package; not valid for this Legacy DFU client. |

ZIPs must use `STORE` (no compression). This is what `nrfutil pkg generate` produces by default.

An accepted ZIP must contain exactly one of the four supported Legacy DFU
sections. Multi-entry Legacy packages (for example `softdevice_bootloader` plus
`application`) require a second DFU session and are rejected; the updater never
silently flashes one entry and deletes the unapplied remainder. The `.zip` is
also validated by locating the referenced, non-empty `.bin` and its `.dat` init
packet. This updater requires a 1..128-byte `.dat` for every bundle, matching
the OTAFIX SDK11 bootloader's init-packet buffer.

## Build & flash

Three build environments are available:

| `pio` env | Board | QSPI flash | Drive label |
|---|---|---|---|
| `xiao_nrf52840` | Seeed XIAO nRF52840 | on-board Puya P25Q16H (2 MB) | `XIAO DFU` |
| `rak4631` | RAKwireless RAK4631 | external RAK15001 module - GigaDevice GD25Q16 (2 MB) | `RAK DFU` |
| `heltec_v4` | Heltec WiFi LoRa 32 V4 | internal 3.375 MiB SPIFFS partition | none (serial framing) |

```bash
pio run -e xiao_nrf52840                  # build XIAO target
pio run -e xiao_nrf52840 -t upload        # flash XIAO via factory bootloader (nrfutil)

# or
pio run -e rak4631                        # build RAK target
pio run -e rak4631 -t upload              # flash via factory bootloader (nrfutil)

# Heltec V4 ESP32-S3 BLE central
pio run -e heltec_v4

# focused host-side parser/protocol/retry-policy tests
bash tests/run_host_tests.sh
python3 -m unittest -v tests/test_v4_serial_upload.py

# optional debug
pio device monitor                        # 115200 baud, watches serial
```

Requirements: PlatformIO with the upstream `nordicnrf52` platform. The project's `platformio.ini` pins the BSP to a meshcore-dev fork of `framework-arduinoadafruitnrf52` (BLE stack patches). Board JSONs and linker script are vendored under `boards/`. Variants under `variants/xiao_nrf52/` and `variants/rak4631/`.

The Heltec environment separately pins `platformio/espressif32@6.11.0`
(Arduino-ESP32 2.0.17), `bblanchon/ArduinoJson@7.4.3`, and
`h2zero/NimBLE-Arduino@1.4.2`. It clears
the base environment's nRF52 `platform_packages` so the two BSPs cannot mix.

No bootloader replacement is required on either board — the factory UF2 bootloader works.

## Project layout

```text
src/
  main.cpp           - state machine, LED rendering, triggers, glue
  storage.{h,cpp}    - QSPI flash bring-up, FAT12 mount, mini-formatter
  usb_msc.{h,cpp}    - TinyUSB MSC class, eject detection via SCSI Start-Stop
  logger.{h,cpp}     - printf-style logger to Serial + LOG.TXT
  deferred_logger.{h,cpp} - callback-task event queue drained by the main task
  config.{h,cpp}     - CONFIG.TXT parser
  zip_reader.{h,cpp} - minimal STORE-only ZIP walker
  firmware_zip.{h,cpp} - manifest.json parsing, locates .bin + .dat
  dfu_image_layout.{h,cpp} - validates manifest type/sizes and encodes the Legacy DFU size tuple
  ble_scanner.{h,cpp}  - Bluefruit central scan, UUID + pipe-delimited name + RSSI + MAC/MAC+1 filtering
  dfu_legacy.{h,cpp}   - Legacy DFU client state machine (mirrors LegacyDfuImpl.java)
  v4/                  - Heltec V4 serial staging, SPIFFS ZIP reader, NimBLE central, DFU state machine
tools/v4_serial_upload.py - explicit-port host staging helper (START is opt-in)
boards/              - board JSONs (xiao, rak4631) + s140 linker script
variants/xiao_nrf52/ - XIAO pin map + variant.cpp (BSP doesn't ship one)
variants/rak4631/    - RAK4631 (WisBlock) pin map + variant.cpp
```

## Known limitations

- Single-LUN MSC; only one zip is expected.
- Combined SD+BL+App in a single zip isn't supported.
- No Secure DFU (only Legacy, as per the project scope).
- No host-set wall clock; `LOG.TXT` timestamps are boot-relative.
- MSC is one-shot per boot: after eject, reset/power-cycle before staging another bundle.
- Progress LED + DFU stream both share the main thread, so log throughput in the stream loop is the bottleneck. With `high_mtu=1` we get ~14 KB/s, which means ~13 s for a 191 KB SD+BL bundle.
- Logs written while the host owns MSC don't reach `LOG.TXT` (Serial-only).
- The Heltec V4 path was bench-qualified on 2026-08-22 against a Seeed XIAO
  nRF52840 using exact-MAC targeting. The run covered app→buttonless→boot,
  application-only and bootloader-only Legacy DFU, every response-bearing
  opcode, activation/reconnect verification, failure cleanup, and full-flash
  V4 recovery. It intentionally has no USB MSC and logs only to Serial. The
  app→boot transition is limited to one trigger and its strict original-MAC/
  MAC+1 rescan is capped at 45 seconds.
- A target with an invalid application may remain in a USB-only bootloader
  recovery mode while its cable is attached. That transport state does not
  advertise BLE; restore its application over the bootloader's wired serial
  DFU interface, then retry the normal app→buttonless BLE path.
- Legacy `ACTIVATE` success is inferred from the expected disconnect. Hardware
  validation must reconnect to the target and independently verify its running
  firmware/version before the V4 BLE path is considered proven.

## References

- [Nordic nRF5 SDK Legacy DFU](https://infocenter.nordicsemi.com/topic/sdk_nrf5_v17.1.0/lib_bootloader_dfu_keys.html) - protocol spec the client is built against.
- [Android DFU Library](https://github.com/nordicsemi/Android-DFU-Library) - authoritative reference for legacy DFU protocol.
- [Adafruit nRF52 Bootloader](https://github.com/adafruit/adafruit_nrf52_bootloader) - USB/MSC code, the reference for FAT layout + UF2.
