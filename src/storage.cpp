#include "storage.h"

#include <Arduino.h>
#include <nrfx_qspi.h>

namespace storage {

static Adafruit_FlashTransport_QSPI s_transport;
static Adafruit_SPIFlash            s_flash(&s_transport);
static FatVolume                    s_fatfs;

Adafruit_SPIFlash& flash() { return s_flash; }
FatVolume&         fs()    { return s_fatfs; }

bool prepare_for_host_access() {
  // Order matters: cacheClear() may write a dirty SdFat sector into the flash
  // library's 4 KiB erase cache. Flush that erase cache only afterwards.
  if (s_fatfs.cacheClear() == nullptr) return false;
  return s_flash.syncBlocks();
}

bool refresh_after_host_write() {
  // Raw MSC writes land in the flash library's erase cache. Commit those
  // first, then discard the now-stale (and, by ownership contract, clean)
  // SdFat sector cache before opening files from firmware.
  if (!s_flash.syncBlocks()) return false;
  return s_fatfs.cacheClear() != nullptr;
}

// Board-specific QSPI pinout + drive label. XIAO routes its on-board Puya
// P25Q16H to a fixed P0 set; RAK4631 brings the SoC QSPI out to WisBlock
// Slot A where the user wires their own GD25Q16.
#if defined(BOARD_RAK4631)
  static constexpr uint8_t kQspiSck = 3;   // P0.03
  static constexpr uint8_t kQspiCsn = 26;  // P0.26
  static constexpr uint8_t kQspiIo0 = 30;  // P0.30
  static constexpr uint8_t kQspiIo1 = 29;  // P0.29
  static constexpr uint8_t kQspiIo2 = 28;  // P0.28
  static constexpr uint8_t kQspiIo3 = 2;   // P0.02
  static constexpr const char kVolLabel[12] = "RAK DFU    ";
#else  // BOARD_XIAO_NRF52840
  static constexpr uint8_t kQspiSck = 21;
  static constexpr uint8_t kQspiCsn = 25;
  static constexpr uint8_t kQspiIo0 = 20;
  static constexpr uint8_t kQspiIo1 = 24;
  static constexpr uint8_t kQspiIo2 = 22;
  static constexpr uint8_t kQspiIo3 = 23;
  static constexpr const char kVolLabel[12] = "XIAO DFU   ";
#endif

// SdFat's FatFormatter rejects volumes <= 6 MB because it's built for SD cards.
// We need to format a 2 MB QSPI flash, so we write a minimal FAT12 image
// directly. Layout (all values are 512 B sectors):
//
//   sector  0       : Boot block / BPB
//   sector  1..2    : FAT #1 (2 sectors)
//   sector  3..4    : FAT #2 (2 sectors)
//   sector  5..12   : Root directory (128 entries, 8 sectors, empty)
//   sector 13..     : Data area (~510 clusters of 8 sectors each)
//
// 128 root entries because macOS burns through dir slots fast with long
// filenames + its `.DS_Store`, `._foo`, `.Spotlight-V100`, `.fseventsd`,
// `.Trashes` sidecars. With the previous 16 we'd report "disk full" while
// clusters were nearly untouched.
//
// Each cluster is 4 KB, identical to the flash erase block, so user file
// writes align naturally with the wear layout.
static bool format_flash() {
  Serial.println("storage: erasing chip...");
  uint32_t t0 = millis();
  if (!s_flash.eraseChip()) {
    Serial.println("storage: eraseChip() failed");
    return false;
  }
  // eraseChip is async on the flash; wait for it to finish.
  while (s_flash.readStatus() & 0x01) {
    delay(1);
  }
  Serial.print("storage: chip erased in "); Serial.print(millis() - t0); Serial.println(" ms");

  uint8_t sec[512];

  // ---- Boot block / BIOS Parameter Block ----
  memset(sec, 0, sizeof(sec));
  sec[0] = 0xEB; sec[1] = 0x3C; sec[2] = 0x90;          // JMP boot
  memcpy(sec + 3,  "MSDOS5.0", 8);                       // OEM
  sec[11] = 0x00; sec[12] = 0x02;                        // 512 bytes/sector
  sec[13] = 8;                                           // 8 sectors/cluster
  sec[14] = 1; sec[15] = 0;                              // 1 reserved sector
  sec[16] = 2;                                           // 2 FATs
  sec[17] = 128; sec[18] = 0;                            // 128 root-dir entries (8 sectors)
  sec[19] = 0x00; sec[20] = 0x10;                        // total sectors = 4096
  sec[21] = 0xF8;                                        // media descriptor
  sec[22] = 2;  sec[23] = 0;                             // sectors per FAT = 2
  sec[24] = 1;  sec[25] = 0;                             // sectors per track
  sec[26] = 1;  sec[27] = 0;                             // heads
  sec[36] = 0x80;                                        // physical drive
  sec[38] = 0x29;                                        // extended boot sig
  sec[39] = 0x42; sec[40] = 0x42; sec[41] = 0x42; sec[42] = 0x42;
  memcpy(sec + 43, kVolLabel, 11);                       // volume label
  memcpy(sec + 54, "FAT12   ", 8);                       // FS type
  sec[510] = 0x55; sec[511] = 0xAA;                      // boot signature
  if (!s_flash.writeBlocks(0, sec, 1)) return false;

  // ---- First sector of each FAT: media descriptor + EOC for entry 1 ----
  memset(sec, 0, sizeof(sec));
  sec[0] = 0xF8; sec[1] = 0xFF; sec[2] = 0xFF;
  if (!s_flash.writeBlocks(1, sec, 1)) return false;     // FAT1
  if (!s_flash.writeBlocks(3, sec, 1)) return false;     // FAT2

  // ---- Remaining FAT sector and root directory: all zeros ----
  // (Erased flash is 0xFF; zero means "free cluster" / "unused dir entry".)
  memset(sec, 0, sizeof(sec));
  if (!s_flash.writeBlocks(2, sec, 1)) return false;     // FAT1 second sector
  if (!s_flash.writeBlocks(4, sec, 1)) return false;     // FAT2 second sector
  for (uint32_t i = 0; i < 8; i++) {                     // root dir, 8 sectors
    if (!s_flash.writeBlocks(5 + i, sec, 1)) return false;
  }

  s_flash.syncBlocks();
  Serial.println("storage: FAT12 image written");
  return true;
}

// Detect whether the existing FAT image was written by an old build of this
// firmware (smaller root directory). Returns true if our current 128-entry
// layout is in place, false if we should reformat.
static bool layout_is_current() {
  uint8_t sec[512];
  if (!s_flash.readBlocks(0, sec, 1)) return false;
  // Boot sector signature must be present.
  if (sec[510] != 0x55 || sec[511] != 0xAA) return false;
  // Root directory entries (BPB offset 17, 16-bit LE) must match what
  // format_flash() writes today.
  uint16_t root_entries = (uint16_t)sec[17] | ((uint16_t)sec[18] << 8);
  return root_entries == 128;
}

bool begin() {
  // ---------- low-level QSPI bring-up (bypasses Adafruit transport) ----------
  // We control nrfx_qspi directly so failures are visible. The Adafruit
  // transport ignores all return codes.
  nrfx_qspi_config_t cfg = {};
  cfg.xip_offset           = 0;
  cfg.pins.sck_pin         = kQspiSck;
  cfg.pins.csn_pin         = kQspiCsn;
  cfg.pins.io0_pin         = kQspiIo0;
  cfg.pins.io1_pin         = kQspiIo1;
  cfg.pins.io2_pin         = kQspiIo2;
  cfg.pins.io3_pin         = kQspiIo3;
  cfg.prot_if.readoc       = NRF_QSPI_READOC_FASTREAD; // 1-bit, no QE bit needed
  cfg.prot_if.writeoc      = NRF_QSPI_WRITEOC_PP;
  cfg.prot_if.addrmode     = NRF_QSPI_ADDRMODE_24BIT;
  cfg.prot_if.dpmconfig    = false;
  cfg.phy_if.sck_freq      = NRF_QSPI_FREQ_32MDIV16; // ~2 MHz, safest
  cfg.phy_if.sck_delay     = 10;
  cfg.phy_if.spi_mode      = NRF_QSPI_MODE_0;
  cfg.phy_if.dpmen         = false;
  cfg.irq_priority         = 7;

  nrfx_err_t err = nrfx_qspi_init(&cfg, NULL, NULL);
  Serial.print("storage: nrfx_qspi_init -> 0x"); Serial.println(err, HEX);
  if (err != NRFX_SUCCESS) {
    Serial.println("storage: nrfx_qspi_init failed");
    return false;
  }

  // Wake the chip from deep power-down (Seeed bootloader leaves it asleep).
  {
    nrf_qspi_cinstr_conf_t c = {};
    c.opcode = 0xAB; c.length = NRF_QSPI_CINSTR_LEN_1B;
    c.io2_level = true; c.io3_level = true;
    nrfx_qspi_cinstr_xfer(&c, NULL, NULL);
  }
  delayMicroseconds(50);

  // Direct JEDEC read to confirm the bus is alive.
  {
    uint8_t jedec[3] = { 0, 0, 0 };
    nrf_qspi_cinstr_conf_t c = {};
    c.opcode = 0x9F; c.length = NRF_QSPI_CINSTR_LEN_4B;
    c.io2_level = true; c.io3_level = true;
    nrfx_qspi_cinstr_xfer(&c, NULL, jedec);
    Serial.print("storage: direct JEDEC = ");
    for (int i = 0; i < 3; i++) {
      if (jedec[i] < 0x10) Serial.print('0');
      Serial.print(jedec[i], HEX); Serial.print(' ');
    }
    Serial.println();
  }

  // Hand off to Adafruit_SPIFlash for filesystem-level access. The library's
  // default device table is too narrow and rejects valid JEDEC IDs we care
  // about (Puya P25Q16H on the XIAO; GigaDevice GD25Q16 wired into RAK
  // Slot A), so we pass the device descriptor explicitly per board.
#if defined(BOARD_RAK4631)
  static const SPIFlash_Device_t s_flash_dev = GD25Q16C;
#else
  static const SPIFlash_Device_t s_flash_dev = P25Q16H;
#endif
  if (!s_flash.begin(&s_flash_dev, 1)) {
    Serial.println("storage: flash.begin() failed (no JEDEC match?)");
    return false;
  }

  Serial.print("storage: JEDEC = 0x");
  Serial.println(s_flash.getJEDECID(), HEX);
  Serial.print("storage: QSPI flash size = ");
  Serial.print(s_flash.size() / 1024);
  Serial.println(" KB");

  bool need_format = !s_fatfs.begin(&s_flash);
  if (!need_format && !layout_is_current()) {
    Serial.println("storage: old FAT layout detected, reformatting");
    need_format = true;
  }

  if (need_format) {
    if (!format_flash()) return false;
    if (!s_fatfs.begin(&s_flash)) {
      Serial.println("storage: fatfs.begin() FAILED after format");
      return false;
    }
  }

  Serial.println("storage: filesystem mounted");
  return true;
}

static bool ends_with_zip(const char* name) {
  size_t n = strlen(name);
  if (n < 4) return false;
  const char* ext = name + n - 4;
  return ext[0] == '.' &&
         (ext[1] == 'z' || ext[1] == 'Z') &&
         (ext[2] == 'i' || ext[2] == 'I') &&
         (ext[3] == 'p' || ext[3] == 'P');
}

int find_single_zip(char* out, size_t out_len) {
  File root = s_fatfs.open("/", O_RDONLY);
  if (!root) return -1;

  int count = 0;
  File f;
  while (f.openNext(&root, O_RDONLY)) {
    if (!f.isDir() && !f.isHidden()) {
      char name[64];
      f.getName(name, sizeof(name));
      // Skip macOS metadata siblings (`._foo.zip`) and any other dot-prefixed
      // files - Finder writes these on FAT volumes to hold extended attrs.
      if (name[0] != '.' && ends_with_zip(name)) {
        if (count == 0 && out_len > 0) {
          snprintf(out, out_len, "%s", name);
        }
        count++;
      }
    }
    f.close();
  }
  root.close();
  return count;
}

bool delete_file(const char* name) {
  return s_fatfs.remove(name);
}

}  // namespace storage
