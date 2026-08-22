#include "v4_zip.h"

#include <FS.h>
#include <SPIFFS.h>

namespace v4_zip {

namespace {

constexpr uint32_t kLocalHeaderSignature = 0x04034b50;
constexpr uint32_t kCentralHeaderSignature = 0x02014b50;
constexpr uint32_t kEndRecordSignature = 0x06054b50;

fs::File s_file;
uint32_t s_cursor = 0;
uint32_t s_archive_size = 0;

bool read_u16(uint16_t* out) {
  uint8_t bytes[2];
  if (!out || s_file.read(bytes, sizeof(bytes)) != sizeof(bytes)) return false;
  *out = static_cast<uint16_t>(bytes[0]) |
         static_cast<uint16_t>(bytes[1] << 8);
  return true;
}

bool read_u32(uint32_t* out) {
  uint8_t bytes[4];
  if (!out || s_file.read(bytes, sizeof(bytes)) != sizeof(bytes)) return false;
  *out = static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) |
         (static_cast<uint32_t>(bytes[3]) << 24);
  return true;
}

bool span_valid(uint32_t offset, uint32_t size) {
  return offset <= s_archive_size && size <= s_archive_size - offset;
}

bool at_directory_boundary() {
  if (!span_valid(s_cursor, 4) || !s_file.seek(s_cursor, SeekSet)) return false;
  uint32_t signature = 0;
  return read_u32(&signature) &&
         (signature == kCentralHeaderSignature ||
          signature == kEndRecordSignature);
}

}  // namespace

bool open(const char* path) {
  close();
  s_file = SPIFFS.open(path, FILE_READ);
  if (!s_file) return false;
  uint64_t size = s_file.size();
  if (size > UINT32_MAX) {
    close();
    return false;
  }
  s_archive_size = static_cast<uint32_t>(size);
  return true;
}

void close() {
  if (s_file) s_file.close();
  s_cursor = 0;
  s_archive_size = 0;
}

bool next(Entry* out) {
  if (!out || !s_file || !span_valid(s_cursor, 30) ||
      !s_file.seek(s_cursor, SeekSet)) {
    return false;
  }

  uint32_t signature = 0;
  if (!read_u32(&signature) || signature != kLocalHeaderSignature) return false;

  uint16_t flags = 0;
  uint16_t method = 0;
  if (!s_file.seek(s_cursor + 6, SeekSet) || !read_u16(&flags) ||
      !read_u16(&method)) {
    return false;
  }
  // Encryption and trailing data descriptors make local-header walking
  // ambiguous. nrfutil Legacy packages use neither and store their members.
  if ((flags & (0x0001u | 0x0008u)) != 0 || method != 0) return false;

  uint32_t expected_crc = 0;
  uint32_t compressed_size = 0;
  uint32_t uncompressed_size = 0;
  uint16_t name_len = 0;
  uint16_t extra_len = 0;
  if (!s_file.seek(s_cursor + 14, SeekSet) || !read_u32(&expected_crc) ||
      !read_u32(&compressed_size) || !read_u32(&uncompressed_size) ||
      !read_u16(&name_len) || !read_u16(&extra_len)) {
    return false;
  }
  if (compressed_size != uncompressed_size ||
      name_len == 0 || name_len >= sizeof(out->name)) {
    return false;
  }

  uint64_t data_offset64 = static_cast<uint64_t>(s_cursor) + 30u +
                           name_len + extra_len;
  if (data_offset64 > UINT32_MAX) return false;
  uint32_t data_offset = static_cast<uint32_t>(data_offset64);
  if (!span_valid(data_offset, compressed_size) ||
      !s_file.seek(s_cursor + 30, SeekSet) ||
      s_file.read(reinterpret_cast<uint8_t*>(out->name), name_len) != name_len) {
    return false;
  }
  out->name[name_len] = '\0';
  out->data_offset = data_offset;
  out->size = uncompressed_size;
  out->crc32 = expected_crc;
  s_cursor = data_offset + compressed_size;
  return true;
}

bool find(const char* name, Entry* out) {
  if (!name || !out || !s_file) return false;
  s_cursor = 0;
  Entry entry;
  bool found = false;
  while (next(&entry)) {
    if (strcmp(entry.name, name) == 0) {
      if (found) return false;  // ambiguous duplicate member name
      *out = entry;
      found = true;
    }
  }
  // next() also returns false for malformed/truncated local headers. Accept a
  // completed walk only when the cursor is at a real central-directory/EOCD
  // record, never merely because required entries appeared earlier.
  return found && at_directory_boundary();
}

int read(const Entry& entry, uint32_t offset, void* buf, uint32_t len) {
  if (!buf || !s_file || offset >= entry.size) return offset == entry.size ? 0 : -1;
  if (len > entry.size - offset) len = entry.size - offset;
  uint32_t absolute = entry.data_offset + offset;
  if (!span_valid(absolute, len) || !s_file.seek(absolute, SeekSet)) return -1;
  return s_file.read(reinterpret_cast<uint8_t*>(buf), len);
}

bool verify_crc(const Entry& entry) {
  static constexpr uint32_t kNibbleTable[16] = {
      0x00000000u, 0x1db71064u, 0x3b6e20c8u, 0x26d930acu,
      0x76dc4190u, 0x6b6b51f4u, 0x4db26158u, 0x5005713cu,
      0xedb88320u, 0xf00f9344u, 0xd6d6a3e8u, 0xcb61b38cu,
      0x9b64c2b0u, 0x86d3d2d4u, 0xa00ae278u, 0xbdbdf21cu,
  };
  uint32_t crc = 0xffffffffu;
  uint32_t offset = 0;
  uint8_t buf[1024];
  while (offset < entry.size) {
    uint32_t want = entry.size - offset;
    if (want > sizeof(buf)) want = sizeof(buf);
    int n = read(entry, offset, buf, want);
    if (n != static_cast<int>(want)) return false;
    for (int i = 0; i < n; ++i) {
      crc ^= buf[i];
      crc = (crc >> 4) ^ kNibbleTable[crc & 0x0fu];
      crc = (crc >> 4) ^ kNibbleTable[crc & 0x0fu];
    }
    offset += static_cast<uint32_t>(n);
  }
  return ~crc == entry.crc32;
}

}  // namespace v4_zip
