#include "v4_storage.h"

#include <FS.h>
#include <SPIFFS.h>
#include <mbedtls/sha256.h>

namespace v4_storage {

namespace {

constexpr const char* kStagedShaPath = "/firmware.sha256";
constexpr uint32_t kMaxBundleBytes = 2u * 1024u * 1024u;
constexpr uint32_t kReceiveIdleTimeoutMs = 10000;
constexpr uint32_t kUploadChunkBytes = 512;

bool s_ready = false;
fs::File s_upload_file;
mbedtls_sha256_context s_upload_sha;
bool s_upload_prepared = false;
uint8_t s_upload_expected[32];
uint32_t s_upload_size = 0;
bool s_received_valid = false;
char s_received_hex[65] = {};

int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool parse_sha256(const char* text, uint8_t out[32]) {
  if (!text || strlen(text) != 64) return false;
  for (size_t i = 0; i < 32; ++i) {
    int hi = hex_value(text[i * 2]);
    int lo = hex_value(text[i * 2 + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  return true;
}

void encode_sha256(const uint8_t digest[32], char out[65]) {
  static constexpr char kHex[] = "0123456789abcdef";
  for (size_t i = 0; i < 32; ++i) {
    out[i * 2] = kHex[digest[i] >> 4];
    out[i * 2 + 1] = kHex[digest[i] & 0x0f];
  }
  out[64] = '\0';
}

void close_upload() {
  if (s_upload_file) s_upload_file.close();
  if (s_upload_prepared) mbedtls_sha256_free(&s_upload_sha);
  s_upload_prepared = false;
  s_upload_size = 0;
  memset(s_upload_expected, 0, sizeof(s_upload_expected));
}

bool remove_path(const char* path) {
  return !SPIFFS.exists(path) || SPIFFS.remove(path);
}

bool hash_file(const char* path, uint8_t digest[32],
               char* err, size_t err_len) {
  fs::File file = SPIFFS.open(path, FILE_READ);
  if (!file) {
    snprintf(err, err_len, "cannot open %s", path);
    return false;
  }

  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  if (mbedtls_sha256_starts_ret(&ctx, 0) != 0) {
    file.close();
    mbedtls_sha256_free(&ctx);
    snprintf(err, err_len, "sha256 init failed");
    return false;
  }

  uint8_t buf[1024];
  bool ok = true;
  while (file.available()) {
    int n = file.read(buf, sizeof(buf));
    if (n <= 0 || mbedtls_sha256_update_ret(&ctx, buf, n) != 0) {
      ok = false;
      break;
    }
  }
  file.close();
  if (ok && mbedtls_sha256_finish_ret(&ctx, digest) != 0) ok = false;
  mbedtls_sha256_free(&ctx);
  if (!ok) snprintf(err, err_len, "sha256 read failed");
  return ok;
}

bool write_sha_file(const char* hex) {
  if (!remove_path(kStagedShaPath)) return false;
  fs::File file = SPIFFS.open(kStagedShaPath, FILE_WRITE);
  if (!file) return false;
  bool ok = file.print(hex) == 64 && file.print('\n') == 1;
  file.close();
  return ok;
}

bool read_sha_file(char out[65]) {
  fs::File file = SPIFFS.open(kStagedShaPath, FILE_READ);
  if (!file) return false;
  size_t file_size = file.size();
  size_t n = file.readBytes(out, 64);
  int terminator = file.read();
  file.close();
  out[n] = '\0';
  uint8_t ignored[32];
  return file_size == 65 && n == 64 && terminator == '\n' &&
         parse_sha256(out, ignored);
}

bool digest_matches_file(const char* expected_hex,
                         char* err, size_t err_len) {
  uint8_t expected[32];
  if (!parse_sha256(expected_hex, expected)) {
    snprintf(err, err_len, "stored sha256 malformed");
    return false;
  }
  uint8_t actual[32];
  if (!hash_file(kStagedPath, actual, err, err_len)) return false;
  if (memcmp(expected, actual, sizeof(actual)) != 0) {
    char actual_hex[65];
    encode_sha256(actual, actual_hex);
    snprintf(err, err_len, "sha256 mismatch got %.16s...", actual_hex);
    return false;
  }
  return true;
}

void remove_uncommitted() {
  close_upload();
  s_received_valid = false;
  memset(s_received_hex, 0, sizeof(s_received_hex));
  remove_path(kStagedPath);
  remove_path(kStagedShaPath);
}

}  // namespace

bool begin() {
  // Never auto-format: MeshCore uses this same partition for persisted state.
  // A missing/corrupt/unexpected filesystem is a hard staging failure.
  s_ready = SPIFFS.begin(false);
  if (!s_ready) return false;

  // A reset during UPLOAD can leave a ZIP without its commit sidecar (or a
  // torn sidecar). Such a file is never a staged bundle and is removed before
  // commands are accepted. A valid committed pair is retained.
  if (SPIFFS.exists(kStagedPath) || SPIFFS.exists(kStagedShaPath)) {
    char expected[65];
    char ignored[64];
    if (!SPIFFS.exists(kStagedPath) || !read_sha_file(expected) ||
        !digest_matches_file(expected, ignored, sizeof(ignored))) {
      remove_path(kStagedPath);
      remove_path(kStagedShaPath);
    }
  }
  return true;
}

bool ready() { return s_ready; }

bool has_staged() {
  return s_ready && SPIFFS.exists(kStagedPath) &&
         SPIFFS.exists(kStagedShaPath);
}

uint32_t staged_size() {
  if (!has_staged()) return 0;
  fs::File file = SPIFFS.open(kStagedPath, FILE_READ);
  if (!file) return 0;
  uint32_t size = file.size();
  file.close();
  return size;
}

bool prepare_incoming(uint32_t size, const char* expected_hex,
                      char* err, size_t err_len) {
  if (!s_ready) {
    snprintf(err, err_len, "filesystem unavailable");
    return false;
  }
  uint8_t expected[32];
  if (!parse_sha256(expected_hex, expected)) {
    snprintf(err, err_len, "sha256 must be exactly 64 hex characters");
    return false;
  }
  if (size == 0 || size > kMaxBundleBytes) {
    snprintf(err, err_len, "size must be 1..%lu",
             static_cast<unsigned long>(kMaxBundleBytes));
    return false;
  }

  // Destructive staging is intentional: after a new UPLOAD starts there is no
  // old bundle which a stray command could launch. The new sidecar is written
  // only after the full frame, hash, and manifest have passed.
  remove_uncommitted();
  if (SPIFFS.exists(kStagedPath) || SPIFFS.exists(kStagedShaPath)) {
    snprintf(err, err_len, "cannot clear previous staged bundle");
    return false;
  }
  s_upload_file = SPIFFS.open(kStagedPath, FILE_WRITE);
  if (!s_upload_file) {
    snprintf(err, err_len, "cannot create upload file");
    return false;
  }

  mbedtls_sha256_init(&s_upload_sha);
  if (mbedtls_sha256_starts_ret(&s_upload_sha, 0) != 0) {
    s_upload_file.close();
    mbedtls_sha256_free(&s_upload_sha);
    remove_path(kStagedPath);
    snprintf(err, err_len, "sha256 init failed");
    return false;
  }
  memcpy(s_upload_expected, expected, sizeof(s_upload_expected));
  s_upload_size = size;
  s_upload_prepared = true;
  return true;
}

ReceiveResult receive_prepared(char* err, size_t err_len) {
  if (!s_upload_prepared || !s_upload_file) {
    snprintf(err, err_len, "upload was not prepared");
    remove_uncommitted();
    return ReceiveResult::kPartialFailure;
  }

  uint8_t chunk[kUploadChunkBytes];
  uint32_t consumed = 0;
  bool io_ok = true;
  while (io_ok && consumed < s_upload_size) {
    uint32_t chunk_size = s_upload_size - consumed;
    if (chunk_size > sizeof(chunk)) chunk_size = sizeof(chunk);
    uint32_t chunk_received = 0;
    uint32_t last_data_at = millis();
    while (chunk_received < chunk_size) {
      int available = Serial.available();
      if (available <= 0) {
        if (static_cast<uint32_t>(millis() - last_data_at) >=
            kReceiveIdleTimeoutMs) {
          snprintf(err, err_len, "upload timed out at %lu/%lu",
                   static_cast<unsigned long>(consumed + chunk_received),
                   static_cast<unsigned long>(s_upload_size));
          io_ok = false;
          break;
        }
        delay(1);
        continue;
      }
      uint32_t want = chunk_size - chunk_received;
      if (want > static_cast<uint32_t>(available)) want = available;
      int n = Serial.read(chunk + chunk_received, want);
      if (n <= 0) continue;
      chunk_received += static_cast<uint32_t>(n);
      last_data_at = millis();
    }
    if (!io_ok) break;

    // These raw bytes have now been consumed even if persistence fails.
    consumed += chunk_size;
    if (s_upload_file.write(chunk, chunk_size) != chunk_size ||
        mbedtls_sha256_update_ret(&s_upload_sha, chunk, chunk_size) != 0) {
      snprintf(err, err_len, "upload write failed at %lu",
               static_cast<unsigned long>(consumed));
      io_ok = false;
      break;
    }
    Serial.printf("ACK %lu\r\n", static_cast<unsigned long>(consumed));
    Serial.flush();
  }
  s_upload_file.close();

  uint8_t actual[32] = {};
  if (io_ok && mbedtls_sha256_finish_ret(&s_upload_sha, actual) != 0) {
    snprintf(err, err_len, "sha256 finish failed");
    io_ok = false;
  }
  mbedtls_sha256_free(&s_upload_sha);
  s_upload_prepared = false;
  bool frame_complete = consumed == s_upload_size;
  s_upload_size = 0;

  if (io_ok && memcmp(s_upload_expected, actual, sizeof(actual)) != 0) {
    snprintf(err, err_len, "upload sha256 mismatch");
    io_ok = false;
  }
  memset(s_upload_expected, 0, sizeof(s_upload_expected));
  if (!io_ok) {
    remove_uncommitted();
    return frame_complete ? ReceiveResult::kFrameCompleteInvalid
                          : ReceiveResult::kPartialFailure;
  }

  encode_sha256(actual, s_received_hex);
  s_received_valid = true;
  return ReceiveResult::kOk;
}

bool commit_received(char* err, size_t err_len) {
  if (!err || err_len == 0) return false;
  err[0] = '\0';
  if (!s_received_valid || !SPIFFS.exists(kStagedPath)) {
    snprintf(err, err_len, "no validated upload to commit");
    return false;
  }
  if (!write_sha_file(s_received_hex) ||
      !digest_matches_file(s_received_hex, err, err_len)) {
    if (err[0] == '\0') snprintf(err, err_len, "cannot commit upload hash");
    remove_uncommitted();
    return false;
  }
  s_received_valid = false;
  memset(s_received_hex, 0, sizeof(s_received_hex));
  return true;
}

void discard_received() { remove_uncommitted(); }

bool verify_staged(char* err, size_t err_len) {
  if (!has_staged()) {
    snprintf(err, err_len, "no staged bundle");
    return false;
  }
  char expected[65];
  if (!read_sha_file(expected)) {
    snprintf(err, err_len, "stored sha256 missing or malformed");
    return false;
  }
  return digest_matches_file(expected, err, err_len);
}

bool erase_staged() {
  if (!s_ready) return false;
  close_upload();
  s_received_valid = false;
  memset(s_received_hex, 0, sizeof(s_received_hex));
  bool zip_ok = remove_path(kStagedPath);
  bool sha_ok = remove_path(kStagedShaPath);
  return zip_ok && sha_ok;
}

}  // namespace v4_storage
