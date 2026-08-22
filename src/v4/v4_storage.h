#pragma once

#include <Arduino.h>

namespace v4_storage {

static constexpr const char* kStagedPath = "/firmware.zip";

enum class ReceiveResult : uint8_t {
  kOk,
  // The entire declared raw frame was consumed, so command framing remains
  // synchronized even though hashing or persistence failed.
  kFrameCompleteInvalid,
  // Some declared bytes were not consumed. The caller must quarantine the
  // command parser until reboot.
  kPartialFailure,
};

bool begin();
bool ready();
bool has_staged();
uint32_t staged_size();

// Destructively clear any old bundle, open the upload file, and initialize the
// hash transaction. The caller must
// emit READY only after this succeeds, then call receive_prepared().
bool prepare_incoming(uint32_t size, const char* expected_hex,
                      char* err, size_t err_len);
ReceiveResult receive_prepared(char* err, size_t err_len);
// Called only after the uploaded ZIP manifest/layout has also been validated.
// This writes the checksum sidecar which makes has_staged() true.
bool commit_received(char* err, size_t err_len);
void discard_received();
bool verify_staged(char* err, size_t err_len);
bool erase_staged();

}  // namespace v4_storage
