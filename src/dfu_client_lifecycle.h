#pragma once

namespace dfu_client_lifecycle {

// Bluefruit client begin() registers an object in an append-only dispatch
// list.  A DFU attempt has two run() legs (application/buttonless, then
// bootloader), so registration must be process-wide rather than per leg.
class RegistrationState {
 public:
  RegistrationState() : initialized_(false) {}

  bool initialized() const { return initialized_; }
  void mark_initialized() { initialized_ = true; }

 private:
  bool initialized_;
};

}  // namespace dfu_client_lifecycle
