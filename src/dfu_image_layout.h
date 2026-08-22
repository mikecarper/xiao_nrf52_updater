#pragma once

#include <stddef.h>
#include <stdint.h>

namespace dfu_image_layout {

// Manifest firmware sections. These bits describe which JSON objects were
// present; they are deliberately distinct from the Legacy DFU type byte.
enum ManifestSection : uint8_t {
  kApplication = 1u << 0,
  kBootloader = 1u << 1,
  kSoftdevice = 1u << 2,
  kSoftdeviceBootloader = 1u << 3,

  // These combined-bin sections are Secure-DFU package shapes. A Legacy DFU
  // client must reject them rather than mislabeling the bin as an application.
  kBootloaderApplication = 1u << 4,
  kSoftdeviceApplication = 1u << 5,
  kSoftdeviceBootloaderApplication = 1u << 6,
};

static constexpr uint8_t kLegacySectionMask =
    kApplication | kBootloader | kSoftdevice | kSoftdeviceBootloader;
static constexpr uint8_t kSecureSectionMask =
    kBootloaderApplication | kSoftdeviceApplication |
    kSoftdeviceBootloaderApplication;
static constexpr uint8_t kKnownSectionMask =
    kLegacySectionMask | kSecureSectionMask;

// Nordic Legacy DFU START_DFU type bits.
static constexpr uint8_t kTypeSoftdevice = 0x01;
static constexpr uint8_t kTypeBootloader = 0x02;
static constexpr uint8_t kTypeApplication = 0x04;

struct Layout {
  uint8_t type;
  uint32_t sd_size;
  uint32_t bl_size;
  uint32_t app_size;
};

enum class Status : uint8_t {
  kOk,
  kInvalidArgument,
  kNoFirmwareSection,
  kMultipleLegacySections,
  kSecureDfuRequired,
  kEmptyImage,
  kCombinedSizesMissing,
  kCombinedSizeMismatch,
};

// Select exactly one supported Legacy DFU manifest section. Multi-entry
// Legacy packages need a two-session flow and are intentionally rejected.
Status select_section(uint8_t section_mask, ManifestSection* selected);

// Convert a selected manifest entry into the type byte and the three image
// sizes required by Legacy DFU. For SD+BL, the two metadata sizes must be
// non-zero and add up exactly to the combined bin size.
Status make_layout(ManifestSection section, uint32_t bin_size,
                   uint32_t combined_sd_size, uint32_t combined_bl_size,
                   Layout* out);

// Encode [SoftDevice, Bootloader, Application] as three uint32 little-endian
// values, exactly as sent on the Legacy DFU Packet characteristic.
bool encode_size_tuple(const Layout& layout, uint8_t* out, size_t out_len);

const char* status_message(Status status);

}  // namespace dfu_image_layout
