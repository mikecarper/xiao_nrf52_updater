#include "dfu_image_layout.h"

namespace dfu_image_layout {

namespace {

bool has_multiple_bits(uint8_t value) {
  return value != 0 && (value & static_cast<uint8_t>(value - 1u)) != 0;
}

void put_u32le(uint8_t* out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
  out[2] = static_cast<uint8_t>(value >> 16);
  out[3] = static_cast<uint8_t>(value >> 24);
}

bool valid_tuple_shape(const Layout& layout) {
  switch (layout.type) {
    case kTypeApplication:
      return layout.sd_size == 0 && layout.bl_size == 0 &&
             layout.app_size != 0;
    case kTypeBootloader:
      return layout.sd_size == 0 && layout.bl_size != 0 &&
             layout.app_size == 0;
    case kTypeSoftdevice:
      return layout.sd_size != 0 && layout.bl_size == 0 &&
             layout.app_size == 0;
    case kTypeSoftdevice | kTypeBootloader:
      return layout.sd_size != 0 && layout.bl_size != 0 &&
             layout.app_size == 0;
    default:
      return false;
  }
}

}  // namespace

Status select_section(uint8_t section_mask, ManifestSection* selected) {
  if (!selected || (section_mask & static_cast<uint8_t>(~kKnownSectionMask))) {
    return Status::kInvalidArgument;
  }
  *selected = static_cast<ManifestSection>(0);

  // Secure combined-bin sections cannot be interpreted as Legacy DFU images,
  // even if the manifest also happens to contain a supported legacy section.
  if (section_mask & kSecureSectionMask) return Status::kSecureDfuRequired;

  uint8_t legacy = section_mask & kLegacySectionMask;
  if (legacy == 0) return Status::kNoFirmwareSection;
  if (has_multiple_bits(legacy)) return Status::kMultipleLegacySections;

  *selected = static_cast<ManifestSection>(legacy);
  return Status::kOk;
}

Status make_layout(ManifestSection section, uint32_t bin_size,
                   uint32_t combined_sd_size, uint32_t combined_bl_size,
                   Layout* out) {
  if (!out) return Status::kInvalidArgument;
  *out = {};
  if (bin_size == 0) return Status::kEmptyImage;

  switch (section) {
    case kApplication:
      out->type = kTypeApplication;
      out->app_size = bin_size;
      break;
    case kBootloader:
      out->type = kTypeBootloader;
      out->bl_size = bin_size;
      break;
    case kSoftdevice:
      out->type = kTypeSoftdevice;
      out->sd_size = bin_size;
      break;
    case kSoftdeviceBootloader:
      if (combined_sd_size == 0 || combined_bl_size == 0) {
        return Status::kCombinedSizesMissing;
      }
      // Subtraction avoids overflow in sd_size + bl_size.
      if (combined_sd_size > bin_size ||
          combined_bl_size != bin_size - combined_sd_size) {
        return Status::kCombinedSizeMismatch;
      }
      out->type = kTypeSoftdevice | kTypeBootloader;
      out->sd_size = combined_sd_size;
      out->bl_size = combined_bl_size;
      break;
    default:
      return Status::kInvalidArgument;
  }
  return Status::kOk;
}

bool encode_size_tuple(const Layout& layout, uint8_t* out, size_t out_len) {
  if (!out || out_len < 12 || !valid_tuple_shape(layout)) return false;
  put_u32le(out + 0, layout.sd_size);
  put_u32le(out + 4, layout.bl_size);
  put_u32le(out + 8, layout.app_size);
  return true;
}

const char* status_message(Status status) {
  switch (status) {
    case Status::kOk:
      return "ok";
    case Status::kInvalidArgument:
      return "invalid firmware section";
    case Status::kNoFirmwareSection:
      return "no recognised firmware section";
    case Status::kMultipleLegacySections:
      return "multiple Legacy DFU sections require a multi-session update";
    case Status::kSecureDfuRequired:
      return "combined application section requires Secure DFU";
    case Status::kEmptyImage:
      return "firmware image is empty";
    case Status::kCombinedSizesMissing:
      return "softdevice_bootloader missing sd_size / bl_size";
    case Status::kCombinedSizeMismatch:
      return "softdevice_bootloader sizes do not match bin size";
  }
  return "unknown image layout error";
}

}  // namespace dfu_image_layout
