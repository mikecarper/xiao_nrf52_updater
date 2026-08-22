#include "v4_bundle.h"

#include <ArduinoJson.h>

#include "dfu_image_layout.h"

namespace v4_bundle {

namespace {

constexpr uint32_t kMaxInitPacketBytes = 128;

bool load_manifest(JsonDocument& doc, char* err, size_t err_len) {
  v4_zip::Entry manifest;
  if (!v4_zip::find("manifest.json", &manifest)) {
    snprintf(err, err_len, "manifest.json missing");
    return false;
  }
  if (manifest.size == 0 || manifest.size > 2048) {
    snprintf(err, err_len, "manifest size %lu out of range",
             static_cast<unsigned long>(manifest.size));
    return false;
  }
  if (!v4_zip::verify_crc(manifest)) {
    snprintf(err, err_len, "manifest CRC mismatch");
    return false;
  }
  char bytes[2048];
  int n = v4_zip::read(manifest, 0, bytes, manifest.size);
  if (n != static_cast<int>(manifest.size)) {
    snprintf(err, err_len, "manifest read truncated");
    return false;
  }
  DeserializationError json_error = deserializeJson(doc, bytes, n);
  if (json_error) {
    snprintf(err, err_len, "manifest parse: %s", json_error.c_str());
    return false;
  }
  return true;
}

bool resolve(JsonObject node, v4_zip::Entry* bin, v4_zip::Entry* dat,
             char* err, size_t err_len) {
  const char* bin_name = node["bin_file"] | static_cast<const char*>(nullptr);
  const char* dat_name = node["dat_file"] | static_cast<const char*>(nullptr);
  if (!bin_name || !dat_name) {
    snprintf(err, err_len, "manifest entry missing bin_file/dat_file");
    return false;
  }
  if (!v4_zip::find(bin_name, bin)) {
    snprintf(err, err_len, "%.48s missing", bin_name);
    return false;
  }
  if (!v4_zip::find(dat_name, dat)) {
    snprintf(err, err_len, "%.48s missing", dat_name);
    return false;
  }
  if (!v4_zip::verify_crc(*bin) || !v4_zip::verify_crc(*dat)) {
    snprintf(err, err_len, "firmware/init CRC mismatch");
    return false;
  }
  return true;
}

}  // namespace

bool parse(const char* path, Parsed* out, char* err, size_t err_len) {
  if (!out || !err || err_len == 0) return false;
  *out = {};
  err[0] = '\0';
  if (!v4_zip::open(path)) {
    snprintf(err, err_len, "cannot open ZIP");
    return false;
  }

  JsonDocument doc;
  if (!load_manifest(doc, err, err_len)) {
    v4_zip::close();
    return false;
  }
  JsonObject manifest = doc["manifest"].as<JsonObject>();
  if (manifest.isNull()) {
    snprintf(err, err_len, "top-level manifest missing");
    v4_zip::close();
    return false;
  }

  uint8_t sections = 0;
  if (manifest["application"].is<JsonObject>())
    sections |= dfu_image_layout::kApplication;
  if (manifest["bootloader"].is<JsonObject>())
    sections |= dfu_image_layout::kBootloader;
  if (manifest["softdevice"].is<JsonObject>())
    sections |= dfu_image_layout::kSoftdevice;
  if (manifest["softdevice_bootloader"].is<JsonObject>())
    sections |= dfu_image_layout::kSoftdeviceBootloader;
  if (manifest["bootloader_application"].is<JsonObject>())
    sections |= dfu_image_layout::kBootloaderApplication;
  if (manifest["softdevice_application"].is<JsonObject>())
    sections |= dfu_image_layout::kSoftdeviceApplication;
  if (manifest["softdevice_bootloader_application"].is<JsonObject>())
    sections |= dfu_image_layout::kSoftdeviceBootloaderApplication;

  dfu_image_layout::ManifestSection selected;
  dfu_image_layout::Status status =
      dfu_image_layout::select_section(sections, &selected);
  if (status != dfu_image_layout::Status::kOk) {
    snprintf(err, err_len, "%s", dfu_image_layout::status_message(status));
    v4_zip::close();
    return false;
  }

  const char* section_name = nullptr;
  switch (selected) {
    case dfu_image_layout::kApplication:
      section_name = "application";
      break;
    case dfu_image_layout::kBootloader:
      section_name = "bootloader";
      break;
    case dfu_image_layout::kSoftdevice:
      section_name = "softdevice";
      break;
    case dfu_image_layout::kSoftdeviceBootloader:
      section_name = "softdevice_bootloader";
      break;
    default:
      snprintf(err, err_len, "invalid selected section");
      v4_zip::close();
      return false;
  }

  JsonObject node = manifest[section_name].as<JsonObject>();
  if (!resolve(node, &out->bin, &out->dat, err, err_len)) {
    v4_zip::close();
    return false;
  }
  if (out->dat.size == 0 || out->dat.size > kMaxInitPacketBytes) {
    snprintf(err, err_len, "init packet size %lu must be 1..%lu",
             static_cast<unsigned long>(out->dat.size),
             static_cast<unsigned long>(kMaxInitPacketBytes));
    v4_zip::close();
    return false;
  }

  uint32_t sd_size = 0;
  uint32_t bl_size = 0;
  if (selected == dfu_image_layout::kSoftdeviceBootloader) {
    sd_size = node["sd_size"] | 0u;
    bl_size = node["bl_size"] | 0u;
    if (sd_size == 0)
      sd_size = node["info_read_only_metadata"]["sd_size"] | 0u;
    if (bl_size == 0)
      bl_size = node["info_read_only_metadata"]["bl_size"] | 0u;
  }

  dfu_image_layout::Layout layout;
  status = dfu_image_layout::make_layout(selected, out->bin.size,
                                         sd_size, bl_size, &layout);
  if (status != dfu_image_layout::Status::kOk) {
    snprintf(err, err_len, "%s", dfu_image_layout::status_message(status));
    v4_zip::close();
    return false;
  }
  out->type = layout.type;
  out->sd_size = layout.sd_size;
  out->bl_size = layout.bl_size;
  out->app_size = layout.app_size;
  return true;
}

}  // namespace v4_bundle
