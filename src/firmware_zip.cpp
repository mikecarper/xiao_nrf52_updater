#include "firmware_zip.h"

#include <ArduinoJson.h>

#include "storage.h"

namespace firmware_zip {

namespace {

// Pull "manifest.json" out of the open archive into a small RAM buffer and
// hand it to ArduinoJson. The file is tiny (<1 KB in practice for nrfutil
// outputs), so we don't bother streaming.
bool load_manifest(JsonDocument& doc, char* err, size_t err_len) {
  zip_reader::Entry manifest;
  if (!zip_reader::find("manifest.json", &manifest)) {
    snprintf(err, err_len, "manifest.json missing from zip");
    return false;
  }
  if (manifest.size == 0 || manifest.size > 2048) {
    snprintf(err, err_len, "manifest.json size %lu out of range", (unsigned long)manifest.size);
    return false;
  }

  char buf[2048];
  int n = zip_reader::read(manifest, 0, buf, manifest.size);
  if (n != (int)manifest.size) {
    snprintf(err, err_len, "manifest.json read truncated");
    return false;
  }

  DeserializationError jerr = deserializeJson(doc, buf, n);
  if (jerr) {
    snprintf(err, err_len, "manifest.json parse: %s", jerr.c_str());
    return false;
  }
  return true;
}

bool resolve(JsonObject node, zip_reader::Entry* bin, zip_reader::Entry* dat,
             char* err, size_t err_len) {
  const char* bin_name = node["bin_file"] | (const char*)nullptr;
  const char* dat_name = node["dat_file"] | (const char*)nullptr;
  if (!bin_name) {
    snprintf(err, err_len, "manifest entry missing bin_file");
    return false;
  }
  if (!zip_reader::find(bin_name, bin)) {
    snprintf(err, err_len, "%s not in zip", bin_name);
    return false;
  }
  // Init packet (.dat) is optional in very old DFU bootloaders, but every
  // modern nrfutil bundle includes one. We require it.
  if (!dat_name) {
    snprintf(err, err_len, "manifest entry missing dat_file");
    return false;
  }
  if (!zip_reader::find(dat_name, dat)) {
    snprintf(err, err_len, "%s not in zip", dat_name);
    return false;
  }
  return true;
}

}  // namespace

bool parse(const char* zip_path, Parsed* out, char* err, size_t err_len) {
  *out = {};
  err[0] = '\0';

  if (!zip_reader::open(zip_path)) {
    snprintf(err, err_len, "cannot open %s", zip_path);
    return false;
  }

  JsonDocument doc;
  if (!load_manifest(doc, err, err_len)) {
    zip_reader::close();
    return false;
  }

  JsonObject m = doc["manifest"].as<JsonObject>();
  if (m.isNull()) {
    snprintf(err, err_len, "manifest.json: top-level `manifest` missing");
    zip_reader::close();
    return false;
  }

  // Inventory every known image section before choosing one. In particular,
  // never silently select the first entry of a multi-image package and then
  // delete the ZIP with another image still unapplied.
  uint8_t sections = 0;
  if (m["application"].is<JsonObject>()) {
    sections |= dfu_image_layout::kApplication;
  }
  if (m["bootloader"].is<JsonObject>()) {
    sections |= dfu_image_layout::kBootloader;
  }
  if (m["softdevice"].is<JsonObject>()) {
    sections |= dfu_image_layout::kSoftdevice;
  }
  if (m["softdevice_bootloader"].is<JsonObject>()) {
    sections |= dfu_image_layout::kSoftdeviceBootloader;
  }
  if (m["bootloader_application"].is<JsonObject>()) {
    sections |= dfu_image_layout::kBootloaderApplication;
  }
  if (m["softdevice_application"].is<JsonObject>()) {
    sections |= dfu_image_layout::kSoftdeviceApplication;
  }
  if (m["softdevice_bootloader_application"].is<JsonObject>()) {
    sections |= dfu_image_layout::kSoftdeviceBootloaderApplication;
  }

  dfu_image_layout::ManifestSection selected;
  dfu_image_layout::Status layout_status =
      dfu_image_layout::select_section(sections, &selected);
  if (layout_status != dfu_image_layout::Status::kOk) {
    snprintf(err, err_len, "manifest.json: %s",
             dfu_image_layout::status_message(layout_status));
    zip_reader::close();
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
      snprintf(err, err_len, "manifest.json: invalid firmware section");
      zip_reader::close();
      return false;
  }

  JsonObject node = m[section_name].as<JsonObject>();
  if (!resolve(node, &out->bin, &out->dat, err, err_len)) {
    zip_reader::close();
    return false;
  }

  uint32_t combined_sd_size = 0;
  uint32_t combined_bl_size = 0;
  if (selected == dfu_image_layout::kSoftdeviceBootloader) {
    // Different nrfutil versions place sizes either at the entry's top level
    // or under info_read_only_metadata. Accept both.
    combined_sd_size = node["sd_size"] | 0u;
    combined_bl_size = node["bl_size"] | 0u;
    if (combined_sd_size == 0) {
      combined_sd_size =
          node["info_read_only_metadata"]["sd_size"] | 0u;
    }
    if (combined_bl_size == 0) {
      combined_bl_size =
          node["info_read_only_metadata"]["bl_size"] | 0u;
    }
  }

  dfu_image_layout::Layout layout;
  layout_status = dfu_image_layout::make_layout(
      selected, out->bin.size, combined_sd_size, combined_bl_size, &layout);
  if (layout_status != dfu_image_layout::Status::kOk) {
    snprintf(err, err_len, "manifest.json: %s",
             dfu_image_layout::status_message(layout_status));
    zip_reader::close();
    return false;
  }
  out->type = layout.type;
  out->sd_size = layout.sd_size;
  out->bl_size = layout.bl_size;
  out->app_size = layout.app_size;

  return true;
}

}  // namespace firmware_zip
