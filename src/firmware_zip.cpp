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

  // The legacy bootloader can't take SD+BL+App in one go (the Java reference
  // explicitly splits them across two connections), and we don't have a
  // sample yet to verify that path. So we accept the simple shapes here:
  // application / bootloader / softdevice / softdevice_bootloader.
  if (m["softdevice_bootloader"].is<JsonObject>()) {
    JsonObject n = m["softdevice_bootloader"];
    if (!resolve(n, &out->bin, &out->dat, err, err_len)) goto fail;
    out->type = TYPE_SOFTDEVICE | TYPE_BOOTLOADER;
    // Different nrfutil versions place sizes either at the entry's top level
    // or under info_read_only_metadata. Accept both.
    out->sd_size = n["sd_size"] | n["info_read_only_metadata"]["sd_size"] | 0u;
    out->bl_size = n["bl_size"] | n["info_read_only_metadata"]["bl_size"] | 0u;
    if (out->sd_size == 0 || out->bl_size == 0) {
      snprintf(err, err_len, "softdevice_bootloader missing sd_size / bl_size");
      goto fail;
    }
  } else if (m["application"].is<JsonObject>()) {
    if (!resolve(m["application"], &out->bin, &out->dat, err, err_len)) goto fail;
    out->type = TYPE_APPLICATION;
  } else if (m["bootloader"].is<JsonObject>()) {
    if (!resolve(m["bootloader"], &out->bin, &out->dat, err, err_len)) goto fail;
    out->type = TYPE_BOOTLOADER;
    out->bl_size = out->bin.size;
  } else if (m["softdevice"].is<JsonObject>()) {
    if (!resolve(m["softdevice"], &out->bin, &out->dat, err, err_len)) goto fail;
    out->type = TYPE_SOFTDEVICE;
    out->sd_size = out->bin.size;
  } else {
    snprintf(err, err_len, "manifest.json: no recognised firmware section");
    goto fail;
  }

  return true;

fail:
  zip_reader::close();
  return false;
}

}  // namespace firmware_zip
