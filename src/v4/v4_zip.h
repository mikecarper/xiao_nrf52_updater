#pragma once

#include <Arduino.h>

namespace v4_zip {

struct Entry {
  char name[96];
  uint32_t data_offset;
  uint32_t size;
  uint32_t crc32;
};

bool open(const char* path);
void close();
bool next(Entry* out);
// Finds exactly one matching local-header member. Duplicate names are rejected
// so central-directory and local-header selection cannot disagree.
bool find(const char* name, Entry* out);
int read(const Entry& entry, uint32_t offset, void* buf, uint32_t len);
bool verify_crc(const Entry& entry);

}  // namespace v4_zip
