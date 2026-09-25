// BLE address text <-> bytes, in NimBLE's byte order. Pure and framework-free
// (<stdint.h>/<stdbool.h> + <stdio.h>), so both src/bms_ble.cpp (the BMS peer) and
// src/deadman.cpp (the beacon tag) use it and the native tests can cover the parser's
// edge cases without the NimBLE headers.
//
// Printed order is the human one, "aa:bb:cc:dd:ee:ff", most significant octet first.
// NimBLE stores the reverse: val[0] is the LAST printed octet. Every conversion here
// keeps that relationship, which is the bug this header exists to stop repeating.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// "C8:47:8C:12:34:56" (':' or '-' separators, either case) -> val[0] = last printed octet.
// Returns false and leaves `out` zeroed on any deviation: wrong length, bad separator,
// non-hex digit, or a null pointer. An empty string is not an address.
inline bool ble_addr_parse(const char *str, uint8_t out[6]) {
  memset(out, 0, 6);
  if (str == nullptr || strlen(str) != 17) return false;
  for (int i = 0; i < 6; ++i) {
    const char *p = str + 3 * i;
    unsigned v = 0;
    for (int k = 0; k < 2; ++k) {
      const char c = p[k];
      unsigned d;
      if (c >= '0' && c <= '9') d = (unsigned)(c - '0');
      else if (c >= 'a' && c <= 'f') d = (unsigned)(c - 'a' + 10);
      else if (c >= 'A' && c <= 'F') d = (unsigned)(c - 'A' + 10);
      else { memset(out, 0, 6); return false; }
      v = v * 16u + d;
    }
    if (i < 5 && p[2] != ':' && p[2] != '-') { memset(out, 0, 6); return false; }
    out[5 - i] = (uint8_t)v;
  }
  return true;
}

// val[0] = last printed octet -> "aa:bb:cc:dd:ee:ff" (lower case, always 17 chars + NUL).
inline void ble_addr_str(const uint8_t val[6], char out[18]) {
  snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x", val[5], val[4], val[3], val[2], val[1], val[0]);
}

inline bool ble_addr_equal(const uint8_t a[6], const uint8_t b[6]) { return memcmp(a, b, 6) == 0; }
