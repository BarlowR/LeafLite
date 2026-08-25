#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#include "config.h"
#else
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#endif

// LC86G GPS on UART0 (GPIO 43/44). Hand-rolled RMC/GGA parser with checksum
// validation — replaces TinyGPS++. Parses only what IGC + phone apps need.
struct GpsFix {
  bool valid = false;      // RMC status == 'A'
  double latDeg = 0;       // signed decimal degrees
  double lonDeg = 0;
  float altM = 0;          // GGA MSL altitude
  float speedKmh = 0;
  uint8_t sats = 0;
  // UTC from RMC (date) + latest time
  uint8_t hh = 0, mm = 0, ss = 0;
  uint8_t day = 0, mon = 0;
  uint16_t year = 0;
};

class Gps {
 public:
#ifdef ARDUINO
  void init() {
    pinMode(PIN_GPS_BACKUP_EN, OUTPUT);
    digitalWrite(PIN_GPS_BACKUP_EN, HIGH);  // retain ephemeris for hot starts
    pinMode(PIN_GPS_RESET, OUTPUT);
    digitalWrite(PIN_GPS_RESET, HIGH);
    Serial0.begin(GPS_BAUD);
  }
#endif

  // Drain UART; returns true if a valid sentence completed this call.
  // Complete raw sentences are also handed to `sink` (for BLE passthrough).
#ifdef ARDUINO
  template <typename F>
  bool poll(F&& sink) {
    bool updated = false;
    while (Serial0.available()) updated |= feed(Serial0.read(), sink);
    return updated;
  }
#endif

  // Feed one byte; testable on host. Returns true when a valid sentence parsed.
  template <typename F>
  bool feed(char c, F&& sink) {
    bool updated = false;
    if (c == '$') { len_ = 0; }
    if (len_ < sizeof(buf_) - 1) buf_[len_++] = c;
    if (c == '\n') {
      buf_[len_] = 0;
      if (checksumOk(buf_)) {
        sink(buf_);
        updated = parse(buf_);
      }
      len_ = 0;
    }
    return updated;
  }

  const GpsFix& fix() const { return fix_; }

 private:
  static bool checksumOk(const char* s) {
    if (s[0] != '$') return false;
    uint8_t cs = 0;
    const char* p = s + 1;
    while (*p && *p != '*') cs ^= *p++;
    if (*p != '*') return false;
    return strtol(p + 1, nullptr, 16) == cs;
  }

  // "ddmm.mmmm" -> signed decimal degrees
  static double toDeg(const char* f, char hemi, int degDigits) {
    if (!*f) return 0;
    double v = atof(f);
    int deg = (int)(v / 100);
    double d = deg + (v - deg * 100) / 60.0;
    (void)degDigits;
    return (hemi == 'S' || hemi == 'W') ? -d : d;
  }

  bool parse(char* s) {
    // Tokenize in place; empty fields yield empty strings.
    char* f[20] = {};
    int n = 0;
    for (char* p = s; *p && n < 20; p++) {
      if (p == s || *(p - 1) == 0) f[n++] = p;
      if (*p == ',' || *p == '*') *p = 0;
    }
    if (n < 1) return false;
    const char* id = f[0];  // e.g. "$GNRMC"
    if (strlen(id) < 6) return false;

    if (strcmp(id + 3, "RMC") == 0 && n >= 10) {
      if (strlen(f[1]) >= 6) {
        fix_.hh = (f[1][0] - '0') * 10 + f[1][1] - '0';
        fix_.mm = (f[1][2] - '0') * 10 + f[1][3] - '0';
        fix_.ss = (f[1][4] - '0') * 10 + f[1][5] - '0';
      }
      fix_.valid = (f[2][0] == 'A');
      if (fix_.valid) {
        fix_.latDeg = toDeg(f[3], f[4][0], 2);
        fix_.lonDeg = toDeg(f[5], f[6][0], 3);
        fix_.speedKmh = atof(f[7]) * 1.852f;
      }
      if (strlen(f[9]) >= 6) {
        fix_.day = (f[9][0] - '0') * 10 + f[9][1] - '0';
        fix_.mon = (f[9][2] - '0') * 10 + f[9][3] - '0';
        fix_.year = 2000 + (f[9][4] - '0') * 10 + f[9][5] - '0';
      }
      return true;
    }
    if (strcmp(id + 3, "GGA") == 0 && n >= 10) {
      fix_.sats = atoi(f[7]);
      fix_.altM = atof(f[9]);
      return true;
    }
    return false;
  }

  GpsFix fix_;
  char buf_[128];
  size_t len_ = 0;
};
