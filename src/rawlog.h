#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <stdarg.h>

#include "gps.h"

// Raw sensor logger for offline vario tuning. Tagged CSV rows, one file per
// session, replayable through the exact firmware pipeline with
// test/rawlog_replay.cpp:
//   B,<t_ms>,<pressure_Pa>,<temp_C>                      ~20Hz (every baro sample)
//   I,<t_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>               ~100Hz (g / rad/s, raw body frame)
//   G,<t_ms>,<lat>,<lon>,<alt_m>,<speed_kmh>,<sats>      1Hz (ground truth cross-check)
// ~6KB/s ≈ 21MB/h — trivial for any SD card. Rows are RAM-buffered and written
// in ~2KB chunks so the loop never blocks on FAT bookkeeping per sample;
// flushed every 10s to bound loss if power dies mid-flight.
class RawLogger {
 public:
  // Starts a new log in /raw. Named by the GPS timestamp (UTC) at start when
  // there is a fix, else a sequential nofix-NNN.csv so bench sessions work.
  bool start(const GpsFix& fix) {
    if (active_) return true;
    char path[48];
    if (fix.valid && fix.year >= 2020) {
      snprintf(path, sizeof(path), "/raw/%04u-%02u-%02u-%02u%02u%02u.csv",
               fix.year, fix.mon, fix.day, fix.hh, fix.mm, fix.ss);
    } else {
      for (int i = 0; i < 1000; i++) {
        snprintf(path, sizeof(path), "/raw/nofix-%03d.csv", i);
        if (!SD_MMC.exists(path)) break;
      }
    }
    f_ = SD_MMC.open(path, FILE_WRITE);
    if (!f_) return false;
    n_ = snprintf(buf_, sizeof(buf_),
                  "# leaflite rawlog v1\n"
                  "# B,t_ms,pressure_Pa,temp_C\n"
                  "# I,t_ms,ax_g,ay_g,az_g,gx_rads,gy_rads,gz_rads\n"
                  "# G,t_ms,lat_deg,lon_deg,alt_m,speed_kmh,sats\n");
    lastFlushMs_ = millis();
    lastGpsSec_ = 255;
    active_ = true;
    return true;
  }

  void logBaro(uint32_t tMs, float pa, float tC) {
    if (active_) line("B,%lu,%.0f,%.2f\n", (unsigned long)tMs, pa, tC);
  }

  void logImu(uint32_t tMs, float ax, float ay, float az, float gx, float gy, float gz) {
    if (active_)
      line("I,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", (unsigned long)tMs, ax, ay, az, gx, gy, gz);
  }

  void logGps(uint32_t tMs, const GpsFix& fix) {
    if (!active_ || !fix.valid || fix.ss == lastGpsSec_) return;
    lastGpsSec_ = fix.ss;
    line("G,%lu,%.6f,%.6f,%.1f,%.1f,%u\n", (unsigned long)tMs, fix.latDeg, fix.lonDeg,
         fix.altM, fix.speedKmh, fix.sats);
  }

  void stop() {
    if (!active_) return;
    if (n_) f_.write((const uint8_t*)buf_, n_);
    n_ = 0;
    f_.flush();
    f_.close();
    active_ = false;
  }

  bool active() const { return active_; }

 private:
  void line(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf_ + n_, sizeof(buf_) - n_, fmt, ap);
    va_end(ap);
    if (w < 0) return;
    if ((size_t)w >= sizeof(buf_) - n_) {  // didn't fit: drain buffer, retry once
      f_.write((const uint8_t*)buf_, n_);
      n_ = 0;
      va_start(ap, fmt);
      w = vsnprintf(buf_, sizeof(buf_), fmt, ap);
      va_end(ap);
      if (w < 0 || (size_t)w >= sizeof(buf_)) return;
    }
    n_ += (size_t)w;
    if (n_ >= sizeof(buf_) - 96) {
      f_.write((const uint8_t*)buf_, n_);
      n_ = 0;
    }
    if (millis() - lastFlushMs_ > 10000) {
      f_.flush();
      lastFlushMs_ = millis();
    }
  }

  File f_;
  char buf_[2048];
  size_t n_ = 0;
  unsigned long lastFlushMs_ = 0;
  uint8_t lastGpsSec_ = 255;
  bool active_ = false;
};
