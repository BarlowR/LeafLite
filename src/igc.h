#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>

#include "config.h"
#include "gps.h"

// IGC logger: header + 1Hz B-records to SD (SD_MMC 1-bit mode — no external lib).
//
// HONESTY NOTE: this produces IGC files that XCTrack/SeeYou/ayvri open fine and
// that XContest generally accepts for the fun leagues, but there is NO G-record
// (cryptographic signature), so files are NOT valid for FAI Cat 1 or anywhere a
// signed tracklog is required. Upstream Leaf uses scottyob/IgcLogger for that.
class IgcLogger {
 public:
  bool mountSd() {
    pinMode(PIN_SD_DETECT, INPUT_PULLUP);
    if (!SD_MMC.setPins(PIN_SDIO_CLK, PIN_SDIO_CMD, PIN_SDIO_D0)) return false;
    return SD_MMC.begin("/sd", /*mode1bit=*/true);
  }

  bool start(const GpsFix& fix) {
    if (active_) return true;
    if (!fix.valid || fix.year < 2020) return false;  // need date for header/filename

    char path[48];
    snprintf(path, sizeof(path), "/%04u-%02u-%02u-LFL-%02u%02u%02u.igc", fix.year,
             fix.mon, fix.day, fix.hh, fix.mm, fix.ss);
    file_ = SD_MMC.open(path, FILE_WRITE);
    if (!file_) return false;

    file_.printf("AXLF001 leaflite\r\n");  // "X" prefix: unregistered manufacturer code
    file_.printf("HFDTEDATE:%02u%02u%02u\r\n", fix.day, fix.mon, fix.year % 100);
    file_.printf("HFPLTPILOTINCHARGE:NOT_SET\r\n");
    file_.printf("HFGTYGLIDERTYPE:NOT_SET\r\n");
    file_.printf("HFFTYFRTYPE:leaflite\r\n");
    file_.printf("HFDTM100GPSDATUM:WGS-1984\r\n");
    active_ = true;
    lastSec_ = 255;
    return true;
  }

  // Call every loop; writes one B-record per GPS second.
  void update(const GpsFix& fix, float baroAltM) {
    if (!active_ || !fix.valid || fix.ss == lastSec_) return;
    lastSec_ = fix.ss;

    // Lat: DDMMmmm[N/S], Lon: DDDMMmmm[E/W]  (mmm = thousandths of minutes)
    double alat = fabs(fix.latDeg), alon = fabs(fix.lonDeg);
    int latD = (int)alat;
    int lonD = (int)alon;
    long latMm = lroundl((alat - latD) * 60000.0);
    long lonMm = lroundl((alon - lonD) * 60000.0);

    file_.printf("B%02u%02u%02u%02d%05ld%c%03d%05ld%cA%05ld%05ld\r\n", fix.hh, fix.mm,
                 fix.ss, latD, latMm, fix.latDeg >= 0 ? 'N' : 'S', lonD, lonMm,
                 fix.lonDeg >= 0 ? 'E' : 'W', lroundf(baroAltM), lroundf(fix.altM));

    if (++sinceFlush_ >= 10) {  // flush every 10s: bounded loss if battery dies mid-flight
      file_.flush();
      sinceFlush_ = 0;
    }
  }

  void stop() {
    if (!active_) return;
    file_.flush();
    file_.close();
    active_ = false;
  }

  bool active() const { return active_; }

 private:
  File file_;
  bool active_ = false;
  uint8_t lastSec_ = 255;
  uint8_t sinceFlush_ = 0;
};
