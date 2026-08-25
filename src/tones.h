#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

// XCTracer-style tone profile. The response is a table of up to 12 points
//   tone=<climb m/s>,<frequency Hz>,<cycle ms>,<duty %>
// interpolated linearly between points (clamped at the ends), exactly like an
// XC Tracer's XCTRACER.TXT. duty is the audible fraction of each cycle;
// duty=100 is a continuous tone. Activation has separate on/off thresholds
// (hysteresis) for climb and sink, also in XCTracer's config vocabulary:
//   ClimbToneOnThreshold / ClimbToneOffThreshold
//   SinkToneOnThreshold  / SinkToneOffThreshold
// Between the climb and sink thresholds the vario is silent.
//
// Defaults are the published XCTracer profile from
// cloudbasemayhem.com/xctracer-tone-settings (12 points, -10..+10 m/s).
// Override at boot with /tones.txt on the SD card; unknown keys (setVolume,
// dampingFactor, beepOnlyWhenFlying, ...) are ignored — volume is hardware
// buttons here and damping is the Kalman's job.
class ToneProfile {
 public:
  struct Entry {
    float climb;
    uint16_t freq;
    uint16_t cycle;
    uint8_t duty;
  };
  static const int MAX_TONES = 12;

  ToneProfile() { loadDefaults(); }

  void loadDefaults() {
    static const Entry kDefault[MAX_TONES] = {
        {-10.00f, 200, 200, 100}, {-2.02f, 294, 200, 100}, {-1.99f, 294, 700, 82},
        {-1.01f, 320, 400, 10},   {-0.98f, 320, 651, 29},  {0.00f, 374, 300, 5},
        {0.00f, 432, 500, 15},    {0.40f, 505, 399, 35},   {1.01f, 579, 335, 50},
        {1.99f, 652, 275, 50},    {8.00f, 1517, 241, 66},  {10.00f, 1800, 150, 70},
    };
    n_ = MAX_TONES;
    memcpy(tones_, kDefault, sizeof(kDefault));
    climbOn_ = 0.10f;
    climbOff_ = 0.10f;
    sinkOn_ = -4.00f;
    sinkOff_ = -4.00f;
  }

  // Call before feeding a config file. The default table is only discarded
  // once the first valid tone= line arrives, so a file with no (or only
  // broken) tone lines leaves the defaults in place.
  void beginParse() { parsedTones_ = 0; }

  // Feed one config line. Returns true if the line was understood.
  bool parseLine(const char* line) {
    while (*line == ' ' || *line == '\t') line++;
    if (*line == 0 || *line == '#') return false;

    if (strncasecmp(line, "tone=", 5) == 0) {
      float v[4];
      if (!parseFloats(line + 5, v, 4)) return false;
      if (parsedTones_ >= MAX_TONES) return false;
      if (v[1] < 50 || v[1] > 20000 || v[2] < 20 || v[2] > 65535) return false;
      if (parsedTones_ == 0) n_ = 0;      // first good line replaces defaults
      if (n_ > 0 && v[0] < tones_[n_ - 1].climb) return false;  // must ascend
      float d = v[3] < 0 ? 0 : (v[3] > 100 ? 100 : v[3]);
      tones_[n_++] = {v[0], (uint16_t)v[1], (uint16_t)v[2], (uint8_t)d};
      parsedTones_++;
      return true;
    }
    if (strncasecmp(line, "ClimbToneOnThreshold=", 21) == 0) { climbOn_ = atof(line + 21); return true; }
    if (strncasecmp(line, "ClimbToneOffThreshold=", 22) == 0) { climbOff_ = atof(line + 22); return true; }
    if (strncasecmp(line, "SinkToneOnThreshold=", 20) == 0) { sinkOn_ = atof(line + 20); return true; }
    if (strncasecmp(line, "SinkToneOffThreshold=", 21) == 0) { sinkOff_ = atof(line + 21); return true; }
    return false;
  }

  // Threshold state machine. Returns true while any tone should sound.
  bool updateActive(float climb) {
    if (climbActive_) {
      if (climb < climbOff_) climbActive_ = false;
    } else if (climb >= climbOn_) {
      climbActive_ = true;
    }
    if (sinkActive_) {
      if (climb > sinkOff_) sinkActive_ = false;
    } else if (climb <= sinkOn_) {
      sinkActive_ = true;
    }
    return climbActive_ || sinkActive_;
  }

  // Interpolated tone parameters at `climb` (m/s), clamped to the table ends.
  void lookup(float climb, uint16_t& freq, uint16_t& cycle, uint8_t& duty) const {
    if (n_ == 0) { freq = 0; cycle = 500; duty = 0; return; }
    int i = 0;
    while (i + 1 < n_ && tones_[i + 1].climb <= climb) i++;
    if (i + 1 >= n_ || climb <= tones_[0].climb) {
      const Entry& e = tones_[climb <= tones_[0].climb ? 0 : n_ - 1];
      freq = e.freq; cycle = e.cycle; duty = e.duty;
      return;
    }
    const Entry &a = tones_[i], &b = tones_[i + 1];
    float span = b.climb - a.climb;
    float t = span > 1e-6f ? (climb - a.climb) / span : 1.0f;
    freq = (uint16_t)(a.freq + t * ((int32_t)b.freq - a.freq));
    cycle = (uint16_t)(a.cycle + t * ((int32_t)b.cycle - a.cycle));
    duty = (uint8_t)(a.duty + t * ((int32_t)b.duty - a.duty));
  }

  int count() const { return n_; }

 private:
  static bool parseFloats(const char* s, float* out, int n) {
    char* end;
    for (int i = 0; i < n; i++) {
      out[i] = strtof(s, &end);
      if (end == s) return false;
      s = end;
      if (i < n - 1) {
        if (*s != ',') return false;
        s++;
      }
    }
    return true;
  }

  Entry tones_[MAX_TONES];
  int n_ = 0;
  int parsedTones_ = 0;
  float climbOn_ = 0.10f, climbOff_ = 0.10f, sinkOn_ = -4.00f, sinkOff_ = -4.00f;
  bool climbActive_ = false, sinkActive_ = false;
};
