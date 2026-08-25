#pragma once
#include <Arduino.h>

#include "config.h"
#include "tones.h"

// Piezo driver with an XCTracer-style tone response: a 12-point table maps
// climb to {frequency, cycle length, duty %}, interpolated between points
// (tones.h — defaults baked in, overridable by /tones.txt on the SD card).
// Loudness is still the 2-bit hardware volume select; duty shapes cadence.
class Speaker {
 public:
  void init() {
    pinMode(PIN_SPKR_VOLA, OUTPUT);
    pinMode(PIN_SPKR_VOLB, OUTPUT);
    ledcAttach(PIN_SPEAKER, 1000, 10);
    ledcWriteTone(PIN_SPEAKER, 0);
    setVolume(2);
  }

  // 0=mute, 1=low, 2=med, 3=high (drives the two volume-select pins)
  void setVolume(uint8_t v) {
    volume_ = v > 3 ? 3 : v;
    digitalWrite(PIN_SPKR_VOLA, volume_ & 0x01);
    digitalWrite(PIN_SPKR_VOLB, (volume_ >> 1) & 0x01);
  }
  uint8_t volume() const { return volume_; }

  ToneProfile& tones() { return tones_; }

  // Call at ~BARO_HZ with the latest filtered climb (cm/s)
  void update(int32_t climbCms) {
    if (volume_ == 0) { tone(0); return; }
    float climb = climbCms / 100.0f;
    if (!tones_.updateActive(climb)) {
      tone(0);
      phaseMs_ = 0;
      haveLast_ = false;
      return;
    }
    uint16_t f, cyc;
    uint8_t duty;
    tones_.lookup(climb, f, cyc, duty);
    unsigned long now = millis();
    if (haveLast_) phaseMs_ += (uint32_t)(now - lastMs_);
    lastMs_ = now;
    haveLast_ = true;
    if (cyc < 20) cyc = 20;
    if (phaseMs_ >= cyc) phaseMs_ %= cyc;
    // Audible window per cycle. We sample at ~50ms (BARO_HZ), so stretch very
    // short blips to one tick or they'd be skipped entirely.
    uint32_t onMs = (uint32_t)cyc * duty / 100;
    if (duty > 0 && duty < 100 && onMs < 1000 / BARO_HZ) onMs = 1000 / BARO_HZ;
    tone(phaseMs_ < onMs ? f : 0);
  }

  void playStartup() {  // three ascending chirps; blocking, boot only
    for (uint32_t f : {800, 1100, 1400}) { tone(f); delay(90); }
    tone(0);
  }

 private:
  void tone(uint32_t freq) {
    if (freq == lastFreq_) return;  // upstream does this too: avoid LEDC re-config churn
    ledcWriteTone(PIN_SPEAKER, freq);
    lastFreq_ = freq;
  }

  ToneProfile tones_;
  uint32_t lastFreq_ = 1;
  uint32_t phaseMs_ = 0;
  unsigned long lastMs_ = 0;
  bool haveLast_ = false;
  uint8_t volume_ = 2;
};
