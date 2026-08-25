#pragma once
#include <math.h>
#include <stdint.h>

// Altitude + climb-rate estimator.
// Two-state Kalman filter (altitude, vertical speed) fed by baro altitude at 20Hz.
// This REPLACES upstream's filter stack (linear regression + running averages).
// It is the standard approach in open vario firmware, but it is NOT tuned to feel
// identical to Leaf — expect to adjust accelVariance/measurementVariance after
// bench + flight comparison.
class Vario {
 public:
  void reset(float altM) {
    alt_ = altM;
    vel_ = 0;
    p00_ = 100; p01_ = 0; p10_ = 0; p11_ = 10;
    initialized_ = true;
  }

  bool initialized() const { return initialized_; }

  // Pressure (Pa) -> ISA pressure altitude (m), QNE 1013.25
  static float pressureToAltM(float pa) {
    return 44330.0f * (1.0f - powf(pa / 101325.0f, 0.1902949f));
  }

  void update(float measuredAltM, float dt) {
    if (!initialized_) { reset(measuredAltM); return; }

    // Predict
    alt_ += vel_ * dt;
    float q = accelVariance;
    p00_ += dt * (2.0f * p01_ + dt * p11_) + q * dt * dt * dt * dt / 4.0f;
    p01_ += dt * p11_ + q * dt * dt * dt / 2.0f;
    p10_ = p01_;
    p11_ += q * dt * dt;

    // Update
    float y = measuredAltM - alt_;
    float s = p00_ + measurementVariance;
    float k0 = p00_ / s;
    float k1 = p10_ / s;
    alt_ += k0 * y;
    vel_ += k1 * y;
    p11_ -= k1 * p01_;
    p10_ -= k1 * p00_;
    p01_ -= k0 * p01_;
    p00_ -= k0 * p00_;
  }

  float altM() const { return alt_; }
  int32_t climbCms() const { return (int32_t)lroundf(vel_ * 100.0f); }

  // Tuning knobs (see header comment)
  // Tuned by replaying upstream's recorded 50Hz baro data (test/replay.cpp):
  // real baro alt-noise ~0.10m stddev; these values give ~0.22 m/s rest noise
  // at ~1s lag. This is FALLBACK-quality (typical cheap baro vario) — the fused
  // IMU path in fusion.h is the primary instrument (0.07 m/s, leads by ~1s).
  float accelVariance = 0.02f;       // process noise: higher = snappier, noisier
  float measurementVariance = 0.4f;  // baro measurement variance (m^2)

 private:
  bool initialized_ = false;
  float alt_ = 0, vel_ = 0;
  float p00_ = 0, p01_ = 0, p10_ = 0, p11_ = 0;
};
