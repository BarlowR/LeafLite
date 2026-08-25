#pragma once
#include <Arduino.h>

#include "config.h"

// Charger + battery per upstream power.cpp semantics.
// Init sets 500mA input current (upstream's default, safe on any USB port).
// If you know your charger/port can take it, PowerMax selects ~1.35A input
// (~810mA to the battery per upstream's ILIM/ISET notes).
class PowerMon {
 public:
  void init() {
    // FIRST: latch our own power on. On battery the regulator only stays up
    // while the power button is physically held — until this pin goes HIGH.
    pinMode(PIN_POWER_LATCH, OUTPUT);
    digitalWrite(PIN_POWER_LATCH, HIGH);
    pinMode(PIN_CHG_I1, OUTPUT);
    pinMode(PIN_CHG_I2, OUTPUT);
    pinMode(PIN_CHG_GOOD, INPUT_PULLUP);
    pinMode(PIN_BATT_SENSE, INPUT);
    setInput500mA();
  }

  // Drop the power latch. On battery this cuts our own power dead (true off);
  // on USB the rail stays up, so callers should still deep-sleep afterwards.
  void latchOff() { digitalWrite(PIN_POWER_LATCH, LOW); }

  // I2 low FIRST when leaving Max, so we never pass through HIGH/HIGH
  // (= USB suspend = charging silently off). Ordering is load-bearing.
  void setInput500mA() {
    digitalWrite(PIN_CHG_I2, LOW);
    digitalWrite(PIN_CHG_I1, HIGH);
  }
  void setInputMax() {
    digitalWrite(PIN_CHG_I1, LOW);
    digitalWrite(PIN_CHG_I2, HIGH);
  }

  bool charging() { return digitalRead(PIN_CHG_GOOD) == LOW; }  // active low

  // Upstream power.cpp calibration
  static const int32_t BATT_FULL_MV = 4080;
  static const int32_t BATT_EMPTY_MV = 3250;
  static const int32_t BATT_SHUTDOWN_MV = 3200;

  uint32_t batteryMV() {
    // Upstream's divider math: mV * 69/41, then an EMA (~2s at the ~4Hz call
    // rate from display+BLE): raw reads bounce hard while the charger is
    // switching, and the linear percent map amplifies every wobble.
    uint32_t sum = 0;
    for (int i = 0; i < 4; i++) sum += analogReadMilliVolts(PIN_BATT_SENSE);
    uint32_t mv = (sum / 4) * 69 / 41;
    if (filtMv_ == 0) filtMv_ = mv;
    filtMv_ = (filtMv_ * 7 + mv + 4) / 8;
    return filtMv_;
  }

  // Linear map over upstream's calibrated range. Still optimistic under load
  // and while charging (voltage rides high) — fine for an icon.
  uint8_t batteryPercent() {
    int32_t mv = (int32_t)batteryMV();
    int32_t pct = (mv - BATT_EMPTY_MV) * 100 / (BATT_FULL_MV - BATT_EMPTY_MV);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
  }

  // True below upstream's 3.20V cutoff (filtered; false until first reading).
  bool belowShutdown() const { return filtMv_ != 0 && filtMv_ < (uint32_t)BATT_SHUTDOWN_MV; }

  // Last filtered value without touching the ADC (0 until first reading).
  uint32_t filteredMV() const { return filtMv_; }

 private:
  uint32_t filtMv_ = 0;
};
