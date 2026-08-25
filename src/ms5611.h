#pragma once
#include <stdint.h>

// MS5611 barometer, I2C @ 0x77, OSR 4096.
// Non-blocking state machine ported from upstream Leaf (hardware/ms5611.cpp):
// call update() every loop tick; it alternates pressure/temperature conversions
// and never blocks on the ~9ms ADC time.
class MS5611 {
 public:
  bool init();          // reset, read PROM calibration, start first conversion
  void update();        // advance state machine; sets hasNewPressure()
  bool hasNewPressure();  // true once per completed pressure sample (clears on read)

  float pressurePa() const { return pressurePa_; }
  float temperatureC() const { return tempC_; }

 private:
  enum class State { Idle, MeasuringPressure, MeasuringTemp };

  uint8_t sendCommand(uint8_t cmd);
  uint16_t readProm(uint8_t addr);
  uint32_t readAdc();
  void compensate();

  State state_ = State::Idle;
  uint16_t c1_ = 0, c2_ = 0, c3_ = 0, c4_ = 0, c5_ = 0, c6_ = 0;
  uint32_t d1_ = 0, d2_ = 0, d1last_ = 0, d2last_ = 0;
  unsigned long convStartUs_ = 0;
  float pressurePa_ = 0;
  float tempC_ = 0;
  bool newPressure_ = false;
};
