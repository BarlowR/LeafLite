#include "ms5611.h"

#include <Arduino.h>
#include <Wire.h>

static const uint8_t ADDR = 0x77;
static const uint8_t CMD_RESET = 0x1E;
static const uint8_t CMD_CONV_P = 0x48;  // D1, OSR 4096 (matches upstream)
static const uint8_t CMD_CONV_T = 0x58;  // D2, OSR 4096
static const unsigned long CONV_TIME_US = 9000;  // OSR4096 max ~9.04ms

uint8_t MS5611::sendCommand(uint8_t cmd) {
  Wire.beginTransmission(ADDR);
  Wire.write(cmd);
  return Wire.endTransmission();
}

uint16_t MS5611::readProm(uint8_t addr) {
  sendCommand(0xA0 | (addr << 1));
  Wire.requestFrom(ADDR, (uint8_t)2);
  uint16_t v = (uint16_t)Wire.read() << 8;
  v |= Wire.read();
  return v;
}

uint32_t MS5611::readAdc() {
  sendCommand(0x00);
  Wire.requestFrom(ADDR, (uint8_t)3);
  uint32_t v = (uint32_t)Wire.read() << 16;
  v |= (uint32_t)Wire.read() << 8;
  v |= Wire.read();
  return v;
}

bool MS5611::init() {
  sendCommand(CMD_RESET);
  delay(5);
  c1_ = readProm(1);  // SENS_T1
  c2_ = readProm(2);  // OFF_T1
  c3_ = readProm(3);  // TCS
  c4_ = readProm(4);  // TCO
  c5_ = readProm(5);  // T_REF
  c6_ = readProm(6);  // TEMPSENS
  if (c1_ == 0 || c1_ == 0xFFFF) return false;  // no sensor / bus fault

  // Prime one temperature reading synchronously so the first pressure
  // compensation isn't garbage.
  sendCommand(CMD_CONV_T);
  delay(10);
  d2_ = readAdc();
  d2last_ = d2_;

  sendCommand(CMD_CONV_P);
  convStartUs_ = micros();
  state_ = State::MeasuringPressure;
  return true;
}

void MS5611::update() {
  switch (state_) {
    case State::Idle:
      sendCommand(CMD_CONV_P);
      convStartUs_ = micros();
      state_ = State::MeasuringPressure;
      return;

    case State::MeasuringPressure:
      if (micros() - convStartUs_ <= CONV_TIME_US) return;
      d1_ = readAdc();
      if (d1_ == 0) d1_ = d1last_;  // upstream quirk-handling: reuse last on bad read
      else d1last_ = d1_;
      compensate();
      newPressure_ = true;
      // Interleave a temperature conversion so temp tracks slowly (every other cycle)
      sendCommand(CMD_CONV_T);
      convStartUs_ = micros();
      state_ = State::MeasuringTemp;
      return;

    case State::MeasuringTemp:
      if (micros() - convStartUs_ <= CONV_TIME_US) return;
      d2_ = readAdc();
      if (d2_ == 0) d2_ = d2last_;
      else d2last_ = d2_;
      state_ = State::Idle;
      return;
  }
}

bool MS5611::hasNewPressure() {
  bool v = newPressure_;
  newPressure_ = false;
  return v;
}

// Datasheet first + second order compensation.
void MS5611::compensate() {
  int32_t dT = (int32_t)d2_ - ((int32_t)c5_ << 8);
  int32_t temp = 2000 + (((int64_t)dT * c6_) >> 23);  // centi-degC

  int64_t off = ((int64_t)c2_ << 16) + (((int64_t)c4_ * dT) >> 7);
  int64_t sens = ((int64_t)c1_ << 15) + (((int64_t)c3_ * dT) >> 8);

  // Second-order correction below 20C (matters on cold mornings at launch)
  if (temp < 2000) {
    int32_t t2 = (int32_t)(((int64_t)dT * dT) >> 31);
    int64_t off2 = 5 * ((int64_t)(temp - 2000) * (temp - 2000)) / 2;
    int64_t sens2 = off2 / 2;
    if (temp < -1500) {
      off2 += 7 * ((int64_t)(temp + 1500) * (temp + 1500));
      sens2 += 11 * ((int64_t)(temp + 1500) * (temp + 1500)) / 2;
    }
    temp -= t2;
    off -= off2;
    sens -= sens2;
  }

  int32_t p = (int32_t)((((int64_t)d1_ * sens >> 21) - off) >> 15);  // Pa
  pressurePa_ = (float)p;
  tempC_ = temp / 100.0f;
}
