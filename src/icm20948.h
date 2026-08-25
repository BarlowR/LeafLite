#pragma once
#include <Arduino.h>
#include <Wire.h>

// Minimal raw ICM-20948 driver: accel ±4g + gyro ±500dps with DLPF, ~100Hz ODR.
// Deliberately NO DMP (upstream uses SparkFun's DMP path — that means loading a
// 14KB Invensense firmware blob; the whole reason to rewrite is to not need it)
// and NO magnetometer (a vario needs gravity direction, not heading).
// I2C address 0x68 (AD0=0, per upstream hardware/icm_20948.cpp).
class Icm20948 {
 public:
  bool init() {
    if (read8(0, 0x00) != 0xEA) return false;  // WHO_AM_I
    write8(0, 0x06, 0x80);                     // PWR_MGMT_1: reset
    delay(20);
    write8(0, 0x06, 0x01);  // clear SLEEP, auto clock
    write8(0, 0x07, 0x00);  // PWR_MGMT_2: all axes on
    delay(10);
    // Bank 2: DLPF on, FS accel=±4g gyro=±500dps, ODR = 1125/(1+10) ≈ 102 Hz
    write8(2, 0x00, 10);                       // GYRO_SMPLRT_DIV
    write8(2, 0x01, (3 << 3) | (1 << 1) | 1);  // GYRO_CONFIG_1
    write8(2, 0x10, 0);                        // ACCEL_SMPLRT_DIV_1
    write8(2, 0x11, 10);                       // ACCEL_SMPLRT_DIV_2
    write8(2, 0x14, (3 << 3) | (1 << 1) | 1);  // ACCEL_CONFIG
    setBank(0);
    return true;
  }

  // Burst-read accel+gyro. ax..az in g, gx..gz in rad/s.
  bool read(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
    Wire.beginTransmission(ADDR);
    Wire.write(0x2D);  // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(ADDR, (uint8_t)12) != 12) return false;
    int16_t r[6];
    for (int i = 0; i < 6; i++) {
      r[i] = (int16_t)((Wire.read() << 8) | Wire.read());
    }
    ax = r[0] / 8192.0f;  // ±4g
    ay = r[1] / 8192.0f;
    az = r[2] / 8192.0f;
    const float d2r = 0.017453293f / 65.5f;  // ±500dps -> rad/s
    gx = r[3] * d2r;
    gy = r[4] * d2r;
    gz = r[5] * d2r;
    return true;
  }

 private:
  static const uint8_t ADDR = 0x68;

  void setBank(uint8_t b) {
    if (b == bank_) return;
    Wire.beginTransmission(ADDR);
    Wire.write(0x7F);
    Wire.write(b << 4);
    Wire.endTransmission();
    bank_ = b;
  }
  void write8(uint8_t bank, uint8_t reg, uint8_t val) {
    setBank(bank);
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }
  uint8_t read8(uint8_t bank, uint8_t reg) {
    setBank(bank);
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR, (uint8_t)1);
    return Wire.read();
  }

  uint8_t bank_ = 0xFF;
};
