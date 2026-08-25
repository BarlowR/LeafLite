#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cmath>
#endif

// 6-axis Mahony attitude filter. Replaces the ICM-20948 DMP that upstream uses:
// same job (orientation quaternion so we can find "up" regardless of how the
// device is mounted on a riser), ~80 lines instead of a firmware blob.
// No magnetometer: yaw drifts freely, which is irrelevant for vertical accel.
class Ahrs {
 public:
  // Call at IMU rate (~100Hz). accel in g, gyro in rad/s, dt in seconds.
  void update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0.01f) {
      float rn = 1.0f / norm;
      float axn = ax * rn, ayn = ay * rn, azn = az * rn;
      // Estimated gravity direction in body frame from quaternion
      float vx = 2.0f * (q1 * q3 - q0 * q2);
      float vy = 2.0f * (q0 * q1 + q2 * q3);
      float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
      // Error = cross(measured, estimated); P-only correction (bias drift is
      // handled downstream by the gravity tracker, so no integral term here)
      float ex = ayn * vz - azn * vy;
      float ey = azn * vx - axn * vz;
      float ez = axn * vy - ayn * vx;
      gx += kp_ * ex;
      gy += kp_ * ey;
      gz += kp_ * ez;
    }
    // Integrate quaternion rate
    float halfDt = 0.5f * dt;
    float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * halfDt;
    float dq1 = (q0 * gx + q2 * gz - q3 * gy) * halfDt;
    float dq2 = (q0 * gy - q1 * gz + q3 * gx) * halfDt;
    float dq3 = (q0 * gz + q1 * gy - q2 * gx) * halfDt;
    q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;
    float rn = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= rn; q1 *= rn; q2 *= rn; q3 *= rn;
  }

  // World-frame vertical (up) component of body-frame acceleration, in g.
  // At rest this reads ≈ +1.0 regardless of device orientation.
  float verticalAccelG(float ax, float ay, float az) const {
    // Third row of R(q): dot(accel_body, up_in_body)
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    return ax * vx + ay * vy + az * vz;
  }

  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

 private:
  float kp_ = 1.0f;  // higher = trusts accel more (faster converge, more maneuver error)
};
