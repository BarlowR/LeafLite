#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cmath>
#include <cstdint>
using std::isnan;
using std::isinf;
#endif

// Accel + baro fusion — a faithful port of upstream Leaf's flight-tested logic
// (instruments/imu.cpp gravity tracking + math/kalman.cpp), constants included.
// This is the part of upstream worth copying verbatim: the Kalman math is
// textbook, but the gravity-estimate gating below is what stops the vario from
// chirping at brake inputs and thermal turbulence, and those tolerances were
// tuned in the air, not at a desk.

// ---- Upstream constants (imu.cpp) ----
static const double FUSION_GRAVITY_ACCEL_TOL_G = 0.065;   // maneuvering gate
static const double FUSION_GRAVITY_VERT_TOL_G = 0.10;     // sustained-lift gate
static const double FUSION_GRAVITY_MAX_SLEW_G_S = 0.05;
static const double FUSION_MIN_GRAVITY_EST_G = 0.94;
static const double FUSION_MAX_GRAVITY_EST_G = 1.06;
static const int FUSION_VERT_REJECT_RESET_SAMPLES = 40;   // 2s @ 20Hz
// K_UPDATE: 90% weight to new data after 5s => ln(0.1)/5
static const double FUSION_K_UPDATE = -0.4605170186;
// Kalman: pVar = 0.1^2, aVar = 0.3^2 (imu.h)
static const double FUSION_POS_VAR = 0.01;
static const double FUSION_ACCEL_VAR = 0.09;

// Port of upstream KalmanFilterPA: 2-state (position, velocity), acceleration
// as measured input. Same propagation/update equations; fatalError() paths
// replaced with self-reinit since there is no display to die politely on.
class KalmanPA {
 public:
  void update(double t, double pos, double accel) {
    if (isnan(pos) || isinf(pos) || isnan(accel) || isinf(accel)) return;
    if (!initialized_) { init(t, pos, accel); return; }
    double dt = t - t_;
    if (dt <= 0.0) return;
    if (dt > 1.0) { init(t, pos, accel); return; }

    double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    t_ = t;
    // Prediction
    a_ = accel;
    p_ += dt * v_ + dt2 * a_ / 2;
    v_ += dt * a_;
    // Covariance (upstream's exact form)
    double inc = dt * p22_ + dt3 * FUSION_ACCEL_VAR / 2;
    p11_ += dt * (p12_ + p21_ + inc) - (dt4 * FUSION_ACCEL_VAR / 4);
    p21_ += inc;
    p12_ += inc;
    p22_ += dt2 * FUSION_ACCEL_VAR;
    // Gain + update
    double s = p11_ + FUSION_POS_VAR;
    double k11 = p11_ / s, k12 = p12_ / s;
    double dp = pos - p_;
    p_ += k11 * dp;
    v_ += k12 * dp;
    p22_ -= k12 * p12_;
    p12_ -= k12 * p11_;
    p21_ -= k11 * p21_;
    p11_ -= k11 * p11_;
    if (isnan(p_) || isnan(v_)) init(t, pos, accel);
  }

  bool initialized() const { return initialized_; }
  double position() const { return p_; }
  double velocity() const { return v_; }

 private:
  void init(double t, double pos, double accel) {
    t_ = t; p_ = pos; v_ = 0; a_ = accel;
    p11_ = p12_ = p21_ = p22_ = 0;
    initialized_ = true;
  }
  bool initialized_ = false;
  double t_ = 0, p_ = 0, v_ = 0, a_ = 0;
  double p11_ = 0, p12_ = 0, p21_ = 0, p22_ = 0;
};

// Gravity tracker: maintains a slow estimate of what "1g" currently reads and
// gates when it may update. Output: net vertical accel for the Kalman, with
// the input zeroed (not trusted) during sustained vertical disagreement —
// upstream does exactly this (kalmanAccelVert_ = 0.0 on vertical reject).
class GravityTracker {
 public:
  // awzG: world-frame vertical accel in g. Returns accel input for Kalman in g.
  double process(double awzG, double dtS) {
    double accelTot = lastTotG_;  // caller sets via setTotal() before process()
    double accelVert = awzG - gravity_;
    double kalmanAccel = accelVert;

    if (fabs(accelTot - 1.0) > FUSION_GRAVITY_ACCEL_TOL_G) {
      // Maneuvering (turns, brake input): don't update gravity, but the
      // vertical accel is still real — pass it through.
      vertRejects_ = 0;
    } else if (fabs(accelVert) > FUSION_GRAVITY_VERT_TOL_G) {
      // Sustained vertical disagreement: either strong lift or a stale gravity
      // estimate. Don't trust it as Kalman input; after 2s of this with an
      // implausible estimate, reset gravity (upstream recovery path).
      kalmanAccel = 0.0;
      if (++vertRejects_ >= FUSION_VERT_REJECT_RESET_SAMPLES &&
          fabs(gravity_ - 1.0) > FUSION_GRAVITY_VERT_TOL_G) {
        gravity_ = 1.0;
        vertRejects_ = 0;
      }
    } else {
      // Quiet: slow exponential update with slew limit and plausibility clamp
      vertRejects_ = 0;
      if (dtS > 0.0 && dtS < 1.0) {
        double f = exp(FUSION_K_UPDATE * dtS);
        double next = gravity_ * f + awzG * (1 - f);
        double maxDelta = FUSION_GRAVITY_MAX_SLEW_G_S * dtS;
        double delta = next - gravity_;
        if (fabs(delta) > maxDelta) next = gravity_ + (delta > 0 ? maxDelta : -maxDelta);
        if (next < FUSION_MIN_GRAVITY_EST_G) next = FUSION_MIN_GRAVITY_EST_G;
        if (next > FUSION_MAX_GRAVITY_EST_G) next = FUSION_MAX_GRAVITY_EST_G;
        gravity_ = next;
      }
    }
    return kalmanAccel;
  }

  void setTotal(double totG) { lastTotG_ = totG; }
  double gravity() const { return gravity_; }

 private:
  double gravity_ = 1.0;
  double lastTotG_ = 1.0;
  int vertRejects_ = 0;
};
