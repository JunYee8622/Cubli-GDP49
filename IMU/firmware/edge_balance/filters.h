// filters.h — 1D tilt estimators for the edge-balancing stage.
// Units: angles in degrees, rates in deg/s, dt in seconds.
// The same classes can be dropped straight into the balancing controller later.
#pragma once
#include <math.h>

// Wrap an angle difference into [-180, 180) deg, so the accelerometer flipping past +/-180
// during violent motion can't drag the estimate by 360 deg.
inline float wrap180(float x) {
  x = fmodf(x + 180.0f, 360.0f);
  if (x < 0.0f) x += 360.0f;
  return x - 180.0f;
}

// ---------------------------------------------------------------------------
// Complementary filter: fixed blend of integrated gyro and accelerometer angle.
//   angle = alpha * (angle + rate*dt) + (1 - alpha) * accAngle
// alpha close to 1 = trust gyro more. Crossover time constant tau = alpha*dt/(1-alpha).
// ---------------------------------------------------------------------------
class ComplementaryFilter {
 public:
  explicit ComplementaryFilter(float alpha) : alpha_(alpha) {}

  void reset(float angle) { angle_ = angle; initialised_ = true; }

  float update(float gyroRate, float accAngle, float dt) {
    if (!initialised_) reset(accAngle);
    const float pred = angle_ + gyroRate * dt;
    angle_ = pred + (1.0f - alpha_) * wrap180(accAngle - pred);  // = alpha*pred + (1-alpha)*acc
    return angle_;
  }

  float angle() const { return angle_; }
  void setAlpha(float a) { alpha_ = a; }

 private:
  float alpha_;
  float angle_ = 0.0f;
  bool initialised_ = false;
};

// ---------------------------------------------------------------------------
// 1D Kalman filter.
//   State     x = [angle, gyroBias]
//   Input     u = gyro rate (raw, bias is estimated by the filter)
//   Measure   z = accelerometer angle
// Tuning:
//   qAngle — how much to distrust the gyro-based prediction (process noise)
//   qBias  — how fast the gyro bias is allowed to wander
//   rMeas  — how noisy the accelerometer angle is (measurement noise)
// ---------------------------------------------------------------------------
class KalmanTilt {
 public:
  KalmanTilt(float qAngle, float qBias, float rMeas)
      : qAngle_(qAngle), qBias_(qBias), r_(rMeas) {}

  void reset(float angle, float bias = 0.0f) {
    angle_ = angle;
    bias_ = bias;
    P00_ = 10.0f;  // initial angle uncertainty (deg^2)
    P01_ = P10_ = 0.0f;
    P11_ = 1.0f;   // initial bias uncertainty ((deg/s)^2)
    initialised_ = true;
  }

  float update(float gyroRate, float accAngle, float dt) {
    if (!initialised_) reset(accAngle);

    // --- Predict: integrate bias-corrected gyro, grow uncertainty ---
    angle_ += (gyroRate - bias_) * dt;
    P00_ += dt * (dt * P11_ - P01_ - P10_ + qAngle_);
    P01_ -= dt * P11_;
    P10_ -= dt * P11_;
    P11_ += qBias_ * dt;

    // --- Update: compare with accelerometer, shrink uncertainty ---
    const float y = wrap180(accAngle - angle_);  // innovation (measurement - prediction)
    const float S = P00_ + r_;           // innovation variance
    const float K0 = P00_ / S;           // gain on angle  (the "blending weight")
    const float K1 = P10_ / S;           // gain on bias

    angle_ += K0 * y;
    bias_ += K1 * y;

    const float P00 = P00_, P01 = P01_;
    P00_ -= K0 * P00;
    P01_ -= K0 * P01;
    P10_ -= K1 * P00;
    P11_ -= K1 * P01;

    lastGain_ = K0;
    return angle_;
  }

  float angle() const { return angle_; }
  float bias() const { return bias_; }
  float gain() const { return lastGain_; }

 private:
  float qAngle_, qBias_, r_;
  float angle_ = 0.0f, bias_ = 0.0f;
  float P00_ = 0.0f, P01_ = 0.0f, P10_ = 0.0f, P11_ = 0.0f;
  float lastGain_ = 0.0f;
  bool initialised_ = false;
};
