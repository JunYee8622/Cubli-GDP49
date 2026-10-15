// imu_logger.ino — ESP32 + Adafruit ICM-20948 test logger.
//
// Streams one CSV line per loop at LOOP_HZ:
//   t_us,ax,ay,az,gx,gy,gz,temp,marker,ang_comp,ang_kf
//   ax..az  raw accelerometer   [m/s^2]  (uncalibrated: calibration is applied offline)
//   gx..gz  raw gyro            [deg/s]  (uncalibrated)
//   marker  increments every time an 'm' arrives on serial (press Enter in log_serial.py)
//   ang_*   live 1D tilt estimates using config.h calibration + tuning [deg]
//
// Libraries (Arduino Library Manager): "Adafruit ICM20X", "Adafruit BusIO", "Adafruit Unified Sensor".

#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include "config.h"
#include "filters.h"

static const float RAD2DEG = 57.29577951f;

Adafruit_ICM20948 icm;
ComplementaryFilter comp(COMP_ALPHA);
KalmanTilt kf(KF_Q_ANGLE, KF_Q_BIAS, KF_R_MEAS);

uint32_t marker = 0;
uint32_t overruns = 0;

static float accelAngle(const float a[3]) {
#if TILT_AXIS == 0
  return atan2f(a[1], a[2]) * RAD2DEG;
#else
  return atan2f(-a[0], a[2]) * RAD2DEG;
#endif
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  while (!icm.begin_I2C(ICM_ADDR, &Wire)) {
    Serial.println("# ERROR: ICM-20948 not found. Check wiring / address.");
    delay(1000);
  }
  icm.setAccelRange(ACC_RANGE);
  icm.setGyroRange(GYRO_RANGE);
  // Output data rate = 1125 / (1 + divisor) Hz. Divisor 1 -> 562.5 Hz (just above LOOP_HZ).
  icm.setAccelRateDivisor(1);
  icm.setGyroRateDivisor(1);

  Serial.println("# imu_logger ready");
  Serial.println("t_us,ax,ay,az,gx,gy,gz,temp,marker,ang_comp,ang_kf");
}

void loop() {
  static const uint32_t period = 1000000UL / LOOP_HZ;
  static uint32_t next = micros();
  static uint32_t last = micros();
  static uint32_t lastReport = millis();

  while (Serial.available()) {
    if (Serial.read() == 'm') marker++;
  }

  uint32_t now = micros();
  if ((int32_t)(now - next) < 0) return;          // not time yet
  if ((int32_t)(now - next) > (int32_t)period) {  // late by more than a full period
    overruns++;
    if ((int32_t)(now - next) > (int32_t)(10 * period)) next = now;  // resync if far behind
  }
  next += period;

  sensors_event_t accel, gyro, temp, mag;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  const float aRaw[3] = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};
  const float gRaw[3] = {gyro.gyro.x * RAD2DEG, gyro.gyro.y * RAD2DEG, gyro.gyro.z * RAD2DEG};

  float a[3], g[3];
  for (int i = 0; i < 3; i++) {
    a[i] = (aRaw[i] - ACC_OFFSET[i]) / ACC_SCALE[i];
    g[i] = gRaw[i] - GYRO_BIAS[i];
  }

  const float dt = (now - last) * 1e-6f;
  last = now;
  const float accAng = accelAngle(a);
  const float rate = g[TILT_AXIS];
  const float angComp = comp.update(rate, accAng, dt);
  const float angKf = kf.update(rate, accAng, dt);

  char buf[160];
  int n = snprintf(buf, sizeof(buf),
                   "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%lu,%.3f,%.3f\n",
                   (unsigned long)now, aRaw[0], aRaw[1], aRaw[2], gRaw[0], gRaw[1], gRaw[2],
                   temp.temperature, (unsigned long)marker, angComp, angKf);
  Serial.write((const uint8_t*)buf, n);

  if (millis() - lastReport > 5000) {
    lastReport = millis();
    Serial.printf("# overruns=%lu\n", (unsigned long)overruns);
  }
}
