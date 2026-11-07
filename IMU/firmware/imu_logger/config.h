// config.h — everything you are expected to edit lives here.
#pragma once

// ---------------- Hardware ----------------
#define I2C_SDA      47          // ESP32 default I2C pins (DevKit V1). Change for your board.
#define I2C_SCL      21
#define ICM_ADDR     0x68        // Adafruit ICM-20948 default. 0x68 if the AD0 jumper is bridged.

// ---------------- Sampling ----------------
#define LOOP_HZ      500         // fixed loop rate. Overruns are reported as "# overruns=N" lines.
#define SERIAL_BAUD  921600

// Sensor ranges. Calibrate in the SAME range you will use on the cube.
// 16 g / 2000 dps is what jump-up needs (brake impact). For pure balancing, 4 g / 500 dps
// gives finer resolution. Options: ICM20948_ACCEL_RANGE_{2,4,8,16}_G, ICM20948_GYRO_RANGE_{250,500,1000,2000}_DPS
#define ACC_RANGE    ICM20948_ACCEL_RANGE_16_G
#define GYRO_RANGE   ICM20948_GYRO_RANGE_2000_DPS

// ---------------- 1D tilt axis ----------------
// 0 = rotation about IMU X  -> accel angle = atan2(ay, az), rate = gx
// 1 = rotation about IMU Y  -> accel angle = atan2(-ax, az), rate = gy
#define TILT_AXIS    1

// ---------------- Calibration ----------------
// Paste the values printed by test_1_accel_calibration.py and test_2_gyro_still.py.
// calibrated_acc = (raw_acc - ACC_OFFSET) / ACC_SCALE      [m/s^2]
// calibrated_gyro = raw_gyro - GYRO_BIAS                    [deg/s]
static const float ACC_OFFSET[3] = {-0.04315f, -0.22845f, 0.20643f}; 
static const float ACC_SCALE[3]  = {1.00190f, 1.00292f, 1.00528f};
static const float GYRO_BIAS[3]  = {-1.02486f, 0.12553f, 0.44382f};

// ---------------- Filter tuning (start values) ----------------
#define COMP_ALPHA   0.98f       // complementary: closer to 1 = trust gyro more
#define KF_Q_ANGLE   0.001f      // Kalman: distrust of gyro prediction
#define KF_Q_BIAS    0.003f      // Kalman: how fast gyro bias may wander
#define KF_R_MEAS    0.03f       // Kalman: accelerometer angle noise (deg^2)

#define MOTOR_PWM_PIN        6     // DI1_M3
#define MOTOR_EN_PIN         5     // DI2_M3
#define ESC_SPEED_ADC_PIN    7     // AO1_M3

#define ESC_ADC_GAIN         1.0f
#define ESC_ADC_OFFSET_V     1.6f
#define ESC_SPEED_RPM_PER_V  5714.0f