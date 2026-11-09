// balance_config.h — settings for edge_balance.ino (single motor, 1D edge balancing).
// Everything you tune for balancing lives here. config.h (pins, IMU calibration, Kalman tuning)
// is the same file as in imu_logger.
#pragma once

// ---------------- Balance point ----------------
// ang_kf when the cube is balanced on its edge. Hold the cube at its balance point and send 'z'
// to measure it, then fine-tune live with '+' / '-' (0.1 deg steps). Paste the final value here.
#define THETA_EQ_DEG      (-45.4f)

// +1 or -1. Run the 'p' sign test once (cube held loosely on its edge) and paste what it prints.
#define TILT_SIGN         (1.0f)

// ---------------- LQR gains (paste the three lines printed by lqr_edge.m) ----------------
// Control law: current [A] = -(K_TH*theta + K_RATE*theta_dot + K_WHEEL*wheel_speed), SI units.
// While all three are 0 the firmware refuses to arm.
#define K_TH              (-87.13036f)     // A per rad
#define K_RATE            (-10.32519f)     // A per (rad/s)
#define K_WHEEL           (-0.03118f)      // A per (rad/s) of wheel speed

// Low-pass filter on the body rate used by the controller. Wheel vibration (worse at high wheel
// speed) gets into the gyro, and K_RATE amplifies it into rapid +/- current switching. 30 Hz removes
// that while adding almost no lag at the ~1-2 Hz balancing motion. 0 = off.
#define RATE_LPF_HZ       30.0f

// ---------------- Safety ----------------
#define CATCH_DEG         3.0f       // controller takes over when |theta - theta_eq| is below this
#define FALL_DEG          20.0f      // motor off when |theta - theta_eq| exceeds this
#define CATCH_MAX_RPM     200.0f     // only catch if the wheel has (almost) stopped
#define WHEEL_MAX_RPM     3000.0f    // motor off if the wheel saturates
#define I_MAX_BAL         3.0f       // A, current clamp (= ESCON max output limit; motor continuous 2 A)
#define REQUIRE_LINK      1          // 1 = disarm if armed from the laptop and the WiFi link is lost

// ---------------- Stopping ----------------
// After a fall, 'x', link loss or the E-stop button, the wheel is actively braked (current opposing
// its speed) instead of coasting, then the motor is switched off. 0 = just switch off and coast.
#define BRAKE_A           0.0f       // A. 0 = coast. On a bench supply braking trips the ESCON
                                     // (regenerated energy -> overvoltage). Use >0 only with a battery.
#define BRAKE_DONE_RPM    30.0f      // stop braking below this speed
#define BRAKE_TIMEOUT_MS  8000       // give up braking after this long
// Physical kill button: pressing it (pin pulled to GND) brakes the wheel and disarms.
// GPIO0 is the BOOT button on the ESP32-S3 DevKit, so it works with no extra wiring.
// To use an external button, wire it from a free GPIO to GND and put that GPIO here. -1 = none.
#define ESTOP_PIN         0

// ---------------- ESCON response check ----------------
// While balancing, the wheel must speed up when current is pushed. If over 300 ms the wheel changes by
// less than a quarter of what this current should give, the ESCON is probably in a fault: disarm.
#define WHEEL_INERTIA     5.934e-4f  // kg m^2 (your pulse test)
#define KM_NM_PER_A       0.0396f    // Nm/A

// ---------------- Sign test ('p') ----------------
#define PULSE_A           1.0f       // A
#define PULSE_MS          100
#define PULSE_WATCH_MS    250        // keep measuring the body rotation this long after the pulse

// ---------------- ESCON mapping: must match ESCON Studio ----------------
#define IA_MAX_ESCON      6.0f       // PWM 10 % = -6 A, 50 % = 0 A, 90 % = +6 A
#define SPEED_V_LO        0.2f       // AnOUT1: 0.2 V = -SPEED_RPM_FS
#define SPEED_V_HI        3.0f       //         3.0 V = +SPEED_RPM_FS
#define SPEED_RPM_FS      7000.0f
#define SPEED_SIGN        1.0f       // positive current -> positive rpm (as in the speed test)
#define SPEED_BIAS_MAX    0.15f      // V: refuse to arm if the at-rest speed signal is further off
