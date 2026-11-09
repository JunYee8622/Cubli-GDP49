// motor.h — ESP32-S3 <-> maxon ESCON Module 24/2 interface for the reaction wheel.
//
// The ESCON runs in CURRENT (torque) mode with a PWM set value. This file:
//   - commands motor current with a 1 kHz PWM on DigIN1 and an enable line on DigIN2,
//   - reads actual speed / actual current from the ESCON analog outputs (AnOUT1 / AnOUT2),
//   - reads wheel speed and direction directly from the motor Hall sensors,
//   - reads the ESCON "Ready" digital output.
//
// Every pin defaults to -1 (not connected). Set the ones you have wired in config.h.
//
// ------------------------------------------------------------------------------------------
// WIRING  (ESCON Module 24/2 connector pin numbers from the maxon Hardware Reference)
//
//  ESP32-S3                          ESCON Module 24/2
//  MOTOR_PWM_PIN    ------------->   DigIN1  (pin 17)  PWM set value, 1 kHz, 10..90 % duty
//  MOTOR_EN_PIN     ------------->   DigIN2  (pin 16)  Enable, high active
//  MOTOR_READY_PIN  <-------------   DigIN/DigOUT4 (pin 14) configured as "Ready"
//                                    + 4.7k pull-up from the ESP32 pin to 3.3 V
//  ESC_SPEED_ADC_PIN   <--[LS]----   AnOUT1  (pin 20)  "Actual speed averaged",   -4..+4 V
//  ESC_CURRENT_ADC_PIN <--[LS]----   AnOUT2  (pin 19)  "Actual current averaged", -4..+4 V
//  HALL1/2/3_PIN    <--[HD]-------   Hall sensor 1/2/3 (pins 7/8/9), tapped in parallel
//  GND              --------------   GND     (pin 18)  common ground is essential
//
//  [LS] Level shifter, one per analog output. The ESP32 pin must never go below 0 V or above
//       3.3 V, but AnOUT swings -4..+4 V:
//
//         AnOUT --- 30k ---+--- 22k --- 3V3
//                          +--- 51k --- GND
//                          +--- 10nF -- GND
//                          +----------> ESP32 ADC pin
//
//       V_pin = 1.525 V + 0.339 * V_anout   ->  0.17 V (at -4 V) .. 2.88 V (at +4 V)
//       The ADC pins MUST be ADC1 (GPIO1..GPIO10 on the ESP32-S3): ADC2 does not work while
//       WiFi is on.
//
//  [HD] Hall divider, one per Hall line. The ESCON pulls each Hall line up to 5.45 V inside,
//       and ESP32-S3 pins are NOT 5 V tolerant:
//
//         Hall line --- 68k ---+----------> ESP32 pin
//                              +--- 100k -- GND
//
//       5.1 V -> 3.06 V at the ESP32. The ESCON still sees > 5 V, so its own commutation is
//       unaffected.
//
//  ESP32 3.3 V logic is enough for DigIN1/DigIN2 (logic 1 > 2.4 V typ.).
//  Avoid strapping pins (GPIO0, 3, 45, 46), USB pins (19, 20), the I2C pins in config.h,
//  and GPIO26..37 (flash / PSRAM on most S3 modules).
//
//  CHECK WITH A MULTIMETER BEFORE CONNECTING ANY ESCON OUTPUT TO THE ESP32:
//    - each [LS] output stays between 0.1 and 3.0 V while the motor runs both directions,
//    - each [HD] output is below 3.3 V,
//    - the Ready line is below 3.3 V when high.
//
// ------------------------------------------------------------------------------------------
// ESCON STUDIO SETTINGS THAT MUST MATCH THE NUMBERS BELOW
//   Mode: Current controller.  Set value: PWM on DigIN1,
//         MOTOR_I_AT_10PCT A at 10 % duty, MOTOR_I_AT_90PCT A at 90 % duty (50 % = 0 A).
//   Enable: DigIN2, "Enable", high active.
//   AnOUT1: Actual speed averaged,   ESC_SPEED_RPM_PER_V rpm per volt (e.g. -8000..+8000 rpm = -4..+4 V)
//   AnOUT2: Actual current averaged, ESC_CURRENT_A_PER_V A per volt   (e.g. -6..+6 A = -4..+4 V)
//   DigOUT4: Ready.
// ------------------------------------------------------------------------------------------
#pragma once
#include <Arduino.h>
#include <math.h>
#include <stdarg.h>
#include "esp_arduino_version.h"
#include "esp_timer.h"
#include "soc/gpio_reg.h"

// ---------------- pins (-1 = not connected; set in config.h) ----------------
#ifndef MOTOR_PWM_PIN
  #define MOTOR_PWM_PIN        -1   // -> ESCON DigIN1
#endif
#ifndef MOTOR_EN_PIN
  #define MOTOR_EN_PIN         -1   // -> ESCON DigIN2
#endif
#ifndef MOTOR_READY_PIN
  #define MOTOR_READY_PIN      -1   // <- ESCON DigOUT4 (Ready)
#endif
#ifndef ESC_SPEED_ADC_PIN
  #define ESC_SPEED_ADC_PIN    -1   // <- AnOUT1 via level shifter (ADC1 pin only)
#endif
#ifndef ESC_CURRENT_ADC_PIN
  #define ESC_CURRENT_ADC_PIN  -1   // <- AnOUT2 via level shifter (ADC1 pin only)
#endif
#ifndef HALL1_PIN
  #define HALL1_PIN            -1   // <- Hall 1 via divider
#endif
#ifndef HALL2_PIN
  #define HALL2_PIN            -1
#endif
#ifndef HALL3_PIN
  #define HALL3_PIN            -1
#endif

// ---------------- ESCON Studio mapping (must match ESCON Studio) ----------------
#ifndef MOTOR_PWM_FREQ_HZ
  #define MOTOR_PWM_FREQ_HZ    1000      // ESCON accepts 10 Hz..5 kHz; maxon suggests ~1 kHz
#endif
#ifndef MOTOR_I_AT_10PCT
  #define MOTOR_I_AT_10PCT     -2.0f     // A at 10 % duty
#endif
#ifndef MOTOR_I_AT_90PCT
  #define MOTOR_I_AT_90PCT      2.0f     // A at 90 % duty
#endif
#ifndef ESC_SPEED_RPM_PER_V
  #define ESC_SPEED_RPM_PER_V  2000.0f   // AnOUT1 scale: rpm per volt at the ESCON pin
#endif
#ifndef ESC_CURRENT_A_PER_V
  #define ESC_CURRENT_A_PER_V  1.5f      // AnOUT2 scale: A per volt at the ESCON pin
#endif
#ifndef READY_ACTIVE_LEVEL
  #define READY_ACTIVE_LEVEL   HIGH      // flip to LOW if 'ready' reads 0 when the ESCON is fine
#endif

// ---------------- level shifter + ADC ----------------
#ifndef ESC_ADC_GAIN
  #define ESC_ADC_GAIN         0.339f    // V_pin per V_anout for the 30k/22k/51k network
#endif
#ifndef ESC_ADC_OFFSET_V
  #define ESC_ADC_OFFSET_V     1.525f    // V_pin at V_anout = 0 (re-measured at boot)
#endif
#ifndef ESC_ADC_SAMPLES
  #define ESC_ADC_SAMPLES      2         // reads averaged per loop per channel
#endif

// ---------------- motor / Halls ----------------
#ifndef HALL_POLE_PAIRS
  #define HALL_POLE_PAIRS      8         // CHECK the motor datasheet ("Number of pole pairs")
#endif
#ifndef HALL_DIR_SIGN
  #define HALL_DIR_SIGN        1         // flip to -1 if hall_rpm and esc_rpm have opposite signs
#endif
#ifndef HALL_STOP_TIMEOUT_US
  #define HALL_STOP_TIMEOUT_US 200000    // no Hall edge for this long -> speed = 0
#endif

// ---------------- test-mode safety ----------------
#ifndef MOTOR_TEST_MAX_A
  #define MOTOR_TEST_MAX_A     0.5f      // commands from the laptop are clamped to +/- this
#endif
#ifndef MOTOR_TEST_MAX_RPM
  #define MOTOR_TEST_MAX_RPM   2000.0f   // above this, any command that speeds the wheel up is zeroed
#endif
#ifndef MOTOR_CMD_TIMEOUT_MS
  #define MOTOR_CMD_TIMEOUT_MS 2500UL    // laptop link silent this long -> motor disabled
#endif

#define MOTOR_PWM_RES_BITS 12
#define MOTOR_LEDC_CHANNEL 0             // only used on Arduino-ESP32 core 2.x

#define HAS_MOTOR_PWM (MOTOR_PWM_PIN >= 0 && MOTOR_EN_PIN >= 0)
#define HAS_HALL      (HALL1_PIN >= 0 && HALL2_PIN >= 0 && HALL3_PIN >= 0)

// ------------------------------------------------------------------------------------------
struct MotorTelemetry {
  bool  enabled = false;
  float cmdA    = 0.0f;   // commanded current [A]
  float duty    = 50.0f;  // PWM duty sent to the ESCON [%]
  float hallRpm = NAN;    // wheel speed from the Hall sensors [rpm], signed
  float escRpm  = NAN;    // ESCON "actual speed averaged" [rpm], signed
  float escA    = NAN;    // ESCON "actual current averaged" [A], signed
  int   ready   = -1;     // 1 ready, 0 fault / not ready, -1 not connected
  uint32_t hallErrors = 0;
};

static MotorTelemetry mt;
static void (*motorStatusFn)(const char*) = nullptr;
static float escSpeedZeroV = ESC_ADC_OFFSET_V;
static float escCurZeroV   = ESC_ADC_OFFSET_V;

static void motorMsg(const char* fmt, ...) {
  char b[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  if (motorStatusFn) motorStatusFn(b);
  else Serial.print(b);
}

// ------------------------------------------------------------------------------------------
// Hall sensors: an interrupt on every edge of all three lines. The 3-bit state steps through
// a fixed 6-state cycle; the step direction gives the sign, the edge rate gives the speed.
// 6 * pole_pairs edges per mechanical revolution.
// ------------------------------------------------------------------------------------------
#if HAS_HALL
static DRAM_ATTR const int8_t HALL_IDX[8] = {-1, 0, 2, 1, 4, 5, 3, -1};  // cycle 1,3,2,6,4,5
static portMUX_TYPE hallMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t hallCount = 0;
static volatile int64_t hallLastEdgeUs = 0;
static volatile int8_t  hallPrevIdx = -1;
static volatile uint32_t hallErrCount = 0;

static inline int IRAM_ATTR hallRead(int pin) {
  return pin < 32 ? (int)((REG_READ(GPIO_IN_REG) >> pin) & 1u)
                  : (int)((REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1u);
}

static void IRAM_ATTR hallISR() {
  const int s = hallRead(HALL1_PIN) | (hallRead(HALL2_PIN) << 1) | (hallRead(HALL3_PIN) << 2);
  const int8_t idx = HALL_IDX[s];
  portENTER_CRITICAL_ISR(&hallMux);
  if (idx < 0) {
    hallErrCount++;                               // 000 or 111: wiring or noise problem
  } else if (hallPrevIdx >= 0 && idx != hallPrevIdx) {
    const int d = (idx - hallPrevIdx + 6) % 6;
    if (d == 1)      hallCount++;
    else if (d == 5) hallCount--;
    else             hallErrCount++;              // skipped a state
    hallLastEdgeUs = esp_timer_get_time();
  }
  if (idx >= 0) hallPrevIdx = idx;
  portEXIT_CRITICAL_ISR(&hallMux);
}

// Speed from edge timestamps (not loop time), averaged over >= 1 electrical cycle (6 edges)
// or >= 20 ms, whichever comes first. This removes Hall placement ripple at low speed.
static float hallSpeedRpm() {
  static int32_t refCnt = 0;
  static int64_t refT = 0;
  static float edgesPerS = 0.0f;

  int32_t cnt;
  int64_t tEdge;
  portENTER_CRITICAL(&hallMux);
  cnt = hallCount;
  tEdge = hallLastEdgeUs;
  mt.hallErrors = hallErrCount;
  portEXIT_CRITICAL(&hallMux);

  const int64_t now = esp_timer_get_time();
  if (refT == 0) {                                // first call
    refCnt = cnt;
    refT = tEdge ? tEdge : now;
  } else if (cnt != refCnt) {
    const int64_t dt = tEdge - refT;
    if (abs(cnt - refCnt) >= 6 || dt >= 20000) {
      if (dt > 0 && dt < HALL_STOP_TIMEOUT_US) edgesPerS = (cnt - refCnt) * 1e6f / (float)dt;
      else                                     edgesPerS = 0.0f;  // restarting from standstill
      refCnt = cnt;
      refT = tEdge;
    }
  }

  // Decay towards 0 when edges stop arriving.
  const int64_t since = now - (tEdge ? tEdge : refT);
  if (since > HALL_STOP_TIMEOUT_US) {
    edgesPerS = 0.0f;
    refCnt = cnt;
    refT = now;
  } else if (since > 0 && fabsf(edgesPerS) * since * 1e-6f > 1.0f) {
    edgesPerS = copysignf(1e6f / (float)since, edgesPerS);  // can't be faster than 1 edge / since
  }

  return HALL_DIR_SIGN * edgesPerS * 60.0f / (6.0f * HALL_POLE_PAIRS);
}
#endif

// ------------------------------------------------------------------------------------------
// ESCON command
// ------------------------------------------------------------------------------------------
static float motorCurrentToDuty(float amps) {
  float d = 10.0f + 80.0f * (amps - MOTOR_I_AT_10PCT) / (MOTOR_I_AT_90PCT - MOTOR_I_AT_10PCT);
  if (d < 10.0f) d = 10.0f;   // never 0 % or 100 %: the ESCON flags that as a set-value error
  if (d > 90.0f) d = 90.0f;
  return d;
}

static void motorWriteDuty(float dutyPct) {
#if HAS_MOTOR_PWM
  const uint32_t maxCount = (1u << MOTOR_PWM_RES_BITS) - 1u;
  const uint32_t c = (uint32_t)lroundf(dutyPct * 0.01f * maxCount);
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(MOTOR_PWM_PIN, c);
  #else
    ledcWrite(MOTOR_LEDC_CHANNEL, c);
  #endif
#endif
  mt.duty = dutyPct;
}

// Set the commanded current [A]. Clamped to +/- MOTOR_TEST_MAX_A.
static void motorSetCurrent(float amps) {
  if (!isfinite(amps)) amps = 0.0f;
  if (amps >  MOTOR_TEST_MAX_A) amps =  MOTOR_TEST_MAX_A;
  if (amps < -MOTOR_TEST_MAX_A) amps = -MOTOR_TEST_MAX_A;
  mt.cmdA = amps;
  motorWriteDuty(motorCurrentToDuty(amps));
}

static void motorEnable(bool on) {
#if HAS_MOTOR_PWM
  digitalWrite(MOTOR_EN_PIN, on ? HIGH : LOW);
  mt.enabled = on;
#else
  if (on) motorMsg("# MOTOR: MOTOR_PWM_PIN / MOTOR_EN_PIN not set in config.h, ignoring\n");
  mt.enabled = false;
#endif
}

// Zero the command first, then drop enable.
static void motorStop() {
  motorSetCurrent(0.0f);
  motorEnable(false);
}

// ------------------------------------------------------------------------------------------
static float adcVolts(int pin, int n) {
  uint32_t s = 0;
  for (int i = 0; i < n; i++) s += analogReadMilliVolts(pin);
  return s * 0.001f / n;
}

static float zeroAdc(int pin, const char* name) {
  const float v = adcVolts(pin, 64);
  if (fabsf(v - ESC_ADC_OFFSET_V) < 0.25f) {
    motorMsg("# MOTOR: %s zero = %.3f V\n", name, v);
    return v;
  }
  motorMsg("# MOTOR: WARNING %s reads %.3f V at rest (expected ~%.2f V). Check level shifter. "
           "Using nominal.\n", name, v, ESC_ADC_OFFSET_V);
  return ESC_ADC_OFFSET_V;
}

// Call once at the top of setup(), before WiFi, so the motor is disabled as early as possible.
// The wheel should be still at boot, so both analog outputs are at their zero.
static void motorBegin(void (*statusFn)(const char*)) {
  motorStatusFn = statusFn;

#if HAS_MOTOR_PWM
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, LOW);
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PWM_PIN, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
  #else
    ledcSetup(MOTOR_LEDC_CHANNEL, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcAttachPin(MOTOR_PWM_PIN, MOTOR_LEDC_CHANNEL);
  #endif
  motorStop();
  motorMsg("# MOTOR: PWM on GPIO%d @ %d Hz, enable on GPIO%d (disabled)\n",
           MOTOR_PWM_PIN, MOTOR_PWM_FREQ_HZ, MOTOR_EN_PIN);
#else
  motorMsg("# MOTOR: no PWM/enable pins set, motor command disabled\n");
#endif

#if MOTOR_READY_PIN >= 0
  pinMode(MOTOR_READY_PIN, INPUT);   // external 4.7k pull-up to 3.3 V
#endif

#if HAS_HALL
  pinMode(HALL1_PIN, INPUT);         // external 68k/100k dividers, no internal pulls
  pinMode(HALL2_PIN, INPUT);
  pinMode(HALL3_PIN, INPUT);
  hallPrevIdx = HALL_IDX[digitalRead(HALL1_PIN) | (digitalRead(HALL2_PIN) << 1) |
                         (digitalRead(HALL3_PIN) << 2)];   // initial state
  attachInterrupt(digitalPinToInterrupt(HALL1_PIN), hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL2_PIN), hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL3_PIN), hallISR, CHANGE);
  motorMsg("# MOTOR: Halls on GPIO%d/%d/%d, %d pole pairs\n",
           HALL1_PIN, HALL2_PIN, HALL3_PIN, HALL_POLE_PAIRS);
#endif

#if ESC_SPEED_ADC_PIN >= 0
  escSpeedZeroV = zeroAdc(ESC_SPEED_ADC_PIN, "AnOUT1 speed");
#endif
#if ESC_CURRENT_ADC_PIN >= 0
  escCurZeroV = zeroAdc(ESC_CURRENT_ADC_PIN, "AnOUT2 current");
#endif
}

// Disable the motor if the command link has gone quiet.
static void motorWatchdog(bool linkAlive) {
  if (mt.enabled && !linkAlive) {
    motorStop();
    motorMsg("# MOTOR: command link lost, motor disabled\n");
  }
}

// Call once per control loop. Reads all feedback into mt and applies test-mode limits.
static void motorUpdate() {
#if HAS_HALL
  mt.hallRpm = hallSpeedRpm();
#endif
#if ESC_SPEED_ADC_PIN >= 0
  mt.escRpm = (adcVolts(ESC_SPEED_ADC_PIN, ESC_ADC_SAMPLES) - escSpeedZeroV) / ESC_ADC_GAIN
              * ESC_SPEED_RPM_PER_V;
#endif
#if ESC_CURRENT_ADC_PIN >= 0
  mt.escA = (adcVolts(ESC_CURRENT_ADC_PIN, ESC_ADC_SAMPLES) - escCurZeroV) / ESC_ADC_GAIN
            * ESC_CURRENT_A_PER_V;
#endif
#if MOTOR_READY_PIN >= 0
  const int r = (digitalRead(MOTOR_READY_PIN) == READY_ACTIVE_LEVEL) ? 1 : 0;
  if (r != mt.ready) motorMsg(r ? "# MOTOR: ESCON ready\n" : "# MOTOR: ESCON NOT ready (fault?)\n");
  mt.ready = r;
#endif

  // Test-mode speed limit: in current mode a constant current keeps accelerating the wheel,
  // so zero any command that would push it further past the limit (braking is still allowed).
  const float spd = isfinite(mt.hallRpm) ? mt.hallRpm : mt.escRpm;
  if (mt.enabled && isfinite(spd) && fabsf(spd) > MOTOR_TEST_MAX_RPM && mt.cmdA * spd > 0.0f) {
    motorSetCurrent(0.0f);
    motorMsg("# MOTOR: |speed| %.0f rpm > %.0f rpm limit, command set to 0 A\n",
             spd, MOTOR_TEST_MAX_RPM);
  }
}
