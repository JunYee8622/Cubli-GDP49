// cubli_motor_test_wifi.ino — reaction-wheel MOTOR test (no IMU, no balancing) with WiFi OTA + UDP.
//
// Same ESCON interface as edge_balance.ino (balance_config.h / config.h):
//   current command = PWM on MOTOR_PWM_PIN (10 % = -IA_MAX_ESCON, 50 % = 0 A, 90 % = +IA_MAX_ESCON)
//   enable          = MOTOR_EN_PIN
//   wheel speed     = ESCON AnOUT1 on ESC_SPEED_ADC_PIN, scaled by SPEED_V_LO/HI, SPEED_RPM_FS
//
// CSV has the SAME 15 columns as edge_balance (IMU columns = 0) plus ref_rpm at the end, so the
// original log_udp.py works unchanged.
//
// Modes (type in log_udp.py + Enter, or in the Serial Monitor with "newline" line ending):
//   a / g    profile: 0 -> +TARGET_RPM (linear ramp), FAST STOP the instant the peak is reached
//            (reference falls to 0 in STOP_TIME_S, motor driven with reverse current), repeat in the
//            other direction. This is the "reverse current to brake" test.
//   c 0.3    OPEN-LOOP current test: applies 0.3 A, after 1 s prints measured vs ideal speed change.
//   r 500    CLOSED-LOOP speed hold at 500 rpm (slewed at SLEW_RPM_S). "r 0" brings it back to 0.
//   x / s    stop (brake per BRAKE_A in balance_config.h, then motor off)
//   l        toggle profile looping       v   print raw speed-signal voltage and scaling in use
//   (Enter)  marker
//
// CSV: t_us,ax,ay,az,gx,gy,gz,temp,marker,ang_comp,ang_kf,en,th_err,rpm,cmd_A,ref_rpm
//   en = state: 0 off, 1 profile, 2 braking, 3 open-loop current, 4 speed hold
//
// PHYSICS OF THE STOP: the wheel decelerates at Km*I/J. Stopping 2000 rpm in 0.5 s needs ~6.6 A, the
// ESCON Module 24/2 peak is 6 A (< 4 s), so ~0.55-0.6 s is the real limit at 2000 rpm. The firmware therefore
// makes the stop reference fall as fast as STOP_TIME_S asks OR as fast as the I_STOP_MAX clamp can deliver
// (whichever is slower), and prints which one it used, so measured rpm follows the reference.
// For a stop in <= 0.5 s with margin, lower TARGET_RPM to ~1500.
//
// SAFETY NET: if the commanded current does not change the wheel speed as Km/J predict (ESCON fault, e.g.
// overvoltage trip from braking energy), the test disarms and says so instead of carrying on.

#define OTA 1
#define UDP_STREAM 1        // requires OTA 1

#include <Arduino.h>
#include <stdarg.h>
#include "config.h"
#include "balance_config.h"

#if OTA
  #include <WiFi.h>
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
  #ifndef WIFI_SSID
    #define WIFI_SSID     "Jun Yee"
  #endif
  #ifndef WIFI_PASSWORD
    #define WIFI_PASSWORD "12345678900"
  #endif
  #ifndef OTA_HOSTNAME
    #define OTA_HOSTNAME  "cube-esp32"
  #endif
  #define WIFI_TIMEOUT_MS     10000UL
  #define UDP_PORT            4210
  #define UDP_PEER_TIMEOUT_MS 5000UL
  #define UDP_FLUSH_MS        20UL
  #define UDP_MAX_PAYLOAD     1400
  volatile bool otaActive = false;
  bool wifiOk = false;
#endif

// ======================= TEST SETTINGS (edit here) =======================
// Motor constant used by this test only (feed-forward, stop planning, torque check).
// maxon EC 45 flat 591477 (24 V): speed constant 265 rpm/V -> Km = 9.549/265 = 0.0360 Nm/A.
#define KM_FF 0.0360f

// Profile
static const float TARGET_RPM      = 2000.0f;  // try 1500 for a stop that really fits in 0.5 s
static const float RAMP_TIME_S     = 2.5f;     // 0 -> TARGET_RPM, linear
static const float HOLD_TOP_S      = 0.0f;     // 0 = brake the instant the peak is reached
static const float REST_S          = 3.0f;     // pause at 0 rpm between directions (ESCON/motor recover)
static bool        LOOP_SEQUENCE   = true;

// Stop. 1 = actively brake with reverse current (energy flows back to the supply; a supply that cannot
// absorb it can make the ESCON trip on overvoltage -> use a battery). 0 = cut current and coast.
#define STOP_ACTIVE 1
static const float STOP_TIME_S       = 0.5f;   // requested time for peak -> 0 rpm. 0 = true step to zero
static const float I_STOP_MAX        = 6.0f;   // A. ESCON Module 24/2: 6 A peak for < 4 s. In ESCON Studio the
                                               // max output current limit must be >= this, or it is clipped.
static const float STOP_DECEL_MARGIN = 0.92f;  // plan the stop reference at 92 % of what the clamp can do
static const float STOP_TIMEOUT_S    = STOP_ACTIVE ? 3.0f : 30.0f;

// Limits
static const float I_TEST_MAX      = 2.0f;     // A  clamp while ramping up / holding speed (ESCON continuous)
static const float WHEEL_ABORT_RPM = 2600.0f;  // hard overspeed cut-off (profile / speed hold)
static const float MAN_MAX_RPM     = 1500.0f;  // open-loop current test stops here
static const float MAN_TIMEOUT_S   = 6.0f;     // open-loop current test duration limit
static const float SLEW_RPM_S      = 500.0f;   // speed-hold setpoint slew rate

// Speed PI (output in amps). wn ~ 3.6 rad/s, damping ~ 0.7 with b = Km/J.
static const float KP_A_PER_RPM    = 0.0080f;
static const float KI_A_PER_RPM_S  = 0.0200f;
// =========================================================================

static const float RPM2RADS = 0.10471975512f;
static const char CSV_HEADER[] =
    "t_us,ax,ay,az,gx,gy,gz,temp,marker,ang_comp,ang_kf,en,th_err,rpm,cmd_A,ref_rpm\n";

#ifndef ESTOP_PIN
  #define ESTOP_PIN -1
#endif
#ifndef REQUIRE_LINK
  #define REQUIRE_LINK 0
#endif

// ---------------- ESCON pins ----------------
const int PIN_CURRENT = MOTOR_PWM_PIN;
const int PIN_ENABLE  = MOTOR_EN_PIN;
const int PIN_SPEED   = ESC_SPEED_ADC_PIN;

const uint32_t PWM_FREQ = 1000;
const uint8_t  PWM_BITS = 12;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH() ledcAttach(PIN_CURRENT, PWM_FREQ, PWM_BITS)
  #define PWM_WRITE(d) ledcWrite(PIN_CURRENT, (d))
#else
  const int PWM_CH = 0;
  #define PWM_ATTACH() do { ledcSetup(PWM_CH, PWM_FREQ, PWM_BITS); ledcAttachPin(PIN_CURRENT, PWM_CH); } while (0)
  #define PWM_WRITE(d) ledcWrite(PWM_CH, (d))
#endif

// ---------------- state ----------------
enum Phase { P_RAMP_POS, P_HOLD_POS, P_STOP_POS, P_REST_1,
             P_RAMP_NEG, P_HOLD_NEG, P_STOP_NEG, P_REST_2 };
enum { ST_DISARMED = 0, ST_RUN = 1, ST_BRAKE = 2, ST_MAN_I = 3, ST_MAN_SPD = 4 };

int      state = ST_DISARMED;
Phase    phase = P_RAMP_POS;
uint32_t phaseStartMs = 0, brakeStartMs = 0, modeStartMs = 0;
uint32_t marker = 0, overruns = 0;
float    lastIa = 0.0f, lastRpm = 0.0f, lastRef = 0.0f, integ = 0.0f;
float    speedBiasV = 0.0f;
float    stopV0 = 0.0f;               // speed at the moment the stop phase starts
float    stopDecel = 0.0f;            // rpm/s actually used for the stop reference (0 = step)
bool     speedSignalOk = false;
bool     armedByLaptop = false;

// manual modes
float    manualA = 0.0f, manualTarget = 0.0f, manualRef = 0.0f;
float    chkInt = 0.0f, chkRpm0 = 0.0f;
bool     chkDone = false;

// torque-response watchdog
float    tlInt = 0.0f, tlRpm0 = 0.0f;
uint32_t tlStartMs = 0;

static void statusf(const char* fmt, ...);
static void handleCommand(char c, bool fromLaptop);
static void handleLine(const char* s, bool fromLaptop);

// =====================================================================================
//  ESCON interface (identical to edge_balance.ino)
// =====================================================================================
static void writeCurrent(float ia) {
  if (ia >  IA_MAX_ESCON) ia =  IA_MAX_ESCON;
  if (ia < -IA_MAX_ESCON) ia = -IA_MAX_ESCON;
  const float duty = 0.5f + ia * (0.8f / (2.0f * IA_MAX_ESCON));
  PWM_WRITE((uint32_t)(duty * 4095.0f));
}

static void motorOff() {
  writeCurrent(0);
  digitalWrite(PIN_ENABLE, LOW);
  lastIa = 0.0f;
}

static float speedVolts(int n) {
  uint32_t s = 0;
  for (int i = 0; i < n; i++) s += analogReadMilliVolts(PIN_SPEED);
  return s * 0.001f / n;
}

static float readSpeedRpm() {
  const float vMid = 0.5f * (SPEED_V_LO + SPEED_V_HI);
  const float v = speedVolts(4) - speedBiasV;
  const float raw = SPEED_SIGN * (v - vMid) * (2.0f * SPEED_RPM_FS / (SPEED_V_HI - SPEED_V_LO));
  static float filt = 0.0f;
  filt += 0.17f * (raw - filt);
  return filt;
}

static void calibrateSpeed() {      // wheel must be still, ESCON powered. Blocks ~0.5 s.
  const float vMid = 0.5f * (SPEED_V_LO + SPEED_V_HI);
  float sum = 0;
  for (int i = 0; i < 50; i++) { sum += speedVolts(1); delay(10); }
  const float rest = sum / 50.0f;
  speedBiasV = rest - vMid;
  speedSignalOk = fabsf(speedBiasV) <= SPEED_BIAS_MAX;
  if (speedSignalOk) statusf("# TEST: speed signal at rest %.3f V (expected %.2f V) OK\n", rest, vMid);
  else statusf("# TEST: SPEED SIGNAL BAD: %.3f V at rest, expected %.2f V. ESCON powered? "
               "AnOUT1 scaling as in balance_config.h?\n", rest, vMid);
}

// =====================================================================================
//  Start / stop
// =====================================================================================
// Actively slow the wheel (BRAKE_A from balance_config.h), then switch off. BRAKE_A = 0 -> coast.
static void startBrake(const char* why) {
  if (BRAKE_A <= 0.0f) { motorOff(); state = ST_DISARMED; statusf("# TEST: %s -> motor off (coasting)\n", why); return; }
  digitalWrite(PIN_ENABLE, HIGH);
  brakeStartMs = millis();
  state = ST_BRAKE;
  statusf("# TEST: %s -> braking wheel\n", why);
}

// brake=true: slow the wheel first. false: motor off immediately (OTA).
static void disarm(const char* why, bool brake = true) {
  if (state == ST_DISARMED || state == ST_BRAKE) return;
  if (brake && fabsf(lastRpm) > BRAKE_DONE_RPM) startBrake(why);
  else { motorOff(); state = ST_DISARMED; statusf("# TEST: DISARMED (%s)\n", why); }
}

static void torqueCheckReset() { tlInt = 0.0f; tlRpm0 = lastRpm; tlStartMs = millis(); }

// Common start: checks, speed-signal zero, ESCON enable. Returns true if ready to run.
static bool prepare(bool fromLaptop) {
#if ESTOP_PIN >= 0
  if (digitalRead(ESTOP_PIN) == LOW) { statusf("# TEST: REFUSED: E-stop pressed\n"); return false; }
#endif
  statusf("# TEST: zeroing speed signal, keep the wheel still...\n");
  calibrateSpeed();
  if (!speedSignalOk) { statusf("# TEST: REFUSED (speed signal)\n"); return false; }
  armedByLaptop = fromLaptop;
  integ = 0.0f;
  writeCurrent(0);
  digitalWrite(PIN_ENABLE, HIGH);
  modeStartMs = millis();
  torqueCheckReset();
  return true;
}

static void enterPhase(Phase p) {
  static const char* names[] = {"RAMP +", "HOLD +", "STOP +", "REST", "RAMP -", "HOLD -", "STOP -", "REST"};
  phase = p;
  phaseStartMs = millis();
  integ = 0.0f;
  if (p == P_STOP_POS || p == P_STOP_NEG) {
    stopV0 = lastRpm;
    stopDecel = 0.0f;
    if (STOP_TIME_S > 0.0f && fabsf(stopV0) > 50.0f) {
      const float need = fabsf(stopV0) / STOP_TIME_S;                                        // rpm/s wanted
      const float cap  = STOP_DECEL_MARGIN * I_STOP_MAX * KM_FF / WHEEL_INERTIA * 9.549f;    // rpm/s the clamp gives
      stopDecel = fminf(need, cap);
      statusf("# TEST: stop from %+.0f rpm: %.2f s wanted needs %.0f rpm/s (~%.1f A), clamp %.1f A allows %.0f rpm/s "
              "-> using %.0f rpm/s, ~%.2f s\n",
              stopV0, STOP_TIME_S, need, need * RPM2RADS * WHEEL_INERTIA / KM_FF, I_STOP_MAX, cap,
              stopDecel, fabsf(stopV0) / stopDecel);
    }
  }
  statusf("# TEST: phase %s\n", names[p]);
}

static void startProfile(bool fromLaptop) {
  if (state != ST_DISARMED) { statusf("# TEST: stop first (x)\n"); return; }
  if (!prepare(fromLaptop)) return;
  state = ST_RUN;
  enterPhase(P_RAMP_POS);
  statusf("# TEST: PROFILE running: +/-%.0f rpm, ramp %.1f s (clamp %.1f A), stop %s in %.2f s (clamp %.1f A)\n",
          TARGET_RPM, RAMP_TIME_S, I_TEST_MAX, STOP_ACTIVE ? "ACTIVE" : "COAST", STOP_TIME_S, I_STOP_MAX);
}

static void startCurrent(float amps, bool fromLaptop) {
  if (state == ST_DISARMED) { if (!prepare(fromLaptop)) return; }
  else if (state != ST_MAN_I) { statusf("# TEST: stop first (x)\n"); return; }
  manualA = constrain(amps, -I_TEST_MAX, I_TEST_MAX);
  state = ST_MAN_I;
  modeStartMs = millis();
  chkInt = 0.0f; chkRpm0 = lastRpm; chkDone = false;
  statusf("# TEST: open-loop current %+.2f A for up to %.0f s (stops at %.0f rpm)\n",
          manualA, MAN_TIMEOUT_S, MAN_MAX_RPM);
}

static void startSpeed(float rpm, bool fromLaptop) {
  const bool fresh = (state == ST_DISARMED);
  if (fresh) { if (!prepare(fromLaptop)) return; }
  else if (state != ST_MAN_SPD) { statusf("# TEST: stop first (x)\n"); return; }
  manualTarget = constrain(rpm, -TARGET_RPM, TARGET_RPM);
  if (fresh) manualRef = lastRpm;
  state = ST_MAN_SPD;
  modeStartMs = millis();
  statusf("# TEST: speed hold -> %+.0f rpm (slew %.0f rpm/s, clamp %.1f A)\n", manualTarget, SLEW_RPM_S, I_TEST_MAX);
}

// =====================================================================================
//  Profile + control
// =====================================================================================
static float reference(float t, float &iLim) {
  iLim = I_TEST_MAX;
  const float r = fminf(t / RAMP_TIME_S, 1.0f);
  switch (phase) {
    case P_RAMP_POS: return  TARGET_RPM * r;
    case P_HOLD_POS: return  TARGET_RPM;
    case P_RAMP_NEG: return -TARGET_RPM * r;
    case P_HOLD_NEG: return -TARGET_RPM;
    case P_STOP_POS:
    case P_STOP_NEG: {
      iLim = I_STOP_MAX;
      if (stopDecel <= 0.0f) return 0.0f;                           // true step to zero
      const float v = fabsf(stopV0) - stopDecel * t;                // decel ramp from where we were
      return (v > 0.0f) ? copysignf(v, stopV0) : 0.0f;
    }
    default: return 0.0f;
  }
}

// Feed-forward current to follow a speed ramp: Km*i = J*dw/dt
static float rampFF(float rpmPerS) {
  return WHEEL_INERTIA * rpmPerS * RPM2RADS / KM_FF;
}

static void nextPhase(float t) {
  const bool still = fabsf(lastRpm) < 20.0f;
  switch (phase) {
    case P_RAMP_POS: if (t >= RAMP_TIME_S)    enterPhase(P_HOLD_POS); break;
    case P_HOLD_POS: if (t >= HOLD_TOP_S)     enterPhase(P_STOP_POS); break;
    case P_STOP_POS: if ((still && t > 0.05f) || t >= STOP_TIMEOUT_S) enterPhase(P_REST_1); break;
    case P_REST_1:   if (t >= REST_S)         enterPhase(P_RAMP_NEG); break;
    case P_RAMP_NEG: if (t >= RAMP_TIME_S)    enterPhase(P_HOLD_NEG); break;
    case P_HOLD_NEG: if (t >= HOLD_TOP_S)     enterPhase(P_STOP_NEG); break;
    case P_STOP_NEG: if ((still && t > 0.05f) || t >= STOP_TIMEOUT_S) enterPhase(P_REST_2); break;
    case P_REST_2:
      if (t >= REST_S) {
        if (LOOP_SEQUENCE) enterPhase(P_RAMP_POS);
        else disarm("sequence done");
      }
      break;
  }
}

// PI + feed-forward, current clamp iLim, with anti-windup.
static float speedPI(float ref, float ff, float iLim, float dt) {
  const float err = ref - lastRpm;
  const float u0 = ff + KP_A_PER_RPM * err + KI_A_PER_RPM_S * integ;
  const float uSat = constrain(u0, -iLim, iLim);
  if (u0 == uSat || ((u0 > 0.0f) != (err > 0.0f))) integ += err * dt;
  return constrain(ff + KP_A_PER_RPM * err + KI_A_PER_RPM_S * integ, -iLim, iLim);
}

// True if the wheel is not following a clearly non-zero reference (ESCON fault / wrong sign / no motor).
static bool noResponse(float ref, float t) {
  if (t < 1.5f || fabsf(ref) < 300.0f) return false;
  return (lastRpm * ref < 0.0f) || fabsf(lastRpm) < 0.25f * fabsf(ref);
}

static void faultNoResponse(float ref) {
  motorOff();
  state = ST_DISARMED;
  statusf("# TEST: NO / WRONG-SIGN RESPONSE (ref %+.0f, got %+.0f rpm). Check ESCON LED/fault (an "
          "overvoltage trip needs a 24 V power cycle), ESCON current scaling vs IA_MAX_ESCON, "
          "SPEED_SIGN. DISARMED.\n", ref, lastRpm);
}

// Every 300 ms: did the applied current change the wheel speed by at least a quarter of Km*i*dt/J?
// If not, the ESCON has stopped delivering torque (fault, overvoltage trip, disabled).
static bool torqueLost(float u, float dt) {
  tlInt += u * dt;
  if (millis() - tlStartMs < 300) return false;
  const float expRpm = KM_FF * tlInt / WHEEL_INERTIA * 9.549f;
  const float gotRpm = lastRpm - tlRpm0;
  const bool bad = fabsf(expRpm) > 200.0f && gotRpm * expRpm < 0.25f * expRpm * expRpm;
  if (bad)
    statusf("# TEST: ESCON NOT DELIVERING TORQUE: current should have changed the wheel by %+.0f rpm in 0.3 s, it changed "
            "%+.0f. Check the ESCON LED (likely overvoltage trip from braking energy: use a battery / lower I_STOP_MAX), "
            "power-cycle the 24 V and re-arm. DISARMED.\n", expRpm, gotRpm);
  tlInt = 0.0f; tlRpm0 = lastRpm; tlStartMs = millis();
  return bad;
}

static void controlUpdate(float dt) {
  if (state == ST_DISARMED) { lastIa = 0.0f; lastRef = 0.0f; return; }

  if (state == ST_BRAKE) {
    lastRef = 0.0f;
    if (fabsf(lastRpm) < BRAKE_DONE_RPM || millis() - brakeStartMs > (uint32_t)BRAKE_TIMEOUT_MS) {
      motorOff();
      state = ST_DISARMED;
      statusf("# TEST: wheel stopped, motor off, DISARMED\n");
      return;
    }
    const float ia = (lastRpm > 0.0f) ? -BRAKE_A : BRAKE_A;
    writeCurrent(ia);
    lastIa = ia;
    return;
  }

  const float tMode = (millis() - modeStartMs) * 1e-3f;

  // ---------------- open-loop current test ----------------
  if (state == ST_MAN_I) {
    lastRef = 0.0f;
    if (fabsf(lastRpm) > MAN_MAX_RPM) { startBrake("reached MAN_MAX_RPM"); return; }
    if (tMode > MAN_TIMEOUT_S)        { disarm("current test time limit"); return; }
    writeCurrent(manualA);
    lastIa = manualA;
    chkInt += manualA * dt;
    if (!chkDone && tMode >= 1.0f) {
      chkDone = true;
      const float expRpm = KM_FF * chkInt / WHEEL_INERTIA * 9.549f;
      const float gotRpm = lastRpm - chkRpm0;
      const float ratio = (fabsf(expRpm) > 1.0f) ? gotRpm / expRpm : 0.0f;
      statusf("# TEST: %+.2f A for %.1f s: wheel changed %+.0f rpm, ideal (Km, J) %+.0f rpm, ratio %.2f\n",
              manualA, tMode, gotRpm, expRpm, ratio);
      if (fabsf(manualA) > 0.05f) {
        if (gotRpm * expRpm < 0.0f)
          statusf("# TEST: WRONG DIRECTION: positive current gave negative rpm -> flip SPEED_SIGN, or the speed-signal wiring\n");
        else if (ratio < 0.5f)
          statusf("# TEST: ratio < 0.5: ESCON delivers less current than commanded (check ESCON Studio PWM scaling = +/-%.1f A) "
                  "or speed scale is off or friction is large\n", IA_MAX_ESCON);
        else if (ratio > 1.3f)
          statusf("# TEST: ratio > 1.3: wheel speeds up faster than Km/J predict (check WHEEL_INERTIA, KM_FF, speed scale)\n");
        else
          statusf("# TEST: ratio is plausible (0.5..1.3 is normal: friction and back-EMF eat a part)\n");
      }
    }
    return;
  }

  // ---------------- speed-hold / profile ----------------
  if (fabsf(lastRpm) > WHEEL_ABORT_RPM) {
    statusf("# TEST: OVERSPEED %.0f rpm\n", lastRpm);
    startBrake("overspeed");
    return;
  }

  if (state == ST_MAN_SPD) {
    // slew the reference towards the target
    const float step = SLEW_RPM_S * dt;
    const float diff = manualTarget - manualRef;
    const bool slewing = fabsf(diff) > step;
    manualRef += slewing ? copysignf(step, diff) : diff;
    lastRef = manualRef;
    if (noResponse(manualRef, tMode)) { faultNoResponse(manualRef); return; }
    const float ff = slewing ? copysignf(rampFF(SLEW_RPM_S), diff) : 0.0f;
    const float u = speedPI(manualRef, ff, I_TEST_MAX, dt);
    writeCurrent(u);
    lastIa = u;
    if (torqueLost(u, dt)) { motorOff(); state = ST_DISARMED; }
    return;
  }

  // ST_RUN: profile
  const float t = (millis() - phaseStartMs) * 1e-3f;
  float iLim;
  const float ref = reference(t, iLim);
  lastRef = ref;
  if ((phase == P_RAMP_POS || phase == P_RAMP_NEG) && noResponse(ref, t)) { faultNoResponse(ref); return; }

  float u;
#if !STOP_ACTIVE
  if (phase == P_STOP_POS || phase == P_STOP_NEG) { u = 0.0f; integ = 0.0f; }
  else
#endif
  {
    float ff = 0.0f;
    if (phase == P_RAMP_POS) ff =  rampFF(TARGET_RPM / RAMP_TIME_S);
    if (phase == P_RAMP_NEG) ff = -rampFF(TARGET_RPM / RAMP_TIME_S);
    // fast stop: torque needed to decelerate along the reference ramp (opposite to the speed)
    if ((phase == P_STOP_POS || phase == P_STOP_NEG) && stopDecel > 0.0f && ref != 0.0f)
      ff = -copysignf(rampFF(stopDecel), ref);
    u = speedPI(ref, ff, iLim, dt);
  }
  writeCurrent(u);
  lastIa = u;
  if (torqueLost(u, dt)) { motorOff(); state = ST_DISARMED; return; }
  nextPhase(t);
}

// =====================================================================================
//  UDP streaming (as edge_balance / imu_logger)
// =====================================================================================
#if OTA && UDP_STREAM
WiFiUDP udp;
IPAddress peerIP;
uint16_t peerPort = 0;
uint32_t peerLastSeen = 0;
static char udpBuf[UDP_MAX_PAYLOAD];
static size_t udpLen = 0;
static uint32_t udpLastFlush = 0;

static bool peerActive() {
  return peerPort != 0 && (millis() - peerLastSeen) < UDP_PEER_TIMEOUT_MS;
}
static void udpFlush() {
  if (udpLen > 0 && peerActive()) {
    udp.beginPacket(peerIP, peerPort);
    udp.write((const uint8_t*)udpBuf, udpLen);
    udp.endPacket();
  }
  udpLen = 0;
  udpLastFlush = millis();
}
static void udpQueue(const char* s, size_t n) {
  if (!peerActive() || n > sizeof(udpBuf)) return;
  if (udpLen + n > sizeof(udpBuf)) udpFlush();
  memcpy(udpBuf + udpLen, s, n);
  udpLen += n;
}
static void udpPoll() {
  while (udp.parsePacket() > 0) {
    char msg[32];
    int n = udp.read(msg, sizeof(msg) - 1);
    if (n < 0) n = 0;
    msg[n] = '\0';

    const IPAddress ip = udp.remoteIP();
    const uint16_t port = udp.remotePort();
    const bool samePeer = (ip == peerIP && port == peerPort);
    if (peerActive() && !samePeer) continue;

    const bool isNew = !peerActive();
    peerIP = ip; peerPort = port; peerLastSeen = millis();
    if (isNew) {
      udpLen = 0;
      static const char start[] = "# cubli_motor_test: a = profile, c <A> = current test, r <rpm> = speed hold, x = stop, v = info\n";
      udpQueue(start, sizeof(start) - 1);
      udpQueue(CSV_HEADER, sizeof(CSV_HEADER) - 1);
      udpFlush();
      Serial.printf("# UDP: streaming to %s:%u\n", ip.toString().c_str(), (unsigned)port);
    }
    if (strcmp(msg, "hello") == 0) continue;
    if (strcmp(msg, "h") == 0) { udpQueue(CSV_HEADER, sizeof(CSV_HEADER) - 1); continue; }
    if (strcmp(msg, "stop") == 0) { handleCommand('x', true); continue; }
    if (n >= 1) handleLine(msg, true);
  }
}
#endif

static void emitStatus(const char* line) {
  Serial.print(line);
#if OTA && UDP_STREAM
  udpQueue(line, strlen(line));
#endif
}
static void statusf(const char* fmt, ...) {
  char b[400];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  emitStatus(b);
}
static void emitCsv(const char* s, size_t n) {
#if OTA && UDP_STREAM
  if (peerActive()) { udpQueue(s, n); return; }
#endif
  Serial.write((const uint8_t*)s, n);
}

static void handleCommand(char c, bool fromLaptop) {
  switch (c) {
    case 'a': case 'A': case 'g': case 'G': startProfile(fromLaptop); break;
    case 'x': case 'X': case 's': case 'S': disarm("user"); break;
    case 'l': case 'L':
      LOOP_SEQUENCE = !LOOP_SEQUENCE;
      statusf("# TEST: LOOP_SEQUENCE = %s\n", LOOP_SEQUENCE ? "on" : "off");
      break;
    case 'v': case 'V': {
      const float v = speedVolts(16);
      statusf("# INFO: speed pin %.3f V (rest-zero %.3f V), %.0f rpm | scale %.0f rpm/V (V %.2f..%.2f = +/-%.0f rpm) | "
              "current map +/-%.1f A | Km(test) %.4f Nm/A, J %.3e kg m^2 | stop %.2f s, clamp %.1f A\n",
              v, speedBiasV, lastRpm, 2.0f * SPEED_RPM_FS / (SPEED_V_HI - SPEED_V_LO),
              SPEED_V_LO, SPEED_V_HI, SPEED_RPM_FS, IA_MAX_ESCON, KM_FF, WHEEL_INERTIA,
              STOP_TIME_S, I_STOP_MAX);
      break;
    }
    case 'm': marker++; break;
    default: break;
  }
}

// Lines like "c 0.3" / "r 500", or a single command character.
static void handleLine(const char* s, bool fromLaptop) {
  while (*s == ' ') s++;
  if (!*s) return;
  if (*s == 'c' || *s == 'C') { startCurrent(atof(s + 1), fromLaptop); return; }
  if (*s == 'r' || *s == 'R') { startSpeed(atof(s + 1), fromLaptop); return; }
  handleCommand(*s, fromLaptop);
}

// =====================================================================================
#if OTA
static void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) delay(100);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("# WiFi: connection FAILED to \"%s\" (status %d), continuing without OTA/UDP\n",
                  WIFI_SSID, (int)WiFi.status());
    return;
  }
  Serial.print("# WiFi: connected, IP ");
  Serial.println(WiFi.localIP());
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA
    .onStart([]() { disarm("OTA update", false); otaActive = true; Serial.println("# OTA: update starting"); })
    .onEnd([]() { Serial.println("# OTA: update complete, rebooting"); })
    .onError([](ota_error_t error) { otaActive = false; Serial.printf("# OTA: error %u\n", (unsigned)error); });
  ArduinoOTA.begin();
#if UDP_STREAM
  udp.begin(UDP_PORT);
  Serial.printf("# UDP: listening on port %d (run log_udp.py)\n", UDP_PORT);
#endif
  wifiOk = true;
}
#endif

void setup() {
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);          // motor safe before anything else
  PWM_ATTACH();
  writeCurrent(0);
  analogReadResolution(12);
#if ESTOP_PIN >= 0
  pinMode(ESTOP_PIN, INPUT_PULLUP);
#endif

  Serial.begin(SERIAL_BAUD);
  delay(200);
#if OTA
  setupOTA();
#endif
  Serial.println("# cubli_motor_test ready. a = profile, c <A> = current test, r <rpm> = speed hold, x = stop, v = info");
  Serial.print(CSV_HEADER);
  calibrateSpeed();
}

void loop() {
#if OTA
  ArduinoOTA.handle();
  if (otaActive) return;
#endif

  static const uint32_t period = 1000000UL / LOOP_HZ;
  static uint32_t next = micros();
  static uint32_t last = micros();
  static uint32_t lastReport = millis();
  static char sbuf[32];
  static int sl = 0;

  while (Serial.available()) {                 // serial commands are line based (end with Enter)
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (sl > 0) { sbuf[sl] = '\0'; handleLine(sbuf, false); sl = 0; }
    } else if (sl < (int)sizeof(sbuf) - 1) {
      sbuf[sl++] = ch;
    }
  }

  uint32_t now = micros();
  if ((int32_t)(now - next) < 0) return;
  if ((int32_t)(now - next) > (int32_t)period) {
    overruns++;
    if ((int32_t)(now - next) > (int32_t)(10 * period)) next = now;
  }
  next += period;

#if OTA && UDP_STREAM
  if (wifiOk) udpPoll();
  if (REQUIRE_LINK && armedByLaptop && state != ST_DISARMED &&
      !(peerActive() && millis() - peerLastSeen < 2500)) {
    disarm("laptop link lost");
  }
#endif
#if ESTOP_PIN >= 0
  if (digitalRead(ESTOP_PIN) == LOW) disarm("E-STOP button");
#endif

  const float dt = (now - last) * 1e-6f;
  last = now;

  lastRpm = readSpeedRpm();
  controlUpdate(dt);

  char buf[160];
  int n = snprintf(buf, sizeof(buf), "%lu,0,0,0,0,0,0,0,%lu,0,0,%d,0,%.0f,%.3f,%.0f\n",
                   (unsigned long)now, (unsigned long)marker, state, lastRpm, lastIa, lastRef);
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  emitCsv(buf, n);

  if (millis() - lastReport > 5000) {
    lastReport = millis();
    statusf("# overruns=%lu  state=%d  rpm=%.0f  cmd=%.2fA\n",
            (unsigned long)overruns, state, lastRpm, lastIa);
  }

#if OTA && UDP_STREAM
  if (millis() - udpLastFlush >= UDP_FLUSH_MS) udpFlush();
#endif
}
