// edge_balance.ino — 1D edge balancing of the reaction-wheel cube (one motor on the ESCON M3 slot).
//
// Built from imu_logger.ino: same IMU read, calibration (config.h), Kalman filter (filters.h),
// WiFi OTA upload and UDP streaming to log_udp.py. The speed-test PI loop is replaced by an LQR
// balance controller that runs every IMU sample (LOOP_HZ, 500 Hz).
//
// Control law (balance_config.h holds the gains and limits):
//   theta      = TILT_SIGN * (ang_kf - theta_eq)          [rad]
//   theta_dot  = TILT_SIGN * (gyro rate - Kalman bias)    [rad/s]
//   omega      = wheel speed from ESCON AnOUT1             [rad/s]
//   current    = -(K_TH*theta + K_RATE*theta_dot + K_WHEEL*omega), clamped to +/- I_MAX_BAL
//
// States: 0 DISARMED (motor off) -> 'a' -> 1 WAITING (motor off, waiting for you to hold the cube
//   within CATCH_DEG of the balance point) -> 2 BALANCING -> falls past FALL_DEG or wheel past
//   WHEEL_MAX_RPM -> 4 BRAKING (wheel actively slowed) -> back to WAITING.   3 = sign-test pulse.
//   'x', laptop link loss and the E-stop button (ESTOP_PIN, BOOT button by default) also brake,
//   then go to DISARMED.
//
// Commands (Serial Monitor at SERIAL_BAUD, or typed in log_udp.py + Enter):
//   a or g   arm            x or s   disarm (motor off)
//   z        theta_eq = current angle (hold the cube at its balance point first)
//   + / -    trim theta_eq by +/- 0.1 deg
//   p        sign test: 100 ms current pulse, prints the TILT_SIGN to use (only while disarmed)
//   (Enter)  marker, as before
//
// CSV columns (first 11 unchanged, so the MATLAB tests still read the file):
//   t_us,ax,ay,az,gx,gy,gz,temp,marker,ang_comp,ang_kf,en,th_err,rpm,cmd_A
//   en = state (0..4), th_err = ang_kf - theta_eq [deg], rpm = wheel speed, cmd_A = current command

#define OTA 1               // WiFi + over-the-air firmware upload
#define UDP_STREAM 1        // stream CSV over WiFi to log_udp.py (requires OTA 1)

#include <Wire.h>
#include <stdarg.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include "config.h"
#include "filters.h"
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

static const float RAD2DEG = 57.29577951f;
static const float DEG2RAD = 0.01745329252f;
static const float RPM2RADS = 0.10471975512f;
static const char CSV_HEADER[] =
    "t_us,ax,ay,az,gx,gy,gz,temp,marker,ang_comp,ang_kf,en,th_err,rpm,cmd_A\n";

Adafruit_ICM20948 icm;
ComplementaryFilter comp(COMP_ALPHA);
KalmanTilt kf(KF_Q_ANGLE, KF_Q_BIAS, KF_R_MEAS);

uint32_t marker = 0;
uint32_t overruns = 0;

// ---------------- motor pins (config.h: M3 slot = ESCON U3) ----------------
const int PIN_CURRENT = MOTOR_PWM_PIN;     // ESCON DigIN1, PWM current set value
const int PIN_ENABLE  = MOTOR_EN_PIN;      // ESCON DigIN2, enable
const int PIN_SPEED   = ESC_SPEED_ADC_PIN; // ESCON AnOUT1, actual speed averaged

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

// ---------------- balance state ----------------
enum { ST_DISARMED = 0, ST_WAIT = 1, ST_BAL = 2, ST_PULSE = 3, ST_BRAKE = 4 };
int   state = ST_DISARMED;
float thetaEq = THETA_EQ_DEG;
float lastIa = 0.0f, lastRpm = 0.0f, lastErr = 0.0f, lastAng = 0.0f;
bool  armedByLaptop = false;
float speedBiasV = 0.0f, speedRestV = NAN;
bool  speedSignalOk = false;
uint32_t pulseStartMs = 0;
float pulseRate0 = 0.0f, pulseAngle = 0.0f, pulseRpmMax = 0.0f;
bool  pulseNeedRate0 = false;
uint32_t balStartMs = 0;
uint32_t brakeStartMs = 0;
bool  balFresh = false;           // first sample after a catch
float rateF = 0.0f, iaInt = 0.0f, rpmRef = 0.0f;
uint32_t chkStartMs = 0;
int   brakeNext = ST_DISARMED;

// prototypes
void setup();
void loop();
static void statusf(const char* fmt, ...);
static void emitStatus(const char* line);
static void emitCsv(const char* s, size_t n);
static void handleCommand(char c, bool fromLaptop);

// =====================================================================================
//  ESCON interface
// =====================================================================================
void writeCurrent(float ia) {
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

// Actively slow the wheel, then go to 'next' (ST_WAIT after a fall, ST_DISARMED after a stop).
static void startBrake(const char* why, int next) {
  brakeNext = next;
  if (BRAKE_A <= 0.0f) { motorOff(); state = next; return; }
  digitalWrite(PIN_ENABLE, HIGH);
  brakeStartMs = millis();
  state = ST_BRAKE;
  statusf("# BAL: %s -> braking wheel\n", why);
}

static float speedVolts(int n) {
  uint32_t s = 0;
  for (int i = 0; i < n; i++) s += analogReadMilliVolts(PIN_SPEED);
  return s * 0.001f / n;
}

// Wheel speed [rpm], low-passed (~15 Hz at 500 Hz) to tame ESP32 ADC noise.
float readSpeedRpm() {
  const float vMid = 0.5f * (SPEED_V_LO + SPEED_V_HI);
  const float v = speedVolts(4) - speedBiasV;
  const float raw = SPEED_SIGN * (v - vMid) * (2.0f * SPEED_RPM_FS / (SPEED_V_HI - SPEED_V_LO));
  static float filt = 0.0f;
  filt += 0.17f * (raw - filt);
  return filt;
}

// Zero the speed signal. Wheel must be still and the ESCON powered. Blocks ~0.5 s.
void calibrateSpeed() {
  const float vMid = 0.5f * (SPEED_V_LO + SPEED_V_HI);
  float sum = 0;
  for (int i = 0; i < 50; i++) { sum += speedVolts(1); delay(10); }
  speedRestV = sum / 50.0f;
  speedBiasV = speedRestV - vMid;
  speedSignalOk = fabsf(speedBiasV) <= SPEED_BIAS_MAX;
  if (speedSignalOk) {
    statusf("# BAL: speed signal at rest %.3f V (expected %.2f V) OK\n", speedRestV, vMid);
  } else {
    statusf("# BAL: SPEED SIGNAL BAD: %.3f V at rest, expected %.2f V. ESCON powered? AnOUT1 set up "
            "as in balance_config.h?\n", speedRestV, vMid);
  }
}

// =====================================================================================
//  Balance controller (called every IMU sample)
// =====================================================================================
static void balanceUpdate(float thErrDeg, float rateDps, float rpm, float dt) {
  switch (state) {
    case ST_DISARMED:
      lastIa = 0.0f;
      return;

    case ST_WAIT:
      lastIa = 0.0f;
      if (fabsf(thErrDeg) < CATCH_DEG && fabsf(rpm) < CATCH_MAX_RPM) {
        writeCurrent(0);
        digitalWrite(PIN_ENABLE, HIGH);
        state = ST_BAL;
        balStartMs = millis();
        balFresh = true;
        statusf("# BAL: caught at %+.2f deg, BALANCING\n", thErrDeg);
      }
      return;

    case ST_BAL: {
      if (fabsf(thErrDeg) > FALL_DEG) {
        statusf("# BAL: fell (%+.1f deg) after %.1f s\n", thErrDeg, (millis() - balStartMs) * 1e-3f);
        startBrake("fall", ST_WAIT);
        return;
      }
      if (fabsf(rpm) > WHEEL_MAX_RPM) {
        statusf("# BAL: wheel saturated (%.0f rpm) - theta_eq is probably off: trim it with + / -\n", rpm);
        startBrake("wheel limit", ST_WAIT);
        return;
      }
      if (balFresh) {                                           // fresh start on every catch
        balFresh = false;
        rateF = rateDps;
        iaInt = 0.0f; rpmRef = rpm; chkStartMs = millis();
      }
      // ESCON response check: pushing current must change the wheel speed.
      iaInt += lastIa * dt;                                     // current applied since last check
      if (millis() - chkStartMs >= 300) {
        const float expRpm = KM_NM_PER_A * iaInt / WHEEL_INERTIA * 9.549f;
        const float gotRpm = rpm - rpmRef;
        if (fabsf(expRpm) > 200.0f && fabsf(gotRpm) < 0.25f * fabsf(expRpm)) {
          motorOff();
          state = ST_DISARMED;
          statusf("# BAL: ESCON NOT RESPONDING: current should have changed the wheel by %+.0f rpm, "
                  "it changed %+.0f. ESCON fault? Check its LED, switch the 24 V off/on, arm again.\n",
                  expRpm, gotRpm);
          return;
        }
        iaInt = 0.0f; rpmRef = rpm; chkStartMs = millis();
      }
      // Low-pass the body rate (see RATE_LPF_HZ) so wheel vibration doesn't make the current chatter.
      if (RATE_LPF_HZ > 0.0f) {
        const float kLpf = 1.0f - expf(-2.0f * 3.14159265f * RATE_LPF_HZ * dt);
        rateF += kLpf * (rateDps - rateF);
      } else {
        rateF = rateDps;
      }
      const float th  = TILT_SIGN * thErrDeg * DEG2RAD;
      const float thd = TILT_SIGN * rateF * DEG2RAD;
      const float w   = rpm * RPM2RADS;
      float ia = -(K_TH * th + K_RATE * thd + K_WHEEL * w);
      if (ia >  I_MAX_BAL) ia =  I_MAX_BAL;
      if (ia < -I_MAX_BAL) ia = -I_MAX_BAL;
      writeCurrent(ia);
      lastIa = ia;
      return;
    }

    case ST_BRAKE: {
      if (fabsf(rpm) < BRAKE_DONE_RPM || millis() - brakeStartMs > (uint32_t)BRAKE_TIMEOUT_MS) {
        motorOff();
        state = brakeNext;
        statusf("# BAL: wheel stopped, motor off, %s\n",
                brakeNext == ST_WAIT ? "WAITING for a new catch" : "DISARMED");
        return;
      }
      const float ia = (rpm > 0.0f) ? -BRAKE_A : BRAKE_A;   // oppose the wheel's speed
      writeCurrent(ia);
      lastIa = ia;
      return;
    }

    case ST_PULSE: {
      const uint32_t el = millis() - pulseStartMs;
      if (el < (uint32_t)PULSE_MS) { writeCurrent(PULSE_A); lastIa = PULSE_A; }
      else                         { writeCurrent(0);       lastIa = 0.0f; }
      pulseAngle += (rateDps - pulseRate0) * dt;           // body rotation caused by the pulse [deg]
      if (fabsf(rpm) > pulseRpmMax) pulseRpmMax = fabsf(rpm);
      if (el >= (uint32_t)(PULSE_MS + PULSE_WATCH_MS)) {
        motorOff();
        state = ST_DISARMED;
        if (fabsf(pulseAngle) < 0.2f) {
          statusf("# SIGN TEST inconclusive: body moved only %+.2f deg (wheel peak %.0f rpm). Hold the cube "
                  "more loosely on its edge and send p again.\n", pulseAngle, pulseRpmMax);
        } else {
          // Model: positive current pushes the body towards negative theta.
          const int sign = (pulseAngle < 0.0f) ? 1 : -1;
          statusf("# SIGN TEST: +%.1f A pulse turned the body %+.2f deg (wheel peak %.0f rpm) -> "
                  "set TILT_SIGN (%d.0f) in balance_config.h\n", PULSE_A, pulseAngle, pulseRpmMax, sign);
        }
      }
      return;
    }
  }
}

static void arm(bool fromLaptop) {
  if (state != ST_DISARMED) return;
#if ESTOP_PIN >= 0
  if (digitalRead(ESTOP_PIN) == LOW) { statusf("# BAL: ARM REFUSED: E-stop button pressed\n"); return; }
#endif
  if (K_TH == 0.0f && K_RATE == 0.0f && K_WHEEL == 0.0f) {
    statusf("# BAL: ARM REFUSED: gains are 0. Paste K_TH / K_RATE / K_WHEEL from lqr_edge.m.\n");
    return;
  }
  statusf("# BAL: zeroing speed signal, keep the wheel still...\n");
  calibrateSpeed();
  if (!speedSignalOk) { statusf("# BAL: ARM REFUSED (speed signal)\n"); return; }
  armedByLaptop = fromLaptop;
  state = ST_WAIT;
  statusf("# BAL: ARMED. theta_eq = %.2f deg. Hold the cube on its edge within %.0f deg of it.\n",
          thetaEq, CATCH_DEG);
}

// brake = true: slow the wheel first (normal stops). false: switch off at once (OTA update).
static void disarm(const char* why, bool brake = true) {
  if (state == ST_DISARMED) return;
  if (state == ST_BRAKE && brakeNext == ST_DISARMED) return;   // already stopping
  statusf("# BAL: STOP (%s)\n", why);
  if (brake && state != ST_WAIT && state != ST_PULSE) startBrake(why, ST_DISARMED);
  else { motorOff(); state = ST_DISARMED; statusf("# BAL: DISARMED\n"); }
}

static void startPulse() {
  if (state != ST_DISARMED) { statusf("# SIGN TEST: disarm first (x)\n"); return; }
  calibrateSpeed();
  if (!speedSignalOk) return;
  pulseAngle = 0.0f;
  pulseRpmMax = 0.0f;
  pulseNeedRate0 = true;   // reference rate is taken on the next IMU sample
  writeCurrent(0);
  digitalWrite(PIN_ENABLE, HIGH);
  delay(20);
  pulseStartMs = millis();
  state = ST_PULSE;
  statusf("# SIGN TEST: pulse %.1f A for %d ms...\n", PULSE_A, PULSE_MS);
}

// =====================================================================================
//  Fast IMU read (as imu_logger): ACCEL_XOUT_H (0x2D, bank 0) .. TEMP_OUT_L, 14 bytes.
// =====================================================================================
static float accLsbPerG = 2048.0f, gyroLsbPerDps = 16.4f;
uint32_t imuErrors = 0;

static void imuScalesFromConfig() {
  switch (ACC_RANGE) {
    case ICM20948_ACCEL_RANGE_2_G:  accLsbPerG = 16384.0f; break;
    case ICM20948_ACCEL_RANGE_4_G:  accLsbPerG = 8192.0f;  break;
    case ICM20948_ACCEL_RANGE_8_G:  accLsbPerG = 4096.0f;  break;
    default:                        accLsbPerG = 2048.0f;  break;
  }
  switch (GYRO_RANGE) {
    case ICM20948_GYRO_RANGE_250_DPS:  gyroLsbPerDps = 131.0f; break;
    case ICM20948_GYRO_RANGE_500_DPS:  gyroLsbPerDps = 65.5f;  break;
    case ICM20948_GYRO_RANGE_1000_DPS: gyroLsbPerDps = 32.8f;  break;
    default:                           gyroLsbPerDps = 16.4f;  break;
  }
}

static bool imuReadFast(float acc[3], float gyr[3], float* tempC) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x2D);
  if (Wire.endTransmission(false) != 0) { imuErrors++; return false; }
  if (Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)14) != 14) { imuErrors++; return false; }
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  for (int i = 0; i < 3; i++) {
    acc[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]) / accLsbPerG * 9.80665f;
    gyr[i] = (int16_t)((b[6 + 2 * i] << 8) | b[7 + 2 * i]) / gyroLsbPerDps;
  }
  *tempC = (int16_t)((b[12] << 8) | b[13]) / 333.87f + 21.0f;
  return true;
}

static float accelAngle(const float a[3]) {
#if TILT_AXIS == 0
  return atan2f(a[1], a[2]) * RAD2DEG;
#else
  return atan2f(-a[0], a[2]) * RAD2DEG;
#endif
}

// =====================================================================================
//  UDP streaming (as imu_logger)
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
    if (peerActive() && !samePeer) continue;          // one listener at a time

    const bool isNew = !peerActive();
    peerIP = ip;
    peerPort = port;
    peerLastSeen = millis();
    if (isNew) {
      udpLen = 0;
      static const char start[] = "# edge_balance: UDP stream started (a = arm, x = stop, z/+/- = balance point, p = sign test)\n";
      udpQueue(start, sizeof(start) - 1);
      udpQueue(CSV_HEADER, sizeof(CSV_HEADER) - 1);
      udpFlush();
      Serial.printf("# UDP: streaming to %s:%u\n", ip.toString().c_str(), (unsigned)port);
    }

    if (strcmp(msg, "hello") == 0) continue;
    if (strcmp(msg, "h") == 0) { udpQueue(CSV_HEADER, sizeof(CSV_HEADER) - 1); continue; }
    if (strcmp(msg, "stop") == 0) { handleCommand('x', true); continue; }
    if (n >= 1) handleCommand(msg[0], true);           // single-character commands
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
  char b[256];
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
    case 'm': marker++; break;
    case 'a': case 'A': case 'g': case 'G': arm(fromLaptop); break;
    case 'x': case 'X': case 's': case 'S': disarm("user"); break;
    case 'z': case 'Z':
      thetaEq = lastAng;
      statusf("# BAL: theta_eq set to %.2f deg  ->  #define THETA_EQ_DEG (%.2ff)\n", thetaEq, thetaEq);
      break;
    case '+': thetaEq += 0.1f; statusf("# BAL: theta_eq = %.2f deg\n", thetaEq); break;
    case '-': thetaEq -= 0.1f; statusf("# BAL: theta_eq = %.2f deg\n", thetaEq); break;
    case 'p': case 'P': startPulse(); break;
    default: break;
  }
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
  // Motor outputs safe before anything else
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);
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

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  while (!icm.begin_I2C(ICM_ADDR, &Wire)) {
    Serial.println("# ERROR: ICM-20948 not found. Check wiring / address.");
#if OTA
    for (int i = 0; i < 100; i++) { ArduinoOTA.handle(); delay(10); }
#else
    delay(1000);
#endif
  }
  icm.setAccelRange(ACC_RANGE);
  icm.setGyroRange(GYRO_RANGE);
  icm.setAccelRateDivisor(1);
  icm.setGyroRateDivisor(1);
  imuScalesFromConfig();

  Serial.println("# edge_balance ready. a = arm, x = stop, z = set balance point, +/- = trim, p = sign test");
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

  while (Serial.available()) handleCommand((char)Serial.read(), false);

  uint32_t now = micros();
  if ((int32_t)(now - next) < 0) return;
  if ((int32_t)(now - next) > (int32_t)period) {
    overruns++;
    if ((int32_t)(now - next) > (int32_t)(10 * period)) next = now;
  }
  next += period;
  static uint32_t tMax = 0;
  const uint32_t tStart = micros();

#if OTA && UDP_STREAM
  if (wifiOk) udpPoll();
  if (REQUIRE_LINK && armedByLaptop && state != ST_DISARMED &&
      !(peerActive() && millis() - peerLastSeen < 2500)) {
    disarm("laptop link lost");
  }
#endif

  // ---- IMU ----
  static float aRaw[3] = {0, 0, 9.80665f}, gRaw[3] = {0, 0, 0}, tempC = 0;
  imuReadFast(aRaw, gRaw, &tempC);   // on a failed read the previous sample is reused

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

  // ---- E-stop button ----
#if ESTOP_PIN >= 0
  if (digitalRead(ESTOP_PIN) == LOW) disarm("E-STOP button");
#endif

  // ---- balance ----
  const float rateCorr = rate - kf.bias();
  lastAng = angKf;
  lastErr = wrap180(angKf - thetaEq);
  lastRpm = readSpeedRpm();
  if (pulseNeedRate0) { pulseRate0 = rateCorr; pulseNeedRate0 = false; }
  balanceUpdate(lastErr, rateCorr, lastRpm, dt);

  // ---- log ----
  char buf[200];
  int n = snprintf(buf, sizeof(buf),
                   "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%lu,%.3f,%.3f,%d,%.3f,%.0f,%.3f\n",
                   (unsigned long)now, aRaw[0], aRaw[1], aRaw[2], gRaw[0], gRaw[1], gRaw[2],
                   tempC, (unsigned long)marker, angComp, angKf, state, lastErr, lastRpm, lastIa);
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  emitCsv(buf, n);

  if (millis() - lastReport > 5000) {
    lastReport = millis();
    statusf("# overruns=%lu  state=%d  theta_eq=%.2f  th_err=%+.2f  rpm=%.0f  cmd=%.2fA  loop_max=%luus%s\n",
            (unsigned long)overruns, state, thetaEq, lastErr, lastRpm, lastIa, (unsigned long)tMax,
            imuErrors ? "  IMU_I2C_ERRORS" : "");
    tMax = 0;
  }

#if OTA && UDP_STREAM
  if (millis() - udpLastFlush >= UDP_FLUSH_MS) udpFlush();
#endif
  const uint32_t busy = micros() - tStart;
  if (busy > tMax) tMax = busy;
}
