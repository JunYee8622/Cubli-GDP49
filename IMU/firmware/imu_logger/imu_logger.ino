// imu_logger.ino — ESP32 + Adafruit ICM-20948 logger, WiFi OTA, UDP streaming, ESCON speed-loop test.
//
// Streams one CSV line per loop at LOOP_HZ:
//   t_us,ax,ay,az,gx,gy,gz,temp,marker,ang_comp,ang_kf,en,ref_rpm,rpm,cmd_A
//   ax..az   raw accelerometer   [m/s^2]  (uncalibrated: calibration is applied offline)
//   gx..gz   raw gyro            [deg/s]  (uncalibrated)
//   marker   increments every time an 'm' arrives (Enter in log_serial.py or log_udp.py)
//   ang_*    live 1D tilt estimates using config.h calibration + tuning [deg]
//   en       1 while the motor speed test is running
//   ref_rpm  speed reference from the profile [rpm]
//   rpm      measured wheel speed from ESCON AnOUT1 [rpm]
//   cmd_A    current command sent to the ESCON [A]
//   The first 11 columns are unchanged, so the existing analysis scripts still read the file.
//
// Output routing:
//   - While log_udp.py runs (it sends "hello" every second), CSV goes over WiFi/UDP, not Serial.
//   - Otherwise CSV goes to Serial as before. '#' status lines always go to both.
//
// MOTOR TEST: PI speed loop on one ESCON (current mode). Profile 0 -> 300 -> 400 -> 300 -> 0 rpm,
//   repeating. It does NOT start by itself at boot (set MOTOR_AUTOSTART 1 for the old behaviour).
//   Start: 'g'  (Serial Monitor, or type g + Enter in log_udp.py)
//   Stop:  's'  (Serial Monitor), or x / Ctrl+C in log_udp.py
//   It stops itself, and prints why, on: overspeed, runaway (speed sign reversed), stall (full
//   current but no speed), bad speed signal at boot, OTA upload, laptop link lost.
//   10 Hz summary on Serial only: "#M ref_rpm,actual_rpm,current_mA" (log_serial.py skips '#').
//
// Libraries (Arduino Library Manager): "Adafruit ICM20X", "Adafruit BusIO", "Adafruit Unified Sensor".
// WiFi / ESPmDNS / ArduinoOTA ship with the ESP32 board package.

#define OTA 1               // WiFi + over-the-air firmware upload
#define UDP_STREAM 1        // stream CSV over WiFi to log_udp.py (requires OTA 1)
#define MOTOR_AUTOSTART 0   // 1 = start the speed profile automatically at boot
#define IMU_FAST_READ 1     // 1 = read accel/gyro/temp directly in one I2C transaction (~2x faster)
                            // 0 = Adafruit getEvent() (also reads the unused magnetometer)

#include <Wire.h>
#include <stdarg.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include "config.h"
#include "filters.h"

#if OTA
  #include <WiFi.h>
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>

  // Can be overridden in config.h. Must be a 2.4 GHz WPA2-Personal network (not eduroam).
  #ifndef WIFI_SSID
    #define WIFI_SSID     "Jun Yee"
  #endif
  #ifndef WIFI_PASSWORD
    #define WIFI_PASSWORD "12345678900"
  #endif
  #ifndef OTA_HOSTNAME
    #define OTA_HOSTNAME  "cube-esp32"
  #endif
  #ifndef WIFI_TIMEOUT_MS
    #define WIFI_TIMEOUT_MS 10000UL
  #endif

  #ifndef UDP_PORT
    #define UDP_PORT 4210               // must match --port in log_udp.py
  #endif
  #ifndef UDP_PEER_TIMEOUT_MS
    #define UDP_PEER_TIMEOUT_MS 5000UL  // stop streaming if the laptop goes quiet this long
  #endif
  #ifndef UDP_FLUSH_MS
    #define UDP_FLUSH_MS 20UL           // send a batched packet at least every 20 ms
  #endif
  #define UDP_MAX_PAYLOAD 1400          // stay under the WiFi MTU so packets never fragment

  volatile bool otaActive = false;      // pauses logging while a firmware upload is in progress
  bool wifiOk = false;
#endif

static const float RAD2DEG = 57.29577951f;
static const char CSV_HEADER[] =
    "t_us,ax,ay,az,gx,gy,gz,temp,marker,ang_comp,ang_kf,en,ref_rpm,rpm,cmd_A\n";

Adafruit_ICM20948 icm;
ComplementaryFilter comp(COMP_ALPHA);
KalmanTilt kf(KF_Q_ANGLE, KF_Q_BIAS, KF_R_MEAS);

uint32_t marker = 0;
uint32_t overruns = 0;

// Explicit prototypes so the build never depends on Arduino's auto-generated ones
void setup();
void loop();
static void statusf(const char* fmt, ...);
static void emitStatus(const char* line);
static void emitCsv(const char* s, size_t n);
void writeCurrent(float ia);
float readSpeedRpm();
void calibrateSpeed();
void stopMotor(const char* why);
void startMotor(bool fromLaptop);
void motorUpdate();
static float accelAngle(const float a[3]);
#if OTA
static void flashStatus(uint8_t r, uint8_t g, uint8_t b, int times);
static void setupOTA();
#endif

// =====================================================================================
//  MOTOR TEST (ESCON)
// =====================================================================================

// ---------- Pins (motherboard M3 slot = ESCON U3) ----------
// If the motor is on the M2 slot instead, use 18 / 17 / 9.  M1 slot: 12 / 13 / 10.
const int PIN_CURRENT = 6;    // ESCON DigIN1 (PWM current set value)
const int PIN_ENABLE  = 5;    // ESCON DigIN2 (enable)
const int PIN_SPEED   = 7;    // ESCON AnOUT1 (actual speed, ADC1)

// ---------- ESCON current mapping: MUST match ESCON Studio ----------
// PWM 10 % = -IA_MAX_ESCON, 50 % = 0 A, 90 % = +IA_MAX_ESCON.
const float IA_MAX_ESCON = 6.0f;      // A (Willem's .edc). Use 2.0f if you set +-2 A in ESCON Studio.
const float KM = 36e-3f;              // torque constant, Nm/A

// ---------- ESCON AnOUT1 speed mapping: MUST match ESCON Studio ----------
// Willem's .edc: 0.3 V = -6500 rpm, 3.0 V = +6500 rpm (1.65 V = 0 rpm).
// If you set 0.2 V / 3.0 V / +-8000 rpm instead, change these three lines to 0.2f, 3.0f, 8000.0f.
const float SPEED_V_LO   = 0.2f;      // V at -SPEED_RPM_FS
const float SPEED_V_HI   = 3.0f;      // V at +SPEED_RPM_FS
const float SPEED_RPM_FS = 7000.0f;   // rpm at SPEED_V_HI
// Loop polarity. If the test stops with "RUNAWAY", flip this to -1.0f.
const float SPEED_SIGN   = 1.0f;

// ---------- Test limits ----------
const float IA_LIMIT_TEST  = 1.5f;     // A
const float OVERSPEED_RPM  = 1300.0f;  // trip if measured speed exceeds this
const float SPEED_BIAS_MAX = 0.15f;    // V: refuse to start if the at-rest signal is further off
const float ZERO_BAND_RPM  = 30.0f;    // below this the speed signal is mostly ADC noise

// ---------- PI speed loop ----------
const float KP = 0.004f;              // A per rpm error
const float KI = 0.002f;              // A per (rpm*s)
const uint32_t MOTOR_LOOP_MS = 5;     // 200 Hz

// ---------- PWM (same as Willem: 1 kHz, 12 bit) ----------
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

// ---------- Profile (each step ramps linearly to targetRpm over durationS) ----------
struct Step { float targetRpm; float durationS; };
const Step PROFILE[] = {
  {  200.0f, 3.0f }, {  200.0f, 10.0f },   // 3 s ramp, 10 s hold
  {  400.0f, 3.0f }, {  400.0f, 10.0f },
  {  600.0f, 3.0f }, {  600.0f, 10.0f },
  {  800.0f, 3.0f }, {  800.0f, 10.0f },
  { 1000.0f, 3.0f }, { 1000.0f, 10.0f },
  {    0.0f, 6.0f }, {    0.0f, 30.0f },   // ramp down, then a 30 s pause
};
const bool REVERSE_EACH_RUN = false;
const int N_STEPS = sizeof(PROFILE) / sizeof(PROFILE[0]);

// ---------- Motor state ----------
float speedBiasV = 0.0f;
float speedRestV = NAN;            // voltage measured by the last at-rest check
bool  speedSignalOk = false;       // set by calibrateSpeed()
bool  motorRunning = false, reverseRun = false, startedByLaptop = false;
int   stepIdx = 0;
float stepStartRpm = 0, refRpm = 0, integ = 0;
float lastRpm = 0, lastIa = 0;     // for the CSV
uint32_t stepStartMs = 0, motorLastMs = 0;
int   runawayCount = 0;
uint32_t stallMs = 0;

// Current (A) -> PWM, same mapping as Willem's Motor::set_current (without his sign flip)
void writeCurrent(float ia) {
  if (ia >  IA_MAX_ESCON) ia =  IA_MAX_ESCON;
  if (ia < -IA_MAX_ESCON) ia = -IA_MAX_ESCON;
  float duty = 0.5f + ia * (0.8f / (2.0f * IA_MAX_ESCON));
  PWM_WRITE((uint32_t)(duty * 4095.0f));
}

static float speedVolts(int n) {
  uint32_t s = 0;
  for (int i = 0; i < n; i++) s += analogReadMilliVolts(PIN_SPEED);
  return s * 0.001f / n;
}

float readSpeedRpm() {
  const float vMid = 0.5f * (SPEED_V_LO + SPEED_V_HI);
  const float v = speedVolts(8) - speedBiasV;      // 8 samples: less ADC noise
  const float raw = SPEED_SIGN * (v - vMid) * (2.0f * SPEED_RPM_FS / (SPEED_V_HI - SPEED_V_LO));
  static float filt = 0.0f;
  filt += 0.4f * (raw - filt);                     // ~15 Hz low-pass at the 200 Hz motor loop
  return filt;
}

void calibrateSpeed() {             // wheel must be at rest, ESCON powered
  const float vMid = 0.5f * (SPEED_V_LO + SPEED_V_HI);
  float sum = 0;
  for (int i = 0; i < 100; i++) { sum += speedVolts(1); delay(10); }
  const float vRest = sum / 100.0f;
  speedRestV = vRest;
  speedBiasV = vRest - vMid;
  speedSignalOk = fabsf(speedBiasV) <= SPEED_BIAS_MAX;
  if (speedSignalOk) {
    statusf("# Motor: speed signal at rest %.3f V (expected %.2f V), offset %+.3f V OK\n",
            vRest, vMid, speedBiasV);
  } else {
    statusf("# Motor: SPEED SIGNAL BAD: %.3f V at rest, expected %.2f V. ESCON not powered at boot, "
            "or AnOUT1 not set up as in SPEED_V_LO/HI. Motor start is blocked: fix it, then send "
            "'g' again to re-check.\n", vRest, vMid);
  }
}

void stopMotor(const char* why) {
  writeCurrent(0);
  digitalWrite(PIN_ENABLE, LOW);
  const bool wasRunning = motorRunning;
  motorRunning = false; integ = 0; refRpm = 0; lastIa = 0;
  if (wasRunning || strcmp(why, "user") == 0) statusf("# Motor: STOPPED (%s)\n", why);
}

void startMotor(bool fromLaptop) {
  if (motorRunning) return;
  if (!speedSignalOk || fabsf(lastRpm) < 80.0f) {   // wheel at rest: re-zero before every start
    // Re-check now (wheel idle), so a fixed ESCON setting doesn't need an ESP32 reset.
    statusf("# Motor: zeroing speed signal (last: %.3f V), keep wheel still...\n", speedRestV);
    calibrateSpeed();
    if (!speedSignalOk) {
      statusf("# Motor: START REFUSED: speed signal %.3f V at rest, expected %.2f V.\n",
              speedRestV, 0.5f * (SPEED_V_LO + SPEED_V_HI));
      return;
    }
  }
  writeCurrent(0);
  digitalWrite(PIN_ENABLE, HIGH);
  delay(100);
  motorRunning = true; stepIdx = 0; stepStartRpm = 0; integ = 0;
  runawayCount = 0; stallMs = 0;
  startedByLaptop = fromLaptop;
  motorLastMs = stepStartMs = millis();   // so the first PI step uses a 5 ms dt, not time-since-boot
  statusf("# Motor: START, direction %s\n", reverseRun ? "REVERSE" : "FORWARD");
}

// Non-blocking: called every loop() pass, only acts every MOTOR_LOOP_MS
void motorUpdate() {
  uint32_t nowMs = millis();
  if (nowMs - motorLastMs < MOTOR_LOOP_MS) return;
  float dtM = (nowMs - motorLastMs) / 1000.0f;
  if (dtM > 0.05f) dtM = 0.05f;           // never integrate a long gap
  motorLastMs = nowMs;

  const float rpm = readSpeedRpm();       // read even when stopped, so the coast-down is logged
  lastRpm = rpm;
  if (!motorRunning) return;

  if (fabsf(rpm) > OVERSPEED_RPM) { stopMotor("overspeed"); return; }

  // Reference ramp
  const Step &s = PROFILE[stepIdx];
  float frac = fminf((nowMs - stepStartMs) / 1000.0f / s.durationS, 1.0f);
  float target = reverseRun ? -s.targetRpm : s.targetRpm;
  refRpm = stepStartRpm + (target - stepStartRpm) * frac;

  // PI with clamping anti-windup
  float err = refRpm - rpm;
  float ia = KP * err + KI * integ;
  bool sat = fabsf(ia) >= IA_LIMIT_TEST;
  if (!sat || (ia > 0) != (err > 0)) integ += err * dtM;
  ia = constrain(ia, -IA_LIMIT_TEST, IA_LIMIT_TEST);
  // Near standstill the speed signal is mostly ADC noise: hold 0 A instead of jerking back and forth.
  if (fabsf(refRpm) < ZERO_BAND_RPM && fabsf(rpm) < 2.0f * ZERO_BAND_RPM) {
    ia = 0.0f; integ = 0.0f; sat = false;
  }
  writeCurrent(ia);
  lastIa = ia;

  // Safety checks, every 100 ms
  static uint32_t lastChk = 0;
  static float rpmPrev = 0;
  if (nowMs - lastChk >= 100) {
    // RUNAWAY: current saturated against the measured speed, yet |speed| keeps rising.
    // Real braking makes |speed| fall, so this only happens when the speed sign is reversed.
    if (sat && ia * rpm < 0 && fabsf(rpm) > 200.0f && fabsf(rpm) > fabsf(rpmPrev) + 20.0f) runawayCount++;
    else runawayCount = 0;
    // STALL: full current for 3 s but (almost) no measured speed.
    if (sat && fabsf(rpm) < 50.0f && fabsf(refRpm) > 100.0f) stallMs += nowMs - lastChk;
    else stallMs = 0;
    rpmPrev = rpm;
    lastChk = nowMs;
    if (runawayCount >= 3) {
      stopMotor("RUNAWAY: speed rises while the current pushes against it -> loop polarity "
                "reversed, set SPEED_SIGN = -1.0f");
      return;
    }
    if (stallMs >= 3000) {
      stopMotor("STALL: full test current for 3 s but no speed. Check the ESCON LED for a fault, "
                "that IA_MAX_ESCON matches ESCON Studio, and that the wheel is free");
      return;
    }
  }

  static uint32_t lastPrint = 0;
  if (nowMs - lastPrint >= 100) {
    lastPrint = nowMs;
    Serial.printf("#M %.0f,%.0f,%.0f\n", refRpm, rpm, ia * 1000.0f);
  }

  if (frac >= 1.0f) {
    stepStartRpm = refRpm;
    stepStartMs = nowMs;
    if (++stepIdx >= N_STEPS) {     // loop forever, stay enabled
      stepIdx = 0;
      stepStartRpm = 0;
      if (REVERSE_EACH_RUN) reverseRun = !reverseRun;
    }
  }
}

// =====================================================================================
//  Fast IMU read: ACCEL_XOUT_H (0x2D, bank 0) .. TEMP_OUT_L, 14 bytes, one I2C transaction.
//  Same units as Adafruit getEvent(): accel m/s^2, gyro deg/s, temp degC.
//  The Adafruit library leaves the chip in register bank 0 after begin/config.
// =====================================================================================
static float accLsbPerG = 2048.0f, gyroLsbPerDps = 16.4f;
uint32_t imuErrors = 0;

static void imuScalesFromConfig() {
  switch (ACC_RANGE) {
    case ICM20948_ACCEL_RANGE_2_G:  accLsbPerG = 16384.0f; break;
    case ICM20948_ACCEL_RANGE_4_G:  accLsbPerG = 8192.0f;  break;
    case ICM20948_ACCEL_RANGE_8_G:  accLsbPerG = 4096.0f;  break;
    default:                        accLsbPerG = 2048.0f;  break;   // 16 g
  }
  switch (GYRO_RANGE) {
    case ICM20948_GYRO_RANGE_250_DPS:  gyroLsbPerDps = 131.0f; break;
    case ICM20948_GYRO_RANGE_500_DPS:  gyroLsbPerDps = 65.5f;  break;
    case ICM20948_GYRO_RANGE_1000_DPS: gyroLsbPerDps = 32.8f;  break;
    default:                           gyroLsbPerDps = 16.4f;  break;   // 2000 dps
  }
}

// Returns false (and leaves the outputs unchanged) if the I2C read fails.
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

// =====================================================================================
//  UDP streaming
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

// Send whatever is batched. Lines are never split across packets.
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

// Packets from the laptop: "hello" keeps the stream alive, "m" marker, "g" start, "x" stop.
static void udpPoll() {
  while (udp.parsePacket() > 0) {
    char msg[32];
    int n = udp.read(msg, sizeof(msg) - 1);
    if (n < 0) n = 0;
    msg[n] = '\0';

    const IPAddress ip = udp.remoteIP();
    const uint16_t port = udp.remotePort();
    const bool samePeer = (ip == peerIP && port == peerPort);

    // Already streaming to one listener: ignore any other until the current one goes quiet.
    if (peerActive() && !samePeer) {
      static uint32_t lastBusyMsg = 0;
      if (millis() - lastBusyMsg > 5000) {
        lastBusyMsg = millis();
        char busy[96];
        int len = snprintf(busy, sizeof(busy), "# UDP: busy, already streaming to %s:%u\n",
                           peerIP.toString().c_str(), (unsigned)peerPort);
        udp.beginPacket(ip, port);
        udp.write((const uint8_t*)busy, len);
        udp.endPacket();
      }
      continue;
    }

    const bool isNew = !peerActive();
    peerIP = ip;
    peerPort = port;
    peerLastSeen = millis();

    if (isNew) {
      udpLen = 0;
      static const char start[] = "# imu_logger: UDP stream started (motor test: g = start, x = stop)\n";
      udpQueue(start, sizeof(start) - 1);
      udpQueue(CSV_HEADER, sizeof(CSV_HEADER) - 1);
      udpFlush();
      Serial.printf("# UDP: streaming to %s:%u\n", ip.toString().c_str(), (unsigned)port);
    }

    if (strcmp(msg, "m") == 0)      marker++;
    else if (strcmp(msg, "g") == 0) startMotor(true);
    else if (strcmp(msg, "x") == 0) stopMotor("user");
    else if (strcmp(msg, "h") == 0) udpQueue(CSV_HEADER, sizeof(CSV_HEADER) - 1);
    else if (msg[0] == 'c')         statusf("# 'c <amps>' is not used by this speed-test sketch: use g / x\n");
  }
}
#endif

// '#' status line (must end in '\n'): always to Serial, and to UDP when streaming.
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

// CSV data line: to UDP when a listener is active, otherwise to Serial.
static void emitCsv(const char* s, size_t n) {
#if OTA && UDP_STREAM
  if (peerActive()) {
    udpQueue(s, n);
    return;
  }
#endif
  Serial.write((const uint8_t*)s, n);
}

// =====================================================================================

static float accelAngle(const float a[3]) {
#if TILT_AXIS == 0
  return atan2f(a[1], a[2]) * RAD2DEG;
#else
  return atan2f(-a[0], a[2]) * RAD2DEG;
#endif
}

#if OTA
static void flashStatus(uint8_t r, uint8_t g, uint8_t b, int times) {
#ifdef RGB_BUILTIN
  for (int i = 0; i < times; i++) {
    neopixelWrite(RGB_BUILTIN, r, g, b);
    delay(250);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    delay(250);
  }
#endif
}

static void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);              // modem sleep makes OTA and streaming slow/unreliable
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    const wl_status_t st = WiFi.status();
    const char* why = (st == WL_NO_SSID_AVAIL)   ? "network not found (check name / 2.4 GHz)" :
                      (st == WL_CONNECT_FAILED)  ? "rejected (check password)" :
                      (st == WL_DISCONNECTED)    ? "disconnected / timed out (often wrong password)" :
                                                   "other";
    Serial.printf("# WiFi: connection FAILED to \"%s\", status=%d: %s\n", WIFI_SSID, (int)st, why);

    Serial.println("# WiFi: networks visible to the ESP32:");
    const int nNet = WiFi.scanNetworks();
    for (int i = 0; i < nNet; i++) {
      Serial.printf("#   \"%s\"  RSSI %d dBm  ch %d\n",
                    WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
    }
    if (nNet <= 0) Serial.println("#   (none)");
    WiFi.scanDelete();

    Serial.println("# WiFi: continuing without OTA (USB upload still works)");
    flashStatus(255, 255, 0, 6);     // six yellow flashes
    return;
  }

  Serial.print("# WiFi: connected, IP ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA
    .onStart([]() {
      stopMotor("OTA update");       // never drive the wheel during a reflash
      otaActive = true;
      Serial.println("# OTA: update starting, logging paused");
    })
    .onEnd([]() {
      Serial.println("# OTA: update complete, rebooting");
    })
    .onError([](ota_error_t error) {
      otaActive = false;
      Serial.printf("# OTA: error %u, logging resumed\n", (unsigned)error);
    });
  ArduinoOTA.begin();
  Serial.printf("# OTA: ready as %s\n", OTA_HOSTNAME);

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
  writeCurrent(0);                  // 50 % duty = 0 A
  analogReadResolution(12);

  Serial.begin(SERIAL_BAUD);
  delay(200);

#if OTA
  // Bring up WiFi before the IMU so the board can still be reflashed
  // wirelessly even if the IMU is missing or miswired.
  setupOTA();
#endif

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  while (!icm.begin_I2C(ICM_ADDR, &Wire)) {
    Serial.println("# ERROR: ICM-20948 not found. Check wiring / address.");
#if OTA
    for (int i = 0; i < 100; i++) {
      ArduinoOTA.handle();
      delay(10);
    }
#else
    delay(1000);
#endif
  }
  icm.setAccelRange(ACC_RANGE);
  icm.setGyroRange(GYRO_RANGE);
  // Output data rate = 1125 / (1 + divisor) Hz. Divisor 1 -> 562.5 Hz (just above LOOP_HZ).
  icm.setAccelRateDivisor(1);
  icm.setGyroRateDivisor(1);
  imuScalesFromConfig();

  Serial.println("# imu_logger ready");
  Serial.print(CSV_HEADER);

  Serial.println("# Motor: calibrating speed signal, keep wheel still...");
  calibrateSpeed();
#if MOTOR_AUTOSTART
  delay(1000);
  startMotor(false);
#else
  Serial.println("# Motor: idle. Send 'g' to start the speed profile, 's' (Serial) / 'x' (UDP) to stop.");
#endif
}

void loop() {
#if OTA
  ArduinoOTA.handle();
  if (otaActive) return;
#endif

  static uint32_t mxMot = 0;
  {
    const uint32_t t0 = micros();
    motorUpdate();                  // 200 Hz motor loop, non-blocking
    const uint32_t dtMot = micros() - t0;
    if (dtMot > mxMot) mxMot = dtMot;
  }

  static const uint32_t period = 1000000UL / LOOP_HZ;
  static uint32_t next = micros();
  static uint32_t last = micros();
  static uint32_t lastReport = millis();

  while (Serial.available()) {
    const char c = Serial.read();
    if (c == 'm') marker++;
    else if (c == 's' || c == 'S') stopMotor("user");
    else if (c == 'g' || c == 'G') startMotor(false);
  }

  uint32_t now = micros();
  if ((int32_t)(now - next) < 0) return;          // not time yet
  if ((int32_t)(now - next) > (int32_t)period) {  // late by more than a full period
    overruns++;
    if ((int32_t)(now - next) > (int32_t)(10 * period)) next = now;  // resync if far behind
  }
  next += period;

  // Loop timing per section, reported in the 5 s status line.
  static uint32_t tSum = 0, tMax = 0, tCnt = 0, mxNet = 0, mxImu = 0, mxOut = 0;
  const uint32_t tStart = micros();

#if OTA && UDP_STREAM
  if (wifiOk) udpPoll();
  // Started from the laptop: stop if the laptop goes quiet (script closed, WiFi lost).
  if (motorRunning && startedByLaptop && !(peerActive() && millis() - peerLastSeen < 2500)) {
    stopMotor("laptop link lost");
  }
#endif
  const uint32_t tNet = micros();

  static float aRaw[3] = {0, 0, 9.80665f}, gRaw[3] = {0, 0, 0}, tempC = 0;
#if IMU_FAST_READ
  imuReadFast(aRaw, gRaw, &tempC);        // on a failed read the previous sample is reused
#else
  {
    sensors_event_t accel, gyro, temp, mag;
    icm.getEvent(&accel, &gyro, &temp, &mag);
    aRaw[0] = accel.acceleration.x; aRaw[1] = accel.acceleration.y; aRaw[2] = accel.acceleration.z;
    gRaw[0] = gyro.gyro.x * RAD2DEG; gRaw[1] = gyro.gyro.y * RAD2DEG; gRaw[2] = gyro.gyro.z * RAD2DEG;
    tempC = temp.temperature;
  }
#endif
  const uint32_t tImu = micros();

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

  char buf[200];
  int n = snprintf(buf, sizeof(buf),
                   "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%lu,%.3f,%.3f,%d,%.0f,%.0f,%.3f\n",
                   (unsigned long)now, aRaw[0], aRaw[1], aRaw[2], gRaw[0], gRaw[1], gRaw[2],
                   tempC, (unsigned long)marker, angComp, angKf,
                   motorRunning ? 1 : 0, refRpm, lastRpm, lastIa);
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  emitCsv(buf, n);

  if (millis() - lastReport > 5000) {
    lastReport = millis();
    char tim[128];
    snprintf(tim, sizeof(tim), "  loop_us avg=%lu max=%lu (net=%lu imu=%lu out=%lu motor=%lu)",
             (unsigned long)(tCnt ? tSum / tCnt : 0), (unsigned long)tMax, (unsigned long)mxNet,
             (unsigned long)mxImu, (unsigned long)mxOut, (unsigned long)mxMot);
    tSum = tMax = tCnt = mxNet = mxImu = mxOut = mxMot = 0;

    char mot[128];
    snprintf(mot, sizeof(mot), "  motor=%s rpm=%.0f speed_in=%.3fV%s%s",
             motorRunning ? "RUN" : "idle", lastRpm, speedVolts(4),
             speedSignalOk ? "" : " SPEED_SIGNAL_BAD (boot check failed, send g to re-check)",
             imuErrors ? "  IMU_I2C_ERRORS" : "");

    char rep[320];
#if OTA
    const char* udpState = "";
  #if UDP_STREAM
    udpState = peerActive() ? "  udp=streaming" : "  udp=idle";
  #endif
    if (WiFi.status() == WL_CONNECTED) {
      snprintf(rep, sizeof(rep), "# overruns=%lu%s%s  wifi=connected ip=%s%s\n",
               (unsigned long)overruns, mot, tim, WiFi.localIP().toString().c_str(), udpState);
    } else {
      snprintf(rep, sizeof(rep), "# overruns=%lu%s%s  wifi=NOT connected (status=%d)\n",
               (unsigned long)overruns, mot, tim, (int)WiFi.status());
    }
#else
    snprintf(rep, sizeof(rep), "# overruns=%lu%s%s\n", (unsigned long)overruns, mot, tim);
#endif
    emitStatus(rep);
  }

#if OTA && UDP_STREAM
  if (millis() - udpLastFlush >= UDP_FLUSH_MS) udpFlush();
#endif

  // net = udpPoll + watchdog, imu = I2C read, out = filters + CSV + status + UDP send
  const uint32_t tEnd = micros();
  const uint32_t busy = tEnd - tStart;
  tSum += busy;
  tCnt++;
  if (busy > tMax) tMax = busy;
  if (tNet - tStart > mxNet) mxNet = tNet - tStart;
  if (tImu - tNet > mxImu) mxImu = tImu - tNet;
  if (tEnd - tImu > mxOut) mxOut = tEnd - tImu;
}
