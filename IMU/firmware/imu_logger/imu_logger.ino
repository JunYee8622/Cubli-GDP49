// imu_logger.ino — ESP32 + Adafruit ICM-20948 test logger, with WiFi OTA flashing and UDP streaming.
//
// Streams one CSV line per loop at LOOP_HZ:
//   t_us,ax,ay,az,gx,gy,gz,temp,marker,ang_comp,ang_kf,en,cmd_A,duty,hall_rpm,esc_rpm,esc_A,ready
//   ax..az    raw accelerometer   [m/s^2]  (uncalibrated: calibration is applied offline)
//   gx..gz    raw gyro            [deg/s]  (uncalibrated)
//   marker    increments every time an 'm' arrives (Enter in log_serial.py or log_udp.py)
//   ang_*     live 1D tilt estimates using config.h calibration + tuning [deg]
//   en        ESCON enable output (1/0)
//   cmd_A     commanded motor current [A];  duty = PWM duty sent to the ESCON [%]
//   hall_rpm  wheel speed from the motor Hall sensors [rpm]   (nan if not wired)
//   esc_rpm   ESCON "actual speed averaged" from AnOUT1 [rpm] (nan if not wired)
//   esc_A     ESCON "actual current averaged" from AnOUT2 [A] (nan if not wired)
//   ready     ESCON Ready output (1 ready, 0 fault, -1 not wired)
//   The motor columns are appended at the end, so older analysis scripts still read the file.
//
// Motor test commands (sent by log_udp.py, accepted only from the active listener):
//   "c <amps>"  set current and enable (clamped to MOTOR_TEST_MAX_A)
//   "x"         zero current and disable
// The motor is disabled at boot, during OTA uploads, and if the laptop goes quiet for
// MOTOR_CMD_TIMEOUT_MS. Wiring and ESCON Studio settings: see motor.h.
//
// Output routing:
//   - When log_udp.py is running (it sends "hello" every second), CSV lines go over WiFi/UDP
//     and NOT to Serial, so the UART can't throttle the loop. '#' status lines go to both.
//   - When no UDP listener is active (5 s timeout), CSV goes to Serial as before.
// All non-CSV status output starts with '#' so the loggers can skip it.
//
// Libraries (Arduino Library Manager): "Adafruit ICM20X", "Adafruit BusIO", "Adafruit Unified Sensor".
// WiFi / ESPmDNS / ArduinoOTA ship with the ESP32 board package.

#define OTA 1         // WiFi + over-the-air firmware upload
#define UDP_STREAM 1  // stream CSV over WiFi to log_udp.py (requires OTA 1)

#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include "config.h"
#include "filters.h"
#include "motor.h"         // ESCON motor driver + telemetry

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

  // UDP streaming
  #ifndef UDP_PORT
    #define UDP_PORT 4210               // must match --port in log_udp.py
  #endif
  #ifndef UDP_PEER_TIMEOUT_MS
    #define UDP_PEER_TIMEOUT_MS 5000UL  // stop streaming if the laptop goes quiet this long
  #endif
  #ifndef UDP_FLUSH_MS
    #define UDP_FLUSH_MS 20UL           // send a batched packet at least every 20 ms (50 packets/s)
  #endif
  #define UDP_MAX_PAYLOAD 1400          // stay under the WiFi MTU so packets never fragment

  volatile bool otaActive = false;      // pauses logging while a firmware upload is in progress
  bool wifiOk = false;
#endif

static const float RAD2DEG = 57.29577951f;
static const char CSV_HEADER[] =
    "t_us,ax,ay,az,gx,gy,gz,temp,marker,ang_comp,ang_kf,en,cmd_A,duty,hall_rpm,esc_rpm,esc_A,ready\n";

Adafruit_ICM20948 icm;
ComplementaryFilter comp(COMP_ALPHA);
KalmanTilt kf(KF_Q_ANGLE, KF_Q_BIAS, KF_R_MEAS);

uint32_t marker = 0;
uint32_t overruns = 0;

// ---------------------------------------------------------------------------------------------
// UDP streaming
// ---------------------------------------------------------------------------------------------
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

// Handle packets from the laptop: "hello" keeps the stream alive, "m" adds a marker.
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
    // Without this, two running copies of log_udp.py make the stream ping-pong between them.
    if (peerActive() && !samePeer) {
      static uint32_t lastBusyMsg = 0;
      if (millis() - lastBusyMsg > 5000) {
        lastBusyMsg = millis();
        char busy[96];
        int len = snprintf(busy, sizeof(busy), "# UDP: busy, already streaming to %s:%u\n",
                           peerIP.toString().c_str(), (unsigned)peerPort);
        udp.beginPacket(ip, port);   // tell the second script why it gets no data
        udp.write((const uint8_t*)busy, len);
        udp.endPacket();
      }
      continue;
    }

    const bool isNew = !peerActive();   // true only when a new session starts
    peerIP = ip;
    peerPort = port;
    peerLastSeen = millis();

    if (strcmp(msg, "m") == 0) {
      marker++;
    } else if (msg[0] == 'c' && msg[1] == ' ') {          // "c <amps>": set current + enable
      const float req = strtof(msg + 2, nullptr);
      motorSetCurrent(req);
      motorEnable(true);
      if (mt.enabled) {
        if (fabsf(req - mt.cmdA) > 1e-4f)
          motorMsg("# MOTOR: command %.3f A (clamped from %.3f A by MOTOR_TEST_MAX_A)\n", mt.cmdA, req);
        else
          motorMsg("# MOTOR: command %.3f A, enabled\n", mt.cmdA);
      }
    } else if (strcmp(msg, "x") == 0) {                    // stop
      motorStop();
      motorMsg("# MOTOR: stopped, disabled\n");
    } else if (strcmp(msg, "h") == 0) {                    // resend the CSV header
      udpQueue(CSV_HEADER, sizeof(CSV_HEADER) - 1);
    }

    if (isNew) {
      udpLen = 0;
      static const char start[] = "# imu_logger: UDP stream started\n";
      udpQueue(start, sizeof(start) - 1);
      udpQueue(CSV_HEADER, sizeof(CSV_HEADER) - 1);
      udpFlush();
      Serial.printf("# UDP: streaming to %s:%u\n", ip.toString().c_str(), (unsigned)port);
    }
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

// ---------------------------------------------------------------------------------------------
// WiFi + OTA
// ---------------------------------------------------------------------------------------------
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

    // List what the ESP32 can actually see, to compare against WIFI_SSID.
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
      motorStop();                   // never leave the motor running during a firmware upload
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

// ---------------------------------------------------------------------------------------------

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

  // Motor first: drive enable LOW and the PWM to 0 A as early as possible.
  motorBegin(emitStatus);

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
    // Keep servicing OTA while waiting, instead of blocking for a full second.
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

  Serial.println("# imu_logger ready");
  Serial.print(CSV_HEADER);
}

void loop() {
#if OTA
  // Called on every pass (including the early "not time yet" returns), so it is
  // serviced far more often than LOOP_HZ. Cheap when no upload is in progress.
  ArduinoOTA.handle();
  if (otaActive) return;
#endif

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

#if OTA && UDP_STREAM
  if (wifiOk) udpPoll();
  // Motor commands come from the laptop: disable if it has gone quiet.
  motorWatchdog(peerActive() && (millis() - peerLastSeen) < MOTOR_CMD_TIMEOUT_MS);
#else
  motorWatchdog(false);
#endif

  motorUpdate();

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

  char buf[256];
  int n = snprintf(buf, sizeof(buf),
                   "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%lu,%.3f,%.3f,%d,%.3f,%.1f,%.0f,%.0f,%.3f,%d\n",
                   (unsigned long)now, aRaw[0], aRaw[1], aRaw[2], gRaw[0], gRaw[1], gRaw[2],
                   temp.temperature, (unsigned long)marker, angComp, angKf,
                   mt.enabled ? 1 : 0, mt.cmdA, mt.duty, mt.hallRpm, mt.escRpm, mt.escA, mt.ready);
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  emitCsv(buf, n);

  if (millis() - lastReport > 5000) {
    lastReport = millis();
    char rep[160];
#if OTA
    const char* udpState = "";
  #if UDP_STREAM
    udpState = peerActive() ? "  udp=streaming" : "  udp=idle";
  #endif
    if (WiFi.status() == WL_CONNECTED) {
      snprintf(rep, sizeof(rep), "# overruns=%lu  wifi=connected ip=%s host=%s%s  hall_err=%lu\n",
               (unsigned long)overruns, WiFi.localIP().toString().c_str(), OTA_HOSTNAME, udpState,
               (unsigned long)mt.hallErrors);
    } else {
      snprintf(rep, sizeof(rep), "# overruns=%lu  wifi=NOT connected (status=%d)\n",
               (unsigned long)overruns, (int)WiFi.status());
    }
#else
    snprintf(rep, sizeof(rep), "# overruns=%lu\n", (unsigned long)overruns);
#endif
    emitStatus(rep);
  }

#if OTA && UDP_STREAM
  if (millis() - udpLastFlush >= UDP_FLUSH_MS) udpFlush();
#endif
}
