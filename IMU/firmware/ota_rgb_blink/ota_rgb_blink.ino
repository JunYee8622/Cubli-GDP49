// ota_rgb_blink.ino — minimal WiFi OTA test for ESP32-S3-DevKitC.
//
// Cycles the onboard RGB LED through rainbow colours and accepts firmware
// uploads over WiFi (Arduino IDE: Tools > Port > "cube-esp32 at 172.20.10.x").
//
// LED meaning:
//   blue blink   = connecting to WiFi
//   red blink    = WiFi failed (keeps retrying every 10 s)
//   rainbow      = connected, OTA ready
//   white/purple = OTA upload in progress
//
// Upload this ONCE over USB. After that, every upload can go over WiFi.
// Keep ArduinoOTA in every future sketch, or you lose WiFi upload again.

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// ---------- WiFi (2.4 GHz only; iPhone hotspot: turn on "Maximise Compatibility") ----------
const char* WIFI_SSID     = "Jun Yee";
const char* WIFI_PASSWORD = "12345678900";
const char* OTA_HOSTNAME  = "cube-esp32";

// ---------- RGB LED ----------
// DevKitC-1 v1.0 uses GPIO48, v1.1 uses GPIO38. RGB_BUILTIN is set by the board package.
#ifdef RGB_BUILTIN
  const int LED_PIN = RGB_BUILTIN;
#else
  const int LED_PIN = 48;
#endif
const uint8_t BRIGHTNESS = 40;        // 0..255, keep low, it is very bright

// ---------- Motor safety: hold all ESCON enables low (Willem's board) ----------
const int ESCON_ENABLE_PINS[] = {13, 17, 5};

// Prototypes
void setup();
void loop();
void setLed(uint8_t r, uint8_t g, uint8_t b);
void wheel(uint8_t pos, uint8_t &r, uint8_t &g, uint8_t &b);
bool connectWiFi(uint32_t timeoutMs);
void setupOTA();

volatile bool otaActive = false;
bool otaStarted = false;

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(LED_PIN, r, g, b);
}

// Colour wheel 0..255 -> r,g,b scaled by BRIGHTNESS
void wheel(uint8_t pos, uint8_t &r, uint8_t &g, uint8_t &b) {
  uint8_t rr, gg, bb;
  if (pos < 85)       { rr = 255 - pos * 3; gg = pos * 3;       bb = 0; }
  else if (pos < 170) { pos -= 85; rr = 0;  gg = 255 - pos * 3; bb = pos * 3; }
  else                { pos -= 170; rr = pos * 3; gg = 0;       bb = 255 - pos * 3; }
  r = (uint16_t)rr * BRIGHTNESS / 255;
  g = (uint16_t)gg * BRIGHTNESS / 255;
  b = (uint16_t)bb * BRIGHTNESS / 255;
}

bool connectWiFi(uint32_t timeoutMs) {
  Serial.printf("WiFi: connecting to \"%s\"...\n", WIFI_SSID);
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  bool on = false;
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
    on = !on;
    setLed(0, 0, on ? BRIGHTNESS : 0);   // blue blink
    delay(250);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi: connected, IP ");
    Serial.println(WiFi.localIP());
    return true;
  }
  Serial.printf("WiFi: FAILED (status %d). Retrying in 10 s.\n", (int)WiFi.status());
  return false;
}

void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA
    .onStart([]() {
      otaActive = true;
      Serial.println("OTA: update starting");
    })
    .onProgress([](unsigned int done, unsigned int total) {
      // alternate white / purple while uploading
      bool w = (done / 16384) % 2;
      setLed(BRIGHTNESS, w ? BRIGHTNESS : 0, BRIGHTNESS);   // white / purple
    })
    .onEnd([]() {
      Serial.println("OTA: done, rebooting");
      setLed(0, BRIGHTNESS, 0);
    })
    .onError([](ota_error_t error) {
      otaActive = false;
      Serial.printf("OTA: error %u\n", (unsigned)error);
      setLed(BRIGHTNESS, 0, 0);
    });
  ArduinoOTA.begin();
  otaStarted = true;
  Serial.printf("OTA: ready as \"%s\" at ", OTA_HOSTNAME);
  Serial.println(WiFi.localIP());
}

void setup() {
  for (int p : ESCON_ENABLE_PINS) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  Serial.begin(115200);
  delay(300);
  Serial.println("\nota_rgb_blink starting");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);               // modem sleep makes OTA unreliable

  if (connectWiFi(15000)) setupOTA();
}

void loop() {
  static uint32_t lastRetry = 0, lastLed = 0, lastPrint = 0;
  static uint8_t hue = 0;
  uint32_t now = millis();

  // WiFi lost or never connected: red blink and retry every 10 s
  if (WiFi.status() != WL_CONNECTED) {
    setLed((now / 250) % 2 ? BRIGHTNESS : 0, 0, 0);
    if (now - lastRetry > 10000) {
      lastRetry = now;
      if (connectWiFi(8000) && !otaStarted) setupOTA();
    }
    return;
  }

  ArduinoOTA.handle();
  if (otaActive) return;

  // Rainbow
  if (now - lastLed >= 20) {
    lastLed = now;
    uint8_t r, g, b;
    wheel(hue++, r, g, b);
    setLed(r, g, b);
  }

  // Status line every 5 s
  if (now - lastPrint >= 5000) {
    lastPrint = now;
    Serial.printf("alive  IP %s  RSSI %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }
}
