// Toggle over-the-air (WiFi) programming functionality (0 = disabled, 1 = enabled)
#define OTA 1

// Stream telemetry over WiFi (UDP) to log_udp.py on the laptop (0 = disabled, 1 = enabled). Requires OTA 1.
// Set to 0 for a run where you want the controller to have the CPU entirely to itself.
#define UDP_STREAM 1
#define UDP_PORT 4210
#define TELEMETRY_HZ 100

// Which ESCON slot on the motherboard the single motor is plugged into (1, 2 or 3) for the 1D test.
// The ESCON only sees a valid PWM signal on its own slot's pins; a different slot shows a blinking red LED.
#define MOTOR_SLOT 1

// Wheel speed limit (rpm). Above this, any torque that would speed the wheel up further is set to zero; torque that
// slows it down is still allowed.
#define MAX_WHEEL_RPM 3000

// Toggle between use of nonlinear or linear controller (0 = linear, 1 = nonlinear)
#define USE_NONLINEAR_CONTROLLER 1

// TickTwo provides ESP32-compatible timers
#include <TickTwo.h>
#include "ICM20948_WE.h"

#if OTA
  // Libraries related to wireless functionality
  #include <WiFi.h>
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
#endif

// Cube controller library
#include "src/cube-controller.h"

#if OTA
  // WiFi settings
  const char* wifi_ssid = "Jun Yee";
  const char* wifi_password = "12345678900";
#endif

// Pins of the slot the motor is plugged into (see MOTOR_SLOT above)
#if MOTOR_SLOT == 1
  #define MOTOR_ENABLE_PIN M1_ENABLE
  #define MOTOR_CURRENT_PIN M1_CURRENT
  #define MOTOR_SPEED_PIN M1_SPEED
#elif MOTOR_SLOT == 2
  #define MOTOR_ENABLE_PIN M2_ENABLE
  #define MOTOR_CURRENT_PIN M2_CURRENT
  #define MOTOR_SPEED_PIN M2_SPEED
#else
  #define MOTOR_ENABLE_PIN M3_ENABLE
  #define MOTOR_CURRENT_PIN M3_CURRENT
  #define MOTOR_SPEED_PIN M3_SPEED
#endif

// Instantiation of objects (motor1 / whe_est_1 are the motor and wheel of the 1D test, whichever slot they are in)
Motor motor1(MOTOR_ENABLE_PIN, MOTOR_CURRENT_PIN);//, motor2(M2_ENABLE, M2_CURRENT), motor3(M3_ENABLE, M3_CURRENT);
WheelEstimator whe_est_1(MOTOR_SPEED_PIN), whe_est_2(M2_SPEED), whe_est_3(M3_SPEED);
AttitudeEstimator att_est(IMU_SDA, IMU_SCL);
AttitudeWheelController cont(USE_NONLINEAR_CONTROLLER);
AttitudeTrajectory att_tra;

// Run cube controller at the frequency as specified in parameter file
void controller();
TickTwo timer(controller, dt_us, 0, MICROS_MICROS);

// Status LED control: LED is either solid or blinking at 2.5 Hz
void control_led();
TickTwo timer_led(control_led, 200, 0, MILLIS);

#if OTA
  // Handle OTA flashing requests every 5 seconds
  void handle_ota();
  TickTwo timer_ota(handle_ota, 5000, 0, MILLIS);
#endif

#if OTA && UDP_STREAM
  // Telemetry: send one CSV line to the laptop at TELEMETRY_HZ. Runs from loop(), never inside controller().
  void telemetry();
  TickTwo timer_tlm(telemetry, 1000 / TELEMETRY_HZ, 0, MILLIS);

  WiFiUDP udp;
  IPAddress peer_ip;
  uint16_t peer_port = 0;
  uint32_t peer_last_seen = 0;
  uint32_t status_last_sent = 0;
  unsigned int marker = 0;

  // The laptop sends "hello" every second; the ESP32 only streams while it has heard one in the last 5 seconds
  const uint32_t PEER_TIMEOUT_MS = 5000;
  const char CSV_HEADER[] = "t_us,qw,qx,qy,qz,gx_dps,gy_dps,gz_dps,phi_deg,wheel_rpm,wheel_rad,tau_Nm,armed,halted,marker,"
    "stopped\n";
#endif

// Quaternion and angle error
float qe0, qe1, qe2, qe3;
float phi;
float phi_lim = phi_min;

// Trajectory initialization flag
bool flag_tra = false;

// LED status when blinking
bool led_status = false;

// Security flags
bool flag_arm = false;
bool flag_terminate = false;

// Remote stop from log_udp.py ("x"): motor torque is forced to zero until a remote "g" (go) is received
bool flag_remote_stop = false;

// True while the wheel speed limit (MAX_WHEEL_RPM) is cutting the torque
bool flag_rpm_limit = false;

// Torques
float tau_1 = 0, tau_2 = 0, tau_3 = 0;

void setup() {
  // Start the motor PWM at the neutral value (0 A) before anything slow like WiFi, so the ESCON never sees an
  // out-of-range PWM set value
  motor1.begin();

  // Open serial connection
  Serial.begin(921600);
  Serial.println("This is the ESP32 cube controller.");

  #if OTA
    // Set up WiFi connection
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);

    unsigned int nAttempts = 0;
    while (WiFi.waitForConnectResult() != WL_CONNECTED && nAttempts < 3) {
      delay(1000);
      nAttempts++;
    }

    // If WiFi connection failed, indicate this using six yellow flashes
    if(WiFi.waitForConnectResult() != WL_CONNECTED) {
      for(int i = 0; i < 5; i++) {
        neopixelWrite(LED_PIN, 255, 255, 0);
        delay(250);
        neopixelWrite(LED_PIN, 0, 0, 0);
        delay(250);
      }
    }

    // Set hostname
    ArduinoOTA.setHostname("Cube ESP32");

    // Print WiFi status
    Serial.print("Connected to WiFi (local IP is ");
    Serial.print(WiFi.localIP());
    Serial.println(").");

    // Set up OTA flashing using ArduinoOTA
    ArduinoOTA.begin();

    #if UDP_STREAM
      // Listen for the laptop's log_udp.py (only possible when WiFi connected)
      if(WiFi.status() == WL_CONNECTED) {
        udp.begin(UDP_PORT);
        Serial.print("Telemetry: listening for log_udp.py on UDP port ");
        Serial.println(UDP_PORT);
      }
    #endif
  #endif

  // Delay for one second to allow Maxon ESCON drivers to fully initialize
  delay(1000);

  // Motor setup (also spins each motor in positive direction very briefly)
  motor1.init();
  // motor2.init();
  // motor3.init();

  // Wheel estimator initialization (also calibrates the hall sensor for each wheel)
  whe_est_1.init();
  // whe_est_2.init();
  // whe_est_3.init();

  // Attitude estimator initialization (also calibrates the gyroscope)
  att_est.init();

  // Start controller and LED timers
  timer.start();
  timer_led.start();

  #if OTA
    // OTA handler timer
    timer_ota.start();
  #endif

  #if OTA && UDP_STREAM
    timer_tlm.start();
  #endif
}

void loop() {
  // Update controller timer
  timer.update();
  timer_led.update();

  #if OTA
    // Handle OTA updates
    timer_ota.update();
  #endif

  #if OTA && UDP_STREAM
    timer_tlm.update();
  #endif
}

#if OTA
  void handle_ota(){
    ArduinoOTA.handle();
  }
#endif

#if OTA && UDP_STREAM
  // Send one UDP packet to the laptop (peer) containing the given text
  static void udp_send(const char* text, size_t len) {
    udp.beginPacket(peer_ip, peer_port);
    udp.write((const uint8_t*)text, len);
    udp.endPacket();
  }

  void telemetry() {
    if(WiFi.status() != WL_CONNECTED) return;

    // Read messages from the laptop: "hello" (keepalive), "h" (send header again), "m" (marker), "x" (stop the motor),
    // "g" (go: allow the controller to arm again)
    while(udp.parsePacket() > 0) {
      char msg[16];
      int n = udp.read(msg, sizeof(msg) - 1);
      if(n < 0) n = 0;
      msg[n] = '\0';

      // Only one laptop at a time: ignore everyone else until the current one has been silent for PEER_TIMEOUT_MS
      bool active = (peer_port != 0) && (millis() - peer_last_seen < PEER_TIMEOUT_MS);
      IPAddress sender_ip = udp.remoteIP();
      uint16_t sender_port = udp.remotePort();
      bool same = (sender_ip == peer_ip && sender_port == peer_port);
      if(active && !same) {
        // Tell the other window why it gets no data (at most once every 5 seconds)
        static uint32_t busy_last = 0;
        if(millis() - busy_last >= 5000) {
          busy_last = millis();
          char busy[128];
          int bn = snprintf(busy, sizeof(busy), "# BUSY: already streaming to %s:%u. Close the other log_udp.py first.\n",
            peer_ip.toString().c_str(), (unsigned)peer_port);
          udp.beginPacket(sender_ip, sender_port);
          udp.write((const uint8_t*)busy, bn);
          udp.endPacket();
        }
        continue;
      }

      peer_ip = udp.remoteIP();
      peer_port = udp.remotePort();
      peer_last_seen = millis();

      if(!active || strcmp(msg, "h") == 0) {
        udp_send(CSV_HEADER, sizeof(CSV_HEADER) - 1);
      }
      if(strcmp(msg, "m") == 0) {
        marker++;
      } else if(strcmp(msg, "x") == 0) {
        // Stop: the controller sees this on its next step, sets the torque to zero and the ESCON is disabled
        flag_remote_stop = true;
        const char reply[] = "# STOP: motor torque off (coasting). Send g to allow the controller to run again.\n";
        udp_send(reply, sizeof(reply) - 1);
      } else if(strcmp(msg, "g") == 0) {
        flag_remote_stop = false;
        if(!flag_arm) {
          // Same as a reset: the controller arms again only once the cube is within phi_min of the balance point
          flag_terminate = false;
          flag_tra = false;
          phi_lim = phi_min;
        }
        const char reply[] = "# GO: controller enabled. It arms when the cube is within 5 deg of the balance point.\n";
        udp_send(reply, sizeof(reply) - 1);
      }
    }

    // Nobody is listening
    if(peer_port == 0 || millis() - peer_last_seen >= PEER_TIMEOUT_MS) return;

    // Status line every 5 seconds (lines starting with # are shown by log_udp.py but not saved as data)
    if(millis() - status_last_sent >= 5000) {
      status_last_sent = millis();
      char status[96];
      int n = snprintf(status, sizeof(status), "# motor1=slot M%d armed=%d halted=%d stopped=%d rpm_limit=%d wifi_rssi=%d\n",
        MOTOR_SLOT, flag_arm, flag_terminate, flag_remote_stop, flag_rpm_limit, WiFi.RSSI());
      udp_send(status, n);
    }

    // Data line
    char line[256];
    int n = snprintf(line, sizeof(line), "%lu,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.2f,%.4f,%.5f,%d,%d,%u,%d\n",
      (unsigned long)micros(), att_est.q0, att_est.q1, att_est.q2, att_est.q3,
      att_est.omega_x * 180.0 / pi, att_est.omega_y * 180.0 / pi, att_est.omega_z * 180.0 / pi,
      phi * 180.0 / pi, whe_est_1.omega_w * 30.0 / pi, whe_est_1.theta_w, tau_1,
      flag_arm, flag_terminate, marker, flag_remote_stop);
    if(n > 0) udp_send(line, n);
  }
#endif

void controller() {
  // Estimate wheel velocities
  whe_est_1.estimate(tau_1);
  //whe_est_2.estimate(tau_2);
  //whe_est_3.estimate(tau_3);

  // Estimate cube attitude
  att_est.estimate();

  // Calculate rotation quaternion error
  qe0 = att_est.q0 * att_tra.qr0 + att_est.q1 * att_tra.qr1 + att_est.q2 * att_tra.qr2 + att_est.q3 * att_tra.qr3;
  qe1 = att_est.q0 * att_tra.qr1 - att_est.q1 * att_tra.qr0 - att_est.q2 * att_tra.qr3 + att_est.q3 * att_tra.qr2;
  qe2 = att_est.q0 * att_tra.qr2 - att_est.q2 * att_tra.qr0 + att_est.q1 * att_tra.qr3 - att_est.q3 * att_tra.qr1;
  qe3 = att_est.q0 * att_tra.qr3 - att_est.q1 * att_tra.qr2 + att_est.q2 * att_tra.qr1 - att_est.q3 * att_tra.qr0;

  // Normalize rotation quaternion error (real part only since we don't need the rest)
  qe0 /= sqrt(qe0 * qe0 + qe1 * qe1 + qe2 * qe2 + qe3 * qe3);

  // Calculate error angle
  phi = 2.0 * acos(qe0);

  // Controller enable and disable logic. The controller only activates once its orientation is such that the error with
  // respect to the desired orientation is smaller than phi_min. As soon as this is reached, the error tolerance phi_lim
  // is increased to phi_max. If this error is exceeded, the controller is disabled until the chip is reset.
  if(abs(phi) <= phi_lim && !flag_terminate && !flag_remote_stop) {
    // Controller is active
    flag_arm = true;

    // Increase error range to the maximum limit
    phi_lim = phi_max;

    // Generate trajectory
    // Comment out the generate() method if the cube is balancing on a side instead of a corner
    if(!flag_tra) {
      flag_tra = true;
      att_tra.init();
    }
    //att_tra.generate();

    // Controller calculates motor torques based on cube and wheel states
    cont.control(att_tra.qr0, att_tra.qr1, att_tra.qr2, att_tra.qr3, att_est.q0, att_est.q1, att_est.q2, att_est.q3,
      att_tra.omega_r_x, att_tra.omega_r_y, att_tra.omega_r_z, att_est.omega_x, att_est.omega_y, att_est.omega_z, 
      att_tra.alpha_r_x, att_tra.alpha_r_y, att_tra.alpha_r_z, whe_est_1.theta_w, whe_est_2.theta_w, 
      whe_est_3.theta_w, whe_est_1.omega_w, whe_est_2.omega_w, whe_est_3.omega_w);

    // Get motor torques from controller
    tau_1 = cont.tau_1;
    tau_2 = cont.tau_2;
    tau_3 = cont.tau_3;
  } else {
    // Outside safe limits: reset motor torques to zero
    tau_1 = 0.0;
    tau_2 = 0.0;
    tau_3 = 0.0;

    // Disarm mechanism
    if(flag_arm) {
      flag_arm = false;
      flag_terminate = true;
    }
  }

  // Wheel speed limit: if the wheel is faster than MAX_WHEEL_RPM, do not apply torque that speeds it up even more
  // (positive torque speeds up a positive speed, see the wheel estimator). Braking torque is still allowed.
  float rpm_1 = whe_est_1.omega_w * 30.0 / pi;
  flag_rpm_limit = (abs(rpm_1) > MAX_WHEEL_RPM) && (tau_1 * rpm_1 > 0.0);
  if(flag_rpm_limit) {
    tau_1 = 0.0;
  }

  // Apply torques to motors
  // When balancing on the x side, comment out motor 2 and 3. When balancing on the y side, comment out 1 and 3.
  motor1.set_torque(tau_1);
  //motor2.set_torque(tau_2);
  //motor3.set_torque(tau_3);
}

void control_led() {
  if(flag_remote_stop) {
    // Stopped remotely from the laptop: solid yellow
    neopixelWrite(LED_PIN, 255, 255, 0);
  } else if(flag_arm) {
    if(abs(phi) <= phi_lim - 10 * pi / 180) {
      // Cube is not close to error limit: solid green LED
      neopixelWrite(LED_PIN, 0, 255, 0);
    } else {
      // Cube is close to error limit: alternate between red and green. Cube should be manually put to rest.
      if(led_status) {
        neopixelWrite(LED_PIN, 255, 0, 0);
      } else {
        neopixelWrite(LED_PIN, 0, 255, 0);
      }
      led_status = !led_status;
    }
  } else {
    if(flag_tra) {
      // A trajectory was already generated so the cube must now be halted due to a limit error
      if(led_status) {
        neopixelWrite(LED_PIN, 0, 0, 0);
      } else {
        neopixelWrite(LED_PIN, 255, 0, 0);
      }
      led_status = !led_status;
    } else {
      // Show a solid red LED to indicate readiness to start
      neopixelWrite(LED_PIN, 255, 0, 0);
    }
  }
}
