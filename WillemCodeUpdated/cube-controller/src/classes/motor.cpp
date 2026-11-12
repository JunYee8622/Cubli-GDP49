#include "motor.h"

// Class constructor
Motor::Motor(int pin_enable, int pin_current) : pin_enable(pin_enable), pin_current(pin_current) {
    // Configure enable pin as digital output
    pinMode(pin_enable, OUTPUT);
}

void Motor::begin() {
    // Configure current pin as 800 Hz 12-bit PWM output
    ledcAttach(pin_current, 800, 12);

    // Immediately output the neutral 50 % duty (0 A, driver disabled). Without this the pin sits at 0 % duty, which
    // the ESCON flags as "PWM set value input out of range" (red LED blinking).
    set_current(0);
}

void Motor::init() {
    // Briefly set current to verify motor behaviour
    set_current(-0.5);
    delay(250);
    set_current(0.4);
    delay(250);
    set_current(0);
}

void Motor::set_current(float ia) {
    // Variables
    bool enable;
    float current;

    // Reverse motor current because of definitions (positive corresponds to CCW rotation)
    ia = -ia;
    if (ia == 0.0) {
        // If current is zero, disable motor and set corresponding PWM value
        enable = false;
        current = 0.5;   
    } else {
        // Enable motor
        enable = true;

        // Set PWM value according to current input. Upper and lower 10% are not used for reliability. ESCON drivers are
        // configured accordingly.
        if(ia > ia_max) {
            // Saturation at upper limit
            current = 0.89;
        } else if (ia < -ia_max) {
            // Saturation at lower limit
            current = 0.11;
        } else {
            // Interpolation in valid range
            current = 0.5 + ia * (0.8 / (2.0 * ia_max));
        }
    }
    // Set enable pin
    digitalWrite(pin_enable, enable);

    // Map 0-1 PWM value to 12-bit range (0-4095) and write duty. On Arduino-ESP32 core 3.x ledcWrite() takes the pin
    // number (not a channel number).
    uint32_t duty = current * 4095;
    ledcWrite(pin_current, duty);
}

void Motor::set_torque(float tau) {
    // Convert torque to current using motor torque constant Km
    set_current(tau / Km);
}
