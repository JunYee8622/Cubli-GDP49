#ifndef motor_h
#define motor_h

#include "Arduino.h"

#include "../definitions/parameters.h"

// Motor class
class Motor {
    public:
        // Class constructor
        Motor(int pin_enable, int pin_current);
        // Start PWM output at the neutral value (call first thing in setup(), before anything slow such as WiFi)
        void begin();
        // Initialize (also spins the motor in both directions very briefly)
        void init();
        // Set current (A)
        void set_current(float ia);
        // Set torque (Nm)
        void set_torque(float tau);

    private:
        // Pins
        int pin_enable, pin_current;
};

#endif
