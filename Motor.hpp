#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {

// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor( uint8_t pwm_pin, uint8_t in2) :  pwm_pin(pwm_pin), dir_pin(in2) {

        pinMode(pwm_pin, OUTPUT); 
        pinMode(dir_pin, OUTPUT); 

    }
    Motor( uint8_t pwm_pin, uint8_t in2, uint8_t top_speed) :  pwm_pin(pwm_pin), dir_pin(in2), top_speed(top_speed) {

        pinMode(pwm_pin, OUTPUT); 
        pinMode(dir_pin, OUTPUT); 

    }

    void setPWM(int16_t pwm) {

        if (pwm >= 0) {
        digitalWrite(dir_pin, HIGH); 
        } else {
        digitalWrite(dir_pin, LOW);
        }

        int pwm_signal = abs(pwm);
        if (pwm_signal > top_speed) {
          pwm_signal = top_speed;
        }


        analogWrite(pwm_pin, pwm_signal); 
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
    const uint8_t top_speed = 255;
};

} 
