#pragma once

#include <Arduino.h>
#include "math.h"

#define TOP_SPEED 255  // maximum pwm output of motor controller

namespace mtrn3100 {

// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control

// DO NOT SET TOP SPEED IN MOTOR (AS THAT CAUSES SIGNAL CLIPPING).
// SET TOP SPEED IN PID CONTROLLER.
class Motor {
public:
  Motor(uint8_t pwm_pin, uint8_t in2, bool is_reversed)
    : pwm_pin(pwm_pin), dir_pin(in2), is_reversed(is_reversed) {

    pinMode(pwm_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
  }

  void setPWM(int16_t pwm) {

    if (pwm >= 0) {
      digitalWrite(dir_pin, !is_reversed);
    } else {
      digitalWrite(dir_pin, is_reversed);
    }

    int pwm_signal = abs(pwm);
    if (pwm_signal > TOP_SPEED) {
      pwm_signal = TOP_SPEED;
    }


    analogWrite(pwm_pin, pwm_signal);
  }

private:
  const uint8_t pwm_pin;
  const uint8_t dir_pin;
  bool is_reversed;
};

}
