#pragma once

#include <math.h>

#define INTEGRAL_CLAMP_RATIO 0.25 // clamps integral to a proportion of max_value to prevent windup

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float max_output) : kp(kp), ki(ki), kd(kd), max_output(max_output) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {
      
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        error = setpoint - (input - zero_ref);

        // TODO: IMPLIMENT PID CONTROLLER
        integral = integral + error * dt;
        // integral clamping
        if (abs(integral) > (max_output * INTEGRAL_CLAMP_RATIO)) {
            integral = (integral > 0 ? (max_output * INTEGRAL_CLAMP_RATIO) : (-max_output * INTEGRAL_CLAMP_RATIO));
        }
        derivative = (error - prev_error) / dt;

        output = kp * error + ki * integral + kd * derivative;

        prev_error = error;

        // return output;
        if (abs(output) < max_output) return output;
        else return (output > 0) ? max_output : -max_output;
    }

    // Function used to return the last calculated error. 
    // The error is the difference between the desired position and current position. 
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    float getError() {
      return error;
    }

    // This must be called before trying to achieve a setpoint.
    // The first argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;

        error = 0;
        derivative = 0;
        integral = 0;
        output = 0;
        prev_error = 0;
    }

public:
    uint32_t prev_time, curr_time = micros();
    float dt;

private:
    float kp, ki, kd;
    float error, derivative, integral, output;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;
    float max_output = 255; // magnitude of maximum signal output, default clamped at 255.

    
};

}  // namespace mtrn3100
