// driving operations library
// it contains the two motor objects motorL and motorR,
// the dual encoder object "encoder",
// and as many PID controllers as necessary.
#pragma once

#include <Arduino.h>

#include "Motor.hpp"
#include "PIDController.hpp"
#include "DualEncoder.hpp"

#define WHEEL_RADIUS 16.0
#define ERROR_MARGIN 0.1 // stop controllers when wheel rotation error is reduced to ERROR_MARGIN radians

namespace mtrn3100 {
    class Drive {
    public:
        Drive(mtrn3100::Motor motorL, mtrn3100::Motor motorR, mtrn3100::DualEncoder encoder) : motorL(motorL), motorR(motorR), encoder(encoder) {
        }

        // drive forwards for a specified distance in mm
        forward(double distance) {
            mtrn3100::PIDController controllerL(30, 0, 0);
            mtrn3100::PIDController controllerR(30, 0, 0);
            mtrn3100::PIDController controllerH(20, 0, 0);

            // radius of wheel is 16 mm
            // intended rotation = distance / radius
            // distance given in mm
            controllerL.zeroAndSetTarget(encoder.getLeftRotation(), distance / WHEEL_RADIUS);
            controllerR.zeroAndSetTarget(encoder.getRightRotation(), distance / WHEEL_RADIUS);
            controllerH.zeroAndSetTarget(0.0, 0.0);
            // Serial.begin(115200);

            while (controllerL.getError() < ERROR_MARGIN && controllerR.getError() < ERROR_MARGIN) {
                int adjustment = controllerH.compute(encoder.getRightRotation() - encoder.getLeftRotation()); // find diff. in rotation between left and right wheel
                motorL.setPWM(controllerL.compute(encoder.getLeftRotation()) - adjustment);   // - adjustment
                motorR.setPWM(controllerR.compute(encoder.getRightRotation()) + adjustment);  //  + adjustment

                // Serial.println("Driving forward");
                delay(50);
            }
        }
    private:
        mtrn3100::Motor motorL, motorR;
        mtrn3100::DualEncoder encoder;
    };
}