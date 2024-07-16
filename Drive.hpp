// DRIVING OPERATIONS LIBRARY

// it contains the two motor objects motorL and motorR,
// the dual encoder object "encoder",
// and all the PID controllers to be tuned.

#pragma once

#include <Arduino.h>

#include "Motor.hpp"
#include "PIDController.hpp"
#include "DualEncoder.hpp"
#include <MPU6050_light.h>

#define WHEEL_RADIUS 16.0   // wheel radius of robot
#define ROBOT_RADIUS 52.8     // axle radius of robot
#define ERROR_MARGIN_STR 0.2    // stop controllers when wheel rotation error is reduced to ERROR_MARGIN radians
#define ERROR_MARGIN_ROT 1.2
#define MAX_OUTPUT 150      // maximum pwm output of each pid controller

// TUNE PID CONTROLLERS HERE
mtrn3100::PIDController controllerL(90, 0, 3, MAX_OUTPUT);
mtrn3100::PIDController controllerR(90, 0, 3, MAX_OUTPUT);
mtrn3100::PIDController controllerH(25, 0, 0, MAX_OUTPUT);

mtrn3100::PIDController controllerLRot(9, 0, 0.035, MAX_OUTPUT);
mtrn3100::PIDController controllerRRot(9, 0, 0.035, MAX_OUTPUT);

namespace mtrn3100 {
    class Drive {
    public:
        Drive(mtrn3100::Motor motorL, mtrn3100::Motor motorR, mtrn3100::DualEncoder& encoder, MPU6050& mpu)
            : motorL(motorL), motorR(motorR), encoder(encoder), mpu(mpu) {
        }

        // drive straight for a specified distance in mm, only using encoders and PID.
        straight(double distance) {
            gyroOffset = mpu.getAngleZ();
            controllerL.zeroAndSetTarget(encoder.getLeftRotation(), dist2rot(distance));
            controllerR.zeroAndSetTarget(encoder.getRightRotation(), dist2rot(distance));
            controllerH.zeroAndSetTarget(encoder.getRightRotation() - encoder.getLeftRotation(), 0.0);

            while (true) {
                mpu.update();

                int adjustment = controllerH.compute(encoder.getRightRotation() - encoder.getLeftRotation()); // find diff. in rotation between left and right wheel
                motorL.setPWM(controllerL.compute(encoder.getLeftRotation()) - adjustment);   // - adjustment
                motorR.setPWM(controllerR.compute(encoder.getRightRotation()) + adjustment);  //  + adjustment
                // encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

                // Serial.print(encoder.getLeftRotation());
                // Serial.print('\t');
                // Serial.print(encoder.getRightRotation());
                // Serial.println();

                delay(50);

                if (abs(controllerL.getError()) < ERROR_MARGIN_STR && abs(controllerR.getError()) < ERROR_MARGIN_STR) break;
            }
            // Serial.println("target reached!");
            motorL.setPWM(0);
            motorR.setPWM(0);
        }

        // 0 radius turn using the IMU.
        rotate(double angle) {
            mpu.update();
            
            // we want to rotate until (mpu.getAngleZ() - gyroOffset) == angle
            // more accurately, abs((mpu.getAngleZ() - gyroOffset) - angle) < ERROR_MARGIN
            // gyroOffset = mpu.getAngleZ();

            controllerLRot.zeroAndSetTarget(mpu.getAngleZ(), angle);
            controllerRRot.zeroAndSetTarget(mpu.getAngleZ(), angle);

            while (true) {
                mpu.update();
                // double currentAngle = mpu.getAngleZ() - gyroOffset;

                // motorL.setPWM(currentAngle - angle);
                // motorR.setPWM(angle - currentAngle);

                motorL.setPWM(-controllerLRot.compute(mpu.getAngleZ()));
                motorR.setPWM(controllerRRot.compute(mpu.getAngleZ()));

                Serial.println(controllerLRot.getError());
                // Serial.println(controllerR.getError());
                delay(50);

                if (abs(controllerLRot.getError()) < ERROR_MARGIN_ROT && abs(controllerRRot.getError()) < ERROR_MARGIN_ROT) break;
            }
            motorL.setPWM(0);
            motorR.setPWM(0);
        }

        // rotates ACW the specified angle in degrees. If angle is negative, rotates CW.
        // rotate(double angle) {
        //     gyroOffset = mpu.getAngleZ();

        //     double angleRad = angle * PI / 180;
        //     double wheelRotation = angleRad * ROBOT_RADIUS / WHEEL_RADIUS;
        //     controllerLRot.zeroAndSetTarget(encoder.getLeftRotation(), -wheelRotation);
        //     controllerRRot.zeroAndSetTarget(encoder.getRightRotation(), wheelRotation);
        //     // since rotation radius is 0, keep SUM of left and right encoder constant.
        //     controllerH.zeroAndSetTarget(encoder.getRightRotation() + encoder.getLeftRotation(), 0);

        //     while (true) {
        //         // if value increases, wheels are forward biased (forward turning wheel is stronger). Adjustment will then be negative.
        //         int adjustment = controllerH.compute(encoder.getRightRotation() + encoder.getLeftRotation());
        //         motorL.setPWM(controllerLRot.compute(encoder.getLeftRotation()) + adjustment);
        //         motorR.setPWM(controllerRRot.compute(encoder.getRightRotation()) + adjustment);
        //         Serial.println(controllerLRot.getError());
        //         Serial.println(controllerRRot.getError());
        //         delay(50);

        //         if (controllerLRot.getError() < ERROR_MARGIN_ROT && controllerRRot.getError() < ERROR_MARGIN_ROT) break;
        //     }
        // }
        
        void chain_move(String cmd) {
            String simplified_cmd = "";
            int repeat_count[cmd.length()] = {0};
            int count_index = 0;

            // Simplify the input command and create the repeat count array
            for (int i = 0; i < cmd.length(); i++) {
                char action = cmd.charAt(i);
                if (simplified_cmd.length() == 0 || action != simplified_cmd.charAt(simplified_cmd.length() - 1)) {
                    simplified_cmd += action;
                    repeat_count[count_index++] = 1;
                } else {
                    repeat_count[count_index - 1]++;
                }
        
            }

            // Execute the commands based on the simplified command and repeat count array
            for (int i = 0; i < simplified_cmd.length(); i++) {
                char action = simplified_cmd.charAt(i);
                int repeat = repeat_count[i];
                switch (action) {
                case 'f':
                    straight(250 * repeat);
                    break;
                case 'l':
                    rotate(90 * repeat);
                    break;
                case 'r':
                    rotate(-90 * repeat);
                    break;
                }
                if (i < simplified_cmd.length() - 1) {
                    delay(1000); 
                }
            }
        }

    private:
        double dist2rot(double distance) {
            return distance / WHEEL_RADIUS;
        }

        mtrn3100::Motor motorL, motorR;
        mtrn3100::DualEncoder& encoder;
        MPU6050& mpu;
        double gyroOffset = 0.0;
    };
}