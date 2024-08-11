// DRIVING OPERATIONS LIBRARY

// it contains the two motor objects motorL and motorR,
// the dual encoder object "encoder",
// and all the PID controllers to be tuned.

#pragma once

#include <Arduino.h>

// #include "Display.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "DualEncoder.hpp"
#include <MPU6050_light.h>
#include <VL6180X.h>

// robot dimensions
#define WHEEL_RADIUS 16.0  // wheel radius of robot
#define ROBOT_RADIUS 52.8  // axle radius of robot

// PID stats
#define ERROR_MARGIN_STR 0.2  // stop controllers when wheel rotation error is reduced to ERROR_MARGIN radians
#define ERROR_MARGIN_ROT 1.2
#define MAX_OUTPUT 135  // maximum pwm output of each pid controller

// cell dimensions
#define CELL_LENGTH 245.0      // distance between centres of cells
#define LIDAR_AVERAGE 82       // what the lidars should measure when the robot is centered in the cell
#define FRONT_LIDAR_HALT 90    // halt robot if it gets this close to a front wall
#define NO_WALL_THRESHOLD 110  // if lidar value is above this, then assume there is no wall

// TUNE PID CONTROLLERS HERE
mtrn3100::PIDController controllerL(90, 0, 2.5, MAX_OUTPUT);
mtrn3100::PIDController controllerR(90, 0, 2.5, MAX_OUTPUT);
mtrn3100::PIDController controllerH(50, 0, 0, MAX_OUTPUT);

mtrn3100::PIDController controllerLidar(25, 0, 4, MAX_OUTPUT);
mtrn3100::PIDController controllerIMU(9, 0, 0.035, MAX_OUTPUT);

mtrn3100::PIDController controllerLRot(9, 0, 0.035, MAX_OUTPUT);
mtrn3100::PIDController controllerRRot(9, 0, 0.035, MAX_OUTPUT);

namespace mtrn3100 {
class CellOdometry {
public:
  CellOdometry()
    : x(0), y(0), heading(0) {}

  void updatePosition(double distance, double angle_deg) {
    double angle_rad = angle_deg * M_PI / 180.0;
    x += distance * cos(angle_rad);
    y += distance * sin(angle_rad);
    heading = angle_rad;
  }

  double getCurrentTotalDistance() const {
    return sqrt((x * x) + (y * y));
  }

  double getCurrentHeading() const {
    return heading;
  }

private:
  double x;
  double y;
  double heading;
};

class Drive {
public:
  Drive(mtrn3100::Motor motorL, mtrn3100::Motor motorR, mtrn3100::DualEncoder& encoder, MPU6050& mpu, VL6180X& lidar1, VL6180X& lidar2, VL6180X& lidar3, mtrn3100::Display& display)
    : motorL(motorL), motorR(motorR), encoder(encoder), mpu(mpu), lidar1(lidar1), lidar2(lidar2), lidar3(lidar3), display(display) {
  }

  // drive straight for a specified number of cells, only using encoders and PID.
  straightLidarless(double cells) {
    controllerL.zeroAndSetTarget(encoder.getLeftRotation(), dist2rot(cells * CELL_LENGTH));
    controllerR.zeroAndSetTarget(encoder.getRightRotation(), dist2rot(cells * CELL_LENGTH));
    controllerH.zeroAndSetTarget(encoder.getRightRotation() - encoder.getLeftRotation(), 0.0);


    while (true) {
      mpu.update();

      int adjustment = controllerH.compute(encoder.getRightRotation() - encoder.getLeftRotation());  // find diff. in rotation between left and right wheel
      motorL.setPWM(controllerL.compute(encoder.getLeftRotation()) - adjustment);                    // - adjustment
      motorR.setPWM(controllerR.compute(encoder.getRightRotation()) + adjustment);                   //  + adjustment
      // encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

      // Serial.print(encoder.getLeftRotation());
      // Serial.print('\t');
      // Serial.print(encoder.getRightRotation());
      // Serial.println();

      delay(50);

      if (abs(controllerL.getError()) < ERROR_MARGIN_STR && abs(controllerR.getError()) < ERROR_MARGIN_STR) break;
    }

    odometry.updatePosition(cells * CELL_LENGTH, odometry.getCurrentHeading());

    // Serial.println("target reached!");
    motorL.setPWM(0);
    motorR.setPWM(0);
  }

  // drive straight for a specified number of cells, using left and right LIDARs to centre the robot.
  straight(double cells) {
    controllerL.zeroAndSetTarget(encoder.getLeftRotation(), dist2rot(cells * CELL_LENGTH));
    controllerR.zeroAndSetTarget(encoder.getRightRotation(), dist2rot(cells * CELL_LENGTH));
    controllerLidar.zeroAndSetTarget(0, 0.0);
    controllerIMU.zeroAndSetTarget(mpu.getAngleZ(), 0.0);

    unsigned long timer = 0;

    while (true) {
      mpu.update();

      int leftWallDist = lidar1.readRangeSingleMillimeters();
      int frontWallDist = lidar2.readRangeSingleMillimeters();
      int rightWallDist = lidar3.readRangeSingleMillimeters();

      // 4 conditions. LR, L, R and no walls (driving blind).
      // When driving with walls, use wall to calibrate IMU heading.
      // If lidar output > 100 mm, indicates no wall.
      int adjustment;

      // walls on both sides
      if (leftWallDist < NO_WALL_THRESHOLD && rightWallDist < NO_WALL_THRESHOLD) {
        controllerIMU.zeroAndSetTarget(mpu.getAngleZ(), 0.0);
        adjustment = controllerLidar.compute((rightWallDist - leftWallDist) / 2 * 0.04);
        // display.print("WW  WW");
      }
      // wall only on LEFT
      else if (leftWallDist < NO_WALL_THRESHOLD) {
        controllerIMU.zeroAndSetTarget(mpu.getAngleZ(), 0.0);
        adjustment = controllerLidar.compute((LIDAR_AVERAGE - leftWallDist) * 0.04);
        // display.print("WW  __");
      }
      // wall only on RIGHT
      else if (rightWallDist < NO_WALL_THRESHOLD) {
        controllerIMU.zeroAndSetTarget(mpu.getAngleZ(), 0.0);
        adjustment = controllerLidar.compute((rightWallDist - LIDAR_AVERAGE) * 0.04);
        // display.print("__  WW");
      }
      // no wall
      else {
        controllerLidar.zeroAndSetTarget(0, 0.0);
        adjustment = controllerIMU.compute(mpu.getAngleZ());
        // display.print("__  __");
      }

      // int adjustment = controllerH.compute(encoder.getRightRotation() - encoder.getLeftRotation()); // find diff. in rotation between left and right wheel
      motorL.setPWM(controllerL.compute(encoder.getLeftRotation()) - adjustment);   // - adjustment
      motorR.setPWM(controllerR.compute(encoder.getRightRotation()) + adjustment);  //  + adjustment
      // encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

      display.print(leftWallDist, frontWallDist, rightWallDist);

      if (abs(controllerL.getError()) < ERROR_MARGIN_STR || abs(controllerR.getError()) < ERROR_MARGIN_STR) break;
      if (frontWallDist < FRONT_LIDAR_HALT) break;

      delay(100);
    }

    odometry.updatePosition(cells * CELL_LENGTH, odometry.getCurrentHeading());

    // Serial.println("target reached!");
    motorL.setPWM(0);
    motorR.setPWM(0);
  }

  // 0 radius turn using the IMU.
  rotate(double angle) {
    mpu.update();

    controllerLRot.zeroAndSetTarget(mpu.getAngleZ(), angle);
    controllerRRot.zeroAndSetTarget(mpu.getAngleZ(), angle);
    controllerH.zeroAndSetTarget(encoder.getLeftRotation() + encoder.getRightRotation(), 0.0);

    display.print("SPEEN");
    while (true) {
      mpu.update();

      // if value increases, wheels are forward biased (forward turning wheel is stronger). Adjustment will then be negative.
      int adjustment = controllerH.compute(encoder.getRightRotation() + encoder.getLeftRotation());

      motorL.setPWM(-controllerLRot.compute(mpu.getAngleZ()) + adjustment);
      motorR.setPWM(controllerRRot.compute(mpu.getAngleZ()) + adjustment);

      // Serial.println(controllerLRot.getError());
      // Serial.println(controllerR.getError());

      // int leftWallDist = lidar1.readRangeSingleMillimeters();
      // int frontWallDist = lidar2.readRangeSingleMillimeters();
      // int rightWallDist = lidar3.readRangeSingleMillimeters();
      // display.print(leftWallDist, frontWallDist, rightWallDist);

      delay(50);

      if (abs(controllerLRot.getError()) < ERROR_MARGIN_ROT || abs(controllerRRot.getError()) < ERROR_MARGIN_ROT) break;
    }

    odometry.updatePosition(0, angle);

    motorL.setPWM(0);
    motorR.setPWM(0);
  }

  // rotates ACW the specified angle in degrees. If angle is negative, rotates CW.
  // rotateEncoder(double angle) {

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
    int repeat_count[cmd.length()] = { 0 };
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
          straight(repeat);
          break;
        case 'l':
          rotate(90 * repeat);
          break;
        case 'r':
          rotate(-90 * repeat);
          break;
      }
      if (i < simplified_cmd.length() - 1) {
        delay(600);
      }
    }
  }

  void precise_command(String cmd) {
    int current_command = 0;
    while (current_command < cmd.length()) {
    char action = cmd.charAt(current_command);
    current_command++;

    // Extract the number following the action
    String numStr = "";
    while (current_command < cmd.length() && isDigit(cmd.charAt(current_command))) {
        numStr += cmd.charAt(current_command);
        current_command++;
    }
    
    int value = numStr.toInt(); // Convert string to integer

    // Execute the commands based on the action and the extracted value
    switch (action) {
        case 'f':
        straightLidarless(value / CELL_LENGTH);
        display.print("Forward: ");
        break;
        case 'l':
        rotate(value);
        display.print("Left: ");
        break;
        case 'r':
        rotate(-value);
        display.print("Right: ");
        break;
    }
    
    if (current_command < cmd.length()) {
        delay(100); // Delay between commands
        display.print("hold");
    }
    }
  }

  CellOdometry& getOdometry() {
    return odometry;
  }

private:
  double dist2rot(double distance) {
    return distance / WHEEL_RADIUS;
  }

  mtrn3100::Motor motorL, motorR;
  mtrn3100::DualEncoder& encoder;
  mtrn3100::Display& display;
  VL6180X& lidar1;
  VL6180X& lidar2;
  VL6180X& lidar3;
  MPU6050& mpu;
  CellOdometry odometry;
};
}