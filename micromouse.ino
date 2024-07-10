#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"
#include "PIDController.hpp"
#include "Motor.hpp"
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(16,101); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
mtrn3100::IMUOdometry IMU_odometry;
mtrn3100::PIDController controllerL(30, 0, 0);
mtrn3100::PIDController controllerR(30, 0, 0);
mtrn3100::PIDController controllerH(20, 0, 0);
mtrn3100::Motor motorL(11, 12, 150);
mtrn3100::Motor motorR(9, 10, 150);



void setup() {
    Serial.begin(115200);
    Wire.begin();

    // //Set up the IMU
    // byte status = mpu.begin();
    // Serial.print(F("MPU6050 status: "));
    // Serial.println(status);
    // while(status!=0){ } // stop everything if could not connect to MPU6050
    
    // Serial.println(F("Calculating offsets, do not move MPU6050"));
    // delay(1000);
    // mpu.calcOffsets(true,true);
    // Serial.println("Done!\n");
    controllerL.zeroAndSetTarget(0.0, -10);
    controllerR.zeroAndSetTarget(0.0, 10);
    controllerH.zeroAndSetTarget(encoder.getLeftRotation() + encoder.getRightRotation(), 0.0);


    // 1 sec delay before moving
    delay(1000);
}


void loop() {
    int adjustment = controllerH.compute(encoder.getLeftRotation() + encoder.getRightRotation());
    motorL.setPWM(controllerL.compute(encoder.getLeftRotation()) + adjustment); // - adjustment
    motorR.setPWM(controllerR.compute(encoder.getRightRotation()) + adjustment); //  + adjustment
    // motorL.setPWM(-100);
    // motorR.setPWM(100);
    // encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
    delay(50);

    Serial.print("Difference:\t\t");
    Serial.print(adjustment);
    Serial.print(",\t\t");
    Serial.print(controllerL.getError());
    Serial.print(",\t\t");
    Serial.print(controllerR.getError());
    Serial.print(",\t\t");
    Serial.println(); 

    // Serial.print("Encoders:\t\t");
    // Serial.print(encoder.getLeftRotation());
    // Serial.print(",\t\t");
    // Serial.print(encoder.getRightRotation());
    // Serial.print(",\t\t");
    // Serial.print("ODOM:\t\t");
    // Serial.print(encoder_odometry.getX());
    // Serial.print(",\t\t");
    // Serial.print(encoder_odometry.getY());
    // Serial.print(",\t\t");
    // Serial.print(encoder_odometry.getH());
    // Serial.println();

    // Serial.print("ODOM:\t\t");
    // Serial.print(IMU_odometry.getX());
    // Serial.print(",\t\t");
    // Serial.print(IMU_odometry.getY());
    // Serial.println();
}
