#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"
#include "PIDController.hpp"
#include "Motor.hpp"
#include "Drive.hpp"
#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>

MPU6050 mpu(Wire);
VL6180X lidar1;
VL6180X lidar2;
VL6180X lidar3;

const int lidar1_pin = A0;
const int lidar2_pin = A1;
const int lidar3_pin = A2;

#define EN_1_A 2  //These are the pins for the PCB encoder
#define EN_1_B 7  //These are the pins for the PCB encoder
#define EN_2_A 3  //These are the pins for the PCB encoder
#define EN_2_B 8  //These are the pins for the PCB encoder

// left motor is physically reversed, so we flip its direction in code 
// such that a positive rotation on both motors drives the robot forwards
#define MOTOR_L_REVERSED true
#define MOTOR_R_REVERSED false
#define WHEEL_RADIUS 16 // in milimetres

// #define MAX_OUTPUT 150 // maximum pwm output of each pid controller

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B, MOTOR_L_REVERSED, MOTOR_R_REVERSED);
mtrn3100::EncoderOdometry encoder_odometry(16, 101);  //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
mtrn3100::IMUOdometry IMU_odometry;

mtrn3100::Motor motorL(11, 12, MOTOR_L_REVERSED);
mtrn3100::Motor motorR(9, 10, MOTOR_R_REVERSED);
mtrn3100::Drive drive(motorL, motorR, encoder, mpu, lidar1, lidar2, lidar3);

unsigned long timer = 0;
bool c_cmd = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

    /*
        3-LIDAR SETUP PROCESS
    */
    // SET UP ENABLE PINS AND DISABLE SENSORS
    pinMode(lidar1_pin, OUTPUT);
    pinMode(lidar2_pin, OUTPUT);
    pinMode(lidar3_pin, OUTPUT);
    digitalWrite(lidar1_pin, LOW);
    digitalWrite(lidar2_pin, LOW);
    digitalWrite(lidar3_pin, LOW);

    // ENABLE FIRST SENSOR AND CHANGE THE ADDRESS 
    digitalWrite(lidar1_pin, HIGH);
    delay(50);
    lidar1.init();
    lidar1.configureDefault();
    lidar1.setTimeout(250);
    lidar1.setAddress(0x54);
    delay(50);
    
    // ENABLE SECOND SENSOR AND CHANGE THE ADDRESS 
    // NOTE: WE DO NOT HAVE TO DISABLE THE FIRST SENSOR AS IT IS NOW ON A DIFFERENT ADDRESS 
    digitalWrite(lidar2_pin, HIGH);
    delay(50);
    lidar2.init();
    lidar2.configureDefault();
    lidar2.setTimeout(250);
    lidar2.setAddress(0x56);

        // ENABLE THIRD SENSOR AND CHANGE THE ADDRESS 
    // NOTE: WE DO NOT HAVE TO DISABLE THE SECOND SENSOR AS IT IS NOW ON A DIFFERENT ADDRESS 
    digitalWrite(lidar3_pin, HIGH);
    delay(50);
    lidar3.init();
    lidar3.configureDefault();
    lidar3.setTimeout(250);
    lidar3.setAddress(0x58);


    //Set up the IMU
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(500);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");
}


void loop() {
    drive.rotate(90);

    // if (!c_cmd) { //ini on line 42
    //         // drive.straight(4);
    //         // drive.chain_move("flflffrfrflflfrf");
    //     c_cmd = 1;
    // }

    delay(800); 

    // if ((millis() - timer) > 50) {
    //     Serial.print("IMU Heading: ");
    //     Serial.print(mpu.getAngleZ());
    //     Serial.print("\t\t");
    //     Serial.print(lidar1.readRangeSingleMillimeters());
    //     Serial.print(" | ");
    //     Serial.print(lidar2.readRangeSingleMillimeters());
    //     Serial.print(" | ");
    //     Serial.print(lidar3.readRangeSingleMillimeters());
    //     Serial.println();
    //     if (lidar1.timeoutOccurred()) { Serial.print("Sensor 1 TIMEOUT"); }
    //     if (lidar2.timeoutOccurred()) { Serial.print("Sensor 2 TIMEOUT"); }
    //     if (lidar3.timeoutOccurred()) { Serial.print("Sensor 3 TIMEOUT"); }
        
    //     timer = millis();
    // }

    // double wallAdjustment = (lidar3.readRangeSingleMillimeters() - 70.0) / 20.0;

    // int adjustment = controllerH.compute(encoder.getRightRotation() - encoder.getLeftRotation() + wallAdjustment);
    // motorL.setPWM(controllerL.compute(encoder.getLeftRotation()) - adjustment);   // - adjustment
    // motorR.setPWM(controllerR.compute(encoder.getRightRotation()) + adjustment);  //  + adjustment
    // // motorL.setPWM(-100);
    // // motorR.setPWM(100);
    // // encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
    // delay(10);




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
