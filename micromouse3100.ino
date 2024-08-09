#include "Display.hpp"
#include "DualEncoder.hpp"
#include "PIDController.hpp"
#include "Motor.hpp"
#include "Drive.hpp"
#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <Adafruit_SSD1306.h>

// #defines for the display module
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 16
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
mtrn3100::Display myDisplay(display); // use this library to print easily

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
#define WHEEL_RADIUS 8 // in milimetres

// #define MAX_OUTPUT 150 // maximum pwm output of each pid controller

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B, MOTOR_L_REVERSED, MOTOR_R_REVERSED);

mtrn3100::Motor motorL(11, 12, MOTOR_L_REVERSED);
mtrn3100::Motor motorR(9, 10, MOTOR_R_REVERSED);
mtrn3100::Drive drive(motorL, motorR, encoder, mpu, lidar1, lidar2, lidar3, myDisplay);

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
    //Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(500);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");


    // Set up the display using library functions
    myDisplay.initialise();
}

void loop() {

    if (!c_cmd) { //ini on line 42
        // drive.chain_move("frff");
        drive.precise_command("f23r51f38l8f97r39f102l35f137r37f108r6f271l46f122r34f55l41f96r39f73l24f38r16f32r5f62l28f4");
        // drive.rotate(-90);
        // drive.rotate(-90);
        // drive.rotate(90);
        // drive.rotate(90);
        // drive.straight(4);
        // Serial.print("Distance traveled: ");
        // Serial.println(drive.getOdometry().getCurrentTotalDistance());
        // Serial.print("Heading: ");
        // Serial.println(drive.getOdometry().getCurrentHeading());

        // drive.rotate(90); // Rotate by 90 degrees
        // Serial.print("Heading after rotation: ");
        // Serial.println(drive.getOdometry().getCurrentHeading());

        // drive.straight(250.0); // Move another cell
        // Serial.print("Distance traveled: ");
        // Serial.println(drive.getOdometry().getCurrentTotalDistance());
        // Serial.print("Heading: ");
        // Serial.println(drive.getOdometry().getCurrentHeading());
        c_cmd = 1;
    }

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
}
