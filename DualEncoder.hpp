#pragma once

#include <Arduino.h>

namespace mtrn3100 {


class DualEncoder {
public:
  DualEncoder(uint8_t enc1, uint8_t enc2, uint8_t enc3, uint8_t enc4, bool reversed_l, bool reversed_r)
    : mot1_int(enc1), mot1_dir(enc2), mot2_int(enc3), mot2_dir(enc4), reversed_l(reversed_l), reversed_r(reversed_r) {
    instance = this;  // Store the instance pointer
    pinMode(mot1_int, INPUT_PULLUP);
    pinMode(mot1_dir, INPUT_PULLUP);
    pinMode(mot2_int, INPUT_PULLUP);
    pinMode(mot2_dir, INPUT_PULLUP);
    l_prev_time = micros();
    r_prev_time = micros();

    attachInterrupt(digitalPinToInterrupt(mot1_int), DualEncoder::readLeftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(mot2_int), DualEncoder::readRightEncoderISR, RISING);
  }


  // Encoder function used to update the encoder
  void readLeftEncoder() {
    noInterrupts();
    // Serial.println("left encoder triggered");
    unsigned long currTime = micros();
    direction = (digitalRead(mot1_dir) != reversed_l) ? -1 : 1;
    l_count += direction;
    l_velocity = direction * (1000000.0 / counts_per_revolution) / (currTime - l_prev_time);
    l_prev_time = currTime;
    interrupts();
  }

  void readRightEncoder() {
    noInterrupts();
    // Serial.println("right encoder triggered");
    unsigned long currTime = micros();
    direction = (digitalRead(mot2_dir) != reversed_r) ? -1 : 1;
    r_count += direction;
    r_velocity = direction * (1000000.0 / counts_per_revolution) / (currTime - r_prev_time);
    r_prev_time = currTime;
    interrupts();
  }
  float getLeftVelocity() {
    unsigned long currTime = micros();
    if (currTime - l_prev_time > 250000) return 0;
    return l_velocity;
  }

  float getRightVelocity() {
    unsigned long currTime = micros();
    if (currTime - r_prev_time > 250000) return 0;
    return r_velocity;
  }
  // Helper function which to convert encouder count to radians
  float getLeftRotation() {
    return (static_cast<float>(l_count) / counts_per_revolution) * 2 * PI;
  }

  float getRightRotation() {
    return (static_cast<float>(r_count) / counts_per_revolution) * 2 * PI;
  }

private:
  static void readLeftEncoderISR() {
    if (instance != nullptr) {
      instance->readLeftEncoder();
    }
  }

  static void readRightEncoderISR() {
    if (instance != nullptr) {
      instance->readRightEncoder();
    }
  }

public:
  const uint8_t mot1_int, mot1_dir, mot2_int, mot2_dir;
  volatile int8_t direction;
  float position = 0;
  uint16_t counts_per_revolution = 700;
  volatile long l_count = 0;
  volatile long r_count = 0;
  volatile double l_velocity = 0.0;
  volatile double r_velocity = 0.0;
  uint32_t l_prev_time = 0;
  uint32_t r_prev_time = 0;
  bool read = false;

  bool reversed_l = 0;
  bool reversed_r = 0;

private:
  static DualEncoder* instance;
};

DualEncoder* DualEncoder::instance = nullptr;

}  // namespace mtrn3100
