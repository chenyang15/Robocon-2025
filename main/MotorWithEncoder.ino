#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder
// Warning: Total encoders are limited up to 8

// Constructor for child class of MotorControl
MotorWithEncoder::MotorWithEncoder(uint8_t pin1, uint8_t pwmPin, uint8_t encoderA, uint8_t encoderB, double maxPwmIncrement, double kp, double ki, double kd, double outputMin, double outputMax)
    : Motor(pin1, pwmPin, maxPwmIncrement),           // Motor setup (constructor from parent class)
    currentEncoderCount(0), previousEncoderCount(0), ticksPerSample(0), // Encoder variables initialization
    Encoder(true),                          // Create encoder class
    PID(kp, ki, kd, outputMin, outputMax)   // Create PID controller class
    {
    // Encoder setup
    Encoder.attachFullQuad(encoderA, encoderB);
}

void MotorWithEncoder::update_tick_velocity() {
    this->previousEncoderCount = this->currentEncoderCount;
    this->currentEncoderCount = (int)Encoder.getCount();
    this->ticksPerSample = (currentEncoderCount - previousEncoderCount) / MOTOR_WHEEL_ENCODER_PERIOD;
}