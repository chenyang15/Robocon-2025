#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder
// Warning: Total encoders are limited up to 8

// Constructor for child class of MotorControl
MotorControlWithEncoder::MotorControlWithEncoder(byte pin1, byte pwmPin)
    : MotorControl(pin1, pwmPin) {
        ESP32Encoder Encoder;
        currentEncoderCount = 0;
        previousEncoderCount = 0;
        ticksPerSample = 0;
    }

void MotorControlWithEncoder::update_tick_velocity() {
    previousEncoderCount = currentEncoderCount;
    currentEncoderCount = (int)Encoder.getCount();
    this->ticksPerSample = (currentEncoderCount - previousEncoderCount) / MOTOR_WHEEL_ENCODER_PERIOD;
}