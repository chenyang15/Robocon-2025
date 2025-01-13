#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder
#include "PID.h"

#define MAX_ACCELERATION 9999999 // mm/s^2
#define MAX_VELOCITY     9999999 // mm/s

#define PWM_RES 12
#define PWM_MAX_BIT ((1 << PWM_RES) - 1)    // equivalent to 2^PWM_RES - 1
#define PWM_FREQ 10000              // test 1-20kHz range

class Motor {
private:
    uint8_t motorDirPin;
    uint8_t motorPwmPin;
    double currentDutyCycle;    // Duty cycle (Range: 0~100)
    static uint8_t pwmChannelsUsed;
    uint8_t pwmChannel;

public:
    // Constructor
    Motor(uint8_t pin1, uint8_t pwmPin, double maxPwmIncrement);

    // Members
    double maxPwmIncrement;     // Range: 0~100

    // Methods
    void set_motor_PWM(double dutyCycle);
    void stop_motor();
    void test_motor(double dutyCycle);
    void _ramp_PWM(double dutyCycle);
};

// Subclass of Motor
class MotorWithEncoder : public Motor {
private:
    ESP32Encoder Encoder;
    int32_t currentEncoderCount;
    int32_t previousEncoderCount;
public:
    // Constructor
    MotorWithEncoder(uint8_t pin1, uint8_t pwmPin, uint8_t encoderA, uint8_t encoderB, double maxPwmIncrement, double kp, double ki, double kd, double outputMin = -100.0, double outputMax = 100.0);

    PID_Controller PID;
    double ticksPerSample;
    
    void update_tick_velocity();

};

// Initialize static member in the .cpp file
uint8_t Motor::pwmChannelsUsed = 0;  // Static member initialization

inline int duty_cycle_to_PWM(double dutyCycle);
void forward_hard_coded_with_encoder(double initialPWM, double maxPWM, double rampUpTimeMs, double maxSpeedTime, double rampDownTimeMs, MotorWithEncoder (&wheelMotors)[4]);
void test_all_wheel_motors(Motor* UL_motor, Motor* UR_motor, Motor* BL_motor, Motor* BR_motor);
void forward_hard_coded(double initialPWM, double maxPWM, double rampTime, double duration, Motor wheelMotors[4]);
void ramp_wheel_PWM(MotorWithEncoder (&wheelMotors) [4], double (&wheelMotorPWMs) [4]);
void input_shaping(double (&wheelMotorInputs) [4], double (&previousWheelMotorInputs) [4], MotorWithEncoder& wheelMotor);