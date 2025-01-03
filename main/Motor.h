#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder
#include "PID.h"

#define MAX_ACCELERATION 9999999 // mm/s^2
#define MAX_VELOCITY     9999999 // mm/s

#define PWM_RES 12
#define PWM_MAX_BIT 4095
#define PWM_FREQ 10000              // test 1-20kHz range
//#define PWM_DUTY_CYCLE_INCREMENT 0.8           // for motor actuation period of 50ms

class Motor {
private:
    // Motor pin variables
    byte motorDirPin;
    byte motorPwmPin;
    double currentDutyCycle;    // Duty cycle (Range: 0~100)

public:
    // Constructor
    Motor(byte pin1, byte pwmPin, double maxPwmIncrement);

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
    int currentEncoderCount;
    int previousEncoderCount;
public:
    // Constructor
    MotorWithEncoder(byte pin1, byte pwmPin, byte encoderA, byte encoderB, double maxPwmIncrement, double kp, double ki, double kd, double outputMin = -100.0, double outputMax = 100.0);

    PID_Controller PID;
    double ticksPerSample;
    
    void update_tick_velocity();

};


inline int duty_cycle_to_PWM(double dutyCycle);
void test_all_wheel_motors(Motor* UL_motor, Motor* UR_motor, Motor* BL_motor, Motor* BR_motor);
void forward_hard_coded(double initialPWM, double maxPWM, double rampTime, double duration, Motor wheelMotors[4]);
void ramp_wheel_PWM(MotorWithEncoder (&wheelMotors) [4], double (&wheelMotorPWMs) [4]);
void input_shaping(double (&wheelMotorInputs) [4], double (&previousWheelMotorInputs) [4], MotorWithEncoder& wheelMotor);