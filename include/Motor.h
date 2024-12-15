#pragma once

#define MAX_ACCELERATION 9999999 // mm/s^2
#define MAX_VELOCITY     9999999 // mm/s

#define PWM_RES 12
#define PWM_MAX_BIT 4095
#define PWM_FREQ 10000              // test 1-20kHz range
#define MOTOR_ACTUATION_PERIOD 50
#define PWM_DUTY_CYCLE_INCREMENT 0.8           // for motor actuation period of 50ms

class MotorControl {
private:
    // Motor pin variables
    byte motorDirPin;
    byte motorPwmPin;
    double currentDutyCycle; // in bit

public:
    // Constructor
    MotorControl(byte pin1, byte pwmPin);

    // Methods
    void set_motor_PWM(double dutyCycle);
    void stop_motor();
    void test_motor(double dutyCycle);
    void _ramp_PWM(double dutyCycle);
};



inline int duty_cycle_to_PWM(double dutyCycle);
void test_all_wheel_motors(MotorControl* UL_motor, MotorControl* UR_motor, MotorControl* BL_motor, MotorControl* BR_motor);
void forward_hard_coded(double initialPWM, double maxPWM, double rampTime, double duration, MotorControl wheelMotors[4]);
void ramp_wheel_PWM(MotorControl (&wheelMotors) [4], double (&wheelMotorPWMs) [4]);