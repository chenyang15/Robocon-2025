#include <Arduino.h>
#include "Motor.h"
#include "PinAssignment.h"
#include "Utils.h"

#define MAX_ACCELERATION 1000 // mm/s^2
#define MAX_VELOCITY     1000 // mm/s
#define PWMResolutionMaxValue 255

// Constructor
MotorControl::MotorControl(byte pin1, byte pin2, byte pwmPin)
    : motorPin1(pin1), motorPin2(pin2), motorPWM(pwmPin){
        // Pin Initialisation
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
        pinMode(pwmPin, OUTPUT);

        // TODO: Find max acceleration and then find the max the rate of change of PWM
        // Note: Requires PWM to speed mapping
    }

// Method to set motor speed and direction
void MotorControl::set_motor_PWM(int dutyCycle) {
    dutyCycle = constrain(dutyCycle * driveDir, -PWMResolutionMaxValue, PWMResolutionMaxValue);
    if (dutyCycle > 0) {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        analogWrite(motorPWM, abs(dutyCycle));
    } else if (dutyCycle < 0) {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        analogWrite(motorPWM, abs(dutyCycle));
    } else {
        stop_motor();
    }
}

// Method to stop the motor
void MotorControl::stop_motor() {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, HIGH);
    analogWrite(motorPWM, 0);
}

// (Blocking) Method to test motor
void MotorControl::test_motor(int direction) {
    // Array of PWM values to test
    int pwmValues[] = {20, 40, 60, 80, 100};

    for (int i = 0; i < 5; i++) {
        this->set_motor_PWM(pwmValues[i]); // Set motor PWM
        Serial.print("Testing motor with PWM: ");
        Serial.println(pwmValues[i]);     // Print current PWM value to monitor
        delay(500);                       // Wait 500ms
    }

    // TODO: Implement max PWM rate of change (roc) to get smooth 0 to 100
}

// Method to find max acceleration and then find the max the rate of change of PWM
/*
void MotorControl::find_max_pwm_roc() {


}
*/

// Converts unit of speed from duty cycle to PWM value
inline int duty_cycle_to_PWM(double dutyCycle) {
    return (int) ((dutyCycle * PWMResolutionMaxValue / 100.0) + 0.5);
}

// Test sequentially of all wheel motors
void test_all_wheel_motors(MotorControl* UL_Motor, MotorControl* UR_Motor, MotorControl* BL_Motor, MotorControl* BR_Motor) {
    // Note: Each function is blocking and takes 2.5s
    UL_Motor->test_motor(FORWARD);
    UL_Motor->test_motor(BACKWARD);
    UR_Motor->test_motor(FORWARD);
    UR_Motor->test_motor(BACKWARD);
    BL_Motor->test_motor(FORWARD);
    BL_Motor->test_motor(BACKWARD);
    BR_Motor->test_motor(FORWARD);
    BR_Motor->test_motor(BACKWARD);
}

void wheel_motor_instantiation() {
    MotorControl UL_Motor(
        MOTOR_UL_DIR_1,     // motor Dir Pin 1
        MOTOR_UL_DIR_2,     // motor Dir Pin 2
        MOTOR_UL_PWM        // motor Enable Pin
    );

    MotorControl UR_Motor(
        MOTOR_BL_DIR_1,     // motor Dir Pin 1
        MOTOR_BL_DIR_2,     // motor Dir Pin 2
        MOTOR_BL_PWM        // motor Enable Pin
    );

    MotorControl BL_Motor(
        MOTOR_BR_DIR_1,     // motor Dir Pin 1
        MOTOR_BR_DIR_2,     // motor Dir Pin 2
        MOTOR_BR_PWM        // motor Enable Pin
    );

    MotorControl BR_Motor(
        MOTOR_UR_DIR_1,     // motor Dir Pin 1
        MOTOR_UR_DIR_2,     // motor Dir Pin 2
        MOTOR_UR_PWM        // motor Enable Pin
    );
}

