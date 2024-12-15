#include <Arduino.h>
#include "Motor.h"
#include "PinAssignment.h"
#include "Utils.h"

#define MAX_ACCELERATION 1000 // mm/s^2
#define MAX_VELOCITY     1000 // mm/s

#define PWM_RES 12
#define PWM_MAX_BIT 4095
#define PWM_FREQ 10000        // test 1-20kHz range

// Constructor
MotorControl::MotorControl(byte pin1, byte pwmPin)
    : motorDirPin(pin1), motorPwmPin(pwmPin) {
        // Pin Initialisation
        pinMode(pin1, OUTPUT);
        //pinMode(pwmPin, OUTPUT);
        ledcSetup(pwmPin, PWM_FREQ, PWM_RES);

        // TODO: Find max acceleration and then find the max the rate of change of PWM
        // Note: Requires PWM to speed mapping
    }

// Method to set motor speed and direction
void MotorControl::set_motor_PWM(double dutyCycle) {
    int pwmValue = constrain(dutyCycle, -PWM_MAX_BIT, PWM_MAX_BIT);
    
    if (pwmValue > 0) {            // CW
        digitalWrite(motorDirPin, LOW);
        ledcWrite(motorPwmPin, abs(pwmValue));
        //analogWrite(motorPwmPin, abs(pwmValue));

    } else if (pwmValue < 0) {     // CCW
        digitalWrite(motorDirPin, HIGH);
        ledcWrite(motorPwmPin, abs(pwmValue));
        // analogWrite(motorPwmPin, abs(pwmValue));
    } else {
        ledcWrite(motorPwmPin, abs(pwmValue));
        // analogWrite(motorPwmPin, 0);
    }
}

// (Blocking) Method to test motor
void MotorControl::stop_motor() {
    this->set_motor_PWM(0); // Set motor PWM
}

// (Blocking) Method to test motor
void MotorControl::test_motor(double dutyCycle) {
    this->set_motor_PWM(dutyCycle); // Set motor PWM
}

// Method to find max acceleration and then find the max the rate of change of PWM
/*
void MotorControl::find_max_pwm_roc() {


}
*/

// Converts unit of speed from duty cycle to PWM value
inline int duty_cycle_to_PWM(double dutyCycle) {
    return (int) ((dutyCycle * PWM_MAX_BIT / 100.0) + 0.5);
}

// Test sequentially of all wheel motors
void test_all_wheel_motors(MotorControl* UL_Motor, MotorControl* UR_Motor, MotorControl* BL_Motor, MotorControl* BR_Motor) {

}

void forward_hard_coded(double initialPWM, double maxPWM, double rampTimeMs, double durationMs, MotorControl* UL_Motor, MotorControl* UR_Motor, MotorControl* BL_Motor, MotorControl* BR_Motor) {
   if (2*rampTimeMs < durationMs) {
        double samplingPeriod = 50;
        int maxIter = (int) (rampTimeMs/samplingPeriod);
        double pwmIncrement = (maxPWM-initialPWM) / maxIter;
        double currentPWM = initialPWM;
        // Increasing Velocity
        for (int i = 0; i < maxIter; i++) {
            currentPWM += pwmIncrement;
            UL_Motor->set_motor_PWM(currentPWM);
            UR_Motor->set_motor_PWM(-currentPWM);
            BL_Motor->set_motor_PWM(-currentPWM);
            BR_Motor->set_motor_PWM(currentPWM);
            delay(samplingPeriod);
        }

        // Maintain Max Velocity
        delay(durationMs- 2*rampTimeMs);

        // Decreasing Velocity
        for (int i = 0; i < maxIter; i++) {
            currentPWM -= pwmIncrement;
            UL_Motor->set_motor_PWM(currentPWM);
            UR_Motor->set_motor_PWM(-currentPWM);
            BL_Motor->set_motor_PWM(-currentPWM);
            BR_Motor->set_motor_PWM(currentPWM);
            delay(samplingPeriod);
        }
        
        UL_Motor->stop_motor();
        UR_Motor->stop_motor();
        BL_Motor->stop_motor();
        BR_Motor->stop_motor();
    }
    else {
        Serial.println("Ramp time should be 2x lesser than duration.");
    }
    /*
    if (2*rampTimeMs < durationMs) {
        double samplingPeriod = 50;
        int maxIter = (int) (rampTimeMs/samplingPeriod);
        double pwmIncrement = maxPWM / maxIter;
        double currentPWM = 0;
        // Increasing Velocity
        for (int i = 0; i < maxIter; i++) {
            currentPWM += pwmIncrement;
            UL_Motor->set_motor_PWM(currentPWM);
            UR_Motor->set_motor_PWM(-currentPWM);
            BL_Motor->set_motor_PWM(-currentPWM);
            BR_Motor->set_motor_PWM(currentPWM);
            delay(samplingPeriod);
        }

        // Maintain Max Velocity
        delay(durationMs- 2*rampTimeMs);

        // Decreasing Velocity
        for (int i = 0; i < maxIter; i++) {
            currentPWM -= pwmIncrement;
            UL_Motor->set_motor_PWM(currentPWM);
            UR_Motor->set_motor_PWM(-currentPWM);
            BL_Motor->set_motor_PWM(-currentPWM);
            BR_Motor->set_motor_PWM(currentPWM);
            delay(samplingPeriod);
        }
    }
    else {
        Serial.println("Ramp time should be 2x lesser than duration.");
    }
    */
}