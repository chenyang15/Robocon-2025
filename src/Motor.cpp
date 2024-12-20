#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"

// Constructor
MotorControl::MotorControl(byte pin1, byte pwmPin)
    : motorDirPin(pin1), motorPwmPin(pwmPin) {
        // Pin Initialisation
        pinMode(pin1, OUTPUT);
        //pinMode(pwmPin, OUTPUT);
        ledcSetup(pwmPin, PWM_FREQ, PWM_RES);
        currentDutyCycle = 0;

        // TODO: Find max acceleration and then find the max the rate of change of PWM
        // Note: Requires PWM to speed mapping
    }

// Method to set motor speed and direction
void MotorControl::set_motor_PWM(double dutyCycle) {
    int pwmValue = (int) dutyCycle * PWM_MAX_BIT + 0.5;
    pwmValue = constrain(dutyCycle, -PWM_MAX_BIT, PWM_MAX_BIT);
    
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
    // WARNING: This is open loop, and an assumption that actuation is equals to input
    currentDutyCycle = dutyCycle;
}

void MotorControl::stop_motor() {
    this->set_motor_PWM(0); // Set motor PWM
}

// (Blocking) Method to test motor
void MotorControl::test_motor(double dutyCycle) {
    this->set_motor_PWM(dutyCycle); // Set motor PWM
}

// Do not use function by itself, see example function ramp_wheel_PWM for usage
void MotorControl::_ramp_PWM(double dutyCycle) {
    if (dutyCycle - currentDutyCycle > PWM_DUTY_CYCLE_INCREMENT) {
        this->set_motor_PWM(dutyCycle + PWM_DUTY_CYCLE_INCREMENT);
    }
    else if (dutyCycle - currentDutyCycle < -PWM_DUTY_CYCLE_INCREMENT) {
        this->set_motor_PWM(dutyCycle - PWM_DUTY_CYCLE_INCREMENT);
    }
    else {
        this->set_motor_PWM(dutyCycle);
    }
}


// Method to find max acceleration and then find the max the rate of change of PWM
/*
void MotorControl::find_max_pwm_roc() {


}
*/

// Currently open loop
void ramp_wheel_PWM(MotorControl (&WheelMotors) [4], double (&wheelMotorPWMs) [4]) {
    // Setting up references
    MotorControl& UL_Motor = WheelMotors[0];
    MotorControl& UR_Motor = WheelMotors[1];
    MotorControl& BL_Motor = WheelMotors[2];
    MotorControl& BR_Motor = WheelMotors[3];
    double& ULPWM = wheelMotorPWMs[0];
    double& URPWM = wheelMotorPWMs[1];
    double& BLPWM = wheelMotorPWMs[2];
    double& BRPWM = wheelMotorPWMs[3];

    static unsigned long previousTime = 0;
    unsigned long currentTime = millis();
    if (currentTime-previousTime >= MOTOR_ACTUATION_PERIOD) {
        previousTime = currentTime; // Update current time
        // Actuate motor using ramp
        UL_Motor._ramp_PWM(ULPWM);
        UR_Motor._ramp_PWM(URPWM);
        BL_Motor._ramp_PWM(BLPWM);
        BR_Motor._ramp_PWM(BRPWM);
    }
}

// Converts unit of speed from duty cycle to PWM value
inline int duty_cycle_to_PWM(double dutyCycle) {
    return (int) ((dutyCycle * PWM_MAX_BIT / 100.0) + 0.5);
}

// Test sequentially of all wheel motors
void test_all_wheel_motors(MotorControl* UL_Motor, MotorControl* UR_Motor, MotorControl* BL_Motor, MotorControl* BR_Motor) {

}

void forward_hard_coded(double initialPWM, double maxPWM, double rampTimeMs, double durationMs, MotorControl (&wheelMotors)[4]) {
    // Setting up references
    MotorControl& UL_Motor = wheelMotors[0];
    MotorControl& UR_Motor = wheelMotors[1];
    MotorControl& BL_Motor = wheelMotors[2];
    MotorControl& BR_Motor = wheelMotors[3];
    if (2*rampTimeMs < durationMs) {
        int maxIter = (int) (rampTimeMs/MOTOR_ACTUATION_PERIOD);
        double pwmIncrement = (maxPWM-initialPWM) / maxIter;
        double currentPWM = initialPWM;
        // Increasing Velocity
        for (int i = 0; i < maxIter; i++) {
            currentPWM += pwmIncrement;
            UL_Motor.set_motor_PWM(currentPWM);
            UR_Motor.set_motor_PWM(-currentPWM);
            BL_Motor.set_motor_PWM(-currentPWM);
            BR_Motor.set_motor_PWM(currentPWM);
            delay(MOTOR_ACTUATION_PERIOD);
        }

        // Maintain Max Velocity
        delay(durationMs- 2*rampTimeMs);

        // Decreasing Velocity
        for (int i = 0; i < maxIter; i++) {
            currentPWM -= pwmIncrement;
            UL_Motor.set_motor_PWM(currentPWM);
            UR_Motor.set_motor_PWM(-currentPWM);
            BL_Motor.set_motor_PWM(-currentPWM);
            BR_Motor.set_motor_PWM(currentPWM);
            delay(MOTOR_ACTUATION_PERIOD);
        }
        
        UL_Motor.stop_motor();
        UR_Motor.stop_motor();
        BL_Motor.stop_motor();
        BR_Motor.stop_motor();
    }
    else {
        Serial.println("Ramp time should be 2x lesser than duration.");
    }
    /*
    if (2*rampTimeMs < durationMs) {
        double MOTOR_ACTUATION_PERIOD = 50;
        int maxIter = (int) (rampTimeMs/MOTOR_ACTUATION_PERIOD);
        double pwmIncrement = maxPWM / maxIter;
        double currentPWM = 0;
        // Increasing Velocity
        for (int i = 0; i < maxIter; i++) {
            currentPWM += pwmIncrement;
            UL_Motor.set_motor_PWM(currentPWM);
            UR_Motor.set_motor_PWM(-currentPWM);
            BL_Motor.set_motor_PWM(-currentPWM);
            BR_Motor.set_motor_PWM(currentPWM);
            delay(MOTOR_ACTUATION_PERIOD);
        }

        // Maintain Max Velocity
        delay(durationMs- 2*rampTimeMs);

        // Decreasing Velocity
        for (int i = 0; i < maxIter; i++) {
            currentPWM -= pwmIncrement;
            UL_Motor.set_motor_PWM(currentPWM);
            UR_Motor.set_motor_PWM(-currentPWM);
            BL_Motor.set_motor_PWM(-currentPWM);
            BR_Motor.set_motor_PWM(currentPWM);
            delay(MOTOR_ACTUATION_PERIOD);
        }
    }
    else {
        Serial.println("Ramp time should be 2x lesser than duration.");
    }
    */
}