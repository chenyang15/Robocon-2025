#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"

// Constructor for class MotorControl
Motor::Motor(byte pin1, byte pwmPin, double maxPwmIncrement)
    : motorDirPin(pin1), motorPwmPin(pwmPin), maxPwmIncrement(maxPwmIncrement), currentDutyCycle(0.0) {
        // Pin Initialisation
        pinMode(pin1, OUTPUT);
        //pinMode(pwmPin, OUTPUT);
        ledcSetup(pwmPin, PWM_FREQ, PWM_RES);

        // TODO: Find max acceleration and then find the max the rate of change of PWM
        // Note: Requires PWM to speed mapping
    }

// Method to set motor speed and direction
void Motor::set_motor_PWM(double dutyCycle) {
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
}

void Motor::stop_motor() {
    this->set_motor_PWM(0); // Set motor PWM
    this->currentDutyCycle = 0;
}

// (Blocking) Method to test motor
void Motor::test_motor(double dutyCycle) {
    this->set_motor_PWM(dutyCycle); // Set motor PWM
}

// Do not use function by itself, see example function ramp_wheel_PWM for usage
void Motor::_ramp_PWM(double motorInput) {
    // Find current unclamped increment from controller output
    double unclampedIncrement = motorInput - currentDutyCycle;
    
    // Limit Increment
    double increment;
    if (unclampedIncrement > this->maxPwmIncrement) {
        increment = maxPwmIncrement;
    }
    else if (unclampedIncrement < this->maxPwmIncrement) {
        increment = -maxPwmIncrement;
    }
    else {
        increment = unclampedIncrement;
    }

    // Increment Duty Cycle of Motor
    double dutyCycle = currentDutyCycle + increment;
    this->set_motor_PWM(dutyCycle);
    
    // WARNING: This is an assumption that actuation is equals to input after an actuation period
    // TODO: May requires Speed to PWM mapping in order to properly limit increment and get rid of this assumption.
    currentDutyCycle = dutyCycle;
}

// Currently open loop
/**
 * Accepts PWM input from controller output to actuate wheel motors
 * @param (&WheelMotors)[4] An array of wheel motor classes passed by reference.
 * @param wheelMotorPWMs PWM output of each wheel motors from the PD controller.
 * @return none
 * @warning Do not use this function for other motors other than wheel motors.
 * @note Example use case - ramp_wheel_PWM(wheelMotors, wheelMotorPWMs);
 */
void ramp_wheel_PWM(MotorWithEncoder (&WheelMotors) [4], double (&wheelMotorInputs) [4]) {
    // Setting up references
    static MotorWithEncoder& UL_Motor = WheelMotors[0];
    static MotorWithEncoder& UR_Motor = WheelMotors[1];
    static MotorWithEncoder& BL_Motor = WheelMotors[2];
    static MotorWithEncoder& BR_Motor = WheelMotors[3];
    static double& ULPWM = wheelMotorInputs[0];
    static double& URPWM = wheelMotorInputs[1];
    static double& BLPWM = wheelMotorInputs[2];
    static double& BRPWM = wheelMotorInputs[3];
    
    UL_Motor._ramp_PWM(ULPWM);
    UR_Motor._ramp_PWM(URPWM);
    BL_Motor._ramp_PWM(BLPWM);
    BR_Motor._ramp_PWM(BRPWM);
}

// Converts unit of speed from duty cycle to PWM value
inline int duty_cycle_to_PWM(double dutyCycle) {
    return (int) ((dutyCycle * PWM_MAX_BIT / 100.0) + 0.5);
}

// Test sequentially of all wheel motors
void test_all_wheel_motors(Motor* UL_Motor, Motor* UR_Motor, Motor* BL_Motor, Motor* BR_Motor) {

}

void forward_hard_coded(double initialPWM, double maxPWM, double rampTimeMs, double durationMs, Motor (&wheelMotors)[4]) {
    // Setting up references
    Motor& UL_Motor = wheelMotors[0];
    Motor& UR_Motor = wheelMotors[1];
    Motor& BL_Motor = wheelMotors[2];
    Motor& BR_Motor = wheelMotors[3];
    if (2*rampTimeMs < durationMs) {
        int maxIter = (int) (rampTimeMs/MOTOR_WHEEL_ACTUATION_PERIOD);
        double pwmIncrement = (maxPWM-initialPWM) / maxIter;
        double currentPWM = initialPWM;
        // Increasing Velocity
        for (int i = 0; i < maxIter; i++) {
            currentPWM += pwmIncrement;
            UL_Motor.set_motor_PWM(currentPWM);
            UR_Motor.set_motor_PWM(-currentPWM);
            BL_Motor.set_motor_PWM(-currentPWM);
            BR_Motor.set_motor_PWM(currentPWM);
            delay(MOTOR_WHEEL_ACTUATION_PERIOD);
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
            delay(MOTOR_WHEEL_ACTUATION_PERIOD);
        }
        
        UL_Motor.stop_motor();
        UR_Motor.stop_motor();
        BL_Motor.stop_motor();
        BR_Motor.stop_motor();
    }
    else {
        Serial.print("Ramp time should be 2x lesser than duration.\n");
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