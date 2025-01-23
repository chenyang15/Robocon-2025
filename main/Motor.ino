#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"
#include "Globals.h"

// Constructor for class MotorControl
Motor::Motor(uint8_t pin1, uint8_t pwmPin, double maxPwmIncrement)
    : motorDirPin(pin1), motorPwmPin(pwmPin), maxPwmIncrement(maxPwmIncrement), currentDutyCycle(0.0) {
        // Pin Initialisation
        pinMode(pin1, OUTPUT);
        
        // Check for available PWM channels
        if (Motor::pwmChannelsUsed == 16) {
            Serial.printf("Max PWM channels limit reached. Motor class cannot be initialized.\nStopping program.\n");
            stop_program();
        }
        pwmChannel = Motor::pwmChannelsUsed;
        ledcSetup(pwmChannel, PWM_FREQ, PWM_RES);
        ledcAttachPin(pwmPin, pwmChannel);
        Motor::pwmChannelsUsed++;

        // TODO: Find max acceleration and then find the max the rate of change of PWM
        // Note: Requires PWM to speed mapping
    }

// Method to set motor speed and direction
void Motor::set_motor_PWM(double dutyCycle) {
    int pwmValue = (int) ((dutyCycle * PWM_MAX_BIT + 0.5) / 100);
    pwmValue = constrain(pwmValue, -PWM_MAX_BIT, PWM_MAX_BIT);
       
    if (pwmValue > 0) {            // CW
        digitalWrite(motorDirPin, LOW);
        ledcWrite(pwmChannel, abs(pwmValue));

    } else if (pwmValue < 0) {     // CCW
        digitalWrite(motorDirPin, HIGH);
        ledcWrite(pwmChannel, abs(pwmValue));
    } else {
        ledcWrite(pwmChannel, abs(pwmValue));
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

// Do not use function by itself. It should be called multiple times
/* Example:
 * double targetPWM = 255;
 * for (;;) {
 *     Motor._ramp_PWM(targetPWM);
 *     delay(MOTOR_ACTUATION_PERIOD);
 * }
*/
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
    
    UL_Motor.set_motor_PWM(ULPWM);
    UR_Motor.set_motor_PWM(URPWM);
    BL_Motor.set_motor_PWM(BLPWM);
    BR_Motor.set_motor_PWM(BRPWM);
}

// Converts unit of speed from duty cycle to PWM value
inline int duty_cycle_to_PWM(double dutyCycle) {
    return (int) ((dutyCycle * PWM_MAX_BIT / 100.0) + 0.5);
}

// Test sequentially of all wheel motors
void test_all_wheel_motors(Motor* UL_Motor, Motor* UR_Motor, Motor* BL_Motor, Motor* BR_Motor) {

}

// This is a blocking function and is used for testing and data collection purposes only. Do not use in actual code
void forward_hard_coded_with_encoder(double initialPWM, double maxPWM, double rampUpTimeMs, double maxSpeedTime, double rampDownTimeMs, MotorWithEncoder (&wheelMotors)[4]) {
    // Setting up references
    MotorWithEncoder& UL_Motor = wheelMotors[0];
    MotorWithEncoder& UR_Motor = wheelMotors[1];
    MotorWithEncoder& BL_Motor = wheelMotors[2];
    MotorWithEncoder& BR_Motor = wheelMotors[3];
    
    int rampUpMaxIter = (int) (rampUpTimeMs/MOTOR_WHEEL_ACTUATION_PERIOD);
    double upPwmIncrement = (maxPWM-initialPWM) / rampUpMaxIter;
    double currentPWM = initialPWM;
    // Increasing Velocity
    for (int i = 0; i < rampUpMaxIter; i++) {
        currentPWM += upPwmIncrement;
        UL_Motor.set_motor_PWM(currentPWM);
        UR_Motor.set_motor_PWM(currentPWM);
        BL_Motor.set_motor_PWM(currentPWM);
        BR_Motor.set_motor_PWM(currentPWM);
        
        delay_with_encoder(MOTOR_WHEEL_ACTUATION_PERIOD, wheelMotors);
    }

    // Serial.printf("Max speed reached.\n");
    // Maintain Max Velocity
    delay((int) maxSpeedTime);
    // Serial.printf("Decreasing speed.\n");
    // Decreasing Velocity
    int rampDownMaxIter = (int) (rampDownTimeMs/MOTOR_WHEEL_ACTUATION_PERIOD);
    double downPwmIncrement = (maxPWM-initialPWM) / rampDownMaxIter;
    for (int i = 0; i < rampDownMaxIter; i++) {
        currentPWM -= downPwmIncrement;
        UL_Motor.set_motor_PWM(currentPWM);
        UR_Motor.set_motor_PWM(currentPWM);
        BL_Motor.set_motor_PWM(currentPWM);
        BR_Motor.set_motor_PWM(currentPWM);
        static int loopCount = 0;
        loopCount++;
        if (loopCount % 1 == 0) Serial.printf("PWM:%.2f,", currentPWM/100);
        delay_with_encoder(MOTOR_WHEEL_ACTUATION_PERIOD, wheelMotors);
    }
    // Serial.printf("Stopping.\n");
    
    UL_Motor.stop_motor();
    UR_Motor.stop_motor();
    BL_Motor.stop_motor();
    BR_Motor.stop_motor();
}

void delay_with_encoder(unsigned long delayMs, MotorWithEncoder (&wheelMotors)[4]) {
    // Setting up references
    MotorWithEncoder& UL_Motor = wheelMotors[0];
    MotorWithEncoder& UR_Motor = wheelMotors[1];
    MotorWithEncoder& BL_Motor = wheelMotors[2];
    MotorWithEncoder& BR_Motor = wheelMotors[3];

    int a, b, c, d;

    unsigned long previousTime = millis();
    unsigned long endTime = previousTime + delayMs;
    for(;;){
        unsigned long currentTime = millis();
        if (currentTime-previousTime >= MOTOR_WHEEL_ENCODER_PERIOD) {
            previousTime = currentTime;
            a = UL_Motor.update_tick_velocity();
            b = UR_Motor.update_tick_velocity();
            c = BL_Motor.update_tick_velocity();
            d = BR_Motor.update_tick_velocity();

            static int loopCount = 0;
            if (loopCount % 2 == 0) {
                char buffer [50];
                sprintf(buffer, "1:%d,2:%d,3:%d,4:%d\n", a, b, c, d);
                client.send(buffer);
            }
            loopCount++;
        }
        if (endTime - currentTime > delayMs) {
            return;
        }
    }
}
// void forward_hard_coded(double initialPWM, double maxPWM, double rampTimeMs, double durationMs, Motor (&wheelMotors)[4]) {
//     // Setting up references
//     Motor& UL_Motor = wheelMotors[0];
//     Motor& UR_Motor = wheelMotors[1];
//     Motor& BL_Motor = wheelMotors[2];
//     Motor& BR_Motor = wheelMotors[3];
//     if (2*rampTimeMs < durationMs) {
//         int maxIter = (int) (rampTimeMs/MOTOR_WHEEL_ACTUATION_PERIOD);
//         double pwmIncrement = (maxPWM-initialPWM) / maxIter;
//         double currentPWM = initialPWM;
//         // Increasing Velocity
//         for (int i = 0; i < maxIter; i++) {
//             currentPWM += pwmIncrement;
//             UL_Motor.set_motor_PWM(currentPWM);
//             UR_Motor.set_motor_PWM(-currentPWM);
//             BL_Motor.set_motor_PWM(-currentPWM);
//             BR_Motor.set_motor_PWM(currentPWM);
//             delay(MOTOR_WHEEL_ACTUATION_PERIOD);
//         }

//         // Maintain Max Velocity
//         delay(durationMs- 2*rampTimeMs);

//         // Decreasing Velocity
//         for (int i = 0; i < maxIter; i++) {
//             currentPWM -= pwmIncrement;
//             UL_Motor.set_motor_PWM(currentPWM);
//             UR_Motor.set_motor_PWM(-currentPWM);
//             BL_Motor.set_motor_PWM(-currentPWM);
//             BR_Motor.set_motor_PWM(currentPWM);
//             delay(MOTOR_WHEEL_ACTUATION_PERIOD);
//         }
        
//         UL_Motor.stop_motor();
//         UR_Motor.stop_motor();
//         BL_Motor.stop_motor();
//         BR_Motor.stop_motor();
//     }
//     else {
//         Serial.print("Ramp time should be 2x lesser than duration.\n");
//     }
// }

// To clamp motor input calculated from PS4 analog stick. Clamping is through a ramping function.
void input_shaping(double (&wheelMotorInputs) [4], double (&previousWheelMotorInputs) [4], MotorWithEncoder& UL_Motor) {
    // Assume all maxPwmIncrement (defined in class) is the same across all wheel motors
    static double maxPwmIncrement = UL_Motor.maxPwmIncrement;

    // Using for loop, go through each motor inputs and apply ramping function
    for (int i = 0; i < 4; i++) {
        // Find current unclamped increment from controller output
        double unclampedIncrement = wheelMotorInputs[i] - previousWheelMotorInputs[i];
        
        // Limit Increment
        double increment;
        if (unclampedIncrement > maxPwmIncrement) {
            increment = maxPwmIncrement;
        }
        else if (unclampedIncrement < -maxPwmIncrement) {
            increment = -maxPwmIncrement;
        }
        else {
            increment = unclampedIncrement;
        }

        // Update variables
        wheelMotorInputs[i] = previousWheelMotorInputs[i] + increment; // Apply ramp
        previousWheelMotorInputs[i] = wheelMotorInputs[i];
    }
}