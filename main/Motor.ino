#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"
#include "Globals.h"
#include "RuntimePrints.h"

// Constructor for class MotorControl
Motor::Motor(uint8_t pin1, uint8_t pwmPin, double maxPwmIncrement)
    : motorDirPin(pin1), motorPwmPin(pwmPin), maxPwmIncrement(maxPwmIncrement), previousDutyCycle(0.0) {
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
    int pwmValue = (int) ((dutyCycle * PWM_MAX_BIT + 0.5) / 100);   // converts duty cycle to units of bits while rounds to closest integer
    pwmValue = constrain(pwmValue, -PWM_MAX_BIT, PWM_MAX_BIT);      // limits value between maximum and minimum
       
    if (pwmValue >= 0) {            // CW
        digitalWrite(motorDirPin, LOW);
        ledcWrite(pwmChannel, abs(pwmValue));
    } 
    else if (pwmValue < 0) {     // CCW
        digitalWrite(motorDirPin, HIGH);
        ledcWrite(pwmChannel, abs(pwmValue));
    }
}

void Motor::stop_motor() {
    this->set_motor_PWM(0); // Set motor PWM
    this->previousDutyCycle = 0;
}

// This function applies input shaping (ramping function) to the raw input of the motor and does not actuate motor. 
// Use set_motor_pwm() afterwards.
// Do not use function by itself. It should be called multiple times.
/* Example:
 * double targetPWM = 100;
 * for (;;) {
 *     double shapedInput = Motor.ramp_PWM(targetPWM);
 *     Motor.set_motor_pwm(shapedInput);
 *     delay(MOTOR_ACTUATION_PERIOD);
 * }
*/
double Motor::input_shape_ramp(double rawInput) {
    // Find current unclamped increment from controller output
    double unclampedIncrement = rawInput - this->previousDutyCycle;
    
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
    double shapedInput = this->previousDutyCycle + increment;
    this->previousDutyCycle = shapedInput;

    return shapedInput;
}

// Currently open loop
/**
 * Accepts raw PWM input from PS4 controller. Applies a ramping function and then actuate the wheel motors
 * @param wheelMotors An array of wheel motor classes passed by reference.
 * @param wheelMotorPs4Inputs Raw duty cycle motor inputs derived from PS4 inputs.
 * @return none
 * @warning Do not use this function for other motors other than wheel motors.
 * @note Example use case - ramp_wheel_PWM(wheelMotors, wheelMotorPWMs);
 */
void actuate_motor_wheels() {
    double shapedInputs[4] = {0, 0, 0, 0};

    // Wait for mutex before modifying ps4StickInputs
    if (xSemaphoreTake(xMutex_wheelMotorPs4Inputs, portMAX_DELAY)) {
        // Apply input shaping (ramp function) to raw duty cycle inputs derived from PS4 inputs
        for (int i = 0; i < 4; ++i) {
            shapedInputs[i] = wheelMotors[i].input_shape_ramp(wheelMotorPs4Inputs[i]);
        }
        xSemaphoreGive(xMutex_wheelMotorPs4Inputs);  // Release the mutex after modifying the variable
    }
    
    // Apply PD to get closed loop input to motor
    double pidOutput[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i) {
        pidOutput[i] = shapedInputs[i]; // Get rid of this line and uncomment line below
        // pidOutput[i] = wheelMotors[i].PID.compute(shapedInputs[i], wheelMotors[i].measuredPwmSpeed);
    }

    // Actuate each motors using shaped feedforward inputs and PID output (summed)
    for (int i = 0; i < 4; ++i) {
        wheelMotors[i].set_motor_PWM(pidOutput[i]);
    }

    // Printing in WiFi WebSocket //
    // Print clamped wheel inputs (unit: duty cycle)
    {
        #if (PRINT_WHEEL_INPUT_CLAMPED_VELOCITY == 1) 
        char formattedMessage[128];  // Buffer to store the formatted message
        // Create formatted message
        snprintf(
            formattedMessage, 
            sizeof(formattedMessage), 
            "Wheels' Clamped Duty Input(1: %.2f, 2: %.2f, 3: %.2f, 4: %.2f\n", shapedInputs[0], shapedInputs[1], shapedInputs[2], shapedInputs[3]
        );
        // Send the formatted message to the queue
        xQueueSend(xQueue_wifi, &formattedMessage, 15 / portTICK_PERIOD_MS);
        #endif
    }

    // Print PID output and feedforward input (unit: duty cycle) 
    {
    #if (PRINT_PID_OUTPUT_PLUS_FEEDFORWARD == 1)
        char formattedMessage[128];  // Buffer to store the formatted message
        // Create formatted message
        snprintf(
            formattedMessage, 
            sizeof(formattedMessage), 
            "Wheels' PID and FF Duty Sum (1: %.2f, 2: %.2f, 3: %.2f, 4: %.2f\n", pidOutput[0], pidOutput[1], pidOutput[2], pidOutput[3]
        );
        // Send the formatted message to the queue
        xQueueSend(xQueue_wifi, &formattedMessage, 15 / portTICK_PERIOD_MS);
    #endif
    }
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
                char buffer [64];
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