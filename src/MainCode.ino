#include <Arduino.h>

#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder

// Total encoders are limited up to 8
ESP32Encoder UL_Encoder;
ESP32Encoder UR_Encoder;
ESP32Encoder BL_Encoder;
ESP32Encoder BR_Encoder;

MotorControl UL_Motor(
    MOTOR_UL_DIR_1,     // motor Dir Pin 1
    MOTOR_UL_PWM        // motor Enable Pin
);

MotorControl UR_Motor(
    MOTOR_BL_DIR_1,     // motor Dir Pin 1
    MOTOR_BL_PWM        // motor Enable Pin
);

MotorControl BL_Motor(
    MOTOR_BR_DIR_1,     // motor Dir Pin 1
    MOTOR_BR_PWM        // motor Enable Pin
);

MotorControl BR_Motor(
    MOTOR_UR_DIR_1,     // motor Dir Pin 1
    MOTOR_UR_PWM        // motor Enable Pin
);

MotorControl wheelMotors [4] = {UL_Motor, UR_Motor, BL_Motor, BR_Motor};
double wheelMotorPWMs [4] = {0, 0, 0, 0};

void setup(){
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);

    timing_setup_check();
    
    // Encoder setup
   	// Enable the weak pull down resistors
    //ESP32Encoder::useInternalWeakPullResistors = puType::down;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors = puType::up;



}

void loop(){
    digitalWrite(LED_PIN, HIGH);
    ramp_wheel_PWM(wheelMotors, wheelMotorPWMs);
    //forward_hard_coded(20, 100, 5000, 20000, wheelMotors);
    digitalWrite(LED_PIN, LOW);
    
    stop_program();

    // Main loop setup
    uint32_t loopCount = 0;
    int maxPeriod = maxValue(MOTOR_ACTUATION_PERIOD, 1, 2, 3); // Substitute 1, 2, 3, and Add more values for polling tasks if needed
    int maxLoopCount = maxPeriod / MAIN_LOOP_PERIOD;
    // Main loop
    for(;;){
        unsigned long previousTime = 0;
        unsigned long currentTime = millis();
        if (currentTime-previousTime >= MAIN_LOOP_PERIOD){
            previousTime = currentTime;
            loopCount++;
            if (loopCount % MOTOR_WHEEL_ACTUATION_MOD == 0) {
                // Set sampling rate of velocity calculation to be the same as the motor actuation period (may need to be separate)
                (int32_t)UL_Encoder.getCount();
                (int32_t)UR_Encoder.getCount();
                (int32_t)BL_Encoder.getCount();
                (int32_t)BR_Encoder.getCount();
                // Find velocity of each motor
            }

            
        }
    }
}

