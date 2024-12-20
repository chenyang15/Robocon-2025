#include <Arduino.h>
#include "Motor.h"
#include "PinAssignment.h"
#include "Utils.h"

// i am depressed
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


void setup(){
    pinMode(LED_PIN, OUTPUT);
}

void loop(){
    digitalWrite(LED_PIN, HIGH);
    forward_hard_coded(20, 100, 5000, 20000, &UL_Motor, &UR_Motor, &BL_Motor, &BR_Motor);
    digitalWrite(LED_PIN, LOW);
    for(;;)
    {
        
    }
}