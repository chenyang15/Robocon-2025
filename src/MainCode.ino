#include <Arduino.h>
#include "Motor.h"
#include "PinAssignment.h"
#include "Utils.h"


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
}

void loop(){
    test_all_wheel_motors(&UL_Motor, &UR_Motor, &BL_Motor, &BR_Motor);
    forward_hard_coded(50, 2.5, 7, &UL_Motor, &UR_Motor, &BL_Motor, &BR_Motor);
    Serial.println("Done testing.");
    for(;;){
        //Do nothing
    }
}