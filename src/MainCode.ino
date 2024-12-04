#include <Arduino.h>
#include "Motor.h"
#include "PinAssignment.h"
#include "Utils.h"


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


void setup(){
}

void loop(){
    test_all_wheel_motors(&UL_Motor, &UR_Motor, &BL_Motor, &BR_Motor);
}