#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"

/*========================================================================================
=                                WHEEL MOTOR GLOBAL VARIABLES                            =
========================================================================================*/
// Upper Left Wheel Motor
MotorWithEncoder UL_Motor(
    MOTOR_UL_DIR_1,     // Motor Dir Pin 1
    MOTOR_UL_PWM,       // Motor Enable Pin
    MOTOR_UL_ENCODER_A, // Encoder Pin A
    MOTOR_UL_ENCODER_B, // Encoder Pin B
    1,                  // Max Pwm Increment Per Acutation Period (units: duty cycle; range: 0~100)
    10,                 // Kp
    0,                  // Ki
    0                   // Kd
);

// Upper Right Wheel Motor
MotorWithEncoder UR_Motor(
    MOTOR_UR_DIR_1,     // Motor Dir Pin 1
    MOTOR_UR_PWM,       // Motor Enable Pin
    MOTOR_UR_ENCODER_A, // Encoder Pin A
    MOTOR_UR_ENCODER_B, // Encoder Pin B
    1,                  // Max Pwm Increment Per Acutation Period (units: duty cycle; range: 0~100)
    10,                 // Kp
    0,                  // Ki
    0                   // Kd
);

// Bottom Left Wheel Motor
MotorWithEncoder BL_Motor(
    MOTOR_BL_DIR_1,     // Motor Dir Pin 1
    MOTOR_BL_PWM,       // Motor Enable Pin
    MOTOR_BL_ENCODER_A, // Encoder Pin A
    MOTOR_BL_ENCODER_B, // Encoder Pin B
    1,                  // Max Pwm Increment Per Acutation Period (units: duty cycle; range: 0~100)
    10,                 // Kp
    0,                  // Ki
    0                   // Kd
);

// Bottom Right Wheel Motor
MotorWithEncoder BR_Motor(
    MOTOR_BR_DIR_1,     // Motor Dir Pin 1
    MOTOR_BR_PWM,       // Motor Enable Pin
    MOTOR_BR_ENCODER_A, // Encoder Pin A
    MOTOR_BR_ENCODER_B, // Encoder Pin B
    1,                  // Max Pwm Increment Per Acutation Period (units: duty cycle; range: 0~100)
    10,                 // Kp
    0,                  // Ki
    0                   // Kd
);

// An array of wheel motor of class MotorWithEncoder
MotorWithEncoder wheelMotors [4] = {UL_Motor, UR_Motor, BL_Motor, BR_Motor};
// Inputs for all 4 wheel motors computed from PD velocity controller.
double wheelMotorInputs [4] = {0, 0, 0, 0};

/*========================================================================================
=                                       SECTION NAME                                     =
========================================================================================*/