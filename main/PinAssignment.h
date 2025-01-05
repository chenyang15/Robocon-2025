#pragma once
// Warning: Do not use pin 0 and pin 1

#define MOTOR_UL_PWM 16
#define MOTOR_UL_DIR_1 17
#define MOTOR_UL_ENCODER_A 18
#define MOTOR_UL_ENCODER_B 19

#define MOTOR_UR_PWM 23
#define MOTOR_UR_DIR_1 24
#define MOTOR_UR_ENCODER_A 25
#define MOTOR_UR_ENCODER_B 26

#define MOTOR_BL_PWM 27
#define MOTOR_BL_DIR_1 28
#define MOTOR_BL_ENCODER_A 29
#define MOTOR_BL_ENCODER_B 30

#define MOTOR_BR_PWM 31
#define MOTOR_BR_DIR_1 32
#define MOTOR_BR_ENCODER_A 33
#define MOTOR_BR_ENCODER_B 34

#define LED_PIN 13

// Weight applied on wheels
/* Procedure on value acquisition
 1. Put the robot on 4 weighing scales. All scales should be at the same Y height
 2. Take measurements at each weight scale
 3. Minus the weight of the mecanum wheel as that should be accounted elsewhere as rotational inertia
*/
#define MOTOR_UL_WEIGHT 2
#define MOTOR_UR_WEIGHT 2
#define MOTOR_BL_WEIGHT 2
#define MOTOR_BR_WEIGHT 2

