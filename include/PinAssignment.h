#pragma once
// Warning: Do not use pin 0 and pin 1

#define MOTOR_UL_PWM 3
#define MOTOR_UL_DIR_1 2

#define MOTOR_BL_PWM 6
#define MOTOR_BL_DIR_1 10

#define MOTOR_BR_PWM 9
#define MOTOR_BR_DIR_1 13

#define MOTOR_UR_PWM 5
#define MOTOR_UR_DIR_1 7

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

