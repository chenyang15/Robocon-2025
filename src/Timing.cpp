#include "Timing.h"

int timing_setup_check(){
    int flag = 1;
    // Motor Wheel Check
    if (!MOTOR_WHEEL_SETUP_SUCCESS){
        serialPrintf("MAIN_LOOP_PERIOD (%d) is not a multiple of MOTOR_WHEEL_ACTUATION_PERIOD (%d)", MAIN_LOOP_PERIOD, MOTOR_WHEEL_ACTUATION_PERIOD);
        flag = 0;
    }

    // Motor Wheel Encoder Check
    if (!MOTOR_WHEEL_ENCODER_SETUP_SUCCESS){
        serialPrintf("MAIN_LOOP_PERIOD (%d) is not a multiple of MOTOR_WHEEL_ENCODER_PERIOD (%d)", MAIN_LOOP_PERIOD, MOTOR_WHEEL_ENCODER_PERIOD);
        flag = 0;
    }
    return flag;
}