#pragma once
#include "Motor.h"
#include "Utils.h"

#define MAIN_LOOP_PERIOD 10
#define TIMING_SETUP_SUCCESS 1  // will be set to 0 if conditions any else statements below are called

#if (MOTOR_ACTUATION_PERIOD % MAIN_LOOP_PERIOD == 0)
    #define MOTOR_WHEEL_ACTUATION_MOD (MOTOR_ACTUATION_PERIOD / MAIN_LOOP_PERIOD)
    #define MOTOR_WHEEL_SETUP_SUCCESS 1
#else
    #define TIMING_SETUP_SUCCESS 0
    #define MOTOR_WHEEL_SETUP_SUCCESS 0
#endif

void timing_setup_check(){
    // Motor Wheel Check
    if (!MOTOR_WHEEL_SETUP_SUCCESS){
        serialPrintf("MAIN_LOOP_PERIOD (%d) is not a multiple of MOTOR_ACTUATION_PERIOD (%d)", MAIN_LOOP_PERIOD, MOTOR_ACTUATION_PERIOD);
    }
}