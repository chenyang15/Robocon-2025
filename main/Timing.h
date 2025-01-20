#pragma once
#include "Utils.h"

#define MAIN_LOOP_PERIOD                10
#define MOTOR_WHEEL_ACTUATION_PERIOD    100
#define MOTOR_WHEEL_ENCODER_PERIOD      50
#define PS4_SAMPLING_PERIOD             100
#define CPU_UTIL_CALCULATION_PERIOD     100
#define WEBSOCKET_HANDLING_PERIOD       100

// Define period (run once every x loopCount) for each tasks
#define MOTOR_WHEEL_ACTUATION_MOD   (MOTOR_WHEEL_ACTUATION_PERIOD   / MAIN_LOOP_PERIOD)
#define MOTOR_WHEEL_ENCODER_MOD     (MOTOR_WHEEL_ENCODER_PERIOD     / MAIN_LOOP_PERIOD)
#define PS4_SAMPLING_MOD            (PS4_SAMPLING_PERIOD            / MAIN_LOOP_PERIOD)
#define CPU_UTIL_CALCULATION_MOD    (CPU_UTIL_CALCULATION_PERIOD    / MAIN_LOOP_PERIOD)
#define WEBSOCKET_HANDLING_MOD      (WEBSOCKET_HANDLING_PERIOD      / MAIN_LOOP_PERIOD)

// Set task timing offset (in ms)
#define MOTOR_WHEEL_ENCODER_TIME_OFFSET     -10
#define PS4_SAMPLING_TIME_OFFSET            -10
#define CPU_UTIL_CALCULATION_TIME_OFFSET    -30
#define WEBSOCKET_HANDLING_TIME_OFFSET      -40

// Find loop offset based on time offset set above
#define MOTOR_WHEEL_ENCODER_LOOP_OFFSET     (MOTOR_WHEEL_ENCODER_TIME_OFFSET    / MAIN_LOOP_PERIOD)
#define PS4_SAMPLING_LOOP_OFFSET            (PS4_SAMPLING_TIME_OFFSET           / MAIN_LOOP_PERIOD)
#define CPU_UTIL_CALCULATION_LOOP_OFFSET    (CPU_UTIL_CALCULATION_TIME_OFFSET   / MAIN_LOOP_PERIOD)
#define WEBSOCKET_HANDLING_LOOP_OFFSET      (WEBSOCKET_HANDLING_TIME_OFFSET     / MAIN_LOOP_PERIOD)

int timing_setup_check();