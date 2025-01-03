#include <Arduino.h>
#include "Timing.h"

int timing_setup_check() {
    int flag = 1;

    // Motor Wheel Actuation Check
    if (MOTOR_WHEEL_ACTUATION_PERIOD % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("MAIN_LOOP_PERIOD (%d) is not a multiple of MOTOR_WHEEL_ACTUATION_PERIOD (%d)\n",
                     MAIN_LOOP_PERIOD, MOTOR_WHEEL_ACTUATION_PERIOD);
        flag = 0;
    }

    // Motor Wheel Encoder Sampling Check
    if (MOTOR_WHEEL_ENCODER_PERIOD % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("MAIN_LOOP_PERIOD (%d) is not a multiple of MOTOR_WHEEL_ENCODER_PERIOD (%d)\n",
                     MAIN_LOOP_PERIOD, MOTOR_WHEEL_ENCODER_PERIOD);
        flag = 0;
    }
    
    // Motor Wheel Encoder Offset Check
    if (MOTOR_WHEEL_ENCODER_TIME_OFFSET % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("MAIN_LOOP_PERIOD (%d) is not a multiple of MOTOR_WHEEL_ENCODER_TIME_OFFSET (%d)\n",
                        MAIN_LOOP_PERIOD, MOTOR_WHEEL_ENCODER_TIME_OFFSET);
        flag = 0;
    }

    // PS4 Sampling Period Check
    if (PS4_SAMPLING_PERIOD % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("MAIN_LOOP_PERIOD (%d) is not a multiple of PS4_SAMPLING_PERIOD (%d)\n",
                        MAIN_LOOP_PERIOD, PS4_SAMPLING_PERIOD);
        flag = 0;
    }

    // PS4 Sampling Period Check
    if (PS4_SAMPLING_PERIOD_OFFSET % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("MAIN_LOOP_PERIOD (%d) is not a multiple of PS4_SAMPLING_PERIOD_OFFSET (%d)\n",
                        MAIN_LOOP_PERIOD, PS4_SAMPLING_PERIOD_OFFSET);
        flag = 0;
    }

    return flag;
}
