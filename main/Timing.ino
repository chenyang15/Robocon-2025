#include <Arduino.h>
#include "Timing.h"

int timing_setup_check() {
    int flag = 1;

    // Motor Wheel Actuation Check
    if (MOTOR_WHEEL_ACTUATION_PERIOD % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("MOTOR_WHEEL_ACTUATION_PERIOD (%d) is not a multiple of MAIN_LOOP_PERIOD (%d)\n",
                     MOTOR_WHEEL_ACTUATION_PERIOD, MAIN_LOOP_PERIOD);
        flag = 0;
    }

    // Motor Wheel Encoder Sampling Check
    if (MOTOR_WHEEL_ENCODER_PERIOD % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("MOTOR_WHEEL_ENCODER_PERIOD (%d) is not a multiple of MAIN_LOOP_PERIOD (%d)\n",
                     MOTOR_WHEEL_ENCODER_PERIOD, MAIN_LOOP_PERIOD);
        flag = 0;
    }
    
    // Motor Wheel Encoder Offset Check
    if (MOTOR_WHEEL_ENCODER_TIME_OFFSET % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("MOTOR_WHEEL_ENCODER_TIME_OFFSET (%d) is not a multiple of MAIN_LOOP_PERIOD (%d)\n",
                        MOTOR_WHEEL_ENCODER_TIME_OFFSET, MAIN_LOOP_PERIOD);
        flag = 0;
    }

    // PS4 Sampling Period Check
    if (PS4_SAMPLING_PERIOD % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("PS4_SAMPLING_PERIOD (%d) is not a multiple of MAIN_LOOP_PERIOD (%d)\n",
                        PS4_SAMPLING_PERIOD, MAIN_LOOP_PERIOD);
        flag = 0;
    }

    // PS4 Sampling Offset Check
    if (PS4_SAMPLING_TIME_OFFSET % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("PS4_SAMPLING_PERIOD_OFFSET (%d) is not a multiple of MAIN_LOOP_PERIOD (%d)\n",
                        PS4_SAMPLING_TIME_OFFSET, MAIN_LOOP_PERIOD);
        flag = 0;
    }

    // Websocket Period Check
    if (WEBSOCKET_HANDLING_PERIOD % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("WEBSOCKET_HANDLING_PERIOD (%d) is not a multiple of MAIN_LOOP_PERIOD (%d)\n",
                        WEBSOCKET_HANDLING_PERIOD, MAIN_LOOP_PERIOD);
        flag = 0;
    }

    // Websocket Offset Check
    if (WEBSOCKET_HANDLING_TIME_OFFSET % MAIN_LOOP_PERIOD != 0) {
        Serial.printf("WEBSOCKET_HANDLING_TIME_OFFSET (%d) is not a multiple of MAIN_LOOP_PERIOD (%d)\n",
                        WEBSOCKET_HANDLING_TIME_OFFSET, MAIN_LOOP_PERIOD);
        flag = 0;
    }

    // Return timing check results
    return flag;
}
