// #if defined(PLATFORMIO)
// #include <Arduino.h>
// #endif
#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder

/* To-do:
 * - Test encoder signals
 * - PS4 controller left and right analog stick mapping using BluePad32.h
 * - For limiting motor current, find optimum max pwm increment per actuation period 
*/

// Initialize global variables
#include "Globals.h"      // Note: Variables in here can be accessed anywhere in main file

void setup(){
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);

    if(!timing_setup_check()) {
        Serial.print("Exiting program.\n");
        stop_program();
    }
    
    // Encoder setup
   	// Enable the weak pull down resistors
    //ESP32Encoder::useInternalWeakPullResistors = puType::down;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors = puType::up;



}

void loop(){
    digitalWrite(LED_PIN, HIGH);

    // Main loop setup
    uint32_t loopCount = 0;
    // Main loop
    for(;;) {
        unsigned long previousTime = 0;
        unsigned long currentTime = millis();
        
        if (currentTime-previousTime >= MAIN_LOOP_PERIOD) {
            previousTime = currentTime;
            // Wheel motors - Get encoder count
            if ((loopCount+MOTOR_WHEEL_ENCODER_LOOP_OFFSET) % MOTOR_WHEEL_ENCODER_MOD == 0) {
                // Update tick velocity
                UL_Motor.update_tick_velocity();
                UR_Motor.update_tick_velocity();
                BL_Motor.update_tick_velocity();
                BR_Motor.update_tick_velocity();
            }

            // Wheel motors - Actuation
            if (loopCount % MOTOR_WHEEL_ACTUATION_MOD == 0) {
                // TODO: Update setpoint of PID (get input from joystick)
                // May want the update to be in a different if statement
                stop_program(); //placeholder. refer TODO

                // Calculate PD PWM output of each wheel's motor
                wheelMotorInputs[0] = UL_Motor.PID.compute(UL_Motor.ticksPerSample);
                wheelMotorInputs[1] = UR_Motor.PID.compute(UR_Motor.ticksPerSample);
                wheelMotorInputs[2] = BL_Motor.PID.compute(BL_Motor.ticksPerSample);
                wheelMotorInputs[3] = BR_Motor.PID.compute(BR_Motor.ticksPerSample);
                // Actuate Motor
                ramp_wheel_PWM(wheelMotors, wheelMotorInputs);
            }

            
            loopCount++;
        }
    digitalWrite(LED_PIN, LOW);
    }
}

