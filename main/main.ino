#include <Arduino.h>
// Initialize global variables
#include "Globals.h"      // Note: Variables in here can be accessed anywhere in main file

#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"
#include "PS4.h"
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder

/* To-do:
 * - Test encoder signals
 * - PS4 controller left and right analog stick mapping using BluePad32.h
 * - For limiting motor current, find optimum max pwm increment per actuation period 
*/


void setup(){
    Serial.begin(115200);
    Serial.printf("Starting program.\n");
    
    // PS4 Controller Setup
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

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
                // TODO: Need PWM to speed mapping
                // Calculate PD PWM output of each wheel's motor
                // wheelMotorInputs[0] = UL_Motor.PID.compute(wheelMotorInputs[0], UL_Motor.ticksPerSample);
                // wheelMotorInputs[1] = UR_Motor.PID.compute(wheelMotorInputs[1], UR_Motor.ticksPerSample);
                // wheelMotorInputs[2] = BL_Motor.PID.compute(wheelMotorInputs[2], BL_Motor.ticksPerSample);
                // wheelMotorInputs[3] = BR_Motor.PID.compute(wheelMotorInputs[3], BR_Motor.ticksPerSample);
                // Actuate Motor
                ramp_wheel_PWM(wheelMotors, wheelMotorInputs);
            }

            if ((loopCount+PS4_SAMPLING_PERIOD_OFFSET) % PS4_SAMPLING_MOD == 0) {
                bool dataUpdated = BP32.update();
                if (dataUpdated) {
                    processControllers(PS4StickOutputs);
                }
                // Calculate motor input based on PS4 analog stick
                PS4_input_to_wheel_velocity(wheelMotorInputs, PS4StickOutputs);
                Serial.printf("Wheel Motor 1: %d, Wheel Motor 2: %d, Wheel Motor 3: %d, Wheel Motor 4: %d, ", wheelMotorInputs[0], wheelMotorInputs[1], wheelMotorInputs[2], wheelMotorInputs[3]);
                // Input shaping to apply ramping function to motor input calculated from PS4 stick input
                input_shaping(wheelMotorInputs, previousWheelMotorInputs, UL_Motor);
                Serial.printf("Clamped Wheel Motor 1: %d, Clamped Wheel Motor 2: %d, Clamped Wheel Motor 3: %d, Clamped Wheel Motor 4: %d, ", wheelMotorInputs[0], wheelMotorInputs[1], wheelMotorInputs[2], wheelMotorInputs[3]);
                vTaskDelay(1);
            }

            // Increment loop count
            loopCount++;
        }
    digitalWrite(LED_PIN, LOW);
    }
}

