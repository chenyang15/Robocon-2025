#include <Arduino.h>
// Initialize global variables
#include "Globals.h"      // Note: Variables in here can be accessed anywhere in main file

#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"
#include "PS4.h"
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder

/* Currently implementing:
 * - CPU utilization calculation
 *
 * Last implemented:
 * - data transmission over WiFi
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
	ESP32Encoder::useInternalWeakPullResistors = puType::up;    // Enable the weak pull up resistors
        
    // WebSocket Server Setup
    WiFi.begin(ssid, password);
    // Wait for the ESP32 to connect to Wi-Fi
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.printf("Connecting to WiFi (%s) ...\n", ssid);
    }
    Serial.printf("Connected to WiFi!\nESP32 IP Address: ");
    Serial.print(WiFi.localIP());
    Serial.printf(":81\n");

    // Start the WebSocket server
    server.listen(81); // Listen on port 81
    Serial.println("WebSocket server started!");

    // Wait for PS4 Connection
    bool dataUpdated = BP32.update();
    // while (!dataUpdated) {
    //     dataUpdated = BP32.update();
    //     delay(250);
    // }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Connected. Running Program.");
    delay(500);
    digitalWrite(LED_PIN, LOW);
}

void loop(){
    // Main loop setup
    uint32_t loopCount = 0;
    unsigned long previousTime = 0;

    // Timing variables used only for calculating CPU utilization
    unsigned long encoderWheelTime = 0;
    unsigned long motorActuationTime = 0;
    unsigned long ps4SamplingTime = 0;
    unsigned long loopEndTime = 0;
    uint8_t taskRan = 0;    // Flag that indicates task(s) had been ran in current loop.
    // Main loop
    for(;;) {
        unsigned long currentTime = millis();

        if (currentTime-previousTime >= MAIN_LOOP_PERIOD) {
            previousTime += 10;
            // Wheel motors - Get encoder count //
            if ((loopCount+MOTOR_WHEEL_ENCODER_LOOP_OFFSET) % MOTOR_WHEEL_ENCODER_MOD == 0) {
                encoderWheelTime = millis();
                taskRan = 1;
                // Update tick velocity
                UL_Motor.update_tick_velocity();
                UR_Motor.update_tick_velocity();
                BL_Motor.update_tick_velocity();
                BR_Motor.update_tick_velocity();
            }

            // Wheel motors - Actuation //
            if (loopCount % MOTOR_WHEEL_ACTUATION_MOD == 0) {
                motorActuationTime = millis();
                taskRan = 1;
                // TODO: Need PWM to speed mapping
                // Calculate PD PWM output of each wheel's motor
                // wheelMotorInputs[0] = UL_Motor.PID.compute(wheelMotorInputs[0], UL_Motor.ticksPerSample);
                // wheelMotorInputs[1] = UR_Motor.PID.compute(wheelMotorInputs[1], UR_Motor.ticksPerSample);
                // wheelMotorInputs[2] = BL_Motor.PID.compute(wheelMotorInputs[2], BL_Motor.ticksPerSample);
                // wheelMotorInputs[3] = BR_Motor.PID.compute(wheelMotorInputs[3], BR_Motor.ticksPerSample);
                // Actuate Motor
                // ramp_wheel_PWM(wheelMotors, wheelMotorInputs);
            }

            // PS4 Sampling //
            if ((loopCount+PS4_SAMPLING_LOOP_OFFSET) % PS4_SAMPLING_MOD == 0) {
                ps4SamplingTime = millis();
                taskRan = 1;
                bool dataUpdated = BP32.update();
                if (dataUpdated) {
                    processControllers(PS4StickOutputs);
                }
                // Calculate motor input based on PS4 analog stick
                static int printLoop = 0;
                printLoop++;
                PS4_input_to_wheel_velocity(wheelMotorInputs, PS4StickOutputs);
                // if (printLoop % (300/PS4_SAMPLING_PERIOD) == 0) Serial.printf("Lx: %d, Ly: %d, Rx: %d, Ry: %d\n", PS4StickOutputs[0], PS4StickOutputs[1], PS4StickOutputs[2], PS4StickOutputs[3]);
                // if (printLoop % (500/PS4_SAMPLING_PERIOD) == 0) Serial.printf("1:%.2f,2:%.2f,3:%.2f,4:%.2f,", wheelMotorInputs[0], wheelMotorInputs[1], wheelMotorInputs[2], wheelMotorInputs[3]);
                // Input shaping to apply ramping function to motor input calculated from PS4 stick input
                input_shaping(wheelMotorInputs, previousWheelMotorInputs, UL_Motor);
                // if (printLoop % (500/PS4_SAMPLING_PERIOD) == 0) Serial.printf("C1:%.2f,C2:%.2f,C3:%.2f,C4:%.2f\n", wheelMotorInputs[0], wheelMotorInputs[1], wheelMotorInputs[2], wheelMotorInputs[3]);
                vTaskDelay(1);
            }

            // CPU Utilization Calculation //
            if ((loopCount+CPU_UTIL_CALCULATION_LOOP_OFFSET) % CPU_UTIL_CALCULATION_MOD == 0) {

            }

            // WebSocket Handling //
            if ((loopCount+WEBSOCKET_HANDLING_LOOP_OFFSET) % WEBSOCKET_HANDLING_MOD == 0) {
                // Accept new WebSocket client connections
                if (!clientConnected) {
                    auto newClient = server.accept();
                    if (newClient.available()) {
                        Serial.println("New WebSocket client connected!");
                        client = newClient;
                        clientConnected = true;
                    }
                }

                // (TESTING) Send data continuously to the connected client
                if (clientConnected && client.available()) {
                    static int printLoop = 0;
                    printLoop++;
                    if (printLoop % (100/WEBSOCKET_HANDLING_PERIOD) == 0) {
                        // Example data to send
                        String data = "Current Time: " + String(loopCount*0.01) + " seconds";
                        client.send(data); // Send the message
                    }
                }

                // Handle client disconnection
                if (clientConnected && !client.available()) {
                    Serial.println("Client disconnected!");
                    client.close();
                    clientConnected = false;
                }
            }

            // Increment loop count
            loopCount++;
            loopEndTime = millis();
        }
    }
}

