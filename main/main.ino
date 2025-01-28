#include <Arduino.h>
// Initialize global variables
#include "Globals.h"      // Note: Variables in here can be accessed anywhere in main file
#include "RuntimePrints.h"
#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"
#include "PS4.h"
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder
#include "Wire.h"
#define SLAVE_ADDR_1 0x10
#define SLAVE_ADDR_2 0x20

/* Currently implementing:
 * - RTOS (priority, cpu usage, stack usage)
 * Next to be implemented:
 * - sending data to queue and printing through WiFi
 * To be tested:
 * - Pin assignment
 * 
*/

// #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  // Initialize your timer
// #define portGET_RUN_TIME_COUNTER_VALUE() (TIMx->CNT) // Read timer value

// Define a struct for the data packet with const char* for data
struct DataPacket {
    uint8_t slaveAddress;
    const char* data;
};

// Function prototypes for setup functions
void websocket_setup();
void ps4_setup();

// Task function protoyypes
void task_ps4_sampling      (void *pvParameters);
void task_update_encoders   (void *pvParameters);
void task_actuate_motors    (void *pvParameters);
void task_websocket_handler (void *pvParameters);
void task_send_to_wifi      (void *pvParameters);
void task_send_to_i2c       (void *pvParameters);

// Task Handles
TaskHandle_t xTask_Ps4Sampling;
TaskHandle_t xTask_UpdateEncoders;
TaskHandle_t xTask_ActuateMotors;
TaskHandle_t xTask_WebsocketHandler;
TaskHandle_t xTask_SendToWiFi;
TaskHandle_t xTask_SendToI2C;

SemaphoreHandle_t xMutex_wheelMotorPs4Inputs;
SemaphoreHandle_t bsem_;

QueueHandle_t xQueue_wifi;
QueueHandle_t xQueue_i2c;

// WebSocket Server Setup
void websocket_setup() {
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
}

// PS4 Controller Connection Setup
void ps4_setup() {
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);
    // Wait for ps4 Connection
    bool dataUpdated = BP32.update();
    while (!dataUpdated) {
        dataUpdated = BP32.update();
        delay(250);
    }
}

void setup(){
    Serial.begin(115200);
    Serial.printf("Initializing...\n");
    
    // Encoder setup
	ESP32Encoder::useInternalWeakPullResistors = puType::up;    // Enable the weak pull up resistors

    // Setup
    websocket_setup();  // WebSocket Server Setup
    ps4_setup();        // PS4 Controller Setup
    // Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Initialize I2C
    
    // Task creation result variables
    BaseType_t taskCreation_ps4Sampling;
    BaseType_t taskCreation_UpdateEncoders;
    BaseType_t taskCreation_ActuateMotors;
    BaseType_t taskCreation_WebsocketHandler;
    BaseType_t taskCreation_SendToWiFi;
    BaseType_t taskCreation_SendToI2C;

    bool creationStatus = 1; // Creation status flag
    // Create Mutex (Mutual Exclusion Semaphore) for global variables
    // Note: These semaphores are declared in Globals.h so that they can be accessed in any file.
    xMutex_wheelMotorPs4Inputs = xSemaphoreCreateMutex();        // Mutex for global var ps4StickOutputs
    bsem_ = xSemaphoreCreateBinary(); // Binary semaphore to indicate that new data is acquired in 
    // Check creation status for each semaphore/mutex
    check_sem_creation(creationStatus, xMutex_wheelMotorPs4Inputs, "Mutex - PS4 Stick Outputs");
    check_sem_creation(creationStatus, bsem_, "Binary Semaphore - Placeholder");

    // Create queues
    // Note: These queues are declared in Globals.h so that they can be accessed in any file.
    xQueue_wifi = xQueueCreate(10, sizeof(char*));  // Create a queue for WiFi messages to be sent
    xQueue_i2c = xQueueCreate(10, sizeof(uint8_t));  // Create a queue for I2C messages to be sent
    // Check creation status for each queue
    check_queue_creation(creationStatus, xQueue_wifi, "Queue - Send to WiFi");
    check_queue_creation(creationStatus, xQueue_i2c, "Queue - Send to I2C");

    // Create tasks
    taskCreation_ps4Sampling        = xTaskCreate(
        task_ps4_sampling,              // Task function
        "Task - PS4 Sampling",          // Task name
        4096,                           // Stack size (bytes)
        NULL,                           // Parameters
        1,                              // Priority
        &xTask_Ps4Sampling              // Task handle
    );
    print_free_stack(xTask_Ps4Sampling, "Task - PS4 Sampling");

    taskCreation_UpdateEncoders     = xTaskCreate(
        task_update_encoders,           // Task function
        "Task - Update Encoders",       // Task name
        2048,                           // Stack size (bytes)
        NULL,                           // Parameters
        1,                              // Priority
        &xTask_UpdateEncoders           // Task handle
    );
    print_free_stack(xTask_UpdateEncoders, "Task - Update Encoders");

    taskCreation_ActuateMotors      = xTaskCreate(
        task_actuate_motors,            // Task function
        "Task - Actuate Motors",        // Task name
        4096,                           // Stack size (bytes)
        NULL,                           // Parameters
        1,                              // Priority
        &xTask_ActuateMotors            // Task handle
    );
    print_free_stack(xTask_ActuateMotors, "Task - Actuate Motors");

    taskCreation_WebsocketHandler   = xTaskCreate(
        task_websocket_handler,         // Task function
        "Task - WebSocket Handler",     // Task name
        4096,                           // Stack size (bytes)
        NULL,                           // Parameters
        1,                              // Priority
        &xTask_WebsocketHandler         // Task handle
    );
    print_free_stack(xTask_WebsocketHandler, "Task - WebSocket Handler");

    taskCreation_SendToWiFi         = xTaskCreate(
        task_send_to_wifi,              // Task function
        "Task - Send Data",             // Task name
        2048,                           // Stack size (bytes)
        NULL,                           // Parameters
        1,                              // Priority
        &xTask_SendToWiFi               // Task handle
    );
    print_free_stack(xTask_SendToWiFi, "Task - Send Data");

    taskCreation_SendToI2C         = xTaskCreate(
        task_send_to_i2c,               // Task function
        "Task - Send I2C Data",         // Task name
        2048,                           // Stack size (bytes)
        NULL,                           // Parameters
        1,                              // Priority
        &xTask_SendToI2C                // Task handle
    );
    print_free_stack(xTask_SendToI2C, "Task - Send I2C Data");

    // Check creation status for each task
    check_task_creation(creationStatus, taskCreation_ps4Sampling,   "Task - PS4 Sampling");
    check_task_creation(creationStatus, taskCreation_UpdateEncoders, "Task - Update Encoders");
    check_task_creation(creationStatus, taskCreation_ActuateMotors, "Task - Actuate Motors");
    check_task_creation(creationStatus, taskCreation_WebsocketHandler, "Task - WebSocket Handler");
    check_task_creation(creationStatus, taskCreation_SendToWiFi, "Task - Send to WiFi");
    check_task_creation(creationStatus, taskCreation_SendToI2C, "Task - Send to I2C");


    // If any of the semaphore/mutex and queue has failed to create, exit
    if (creationStatus == 0) {
        Serial.printf("Exiting program.\n");
        stop_program();
    }

    // Display blinking LED to indicate start of program.
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Starting program.");
    // delay(500);
    // digitalWrite(LED_PIN, LOW);
}

// Nothing should be in this loop.
void loop() {}

// Task - Get input from PS4
void task_ps4_sampling(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(PS4_SAMPLING_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    bool dataUpdated;
    for (;;) {
        dataUpdated = BP32.update();
        if (dataUpdated) {
            processControllers();
        }
        // Calculate motor input based on ps4 analog stick and modifies wheelMotorps4Inputs. Does not include ramp function
        ps4_input_to_wheel_velocity();

        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task - Get encoder count from all motors//
void task_update_encoders(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_WHEEL_ENCODER_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    for (;;) {
        // Update tick velocity
        UL_Motor.update_tick_velocity();
        UR_Motor.update_tick_velocity();
        BL_Motor.update_tick_velocity();
        BR_Motor.update_tick_velocity();

        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task - Actuate all motors //
void task_actuate_motors(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_WHEEL_ACTUATION_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    for (;;) {
        // TODO: Need PWM to speed mapping
        // Calculate PD PWM output of each wheel's motor
        // wheelMotorInputs[0] = UL_Motor.PID.compute(wheelMotorInputs[0], UL_Motor.ticksPerSample);
        // wheelMotorInputs[1] = UR_Motor.PID.compute(wheelMotorInputs[1], UR_Motor.ticksPerSample);
        // wheelMotorInputs[2] = BL_Motor.PID.compute(wheelMotorInputs[2], BL_Motor.ticksPerSample);
        // wheelMotorInputs[3] = BR_Motor.PID.compute(wheelMotorInputs[3], BR_Motor.ticksPerSample);
        // Actuate Wheel Motor
        actuate_motor_wheels();

        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// WebSocket Handling //
void task_websocket_handler(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(WEBSOCKET_HANDLING_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    for (;;) {
        // Accept new WebSocket client connections
        if (!clientConnected) {
            auto newClient = server.accept();
            if (newClient.available()) {
                Serial.println("New WebSocket client connected!");
                client = newClient;
                clientConnected = true;
            }
        }

        // // (TESTING) Send data continuously to the connected client
        // if (clientConnected && client.available()) {
        //     static int printLoop = 0;
        //     printLoop++;
            
        //     // Example data to send
        //     String data = "Current Time: " + String(printLoop * 1/WEBSOCKET_HANDLING_PERIOD) + " seconds";
        //     client.send(data); // Send the message
        // }

        // Handle client disconnection
        if (clientConnected && !client.available()) {
            Serial.println("Client disconnected!");
            client.close();
            clientConnected = false;
        }

        // // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task to print messages in serial monitor (to be changed to WiFi sending)
void task_send_to_wifi(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(SEND_TO_WIFI_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    char *message;
    for (;;) {
        // Wait until there is data in the WiFi queue
        if (xQueueReceive(xQueue_wifi, &message, portMAX_DELAY)) {
            if (clientConnected && client.available()) Serial.print(message);
            // client.send(message);  // Send the received message to WebSocket server (Important: make sure 'message' is null-terminated)
        }
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task to send data to other ESP32 through I2C
void task_send_to_i2c(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(SEND_TO_I2C_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    DataPacket packet;
    for (;;) {
        // // Wait until there is data in the I2C queue
        // if (xQueueReceive(xQueue_i2c, &packet, portMAX_DELAY) == pdPASS) {
        //     Wire.beginTransmission(packet.slaveAddress);    // Set to send to specified slave
        //     Wire.write(packet.data);    // Send data
        //     if (Wire.endTransmission() == 0) {
        //         Serial.printf("Data sent successfully to slave.\n");
        //     } 
        //     else {
        //         Serial.printf("Failed to send data.\n");
        //     }
        // }
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}