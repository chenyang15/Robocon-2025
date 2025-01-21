#pragma once
#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"
#include <Bluepad32.h>
#include <ArduinoWebsockets.h>
#include "CpuUtilisation.h"

/*========================================================================================
=                            WHEEL MOTOR GLOBAL VARIABLES                                =
========================================================================================*/

extern MotorWithEncoder UL_Motor; // Upper Left Wheel Motor
extern MotorWithEncoder UR_Motor; // Upper Right Wheel Motor
extern MotorWithEncoder BL_Motor; // Bottom Left Wheel Motor
extern MotorWithEncoder BR_Motor; // Bottom Right Wheel Motor

// An array of wheel motor of class MotorWithEncoder
extern MotorWithEncoder wheelMotors [4];
// Inputs for all 4 wheel motors computed from PD velocity controller.
extern double wheelMotorInputs [4];
extern double previousWheelMotorInputs [4];

/*========================================================================================
=                                PS4 GLOBAL VARIABLES                                    =
========================================================================================*/
extern int PS4StickOutputs [4];
extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];



/*========================================================================================
=                      WiFi DATA TRANSMISSION GLOBAL VARIABLES                           =
========================================================================================*/
using namespace websockets;

extern const char* ssid;        // Replace with your Wi-Fi SSID
extern const char* password;    // Replace with your Wi-Fi password

extern WebsocketsServer server; // Create a WebSocket server
extern WebsocketsClient client; // Store the connected client
extern bool clientConnected;    // Track client connection status


/*========================================================================================
=                          CPU UTILISATION GLOBAL VARIABLES                              =
========================================================================================*/
extern CpuUtilisation EncoderTask;
extern CpuUtilisation WheelActuationTask;
extern CpuUtilisation PS4SamplingTask;
extern CpuUtilisation CpuUtilTask;
extern CpuUtilisation WebSocketTask;