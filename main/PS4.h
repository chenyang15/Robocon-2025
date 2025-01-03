#include <Bluepad32.h>

#define MAX_ANALOG_STICK_VALUE 512
#define PS4_DEADZONE 4

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void dumpGamepad(ControllerPtr ctl);
void dumpMouse(ControllerPtr ctl);
void dumpKeyboard(ControllerPtr ctl);
void dumpBalanceBoard(ControllerPtr ctl);
void processGamepad(ControllerPtr ctl);
void processMouse(ControllerPtr ctl);
void processKeyboard(ControllerPtr ctl);
void processBalanceBoard(ControllerPtr ctl);
void processStick(ControllerPtr ctl, double(&PS4StickOutputs)[4]);
void processControllers(double (&PS4StickOutputs)[4]);
void PS4_input_to_wheel_velocity (double (&motorPWM) [4], double PS4StickOutputs[4]);
double check_deadzone(double value);