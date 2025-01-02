#define MAX_ANALOG_STICK_VALUE 255
#define PS4_DEADZONE 4

void PS4_input_to_wheel_velocity (double (&motorPWM) [4], double stickLx, double stickLy, double stickRx, double stickRy);
double check_deadzone(double value);