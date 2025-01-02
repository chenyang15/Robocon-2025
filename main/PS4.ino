#include <Arduino.h>
#include "PS4.h"
#include "math.h"

// Function to convert left and right analog stick of PS4 to velocity for each wheel motors
// Implementation method is based on this website: https://seamonsters-2605.github.io/archive/mecanum/
void PS4_input_to_wheel_velocity (double (&motorPWM) [4], double stickLxRaw, double stickLyRaw, double stickRxRaw, double stickRyRaw) {        
    // If value input is low and within deadzone, ignore it
    double stickLx = check_deadzone(stickLxRaw);
    double stickLy = check_deadzone(stickLyRaw);
    double stickRx = check_deadzone(stickRxRaw);
    double stickRy = check_deadzone(stickRyRaw);
    
    // Get actuation effort and angle from left stick input
    double leftStickActuation = std::sqrt(stickLx*stickLx + stickLy*stickLy);
    double leftStickAngle = atan2(stickLy, stickLx);
    // Get actuation effort from right stick input
    double rightStickActuation = hypot(stickRx, stickRy);

    // Compute motor speeds for omniwheel drive
    motorPWM[0] = leftStickActuation*sin(leftStickAngle + 0.25*PI) + rightStickActuation; // Upper-left motor
    motorPWM[1] = leftStickActuation*sin(leftStickAngle - 0.25*PI) + rightStickActuation; // Upper-right motor
    motorPWM[0] = map(motorPWM[0], -MAX_ANALOG_STICK_VALUE, MAX_ANALOG_STICK_VALUE, 0, 100); // map to 0~100 PWM value, output is not clamped and can go up to 200
    motorPWM[1] = map(motorPWM[1], -MAX_ANALOG_STICK_VALUE, MAX_ANALOG_STICK_VALUE, 0, 100); // map to 0~100 PWM value, output is not clamped and can go up to 200
    motorPWM[2] = motorPWM[1];  // Bottom-left motor (same as upper-right motor)
    motorPWM[3] = motorPWM[0];  // Bottom-right motor (same as upper-left motor)

    // Scale motor speeds down in case motor speed is above 100
    double maxInput = max(abs(motorPWM[0]), abs(motorPWM[1]));
    if (maxInput > 100.0) {
        motorPWM[0] /= maxInput;
        motorPWM[1] /= maxInput;
        motorPWM[2] /= maxInput;
        motorPWM[3] /= maxInput;
    }
}

double check_deadzone(double value) {
    if (value > PS4_DEADZONE) {
        return value;
    }
    else if (value < -PS4_DEADZONE){
        return value;
    }
    return 0.0;
}