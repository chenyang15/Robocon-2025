#include <Arduino.h>
#include <Bluepad32.h>
#include "PS4.h"
#include "math.h"



// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void dumpMouse(ControllerPtr ctl) {
    Serial.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                   ctl->index(),        // Controller Index
                   ctl->buttons(),      // bitmask of pressed buttons
                   ctl->scrollWheel(),  // Scroll Wheel
                   ctl->deltaX(),       // (-511 - 512) left X Axis
                   ctl->deltaY()        // (-511 - 512) left Y axis
    );
}

void dumpKeyboard(ControllerPtr ctl) {
    static const char* key_names[] = {
        // clang-format off
        // To avoid having too much noise in this file, only a few keys are mapped to strings.
        // Starts with "A", which is offset 4.
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V",
        "W", "X", "Y", "Z", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
        // Special keys
        "Enter", "Escape", "Backspace", "Tab", "Spacebar", "Underscore", "Equal", "OpenBracket", "CloseBracket",
        "Backslash", "Tilde", "SemiColon", "Quote", "GraveAccent", "Comma", "Dot", "Slash", "CapsLock",
        // Function keys
        "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
        // Cursors and others
        "PrintScreen", "ScrollLock", "Pause", "Insert", "Home", "PageUp", "Delete", "End", "PageDown",
        "RightArrow", "LeftArrow", "DownArrow", "UpArrow",
        // clang-format on
    };
    static const char* modifier_names[] = {
        // clang-format off
        // From 0xe0 to 0xe7
        "Left Control", "Left Shift", "Left Alt", "Left Meta",
        "Right Control", "Right Shift", "Right Alt", "Right Meta",
        // clang-format on
    };
    Serial.printf("idx=%d, Pressed keys: ", ctl->index());
    for (int key = Keyboard_A; key <= Keyboard_UpArrow; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = key_names[key-4];
            Serial.printf("%s,", keyName);
       }
    }
    for (int key = Keyboard_LeftControl; key <= Keyboard_RightMeta; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = modifier_names[key-0xe0];
            Serial.printf("%s,", keyName);
        }
    }
    Console.printf("\n");
}

void dumpBalanceBoard(ControllerPtr ctl) {
    Serial.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                   ctl->index(),        // Controller Index
                   ctl->topLeft(),      // top-left scale
                   ctl->topRight(),     // top-right scale
                   ctl->bottomLeft(),   // bottom-left scale
                   ctl->bottomRight(),  // bottom-right scale
                   ctl->temperature()   // temperature: used to adjust the scale value's precision
    );
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    // dumpGamepad(ctl);
}

void processMouse(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->scrollWheel() > 0) {
        // Do Something
    } else if (ctl->scrollWheel() < 0) {
        // Do something else
    }

    // See "dumpMouse" for possible things to query.
    // dumpMouse(ctl);
}

void processKeyboard(ControllerPtr ctl) {
    if (!ctl->isAnyKeyPressed())
        return;

    // This is just an example.
    if (ctl->isKeyPressed(Keyboard_A)) {
        // Do Something
        Serial.println("Key 'A' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftShift)) {
        // Do something else
        Serial.println("Key 'LEFT SHIFT' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
        // Do something else
        Serial.println("Key 'Left Arrow' pressed");
    }

    // See "dumpKeyboard" for possible things to query.
    // dumpKeyboard(ctl);
}

void processBalanceBoard(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->topLeft() > 10000) {
        // Do Something
    }

    // See "dumpBalanceBoard" for possible things to query.
    // dumpBalanceBoard(ctl);
}

void processStick(ControllerPtr ctl, int(&PS4StickOutputs)[4]) {
    PS4StickOutputs[0] = ctl->axisX();        // (-511 - 512) left X Axis
    PS4StickOutputs[1] = -1*(ctl->axisY());        // (-511 - 512) left Y axis
    PS4StickOutputs[2] = ctl->axisRX();       // (-511 - 512) right X axis
    PS4StickOutputs[3] = -1*(ctl->axisRY());       // (-511 - 512) right Y axis
}

void processControllers(int (&PS4StickOutputs)[4]) {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            processStick(myController, PS4StickOutputs);
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else if (myController->isMouse()) {
                processMouse(myController);
            } else if (myController->isKeyboard()) {
                processKeyboard(myController);
            } else if (myController->isBalanceBoard()) {
                processBalanceBoard(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

// Function to convert left and right analog stick of PS4 to velocity for each wheel motors
// Implementation method is based on this website: https://seamonsters-2605.github.io/archive/mecanum/
void PS4_input_to_wheel_velocity (double (&motorPWMArg) [4], int PS4StickOutputs [4]) {
    // If value input is low and within deadzone, ignore it
    double stickLx = (double) check_deadzone(PS4StickOutputs[0]);
    double stickLy = (double) check_deadzone(PS4StickOutputs[1]);
    double stickRx = (double) check_deadzone(PS4StickOutputs[2]);
    double stickRy = (double) check_deadzone(PS4StickOutputs[3]);
    
    // Get actuation effort and angle from left stick input
    double leftStickActuation = std::sqrt(stickLx*stickLx + stickLy*stickLy);
    double leftStickAngle = atan2(stickLy, stickLx);
    // Get actuation effort from right stick input
    double rightStickActuation;
    if (stickRx > 0) {
        rightStickActuation = hypot(stickRx, stickRy);
    } else {
        rightStickActuation = -hypot(stickRx, stickRy);
    }
    

    double motorPWM [4] = {0, 0, 0, 0}; // temporary variable for motor pwm
    // Compute motor speeds for omniwheel drive (Equations based on https://seamonsters-2605.github.io/archive/mecanum/)
    motorPWM[0] = leftStickActuation*sin(leftStickAngle + 0.25*PI) + rightStickActuation; // Upper-left motor
    motorPWM[1] = leftStickActuation*sin(leftStickAngle - 0.25*PI) - rightStickActuation; // Upper-right motor
    motorPWM[2] = leftStickActuation*sin(leftStickAngle - 0.25*PI) + rightStickActuation; // Bottom-left motor
    motorPWM[3] = leftStickActuation*sin(leftStickAngle + 0.25*PI) - rightStickActuation; // Bottom-right motor
    
    // Printing
    static int printLoop = 0;
    printLoop++;

    // Map to 0~100 PWM value, output is not clamped and can go up to 200
    motorPWM[0] = map(motorPWM[0], -MAX_ANALOG_STICK_VALUE, MAX_ANALOG_STICK_VALUE, -100, 100); // Upper-left motor
    motorPWM[1] = map(motorPWM[1], -MAX_ANALOG_STICK_VALUE, MAX_ANALOG_STICK_VALUE, -100, 100); // Upper-right motor
    motorPWM[2] = map(motorPWM[2], -MAX_ANALOG_STICK_VALUE, MAX_ANALOG_STICK_VALUE, -100, 100); // Bottom-left motor
    motorPWM[3] = map(motorPWM[3], -MAX_ANALOG_STICK_VALUE, MAX_ANALOG_STICK_VALUE, -100, 100); // Bottom-right motor
    if (printLoop % (300/100) == 0) Serial.printf("1: %.2f, 2: %.2f, 3: %.2f, 4: %.2f\n", motorPWM[0], motorPWM[1], motorPWM[2], motorPWM[3]);

    // Scale motor speeds down in case calculated motor speed is above 100
    double maxInput = max(max(abs(motorPWM[0]), abs(motorPWM[1])), max(abs(motorPWM[2]), abs(motorPWM[3])));
    if (maxInput > 100.0) {
        motorPWM[0] = (motorPWM[0]*100)/maxInput;
        motorPWM[1] = (motorPWM[1]*100)/maxInput;
        motorPWM[2] = (motorPWM[2]*100)/maxInput;
        motorPWM[3] = (motorPWM[3]*100)/maxInput;
    }
    
    // Write to output argument
    motorPWMArg[0] =  motorPWM[0];
    motorPWMArg[1] = -motorPWM[1]; // -ve to consider cw and ccw direction
    motorPWMArg[2] =  motorPWM[2]; // -ve to consider cw and ccw direction
    motorPWMArg[3] = -motorPWM[3];
}

inline int check_deadzone(int value) {
    if (value > PS4_DEADZONE) {
        return value;
    }
    else if (value < -PS4_DEADZONE){
        return value;
    }
    return 0;
}