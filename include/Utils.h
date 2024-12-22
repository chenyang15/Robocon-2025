#pragma once

enum Direction : unsigned char {
    FORWARD, BACKWARD, LEFT, RIGHT
};

int serialPrintf(const char *format, ...) {
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    return Serial.print(buf);
}

void stop_program(){
    for(;;);
}

template <typename T>
T maxValue(T value) {
    return value;  // Base case: single value
}

template <typename T, typename... Args>
T maxValue(T first, Args... rest) {
    T maxRest = maxValue(rest...);  // Recursively find max of the rest
    return (first > maxRest) ? first : maxRest;
}