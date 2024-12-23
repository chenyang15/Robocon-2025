#pragma once

enum Direction : unsigned char {
    FORWARD, BACKWARD, LEFT, RIGHT
};

int serialPrintf(const char *format, ...);

void stop_program();

template <typename T>
T maxValue(T value);

template <typename T, typename... Args>
T maxValue(T first, Args... rest);