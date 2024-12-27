#pragma once

enum Direction : unsigned char {
    FORWARD, BACKWARD, LEFT, RIGHT
};

int serialPrintf(const char *format, ...);

void stop_program();