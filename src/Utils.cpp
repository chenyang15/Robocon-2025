#include "Utils.h"
#include <Arduino.h>

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