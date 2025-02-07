// IR_Encoder.h - IR Encoder Library Header File
#pragma once
#include <Arduino.h>
class IREncoder {
private:
    static IREncoder* instances[2]; // Supports up to 2 instances (expand as needed)
    uint8_t encoderPin;             // Pin connected to the IR encoder
    volatile unsigned int pulseCount; // Pulse count (volatile for ISR)
    unsigned long lastTime;         // Time of the last RPM calculation
    unsigned long interval;         // Time interval for RPM calculation
    float rpm;                      // Calculated RPM
    uint8_t instanceIndex;          // Instance index (0 or 1)

    // Static ISR handlers
    static void handleInterrupt0() { if (instances[0]) instances[0]->incrementPulse(); }
    static void handleInterrupt1() { if (instances[1]) instances[1]->incrementPulse(); }

    // Increment pulse count (called by ISR)
    void incrementPulse() { pulseCount++; }

public:
    // Constructor
    IREncoder(uint8_t pin, unsigned long calcInterval = 1000);

    // Initialize the encoder
    void begin();

    // Get RPM
    float getRPM();

    // Reset pulse count
    void reset();
};

// Initialize static instance array
IREncoder* IREncoder::instances[2] = {nullptr, nullptr};
