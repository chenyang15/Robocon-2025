#pragma once
#include <Arduino.h>

class TaskCpuUtilization {
private:
    uint32_t startTime, endTime;    // Task start time, end time
    double taskDuration;           // Task duration (can sometimes be negative if too less delay between start and end)
    uint32_t taskExecutionCount;    // Amount of times task has run within CPU_UTIL_CALCULATION_PERIOD
    double taskUtilization;         // Percentage time spent in task during CPU_UTIL_CALCULATION_PERIOD
    const char* taskToMonitor;
    char formattedMessage[128];     // Buffer to store the message to send to WiFi
    SemaphoreHandle_t xSemaphore_ExecutedTask;

public:
    // Constructor
    TaskCpuUtilization(uint32_t taskPeriod, const char* taskToMonitor);

    inline void set_start_time();
    inline void set_end_time();
    inline void send_util_to_wifi();
};