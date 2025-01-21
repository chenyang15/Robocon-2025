#pragma once
#include <Arduino.h>
#include "Timing.h"

#define PRINT_CPU_UTIL_PERIOD   1000
#define CPU_UTIL_ARR_SIZE       (PRINT_CPU_UTIL_PERIOD / CPU_UTIL_CALCULATION_PERIOD)

class CpuUtilisation {
private:
    unsigned long startTime;
    unsigned long endTime;
    const char* taskName;
    double cpuUtil [CPU_UTIL_ARR_SIZE];
    uint8_t arrayCounter;

public:
    static unsigned long mainLoopStartTime;
    static unsigned long mainLoopEndTime;

    // Constructor
    CpuUtilisation(const char* taskName);

    inline void set_start_time();
    inline void set_end_time();
    inline void calculate_cpu_util();
    inline void print_cpu_util();  // Gets max out of cpuUtil array
};

inline void send_main_loop_time(unsigned long startTimeArg, unsigned long endTimeArg);

// Initialize static member
unsigned long CpuUtilisation::mainLoopStartTime = 0;
unsigned long CpuUtilisation::mainLoopEndTime = 0; 