#include <Arduino.h>
#include "CpuUtilisation.h"
#include "Globals.h"

CpuUtilisation::CpuUtilisation(const char* taskName)
    : taskName(taskName), startTime(0), endTime(0), arrayCounter(0) {}


inline void CpuUtilisation::set_start_time() {
    this->startTime = micros();
}

inline void CpuUtilisation::set_end_time() {
    this->endTime = micros();
}

inline void CpuUtilisation::calculate_cpu_util() {
    unsigned long taskDuration = this->endTime - this->startTime;
    unsigned long loopDuration = CpuUtilisation::mainLoopEndTime - CpuUtilisation::mainLoopStartTime;
    if (loopDuration != 0) {
        if (taskDuration != 0) 
            this->cpuUtil[arrayCounter] = (double) taskDuration * 100.0 / loopDuration;
        else 
            this->cpuUtil[arrayCounter] = 0.0;
    }
    else this->cpuUtil[arrayCounter] = -1.0;
    
    
    // Increment counter
    arrayCounter++;
    if (arrayCounter == CPU_UTIL_ARR_SIZE) {
        arrayCounter = 0;
    }
}

inline void CpuUtilisation::print_cpu_util() {
    char data[128];

    // Get maximum value in cpuUtil array
    double maxCpuUtil = this->cpuUtil[0];  // Assume the first value is the maximum

    for (int i = 1; i < CPU_UTIL_ARR_SIZE; ++i) {
        if (cpuUtil[i] > maxCpuUtil) {
            maxCpuUtil = cpuUtil[i];  // Update the maximum value
        }
    }
    // sprintf(data, "%s:\t%3.2f%%\tLoop Time:%lu\n", this->taskName, maxCpuUtil, CpuUtilisation::mainLoopEndTime-CpuUtilisation::mainLoopStartTime);
    sprintf(data, "%20s\t|EndT:%lu\t|StartT:%lu\t|L-EndT:%lu\t|L-StartT:%lu\n", this->taskName, this->endTime, this->startTime, CpuUtilisation::mainLoopEndTime, CpuUtilisation::mainLoopStartTime);
    Serial.print(data);
    // client.send(data);
}

inline void send_main_loop_time(unsigned long startTimeArg, unsigned long endTimeArg) {
    CpuUtilisation::mainLoopStartTime = startTimeArg;
    CpuUtilisation::mainLoopEndTime = endTimeArg;
}