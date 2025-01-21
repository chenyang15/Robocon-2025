#include <Arduino.h>
#include "CpuUtilisation.h"
#include "Globals.h"

CpuUtilisation::CpuUtilisation(const char* taskName)
    : taskName(taskName), startTime(0), endTime(0), arrayCounter(0) {}


inline void CpuUtilisation::set_start_time() {
    this->startTime = millis();
}

inline void CpuUtilisation::set_end_time() {
    this->endTime = millis();
}

inline void CpuUtilisation::calculate_cpu_util() {
    this->cpuUtil[arrayCounter] = (this->endTime - this->startTime) * 100.0 / (CpuUtilisation::mainLoopEndTime - CpuUtilisation::mainLoopStartTime);
    
    // Increment counter
    arrayCounter++;
    if (arrayCounter == CPU_UTIL_ARR_SIZE) {
        arrayCounter = 0;
    }
}

inline void CpuUtilisation::print_cpu_util() {
    char data[50];

    // Get maximum value in cpuUtil array
    double maxCpuUtil = this->cpuUtil[0];  // Assume the first value is the maximum

    for (int i = 1; i < CPU_UTIL_ARR_SIZE; ++i) {
        if (cpuUtil[i] > maxCpuUtil) {
            maxCpuUtil = cpuUtil[i];  // Update the maximum value
        }
    }
    //sprintf(data, "%s:\t%f%%\n", this->taskName, maxCpuUtil);
    sprintf(data, "%s:\tTask Duration:%lu\tLoop time:%lu", this->taskName, this->endTime - this->startTime, CpuUtilisation::mainLoopEndTime-CpuUtilisation::mainLoopStartTime);
    Serial.print(data);
    // client.send(data);
}

inline void send_main_loop_time(unsigned long startTimeArg, unsigned long endTimeArg) {
    CpuUtilisation::mainLoopStartTime = startTimeArg;
    CpuUtilisation::mainLoopEndTime = endTimeArg;
}