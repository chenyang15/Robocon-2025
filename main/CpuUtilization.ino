#include "CpuUtilization.h"
#include "Timing.h"
#include "Globals.h"
#include "RuntimePrints.h"

// Constructor
TaskCpuUtilization::TaskCpuUtilization(uint32_t taskPeriod, const char* taskToMonitor)
    : startTime(0), endTime(0), taskDuration(0.0), taskUtilization(0), taskToMonitor(taskToMonitor) {
        // Find amount of times task will run within CPU_UTIL_CALCULATION_PERIOD
        taskExecutionCount = ((double) CPU_UTIL_CALCULATION_PERIOD / taskPeriod) + 0.5;

        // Create semaphore for synchronizing calculation of task utilization after task execution
        xSemaphore_ExecutedTask = xSemaphoreCreateBinary();
        // Check semaphore creation status
        if (xSemaphore_ExecutedTask == NULL)
            Serial.printf("Failed to create semaphore for CPU Utilization for '%s'", taskToMonitor);
    }

inline void TaskCpuUtilization::set_start_time() {
    // Set task start time
    startTime = micros();
}

inline void TaskCpuUtilization::set_end_time() {
    // Set task end time
    endTime = micros();
    // Give semaphore
    xSemaphoreGive(xSemaphore_ExecutedTask);
}

inline void TaskCpuUtilization::send_util_to_wifi(){
    // Take semaphore
    if (xSemaphoreTake(xSemaphore_ExecutedTask, portMAX_DELAY)) {
        // Calculate task duration
        taskDuration = (double) endTime - startTime;
        if (taskDuration > 0.0)
            // Calculate total time spent in task during CPU_UTIL_CALCULATION_PERIOD
            taskUtilization = taskExecutionCount * ((double)endTime - startTime) / (CPU_UTIL_CALCULATION_PERIOD*1000) * 100;
        else taskUtilization = 0.0;

        // Create formatted message
        snprintf(
            formattedMessage, 
            sizeof(formattedMessage), 
            "%s:\t\t%lu\t%lu\t%.2f", taskToMonitor, endTime, startTime, taskUtilization
        );
        // Send the formatted message to the queue
        xQueueSend(xQueue_wifi, &formattedMessage, 0);
    }
}