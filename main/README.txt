
In the FreeRTOSConfig.h file, find and modify the line:
#define configGENERATE_RUN_TIME_STATS 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

Define macros to provide a timer for run-time statistics:
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  // Initialize your timer
#define portGET_RUN_TIME_COUNTER_VALUE() (TIMx->CNT) // Read timer value
