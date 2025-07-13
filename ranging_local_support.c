#include "ranging_local_support.h"


void DEBUG_PRINT(const char *format, ...) {
    static bool first_call = true;
    va_list args;

    // print to console 
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    
    // print to file
    FILE *log_file = first_call ? fopen("data/dataLog.txt", "w") : fopen("data/dataLog.txt", "a");
    
    if (log_file != NULL) {
        va_start(args, format);  
        vfprintf(log_file, format, args);
        va_end(args);
        fclose(log_file);
        
        first_call = false;

    } else {
        printf("Warning: Could not open dataLog.txt for writing\n");
    }
}

/*
void vTaskDelay(const TickType_t xTicksToDelay) {
    usleep(1000 * (xTicksToDelay));
}
*/

SemaphoreHandle_t xSemaphoreCreateMutex() {
    pthread_mutex_t *mutex = malloc(sizeof(pthread_mutex_t));
    if (mutex == NULL) {
        perror("xSemaphoreCreateMutex: Memory allocation failed");
        return NULL; 
    }
    if (pthread_mutex_init(mutex, NULL) != 0) {
        free(mutex); 
        perror("xSemaphoreCreateMutex: Mutex initialization failed");
        return NULL;
    }
    return mutex;
}