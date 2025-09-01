#include "support.h"


uint16_t TxCount = 0;
uint16_t RxCount = 0;
dwTime_t lastTxTimestamp;                               
dwTime_t lastRxTimestamp;                               
dwTime_t TxTimestamp;                                   
dwTime_t RxTimestamp;                                   


/* DEBUG_PRINT */
void DEBUG_PRINT(const char *format, ...) {
    static bool first_call = true;
    va_list args;

    // print to console 
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    // print to file
    #if defined(IEEE_802_15_4Z)
    FILE *log_file = first_call ? fopen("./data/log/ieee.txt", "w") : fopen("./data/log/ieee.txt", "a");
    #elif defined(SWARM_RANGING_V1)
    FILE *log_file = first_call ? fopen("./data/log/swarm_v1.txt", "w") : fopen("./data/log/swarm_v1.txt", "a");
    #elif defined(SWARM_RANGING_V2)
    FILE *log_file = first_call ? fopen("./data/log/swarm_v2.txt", "w") : fopen("./data/log/swarm_v2.txt", "a");
    #elif defined(DYNAMIC_RANGING_MODE)
    FILE *log_file = first_call ? fopen("./data/log/dynamic.txt", "w") : fopen("./data/log/dynamic.txt", "a");
    #elif defined(COMPENSATE_DYNAMIC_RANGING_MODE)
    FILE *log_file = first_call ? fopen("./data/log/compensate.txt", "w") : fopen("./data/log/compensate.txt", "a");
    #endif

    if (log_file != NULL) {
        va_start(args, format);  
        vfprintf(log_file, format, args);
        va_end(args);
        fclose(log_file);
        first_call = false;
    } 
    else {
        printf("Warning: Could not open modified_Log.txt for writing\n");
    }
}

/* SemaphoreHandle_t */
SemaphoreHandle_t xSemaphoreCreateMutex() {
    pthread_mutex_t *mutex = malloc(sizeof(pthread_mutex_t));
    if (mutex == NULL) {
        perror("Failed to allocate memory for mutex");
        return NULL;
    }
    if (pthread_mutex_init(mutex, NULL) != 0) {
        free(mutex);
        perror("fail to init mutex!\n");
        return NULL;
    }
    return mutex;
}

void xSemaphoreDestroyMutex(SemaphoreHandle_t mutex) {
    if (mutex != NULL) {
        pthread_mutex_destroy(mutex);
        free(mutex);
    }
    else {
        perror("mutex is NULL ,fail to free !\n");
    }
}

int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t xTicksToWait) {
    return pthread_mutex_lock(mutex);
}

int xSemaphoreGive(SemaphoreHandle_t mutex) {
    return pthread_mutex_unlock(mutex);
}

/* SemaphoreHandle_t */
TickType_t xTaskGetTickCount() {
    TickType_t curTicks;
    if(TxTimestamp.full != 0) {
        if(lastTxTimestamp.full == 0) {
            lastTxTimestamp.full = TxTimestamp.full;
        }
        else {
            TxCount += TxTimestamp.full < lastTxTimestamp.full ? 1 : 0;
        }
        curTicks = (TxCount << 16) + (TxTimestamp.full >> 24);
        TxTimestamp.full = 0;
    }
    else if(RxTimestamp.full != 0) {
        if(lastRxTimestamp.full == 0) {
            lastRxTimestamp.full = RxTimestamp.full;
        }
        else {
            RxCount += RxTimestamp.full < lastRxTimestamp.full ? 1 : 0;
        }
        curTicks = (RxCount << 16) + (RxTimestamp.full >> 24);
        RxTimestamp.full = 0;
    }
    return curTicks;
}