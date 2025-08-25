#ifndef SUPPORT_H
#define SUPPORT_H


#include <assert.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dwTypes.h"


#define SWARM_RANGING_MODE
// #define DYNAMIC_SWARM_RANGING_MODE


typedef         uint16_t                    UWB_Address_t;
typedef         uint32_t                    TickType_t;
typedef         long                        BaseType_t;
#define         ASSERT                      assert
#define         DWT_TIME_UNITS              (1.0/499.2e6/128.0) 
#define         M2T(X)                      ((unsigned int)(X))
#define         pdTRUE                      ((BaseType_t)1)
#define         portTickType                TickType_t
#define         portMAX_DELAY               (TickType_t)0xffffffffUL
#define         UWB_DEST_EMPTY              65534
#define         UWB_FRAME_LEN_MAX           256
#define         UWB_PACKET_SIZE_MAX         UWB_FRAME_LEN_MAX
#define         UWB_PAYLOAD_SIZE_MAX        (UWB_PACKET_SIZE_MAX - sizeof(UWB_Packet_Header_t))
#define         UWB_MAX_TIMESTAMP           1099511627776
typedef         pthread_mutex_t             *SemaphoreHandle_t;
typedef         portTickType                Time_t;


typedef enum {
    UWB_REVERSED_MESSAGE = 0,
    UWB_TRANSCEIVE_MESSAGE = 1,
    UWB_RANGING_MESSAGE = 2,
    UWB_FLOODING_MESSAGE = 3,
    UWB_DATA_MESSAGE = 4,
    UWB_AODV_MESSAGE = 5,
    UWB_OLSR_MESSAGE = 6,
    PRINT = 7,
    SNIFFER = 8,
    UWB_MESSAGE_TYPE_COUNT,
} UWB_MESSAGE_TYPE;

typedef struct {
    UWB_Address_t srcAddress;
    UWB_Address_t destAddress;
    uint16_t seqNumber;
    struct {
        UWB_MESSAGE_TYPE type : 6;
        uint16_t length : 10;
    } __attribute__((packed));
} __attribute__((packed)) UWB_Packet_Header_t;


/* DEBUG_PRINT */
void DEBUG_PRINT(const char *format, ...);

/* SemaphoreHandle_t */
SemaphoreHandle_t xSemaphoreCreateMutex();
void xSemaphoreDestroyMutex(SemaphoreHandle_t mutex);
int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t xTicksToWait);
int xSemaphoreGive(SemaphoreHandle_t mutex);

/* TimerHandle_t */
TickType_t xTaskGetTickCount();
#endif