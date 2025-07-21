#ifndef RANGING_LOCAL_SUPPORT_H
#define RANGING_LOCAL_SUPPORT_H
#define _POSIX_C_SOURCE 200809L


#include <assert.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>


// BASE_SUPPORT
#define         M2T(X)                      ((unsigned int)(X))
#define 		pdFALSE 					0
#define 		pdTRUE 						1
typedef         uint32_t                    Time_t;
typedef 		uint32_t 					TickType_t;
typedef 		uint16_t 					UWB_Address_t;
typedef 		long 						BaseType_t;
typedef 		pthread_mutex_t 			*SemaphoreHandle_t;
typedef struct 	itimerval 					TimerHandle_t;
#define         ADDRESS_BASE                34697


// MODE_ENABLE
// #define         COMPENSATE_ENABLE                               // enable compensate for ranging

// // #define         RANDOM_DIFF_TIME_ENABLE                         // enable diff(0 ~ MAX_RANDOM_TIME_OFF) time between drones
// #define         MAX_RANDOM_TIME_OFF         10                  // upon of diff time between drones

// // #define         PACKET_LOSS_ENABLE                              // enable packet loss
// #define         PACKET_LOSS_RATE            25                  // percent rate of packet loss

#define         ALIGN_ENABLE                                    // enable drone to keep in stationary state for alignment
#define         ALIGN_ROUNDS                50                  // initial rounds with stationary state

/* Warning: enable only one of RANDOM_MOVE_ENABLE、OPPOSITE_MOVE_ENABLE */
// #define         RANDOM_MOVE_ENABLE                              // enable random move
#define         RANDOM_VELOCITY             3                   // vx/vy/vz < RANDOM_VELOCITY(m/s)
#define         OPPOSITE_MOVE_ENABLE                            // enable opposite flight
#define         OPPOSITE_VELOCITY           5                   // vx/vy/vz = OPPOSITE_VELOCITY(m/s) / 0(m/s)
#define         OPPOSITE_DISTANCE_BASE      5000                // (mm)low distance of drones in OPPOSITE mode     

// LOCAL_HOST_H
#define         FLIGHT_AREA_UPON_BASE       50000               // (mm)upon space of area for drones to fly
#define         FLIGHT_AREA_LOW_BASE        10000               // (mm)low space of area for drones to fly

// LOCK_H
#define         QUEUE_TASK_LENGTH           3                   // length of queueTask

#define 		systemWaitStart() 			vTaskDelay(10)

// UWB
#define 		UWB_PACKET_SIZE_MAX 		256
#define 		UWB_PAYLOAD_SIZE_MAX 		(UWB_PACKET_SIZE_MAX - sizeof(UWB_Packet_Header_t))
#define 		UWB_DEST_ANY 				65535
#define 		UWB_DEST_EMPTY 				65534
#define 		portMAX_DELAY 				(TickType_t)0xffffffffUL
#define 		ADHOC_UWB_RANGING_TX_TASK_NAME 	"uwbRangingTxTask"
#define 		ADHOC_UWB_RANGING_RX_TASK_NAME 	"uwbRangingRxTask"
#define 		ADHOC_UWB_TASK_PRI 			osPriorityNormal
#define 		UWB_TASK_STACK_SIZE 		(2 * UWB_FRAME_LEN_MAX)
#define 		RANGING_TABLE_HOLD_TIME 	(6 * RANGING_PERIOD_MAX)

// Queue
#define 		RANGING_RX_QUEUE_SIZE 		10
#define 		RANGING_RX_QUEUE_ITEM_SIZE 	sizeof(Ranging_Message_With_Timestamp_t)


typedef struct {
    int x;
    int y;
    int z;
} __attribute__((packed)) Velocity_Tuple_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} __attribute__((packed)) Coordinate_Tuple_t;

typedef union {
	uint8_t raw[5];
	uint64_t full;
	struct {
		uint32_t low32;
		uint8_t high8;
	} __attribute__((packed));
	struct {
		uint8_t low8;
		uint32_t high32;
	} __attribute__((packed));
} dwTime_t;		

/*
typedef enum {
	UWB_REVERSED_MESSAGE,
	UWB_TRANSCEIVE_MESSAGE,
	UWB_RANGING_MESSAGE,
	UWB_FLOODING_MESSAGE,
	UWB_DATA_MESSAGE,
	UWB_AODV_MESSAGE,
	UWB_OLSR_MESSAGE,
	PRINT,
	SNIFFER,
	UWB_MESSAGE_TYPE_COUNT,
} UWB_MESSAGE_TYPE;

typedef struct {
  UWB_Address_t srcAddress;
  UWB_Address_t destAddress;
  uint16_t seqNumber;
  struct {
	  UWB_MESSAGE_TYPE type: 6;
      uint16_t length: 10;
    } __attribute__((packed));
} __attribute__((packed)) UWB_Packet_Header_t;

typedef struct {
  UWB_Packet_Header_t header;
  uint8_t payload[UWB_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) UWB_Packet_t;

typedef void (*UWBCallback)(void *);

typedef struct {
  UWB_MESSAGE_TYPE type;
  QueueHandle_t rxQueue;
  UWBCallback rxCb;
  UWBCallback txCb;
} UWB_Message_Listener_t;
*/


void DEBUG_PRINT(const char *format, ...);
void vTaskDelay(const TickType_t xTicksToDelay);
SemaphoreHandle_t xSemaphoreCreateMutex();

#endif