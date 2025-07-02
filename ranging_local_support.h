#ifndef RANGING_LOCAL_SUPPORT_H
#define RANGING_LOCAL_SUPPORT_H
#define _POSIX_C_SOURCE 200809L

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


typedef struct {
    int x;
    int y;
    int z;
} __attribute__((packed)) Velocity_Tuple_t;


// BASE_SUPPORT
typedef         uint32_t                    Time_t;

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
#define         OPPOSITE_VELOCITY           7                   // vx/vy/vz = OPPOSITE_VELOCITY(m/s) / 0(m/s)
#define         OPPOSITE_DISTANCE_BASE      5000                // (mm)low distance of drones in OPPOSITE mode     

// LOCAL_HOST_H
#define         FLIGHT_AREA_UPON_BASE       50000               // (mm)upon space of area for drones to fly
#define         FLIGHT_AREA_LOW_BASE        10000               // (mm)low space of area for drones to fly

// LOCK_H
#define         QUEUE_TASK_LENGTH           3                   // length of queueTask


void DEBUG_PRINT(const char *format, ...);

#endif