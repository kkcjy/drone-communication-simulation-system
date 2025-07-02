#ifndef LOCAL_HOST_H
#define LOCAL_HOST_H

#include "ranging_local_support.h"
#include "ranging_defconfig.h"
#include "ranging_struct.h"
#include "ranging_protocol.h"


typedef struct {
    uint16_t localAddress;              // local address
    uint64_t baseTime;                  // local base time(ms)  —— for a same world time
    uint64_t lastOperationTime;         // used for modifyLocation
    #ifdef COORDINATE_SEND_ENABLE
    Coordinate_Tuple_t location;        // local location
    #endif
    uint64_t randOffTime;               // rand time(ms)        —— for diff between drones
    #if defined(RANDOM_MOVE_ENABLE) || defined(OPPOSITE_MOVE_ENABLE)
    Velocity_Tuple_t velocity;          // local vilocity
    #endif
} Local_Host_t;


uint16_t uwbGetAddress();
uint64_t get_current_milliseconds();
uint16_t string_to_hash(const char *str);
void localInit(uint16_t address);
uint64_t xTaskGetTickCount();
Coordinate_Tuple_t getCurrentLocation();
void modifyLocation();
void local_sleep(uint64_t milliseconds);

#endif