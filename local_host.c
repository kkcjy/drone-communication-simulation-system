#include "local_host.h"


Local_Host_t *localHost;
extern Ranging_Table_Set_t *rangingTableSet;

static _Atomic uint64_t last_time = 0;


uint16_t uwbGetAddress() {
    return localHost->localAddress;
}

// get current time(ms) 
uint64_t get_current_milliseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    uint64_t current_time = (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;

    uint64_t expected = last_time;
    while (current_time <= expected) {
        current_time = expected + 1;
        if (atomic_compare_exchange_weak(&last_time, &expected, current_time)) {
            break;
        }
    }
    atomic_store(&last_time, current_time);
    return current_time;
}

// convert local address by hash
uint16_t string_to_hash(const char *str) {
    uint16_t hash = 0;
    for (int i = 0; str[i] != '\0'; i++) {
        hash = (hash << 5) + hash + str[i];  
    }
    return hash;
}

// set localAddress and randOffTime(ms)
void localInit(uint16_t address) {
    localHost = (Local_Host_t*)malloc(sizeof(Local_Host_t));
    localHost->localAddress = address;
    localHost->baseTime = 0;
    localHost->lastOperationTime = 0;

    srand((unsigned int)(get_current_milliseconds()));

    #ifdef COORDINATE_SEND_ENABLE
        #if defined(RANDOM_MOVE_ENABLE)
            localHost->location.x = FLIGHT_AREA_LOW_BASE + rand() % (FLIGHT_AREA_UPON_BASE - FLIGHT_AREA_LOW_BASE + 1);
            localHost->location.y = FLIGHT_AREA_LOW_BASE + rand() % (FLIGHT_AREA_UPON_BASE - FLIGHT_AREA_LOW_BASE + 1);
            localHost->location.z = FLIGHT_AREA_LOW_BASE + rand() % (FLIGHT_AREA_UPON_BASE - FLIGHT_AREA_LOW_BASE + 1);
        #elif defined(OPPOSITE_MOVE_ENABLE)
            localHost->location.x = localHost->localAddress % 2 == 0 ? OPPOSITE_DISTANCE_BASE : 0;
            localHost->location.y = 0;
            localHost->location.z = 0;
        #endif
    #endif

    #if defined(RANDOM_MOVE_ENABLE)
        localHost->velocity.x = rand() % (2 * RANDOM_VELOCITY + 1) - RANDOM_VELOCITY;
        localHost->velocity.y = rand() % (2 * RANDOM_VELOCITY + 1) - RANDOM_VELOCITY;
        localHost->velocity.z = rand() % (2 * RANDOM_VELOCITY + 1) - RANDOM_VELOCITY;
    #elif defined(OPPOSITE_MOVE_ENABLE)
        localHost->velocity.x = localHost->localAddress % 2 == 0 ? OPPOSITE_VELOCITY : 0;
        localHost->velocity.y = 0;
        localHost->velocity.z = 0;
    #endif

    #ifdef RANDOM_DIFF_TIME_ENABLE
        localHost->randOffTime = rand() % (MAX_RANDOM_TIME_OFF + 1);
    #endif
}

// return current time(ms)
uint64_t xTaskGetTickCount() {
    #if RANDOM_DIFF_TIME_ENABLE
        return (get_current_milliseconds() - localHost->baseTime) + localHost->randOffTime;
    #else
        return (get_current_milliseconds() - localHost->baseTime);
    #endif
}

Coordinate_Tuple_t getCurrentLocation() {
    return localHost->location;
}

void reverseVilocity() {
    #if defined(RANDOM_MOVE_ENABLE)
        if(localHost->location.x < FLIGHT_AREA_LOW_BASE) {
            localHost->velocity.x = abs(localHost->velocity.x);
        }
        else if(localHost->location.x > FLIGHT_AREA_UPON_BASE) {
            localHost->velocity.x = -abs(localHost->velocity.x);
        }
        if(localHost->location.y < FLIGHT_AREA_LOW_BASE) {
            localHost->velocity.y = abs(localHost->velocity.y);
        }
        else if(localHost->location.y > FLIGHT_AREA_UPON_BASE) {
            localHost->velocity.y = -abs(localHost->velocity.y);
        }
        if(localHost->location.z < FLIGHT_AREA_LOW_BASE) {
            localHost->velocity.z = abs(localHost->velocity.z);
        }
        else if(localHost->location.z > FLIGHT_AREA_UPON_BASE) {
            localHost->velocity.z = -abs(localHost->velocity.z);
        }
    #elif defined(OPPOSITE_MOVE_ENABLE) 
        if(localHost->location.x < OPPOSITE_DISTANCE_BASE) {
            localHost->velocity.x = localHost->localAddress % 2 == 0 ? OPPOSITE_VELOCITY : 0;
        } 
        else if(localHost->location.x > FLIGHT_AREA_UPON_BASE) {
            localHost->velocity.x = localHost->localAddress % 2 == 0 ? -OPPOSITE_VELOCITY : 0;
        }
    #endif
}

void modifyLocation() {
    #ifdef ALIGN_ENABLE
        if(rangingTableSet->localSeqNumber < ALIGN_ROUNDS) {
            localHost->lastOperationTime = xTaskGetTickCount();
        }
        else {
            uint64_t curTime = xTaskGetTickCount();
            uint64_t diffTime = curTime - localHost->lastOperationTime;
            localHost->lastOperationTime = curTime;
            localHost->location.x += localHost->velocity.x * diffTime;
            localHost->location.y += localHost->velocity.y * diffTime;
            localHost->location.z += localHost->velocity.z * diffTime;
        }
    #else
        if(localHost->lastOperationTime == 0) {
            localHost->lastOperationTime = xTaskGetTickCount();
        }
        else {
            uint64_t curTime = xTaskGetTickCount();
            uint64_t diffTime = curTime - localHost->lastOperationTime;
            localHost->lastOperationTime = curTime;
            localHost->location.x += localHost->velocity.x * diffTime;
            localHost->location.y += localHost->velocity.y * diffTime;
            localHost->location.z += localHost->velocity.z * diffTime;
        }
    #endif

    reverseVilocity();
}

void local_sleep(uint64_t milliseconds) {
    struct timespec req, rem;
    req.tv_sec = milliseconds / 1000;
    req.tv_nsec = (milliseconds % 1000) * 1000000; 
    
    while (nanosleep(&req, &rem) == -1) {
        req = rem;
    }
}