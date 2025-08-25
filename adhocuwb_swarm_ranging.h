#ifndef _SWARM_RANGING_H_
#define _SWARM_RANGING_H_


#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "dwTypes.h"


#ifdef SIMULATION_COMPILE
#include "support.h"
#else
#include "adhocuwb_init.h"
#endif

#if !defined(SNIFFER_COMPILE) && !defined(SIMULATION_COMPILE)
#include "adhocuwb_platform.h"
#include "dwm3000_init.h"
#endif


#define RANGING_DEBUG_ENABLE

#define IEEE_802_15_4Z
// #define SWARM_RANGING_V1
// #define SWARM_RANGING_V2

/* Function Switch */
// #define     ENABLE_BUS_BOARDING_SCHEME
// #define     ENABLE_DYNAMIC_RANGING_PERIOD
// #define     ENABLE_OPTIMAL_RANGING_SCHEDULE
#ifdef ENABLE_DYNAMIC_RANGING_PERIOD
#define     DYNAMIC_RANGING_COEFFICIENT         1
#endif

/* Ranging Constants */
#define     RANGING_PERIOD                      200
#define     RANGING_PERIOD_MIN                  50
#define     RANGING_PERIOD_MAX                  500

/* Queue Constants */
#define     RANGING_RX_QUEUE_SIZE               10
#define     RANGING_RX_QUEUE_ITEM_SIZE          sizeof(Ranging_Message_With_Timestamp_t)

/* Ranging Struct Constants */
#define     RANGING_MESSAGE_SIZE_MAX            UWB_PAYLOAD_SIZE_MAX
#define     RANGING_MESSAGE_PAYLOAD_SIZE_MAX    (RANGING_MESSAGE_SIZE_MAX - sizeof(Ranging_Message_Header_t))
#if defined(IEEE_802_15_4Z) || defined(SWARM_RANGING_V1)
#define     RANGING_MAX_Tr_UNIT                 1
#elif defined(SWARM_RANGING_V2)
#define     RANGING_MAX_Tr_UNIT                 5
#endif
#define     RANGING_MAX_BODY_UNIT               (RANGING_MESSAGE_PAYLOAD_SIZE_MAX / sizeof(Body_Unit_t))

#define     RANGING_TABLE_SIZE 32               // default up to 20 one-hop neighbors
#define     RANGING_TABLE_SIZE_MAX              32
#define     RANGING_TABLE_HOLD_TIME             (6 * RANGING_PERIOD_MAX)
#define     RANGING_TABLE_SET_CLEAR_PERIOD      (RANGING_TABLE_HOLD_TIME / RANGING_PERIOD)
#define     Tr_Rr_BUFFER_POOL_SIZE              6
// #define     Tf_BUFFER_POOL_SIZE                 (2 * RANGING_PERIOD_MAX / RANGING_PERIOD_MIN)
#define     Tf_BUFFER_POOL_SIZE                 6

/* Topology Sensing */
#define     NEIGHBOR_ADDRESS_MAX                32
#define     NEIGHBOR_SET_HOLD_TIME              (6 * RANGING_PERIOD_MAX)
#define     NEIGHBOR_SET_CLEAR_PERIOD           (NEIGHBOR_SET_HOLD_TIME / RANGING_PERIOD)

typedef     short                               set_index_t;


#ifdef CONFIG_UWB_LOCALIZATION_ENABLE
#define     RESET_INIT_STAGE                    123 
#define     ZERO_STAGE                          124       // 飞行阶段，0阶段随机飞
#define     FIRST_STAGE                         125
#define     SECOND_STAGE                        126
#define     LAND_STAGE                          127

/*--2添加--*/
typedef struct {
    int16_t distance_history[3];
    uint8_t index_inserting;
} median_data_t;

typedef struct {
    uint16_t address;
    int8_t stage;
    bool keepFlying;
    uint32_t keepFlyingTrueTick;
    bool alreadyTakeoff;
} leaderStateInfo_t;

typedef struct {
    uint16_t distanceTowards[RANGING_TABLE_SIZE + 1];     // cm
    short velocityXInWorld[RANGING_TABLE_SIZE + 1];       // 2byte m/s 在世界坐标系下的速度（不是机体坐标系）
    short velocityYInWorld[RANGING_TABLE_SIZE + 1];       // 2byte cm/s 在世界坐标系下的速度（不是机体坐标系）
    float gyroZ[RANGING_TABLE_SIZE + 1];                  // 4 byte rad/s
    uint16_t positionZ[RANGING_TABLE_SIZE + 1];           // 2 byte cm/s
    bool refresh[RANGING_TABLE_SIZE + 1];                 // 当前信息从上次EKF获取，到现在是否更新
    bool isNewAdd[RANGING_TABLE_SIZE + 1];                // 这个邻居是否是新加入的
    bool isNewAddUsed[RANGING_TABLE_SIZE + 1];
    bool isAlreadyTakeoff[RANGING_TABLE_SIZE + 1];
    /* 用于辅助判断这个邻居是否是新加入的（注意：这里的'新加入'指的是，
    是相对于EKF来说的，主要用于在EKF中判断是否需要执行初始化工作）*/
} neighborStateInfo_t;  /*存储正在和本无人机进行通信的邻居的所有信息（用于EKF）*/

typedef struct {
    UWB_Address_t address[RANGING_TABLE_SIZE + 1];
    int size;
} currentNeighborAddressInfo_t;   /*当前正在和本无人机进行通信的邻居地址信息*/
#endif


/* Timestamp Tuple */
typedef struct {
    dwTime_t timestamp;
    uint16_t seqNumber;
} __attribute__((packed)) Timestamp_Tuple_t;              // 10 byte

typedef union{
    struct {
        uint8_t rawtime[5];
        uint8_t address;
        uint16_t seqNumber;
    }__attribute__((packed));
    dwTime_t timestamp;                                   // 8 byte, 时间戳(低5字节)+地址+序列号, 赋值时先写时间戳再写其他字段防覆盖
} Timestamp_Tuple_t_2;

/* Ranging Message */
typedef struct {
    uint16_t srcAddress;
    uint16_t msgSequence;
    Timestamp_Tuple_t_2 lastTxTimestamps[RANGING_MAX_Tr_UNIT];
    short velocity;                                       // cm/s
    uint16_t msgLength;
    uint16_t filter;                                      // 16 bits bloom filter
    /*
    float posiX;
    float posiY;
    float posiZ;
    */
    #ifdef CONFIG_UWB_LOCALIZATION_ENABLE
    short velocityXInWorld;                               // cm/s 在世界坐标系下的速度(不是基于机体坐标系的速度)
    short velocityYInWorld;                               // cm/s 在世界坐标系下的速度(不是基于机体坐标系的速度)
    float gyroZ;                                          // rad/s
    uint16_t positionZ;                                   // cm/s
    bool keep_flying;                                     // 无人机的飞行状态
    int8_t stage;                                         // 飞行阶段
    #endif
} __attribute__((packed)) Ranging_Message_Header_t;       // 10 byte + 10 byte * MAX_Tr_UNIT

// The pre-optimized version consumes more memory
/*
typedef struct {
    struct {
        uint8_t MPR: 1;
        uint8_t RESERVED: 7;
    } flags;
    uint16_t address;
    Timestamp_Tuple_t timestamp;
} __attribute__((packed)) Body_Unit_t;                    // 13 byte
*/

typedef Timestamp_Tuple_t_2 Body_Unit_t;

typedef struct {
    Ranging_Message_Header_t header;
    Body_Unit_t bodyUnits[RANGING_MAX_BODY_UNIT];
} __attribute__((packed)) Ranging_Message_t;              // 18 + 13 byte * MAX_BODY_UNIT

// Ranging Message With RX Timestamp, used in RX Queue
typedef struct {
    Ranging_Message_t rangingMessage;
    dwTime_t rxTime;
} __attribute__((packed)) Ranging_Message_With_Timestamp_t;


/* Ranging Table Set */
typedef enum {
    RANGING_STATE_RESERVED,
    RANGING_STATE_S1,
    RANGING_STATE_S2,
    RANGING_STATE_S3,
    RANGING_STATE_S4,
    RANGING_STATE_S5,                                     // RANGING_STATE_S5 is effectively a temporary state for distance calculation
    RANGING_TABLE_STATE_COUNT
} RANGING_TABLE_STATE;

typedef enum {
    RANGING_EVENT_TX_Tf,
    RANGING_EVENT_RX_NO_Rf,
    RANGING_EVENT_RX_Rf,
    RANGING_TABLE_EVENT_COUNT
} RANGING_TABLE_EVENT;

typedef struct {
    Timestamp_Tuple_t Tr;
    Timestamp_Tuple_t Rr;
} __attribute__((packed)) Ranging_Table_Tr_Rr_Candidate_t;

// Tr and Rr candidate buffer for each Ranging Table
typedef struct {
    set_index_t latest;                                   // Index of latest valid (Tr,Rr) pair
    set_index_t cur;                                      // Index of current empty slot for next valid (Tr,Rr) pair
    Ranging_Table_Tr_Rr_Candidate_t candidates[Tr_Rr_BUFFER_POOL_SIZE];
} __attribute__((packed)) Ranging_Table_Tr_Rr_Buffer_t;

/* Ranging Table
    +------+------+------+------+------+
    |  Rp  |  Tr  |  Rf  |  P   |  tn  |
    +------+------+------+------+------+
    |  Tp  |  Rr  |  Tf  |  Re  |  ts  |
    +------+------+------+------+------+
*/
typedef struct {
    Timestamp_Tuple_t Rx;
    Timestamp_Tuple_t Tx;
} __attribute__((packed)) Ranging_Table_Tx_Rx_History_t;

typedef struct {
    uint16_t neighborAddress;

    Ranging_Table_Tx_Rx_History_t TxRxHistory;

    Timestamp_Tuple_t Rp;
    Timestamp_Tuple_t Tp;
    Ranging_Table_Tr_Rr_Buffer_t TrRrBuffer;
    Timestamp_Tuple_t Rf;
    Timestamp_Tuple_t Tf;
    Timestamp_Tuple_t Re;
    Timestamp_Tuple_t latestReceived;

    Time_t period;
    Time_t nextExpectedDeliveryTime;
    Time_t expirationTime;
    Time_t lastSendTime;
    int16_t distance;

    RANGING_TABLE_STATE state;
} __attribute__((packed)) Ranging_Table_t;

#ifndef SNIFFER_COMPILE
typedef struct {
    int size;
    SemaphoreHandle_t mu;
    Ranging_Table_t tables[RANGING_TABLE_SIZE_MAX];
} __attribute__((packed)) Ranging_Table_Set_t;

typedef void (*RangingTableEventHandler)(Ranging_Table_t *);


/* Neighbor Set */
typedef void (*neighborSetHook)(UWB_Address_t);

typedef struct {
    uint64_t bits;
    uint8_t size;
} Neighbor_Bit_Set_t;

typedef struct Neighbor_Set_Hook {
    neighborSetHook hook;
    struct Neighbor_Set_Hook_Node *next;
} Neighbor_Set_Hooks_t;

typedef struct {
    uint8_t size;
    SemaphoreHandle_t mu;
    Neighbor_Bit_Set_t oneHop;
    Neighbor_Bit_Set_t twoHop;
    // one hop neighbors can be used to reach the corresponding two hop neighbor
    Neighbor_Bit_Set_t twoHopReachSets[NEIGHBOR_ADDRESS_MAX + 1];
    // hooks for newly added neighbor which neither one-hop nor two-hop
    Neighbor_Set_Hooks_t neighborNewHooks; 
    Neighbor_Set_Hooks_t neighborExpirationHooks;
    Neighbor_Set_Hooks_t neighborTopologyChangeHooks;
    Time_t expirationTime[NEIGHBOR_ADDRESS_MAX + 1];
} Neighbor_Set_t;


#ifdef CONFIG_UWB_LOCALIZATION_ENABLE
/*相对定位代码部分*/
void initNeighborStateInfoAndMedian_data();
/*设置当前无人机已经起飞*/
void setMyTakeoff(bool isAlreadyTakeoff);
/*获取leader的阶段信息*/
int8_t getLeaderStage();
/*初始化leader状态信息*/
void initLeaderStateInfo();
/*set邻居的距离*/
void setNeighborDistance(uint16_t neighborAddress, int16_t distance);
/*set邻居的状态信息*/
void setNeighborStateInfo(uint16_t neighborAddress,  Ranging_Message_Header_t *rangingMessageHeader);
/*set邻居是否是新加入的*/
void setNeighborStateInfo_isNewAdd(uint16_t neighborAddress, bool isNewAddNeighbor);
/*get邻居的状态信息*/
bool getNeighborStateInfo(uint16_t neighborAddress, uint16_t *distance, short *vx, short *vy, float *gyroZ, uint16_t *height, bool *isNewAddNeighbor);
/*getOrSetKeepflying*/
bool getOrSetKeepflying(uint16_t RobIDfromControl, bool keep_flying);
/*get正在和本无人机进行通信的邻居地址信息，供外部调用*/
void getCurrentNeighborAddressInfo_t(currentNeighborAddressInfo_t *currentNeighborAddressInfo);
#endif


/* Ranging Operations */
void rangingInit();
int16_t getDistance(UWB_Address_t neighborAddress);
void setDistance(UWB_Address_t neighborAddress, int16_t distance,uint8_t source);

/* Tr_Rr Buffer Operations */
void rangingTableBufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer);
void rangingTableBufferUpdate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t_2 Tr, Timestamp_Tuple_t Rr);
Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tf,Timestamp_Tuple_t Tp);
Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetLatest(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer);
void rangingTableTxRxHistoryInit(Ranging_Table_Tx_Rx_History_t *history);

/* Tf Buffer Operations */
void updateTfBuffer(Timestamp_Tuple_t timestamp);
Timestamp_Tuple_t findTfBySeqNumber(uint16_t seqNumber);
Timestamp_Tuple_t getLatestTxTimestamp();
void getLatestNTxTimestamps(Timestamp_Tuple_t_2 *timestamps, int n);

/* Ranging Table Operations */
Ranging_Table_Set_t *getGlobalRangingTableSet();
void rangingTableInit(Ranging_Table_t *table, UWB_Address_t neighborAddress);
void rangingTableSetInit(Ranging_Table_Set_t *set);
bool rangingTableSetAddTable(Ranging_Table_Set_t *set, Ranging_Table_t table);
void rangingTableSetUpdateTable(Ranging_Table_Set_t *set, Ranging_Table_t table);
void rangingTableSetRemoveTable(Ranging_Table_Set_t *set, UWB_Address_t neighborAddress);
Ranging_Table_t rangingTableSetFindTable(Ranging_Table_Set_t *set, UWB_Address_t neighborAddress);
void rangingTableOnEvent(Ranging_Table_t *table, RANGING_TABLE_EVENT event);

/* Neighbor Bit Set Operations */
void neighborBitSetInit(Neighbor_Bit_Set_t *bitSet);
void neighborBitSetAdd(Neighbor_Bit_Set_t *bitSet, UWB_Address_t neighborAddress);
void neighborBitSetRemove(Neighbor_Bit_Set_t *bitSet, UWB_Address_t neighborAddress);
void neighborBitSetClear(Neighbor_Bit_Set_t *bitSet);
bool neighborBitSetHas(Neighbor_Bit_Set_t *bitSet, UWB_Address_t neighborAddress);

/* Neighbor Set Operations */
Neighbor_Set_t *getGlobalNeighborSet();
void neighborSetInit(Neighbor_Set_t *set);
bool neighborSetHas(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
bool neighborSetHasOneHop(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
bool neighborSetHasTwoHop(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
void neighborSetAddOneHopNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
void neighborSetAddTwoHopNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
void neighborSetRemoveNeighbor(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
bool neighborSetHasRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to);
void neighborSetAddRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to);
void neighborSetRemoveRelation(Neighbor_Set_t *set, UWB_Address_t from, UWB_Address_t to);
void neighborSetRegisterNewNeighborHook(Neighbor_Set_t *set, neighborSetHook hook);
void neighborSetRegisterExpirationHook(Neighbor_Set_t *set, neighborSetHook hook);
void neighborSetRegisterTopologyChangeHook(Neighbor_Set_t *set, neighborSetHook hook);
void neighborSetHooksInvoke(Neighbor_Set_Hooks_t *hooks, UWB_Address_t neighborAddress);
void neighborSetUpdateExpirationTime(Neighbor_Set_t *set, UWB_Address_t neighborAddress);
int neighborSetClearExpire(Neighbor_Set_t *set);

/* Debug Operations */
void printRangingTable(Ranging_Table_t *rangingTable);
void printRangingTableSet(Ranging_Table_Set_t *set);
void printRangingMessage(Ranging_Message_t *rangingMessage);
void printNeighborBitSet(Neighbor_Bit_Set_t *bitSet);
void printNeighborSet(Neighbor_Set_t *set);
#endif

/* Message Operation */
void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp);
Time_t generateRangingMessage(Ranging_Message_t *rangingMessage);

#endif