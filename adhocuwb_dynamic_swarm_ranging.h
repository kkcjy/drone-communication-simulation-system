#ifndef DYNAMIC_SWARM_RANGING
#define DYNAMIC_SWARM_RANGING


#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef SIMULATION_COMPILE
#include "support.h"
#else
#include "adhocuwb_init.h"
#endif

#if !defined(SNIFFER_COMPILE) && !defined(SIMULATION_COMPILE)
#include "adhocuwb_platform.h"
#include "dwm3000_init.h"
#endif


/* -------------------- Define -------------------- */
#define         CLASSIC_SUPPORT_ENABLE
// #define         STATIC_COMPENSATE_ENABLE                // suitable for mobile scenarios
// #define         DYNAMIC_COMPENSATE_ENABLE               // suitable for scenarios with repeated switching between static and dynamic states
// #define         COORDINATE_SEND_ENABLE
// #define         PACKET_LOSS_ENABLE
#define         OPTIMAL_RANGING_SCHEDULE_ENABLE

/* Ranging Constants */
#define         RANGING_PERIOD              50
#define         RANGING_PERIOD_MIN          50
#define         RANGING_PERIOD_MAX          500

/* Ranging Message */
#define         MESSAGE_TX_POOL_SIZE        3
#define         MESSAGE_BODYUNIT_SIZE       3

/* Ranging Table Set */
#define         Tr_Rr_BUFFER_SIZE           3
#define         RANGING_TABLE_SIZE          10
#define         SEND_LIST_SIZE              5
#define         RECEIVE_LIST_SIZE           RANGING_TABLE_SIZE

/* Effective Distance Range */
#define         UPPER_BOUND_DISTANCE        1000
#define         LOWER_BOUND_DISTANCE        0

/* Compensation Coefficient */
#define         SEQGAP_THRESHOLD            3
#if defined(STATIC_COMPENSATE_ENABLE)
#define         COMPENSATE_RATE             0.7f
#define         DECELERATION_BOUND          15
#elif defined(DYNAMIC_COMPENSATE_ENABLE)
#define         MOTION_THRESHOLD            3
#define         COMPENSATE_RATE_LOW         0.1f
#define         DECELERATION_BOUND_LOW      1
#define         COMPENSATE_RATE_HIGH        0.7f
#define         DECELERATION_BOUND_HIGH     15
#endif

/* Queue Constants */
#define         RANGING_RX_QUEUE_SIZE       10
#define         RANGING_RX_QUEUE_ITEM_SIZE  sizeof(Ranging_Message_With_Additional_Info_t)

/* Else */
#define         CHECK_PERIOD                15
#define         CONVERGENCE_THRESHOLD       0.989f
#define         NEIGHBOR_ADDRESS_MAX        32
#define         PACKET_LOSS_RATE            0.1f
#define         UWB_MAX_TIMESTAMP           1099511627776
#define         VELOCITY                    0.4691763978616f

/* Index */
#define         index_t                     uint16_t
#define         table_index_t               uint8_t

/* Invalid Value */
#define         NULL_ADDRESS                0xFFFF
#define         NULL_ADDR                   0xFF
#define         NULL_DIS                    -1.0f
#define         NULL_INDEX                  0xFF
#define         NULL_SEQ                    0x0
#define         NULL_TIMESTAMP              0xFFFFFFFFFFU
#define         NULL_TOF                    -1.0f


/* -------------------- Base Struct -------------------- */
typedef struct {
    dwTime_t timestamp;
    uint16_t seqNumber;
} __attribute__((packed)) Timestamp_Tuple_t;            // 10 byte

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} __attribute__((packed)) Coordinate_Tuple_t;           // 48 byte

typedef enum {
    UNUSED,
    USING
} TableState;

typedef enum {
    RANGING_STATE_RESERVED,
    RANGING_STATE_S1,
    RANGING_STATE_S2,
    RANGING_STATE_S3,       // RANGING_STATE_S3 is effectively a temporary state for initialization, never been invoked
    RANGING_STATE_S4,
    RANGING_STATE_S5,
    RANGING_STATE_S6,       // RANGING_STATE_S6 is effectively a temporary state for distance calculation, never been invoked
    RANGING_TABLE_STATE_COUNT
} RANGING_TABLE_STATE;

typedef enum {
    RANGING_SUBSTATE_RESERVED,
    RANGING_SUBSTATE_S1,    // initialize
    RANGING_SUBSTATE_S2,    // swarm ranging
    RANGING_SUBSTATE_S3,    // dynamic swarm ranging
    RANGING_TABLE_SUBSTATE_COUNT
} RANGING_TABLE_SUBSTATE;   // used during RANGING_STATE_S4 ~ RANGING_STATE_S6

typedef enum {
    RANGING_EVENT_TX,
    RANGING_EVENT_RX,
    RANGING_EVENT_RX_NO,
    RANGING_TABLE_EVENT_COUNT
} RANGING_TABLE_EVENT;

typedef enum {
    FIRST_CALCULATE,
    SECOND_CALCULATE
} CalculateState;


/* -------------------- Ranging Message -------------------- */
typedef struct {
    uint16_t srcAddress;                // address of the source of message
    uint16_t msgSequence;               // sequence of message
    Timestamp_Tuple_t Txtimestamps[MESSAGE_TX_POOL_SIZE];
                                        // last local Txtimestamps when message is sent
    #ifdef COORDINATE_SEND_ENABLE
    Coordinate_Tuple_t TxCoordinate;    // local coordinate when message is sent
    #endif
    uint16_t filter;                    // bloom filter
    uint16_t msgLength;                 // size of message
} __attribute__((packed)) Ranging_Message_Header_t;     // 8 + 10 * MESSAGE_TX_POOL_SIZE byte = 38 byte

typedef union{
    struct {
        uint8_t rawtime[5]; 
        uint8_t address;                // address of neighbor
        uint16_t seqNumber;             // last local Rxtimestamp.seqNumber when message is received
    } __attribute__((packed));
    dwTime_t timestamp;                 // last local Rxtimestamp.timestamp when message is received
} Ranging_Message_Body_Unit_t;                          // 8 byte

typedef struct {
    Ranging_Message_Header_t header;
    Ranging_Message_Body_Unit_t bodyUnits[MESSAGE_BODYUNIT_SIZE];
} __attribute__((packed)) Ranging_Message_t;            // 38 + 12 * MESSAGE_BODYUNIT_SIZE byte = 158 byte

typedef struct {
    Ranging_Message_t rangingMessage;
    dwTime_t timestamp;                 // local timestamp when message is received
    #ifdef COORDINATE_SEND_ENABLE
    Coordinate_Tuple_t RxCoordinate;    // local coordinate when message is received
    #endif
} __attribute__((packed)) Ranging_Message_With_Additional_Info_t;


/* -------------------- Ranging Table -------------------- */
typedef struct {
    index_t topIndex;
    Timestamp_Tuple_t Txtimestamps[SEND_LIST_SIZE];
    #ifdef COORDINATE_SEND_ENABLE
    Coordinate_Tuple_t TxCoordinate;    // local coordinate when message is sent
    #endif
} __attribute__((packed)) SendList_t;

typedef struct {
    index_t topIndex;
    dwTime_t Rxtimestamps[RECEIVE_LIST_SIZE];
} __attribute__((packed)) ReceiveList_t;

typedef struct {
    Timestamp_Tuple_t Tr;
    Timestamp_Tuple_t Rr;
} __attribute__((packed)) Ranging_Table_Tr_Rr_Candidate_t;

typedef struct {
    index_t topIndex;                   // index of latest valid (Tr,Rr) pair
    Ranging_Table_Tr_Rr_Candidate_t candidates[Tr_Rr_BUFFER_SIZE];
} __attribute__((packed)) Ranging_Table_Tr_Rr_Buffer_t;

/* Ranging Table
    +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |
    +------+------+------+------+------+------+------+
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |
    +------+------+------+------+------+------+------+
    |    EPToF    |    PToF     |
    +------+------+------+------+
    Note:   1. EPToF = ToF_eb + ToF_ep = ToF_ebep
            2. PToF = ToF_b + ToFp = ToF_bp
*/
typedef struct {
    uint16_t neighborAddress;

    Timestamp_Tuple_t ETb;
    Timestamp_Tuple_t ERb;
    Timestamp_Tuple_t ETp;
    Timestamp_Tuple_t ERp;
    Timestamp_Tuple_t Tb;
    Timestamp_Tuple_t Rb;
    Timestamp_Tuple_t Tp;
    Timestamp_Tuple_t Rp;
    Ranging_Table_Tr_Rr_Buffer_t TrRrBuffer;
    Timestamp_Tuple_t Tf;
    Timestamp_Tuple_t Rf;
    Timestamp_Tuple_t Re;

    float PToF;                         // pair of ToF
    float EPToF;                        // early pair of ToF

    bool continuitySign;                // true: contiguous | false: non-contiguous
    bool expirationSign;                // true: no recent access --> expired | recent access --> not expired

    TableState tableState;              // UNUSED / USING
    RANGING_TABLE_STATE rangingState;   // used for state machine
} __attribute__((packed)) Ranging_Table_t;

typedef struct {
    uint16_t size;                                          // number of neighbors
    uint32_t localSeqNumber;                                // seqNumber of message sent
    SendList_t sendList;                                    // timestamps of messages sent to neighbors
    #ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
    ReceiveList_t receiveList;                              // timestamps of messages received from neighbors
    #endif
    Ranging_Table_t rangingTable[RANGING_TABLE_SIZE];
    Timestamp_Tuple_t lastRxtimestamp[RANGING_TABLE_SIZE];  // timestamps of last received messages from neighbors
    index_t priorityQueue[RANGING_TABLE_SIZE];              // circular priority queue used for choosing neighbors to send messages
    #ifndef SNIFFER_COMPILE
    SemaphoreHandle_t mutex;
    #endif
} __attribute__((packed)) Ranging_Table_Set_t;


/* -------------------- Null Struct -------------------- */
static const Timestamp_Tuple_t nullTimestampTuple = {.timestamp.full = NULL_TIMESTAMP, .seqNumber = NULL_SEQ};
#ifdef COORDINATE_SEND_ENABLE
static const Coordinate_Tuple_t nullCoordinateTuple = {.x = -1, .y = -1, .z = -1};
#endif
static const Ranging_Table_Tr_Rr_Candidate_t nullCandidate = {.Tr.timestamp.full = NULL_TIMESTAMP, .Tr.seqNumber = NULL_SEQ, .Rr.timestamp.full = NULL_TIMESTAMP, .Rr.seqNumber = NULL_SEQ,};


/* -------------------- Base Operation -------------------- */
bool COMPARE_TIME(uint64_t time_a, uint64_t time_b);


/* -------------------- Ranging Table Set Operation -------------------- */
index_t findSendList(SendList_t *sendList, uint16_t seqNumber);
#ifdef COORDINATE_SEND_ENABLE
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple, Coordinate_Tuple_t coordinateTuple);
#else
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple);
#endif
#ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
void updateReceiveList(ReceiveList_t *receiveList, dwTime_t timestamp);
#endif

void rangingTableTr_Rr_BufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer);
void updateRangingTableTr_Buffer(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tr);
void updateRangingTableRr_Buffer(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Rr);
Ranging_Table_Tr_Rr_Candidate_t rangingTableTr_Rr_BufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tf);

void rangingTableInit(Ranging_Table_t *rangingTable);
table_index_t registerRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address);
void deregisterRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address);
table_index_t findRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address);
void fillRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf, Timestamp_Tuple_t Re);
void shiftRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, float PToF);
void replaceRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, float PToF);

void updatePriorityQueue(Ranging_Table_Set_t *rangingTableSet, int8_t shiftCount);

void rangingTableSetInit();

void checkExpirationCallback(Ranging_Table_Set_t *rangingTableSet);

int getCurrentSubstate(Ranging_Table_t *rangingTable);


/* -------------------- Calculation Function -------------------- */
float rangingAlgorithm(Timestamp_Tuple_t T1, Timestamp_Tuple_t R1, Timestamp_Tuple_t T2, Timestamp_Tuple_t R2, Timestamp_Tuple_t T3, Timestamp_Tuple_t R3, float ToF12);
float classicCalculatePToF(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
float calculatePToF(Ranging_Table_t *rangingTable, Ranging_Table_Tr_Rr_Candidate_t candidate);
void continuousPacketLossHandler(Ranging_Table_t *rangingTable, Ranging_Table_Tr_Rr_Candidate_t candidate);


/* -------------------- Print Function -------------------- */
void printRangingMessage(Ranging_Message_t *rangingMessage);
void printSendList(SendList_t *sendList);
void printRangingTable(Ranging_Table_t *rangingTable);
void printPriorityQueue(Ranging_Table_Set_t *rangingTableSet);
void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet);
void printclassicCalculateTuple(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
void printCalculateTuple(Timestamp_Tuple_t ETb, Timestamp_Tuple_t ERb, Timestamp_Tuple_t ETp, Timestamp_Tuple_t ERp, Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp);


/* -------------------- State Machine Operation -------------------- */
typedef void (*EventHandlerTable)(Ranging_Table_t*);


/* -------------------- Generate and Process -------------------- */
void generateDSRMessage(Ranging_Message_t *rangingMessage);
void processDSRMessage(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo);

#endif