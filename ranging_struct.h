#ifndef RANGING_STRUCT_H
#define RANGING_STRUCT_H


#include <stdint.h>
#include <stdbool.h>
#include "ranging_local_support.h"
#include "ranging_defconfig.h"


// -------------------- Base Struct --------------------
typedef struct {
    dwTime_t timestamp;                      
    uint16_t seqNumber;   
} __attribute__((packed)) Timestamp_Tuple_t;            // 10 byte

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} __attribute__((packed)) Coordinate_Tuple_t;

static const Timestamp_Tuple_t nullTimestampTuple = {.timestamp.full = NULL_TIMESTAMP, .seqNumber = NULL_SEQ};
static const Coordinate_Tuple_t nullCoordinateTuple = {.x = -1, .y = -1, .z = -1};

typedef enum {
    UNUSED,
    USING
} TableState;

typedef enum {
    RANGING_STATE_RESERVED,
    RANGING_STATE_S1,
    RANGING_STATE_S2,
    RANGING_STATE_S3,   // RANGING_STATE_S3 is effectively a temporary state for initialization, never been invoked
    RANGING_STATE_S4,
    RANGING_STATE_S5,
    RANGING_STATE_S6,   // RANGING_STATE_S6 is effectively a temporary state for distance calculation, never been invoked
    RANGING_TABLE_STATE_COUNT
} RANGING_TABLE_STATE;

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

// -------------------- Ranging Message --------------------
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
} __attribute__((packed)) Message_Header_t;             // 8 + 10 * MESSAGE_TX_POOL_SIZE byte = 38 byte

typedef struct {
    uint16_t address;                   // address of neighbor
    Timestamp_Tuple_t Rxtimestamp;      // last local Rxtimestamp when message is received
} __attribute__((packed)) Message_Body_Unit_t;          // 2 + 10 byte = 12 byte

typedef struct {
    Message_Header_t header;
    Message_Body_Unit_t bodyUnits[MESSAGE_BODY_UNIT_SIZE];
} __attribute__((packed)) Ranging_Message_t;            // 38 + 12 * MESSAGE_BODY_UNIT_SIZE byte = 158 byte

typedef struct {
    Ranging_Message_t rangingMessage;
    dwTime_t timestamp;                 // local timestamp when message is received
    #ifdef COORDINATE_SEND_ENABLE
    Coordinate_Tuple_t RxCoordinate;    // local coordinate when message is received
    #endif
} __attribute__((packed)) Ranging_Message_With_Additional_Info_t;


// -------------------- Ranging Table --------------------
typedef struct {
    uint8_t topIndex;
    Timestamp_Tuple_t Txtimestamps[SEND_LIST_SIZE];
    #ifdef COORDINATE_SEND_ENABLE
    Coordinate_Tuple_t TxCoordinate;    // local coordinate when message is sent
    #endif
} __attribute__((packed)) SendList_t;

/* Ranging Table
    +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  | EPTof| PTof |
    +------+------+------+------+------+------+
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | sign |
    +------+------+------+------+------+------+
    Note:       1. EPTof = Tof1 + Tof2 = Tof12
                2. PTof = Tof3 + Tof4 = Tof34
    Calculate:  1. Tof12 -> Tof23 -> Tof34 => PTof = Tof34 / 2
                2. Tof12 -> Tof23 -> Tof34 => PTof = (Tof23 + Tof34) / 4 + Compensate
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
    Timestamp_Tuple_t Rr;               // last timestamp of message received from neighbor

    float PTof;                         // pair of Tof
    float EPTof;                        // early pair of Tof

    bool continuitySign;                // true: contiguous | false: non-contiguous
    bool expirationSign;                // true: no recent access --> expired | recent access --> not expired
    int8_t initCalculateRound;          // used for initCalculatePTof

    TableState tableState;              // UNUSED / USING
    #ifdef STATE_MACHINE_ENABLE
    RANGING_TABLE_STATE rangingState;   // used for state machine
    #endif
} __attribute__((packed)) Ranging_Table_t;

typedef struct {
    uint16_t counter;                                       // number of neighbors
    uint32_t localSeqNumber;                                // seqNumber of message sent
    SendList_t sendList;                                    // timestamps of messages sent to neighbors
    Ranging_Table_t rangingTable[RANGING_TABLE_SIZE];
    Timestamp_Tuple_t lastRxtimestamp[RANGING_TABLE_SIZE];  
    index_t priorityQueue[RANGING_TABLE_SIZE];              // used for choosing neighbors to send messages
    SemaphoreHandle_t mutex;
} __attribute__((packed)) Ranging_Table_Set_t;


bool COMPARE_TIME(Timestamp_Tuple_t time_a, Timestamp_Tuple_t time_b);
void rangingTableInit(Ranging_Table_t *rangingTable);
table_index_t registerRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address);
void deregisterRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address);
void checkExpiration(Ranging_Table_Set_t *rangingTableSet);
table_index_t findRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address);
void shiftRangingTable(Ranging_Table_t *rangingTable);
void fillRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf, Timestamp_Tuple_t Re, float PTof);
void replaceRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf, Timestamp_Tuple_t Re, float PTof);
float assistedCalculatePTof(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
float calculatePTof(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf, CalculateState state);
void printRangingMessage(Ranging_Message_t *rangingMessage);
void printPriorityQueue(Ranging_Table_Set_t *rngingTableSet);
void printSendList(SendList_t *sendList);
void printRangingTable(Ranging_Table_t *rangingTable);
void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet);
void printAssistedCalculateTuple(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
void printCalculateTuple(Timestamp_Tuple_t ETb, Timestamp_Tuple_t ERb, Timestamp_Tuple_t ETp, Timestamp_Tuple_t ERp, Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp);

#endif