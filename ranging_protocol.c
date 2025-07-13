#include <stdlib.h>

#include "ranging_protocol.h"


static uint16_t MY_UWB_ADDRESS;
Ranging_Table_Set_t *rangingTableSet;
#ifdef COMPENSATE_ENABLE
static float lastD = 0;
#endif

/*
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;
*/


// -------------------- Ranging Table Set Operation --------------------
void rangingTableSetInit() {
    MY_UWB_ADDRESS = uwbGetAddress();

    rangingTableSet = (Ranging_Table_Set_t*)malloc(sizeof(Ranging_Table_Set_t));
    rangingTableSet->counter = 0;
    rangingTableSet->localSeqNumber = NULL_SEQ;
    rangingTableSet->sendList.topIndex = NULL_INDEX;
    for (int i = 0; i < SEND_LIST_SIZE; i++) {
        rangingTableSet->sendList.Txtimestamps[i] = nullTimestampTuple;
    }
    #ifdef COORDINATE_SEND_ENABLE
        rangingTableSet->sendList.TxCoordinate = nullCoordinateTuple;
    #endif
    for (int i = 0; i < RANGING_TABLE_SIZE; i++) {
        rangingTableInit(&rangingTableSet->rangingTable[i]);
        rangingTableSet->lastRxtimestamp[i] = nullTimestampTuple;
        rangingTableSet->priorityQueue[i] = NULL_INDEX;
    }
    rangingTableSet->mutex = xSemaphoreCreateMutex();
}

// search for index of send list whose Tx seqNumber is same as seqNumber
index_t findSendList(SendList_t *sendList, uint16_t seqNumber) {
    if(seqNumber == NULL_SEQ) {
        return NULL_INDEX;
    }
    index_t index = sendList->topIndex;
    for(int i = 0; i < SEND_LIST_SIZE; i++) {
        if(sendList->Txtimestamps[index].seqNumber == seqNumber) {
            return index;
        }
        index = (index - 1 + SEND_LIST_SIZE) % SEND_LIST_SIZE;
    }
    DEBUG_PRINT("Sequence number %u not found in send list\n", seqNumber);
    return NULL_INDEX;
}

#ifdef COORDINATE_SEND_ENABLE
// update the send list with new Tx and Rx timestamps with coordinate
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple, Coordinate_Tuple_t coordinateTuple) {
    sendList->topIndex = (sendList->topIndex + 1) % SEND_LIST_SIZE;
    sendList->Txtimestamps[sendList->topIndex] = timestampTuple;
    sendList->TxCoordinate = coordinateTuple;
}
#else
// update the send list with new Tx and Rx timestamps without coordinate
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple) {
    sendList->topIndex = (sendList->topIndex + 1) % SEND_LIST_SIZE;
    sendList->Txtimestamps[sendList->topIndex] = timestampTuple;
}
#endif

// update the priority queue, push address to the tail
void updatePriority(Ranging_Table_Set_t *rangingTableSet, uint16_t address) {
    index_t *queue = rangingTableSet->priorityQueue;

    for(int i = 0; i < rangingTableSet->counter; i++) {
        if(rangingTableSet->rangingTable[i].neighborAddress == address) {
            // move the index of table to the tail of priority queue
            for(int j = i; j < rangingTableSet->counter - 1; j++) {
                uint8_t tmp = queue[j];
                queue[j] = queue[j + 1];
                queue[j + 1] = tmp;
            }
            queue[rangingTableSet->counter - 1] = i;
            return;
        }
    }
}


// -------------------- State Machine Operation --------------------
#ifdef STATE_MACHINE_ENABLE
static void RESERVED_STUB(Ranging_Table_t *rangingTable) {
    assert(0 && "[RESERVED_STUB]: Should not be called\n");
}

static void S1_TX(Ranging_Table_t *rangingTable) {
    // Don't update Tx here since sending message is an async action, we put all Tx in sendList
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S2;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S1_TX]: S%d -> S%d\n", prevState, curState);
}

static void S1_RX(Ranging_Table_t *rangingTable) {
    // Invalid reception of rangingMessage
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S1;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S1_RX_NO]: S%d -> S%d\n", prevState, curState);
}

static void S1_RX_NO(Ranging_Table_t *rangingTable) {
    // Invalid reception of rangingMessage
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S1;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S1_RX_NO]: S%d -> S%d\n", prevState, curState);
}

static void S2_TX(Ranging_Table_t *rangingTable) {
    // Don't update Tx here since sending message is an async action, we put all Tx in sendList
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S2;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S2_TX]: S%d -> S%d\n", prevState, curState);
}

static void S2_RX(Ranging_Table_t *rangingTable) {
    // Data in rangingTable shift and fill quickly, directly skipping S3 and entering S4
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S2_RX]: S%d -> S%d\n", prevState, curState);
}

static void S2_RX_NO(Ranging_Table_t *rangingTable) {
    // Lack of timestamp, no need to update rangingTable content
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S2;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S2_RX_NO]: S%d -> S%d\n", prevState, curState);
}

static void S3_TX(Ranging_Table_t *rangingTable) {
    assert(0 && "[S3_TX]: Should not be called\n");
}

static void S3_RX(Ranging_Table_t *rangingTable) {
    assert(0 && "[S3_RX]: Should not be called\n");
}

static void S3_RX_NO(Ranging_Table_t *rangingTable) {
    assert(0 && "[S3_RX_NO]: Should not be called\n");
}

static void S4_TX(Ranging_Table_t *rangingTable) {
    // Don't update Tx here since sending message is an async action, we put all Tx in sendList
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S5;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S4_TX]: S%d -> S%d\n", prevState, curState);
}

static void S4_RX(Ranging_Table_t *rangingTable) {
    // Calculate Tof, but not shift and fill rangingTable
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;
    
    DEBUG_PRINT("[S4_RX]: S%d -> S%d\n", prevState, curState);
}

static void S4_RX_NO(Ranging_Table_t *rangingTable) {
    // Calculate Tof, but not shift and fill rangingTable
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S4_RX_NO]: S%d -> S%d\n", prevState, curState);
}

static void S5_TX(Ranging_Table_t *rangingTable) {
    // Don't update Tx here since sending message is an async action, we put all Tx in sendList
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S5;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S5_TX]: S%d -> S%d\n", prevState, curState);
}

static void S5_RX(Ranging_Table_t *rangingTable) {
    // Calculate Tof, shift and fill the rangingTable, calculation is quick, directly skipping S6 and entering S4
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S5_RX]: S%d -> S%d\n", prevState, curState);
}

static void S5_RX_NO(Ranging_Table_t *rangingTable) {
    // Calculate Tof, but not shift and fill rangingTable
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S5;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S5_RX_NO]: S%d -> S%d\n", prevState, curState);
}

static void S6_TX(Ranging_Table_t *rangingTable) {
    assert(0 && "[S6_TX]: Should not be called\n");
}

static void S6_RX(Ranging_Table_t *rangingTable) {
    assert(0 && "[S6_RX]: Should not be called\n");
}

static void S6_RX_NO(Ranging_Table_t *rangingTable) {
    assert(0 && "[S6_RX_NO]: Should not be called\n");
}

static EventHandlerTable EVENT_HANDLER[RANGING_TABLE_STATE_COUNT][RANGING_TABLE_EVENT_COUNT] = {
    {RESERVED_STUB, RESERVED_STUB, RESERVED_STUB},
    {S1_TX, S1_RX, S1_RX_NO},
    {S2_TX, S2_RX, S2_RX_NO},
    {S3_TX, S3_RX, S3_RX_NO},
    {S4_TX, S4_RX, S4_RX_NO},
    {S5_TX, S5_RX, S5_RX_NO},
    {S6_TX, S6_RX, S6_RX_NO}
};

void RangingTableEventHandler(Ranging_Table_t *rangingTable, RANGING_TABLE_EVENT event) {
    assert((rangingTable->rangingState < RANGING_TABLE_STATE_COUNT) && "Warning: Should not be called\n");
    assert((event < RANGING_TABLE_EVENT_COUNT) && "Warning: Should not be called\n");
    EVENT_HANDLER[rangingTable->rangingState][event](rangingTable);
}
#endif


// -------------------- Generate and Process --------------------
Time_t generateMessage(Ranging_Message_t *rangingMessage) {
    Time_t taskDelay = M2T(RANGING_PERIOD);

    int8_t bodyUnitCount = 0;     // counter for valid bodyunits
    rangingTableSet->localSeqNumber++;
    rangingMessage->header.filter = 0;

    /* generate bodyUnit */
    while(bodyUnitCount < MESSAGE_BODY_UNIT_SIZE && bodyUnitCount < rangingTableSet->counter) {
        Message_Body_Unit_t *bodyUnit = &rangingMessage->bodyUnits[bodyUnitCount];
        if(bodyUnit->Rxtimestamp.timestamp.full != nullTimestampTuple.timestamp.full) {
            index_t index = rangingTableSet->priorityQueue[bodyUnitCount];

            bodyUnit->address = rangingTableSet->rangingTable[index].neighborAddress;
            bodyUnit->Rxtimestamp = rangingTableSet->lastRxtimestamp[index];

            rangingMessage->header.filter |= 1 << (rangingTableSet->rangingTable[index].neighborAddress % 16);
            
            #ifdef STATE_MACHINE_ENABLE
                // Si_TX
                RangingTableEventHandler(&rangingTableSet->rangingTable[index], RANGING_EVENT_TX);
            #endif

            bodyUnitCount++;
        }
    }

    /* generate header */
    rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
    rangingMessage->header.msgSequence = rangingTableSet->localSeqNumber;
    index_t index_Tx = rangingTableSet->sendList.topIndex;
    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        if(rangingTableSet->sendList.Txtimestamps[index_Tx].seqNumber != NULL_SEQ) {
            rangingMessage->header.Txtimestamps[i] = rangingTableSet->sendList.Txtimestamps[index_Tx];
            index_Tx = (index_Tx - 1 + SEND_LIST_SIZE) % SEND_LIST_SIZE;
        }
        else {
            rangingMessage->header.Txtimestamps[i] = nullTimestampTuple;
        }
    }
    #ifdef COORDINATE_SEND_ENABLE
        rangingMessage->header.TxCoordinate = rangingTableSet->sendList.TxCoordinate;
    #endif
    rangingMessage->header.msgLength = sizeof(Message_Header_t) + sizeof(Message_Body_Unit_t) * bodyUnitCount;

    Timestamp_Tuple_t curTimestamp;
    curTimestamp.seqNumber = rangingTableSet->localSeqNumber;
    curTimestamp.timestamp.full = xTaskGetTickCount();

    #ifdef COORDINATE_SEND_ENABLE
        modifyLocation();
        Coordinate_Tuple_t curCoordinate = getCurrentLocation();
        updateSendList(&rangingTableSet->sendList, curTimestamp, curCoordinate);
    #else
        updateSendList(&rangingTableSet->sendList, curTimestamp);
    #endif

    return taskDelay;
}

void processMessage(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo) {
    Ranging_Message_t *rangingMessage = &rangingMessageWithAdditionalInfo->rangingMessage;

    // printRangingMessage(rangingMessage);

    uint16_t neighborAddress = rangingMessage->header.srcAddress;
    index_t neighborIndex = findRangingTable(rangingTableSet, neighborAddress);

    // handle new neighbor
    if(neighborIndex == NULL_INDEX) {
        neighborIndex = registerRangingTable(rangingTableSet, neighborAddress);
        if(neighborIndex == NULL_INDEX) {
            DEBUG_PRINT("Warning: Failed to register new neighbor.\n");
            return;
        }
    }
    // update priority queue
    else {
        updatePriority(rangingTableSet, neighborAddress);
    }

    Ranging_Table_t *rangingTable = &rangingTableSet->rangingTable[neighborIndex];

    Timestamp_Tuple_t Re;
    Re.timestamp = rangingMessageWithAdditionalInfo->timestamp;
    Re.seqNumber = rangingMessage->header.msgSequence;
    
    #ifdef COORDINATE_SEND_ENABLE
        Coordinate_Tuple_t TxCoordinate = rangingMessage->header.TxCoordinate;
        Coordinate_Tuple_t RxCoordinate = rangingMessageWithAdditionalInfo->RxCoordinate;
        float TrueDx = (RxCoordinate.x - TxCoordinate.x);
        float TrueDy = (RxCoordinate.y - TxCoordinate.y);
        float TrueDz = (RxCoordinate.z - TxCoordinate.z);
        float TrueD = sqrtf(TrueDx * TrueDx + TrueDy * TrueDy + TrueDz * TrueDz) / 1000;
    #endif

    /* process bodyUnit */
    Timestamp_Tuple_t Rf = nullTimestampTuple;
    if (rangingMessage->header.filter & (1 << (uwbGetAddress() % 16))) {
        uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Message_Header_t)) / sizeof(Message_Body_Unit_t);
        for(int i = 0; i < bodyUnitCount; i++) {
            if(rangingMessage->bodyUnits[i].address == uwbGetAddress()) {
                Rf = rangingMessage->bodyUnits[i].Rxtimestamp;
                break;
            }
        }
    }

    Timestamp_Tuple_t Tf = nullTimestampTuple;
    index_t index_Tn = NULL_INDEX;
    index_Tn = findSendList(&rangingTableSet->sendList, Rf.seqNumber);
    if(index_Tn != NULL_INDEX) {
        Tf = rangingTableSet->sendList.Txtimestamps[index_Tn];
    }

    /* process header */
    // backupTr and backupRr are used for dealing with order problem
    Timestamp_Tuple_t Rr = rangingTableSet->lastRxtimestamp[neighborIndex];
    /*  Rr              - calculate normalfully -> update
        lastRxtimestamp - receive message        -> update
    */
    Timestamp_Tuple_t backupRr = (rangingTable->Rr.seqNumber == rangingTableSet->lastRxtimestamp[neighborIndex].seqNumber) ? nullTimestampTuple : rangingTable->Rr;

    rangingTableSet->lastRxtimestamp[neighborIndex] = Re;

    Timestamp_Tuple_t Tr = nullTimestampTuple;
    Timestamp_Tuple_t backupTr = nullTimestampTuple;

    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        if(rangingMessage->header.Txtimestamps[i].seqNumber == Rr.seqNumber) {
            Tr = rangingMessage->header.Txtimestamps[i];
        }
        if(backupRr.seqNumber != NULL_SEQ && rangingMessage->header.Txtimestamps[i].seqNumber == backupRr.seqNumber) {
            backupTr = rangingMessage->header.Txtimestamps[i];
        }
    }

    /* initialize: classic protocol
        +------+------+------+------+------+------+
        | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |
        +------+------+------+------+------+------+------+
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |
        +------+------+------+------+------+------+------+
    */
    if(rangingTable->initCalculateRound < INIT_CALCULATION_ROUNDS) {
        printAssistedCalculateTuple(rangingTable->Tp, rangingTable->Rp, Tr, Rr, Tf, Rf);

        float initPTof = assistedCalculatePTof(rangingTable, Tr, Rr, Tf, Rf);

        // normal
        if(initPTof != NULL_TOF && initPTof != MISORDER_SIGN && initPTof != INCOMPLETE_SIGN) {
            #ifdef STATE_MACHINE_ENABLE
                // S5_RX
                RangingTableEventHandler(rangingTable, RANGING_EVENT_RX);
            #endif

            rangingTable->initCalculateRound++;
            shiftRangingTable(rangingTable);
            fillRangingTable(rangingTable, Tr, Rr, Tf, Rf, Re, initPTof);

            if(rangingTable->initCalculateRound == INIT_CALCULATION_ROUNDS) {
                DEBUG_PRINT("[initCalculatePTof]: finish calling\n");
            }
        }

        // problem of orderliness
        else if(initPTof == MISORDER_SIGN) {
            #ifdef STATE_MACHINE_ENABLE
                // S5_RX
                RangingTableEventHandler(rangingTable, RANGING_EVENT_RX);
            #endif

            // use backup       -->     shift and fill
            if(backupTr.seqNumber != NULL_SEQ && backupRr.seqNumber != NULL_SEQ && COMPARE_TIME(Tf, Rr)) {
                /* type_2: Tp, Rp, backupTr, backupRr, Tf, Rf
                    Tb           Rp     Tr           Rf
                        
                       Rb     Tp           Tf     Rr
                */
                initPTof = assistedCalculatePTof(rangingTable, backupTr, backupRr, Tf, Rf);
                if(initPTof != NULL_TOF && initPTof != MISORDER_SIGN && initPTof != INCOMPLETE_SIGN) {
                    shiftRangingTable(rangingTable);
                    fillRangingTable(rangingTable, backupTr, backupRr, Tf, Rf, Re, initPTof);
                }
            }
            // use extra node   -->     replace
            if(initPTof == NULL_TOF || initPTof == MISORDER_SIGN || initPTof == INCOMPLETE_SIGN) {
                DEBUG_PRINT("No suitable backupTimestamp found for recalculation\n");
                /* type_1: ETp, ERp, Tr, Rr, Tf, Rf
                    Tb           Tr     Rp           Rf
                    
                       Rb     Tp           Rr     Tf
                */
                if(COMPARE_TIME(Tr, rangingTable->Rp)) {
                    Ranging_Table_t tmpRangingTable;
                    tmpRangingTable.Tp = rangingTable->ETp;
                    tmpRangingTable.Rp = rangingTable->ERp;
                    initPTof = assistedCalculatePTof(&tmpRangingTable, Tr, Rr, Tf, Rf);
                    if(initPTof != NULL_TOF && initPTof != MISORDER_SIGN && initPTof != INCOMPLETE_SIGN) {
                        replaceRangingTable(rangingTable, Tr, Rr, Tf, Rf, Re, initPTof);
                    }
                }
                /* type_2: Tb, Rb, Tp, Rp, Tr, Rr
                    Tb           Rp     Tr           Rf
                        
                       Rb     Tp           Tf     Rr
                */
                else if(COMPARE_TIME(Tf, Rr)) {
                    Ranging_Table_t tmpRangingTable;
                    tmpRangingTable.Tp = rangingTable->Tb;
                    tmpRangingTable.Rp = rangingTable->Rb;
                    tmpRangingTable.PTof = rangingTable->PTof;
                    initPTof = calculatePTof(&tmpRangingTable, rangingTable->Tp, rangingTable->Rp, Tr, Rr, SECOND_CALCULATE);
                    if(initPTof != NULL_TOF && initPTof != MISORDER_SIGN && initPTof != INCOMPLETE_SIGN) {
                        replaceRangingTable(rangingTable, rangingTable->Tb, rangingTable->Rb, Tf, Rf, Re, initPTof);
                    }
               }               
            }
        }

        // problem of completeness
        else if(initPTof == INCOMPLETE_SIGN) {
            // data loss caused by initializing   ---> update
            if(rangingTable->initCalculateRound == 0) {
                DEBUG_PRINT("Date loss caused by initializing --> update rangingTable\n");
                // ensure data completeness
                if(!(Tr.seqNumber == NULL_SEQ || Rr.seqNumber == NULL_SEQ || Tf.seqNumber == NULL_SEQ || Rf.seqNumber == NULL_SEQ)) {
                    #ifdef STATE_MACHINE_ENABLE
                        // S2_RX
                        RangingTableEventHandler(rangingTable, RANGING_EVENT_RX);
                    #endif

                    shiftRangingTable(rangingTable);
                    fillRangingTable(rangingTable, Tr, Rr, Tf, Rf, Re, NULL_TOF);
                }
                else {
                    #ifdef STATE_MACHINE_ENABLE
                        // S2_RX_NO
                        RangingTableEventHandler(rangingTable, RANGING_EVENT_RX_NO);
                    #endif

                    DEBUG_PRINT("Warning: Data is incomplete and the update has failed\n");
                }
            }

            // data loss caused by lossing packet ---> recalculate
            else {
                #ifdef STATE_MACHINE_ENABLE
                    // S5_RX_NO
                    RangingTableEventHandler(rangingTable, RANGING_EVENT_RX_NO);
                #endif

                DEBUG_PRINT("Date loss caused by lossing packet ---> recalculate\n");

                /* Tf and Rf are full  =>  use ETp, ERp, Tb, Rb, Tf, Rf
                +------+------+------+------+------+------+
                | ETb  | ERp  |  Tb  |  Rp  |      |  Rf  |
                +------+------+------+------+------+------+------+
                | ERb  | ETp  |  Rb  |  Tp  |      |  Tf  |  Re  |
                +------+------+------+------+------+------+------+
                */
                if(Tf.seqNumber != NULL_SEQ && Rf.seqNumber != NULL_SEQ) {
                    Ranging_Table_t tmpRangingTable;
                    tmpRangingTable.Tp = rangingTable->ETp;
                    tmpRangingTable.Rp = rangingTable->ERp;
                    initPTof = assistedCalculatePTof(&tmpRangingTable, rangingTable->Tb, rangingTable->Rb, Tf, Rf);
                }

                /* Tr and Rr are full  =>  use Tb, Rb, Tp, Rp, Tr, Rr
                +------+------+------+------+------+------+
                | ETb  | ERp  |  Tb  |  Rp  |  Tr  |      |
                +------+------+------+------+------+------+------+
                | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |  Re  |
                +------+------+------+------+------+------+------+
                */
                else if(Tr.seqNumber != NULL_SEQ && Rr.seqNumber != NULL_SEQ) {
                    Ranging_Table_t tmpRangingTable;
                    tmpRangingTable.Tp = rangingTable->Tb;
                    tmpRangingTable.Rp = rangingTable->Rb;
                    initPTof = assistedCalculatePTof(&tmpRangingTable, rangingTable->Tp, rangingTable->Rp, Tr, Rr);
                }
            }
        }
        
        // recalculation failed, use the Tof calculated last time
        if(initPTof == NULL_TOF || initPTof == MISORDER_SIGN || initPTof == INCOMPLETE_SIGN) {
            DEBUG_PRINT("Warning: recalculation failed\n");
            initPTof = rangingTable->PTof;
        }

        // print result
        if(initPTof != NULL_TOF && initPTof != MISORDER_SIGN && initPTof != INCOMPLETE_SIGN) {
            float initD = (initPTof * VELOCITY) / 2;
            DEBUG_PRINT("[init_%u]: ModifiedD = %f", MY_UWB_ADDRESS, initD);
            #ifdef COORDINATE_SEND_ENABLE
                DEBUG_PRINT(", TrueD = %f", TrueD);
            #endif
            DEBUG_PRINT(", time = %llu\n", Re.timestamp.full);
        }
        else {
            DEBUG_PRINT("[initCalculatePTof]: assistedCalculatePTof failed\n");
        }
    }

    /* calculate: modified protocol
        +------+------+------+------+------+------+
        | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |
        +------+------+------+------+------+------+------+
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |
        +------+------+------+------+------+------+------+
    */
    // modified protocol
    else {
        printCalculateTuple(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, Tr, Rr, Tf, Rf);

        float ModifiedPTof = calculatePTof(rangingTable, Tr, Rr, Tf, Rf, FIRST_CALCULATE);

        // normal
        if(ModifiedPTof != NULL_TOF && ModifiedPTof != MISORDER_SIGN && ModifiedPTof != INCOMPLETE_SIGN) {
            #ifdef STATE_MACHINE_ENABLE
                // S5_RX
                RangingTableEventHandler(rangingTable, RANGING_EVENT_RX);
            #endif

            shiftRangingTable(rangingTable);
            fillRangingTable(rangingTable, Tr, Rr, Tf, Rf, Re, ModifiedPTof);
        }

        // problem of orderliness
        else if(ModifiedPTof == MISORDER_SIGN) {
            #ifdef STATE_MACHINE_ENABLE
                // S5_RX
                RangingTableEventHandler(rangingTable, RANGING_EVENT_RX);
            #endif

            // use backup       -->     shift and fill
            if(backupTr.seqNumber != NULL_SEQ && backupRr.seqNumber != NULL_SEQ && COMPARE_TIME(Tf, Rr)) {
                /* type_2: Tb, Rb, Tp, Rp, backupTr, backupRr, Tf, Rf
                    Tb           Rp     Tr           Rf
                        
                       Rb     Tp           Tf     Rr
                */
                ModifiedPTof = calculatePTof(rangingTable, backupTr, backupRr, Tf, Rf, FIRST_CALCULATE);
                if(ModifiedPTof != NULL_TOF && ModifiedPTof != MISORDER_SIGN && ModifiedPTof != INCOMPLETE_SIGN) {
                    shiftRangingTable(rangingTable);
                    fillRangingTable(rangingTable, backupTr, backupRr, Tf, Rf, Re, ModifiedPTof);
                }
            }
            // use extra node   -->     replace
            if(ModifiedPTof == NULL_TOF || ModifiedPTof == MISORDER_SIGN || ModifiedPTof == INCOMPLETE_SIGN) {
                DEBUG_PRINT("No suitable backupTimestamp found for recalculation\n");
                /* type_1: ETp, ERp, Tr, Rr, Tf, Rf
                    Tb           Tr     Rp           Rf
                    
                       Rb     Tp           Rr     Tf
                */
                if(COMPARE_TIME(Tr, rangingTable->Rp)) {
                    Ranging_Table_t tmpRangingTable;
                    tmpRangingTable.Tb = rangingTable->ETp;
                    tmpRangingTable.Rb = rangingTable->ERp;
                    tmpRangingTable.Tp = Tr;
                    tmpRangingTable.Rp = Rr;
                    tmpRangingTable.PTof = rangingTable->PTof;
                    ModifiedPTof = calculatePTof(&tmpRangingTable, Tf, Rf, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
                    if(ModifiedPTof != NULL_TOF && ModifiedPTof != MISORDER_SIGN && ModifiedPTof != INCOMPLETE_SIGN) {
                        replaceRangingTable(rangingTable, Tr, Rr, Tf, Rf, Re,  ModifiedPTof);
                    }
                }
                /* type_2: Tb, Rb, Tp, Rp, Tr, Rr
                    Tb           Rp     Tr           Rf
                        
                       Rb     Tp           Tf     Rr
                */
               else if(COMPARE_TIME(Tf, Rr)) {
                    Ranging_Table_t tmpRangingTable;
                    tmpRangingTable.Tb = rangingTable->Tb;
                    tmpRangingTable.Rb = rangingTable->Rb;
                    tmpRangingTable.Tp = rangingTable->Tp;
                    tmpRangingTable.Rp = rangingTable->Rp;
                    tmpRangingTable.PTof = rangingTable->PTof;
                    ModifiedPTof = calculatePTof(&tmpRangingTable, Tr, Rr, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
                    if(ModifiedPTof != NULL_TOF && ModifiedPTof != MISORDER_SIGN && ModifiedPTof != INCOMPLETE_SIGN) {
                        replaceRangingTable(rangingTable, rangingTable->Tb, rangingTable->Rb, Tf, Rf, Re, ModifiedPTof);
                    }
               }
            }
        }

        // problem of completeness
        else if(ModifiedPTof == INCOMPLETE_SIGN) {
            #ifdef STATE_MACHINE_ENABLE
                // S5_RX_NO
                RangingTableEventHandler(rangingTable, RANGING_EVENT_RX_NO);
            #endif

            // data loss caused by lossing packet ---> recalculate
            DEBUG_PRINT("Date loss caused by lossing packet ---> recalculate\n");

            /* Tf and Rf are full  =>  usingETp, ERp, Tb, Rb, Tf, Rf
            +------+------+------+------+------+------+
            | ETb  | ERp  |  Tb  |  Rp  |      |  Rf  |
            +------+------+------+------+------+------+------+
            | ERb  | ETp  |  Rb  |  Tp  |      |  Tf  |  Re  |
            +------+------+------+------+------+------+------+
            */
            if(Tf.seqNumber != NULL_SEQ && Rf.seqNumber != NULL_SEQ) {
                Ranging_Table_t tmpRangingTable;
                tmpRangingTable.Tb = rangingTable->ETp;
                tmpRangingTable.Rb = rangingTable->ERp;
                tmpRangingTable.Tp = rangingTable->Tb;
                tmpRangingTable.Rp = rangingTable->Rb;
                tmpRangingTable.PTof = (rangingTable->PTof + rangingTable->EPTof) / 2;
                ModifiedPTof = calculatePTof(&tmpRangingTable, Tf, Rf, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
            }

            /* Tr and Rr are full  =>  using Tb, Rb, Tp, Rp, Tr, Rr
            +------+------+------+------+------+------+
            | ETb  | ERp  |  Tb  |  Rp  |  Tr  |      |
            +------+------+------+------+------+------+------+
            | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |  Re  |
            +------+------+------+------+------+------+------+
            */
            else if(Tr.seqNumber != NULL_SEQ && Rr.seqNumber != NULL_SEQ) {
                Ranging_Table_t tmpRangingTable;
                tmpRangingTable.Tb = rangingTable->Tb;
                tmpRangingTable.Rb = rangingTable->Rb;
                tmpRangingTable.Tp = rangingTable->Tp;
                tmpRangingTable.Rp = rangingTable->Rp;
                tmpRangingTable.PTof = rangingTable->PTof;
                ModifiedPTof = calculatePTof(&tmpRangingTable, Tr, Rr, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
            }
        }

        // recalculation failed, use the Tof calculated last time
        if(ModifiedPTof == NULL_TOF || ModifiedPTof == MISORDER_SIGN || ModifiedPTof == INCOMPLETE_SIGN) {
            DEBUG_PRINT("Warning: recalculation failed\n");
            ModifiedPTof = rangingTable->PTof;
        }

        // print result
        if(ModifiedPTof != NULL_TOF && ModifiedPTof != MISORDER_SIGN && ModifiedPTof != INCOMPLETE_SIGN) {
            float ModifiedD = (ModifiedPTof * VELOCITY) / 2;

            #ifdef COMPENSATE_ENABLE
                if(lastD == 0) {
                    // initialize lastD
                    lastD = ModifiedD;      
                    DEBUG_PRINT("[current_%u]: ModifiedD = %f", MY_UWB_ADDRESS, ModifiedD);
                }
                else {
                    float CompensateD = (ModifiedD - lastD) * COMPENSATE_RATE;
                    lastD = ModifiedD;
                    DEBUG_PRINT("[current_%u]: ModifiedD = %f", MY_UWB_ADDRESS, ModifiedD + CompensateD);
                }
            #else
                DEBUG_PRINT("[current_%u]: ModifiedD = %f", MY_UWB_ADDRESS, ModifiedD);
            #endif

            #ifdef COORDINATE_SEND_ENABLE
                DEBUG_PRINT(", TrueD = %f", TrueD);
            #endif
            DEBUG_PRINT(", time = %llu\n", Re.timestamp.full);
        }
        else {
            DEBUG_PRINT("[CalculatePTof]: CalculatePTof failed\n");
        }
    }

    printRangingTableSet(rangingTableSet);

    rangingTableSet->rangingTable[neighborIndex].expirationSign = false;

    if(rangingTableSet->localSeqNumber % CHECK_PERIOD == 0) {
        checkExpiration(rangingTableSet);
    }
}


// -------------------- Call back --------------------
/*
static void uwbRangingTxTask(void *parameters) {
    systemWaitStart();

    UWB_Packet_t txPacketCache;
    txPacketCache.header.srcAddress = uwbGetAddress();
    txPacketCache.header.destAddress = UWB_DEST_ANY;
    txPacketCache.header.type = UWB_RANGING_MESSAGE;
    txPacketCache.header.length = 0;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)&txPacketCache.payload;

    while (true) {
        xSemaphoreTake(rangingTableSet->mutex, portMAX_DELAY);

        DEBUG_PRINT("[uwbRangingTxTask] Acquired mutex, generating ranging message\n");
        Time_t taskDelay = RANGING_PERIOD;

        generateMessage(rangingMessage);
        
        txPacketCache.header.seqNumber++;
        txPacketCache.header.length = sizeof(UWB_Packet_Header_t) + rangingMessage->header.msgLength;
        
        uwbSendPacketBlock(&txPacketCache);

        xSemaphoreGive(rangingTableSet->mutex);

        vTaskDelay(taskDelay);
    }
}

static void uwbRangingRxTask(void *parameters) {
    systemWaitStart();

    Ranging_Message_With_Additional_Info_t rxPacketCache;

    while (true) {
        if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
            xSemaphoreTake(rangingTableSet->mutex, portMAX_DELAY);

            processRangingMessage(&rxPacketCache);

            xSemaphoreGive(rangingTableSet->mutex);
        }
        vTaskDelay(M2T(1));
    }
}

void rangingTxCallback(void *parameters) {
    UWB_Packet_t *packet = (UWB_Packet_t*)parameters;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)packet->payload;

    dwTime_t txTime;
    dwt_readtxtimestamp((uint8_t*)&txTime.raw);

    Timestamp_Tuple_t timestamp = {.timestamp = txTime, .seqNumber = rangingMessage->header.msgSequence};
    updateSendList(&rangingTableSet->sendList, timestamp);
}

void rangingRxCallback(void *parameters) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    UWB_Packet_t *packet = (UWB_Packet_t*)parameters;

    dwTime_t rxTime;
    dwt_readrxtimestamp((uint8_t*)&rxTime.raw);

    Ranging_Message_With_Additional_Info_t rxMessageWithTimestamp;
    rxMessageWithTimestamp.timestamp = rxTime;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)packet->payload;
    rxMessageWithTimestamp.rangingMessage = *rangingMessage;

    xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
}

void rangingInit() {
    rangingTableSetInit(&rangingTableSet);

    srand(MY_UWB_ADDRESS);
    
    rxQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
    
    listener.type = UWB_RANGING_MESSAGE;
    listener.rxQueue = NULL;
    listener.rxCb = rangingRxCallback;
    listener.txCb = rangingTxCallback;
    uwbRegisterListener(&listener);

    xTaskCreate(uwbRangingTxTask, ADHOC_UWB_RANGING_TX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
                ADHOC_UWB_TASK_PRI, &uwbRangingTxTaskHandle);
    xTaskCreate(uwbRangingRxTask, ADHOC_UWB_RANGING_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
                ADHOC_UWB_TASK_PRI, &uwbRangingRxTaskHandle);
}
*/