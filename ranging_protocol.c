#include <stdlib.h>

#include "ranging_protocol.h"


static uint16_t MY_UWB_ADDRESS;
Ranging_Table_Set_t *rangingTableSet;
static int8_t initCalculateRound = 0;
#ifdef COMPENSATE_ENABLE
static float lastD = 0;
#endif


void rangingTableSetInit() {
    MY_UWB_ADDRESS = uwbGetAddress();

    rangingTableSet = (Ranging_Table_Set_t *)malloc(sizeof(Ranging_Table_Set_t));
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
}

// // search for index of send list whose Tx seqNumber is closest to seqNumber with full info of Tx and Rx
// index_t searchSendList(SendList_t *sendList, uint16_t seqNumber) {
//     if(seqNumber == NULL_SEQ) {
//         return NULL_INDEX;
//     }
//     index_t index = sendList->topIndex;
//     index_t ans = NULL_INDEX;
//     if (index == NULL_INDEX) {
//         DEBUG_PRINT("Send list is empty, cannot search for sequence number: %u\n", seqNumber);
//         return NULL_INDEX;
//     }
//     for(int i = 0; i < SEND_LIST_SIZE; i++) {
//         if(sendList->Txtimestamps[index].seqNumber != NULL_SEQ && sendList->Txtimestamps[index].seqNumber < seqNumber) {
//             if(ans == NULL_INDEX || sendList->Txtimestamps[index].seqNumber > sendList->Txtimestamps[ans].seqNumber) {
//                 ans = index;
//             }
//         }
//         index = (index - 1 + SEND_LIST_SIZE) % SEND_LIST_SIZE;
//     }
//     if (ans == NULL_INDEX) {
//         DEBUG_PRINT("No valid sequence number found in send list for sequence number: %u\n", seqNumber);
//     }
//     return ans;
// }

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

    Timestamp_Tuple_t Rr;
    Rr.timestamp = rangingMessageWithAdditionalInfo->timestamp;
    Rr.seqNumber = rangingMessage->header.msgSequence;
    
    #ifdef COORDINATE_SEND_ENABLE
        Coordinate_Tuple_t TxCoordinate = rangingMessage->header.TxCoordinate;
        Coordinate_Tuple_t RxCoordinate = rangingMessageWithAdditionalInfo->RxCoordinate;
        float TrueDx = (RxCoordinate.x - TxCoordinate.x);
        float TrueDy = (RxCoordinate.y - TxCoordinate.y);
        float TrueDz = (RxCoordinate.z - TxCoordinate.z);
        float TrueD = sqrtf(TrueDx * TrueDx + TrueDy * TrueDy + TrueDz * TrueDz) / 1000;
    #endif

    /* process bodyUnit */
    Timestamp_Tuple_t Rn = nullTimestampTuple;
    if (rangingMessage->header.filter & (1 << (uwbGetAddress() % 16))) {
        uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Message_Header_t)) / sizeof(Message_Body_Unit_t);
        for(int i = 0; i < bodyUnitCount; i++) {
            if(rangingMessage->bodyUnits[i].address == uwbGetAddress()) {
                Rn = rangingMessage->bodyUnits[i].Rxtimestamp;
                break;
            }
        }
    }

    Timestamp_Tuple_t Tn = nullTimestampTuple;
    index_t index_Tn = NULL_INDEX;
    index_Tn = findSendList(&rangingTableSet->sendList, Rn.seqNumber);
    if(index_Tn != NULL_INDEX) {
        Tn = rangingTableSet->sendList.Txtimestamps[index_Tn];
    }

    /* process header */

    // backupTx and backupRx are used for dealing with order problem
    Timestamp_Tuple_t Rx = rangingTableSet->lastRxtimestamp[neighborIndex];
    /*  Rx              - calculate successfully -> update
        lastRxtimestamp - receive message        -> update
    */
    Timestamp_Tuple_t backupRx = (rangingTable->Rx.seqNumber == rangingTableSet->lastRxtimestamp[neighborIndex].seqNumber) ? nullTimestampTuple : rangingTable->Rx;

    rangingTableSet->lastRxtimestamp[neighborIndex] = Rr;

    Timestamp_Tuple_t Tx = nullTimestampTuple;
    Timestamp_Tuple_t backupTx = nullTimestampTuple;

    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        if(rangingMessage->header.Txtimestamps[i].seqNumber == Rx.seqNumber) {
            Tx = rangingMessage->header.Txtimestamps[i];
        }
        if(backupRx.seqNumber != NULL_SEQ && rangingMessage->header.Txtimestamps[i].seqNumber == backupRx.seqNumber) {
            backupTx = rangingMessage->header.Txtimestamps[i];
        }
    }

    /* initialize: classic protocol
        +------+------+------+------+------+------+
        |  T1  |  R2  |  T3  |  R4  |  Tx  |  Rn  |
        +------+------+------+------+------+------+------+
        |  R1  |  T2  |  R3  |  T4  |  Rx  |  Tn  |  Rr  |
        +------+------+------+------+------+------+------+
    */
    if(initCalculateRound < INIT_CALCULATION_ROUNDS) {
        float initPTof = assistedCalculatePTof(rangingTable, Tx, Rx, Tn, Rn);

        // success
        if(initPTof != NULL_TOF && initPTof != MISORDER_SIGN && initPTof != INCOMPLETE_SIGN) {
            initCalculateRound++;
            shiftRangingTable(rangingTable);
            fillRangingTable(rangingTable, Tx, Rx, Tn, Rn, Rr, initPTof);
            if(initCalculateRound == INIT_CALCULATION_ROUNDS) {
                DEBUG_PRINT("[initCalculatePTof]: finish calling\n");
            }
        }

        // problem of orderliness
        else if(initPTof == MISORDER_SIGN) {
            if(backupTx.seqNumber != NULL_SEQ && backupRx.seqNumber != NULL_SEQ) {
                initPTof = assistedCalculatePTof(rangingTable, backupTx, backupRx, Tn, Rn);
            }
            else {
                DEBUG_PRINT("No suitable timestamp found for recalculation\n");
            }
        }

        // problem of completeness
        else if(initPTof == INCOMPLETE_SIGN) {
            // data loss caused by initializing   ---> update
            if(initCalculateRound == 0) {
                DEBUG_PRINT("Date loss caused by initializing --> update rangingTable\n");
                // ensure data completeness
                if(!(Tx.seqNumber == NULL_SEQ || Rx.seqNumber == NULL_SEQ || Tn.seqNumber == NULL_SEQ || Rn.seqNumber == NULL_SEQ)) {
                    shiftRangingTable(rangingTable);
                    fillRangingTable(rangingTable, Tx, Rx, Tn, Rn, Rr, NULL_TOF);
                }
                else {
                    DEBUG_PRINT("Warning: Data is incomplete and the update has failed\n");
                }
            }

            // data loss caused by lossing packet ---> recalculate
            else {
                DEBUG_PRINT("Date loss caused by lossing packet ---> recalculate\n");

                /* Tn and Rn are full  =>  using T2, R2, T3, R3, Tn, Rn
                +------+------+------+------+------+------+
                |  T1  |  R2  |  T3  |  R4  |      |  Rn  |
                +------+------+------+------+------+------+------+
                |  R1  |  T2  |  R3  |  T4  |      |  Tn  |  Rr  |
                +------+------+------+------+------+------+------+
                */
                if(Tn.seqNumber != NULL_SEQ && Rn.seqNumber != NULL_SEQ) {
                    Ranging_Table_t tmpRangingTable;
                    tmpRangingTable.T4 = rangingTable->T2;
                    tmpRangingTable.R4 = rangingTable->R2;
                    initPTof = assistedCalculatePTof(&tmpRangingTable, rangingTable->T3, rangingTable->R3, Tn, Rn);
                }

                /* Tx and Rx are full  =>  using T3, R3, T4, R4, Tx, Rx
                +------+------+------+------+------+------+
                |  T1  |  R2  |  T3  |  R4  |  Tx  |      |
                +------+------+------+------+------+------+------+
                |  R1  |  T2  |  R3  |  T4  |  Rx  |      |  Rr  |
                +------+------+------+------+------+------+------+
                */
                else if(Tx.seqNumber != NULL_SEQ && Rx.seqNumber != NULL_SEQ) {
                    Ranging_Table_t tmpRangingTable;
                    tmpRangingTable.T4 = rangingTable->T3;
                    tmpRangingTable.R4 = rangingTable->R3;
                    initPTof = assistedCalculatePTof(&tmpRangingTable, rangingTable->T4, rangingTable->R4, Tx, Rx);
                }
            }
        }
        
        // recalculation failed, use the Tof calculated last time
        if(initPTof == NULL_TOF || initPTof == MISORDER_SIGN || initPTof == INCOMPLETE_SIGN) {
            initPTof = rangingTable->PTof;
            DEBUG_PRINT("Warning: recalculation failed\n");
        }

        // print result
        if(initPTof != NULL_TOF && initPTof != MISORDER_SIGN && initPTof != INCOMPLETE_SIGN) {
            float initD = (initPTof * VELOCITY) / 2;
            DEBUG_PRINT("[init_%u]: ModifiedD = %f", MY_UWB_ADDRESS, initD);
            #ifdef COORDINATE_SEND_ENABLE
                DEBUG_PRINT(", TrueD = %f", TrueD);
            #endif
            DEBUG_PRINT(", time = %llu\n", Rr.timestamp.full);
        }
        else {
            DEBUG_PRINT("[initCalculatePTof]: assistedCalculatePTof failed\n");
        }
    }

    /* calculate: modified protocol
        +------+------+------+------+------+------+
        |  T1  |  R2  |  T3  |  R4  |  Tx  |  Rn  |
        +------+------+------+------+------+------+------+
        |  R1  |  T2  |  R3  |  T4  |  Rx  |  Tn  |  Rr  |
        +------+------+------+------+------+------+------+
    */
    // modified protocol
    else {
        float ModifiedPTof = calculatePTof(rangingTable, Tx, Rx, Tn, Rn, FIRST_CALCULATE);

        // success
        if(ModifiedPTof != NULL_TOF && ModifiedPTof != MISORDER_SIGN && ModifiedPTof != INCOMPLETE_SIGN) {
            shiftRangingTable(rangingTable);
            fillRangingTable(rangingTable, Tx, Rx, Tn, Rn, Rr, ModifiedPTof);
        }

        // problem of orderliness
        else if(ModifiedPTof == MISORDER_SIGN) {
            if(backupTx.seqNumber != NULL_SEQ && backupRx.seqNumber != NULL_SEQ) {
                ModifiedPTof = calculatePTof(rangingTable, backupTx, backupRx, Tn, Rn, FIRST_CALCULATE);
            }
            else {
                DEBUG_PRINT("No suitable timestamp found for recalculation\n");
                /* type_1
                        Tx     R4           Rn
                    
                     T4           Rx     Tn
                */
                if(Tx.timestamp.full < rangingTable->R4.timestamp.full) {
                    Ranging_Table_t tmpRangingTable;
                    tmpRangingTable.T3 = rangingTable->T2;
                    tmpRangingTable.R3 = rangingTable->R2;
                    tmpRangingTable.T4 = Tx;
                    tmpRangingTable.R4 = Rx;
                    tmpRangingTable.PTof = rangingTable->PTof;
                    ModifiedPTof = calculatePTof(&tmpRangingTable, Tn, Rn, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
                    if(ModifiedPTof != NULL_TOF && ModifiedPTof != MISORDER_SIGN && ModifiedPTof != INCOMPLETE_SIGN) {
                        replaceRangingTable(rangingTable, Tx, Rx, Tn, Rn, ModifiedPTof);
                    }
                }
                /* type_2
                        R4     Tx           Rn
                    
                     T4           Tn     Rx
                */
               else if(Tn.timestamp.full < Rx.timestamp.full) {
                    Ranging_Table_t tmpRangingTable;
                    tmpRangingTable.T3 = rangingTable->T3;
                    tmpRangingTable.R3 = rangingTable->R3;
                    tmpRangingTable.T4 = rangingTable->T4;
                    tmpRangingTable.R4 = rangingTable->R4;
                    tmpRangingTable.PTof = rangingTable->PTof;
                    ModifiedPTof = calculatePTof(&tmpRangingTable, Tx, Rx, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
                    if(ModifiedPTof != NULL_TOF && ModifiedPTof != MISORDER_SIGN && ModifiedPTof != INCOMPLETE_SIGN) {
                        replaceRangingTable(rangingTable, rangingTable->T3, rangingTable->R3, Tn, Rn, ModifiedPTof);
                    }
               }
            }
        }

        // problem of completeness
        else if(ModifiedPTof == INCOMPLETE_SIGN) {
            // data loss caused by lossing packet ---> recalculate
            DEBUG_PRINT("Date loss caused by lossing packet ---> recalculate\n");

            /* Tn and Rn are full  =>  using T2, R2, T3, R3, Tn, Rn
            +------+------+------+------+------+------+
            |  T1  |  R2  |  T3  |  R4  |      |  Rn  |
            +------+------+------+------+------+------+------+
            |  R1  |  T2  |  R3  |  T4  |      |  Tn  |  Rr  |
            +------+------+------+------+------+------+------+
            */
            if(Tn.seqNumber != NULL_SEQ && Rn.seqNumber != NULL_SEQ) {
                Ranging_Table_t tmpRangingTable;
                tmpRangingTable.T3 = rangingTable->T2;
                tmpRangingTable.R3 = rangingTable->R2;
                tmpRangingTable.T4 = rangingTable->T3;
                tmpRangingTable.R4 = rangingTable->R3;
                tmpRangingTable.PTof = (rangingTable->PTof + rangingTable->EPTof) / 2;
                ModifiedPTof = calculatePTof(&tmpRangingTable, Tn, Rn, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
            }
            
            /* Tx and Rx are full  =>  using T3, R3, T4, R4, Tx, Rx
            +------+------+------+------+------+------+
            |  T1  |  R2  |  T3  |  R4  |  Tx  |      |
            +------+------+------+------+------+------+------+
            |  R1  |  T2  |  R3  |  T4  |  Rx  |      |  Rr  |
            +------+------+------+------+------+------+------+
            */
            else if(Tx.seqNumber != NULL_SEQ && Rx.seqNumber != NULL_SEQ) {
                Ranging_Table_t tmpRangingTable;
                tmpRangingTable.T3 = rangingTable->T3;
                tmpRangingTable.R3 = rangingTable->R3;
                tmpRangingTable.T4 = rangingTable->T4;
                tmpRangingTable.R4 = rangingTable->R4;
                tmpRangingTable.PTof = rangingTable->PTof;
                ModifiedPTof = calculatePTof(&tmpRangingTable, Tx, Rx, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
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
            DEBUG_PRINT(", time = %llu\n", Rr.timestamp.full);
        }
        else {
            DEBUG_PRINT("[CalculatePTof]: CalculatePTof failed\n");
        }
    }
    // printRangingTableSet(rangingTableSet);
}