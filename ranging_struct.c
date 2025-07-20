#include "ranging_defconfig.h"
#include "ranging_struct.h"


// comparison of valid parts of timestamps considering cyclic overflow(time_a < time_b)
bool COMPARE_TIME(uint64_t time_a, uint64_t time_b) {
    uint64_t diff = (time_b - time_a) & (UWB_MAX_TIMESTAMP - 1);
    return diff < (UWB_MAX_TIMESTAMP - diff);
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

void rangingTableTr_Rr_BufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer) {
    rangingTableBuffer->topIndex = NULL_INDEX;
    for (table_index_t i = 0; i < Tr_Rr_BUFFER_POOL_SIZE; i++) {
        rangingTableBuffer->candidates[i].Tr = nullTimestampTuple;
        rangingTableBuffer->candidates[i].Rr = nullTimestampTuple;
    }
}

void updateRangingTableTr_Buffer(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tr) {
    rangingTableBuffer->candidates[rangingTableBuffer->topIndex].Tr = Tr;
}

void updateRangingTableRr_Buffer(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Rr) {
    rangingTableBuffer->topIndex = (rangingTableBuffer->topIndex + 1) % Tr_Rr_BUFFER_POOL_SIZE;
    rangingTableBuffer->candidates[rangingTableBuffer->topIndex].Tr = nullTimestampTuple;
    rangingTableBuffer->candidates[rangingTableBuffer->topIndex].Rr = Rr;
}

Ranging_Table_Tr_Rr_Candidate_t rangingTableTr_Rr_BufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tf) {
    index_t index = rangingTableBuffer->topIndex;

    uint64_t leftBound = Rp.timestamp.full % UWB_MAX_TIMESTAMP;
    uint64_t rightBound = Tf.timestamp.full % UWB_MAX_TIMESTAMP;

    Ranging_Table_Tr_Rr_Candidate_t candidate = nullCandidate;

    for (int count = 0; count < Tr_Rr_BUFFER_POOL_SIZE; count++) {
        if (rangingTableBuffer->candidates[index].Tr.timestamp.full 
            && rangingTableBuffer->candidates[index].Rr.timestamp.full 
            && COMPARE_TIME(leftBound, rangingTableBuffer->candidates[index].Tr.timestamp.full)
            && COMPARE_TIME(rangingTableBuffer->candidates[index].Rr.timestamp.full, rightBound)) {      
            candidate.Tr = rangingTableBuffer->candidates[index].Tr;
            candidate.Rr = rangingTableBuffer->candidates[index].Rr;
            break;
        }
        index = (index - 1 + Tr_Rr_BUFFER_POOL_SIZE) % Tr_Rr_BUFFER_POOL_SIZE;
    }

    return candidate;
}

void rangingTableInit(Ranging_Table_t *rangingTable) {
    rangingTable->neighborAddress = NULL_ADDR;
    rangingTable->ETb = nullTimestampTuple;
    rangingTable->ERb = nullTimestampTuple;
    rangingTable->ETp = nullTimestampTuple;
    rangingTable->ERp = nullTimestampTuple;
    rangingTable->Tb = nullTimestampTuple;
    rangingTable->Rb = nullTimestampTuple;
    rangingTable->Tp = nullTimestampTuple;
    rangingTable->Rp = nullTimestampTuple;
    rangingTableTr_Rr_BufferInit(&rangingTable->TrRrBuffer);
    rangingTable->Tf = nullTimestampTuple;
    rangingTable->Rf = nullTimestampTuple;
    rangingTable->Re = nullTimestampTuple;

    rangingTable->PTof = NULL_TOF;
    rangingTable->EPTof = NULL_TOF;
    rangingTable->continuitySign = false;
    rangingTable->expirationSign = true;
    rangingTable->tableState = UNUSED;
    #ifdef STATE_MACHINE_ENABLE
        rangingTable->rangingState = RANGING_STATE_RESERVED;
    #endif
}

// register a new rangingTable and return the index of rangingTable
table_index_t registerRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address) {
    if (rangingTableSet->size >= RANGING_TABLE_SIZE) {
        DEBUG_PRINT("Ranging table Set is full, cannot register new table\n");
        return NULL_INDEX;
    }

    for(table_index_t index = 0; index < RANGING_TABLE_SIZE; index++) {
        if(rangingTableSet->rangingTable[index].tableState == UNUSED) {
            rangingTableSet->rangingTable[index].neighborAddress = address;
            rangingTableSet->rangingTable[index].tableState = USING;
            #ifdef STATE_MACHINE_ENABLE
                rangingTableSet->rangingTable[index].rangingState = RANGING_STATE_S1;
            #endif
            // newly registered neighbor is in the initialization phase and cannot trigger calculations in the short term, so their priority is low
            rangingTableSet->priorityQueue[rangingTableSet->size] = index;
            rangingTableSet->size++;

            DEBUG_PRINT("Registered new ranging table entry: Address = %u\n", address);
            return index;
        }
    }

    assert(0 && "Warning: Should not be called\n");
    return NULL_INDEX;
}

// deregister a rangingTable by address and update the priority queue
void deregisterRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address) {
    if(rangingTableSet->size == 0) {
        DEBUG_PRINT("Ranging table Set is empty, cannot deregister table\n");
        return;
    }

    table_index_t tableIdx = NULL_INDEX;
    for(table_index_t i = 0; i < RANGING_TABLE_SIZE; i++) {
        if(rangingTableSet->rangingTable[i].neighborAddress == address) {
            tableIdx = i;
            break;
        }
    }

    if(tableIdx == NULL_INDEX) {
        assert(0 && "Warning: Should not be called\n");
        return;
    }

    rangingTableInit(rangingTableSet->rangingTable);

    table_index_t queueIdx = NULL_INDEX;
    for(table_index_t i = 0; i < rangingTableSet->size; i++) {
        if(rangingTableSet->priorityQueue[i] == tableIdx) {
            queueIdx = i;
            break;
        }
    }

    if(queueIdx == NULL_INDEX) {
        assert(0 && "Warning: Should not be called\n");
        return;
    }

    for(table_index_t i = queueIdx; i < rangingTableSet->size - 1; i++) {
        rangingTableSet->priorityQueue[i] = rangingTableSet->priorityQueue[i+1];
    }
    rangingTableSet->priorityQueue[rangingTableSet->size - 1] = NULL_INDEX;
    rangingTableSet->size--;

    DEBUG_PRINT("Deregister ranging table entry: Address = %u\n", address);
}

// find the index of rangingTable by address
table_index_t findRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address) {
    if(rangingTableSet->size == 0) {
        // DEBUG_PRINT("Ranging table Set is empty, cannot find table\n");
        return NULL_INDEX;
    }

    for (table_index_t i = 0; i < rangingTableSet->size; i++) {
        table_index_t idx = rangingTableSet->priorityQueue[i];
        if (rangingTableSet->rangingTable[idx].neighborAddress == address) {
            return idx;
        }
    }
    // DEBUG_PRINT("Ranging table Set does not contain the address: %u\n", address);
    return NULL_INDEX;
}

/* fill Ranging Table
    +------+------+------+------+------+------+                   +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |      |                   | ETb  | ERp  |  Tb  |  Rp  |  Tr  | [Rf] |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] | [Re] |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    |    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |
    +------+------+------+------+                                 +------+------+------+------+
*/
void fillRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf, Timestamp_Tuple_t Re) {
    rangingTable->Tf = Tf;
    rangingTable->Rf = Rf;
    rangingTable->Re = Re;
}

/* shift Ranging Table
    +------+------+------+------+------+------+                   +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |                   |[ETb] |[ERp] | [Tb] | [Rp] |  Tr  |      |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |    ===>    |[ERb] |[ETp] | [Rb] | [Tp] |  Rr  |      |      |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    |    EPTof    |    PTof     |                                 |   [EPTof]   |   [PTof]    |
    +------+------+------+------+                                 +------+------+------+------+
*/
void shiftRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, float PTof) {
    rangingTable->ETb = rangingTable->Tb;
    rangingTable->ERb = rangingTable->Rb;
    rangingTable->ETp = rangingTable->Tp;
    rangingTable->ERp = rangingTable->Rp;
    rangingTable->EPTof = rangingTable->PTof;

    rangingTable->Tb = Tr;
    rangingTable->Rb = Rr;
    rangingTable->Tp = rangingTable->Tf;
    rangingTable->Rp = rangingTable->Rf;
    rangingTable->PTof = PTof;

    rangingTable->Tf = nullTimestampTuple;
    rangingTable->Rf = nullTimestampTuple;
    rangingTable->Re = nullTimestampTuple;

    // whether the table is contiguous
    if(rangingTable->ERb.seqNumber + 1 == rangingTable->Rb.seqNumber && rangingTable->ERp.seqNumber + 1 == rangingTable->Rp.seqNumber) {
        if(rangingTable->continuitySign == false) {
            rangingTable->continuitySign = true;
            // correcting PTof
            float correctPTof = classicCalculatePTof(rangingTable->ETp, rangingTable->ERp, rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp);
            rangingTable->PTof = correctPTof;
            DEBUG_PRINT("[correct PTof]: correct PTof = %f\n", correctPTof);
        }
    }
    else {
        rangingTable->continuitySign = false;
    }
}

/* replace Ranging Table
    +------+------+------+------+------+------+                   +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |                   | ETb  | ERp  | [Tb] | [Rp] |  Tr  |      |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |    ===>    | ERb  | ETp  | [Rb] | [Tp] |  Rr  |      |      |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    |    EPTof    |    PTof     |                                 |    EPTof    |   [PTof]    |
    +------+------+------+------+                                 +------+------+------+------+
*/
void replaceRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, float PTof) {
    rangingTable->Tb = Tb;
    rangingTable->Rb = Rb;
    rangingTable->Tp = Tp;
    rangingTable->Rp = Rp;
    rangingTable->PTof = PTof;

    rangingTable->Tf = nullTimestampTuple;
    rangingTable->Rf = nullTimestampTuple;
    rangingTable->Re = nullTimestampTuple;

    rangingTable->continuitySign = false;
}

// update the circular priority queue
void updatePriorityQueue(Ranging_Table_Set_t *rangingTableSet, int8_t shiftCount) {
    if(rangingTableSet->size <= 1 || shiftCount <= 0) {
        return;
    }

    index_t *tmpQueue = malloc(sizeof(index_t) * shiftCount);

    if(!tmpQueue) {
        assert(0 && "Warning: Should not be called\n");
    }

    for(int i = 0; i < shiftCount; i++) {
        tmpQueue[i] = rangingTableSet->priorityQueue[i];
    }

    int8_t idx = 0;

    for(; idx < rangingTableSet->size; idx++) {
        if(idx + shiftCount < rangingTableSet->size) {
            rangingTableSet->priorityQueue[idx] = rangingTableSet->priorityQueue[idx + shiftCount];
        }
        else {
            rangingTableSet->priorityQueue[idx] = tmpQueue[idx + shiftCount - rangingTableSet->size];
        }
    }

    free(tmpQueue);
}

/* ranging algorithm
    T1      <--Ra-->      R2    <--Da-->    T3

       R1   <--Db-->    T2      <--Rb-->       R3
*/
float rangingAlgorithm(Timestamp_Tuple_t T1, Timestamp_Tuple_t R1, Timestamp_Tuple_t T2, Timestamp_Tuple_t R2, Timestamp_Tuple_t T3, Timestamp_Tuple_t R3, float Tof12) {
    uint64_t Ra = (R2.timestamp.full - T1.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    uint64_t Db = (T2.timestamp.full - R1.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    uint64_t Da = (T3.timestamp.full - R2.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    uint64_t Rb = (R3.timestamp.full - T2.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    uint64_t diffA = Ra - Da;
    uint64_t diffB = Rb - Db;

    float Ra_Da = Ra / (float)Da;
    float Rb_Db = Rb / (float)Db;

    float Tof23 = NULL_TOF;

    // meet the convergence condition
    if(Ra_Da < CONVERGENCE_THRESHOLD || Rb_Db < CONVERGENCE_THRESHOLD) {
        if(Ra_Da < Rb_Db) {
            Tof23 = (float)((diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / 2.0f - Ra * Tof12) / (float)Da;
        }
        else {
            Tof23 = (float)((diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / 2.0f - Rb * Tof12) / (float)Db;
        }

        // abnormal result
        float D = (Tof23 * VELOCITY) / 2;
        if(D < LOWER_BOUND_DISTANCE || D > UPPER_BOUND_DISTANCE) {
            return NULL_TOF;
        }
    }

    return Tof23;
}

/* timestamps for classicCalculatePTof
       Rp   <--Db-->    Tr      <--Rb-->       Rf

    Tp      <--Ra-->      Rr    <--Da-->    Tf
*/ 
float classicCalculatePTof(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
    float curPTof = NULL_TOF;
    
    // check completeness
    if(Tp.seqNumber == NULL_SEQ || Rp.seqNumber == NULL_SEQ || Tr.seqNumber == NULL_SEQ
        || Rr.seqNumber == NULL_SEQ || Tf.seqNumber == NULL_SEQ || Rf.seqNumber == NULL_SEQ) {
        DEBUG_PRINT("Warning: Data calculation is not complete\n");
        return NULL_TOF;
    }

    int64_t Ra = (Rr.timestamp.full - Tp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da = (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb = (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db = (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    int64_t diffA = Ra - Da;
    int64_t diffB = Rb - Db;

    curPTof = (diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / (float)(Ra + Db + Rb + Da);

    return curPTof;
}

/* calculate the Time of Flight (Tof) based on the timestamps in the ranging table
    +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |
    +------+------+------+------+------+------+------+
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |
    +------+------+------+------+------+------+------+
    |    EPTof    |    PTof     |
    +------+------+------+------+
*/
float calculatePTof(Ranging_Table_t *rangingTable, Ranging_Table_Tr_Rr_Candidate_t candidate) {
    Timestamp_Tuple_t Tr = candidate.Tr;
    Timestamp_Tuple_t Rr = candidate.Rr;
    Timestamp_Tuple_t Tf = rangingTable->Tf;
    Timestamp_Tuple_t Rf = rangingTable->Rf;

    float tmpPTof = NULL_TOF;
    float curPTof = NULL_TOF;
    
    // check completeness
    if(Tr.seqNumber == NULL_SEQ || Rr.seqNumber == NULL_SEQ || Tf.seqNumber == NULL_SEQ || Rf.seqNumber == NULL_SEQ) {
        DEBUG_PRINT("Warning: Data calculation is not complete\n");

        /* type1
            +------+------+------+------+           +------+------+------+------+
            |  Tb  |  Rp  |  Tr  |      |           | ERp  |  Tb  |  Rp  |  Tr  |
            +------+------+------+------+   ===>    +------+------+------+------+
            |  Rb  |  Tp  |  Rr  |      |           | ETp  |  Rb  |  Tp  |  Rr  |
            +------+------+------+------+           +------+------+------+------+
        */ 
        if(Tr.seqNumber != NULL_SEQ && Rr.seqNumber != NULL_SEQ) {
            tmpPTof = rangingAlgorithm(rangingTable->ETp, rangingTable->ERp, 
                                             rangingTable->Tb, rangingTable->Rb, 
                                             rangingTable->Tp, rangingTable->Rp, (rangingTable->EPTof + rangingTable->PTof) / 2);

            if(tmpPTof != NULL_TOF) {
                curPTof = rangingAlgorithm(rangingTable->Tb, rangingTable->Rb, 
                                           rangingTable->Tp, rangingTable->Rp,
                                           Tr, Rr, tmpPTof);
                // use tmpPTof as the estimated value
                curPTof = (curPTof == NULL_TOF) ? tmpPTof : curPTof;
            }
            else {
                return NULL_TOF;
            }
        }
        
        /* type2
            +------+------+------+------+           +------+------+------+------+
            |  Tb  |  Rp  |      |  Rf  |           | ETb  | ERp  |  Tb  |  Rp  |
            +------+------+------+------+   ===>    +------+------+------+------+
            |  Rb  |  Tp  |      |  Tf  |           | ERb  | ETp  |  Rb  |  Tp  |
            +------+------+------+------+           +------+------+------+------+
        */
        else if(Tf.seqNumber != NULL_SEQ && Rf.seqNumber != NULL_SEQ) {
            tmpPTof = rangingAlgorithm(rangingTable->ETb, rangingTable->ERb, 
                                             rangingTable->ETp, rangingTable->ERp, 
                                             rangingTable->Tb, rangingTable->Rb, (rangingTable->EPTof + rangingTable->PTof) / 2);

            if(tmpPTof != NULL_TOF) {
                curPTof = rangingAlgorithm(rangingTable->ETp, rangingTable->ERp, 
                                           rangingTable->Tb, rangingTable->Rb,
                                           rangingTable->Tp, rangingTable->Rp, tmpPTof);
                // use tmpPTof as the estimated value
                curPTof = (curPTof == NULL_TOF) ? tmpPTof : curPTof;
            }
            else {
                return NULL_TOF;
            }
        }
        
        /* type3
            +------+------+------+------+      
            |  Tb  |  Rp  |      |      |          
            +------+------+------+------+   ===>    PTof
            |  Rb  |  Tp  |      |      |           
            +------+------+------+------+  
        */
        else {
            return NULL_TOF;
        }
    }

    else {
        /* type4
     
            +------+------+------+------+
            |  Tb  |  Rp  |  Tr  |  Rf  |
            +------+------+------+------+   
            |  Rb  |  Tp  |  Rr  |  Tf  | 
            +------+------+------+------+    
        */ 
        tmpPTof = rangingAlgorithm(rangingTable->Tb, rangingTable->Rb, 
                                         rangingTable->Tp, rangingTable->Rp, 
                                         Tr, Rr, rangingTable->PTof);

        if(tmpPTof != NULL_TOF) {
            curPTof = rangingAlgorithm(rangingTable->Tp, rangingTable->Rp, 
                                       Tr, Rr,
                                       rangingTable->Tf, rangingTable->Rf, tmpPTof);
            // use tmpPTof as the estimated value
            curPTof = (curPTof == NULL_TOF) ? tmpPTof : curPTof;
        }
        else {
            return NULL_TOF;
        }
    }

    return curPTof;
}

// check expirationSign of rangingTables and deregister rangingTable expired
void checkExpiration(Ranging_Table_Set_t *rangingTableSet) {
    for(table_index_t i = rangingTableSet->size; i > 0; i--) {
        table_index_t idx = rangingTableSet->priorityQueue[i-1];
        if(rangingTableSet->rangingTable[idx].expirationSign == true) {
            deregisterRangingTable(rangingTableSet, rangingTableSet->rangingTable[idx].neighborAddress);
        } 
        else {
            rangingTableSet->rangingTable[idx].expirationSign = true;
        }
    }
}

void printRangingMessage(Ranging_Message_t *rangingMessage) {
    DEBUG_PRINT("\n{rangingMessage}\n");

    DEBUG_PRINT("srcAddress: %u\n", rangingMessage->header.srcAddress);
    DEBUG_PRINT("msgSequence: %u\n", rangingMessage->header.msgSequence);

    DEBUG_PRINT("[Tx]\n");
    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        DEBUG_PRINT("seqNumber: %u, timestamp: %llu\n", rangingMessage->header.Txtimestamps[i].seqNumber, rangingMessage->header.Txtimestamps[i].timestamp.full);
    }

    DEBUG_PRINT("[Rx]:\n");
    for(int i = 0; i < MESSAGE_BODY_UNIT_SIZE; i++){
        DEBUG_PRINT("address: %u, seqNumber: %u, timestamp: %llu\n", rangingMessage->bodyUnits[i].address, rangingMessage->bodyUnits[i].Rxtimestamp.seqNumber, rangingMessage->bodyUnits[i].Rxtimestamp.timestamp.full);
    }
    DEBUG_PRINT("\n");
}

void printSendList(SendList_t *sendList) {
    DEBUG_PRINT("\n[sendList]\n");
    index_t index = sendList->topIndex;
    DEBUG_PRINT("location: x = %u, y = %u, z = %u\n", sendList->TxCoordinate.x,  sendList->TxCoordinate.y,  sendList->TxCoordinate.z);
    for(int i = 0; i < SEND_LIST_SIZE; i++) {
        DEBUG_PRINT("seqNumber: %u, timestamp: %llu\n", sendList->Txtimestamps[index].seqNumber, sendList->Txtimestamps[index].timestamp.full);
        index = (index - 1 + SEND_LIST_SIZE) % SEND_LIST_SIZE;
    }
}

void printRangingTable(Ranging_Table_t *rangingTable) {
    DEBUG_PRINT("neighborAddress: %u\n", rangingTable->neighborAddress);
    DEBUG_PRINT("(ETb) seqNumber: %u, timestamp: %llu\n", rangingTable->ETb.seqNumber, rangingTable->ETb.timestamp.full);
    DEBUG_PRINT("(ERb) seqNumber: %u, timestamp: %llu\n", rangingTable->ERb.seqNumber, rangingTable->ERb.timestamp.full);
    DEBUG_PRINT("(ETp) seqNumber: %u, timestamp: %llu\n", rangingTable->ETp.seqNumber, rangingTable->ETp.timestamp.full);
    DEBUG_PRINT("(ERp) seqNumber: %u, timestamp: %llu\n", rangingTable->ERp.seqNumber, rangingTable->ERp.timestamp.full);
    DEBUG_PRINT("(Tb) seqNumber: %u, timestamp: %llu\n", rangingTable->Tb.seqNumber, rangingTable->Tb.timestamp.full);
    DEBUG_PRINT("(Rb) seqNumber: %u, timestamp: %llu\n", rangingTable->Rb.seqNumber, rangingTable->Rb.timestamp.full);
    DEBUG_PRINT("(Tp) seqNumber: %u, timestamp: %llu\n", rangingTable->Tp.seqNumber, rangingTable->Tp.timestamp.full);
    DEBUG_PRINT("(Rp) seqNumber: %u, timestamp: %llu\n", rangingTable->Rp.seqNumber, rangingTable->Rp.timestamp.full);
    DEBUG_PRINT("(Tr_Rr_Buffer)\n");
    index_t index = rangingTable->TrRrBuffer.topIndex;
    for(int i = 0; i < Tr_Rr_BUFFER_POOL_SIZE; i++) {
        DEBUG_PRINT("\tseqNumber: %u, timestamp: %llu  -->  seqNumber: %u, timestamp: %llu\n", 
            rangingTable->TrRrBuffer.candidates[index].Tr.seqNumber, rangingTable->TrRrBuffer.candidates[index].Tr.timestamp.full,
            rangingTable->TrRrBuffer.candidates[index].Rr.seqNumber, rangingTable->TrRrBuffer.candidates[index].Rr.timestamp.full);
        index = (index - 1 + Tr_Rr_BUFFER_POOL_SIZE) % Tr_Rr_BUFFER_POOL_SIZE;
    }
    DEBUG_PRINT("(Tf) seqNumber: %u, timestamp: %llu\n", rangingTable->Tf.seqNumber, rangingTable->Tf.timestamp.full);
    DEBUG_PRINT("(Rf) seqNumber: %u, timestamp: %llu\n", rangingTable->Rf.seqNumber, rangingTable->Rf.timestamp.full);
    DEBUG_PRINT("(Re) seqNumber: %u, timestamp: %llu\n", rangingTable->Re.seqNumber, rangingTable->Re.timestamp.full);
    
    DEBUG_PRINT("PTof = %f, EPTof = %f, continuitySign = %s\n", rangingTable->PTof, rangingTable->EPTof, rangingTable->continuitySign == true ? "true" : "false");
}

void printPriorityQueue(Ranging_Table_Set_t *rangingTableSet) {
    DEBUG_PRINT("\n[priorityQueue]\n");
    for(int i = 0; i < rangingTableSet->size; i++) {
        DEBUG_PRINT("priority: %d -> neighborAddress: %u\n", i + 1, rangingTableSet->rangingTable[rangingTableSet->priorityQueue[i]].neighborAddress);
    }
}

void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet) {
    DEBUG_PRINT("\n{rangingTableSet}\n");
    DEBUG_PRINT("size: %u\n", rangingTableSet->size);
    DEBUG_PRINT("localSeqNumber: %u\n", rangingTableSet->localSeqNumber);

    printPriorityQueue(rangingTableSet);

    printSendList(&rangingTableSet->sendList);

    DEBUG_PRINT("\n[lastRxtimestamp]\n");
    for(int i = 0; i < rangingTableSet->size; i++) {
        DEBUG_PRINT("neighborAddress: %u, seqNumber: %u, timestamp: %llu\n", rangingTableSet->rangingTable[i].neighborAddress, rangingTableSet->lastRxtimestamp[i].seqNumber, rangingTableSet->lastRxtimestamp[i].timestamp.full);
    }

    DEBUG_PRINT("\n[rangingTable]\n");
    for(int i = 0; i < rangingTableSet->size; i++) {
        printRangingTable(&rangingTableSet->rangingTable[i]);
    }
}

void printclassicCalculateTuple(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
    DEBUG_PRINT("\nRp[seq=%u, ts=%llu] <--%llu--> Tr[seq=%u, ts=%llu] <--%llu--> Rf[seq=%u, ts=%llu]\n",
        Rp.seqNumber, Rp.timestamp.full, Tr.timestamp.full - Rp.timestamp.full, Tr.seqNumber, Tr.timestamp.full, Rf.timestamp.full - Tr.timestamp.full, Rf.seqNumber, Rf.timestamp.full);
    DEBUG_PRINT("\nTp[seq=%u, ts=%llu] <--%llu--> Rr[seq=%u, ts=%llu] <--%llu--> Tf[seq=%u, ts=%llu]\n",
        Tp.seqNumber, Tp.timestamp.full, Rr.timestamp.full - Tp.timestamp.full, Rr.seqNumber, Rr.timestamp.full, Tf.timestamp.full - Rr.timestamp.full, Tf.seqNumber, Tf.timestamp.full);
}

void printCalculateTuple(Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
    DEBUG_PRINT("\nTb[seq=%u, ts=%llu] <--%llu--> Rp[seq=%u, ts=%llu] <--%llu--> Tr[seq=%u, ts=%llu] <--%llu--> Rf[seq=%u, ts=%llu]\n",
                Tb.seqNumber,Tb.timestamp.full,
                Rp.timestamp.full -Tb.timestamp.full, Rp.seqNumber, Rp.timestamp.full,
                Tr.timestamp.full - Rp.timestamp.full, Tr.seqNumber, Tr.timestamp.full,
                Rf.timestamp.full - Tr.timestamp.full, Rf.seqNumber, Rf.timestamp.full);
    DEBUG_PRINT("\nRb[seq=%u, ts=%llu] <--%llu--> Tp[seq=%u, ts=%llu] <--%llu--> Rr[seq=%u, ts=%llu] <--%llu--> Tf[seq=%u, ts=%llu]\n",
                Rb.seqNumber,Rb.timestamp.full,
                Tp.timestamp.full -Rb.timestamp.full,Tp.seqNumber,Tp.timestamp.full,
                Rr.timestamp.full -Tp.timestamp.full, Rr.seqNumber, Rr.timestamp.full,
                Tf.timestamp.full - Rr.timestamp.full, Tf.seqNumber, Tf.timestamp.full);
}