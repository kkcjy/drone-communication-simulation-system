#include "dynamic_swarm_ranging.h"


static uint16_t MY_UWB_ADDRESS;
Ranging_Table_Set_t *rangingTableSet;
float distanceReal;
float distanceCalculate;

#ifdef COMPENSATE_ENABLE
static float lastD = 0;
#endif

// static UWB_Message_Listener_t listener;
// static TaskHandle_t uwbRangingTxTaskHandle = 0;
// static TaskHandle_t uwbRangingRxTaskHandle = 0;


/* -------------------- Ranging Table Set Operation -------------------- */
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

// init rangingTableTr_Rr_Buffer
void rangingTableTr_Rr_BufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer) {
    rangingTableBuffer->topIndex = NULL_INDEX;
    for (table_index_t i = 0; i < Tr_Rr_BUFFER_POOL_SIZE; i++) {
        rangingTableBuffer->candidates[i].Tr = nullTimestampTuple;
        rangingTableBuffer->candidates[i].Rr = nullTimestampTuple;
    }
}

// update Tr in rangingTableTr_Rr_Buffer
void updateRangingTableTr_Buffer(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tr) {
    rangingTableBuffer->candidates[rangingTableBuffer->topIndex].Tr = Tr;
}

// update Rr in rangingTableTr_Rr_Buffer
void updateRangingTableRr_Buffer(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Rr) {
    rangingTableBuffer->topIndex = (rangingTableBuffer->topIndex + 1) % Tr_Rr_BUFFER_POOL_SIZE;
    rangingTableBuffer->candidates[rangingTableBuffer->topIndex].Tr = nullTimestampTuple;
    rangingTableBuffer->candidates[rangingTableBuffer->topIndex].Rr = Rr;
}

// get candidate in rangingTableTr_Rr_Buffer based on Rp and Tf
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

// init rangingTable
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

// init rangingTableSet
void rangingTableSetInit() {
    MY_UWB_ADDRESS = uwbGetAddress();

    rangingTableSet = (Ranging_Table_Set_t*)malloc(sizeof(Ranging_Table_Set_t));
    rangingTableSet->size = 0;
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


/* -------------------- Calculation Function -------------------- */
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


/* -------------------- Print Function -------------------- */
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


/* -------------------- State Machine Operation -------------------- */
#ifdef STATE_MACHINE_ENABLE
static void RESERVED_STUB(Ranging_Table_t *rangingTable) {
    assert(0 && "[RESERVED_STUB]: Should not be called\n");
}

static void S1_TX(Ranging_Table_t *rangingTable) {
    /* Don't update Tx here since sending message is an async action, we put all Tx in sendList
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |      |  S1  |            |      |      |      |      |      |      |  S2  |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
        |      |      |      |      |      |      |      |    ===>    |      |      |      |      |      | [Tf] |      |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
        |             |             |                                 |             |             |
        +------+------+------+------+                                 +------+------+------+------+
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S2;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S1_TX]: S%d -> S%d\n", prevState, curState);
}

static void S1_RX(Ranging_Table_t *rangingTable) {
    /* Invalid reception of rangingMessage
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |      |  S1  |            |      |      |      |      | [Tr] | [Rf] |  S1  |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+   
        |      |      |      |      |      |      |      |            |      |      |      |      |      |      | [Re] |
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+   
        |             |             |                                 |             |             |
        +------+------+------+------+                                 +------+------+------+------+
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S1;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S1_RX_NO]: S%d -> S%d\n", prevState, curState);
}

static void S1_RX_NO(Ranging_Table_t *rangingTable) {
    /* Invalid reception of rangingMessage
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |      |  S1  |            |      |      |      |      | [Tr] |      |  S1  |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+   
        |      |      |      |      |      |      |      |            |      |      |      |      |      |      | [Re] |
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+   
        |             |             |                                 |             |             |
        +------+------+------+------+                                 +------+------+------+------+
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S1;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S1_RX_NO]: S%d -> S%d\n", prevState, curState);
}

static void S2_TX(Ranging_Table_t *rangingTable) {
    /* Don't update Tx here since sending message is an async action, we put all Tx in sendList
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |      |  S2  |            |      |      |      |      |      |      |  S2  |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+   
        |      |      |      |      |      |  Tf  |      |            |      |      |      |      |      | [Tf] |      |
        +------+------+------+------+------+------+------+    <==     +------+------+------+------+------+------+------+   
        |             |             |                                 |             |             |
        +------+------+------+------+                                 +------+------+------+------+
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S2;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S2_TX]: S%d -> S%d\n", prevState, curState);
}

static void S2_RX(Ranging_Table_t *rangingTable) {
    /* Data in rangingTable shift quickly, directly skipping S3 and entering S4
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |      |  S2  |            |      |      |      |      | [Tr] | [Rf] |  S3  |            |      |      |      |  Rp  |      |      | S4.1 |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |  Tf  |      |    ===>    |      |      |      |      |      |  Tf  | [Re] |    ===>    |      |      |      |  Tp  |  Rr  |      |      |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |             |             |                                 |             |             |                                 |             |             | 
        +------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;

    updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
    shiftRangingTable(rangingTable, nullTimestampTuple, nullTimestampTuple, NULL_TOF);

    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S2_RX]: S%d -> S%d\n", prevState, curState);
}

static void S2_RX_NO(Ranging_Table_t *rangingTable) {
    /* Lack of timestamp, no need to shift
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        |      |      |      |      |      |      |  S2  |            |      |      |      |      | [Tr] |      |  S2  |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+           
        |      |      |      |      |      |  Tf  |      |            |      |      |      |      |      |  Tf  | [Re] |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+           
        |             |             |                                 |             |             |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
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
    /* Don't update Tx here since sending message is an async action, we put all Tx in sendList
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        |      |      |      |  Rp  |      |      | S4.1 |            |      |      |      |  Rp  |      |      | S5.1 |          
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        |      |      |      |  Tp  |  Rr  |      |      |    ===>    |      |      |      |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        |             |             |                                 |             |             |                                
        +------+------+------+------+                                 +------+------+------+------+

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |          
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |    ===>    |      | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        |             |    PTof     |                                 |             |    PTof     |                                
        +------+------+------+------+                                 +------+------+------+------+ 

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |          
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        |    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S5;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S4_TX]: S%d -> S%d\n", prevState, curState);
}

static void S4_RX(Ranging_Table_t *rangingTable) {
    /* Invalid reception of rangingMessage
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |  Rp  |      |      | S4.1 |            |      |      |      |  Rp  | [Tr] | [Rf] | S4.1 |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
        |      |      |      |  Tp  |  Rr  |      |      |            |      |      |      |  Tp  |  Rr  |      | [Re] |
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
        |             |             |                                 |             |             |                                
        +------+------+------+------+                                 +------+------+------+------+

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.2 |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
        |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |            |      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
        |             |    PTof     |                                 |             |    PTof     |                                
        +------+------+------+------+                                 +------+------+------+------+ 

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.3 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+         
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+          
        |    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;
    
    DEBUG_PRINT("[S4_RX]: S%d -> S%d\n", prevState, curState);
}

static void S4_RX_NO(Ranging_Table_t *rangingTable) {
    // Calculate Tof, but not shift rangingTable
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;

    /* initialize               ===>    update
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |  Rp  |      |      | S4.1 |            |      |      |      |  Rp  | [Tr] |      | S4.1 |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
        |      |      |      |  Tp  |  Rr  |      |      |            |      |      |      |  Tp  |  Rr  |      | [Re] |
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
        |             |             |                                 |             |             |                                
        +------+------+------+------+                                 +------+------+------+------+
    */
    if(rangingTable->PTof == NULL_TOF && rangingTable->EPTof == NULL_TOF) {
        updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
    }

    /* swarm ranging            ===>    calculate and update
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S4.2 |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
        |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |            |      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
        |             |    PTof     |                                 |             |    PTof     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    else if(rangingTable->EPTof == NULL_TOF) {
        Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
        float curPTof = classicCalculatePTof(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr);
        distanceCalculate = (curPTof == NULL_TOF) ? (rangingTable->PTof * VELOCITY) / 2 : (curPTof * VELOCITY) / 2;

        updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
    }

    /* dynamic swarm ranging    ===>    calculate and update
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S4.3 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+         
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+          
        |    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    else {
        Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
        float curPTof = calculatePTof(rangingTable, candidate);
        distanceCalculate = (curPTof == NULL_TOF) ? (rangingTable->PTof * VELOCITY) / 2 : (curPTof * VELOCITY) / 2;

        updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
    }

    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S4_RX_NO]: S%d -> S%d\n", prevState, curState);
}

static void S5_TX(Ranging_Table_t *rangingTable) {
    /* Don't update Tx here since sending message is an async action, we put all Tx in sendList
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        |      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  |      |      | S5.1 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+           
        |      |      |      |  Tp  |  Rr  |  Tf  |      |            |      |      |      |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+           
        |             |             |                                 |             |             |                                
        +------+------+------+------+                                 +------+------+------+------+

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |            |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+           
        |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |            |      | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+           
        |             |    PTof     |                                 |             |    PTof     |                                
        +------+------+------+------+                                 +------+------+------+------+ 

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+           
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+           
        |    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S5;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S5_TX]: S%d -> S%d\n", prevState, curState);
}

static void S5_RX(Ranging_Table_t *rangingTable) {
    // Calculate Tof, shift the rangingTable, calculation is quick, directly skipping S6 and entering S4
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;

    /* initialize               ===>    calculate, update and shift
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  | [Tr] | [Rf] | S6.1 |            |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |  Tp  |  Rr  |  Tf  |      |    ===>    |      |      |      |  Tp  |  Rr  |  Tf  | [Re] |    ===>    |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |             |             |                                 |             |             |                                 |             |    PTof     |
        +------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
    */
    if(rangingTable->PTof == NULL_TOF && rangingTable->EPTof == NULL_TOF) {
        Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, rangingTable->Tf);
        float curPTof = classicCalculatePTof(rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr, rangingTable->Tf, rangingTable->Rf);
        if(curPTof != NULL_TOF) {
            distanceCalculate = (curPTof * VELOCITY) / 2;
        }

        updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

        if(curPTof != NULL_TOF) {
            shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPTof);
        }
    }

    /* swarm ranging            ===>    calculate, update and shift
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.2 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |    ===>    |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |             |    PTof     |                                 |             |    PTof     |                                 |   EPTof     |    PTof     |
        +------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
    */
    else if(rangingTable->EPTof == NULL_TOF) {
        Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, rangingTable->Tf);
        float curPTof = classicCalculatePTof(rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr, rangingTable->Tf, rangingTable->Rf);
        distanceCalculate = (curPTof == NULL_TOF) ? (rangingTable->PTof * VELOCITY) / 2 : (curPTof * VELOCITY) / 2;

        updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

        if(curPTof != NULL_TOF) {
            shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPTof);
        }
    }

    /* dynamic swarm ranging    ===>    calculate, update and shift
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                                 |   EPTof     |    PTof     |
        +------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
    */
    else {
        Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, rangingTable->Tf);
        float curPTof = calculatePTof(rangingTable, candidate);
        distanceCalculate = (curPTof == NULL_TOF) ? (rangingTable->PTof * VELOCITY) / 2 : (curPTof * VELOCITY) / 2;

        updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

        if(curPTof != NULL_TOF && candidate.Tr.seqNumber != NULL_SEQ && candidate.Rr.seqNumber != NULL_SEQ) {
            shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPTof);
        }
    }

    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S5_RX]: S%d -> S%d\n", prevState, curState);
}

static void S5_RX_NO(Ranging_Table_t *rangingTable) {
    // Calculate Tof, but not shift rangingTable
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;

    /* initialize               ===>    update
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  | [Tr] |      | S5.1 |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
        |      |      |      |  Tp  |  Rr  |  Tf  |      |            |      |      |      |  Tp  |  Rr  |  Tf  | [Re] |
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
        |             |             |                                 |             |             |                                
        +------+------+------+------+                                 +------+------+------+------+
    */
    if(rangingTable->PTof == NULL_TOF && rangingTable->EPTof == NULL_TOF) {
        updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
    }

    /* swarm ranging            ===>    calculate and update
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S4.2 |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
        |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |            |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
        |             |    PTof     |                                 |             |    PTof     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    else if(rangingTable->EPTof == NULL_TOF) {
        Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
        float curPTof = classicCalculatePTof(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr);
        distanceCalculate = (curPTof == NULL_TOF) ? (rangingTable->PTof * VELOCITY) / 2 : (curPTof * VELOCITY) / 2;

        updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
    }

    /* dynamic swarm ranging    ===>    calculate and update
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S4.3 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+         
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+          
        |    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    else {
        Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
        float curPTof = calculatePTof(rangingTable, candidate);
        distanceCalculate = (curPTof == NULL_TOF) ? (rangingTable->PTof * VELOCITY) / 2 : (curPTof * VELOCITY) / 2;

        updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
    }

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


/* -------------------- Generate and Process -------------------- */
Time_t generateMessage(Ranging_Message_t *rangingMessage) {
    Time_t taskDelay = M2T(RANGING_PERIOD);

    int8_t bodyUnitCount = 0;     
    rangingTableSet->localSeqNumber++;
    rangingMessage->header.filter = 0;

    /* generate bodyUnit */
    while(bodyUnitCount < MESSAGE_BODY_UNIT_SIZE && bodyUnitCount < rangingTableSet->size) {
        Message_Body_Unit_t *bodyUnit = &rangingMessage->bodyUnits[bodyUnitCount];

        index_t index = rangingTableSet->priorityQueue[bodyUnitCount];

        bodyUnit->address = rangingTableSet->rangingTable[index].neighborAddress;
        bodyUnit->Rxtimestamp = rangingTableSet->lastRxtimestamp[index];

        rangingMessage->header.filter |= 1 << (rangingTableSet->rangingTable[index].neighborAddress % 16);
        
        #ifdef STATE_MACHINE_ENABLE
            RangingTableEventHandler(&rangingTableSet->rangingTable[index], RANGING_EVENT_TX);
        #endif

        bodyUnitCount++;
    }

    // update priority queue
    updatePriorityQueue(rangingTableSet, bodyUnitCount);

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

    // should be called in rangingTxCallback(for test here)
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

    // clear expired rangingTable
    if(rangingTableSet->localSeqNumber % CHECK_PERIOD == 0) {
        checkExpiration(rangingTableSet);
    }

    // printRangingMessage(rangingMessage);

    return taskDelay;
}

void processMessage(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo) {
    Ranging_Message_t *rangingMessage = &rangingMessageWithAdditionalInfo->rangingMessage;

    distanceReal = NULL_DIS;
    distanceCalculate = NULL_DIS;

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

    Ranging_Table_t *rangingTable = &rangingTableSet->rangingTable[neighborIndex];

    Timestamp_Tuple_t Re = nullTimestampTuple;
    Re.timestamp = rangingMessageWithAdditionalInfo->timestamp;
    Re.seqNumber = rangingMessage->header.msgSequence;
    rangingTableSet->lastRxtimestamp[neighborIndex] = Re;
    
    #ifdef COORDINATE_SEND_ENABLE
        Coordinate_Tuple_t TxCoordinate = rangingMessage->header.TxCoordinate;
        Coordinate_Tuple_t RxCoordinate = rangingMessageWithAdditionalInfo->RxCoordinate;
        float Dx = (RxCoordinate.x - TxCoordinate.x);
        float Dy = (RxCoordinate.y - TxCoordinate.y);
        float Dz = (RxCoordinate.z - TxCoordinate.z);
        distanceReal = sqrtf(Dx * Dx + Dy * Dy + Dz * Dz) / 1000;
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
    index_t index_Tf = NULL_INDEX;
    index_Tf = findSendList(&rangingTableSet->sendList, Rf.seqNumber);
    if(index_Tf != NULL_INDEX) {
        Tf = rangingTableSet->sendList.Txtimestamps[index_Tf];
    }

    fillRangingTable(rangingTable, Tf, Rf, Re);

    /* process header */
    index_t index_Rr = rangingTable->TrRrBuffer.topIndex;
    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        // find corresponding Tr according to Rr to get a valid Tr-Rr pair, this approach may help when experiencing continuous packet loss
        if(rangingMessage->header.Txtimestamps[i].timestamp.full != NULL_TIMESTAMP 
           && rangingTable->TrRrBuffer.candidates[index_Rr].Rr.timestamp.full != NULL_TIMESTAMP 
           && rangingMessage->header.Txtimestamps[i].seqNumber == rangingTable->TrRrBuffer.candidates[index_Rr].Rr.seqNumber) {
            // Rr is updated in updateRangingTableRr_Buffer
            updateRangingTableTr_Buffer(&rangingTable->TrRrBuffer, rangingMessage->header.Txtimestamps[i]);
            break;
           }
    }

    if(Tf.timestamp.full != NULL_TIMESTAMP && Rf.timestamp.full != NULL_TIMESTAMP && Rf.seqNumber != rangingTable->Rp.seqNumber) {
        // RX
        RangingTableEventHandler(rangingTable, RANGING_EVENT_RX);
    }
    else {
        // RX_NO
        RangingTableEventHandler(rangingTable, RANGING_EVENT_RX_NO);
    }

    // print result
    if(distanceCalculate != NULL_DIS) {
        #ifdef COMPENSATE_ENABLE
            if(lastD == 0) {
                // initialize lastD
                lastD = distanceCalculate;
                DEBUG_PRINT("[current_%u]: ModifiedD = %f", MY_UWB_ADDRESS, distanceCalculate);
            }
            else {
                float CompensateD = (distanceCalculate - lastD) * COMPENSATE_RATE;
                lastD = distanceCalculate;
                DEBUG_PRINT("[current_%u]: ModifiedD = %f", MY_UWB_ADDRESS, distanceCalculate + CompensateD);
            }
        #else
            DEBUG_PRINT("[current_%u]: ModifiedD = %f", MY_UWB_ADDRESS, distanceCalculate);
        #endif

        #ifdef COORDINATE_SEND_ENABLE
            DEBUG_PRINT(", TrueD = %f", distanceReal);
        #endif
        DEBUG_PRINT(", time = %llu\n", Re.timestamp.full);
    }
    else {
        DEBUG_PRINT("[CalculatePTof]: CalculatePTof failed\n");
    }

    rangingTableSet->rangingTable[neighborIndex].expirationSign = false;

    printRangingTableSet(rangingTableSet);
}


/* -------------------- Call back -------------------- */
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