#include "adhocuwb_dynamic_swarm_ranging.h"


static uint16_t MY_UWB_ADDRESS;
Ranging_Table_Set_t *rangingTableSet;

#ifdef SIMULATION_COMPILE
extern const char* localAddress;
#else
static QueueHandle_t rxQueue;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;
#endif

int16_t dis_Real[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_DIS};
int16_t DSR[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_DIS};

#ifdef COMPENSATE_ENABLE
static int16_t last_DSR[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_DIS};
static int16_t his_avg_DSR[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_DIS};
static int16_t last_Seq[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_SEQ};
static int16_t last_SeqGap[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_SEQ};
static bool turning_state[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = false};
static double compensateRate = NULL_RATE;
#endif

#ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
static int8_t rangingPeriodFineTune = 0;


void Midpoint_Adjustment(dwTime_t curTxDwtime, ReceiveList_t *receiveList) {

    uint32_t ErrorAllowedinMs = 2;

    uint64_t minRxLast = UWB_MAX_TIMESTAMP;
    uint64_t minRxNext = UWB_MAX_TIMESTAMP;
    
    if (rangingTableSet->size ==0)
    	return;

    uint64_t TicksPerPeriod = (uint64_t)RANGING_PERIOD / (DWT_TIME_UNITS * 1000);
    uint64_t minRxExpected = TicksPerPeriod / rangingTableSet->size;
    uint64_t minRxError = (uint64_t)ErrorAllowedinMs / (DWT_TIME_UNITS * 1000);

    for (int i = receiveList->topIndex; 
                (i - 1 + RECEIVE_LIST_SIZE) % RECEIVE_LIST_SIZE != receiveList->topIndex; 
                i = (i - 1 + RECEIVE_LIST_SIZE) % RECEIVE_LIST_SIZE) {

        if(receiveList->Rxtimestamps[i].full == NULL_TIMESTAMP) {
            break;
        }
        
        /*
        +-------+------+-------+-------+-------+------+-------+-------+
        |  RX0  |  TX  |  RX1  |  RX2  |  RX3  |  TX  |  RX4  |  RX5  |
        +-------+------+-------+-------+-------+------+-------+-------+
                                           ^      ^       ^
                                         last    now    next
        */
        uint64_t curTxtimestamp = curTxDwtime.full % UWB_MAX_TIMESTAMP;
        uint64_t RxTimeStamp = receiveList->Rxtimestamps[i].full % UWB_MAX_TIMESTAMP;
        uint64_t diffTimestampLast = (curTxtimestamp - RxTimeStamp + UWB_MAX_TIMESTAMP) % (UWB_MAX_TIMESTAMP);
        uint64_t diffTimestampNext = (TicksPerPeriod + RxTimeStamp - curTxtimestamp + UWB_MAX_TIMESTAMP) % (UWB_MAX_TIMESTAMP);

        if(diffTimestampLast > TicksPerPeriod) {
            break;
        }

        minRxLast = diffTimestampLast < minRxLast ? diffTimestampLast : minRxLast;
        minRxNext = diffTimestampNext < minRxNext ? diffTimestampNext : minRxNext;
    }
    if(abs((int64_t)minRxExpected - (int64_t)minRxLast) <= minRxError)  {
        rangingPeriodFineTune = 0;
    }
    else if (minRxLast < minRxNext) {
        rangingPeriodFineTune = +1;
    }
    else if (minRxLast > minRxNext) {
        rangingPeriodFineTune = -1;
    }
    else if(minRxLast == UWB_MAX_TIMESTAMP && minRxNext == UWB_MAX_TIMESTAMP && rangingTableSet->size > 0) {
        rangingPeriodFineTune = -1;
    }
}
#endif


/* -------------------- Base Operation -------------------- */
// comparison of valid parts of timestamps considering cyclic overflow(time_a < time_b)
bool COMPARE_TIME(uint64_t time_a, uint64_t time_b) {
    uint64_t diff = (time_b - time_a) & (UWB_MAX_TIMESTAMP - 1);
    return diff < (UWB_MAX_TIMESTAMP - diff);
}


/* -------------------- Ranging Table Set Operation -------------------- */
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

#ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
void updateReceiveList(ReceiveList_t *receiveList, dwTime_t timestamp) {
    receiveList->topIndex = (receiveList->topIndex + 1) % RECEIVE_LIST_SIZE;
    receiveList->Rxtimestamps[receiveList->topIndex] = timestamp;
}
#endif

// init rangingTableTr_Rr_Buffer
void rangingTableTr_Rr_BufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer) {
    rangingTableBuffer->topIndex = NULL_INDEX;
    for (table_index_t i = 0; i < Tr_Rr_BUFFER_SIZE; i++) {
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
    rangingTableBuffer->topIndex = (rangingTableBuffer->topIndex + 1) % Tr_Rr_BUFFER_SIZE;
    rangingTableBuffer->candidates[rangingTableBuffer->topIndex].Tr = nullTimestampTuple;
    rangingTableBuffer->candidates[rangingTableBuffer->topIndex].Rr = Rr;
}

// get candidate in rangingTableTr_Rr_Buffer based on Rp and Tf
Ranging_Table_Tr_Rr_Candidate_t rangingTableTr_Rr_BufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tf) {
    index_t index = rangingTableBuffer->topIndex;

    uint64_t leftBound = Rp.timestamp.full % UWB_MAX_TIMESTAMP;
    uint64_t rightBound = Tf.timestamp.full % UWB_MAX_TIMESTAMP;

    Ranging_Table_Tr_Rr_Candidate_t candidate = nullCandidate;

    for (int count = 0; count < Tr_Rr_BUFFER_SIZE; count++) {
        if (rangingTableBuffer->candidates[index].Tr.timestamp.full 
            && rangingTableBuffer->candidates[index].Rr.timestamp.full 
            && COMPARE_TIME(leftBound, rangingTableBuffer->candidates[index].Tr.timestamp.full)
            && COMPARE_TIME(rangingTableBuffer->candidates[index].Rr.timestamp.full, rightBound)) {      
            candidate.Tr = rangingTableBuffer->candidates[index].Tr;
            candidate.Rr = rangingTableBuffer->candidates[index].Rr;
            break;
        }
        index = (index - 1 + Tr_Rr_BUFFER_SIZE) % Tr_Rr_BUFFER_SIZE;
    }

    return candidate;
}

// init rangingTable
void rangingTableInit(Ranging_Table_t *rangingTable) {
    rangingTable->neighborAddress = NULL_ADDRESS;
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

    rangingTable->PToF = NULL_TOF;
    rangingTable->EPToF = NULL_TOF;
    rangingTable->continuitySign = false;
    rangingTable->expirationSign = true;
    rangingTable->tableState = UNUSED;
    rangingTable->rangingState = RANGING_STATE_S1;
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

            // newly registered neighbor is in the initialization phase and cannot trigger calculations in the short term, so their priority is low
            rangingTableSet->priorityQueue[rangingTableSet->size] = index;
            rangingTableSet->size++;

            // DEBUG_PRINT("Registered new ranging table entry: Address = %u\n", address);
            return index;
        }
    }

    ASSERT(0 && "Warning: Should not be called\n");
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
        if(rangingTableSet->rangingTable[i].neighborAddress == address &&  rangingTableSet->rangingTable[i].tableState == USING) {
            tableIdx = i;
            break;
        }
    }

    if(tableIdx == NULL_INDEX) {
        ASSERT(0 && "Warning: Should not be called\n");
        return;
    }

    rangingTableInit(&rangingTableSet->rangingTable[tableIdx]);

    table_index_t queueIdx = NULL_INDEX;
    for(table_index_t i = 0; i < rangingTableSet->size; i++) {
        if(rangingTableSet->priorityQueue[i] == tableIdx) {
            queueIdx = i;
            break;
        }
    }

    if(queueIdx == NULL_INDEX) {
        ASSERT(0 && "Warning: Should not be called\n");
        return;
    }

    for(table_index_t i = queueIdx; i < rangingTableSet->size - 1; i++) {
        rangingTableSet->priorityQueue[i] = rangingTableSet->priorityQueue[i + 1];
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
        if (rangingTableSet->rangingTable[idx].neighborAddress == address && rangingTableSet->rangingTable[idx].tableState == USING) {
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
    |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |
    +------+------+------+------+                                 +------+------+------+------+
*/
void fillRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf, Timestamp_Tuple_t Re) {
    rangingTable->Tf = Tf;
    rangingTable->Rf = Rf;
    rangingTable->Re = Re;
}

/* shift Ranging Table
    +------+------+------+------+------+------+                   +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |                   |[ETb] |[ERp] | [Tb] | [Rp] |      |      |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |    ===>    |[ERb] |[ETp] | [Rb] | [Tp] |  Rr  |      |      |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    |    EPToF    |    PToF     |                                 |   [EPToF]   |   [PToF]    |
    +------+------+------+------+                                 +------+------+------+------+
*/
void shiftRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, float PToF) {
    rangingTable->ETb = rangingTable->Tb;
    rangingTable->ERb = rangingTable->Rb;
    rangingTable->ETp = rangingTable->Tp;
    rangingTable->ERp = rangingTable->Rp;
    rangingTable->EPToF = rangingTable->PToF;

    rangingTable->Tb = Tr;
    rangingTable->Rb = Rr;
    rangingTable->Tp = rangingTable->Tf;
    rangingTable->Rp = rangingTable->Rf;
    rangingTable->PToF = PToF;

    rangingTable->Tf = nullTimestampTuple;
    rangingTable->Rf = nullTimestampTuple;
    rangingTable->Re = nullTimestampTuple;

    // whether the table is contiguous
    if(rangingTable->ERb.seqNumber + 1 == rangingTable->Rb.seqNumber && rangingTable->ERp.seqNumber + 1 == rangingTable->Rp.seqNumber) {
        if(rangingTable->continuitySign == false) {
            rangingTable->continuitySign = true;
            // correcting PToF
            float correctPToF = classicCalculatePToF(rangingTable->ETp, rangingTable->ERp, rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp);
            rangingTable->PToF = correctPToF;
            // DEBUG_PRINT("[correct PToF]: correct PToF = %f\n", (double)correctPToF);
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
    |    EPToF    |    PToF     |                                 |    EPToF    |   [PToF]    |
    +------+------+------+------+                                 +------+------+------+------+
*/
void replaceRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, float PToF) {
    rangingTable->Tb = Tb;
    rangingTable->Rb = Rb;
    rangingTable->Tp = Tp;
    rangingTable->Rp = Rp;
    rangingTable->PToF = PToF;

    rangingTable->continuitySign = false;
}

// update the circular priority queue
void updatePriorityQueue(Ranging_Table_Set_t *rangingTableSet, int8_t shiftCount) {
    // no need to shift
    if(rangingTableSet->size <= 1 || shiftCount <= 0 || rangingTableSet->size == shiftCount) {
        return;
    }

    index_t *tmpQueue = malloc(sizeof(index_t) * rangingTableSet->size);
    if (!tmpQueue) {
        ASSERT(0 && "Warning: Should not be called\n");
    }

    for (index_t i = 0; i < rangingTableSet->size; i++) {
        tmpQueue[i] = rangingTableSet->priorityQueue[(i + shiftCount) % rangingTableSet->size];
    }

    for (index_t i = 0; i < rangingTableSet->size; i++) {
        rangingTableSet->priorityQueue[i] = tmpQueue[i];
    }

    free(tmpQueue);
}

// init rangingTableSet
void rangingTableSetInit() {
    #ifdef SIMULATION_COMPILE
        MY_UWB_ADDRESS = (uint16_t)strtoul(localAddress, NULL, 10);
    #endif

    rangingTableSet = (Ranging_Table_Set_t*)malloc(sizeof(Ranging_Table_Set_t));
    rangingTableSet->size = 0;
    rangingTableSet->localSeqNumber = NULL_SEQ;
    rangingTableSet->sendList.topIndex = NULL_INDEX;
    for (int i = 0; i < SEND_LIST_SIZE; i++) {
        rangingTableSet->sendList.Txtimestamps[i] = nullTimestampTuple;
    }
    #ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
    rangingTableSet->receiveList.topIndex = NULL_INDEX;
    for (int i = 0; i < RECEIVE_LIST_SIZE; i++) {
        rangingTableSet->receiveList.Rxtimestamps[i].full = NULL_TIMESTAMP;
    }
    #endif
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
void checkExpirationCallback(Ranging_Table_Set_t *rangingTableSet) {
    for(table_index_t i = rangingTableSet->size; i > 0; i--) {
        table_index_t idx = rangingTableSet->priorityQueue[i - 1];
        if(rangingTableSet->rangingTable[idx].expirationSign == true) {
            deregisterRangingTable(rangingTableSet, rangingTableSet->rangingTable[idx].neighborAddress);
        }
        else {
            rangingTableSet->rangingTable[idx].expirationSign = true;
        }
    }
}

// get substate of rangingTable
int getCurrentSubstate(Ranging_Table_t *rangingTable) {
    if (rangingTable->PToF == NULL_TOF && rangingTable->EPToF == NULL_TOF) {
        return RANGING_SUBSTATE_S1;
    } 
    else if (rangingTable->EPToF == NULL_TOF) {
        return RANGING_SUBSTATE_S2;
    } 
    else {
        return RANGING_SUBSTATE_S3;
    }
}


/* -------------------- Calculation Function -------------------- */
/* ranging algorithm
       R1   <--Db-->    T2      <--Rb-->       R3

    T1      <--Ra-->      R2    <--Da-->    T3
*/
float rangingAlgorithm(Timestamp_Tuple_t T1, Timestamp_Tuple_t R1, Timestamp_Tuple_t T2, Timestamp_Tuple_t R2, Timestamp_Tuple_t T3, Timestamp_Tuple_t R3, float ToF12) {
    uint64_t Ra = (R2.timestamp.full - T1.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    uint64_t Db = (T2.timestamp.full - R1.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    uint64_t Da = (T3.timestamp.full - R2.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    uint64_t Rb = (R3.timestamp.full - T2.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    uint64_t diffA = Ra - Da;
    uint64_t diffB = Rb - Db;

    float Ra_Da = Ra / (float)Da;
    float Rb_Db = Rb / (float)Db;

    float ToF23 = NULL_TOF;

    // meet the convergence condition
    if(Ra_Da < CONVERGENCE_THRESHOLD || Rb_Db < CONVERGENCE_THRESHOLD) {
        if(Ra_Da < Rb_Db) {
            ToF23 = (float)((diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / 2.0f - Ra * ToF12) / (float)Da;
        }
        else {
            ToF23 = (float)((diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / 2.0f - Rb * ToF12) / (float)Db;
        }
    }
    else {
        #ifdef CLASSIC_SUPPORT_ENABLE
            // DEBUG_PRINT("[rangingAlgorithm]: not meet the convergence condition, use classic algorithm.\n");
            ToF23 = classicCalculatePToF(T1, R1, T2, R2, T3, R3);
        #else
            DEBUG_PRINT("[rangingAlgorithm]: not meet the convergence condition.\n");
        #endif
    }

    // abnormal result
    float D = (ToF23 * VELOCITY) / 2;
    if(D < LOWER_BOUND_DISTANCE || D > UPPER_BOUND_DISTANCE) {
        // DEBUG_PRINT("[rangingAlgorithm]: dist = %f out of range.\n", D);
        return NULL_TOF;
    }

    return ToF23;
}

/* timestamps for classicCalculatePToF
       Rp   <--Db-->   Tr     <--Rb-->      Rf

    Tp      <--Ra-->     Rr   <--Da-->   Tf
*/ 
float classicCalculatePToF(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
    float curPToF = NULL_TOF;
    
    // check completeness
    if(Tp.seqNumber == NULL_SEQ || Rp.seqNumber == NULL_SEQ || Tr.seqNumber == NULL_SEQ
        || Rr.seqNumber == NULL_SEQ || Tf.seqNumber == NULL_SEQ || Rf.seqNumber == NULL_SEQ) {
        // DEBUG_PRINT("Warning: Data calculation is not complete\n");
        return NULL_TOF;
    }

    int64_t Ra = (Rr.timestamp.full - Tp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da = (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb = (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db = (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    int64_t diffA = Ra - Da;
    int64_t diffB = Rb - Db;

    curPToF = (diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / (float)(Ra + Db + Rb + Da);

    float D = (curPToF * VELOCITY) / 2;
    if(D < LOWER_BOUND_DISTANCE || D > UPPER_BOUND_DISTANCE) {
        // DEBUG_PRINT("[classicCalculatePToF]: dist = %f out of range.\n", D);
        return NULL_TOF;
    }

    return curPToF;
}

/* calculate the Time of Flight (ToF) based on the timestamps in the ranging table
    +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |
    +------+------+------+------+------+------+------+
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |
    +------+------+------+------+------+------+------+
    |    EPToF    |    PToF     |
    +------+------+------+------+
*/
float calculatePToF(Ranging_Table_t *rangingTable, Ranging_Table_Tr_Rr_Candidate_t candidate) {
    Timestamp_Tuple_t Tr = candidate.Tr;
    Timestamp_Tuple_t Rr = candidate.Rr;
    Timestamp_Tuple_t Tf = rangingTable->Tf;
    Timestamp_Tuple_t Rf = rangingTable->Rf;
    Timestamp_Tuple_t Re = rangingTable->Re;

    float tmpPToF = NULL_TOF;
    float curPToF = NULL_TOF;
    
    // check completeness
    if(Tr.seqNumber == NULL_SEQ || Rr.seqNumber == NULL_SEQ || Tf.seqNumber == NULL_SEQ || Rf.seqNumber == NULL_SEQ) {
        #ifdef COMPENSATE_ENABLE
            compensateRate = NULL_RATE;
        #endif

        /* type1
            +------+------+------+------+           +------+------+------+------+
            |  Tb  |  Rp  |  Tr  |      |           | ERp  |  Tb  |  Rp  |  Tr  |
            +------+------+------+------+   ===>    +------+------+------+------+
            |  Rb  |  Tp  |  Rr  |      |           | ETp  |  Rb  |  Tp  |  Rr  |
            +------+------+------+------+           +------+------+------+------+
        */ 
        if(Tr.seqNumber != NULL_SEQ && Rr.seqNumber != NULL_SEQ) {
            // DEBUG_PRINT("[calculatePToF]: Data calculation is not complete, pair of Tf-Rf is losed\n");
            tmpPToF = rangingAlgorithm(rangingTable->ETp, rangingTable->ERp, 
                                       rangingTable->Tb, rangingTable->Rb, 
                                       rangingTable->Tp, rangingTable->Rp, (rangingTable->EPToF + rangingTable->PToF) / 2);

            if(tmpPToF != NULL_TOF) {
                curPToF = rangingAlgorithm(rangingTable->Tb, rangingTable->Rb, 
                                           rangingTable->Tp, rangingTable->Rp,
                                           Tr, Rr, tmpPToF);
                // use tmpPToF as the estimated value
                curPToF = (curPToF == NULL_TOF) ? tmpPToF : curPToF;
            }
            else {
                return rangingTable->PToF;
            }
        }
        /* type2
            +------+------+------+------+           +------+------+------+------+
            |  Tb  |  Rp  |      |  Rf  |           | ETb  | ERp  |  Tb  |  Rf  |
            +------+------+------+------+   ===>    +------+------+------+------+
            |  Rb  |  Tp  |      |  Tf  |           | ERb  | ETp  |  Rb  |  Tf  |
            +------+------+------+------+           +------+------+------+------+
        */
        else if(Tf.seqNumber != NULL_SEQ && Rf.seqNumber != NULL_SEQ) {
            // DEBUG_PRINT("[calculatePToF]: Data calculation is not complete, pair of Tr-Rr is losed\n");
            tmpPToF = rangingAlgorithm(rangingTable->ETb, rangingTable->ERb, 
                                       rangingTable->ETp, rangingTable->ERp, 
                                       rangingTable->Tb, rangingTable->Rb, (rangingTable->EPToF + rangingTable->PToF) / 2);

            if(tmpPToF != NULL_TOF) {
                curPToF = rangingAlgorithm(rangingTable->ETp, rangingTable->ERp, 
                                           rangingTable->Tb, rangingTable->Rb,
                                           rangingTable->Tf, rangingTable->Rf, tmpPToF);
                // use tmpPToF as the estimated value
                curPToF = (curPToF == NULL_TOF) ? tmpPToF : curPToF;

                // replace pair of Tp-Rp with pair of Tf-Rf
                replaceRangingTable(rangingTable, rangingTable->Tb, rangingTable->Rb, rangingTable->Tf, rangingTable->Rf, curPToF);
            }
            else {
                return rangingTable->PToF;
            }
        }
        /* type3
            +------+------+------+------+      
            |  Tb  |  Rp  |      |      |          
            +------+------+------+------+   ===>    PToF
            |  Rb  |  Tp  |      |      |           
            +------+------+------+------+  
        */
        else {
            // DEBUG_PRINT("[calculatePToF]: Data calculation is not complete, pair of Tf-Rf and Tr-Rr are losed\n");
            return rangingTable->PToF;
        }
    }

    // normal
    else {
        /* type4
     
            +------+------+------+------+
            |  Tb  |  Rp  |  Tr  |  Rf  |
            +------+------+------+------+
            |  Rb  |  Tp  |  Rr  |  Tf  |
            +------+------+------+------+
        */ 
        tmpPToF = rangingAlgorithm(rangingTable->Tb, rangingTable->Rb, 
                                   rangingTable->Tp, rangingTable->Rp, 
                                   Tr, Rr, rangingTable->PToF);

        if(tmpPToF != NULL_TOF) {
            curPToF = rangingAlgorithm(rangingTable->Tp, rangingTable->Rp, 
                                       Tr, Rr,
                                       rangingTable->Tf, rangingTable->Rf, tmpPToF);
            // use tmpPToF as the estimated value
            curPToF = (curPToF == NULL_TOF) ? tmpPToF : curPToF;
            #ifdef COMPENSATE_ENABLE
                uint64_t diffReRr = (Re.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
                uint64_t diffTfRr = (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
                compensateRate = (double)(diffReRr - diffTfRr / 2.0) / (double)diffReRr;
                #endif
        }
        else {
            #ifdef COMPENSATE_ENABLE
                compensateRate = NULL_RATE;
            #endif
            return rangingTable->PToF;
        }
    }
    return curPToF;
}

void continuousPacketLossHandler(Ranging_Table_t *rangingTable, Ranging_Table_Tr_Rr_Candidate_t candidate) {
    // DEBUG_PRINT("[continuousPacketLossHandler]: A long-term continuous packet loss event occurred.\n");
    
    float curPToF_avg = (classicCalculatePToF(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr) 
                       + classicCalculatePToF(rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr, rangingTable->Tf, rangingTable->Rf)) / 2;
    DSR[rangingTable->neighborAddress] = (curPToF_avg * VELOCITY) / 2;
    
    updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

    shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPToF_avg);

    // goto S4.2
    rangingTable->ETb = nullTimestampTuple;
    rangingTable->ERb = nullTimestampTuple;
    rangingTable->EPToF = NULL_TOF;

    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    //DEBUG_PRINT("[S5_RX{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, RANGING_SUBSTATE_S3, curState, RANGING_SUBSTATE_S2);
}


/* -------------------- Print Function -------------------- */
void printRangingMessage(Ranging_Message_t *rangingMessage) {
    DEBUG_PRINT("\n{rangingMessage}\n");

    DEBUG_PRINT("srcAddress: %u\n", rangingMessage->header.srcAddress);
    DEBUG_PRINT("msgSequence: %u\n", rangingMessage->header.msgSequence);

    DEBUG_PRINT("[Tx]\n");
    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        DEBUG_PRINT("seqNumber: %u, timestamp: %llu\n", rangingMessage->header.Txtimestamps[i].seqNumber, rangingMessage->header.Txtimestamps[i].timestamp.full % UWB_MAX_TIMESTAMP);
    }

    DEBUG_PRINT("[Rx]:\n");
    for(int i = 0; i < MESSAGE_BODYUNIT_SIZE; i++){
        DEBUG_PRINT("address: %u, seqNumber: %u, timestamp: %llu\n", rangingMessage->bodyUnits[i].address, rangingMessage->bodyUnits[i].seqNumber, rangingMessage->bodyUnits[i].timestamp.full % UWB_MAX_TIMESTAMP);
    }
    DEBUG_PRINT("\n");
}

void printSendList(SendList_t *sendList) {
    DEBUG_PRINT("\n[sendList]\n");
    index_t index = sendList->topIndex;
    #ifdef COORDINATE_SEND_ENABLE
        DEBUG_PRINT("location: x = %u, y = %u, z = %u\n", sendList->TxCoordinate.x,  sendList->TxCoordinate.y,  sendList->TxCoordinate.z);
    #endif
    for(int i = 0; i < SEND_LIST_SIZE; i++) {
        DEBUG_PRINT("seqNumber: %u, timestamp: %llu\n", sendList->Txtimestamps[index].seqNumber, sendList->Txtimestamps[index].timestamp.full % UWB_MAX_TIMESTAMP);
        index = (index - 1 + SEND_LIST_SIZE) % SEND_LIST_SIZE;
    }
}

void printRangingTable(Ranging_Table_t *rangingTable) {
    DEBUG_PRINT("neighborAddress: %u\n", rangingTable->neighborAddress);
    DEBUG_PRINT("(ETb) seqNumber: %u, timestamp: %llu\n", rangingTable->ETb.seqNumber, rangingTable->ETb.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(ERb) seqNumber: %u, timestamp: %llu\n", rangingTable->ERb.seqNumber, rangingTable->ERb.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(ETp) seqNumber: %u, timestamp: %llu\n", rangingTable->ETp.seqNumber, rangingTable->ETp.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(ERp) seqNumber: %u, timestamp: %llu\n", rangingTable->ERp.seqNumber, rangingTable->ERp.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Tb) seqNumber: %u, timestamp: %llu\n", rangingTable->Tb.seqNumber, rangingTable->Tb.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Rb) seqNumber: %u, timestamp: %llu\n", rangingTable->Rb.seqNumber, rangingTable->Rb.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Tp) seqNumber: %u, timestamp: %llu\n", rangingTable->Tp.seqNumber, rangingTable->Tp.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Rp) seqNumber: %u, timestamp: %llu\n", rangingTable->Rp.seqNumber, rangingTable->Rp.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Tr_Rr_Buffer)\n");
    index_t index = rangingTable->TrRrBuffer.topIndex;
    for(int i = 0; i < Tr_Rr_BUFFER_SIZE; i++) {
        DEBUG_PRINT("\tseqNumber: %u, timestamp: %llu  -->  seqNumber: %u, timestamp: %llu\n", 
            rangingTable->TrRrBuffer.candidates[index].Tr.seqNumber, rangingTable->TrRrBuffer.candidates[index].Tr.timestamp.full % UWB_MAX_TIMESTAMP,
            rangingTable->TrRrBuffer.candidates[index].Rr.seqNumber, rangingTable->TrRrBuffer.candidates[index].Rr.timestamp.full % UWB_MAX_TIMESTAMP);
        index = (index - 1 + Tr_Rr_BUFFER_SIZE) % Tr_Rr_BUFFER_SIZE;
    }
    DEBUG_PRINT("(Tf) seqNumber: %u, timestamp: %llu\n", rangingTable->Tf.seqNumber, rangingTable->Tf.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Rf) seqNumber: %u, timestamp: %llu\n", rangingTable->Rf.seqNumber, rangingTable->Rf.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Re) seqNumber: %u, timestamp: %llu\n", rangingTable->Re.seqNumber, rangingTable->Re.timestamp.full % UWB_MAX_TIMESTAMP);
    
    DEBUG_PRINT("PToF = %f, EPToF = %f, continuitySign = %s\n", (double)rangingTable->PToF, (double)rangingTable->EPToF, rangingTable->continuitySign == true ? "true" : "false");
}

void printPriorityQueue(Ranging_Table_Set_t *rangingTableSet) {
    DEBUG_PRINT("\n[priorityQueue]\n");
    for(int i = 0; i < rangingTableSet->size; i++) {
        DEBUG_PRINT("priority: %d -> neighborAddress: %u\n", i + 1, rangingTableSet->rangingTable[rangingTableSet->priorityQueue[i]].neighborAddress);
    }
}

void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet) {
    DEBUG_PRINT("\n{rangingTableSet}\n");
    // DEBUG_PRINT("size: %u\n", rangingTableSet->size);
    DEBUG_PRINT("localSeqNumber: %lu\n", rangingTableSet->localSeqNumber);

    // printPriorityQueue(rangingTableSet);

    printSendList(&rangingTableSet->sendList);

    DEBUG_PRINT("\n[lastRxtimestamp]\n");
    for(int i = 0; i < rangingTableSet->size; i++) {
        DEBUG_PRINT("neighborAddress: %u, seqNumber: %u, timestamp: %llu\n", rangingTableSet->rangingTable[i].neighborAddress, rangingTableSet->lastRxtimestamp[i].seqNumber, rangingTableSet->lastRxtimestamp[i].timestamp.full % UWB_MAX_TIMESTAMP);
    }

    // DEBUG_PRINT("\n[rangingTable]\n");
    // for(int i = 0; i < rangingTableSet->size; i++) {
    //     printRangingTable(&rangingTableSet->rangingTable[rangingTableSet->priorityQueue[i]]);
    // }
}

void printclassicCalculateTuple(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
    DEBUG_PRINT("\nRp[seq=%u, ts=%llu] <--%llu--> Tr[seq=%u, ts=%llu] <--%llu--> Rf[seq=%u, ts=%llu]\n",
        Rp.seqNumber, Rp.timestamp.full % UWB_MAX_TIMESTAMP, (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP, Tr.seqNumber, Tr.timestamp.full % UWB_MAX_TIMESTAMP, (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Rf.seqNumber, Rf.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("\nTp[seq=%u, ts=%llu] <--%llu--> Rr[seq=%u, ts=%llu] <--%llu--> Tf[seq=%u, ts=%llu]\n",
        Tp.seqNumber, Tp.timestamp.full % UWB_MAX_TIMESTAMP, (Rr.timestamp.full - Tp.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Rr.seqNumber, Rr.timestamp.full % UWB_MAX_TIMESTAMP, (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Tf.seqNumber, Tf.timestamp.full % UWB_MAX_TIMESTAMP);
}

void printCalculateTuple(Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
    DEBUG_PRINT("\nTb[seq=%u, ts=%llu] <--%llu--> Rp[seq=%u, ts=%llu] <--%llu--> Tr[seq=%u, ts=%llu] <--%llu--> Rf[seq=%u, ts=%llu]\n",
                Tb.seqNumber,Tb.timestamp.full % UWB_MAX_TIMESTAMP,
                (Rp.timestamp.full -Tb.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Rp.seqNumber, Rp.timestamp.full % UWB_MAX_TIMESTAMP,
                (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Tr.seqNumber, Tr.timestamp.full % UWB_MAX_TIMESTAMP,
                (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Rf.seqNumber, Rf.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("\nRb[seq=%u, ts=%llu] <--%llu--> Tp[seq=%u, ts=%llu] <--%llu--> Rr[seq=%u, ts=%llu] <--%llu--> Tf[seq=%u, ts=%llu]\n",
                Rb.seqNumber,Rb.timestamp.full % UWB_MAX_TIMESTAMP,
                (Tp.timestamp.full -Rb.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP,Tp.seqNumber,Tp.timestamp.full % UWB_MAX_TIMESTAMP,
                (Rr.timestamp.full -Tp.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Rr.seqNumber, Rr.timestamp.full % UWB_MAX_TIMESTAMP,
                (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Tf.seqNumber, Tf.timestamp.full % UWB_MAX_TIMESTAMP);
}


/* -------------------- State Machine Operation -------------------- */
static void RESERVED_STUB(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[RESERVED_STUB]: Should not be called\n");
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

    //DEBUG_PRINT("[S1_TX]: S%d -> S%d\n", prevState, curState);
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

    //DEBUG_PRINT("[S1_RX{local_%u, neighbor_%u}]: S%d -> S%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState);
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

    //DEBUG_PRINT("[S1_RX_NO{local_%u, neighbor_%u}]: S%d -> S%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState);
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

    //DEBUG_PRINT("[S2_TX]: S%d -> S%d\n", prevState, curState);
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

    //DEBUG_PRINT("[S2_RX{local_%u, neighbor_%u}]: S%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState, RANGING_SUBSTATE_S1);
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

    //DEBUG_PRINT("[S2_RX_NO{local_%u, neighbor_%u}]: S%d -> S%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState);
}

static void S3_TX(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S3_TX]: Should not be called\n");
}

static void S3_RX(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S3_RX]: Should not be called\n");
}

static void S3_RX_NO(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S3_RX_NO]: Should not be called\n");
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
        |             |    PToF     |                                 |             |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |          
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S5;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    int curSubstate = getCurrentSubstate(rangingTable);
    //DEBUG_PRINT("[S4_TX]: S%d.%d -> S%d.%d\n", prevState, curSubstate, curState, curSubstate);
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
        |             |    PToF     |                                 |             |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.3 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+         
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+          
        |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;
    
    int curSubstate = getCurrentSubstate(rangingTable);
    //DEBUG_PRINT("[S4_RX{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, curSubstate);
}

static void S4_RX_NO(Ranging_Table_t *rangingTable) {
    // Calculate ToF, but not shift rangingTable
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    int curSubstate = getCurrentSubstate(rangingTable);

    switch (curSubstate) {
        case RANGING_SUBSTATE_S1: {
            /* initialize               ===>    update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      |      |      |  Rp  |      |      | S4.1 |            |      |      |      |  Rp  | [Tr] |      | S4.1 |
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
                |      |      |      |  Tp  |  Rr  |      |      |            |      |      |      |  Tp  |  Rr  |      | [Re] |
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
                |             |             |                                 |             |             |                                
                +------+------+------+------+                                 +------+------+------+------+
            */
            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        case RANGING_SUBSTATE_S2: {
            /* swarm ranging            ===>    calculate and update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S4.2 |
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
                |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |            |      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
                |             |    PToF     |                                 |             |    PToF     |                                
                +------+------+------+------+                                 +------+------+------+------+ 
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
            float curPToF = classicCalculatePToF(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr);
            DSR[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? ((rangingTable->PToF * VELOCITY) / 2) : ((curPToF * VELOCITY) / 2);

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        case RANGING_SUBSTATE_S3: {
            /* dynamic swarm ranging    ===>    calculate and update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
                | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S4.3 |          
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+         
                | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+          
                |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                
                +------+------+------+------+                                 +------+------+------+------+ 
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
            float curPToF = calculatePToF(rangingTable, candidate);
            DSR[rangingTable->neighborAddress] = (curPToF * VELOCITY) / 2;

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        default: {
            ASSERT(0 && "[S4_RX_NO]: Should not be called\n");
            break;
        }
    }

    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    //DEBUG_PRINT("[S4_RX_NO{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, curSubstate);
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
        |             |    PToF     |                                 |             |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+           
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+           
        |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S5;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    int curSubstate = getCurrentSubstate(rangingTable);
    //DEBUG_PRINT("[S5_TX]: S%d.%d -> S%d.%d\n", prevState, curSubstate, curState, curSubstate);
}

static void S5_RX(Ranging_Table_t *rangingTable) {
    // Calculate ToF, shift the rangingTable, calculation is quick, directly skipping S6 and entering S4
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    int curSubstate = getCurrentSubstate(rangingTable);

    switch (curSubstate) {
        case RANGING_SUBSTATE_S1: {
            /* initialize               ===>    calculate, update and shift
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  | [Tr] | [Rf] | S6.1 |            |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      |      |      |  Tp  |  Rr  |  Tf  |      |    ===>    |      |      |      |  Tp  |  Rr  |  Tf  | [Re] |    ===>    |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |             |             |                                 |             |             |                                 |             |    PToF     |
                +------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, rangingTable->Tf);
            float curPToF = classicCalculatePToF(rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr, rangingTable->Tf, rangingTable->Rf);
            if(curPToF != NULL_TOF) {
                DSR[rangingTable->neighborAddress] = (curPToF * VELOCITY) / 2;
            }

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

            if(curPToF != NULL_TOF) {
                shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPToF);
            }
            break;
        }

        case RANGING_SUBSTATE_S2: {
            /* swarm ranging            ===>    calculate, update and shift
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.2 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |    ===>    |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |             |    PToF     |                                 |             |    PToF     |                                 |   EPToF     |    PToF     |
                +------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, rangingTable->Tf);
            float curPToF = classicCalculatePToF(rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr, rangingTable->Tf, rangingTable->Rf);
            DSR[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? (rangingTable->PToF * VELOCITY) / 2 : (curPToF * VELOCITY) / 2;

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

            if(curPToF != NULL_TOF) {
                shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPToF);
            }
            break;
        }

        case RANGING_SUBSTATE_S3: {
            /* dynamic swarm ranging    ===>    calculate, update and shift
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                 |   EPToF     |    PToF     |
                +------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, rangingTable->Tf);
            
            uint16_t seqGap = rangingTable->Re.seqNumber - candidate.Rr.seqNumber;
            
            // long-term continuous packet loss
            if(candidate.Tr.seqNumber != NULL_SEQ && candidate.Rr.seqNumber != NULL_SEQ && seqGap >= SEQGAP_THRESHOLD) {
                continuousPacketLossHandler(rangingTable, candidate);
                return;
            }

            // normal
            else {
                float curPToF = calculatePToF(rangingTable, candidate);
                DSR[rangingTable->neighborAddress] = (curPToF * VELOCITY) / 2;

                updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

                if(curPToF != NULL_TOF && candidate.Tr.seqNumber != NULL_SEQ && candidate.Rr.seqNumber != NULL_SEQ) {
                    shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPToF);
                }
            }
            break;
        }
        default: {
            ASSERT(0 && "[S5_RX]: Should not be called\n");
            break;
        }
    }

    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    //DEBUG_PRINT("[S5_RX{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, (curSubstate == RANGING_SUBSTATE_S3) ? RANGING_SUBSTATE_S3 : curSubstate + 1);
}

static void S5_RX_NO(Ranging_Table_t *rangingTable) {
    // Calculate ToF, but not shift rangingTable
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    int curSubstate = getCurrentSubstate(rangingTable);

    switch (curSubstate) {
        case RANGING_SUBSTATE_S1: {
            /* initialize               ===>    update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  | [Tr] |      | S5.1 |
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
                |      |      |      |  Tp  |  Rr  |  Tf  |      |            |      |      |      |  Tp  |  Rr  |  Tf  | [Re] |
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
                |             |             |                                 |             |             |                                
                +------+------+------+------+                                 +------+------+------+------+
            */
            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        case RANGING_SUBSTATE_S2: {
            /* swarm ranging            ===>    calculate and update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S4.2 |
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
                |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |            |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |   
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
                |             |    PToF     |                                 |             |    PToF     |                                
                +------+------+------+------+                                 +------+------+------+------+ 
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
            float curPToF = classicCalculatePToF(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr);
            DSR[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? (rangingTable->PToF * VELOCITY) / 2 : (curPToF * VELOCITY) / 2;

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        case RANGING_SUBSTATE_S3: {
            /* dynamic swarm ranging    ===>    calculate and update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
                | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S4.3 |          
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+         
                | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |   
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+          
                |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                
                +------+------+------+------+                                 +------+------+------+------+ 
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
            float curPToF = calculatePToF(rangingTable, candidate);
            DSR[rangingTable->neighborAddress] = (curPToF * VELOCITY) / 2;

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        default: {
            ASSERT(0 && "[S5_RX_NO]: Should not be called\n");
            break;
        }
    }

    rangingTable->rangingState = RANGING_STATE_S5;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    //DEBUG_PRINT("[S5_RX_NO{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, curSubstate);
}

static void S6_TX(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S6_TX]: Should not be called\n");
}

static void S6_RX(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S6_RX]: Should not be called\n");
}

static void S6_RX_NO(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S6_RX_NO]: Should not be called\n");
}

/* State Machine
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |      | [Tr] | [Rf] |  S1  |            |      |      |      |      |      |      |  S1  |            |      |      |      |      | [Tr] |      |  S1  |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |      |      |      | [Re] |    <==>    |      |      |      |      |      |      |      |    <==>    |      |      |      |      |      |      | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
                                                                                          |
                                                                                          |  Tx
                                                                                          |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |      |      |      |  S2  |            |      |      |      |      |      |      |  S2  |            |      |      |      |      | [Tr] |      |  S2  |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |      |      | [Tf] |      |    <==>    |      |      |      |      |      | [Tf] |      |    <==>    |      |      |      |      |      |  Tf  | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
                                                                                          |
                                                                                          |  Rx
                                                                                          |
                                                              +------+------+------+------+------+------+------+
                                                              |      |      |      |      | [Tr] | [Rf] |  S3  |
                                                              +------+------+------+------+------+------+------+
                                                              |      |      |      |      |      |  Tf  | [Re] |
                                                              +------+------+------+------+------+------+------+
                                                              |             |             |
                                                              +------+------+------+------+
                                                                                          |
                                                                                          |  shift
                                                                                          |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |  Rp  | [Tr] | [Rf] | S4.1 |            |      |      |      |  Rp  |      |      | S4.1 |            |      |      |      |  Rp  | [Tr] |      | S4.1 |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |  Tp  |  Rr  |      | [Re] |    <==>    |      |      |      |  Tp  |  Rr  |      |      |    <==>    |      |      |      |  Tp  |  Rr  |      | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+    
                                                                                          |
                                                                                          |  Tx
                                                                                          |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  | [Tr] |      | S5.1 |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |  Tp  |  Rr  | [Tf] |      |    <==>    |      |      |      |  Tp  |  Rr  | [Tf] |      |    <==>    |      |      |      |  Tp  |  Rr  | [Tf] | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+    
                                                                                          |
                                                                                          |  Rx
                                                                                          |
                                                              +------+------+------+------+------+------+------+          
                                                              |      |      |      |  Rp  | [Tr] | [Rf] | S6.1 |         
                                                              +------+------+------+------+------+------+------+  
                                                              |      |      |      |  Tp  |  Rr  |  Tf  | [Re] |  
                                                              +------+------+------+------+------+------+------+         
                                                              |             |             |                               
                                                              +------+------+------+------+                             
                                                                                          |                                                 continuous packet loss
                                                                                          |  shift                  +-------------------------------------------------------------+
                                                                                          |                         |                                                             |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+<---+       +------+------+------+------+------+------+------+    |
|      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.2 |            |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S4.2 |    |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+    |
|      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |    |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|             |    PTof     |                                 |             |    PTof     |                                 |             |    PTof     |                         |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+                         |
                                                                                          |                                                                                       |
                                                                                          |  Tx                                                                                   |
                                                                                          |                                                                                       |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|      | ERp  |  Tb  |  Rp  |      |      | S5.2 |            |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S5.2 |    |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+    |
|      | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |    |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|             |    PTof     |                                 |             |    PTof     |                                 |             |    PTof     |                         |  
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+                         |
                                                                                          |                                                                                       |
                                                                                          |  Rx                                                                                   |
                                                                                          |                                                                                       |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              |      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.2 |                                                                  |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |                                                                  |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              |             |    PTof     |                                                                                       |
                                                              +------+------+------+------+                                                                                       |
                                                                                          |                                                 normal case                                               |
                                                                                          |  shift                  +-------------------------------------------------------------+
                                                                                          |                         |                                                             |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+<---+       +------+------+------+------+------+------+------+    |
| ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S4.3 |    |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+    |
| ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |    |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                         |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+                         |
                                                                                          |                                                                                       |
                                                                                          |  Tx                                                                                   |
                                                                                          |                                                                                       |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
| ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S5.3 |    |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+    |
| ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |    |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|    EPTof    |    PTof     |                                 |             |    PTof     |                                 |    EPTof    |    PTof     |                         |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+                         |
                                                                                          |                                                                                       | 
                                                                                          |  Rx                                                                                   |
                                                                                          |                                                                                       |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              | ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.3 |                                                                  |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |                                                                  |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              |    EPTof    |    PTof     |                                                                                       |
                                                              +------+------+------+------+                                                                                       |
                                                                                          |  shift                                                                                |
                                                                                          +---------------------------------------------------------------------------------------+ 
*/

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
    ASSERT((rangingTable->rangingState < RANGING_TABLE_STATE_COUNT) && "Warning: Should not be called\n");
    ASSERT((event < RANGING_TABLE_EVENT_COUNT) && "Warning: Should not be called\n");
    EVENT_HANDLER[rangingTable->rangingState][event](rangingTable);
}


/* -------------------- Generate and Process -------------------- */
void generateDSRMessage(Ranging_Message_t *rangingMessage) {
    int8_t bodyUnitCount = 0;     
    rangingTableSet->localSeqNumber++;
    rangingMessage->header.filter = 0;

    /* generate bodyUnit */
    while(bodyUnitCount < MESSAGE_BODYUNIT_SIZE && bodyUnitCount < rangingTableSet->size) {
        index_t index = rangingTableSet->priorityQueue[bodyUnitCount];

        // update timestamp field first to avoid memory overwrite
        rangingMessage->bodyUnits[bodyUnitCount].timestamp = rangingTableSet->lastRxtimestamp[index].timestamp;
        rangingMessage->bodyUnits[bodyUnitCount].seqNumber = rangingTableSet->lastRxtimestamp[index].seqNumber;
        rangingMessage->bodyUnits[bodyUnitCount].address = rangingTableSet->rangingTable[index].neighborAddress;

        rangingMessage->header.filter |= 1 << (rangingTableSet->rangingTable[index].neighborAddress % 16);
        
        RangingTableEventHandler(&rangingTableSet->rangingTable[index], RANGING_EVENT_TX);

        bodyUnitCount++;
    }

    // update priority queue
    updatePriorityQueue(rangingTableSet, bodyUnitCount);

    rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Ranging_Message_Body_Unit_t) * bodyUnitCount;

    // fill in empty info
    while(bodyUnitCount < MESSAGE_BODYUNIT_SIZE) {
        rangingMessage->bodyUnits[bodyUnitCount].timestamp.full = NULL_TIMESTAMP;
        rangingMessage->bodyUnits[bodyUnitCount].seqNumber = NULL_SEQ;
        rangingMessage->bodyUnits[bodyUnitCount].address = NULL_ADDR;
        bodyUnitCount++;
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

    // clear expired rangingTable
    if(rangingTableSet->localSeqNumber % CHECK_PERIOD == 0) {
        checkExpirationCallback(rangingTableSet);
    }

    // DEBUG_PRINT("[generate]\n");
    // printRangingMessage(rangingMessage);
}

void processDSRMessage(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo) {
    Ranging_Message_t *rangingMessage = &rangingMessageWithAdditionalInfo->rangingMessage;

    // DEBUG_PRINT("[process]\n");
    // printRangingMessage(rangingMessage);

    uint16_t neighborAddress = rangingMessage->header.srcAddress;

    if(neighborAddress > NEIGHBOR_ADDRESS_MAX) {
        DEBUG_PRINT("[processDSRMessage]: neighborAddress > NEIGHBOR_ADDRESS_MAX\n");
        return;
    }

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
        float distance = sqrtf(Dx * Dx + Dy * Dy + Dz * Dz);
        dis_Real[neighborAddress] = distance;
    #endif

    /* process bodyUnit */
    Timestamp_Tuple_t Rf = nullTimestampTuple;
    if (rangingMessage->header.filter & (1 << (MY_UWB_ADDRESS % 16))) {
        uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Ranging_Message_Body_Unit_t);
        for(int i = 0; i < bodyUnitCount; i++) {
            if(rangingMessage->bodyUnits[i].address == (uint8_t)MY_UWB_ADDRESS) {
                Rf.timestamp = rangingMessage->bodyUnits[i].timestamp;
                Rf.seqNumber = rangingMessage->bodyUnits[i].seqNumber;
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

    /*  即使对于 dynamic swarm ranging(以下用 DSR 简称), 计算出的距离值仍然会存在一定的滞后性。
        定义: ranging_i 为一次报文发送接收的动作, ToF_i 为该次报文传输所用的时间, Dis_i 为该次报文传输时双方的距离, Ti 和 Ri 为发送接收时间戳的时间。

            A   ···      Rp   <--Db-->   Tr      <--Rb-->      Rf

            B   ···   Tp      <--Ra-->      Rr   <--Da-->   Tf                    Re

                #       *                  *                  *                  *
                    Ranging_p          Ranging_r          Ranging_f          Ranging_e
                                          [SR]     [DSR]                        [CDSR]

        我们不妨让:
            ToF_r = ToF_f - ΔT1, 
            ToF_e = ToF_f + ΔT2, 
            ΔT1 + ΔT2 = ΔT
        当下 DSR 计算的过程实际上是用 ToF_DSR = (ToF_r + ToF_f) / 2, DSR = ToF_DSR * velocity 得到的计算距离。
        可以假定 # 号无人机静止(坐标系建立对相对位置关系无任何影响), * 号无人机远离(靠近同理), 我们可以将 ToF_DSR 看作报文从 # 传输到 Ranging_r 和 Ranging_f 中点所用的时间, 
        因此 DSR 获取的距离本质上离 Re 还是有一定的偏差, 尽管 DSR 本身已经相较于 SR 协议的 ToF_SR = ToF_r 更接近于 ToF_e。

        为了进一步贴近于当前状态下的 Ranging_e, 我们提出一种基于 DSR 的补偿机制:
        CDSR = DSR + ΔDis * Rate = ToF_DSR * velocity + ΔDis * Rate
        其中 ToF_DSR 的计算方式已经给出, 现在来探讨关于 ΔDis * Rate 的计算。

        1. 为了补偿 DSR 与真实值之间的距离差, 我们首先要计算出 ΔT 时间间隔内两架无人机相对移动的距离 ΔDis, 也就是 Dis_e - Dis_r。
            ···Tb      ^      Rp      Tr      ^      Rf
                       ^                      ^
            ···   Rb   ^   Tp            Rr   ^   Tf            Re
                   last_DSR                  DSR
        我们现有 last_DSR 和 DSR, 可以得出 ΔDSR = DSR - last_DSR, 且 ToF_DSR = (ToF_r + ToF_f) / 2, ToF_last_DSR = (ToF_b + ToF_p) / 2
        所以 2 * ΔDSR = (Dis_r + Dis_f) - (Dis_b + Dis_p) = (Dis_f - Dis_b) + (Dis_r - Dis_p)
        又因为在接收到报文并查找用于计算距离的上一次发送时间戳时，系统总是选择距离当前接收时刻最近的那一次历史发送时间戳, 并且通信双方发送报文的频率是相同且恒定的, 因此存在关系：
        ToF_r - ToF_p = ToF_e - ToF_f, 即: Dis_r - Dis_p = Dis_e - Dis_f
        所以能得到: 2 * ΔDSR = Dis_e - Dis_b
        SeqGap = Seq_e - Seq_r
        lastSeqGap = Seq_r - Seq_b
        因此 ΔDis = [SeqGap / (SeqGap + lastSeqGap)] * (2 * ΔDSR) = [SeqGap / (SeqGap + lastSeqGap)] * 2 * (DSR - last_DSR)

        2. 接下来要确定 Rate 的数值, Rate 是小于 1 的系数, 决定了补偿距离占 ΔDis 的比例。
        我们知道 ToF_DSR = (ToF_r + ToF_f) / 2, 所以有:
        Rate = [Re - Rr - (Tf - Rr) / 2] / (Re - Rr)

        最终能算出 CDSR = DSR + ΔDis * Rate。

        禁用场景:
        1. 连续丢包场景下, 历史计算值的参考性对于当下结果借鉴性不大, 为避免可能引入的额外误差
        2. 在相对静止场景下, 系统本身的计算值已经较好, 无需补偿引入额外噪声, 反而可能影响结果
        3. 在减速或者转向的场景下, 补偿结果可能会对当前的运动状态起到副作用
    */

    #if defined(COMPENSATE_ENABLE)
        if(last_DSR[neighborAddress] == NULL_DIS || last_Seq[neighborAddress] == NULL_SEQ) {
            last_DSR[neighborAddress] = DSR[neighborAddress];
            his_avg_DSR[neighborAddress] = DSR[neighborAddress];
            last_Seq[neighborAddress] = rangingMessage->header.msgSequence;
            DEBUG_PRINT("[local_%u <- neighbor_%u]: DSR dist = %f", MY_UWB_ADDRESS, neighborAddress, (double)DSR[neighborAddress]);
        }
        else if(last_SeqGap[neighborAddress] == NULL_SEQ) {
            turning_state[neighborAddress] = (DSR[neighborAddress] - last_DSR[neighborAddress]) >= 0;
            last_SeqGap[neighborAddress] = rangingMessage->header.msgSequence - last_Seq[neighborAddress];
            last_DSR[neighborAddress] = DSR[neighborAddress];
            his_avg_DSR[neighborAddress] = (his_avg_DSR[neighborAddress] + DSR[neighborAddress]) / 2;
            last_Seq[neighborAddress] = rangingMessage->header.msgSequence;
            DEBUG_PRINT("[local_%u <- neighbor_%u]: DSR dist = %f", MY_UWB_ADDRESS, neighborAddress, (double)DSR[neighborAddress]);
        }
        else {
            // calculate compensateDis
            uint16_t seqGap = rangingMessage->header.msgSequence - last_Seq[neighborAddress];
            uint16_t total_SeqGap = seqGap + last_SeqGap[neighborAddress];
            double delta_DSR = DSR[neighborAddress] - last_DSR[neighborAddress];
            double compensateDis = ((double)seqGap / total_SeqGap) * (2 * delta_DSR);

            // judge
            bool judge_static = abs(DSR[neighborAddress] - his_avg_DSR[neighborAddress]) < STATIC_BOUND;
            bool judge_deceleration = abs(DSR[neighborAddress] / seqGap) - abs(last_DSR[neighborAddress] / last_SeqGap[neighborAddress]) > DECELERATION_BOUND;
            bool judge_turning = ((DSR[neighborAddress] - last_DSR[neighborAddress]) >= 0) ^ turning_state[neighborAddress];

            // update
            turning_state[neighborAddress] = (DSR[neighborAddress] - last_DSR[neighborAddress]) >= 0;
            last_SeqGap[neighborAddress] = seqGap;
            last_DSR[neighborAddress] = DSR[neighborAddress];
            his_avg_DSR[neighborAddress] = (his_avg_DSR[neighborAddress] + DSR[neighborAddress]) / 2;
            last_Seq[neighborAddress] = rangingMessage->header.msgSequence;

            if(compensateRate != NULL_RATE) {
                double CDSR = DSR[neighborAddress] + compensateDis * compensateRate;
                if(seqGap > SEQGAP_THRESHOLD) {
                    if(seqGap > UINT16_MAX / 2) {
                        last_SeqGap[neighborAddress] = 0;
                    }
                    DEBUG_PRINT("[local_%u <- neighbor_%u]: DSR dist = %f", MY_UWB_ADDRESS, neighborAddress, (double)DSR[neighborAddress]);
                }
                else if(judge_static) {
                    DEBUG_PRINT("[local_%u <- neighbor_%u]: DSR dist = %f", MY_UWB_ADDRESS, neighborAddress, (double)DSR[neighborAddress]);
                }
                else if(judge_deceleration || judge_turning) {
                    DEBUG_PRINT("[local_%u <- neighbor_%u]: DSR dist = %f", MY_UWB_ADDRESS, neighborAddress, (double)DSR[neighborAddress]);
                }
                else {
                    DEBUG_PRINT("[local_%u <- neighbor_%u]: DSR dist = %f", MY_UWB_ADDRESS, neighborAddress, CDSR);
                }
            }
        }
    #else
        DEBUG_PRINT("[local_%u <- neighbor_%u]: DSR dist = %f", MY_UWB_ADDRESS, neighborAddress, (double)DSR[neighborAddress]);
    #endif

    #ifdef COORDINATE_SEND_ENABLE
        DEBUG_PRINT(", TrueD = %f", dis_Real[neighborAddress]);
    #endif
    DEBUG_PRINT(", time = %llu\n", Re.timestamp.full % UWB_MAX_TIMESTAMP);

    rangingTableSet->rangingTable[neighborIndex].expirationSign = false;

    // printRangingTableSet(rangingTableSet);
}


#ifndef SIMULATION_COMPILE
/* -------------------- Call back -------------------- */
static void uwbRangingTxTask(void *parameters) {
    systemWaitStart();

    UWB_Packet_t txPacketCache;
    txPacketCache.header.srcAddress = MY_UWB_ADDRESS;
    txPacketCache.header.destAddress = UWB_DEST_ANY;
    txPacketCache.header.type = UWB_RANGING_MESSAGE;
    txPacketCache.header.length = 0;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)&txPacketCache.payload;

    while (true) {
        xSemaphoreTake(rangingTableSet->mutex, portMAX_DELAY);

        // DEBUG_PRINT("[uwbRangingTxTask]: Acquired mutex, generating ranging message\n");
        Time_t rangingPeriod = RANGING_PERIOD;

        generateDSRMessage(rangingMessage);
        
        txPacketCache.header.seqNumber++;
        txPacketCache.header.length = sizeof(UWB_Packet_Header_t) + rangingMessage->header.msgLength;
        
        // push into txQueue
        uwbSendPacketBlock(&txPacketCache);

        xSemaphoreGive(rangingTableSet->mutex);

        #ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
            rangingPeriod += rangingPeriodFineTune;
            rangingPeriodFineTune = 0;
        #endif

        vTaskDelay(rangingPeriod);
    }
}

static void uwbRangingRxTask(void *parameters) {
    systemWaitStart();

    Ranging_Message_With_Additional_Info_t rxPacketCache;

    while (true) {
        vTaskDelay(M2T(1));
        if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
            xSemaphoreTake(rangingTableSet->mutex, portMAX_DELAY);

            processDSRMessage(&rxPacketCache);
            
            xSemaphoreGive(rangingTableSet->mutex);
        }
    }
}

// called after sending a packet
void rangingTxCallback(void *parameters) {
    UWB_Packet_t *packet = (UWB_Packet_t*)parameters;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)packet->payload;

    dwTime_t txTime;
    dwt_readtxtimestamp((uint8_t*)&txTime.raw);

    Timestamp_Tuple_t timestamp = {.timestamp = txTime, .seqNumber = rangingMessage->header.msgSequence};
    updateSendList(&rangingTableSet->sendList, timestamp);

    #ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
        Midpoint_Adjustment(rangingTableSet->sendList.Txtimestamps[rangingTableSet->sendList.topIndex].timestamp, &rangingTableSet->receiveList);
    #endif
}

// called after receiving a packet
void rangingRxCallback(void *parameters) {
    #ifdef PACKET_LOSS_ENABLE
        int randnum = rand() % 100;
        if(randnum < (int)(PACKET_LOSS_RATE * 100)) {
            return;
        }
    #endif

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    UWB_Packet_t *packet = (UWB_Packet_t*)parameters;

    dwTime_t rxTime;
    dwt_readrxtimestamp((uint8_t*)&rxTime.raw);

    #ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
        updateReceiveList(&rangingTableSet->receiveList, rxTime);
    #endif

    Ranging_Message_With_Additional_Info_t rxMessageWithTimestamp;
    rxMessageWithTimestamp.timestamp = rxTime;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)packet->payload;
    rxMessageWithTimestamp.rangingMessage = *rangingMessage;

    xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
}

void rangingInit() {
    MY_UWB_ADDRESS = uwbGetAddress();

    srand(MY_UWB_ADDRESS);

    rangingTableSetInit(&rangingTableSet);

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

#ifdef  CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
LOG_GROUP_START(Ranging)
LOG_ADD(LOG_INT16, distTo00, DSR + 0)
LOG_ADD(LOG_INT16, distTo01, DSR + 1)
LOG_ADD(LOG_INT16, distTo02, DSR + 2)
LOG_ADD(LOG_INT16, distTo03, DSR + 3)
LOG_ADD(LOG_INT16, distTo04, DSR + 4)
LOG_ADD(LOG_INT16, distTo05, DSR + 5)
LOG_ADD(LOG_INT16, distTo06, DSR + 6)
LOG_ADD(LOG_INT16, distTo07, DSR + 7)
LOG_ADD(LOG_INT16, distTo08, DSR + 8)
LOG_ADD(LOG_INT16, distTo09, DSR + 9)
LOG_ADD(LOG_INT16, distTo10, DSR + 10)
LOG_ADD(LOG_INT16, distTo11, DSR + 11)
LOG_ADD(LOG_INT16, distTo12, DSR + 12)
LOG_ADD(LOG_INT16, distTo13, DSR + 13)
LOG_ADD(LOG_INT16, distTo14, DSR + 14)
LOG_ADD(LOG_INT16, distTo15, DSR + 15)
LOG_ADD(LOG_INT16, distTo16, DSR + 16)
LOG_ADD(LOG_INT16, distTo17, DSR + 17)
LOG_ADD(LOG_INT16, distTo18, DSR + 18)
LOG_ADD(LOG_INT16, distTo19, DSR + 19)
LOG_ADD(LOG_INT16, distTo20, DSR + 20)
LOG_ADD(LOG_INT16, distTo21, DSR + 21)
LOG_ADD(LOG_INT16, distTo22, DSR + 22)
LOG_ADD(LOG_INT16, distTo23, DSR + 23)
LOG_ADD(LOG_INT16, distTo24, DSR + 24)
LOG_ADD(LOG_INT16, distTo25, DSR + 25)
LOG_ADD(LOG_INT16, distTo26, DSR + 26)
LOG_ADD(LOG_INT16, distTo27, DSR + 27)
LOG_ADD(LOG_INT16, distTo28, DSR + 28)
LOG_ADD(LOG_INT16, distTo29, DSR + 29)
LOG_GROUP_STOP(Ranging)
#endif
#endif
