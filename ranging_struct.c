#include "ranging_defconfig.h"
#include "ranging_struct.h"


void rangingTableInit(Ranging_Table_t *rangingTable) {
    rangingTable->neighborAddress = NULL_ADDR;
    rangingTable->T1 = nullTimestampTuple;
    rangingTable->R1 = nullTimestampTuple;
    rangingTable->T2 = nullTimestampTuple;
    rangingTable->R2 = nullTimestampTuple;
    rangingTable->T3 = nullTimestampTuple;
    rangingTable->R3 = nullTimestampTuple;
    rangingTable->T4 = nullTimestampTuple;
    rangingTable->R4 = nullTimestampTuple;
    rangingTable->Rx = nullTimestampTuple;

    rangingTable->PTof = NULL_TOF;
    rangingTable->EPTof = NULL_TOF;
    rangingTable->flag = false;
    rangingTable->state = UNUSED;
}

table_index_t registerRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address) {
    if (rangingTableSet->counter >= RANGING_TABLE_SIZE) {
        DEBUG_PRINT("Ranging table Set is full, cannot register new table\n");
        return NULL_INDEX;
    }

    rangingTableSet->rangingTable[rangingTableSet->counter].neighborAddress = address;
    rangingTableSet->rangingTable[rangingTableSet->counter].state = USING;
    rangingTableSet->priorityQueue[rangingTableSet->counter] = rangingTableSet->counter;
    rangingTableSet->counter++;

    DEBUG_PRINT("Registered new ranging table entry: Address=%u\n", address);

    return rangingTableSet->counter - 1;
}

table_index_t findRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address) {
    if(rangingTableSet->counter == 0) {
        // DEBUG_PRINT("Ranging table Set is empty, cannot find table\n");
        return NULL_INDEX;
    }
    for (table_index_t index = 0; index < rangingTableSet->counter; index++) {
        if (rangingTableSet->rangingTable[index].neighborAddress == address) {
            return index;
        }
    }
    // DEBUG_PRINT("Ranging table Set does not contain the address: %u\n", address);
    return NULL_INDEX;
}

/* Ranging Table Shift
               <-------------
        <-------------              <------    
    +------+------+------+------+------+------+
    |  T1  |  R2  |  T3  |  R4  | EPTof| PTof |
    +------+------+------+------+------+------+
    |  R1  |  T2  |  R3  |  T4  |  Rx  | flag |
    +------+------+------+------+------+------+
        <-------------
               <-------------
*/
void shiftRangingTable(Ranging_Table_t *rangingTable) {
    rangingTable->T1 = rangingTable->T3;
    rangingTable->R1 = rangingTable->R3;
    rangingTable->T2 = rangingTable->T4;
    rangingTable->R2 = rangingTable->R4;

    rangingTable->EPTof = rangingTable->PTof;

    rangingTable->R3 = nullTimestampTuple;
    rangingTable->T3 = nullTimestampTuple;
    rangingTable->T4 = nullTimestampTuple;
    rangingTable->R4 = nullTimestampTuple;
    rangingTable->Rx = nullTimestampTuple;

    rangingTable->PTof = NULL_TOF;
}

/* Ranging Table Insert
                      <-- Tx <-- Rn        <-- PTof
    +------+------+------+------+------+------+
    |  T1  |  R2  |  T3  |  R4  | EPTof| PTof |
    +------+------+------+------+------+------+
    |  R1  |  T2  |  R3  |  T4  |  Rx  | flag |
    +------+------+------+------+------+------+
                      <-- Rx <-- Tn <-- Rr <-- update
*/
void fillRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tx, Timestamp_Tuple_t Rx, Timestamp_Tuple_t Tn, Timestamp_Tuple_t Rn, Timestamp_Tuple_t Rr, float PTof) {
    rangingTable->T3 = Tx;
    rangingTable->R3 = Rx;
    rangingTable->T4 = Tn;
    rangingTable->R4 = Rn;
    rangingTable->Rx = Rr;
    rangingTable->PTof = PTof;
    // whether the table is contiguous
    if(rangingTable->R2.seqNumber + 1 == rangingTable->R4.seqNumber && rangingTable->R3.seqNumber + 1 == rangingTable->Rx.seqNumber) {
        if(rangingTable->flag == false) {
            rangingTable->flag = true;
            // correcting PTof
            Ranging_Table_t correctRangingTable;
            correctRangingTable.T4 = rangingTable->T2;
            correctRangingTable.R4 = rangingTable->R2;
            float correctTof = assistedCalculateTof(&correctRangingTable, Tx, Rx, Tn, Rn, CORRECT);
            rangingTable->PTof = correctTof;
        }
    }
    else if(rangingTable->R3.seqNumber + 1 != rangingTable->Rx.seqNumber){
        rangingTable->flag = false;
    }
}

/* assist calculating the Time of Flight (ToF) based on the timestamps in the ranging table
       Rp   <--Db-->    Tx      <--Rb-->       Rn

    Tp      <--Ra-->      Rx    <--Da-->    Tn

    INIT mode:      data should be in order
    CORRECT mode:   data should be strictly contigurous and in order
*/ 
float assistedCalculateTof(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tx, Timestamp_Tuple_t Rx, Timestamp_Tuple_t Tn, Timestamp_Tuple_t Rn, CalculateMode mode) {
    Timestamp_Tuple_t Tp, Rp;
    Tp = rangingTable->T4;
    Rp = rangingTable->R4;
    if(Tp.seqNumber == NULL_SEQ || Rp.seqNumber == NULL_SEQ || Tx.seqNumber == NULL_SEQ
        || Rx.seqNumber == NULL_SEQ || Tn.seqNumber == NULL_SEQ || Rn.seqNumber == NULL_SEQ) {
        DEBUG_PRINT("Warning: Computational data incompleteness\n");
        return NULL_TOF;
    }

    printAssistedCalculateTuple(Tp, Rp, Tx, Rx, Tn, Rn);

    if(mode == CORRECT) {
        // make sure Tp, Tn are contiguous
        if((uint16_t)(Tp.seqNumber + 1) != Tn.seqNumber) {
            DEBUG_PRINT("Warning: Computational data non-contiguous\n");
            return NULL_TOF;
        }
    }

    // make sure Tp, Rx, Tn are in order
    if(!(Tp.timestamp.full < Rx.timestamp.full && Rx.timestamp.full < Tn.timestamp.full)) {
        DEBUG_PRINT("Warning: Computational data not in order\n");
        return NULL_TOF;
    }

    int64_t Ra = (Rx.timestamp.full - Tp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da = (Tx.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb = (Rn.timestamp.full - Tx.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db = (Tn.timestamp.full - Rx.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    int64_t diffA = Ra - Da;
    int64_t diffB = Rb - Db;

    float PTof = (diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / (float)(Ra + Db + Rb + Da);

    return PTof;
}

/* calculate the Time of Flight (ToF) based on the timestamps in the ranging table
            +------+------+------+------+------+------+          +------+------+------+------+------+------+
            |  T1  |  R2  |  T3  |  R4  |  Tx  |  Rn  |          | ET1  | ER2  |  T1  |  R2  |  T3  |  R4  |
    view    +------+------+------+------+------+------+    as    +------+------+------+------+------+------+
            |  R1  |  T2  |  R3  |  T4  |  Rx  |  Tn  |          | ER1  | ET2  |  R1  |  T2  |  R3  |  T4  |
            +------+------+------+------+------+------+          +------+------+------+------+------+------+
*/
float calculateTof(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tx, Timestamp_Tuple_t Rx, Timestamp_Tuple_t Tn, Timestamp_Tuple_t Rn, CalculateState state) {
    if(rangingTable->T3.seqNumber == NULL_SEQ || rangingTable->R3.seqNumber == NULL_SEQ || rangingTable->T4.seqNumber == NULL_SEQ || rangingTable->R4.seqNumber == NULL_SEQ 
        || Tx.seqNumber == NULL_SEQ || Rx.seqNumber == NULL_SEQ || Tn.seqNumber == NULL_SEQ || Rn.seqNumber == NULL_SEQ) {
        DEBUG_PRINT("Error: Info of rangingTable is not full in calculateTof\n");
        return NULL_TOF;
    }
/*  
first calculation:
    first calling
    T1       <--Ra1-->       R2    <--Da1-->    T3

       R1    <--Db1-->    T2       <--Rb1-->       R3

    second calling
   ET1       <--Ra1-->      ER2    <--Da1-->    T3

      ER1    <--Db1-->   ET2       <--Rb1-->       R3
----------------------------------------------------------------------
    Tof23 = (Ra1 * Rb1 - Da1 * Db1) / Db1 - Rb1 * PTof / Db1
*/
    float Tof23 = NULL_TOF;

    int64_t Ra1 = (rangingTable->R4.timestamp.full - rangingTable->T3.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db1 = (rangingTable->T4.timestamp.full - rangingTable->R3.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb1 = (Rx.timestamp.full - rangingTable->T4.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da1 = (Tx.timestamp.full - rangingTable->R4.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    float Ra_Da_1 = Ra1 / (float)Da1;
    float Rb_Db_1 = Rb1 / (float)Db1;
    int64_t diffA1 = Ra1 - Da1;
    int64_t diffB1 = Rb1 - Db1;

    if(Ra_Da_1 < CONVERGENCE_THRESHOLD || Rb_Db_1 < CONVERGENCE_THRESHOLD) {
        if(Ra_Da_1 < Rb_Db_1) {
            Tof23 = (float)((diffA1 * Rb1 + diffA1 * Db1 + diffB1 * Ra1 + diffB1 * Da1) / 2 - Ra1 * rangingTable->PTof) / (float)Da1;
        } 
        else {
            Tof23 = (float)((diffA1 * Rb1 + diffA1 * Db1 + diffB1 * Ra1 + diffB1 * Da1) / 2 - Rb1 * rangingTable->PTof) / (float)Db1;
        }
        if(state == SECOND_CALCULATE) {
            return Tof23;
        }
    } 
    else {
        if(state == FIRST_CALCULATE) {
            DEBUG_PRINT("Warning: First Calling, Ra1/Da1 and Rb1/Db1 are both greater than CONVERGENCE_THRESHOLD(%d), Ra_Da_1 = %f, Rb_Db_1 = %f\n", CONVERGENCE_THRESHOLD, Ra_Da_1, Rb_Db_1);
            #if defined(CLASSIC_TOF_ENABLE)
                // Fallback to classic ToF calculation
                Tof23 = (diffA1 * Rb1 + diffA1 * Db1 + diffB1 * Ra1 + diffB1 * Da1) / (float)(Ra1 + Db1 + Rb1 + Da1);
            #else
                // Fallback to second calling ToF calculation
                Ranging_Table_t replicaTable;
                replicaTable.T3 = rangingTable->T1;
                replicaTable.R3 = rangingTable->R1;
                replicaTable.T4 = rangingTable->T2;
                replicaTable.R4 = rangingTable->R2;
                replicaTable.PTof = rangingTable->EPTof;

                Tof23 = calculateTof(&replicaTable, Tx, Rx, Tn, Rn, SECOND_CALCULATE);
            #endif
        }
        else if(state == SECOND_CALCULATE) {
            DEBUG_PRINT("Warning: Second Calling, recalculation also failed\n");
            return NULL_TOF;
        }

    }

/*
second calculation:
    first calling
       R2    <--Db2-->    T3      <--Rb2-->       R4

    T2       <--Ra2-->      R3    <--Da2-->    T4

    second calling
      ER2    <--Db2-->    T1      <--Rb2-->       R4

   ET2       <--Ra2-->      R1    <--Da2-->    T4
----------------------------------------------------------------------
    Tof34 = (Ra2 * Rb2 - Da2 * Db2) / Db2 - Rb2 * Tof23 / Db2
*/
    float Tof34 = NULL_TOF;

    int64_t Ra2 = (Rx.timestamp.full - rangingTable->T4.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db2 = (Tx.timestamp.full - rangingTable->R4.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb2 = (Rn.timestamp.full - Tx.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da2 = (Tn.timestamp.full - Rx.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    
    float Ra_Da_2 = Ra2 / (float)Da2;
    float Rb_Db_2 = Rb2 / (float)Db2;
    int64_t diffA2 = Ra2 - Da2;
    int64_t diffB2 = Rb2 - Db2;

    if(Ra_Da_2 < CONVERGENCE_THRESHOLD || Rb_Db_2 < CONVERGENCE_THRESHOLD) {
        if(Ra_Da_2 < Rb_Db_2) {
            Tof34 = (float)((diffA2 * Rb2 + diffA2 * Db2 + diffB2 * Ra2 + diffB2 * Da2) / 2 - Ra2 * Tof23) / (float)Da2;
        } 
        else {
            Tof34 = (float)((diffA2 * Rb2 + diffA2 * Db2 + diffB2 * Ra2 + diffB2 * Da2) / 2 - Rb2 * Tof23) / (float)Db2;
        }
    } 
    else {
        if(state == FIRST_CALCULATE) {
            DEBUG_PRINT("Warning: First Calling, Ra2/Da2 and Rb2/Db2 are both greater than CONVERGENCE_THRESHOLD(%d), Ra_Da_2 = %f, Rb_Db_2 = %f\n", CONVERGENCE_THRESHOLD, Ra_Da_2, Rb_Db_2);
            #if defined(CLASSIC_TOF_ENABLE)
                // Fallback to classic ToF calculation
                Tof34 = (diffA2 * Rb2 + diffA2 * Db2 + diffB2 * Ra2 + diffB2 * Da2) / (float)(Ra2 + Db2 + Rb2 + Da2);
            #else
                // Fallback to second calling ToF calculation
                Ranging_Table_t replicaTable;
                replicaTable.T3 = rangingTable->T2;
                replicaTable.R3 = rangingTable->R2;
                replicaTable.T4 = rangingTable->T3;
                replicaTable.R4 = rangingTable->R3;
                replicaTable.PTof = (rangingTable->PTof + rangingTable->EPTof) / 2;

                Tof34 = calculateTof(&replicaTable, Tn, Rn, Tx, Rx, SECOND_CALCULATE);
            #endif
        }
        else if(state == SECOND_CALCULATE) {
            DEBUG_PRINT("Warning: Should not be called\n");
            return NULL_TOF;
        }
    }
    return Tof34;
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

void printPriorityQueue(Ranging_Table_Set_t *rangingTableSet) {
    DEBUG_PRINT("\n[priorityQueue]\n");
    for(int i = 0; i < rangingTableSet->counter; i++) {
        DEBUG_PRINT("priority: %d -> neighborAddress: %u\n", i + 1, rangingTableSet->rangingTable[rangingTableSet->priorityQueue[i]].neighborAddress);
    }
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
    DEBUG_PRINT("(T1) seqNumber: %u, timestamp: %llu\n", rangingTable->T1.seqNumber, rangingTable->T1.timestamp.full);
    DEBUG_PRINT("(R1) seqNumber: %u, timestamp: %llu\n", rangingTable->R1.seqNumber, rangingTable->R1.timestamp.full);
    DEBUG_PRINT("(T2) seqNumber: %u, timestamp: %llu\n", rangingTable->T2.seqNumber, rangingTable->T2.timestamp.full);
    DEBUG_PRINT("(R2) seqNumber: %u, timestamp: %llu\n", rangingTable->R2.seqNumber, rangingTable->R2.timestamp.full);
    DEBUG_PRINT("(T3) seqNumber: %u, timestamp: %llu\n", rangingTable->T3.seqNumber, rangingTable->T3.timestamp.full);
    DEBUG_PRINT("(R3) seqNumber: %u, timestamp: %llu\n", rangingTable->R3.seqNumber, rangingTable->R3.timestamp.full);
    DEBUG_PRINT("(T4) seqNumber: %u, timestamp: %llu\n", rangingTable->T4.seqNumber, rangingTable->T4.timestamp.full);
    DEBUG_PRINT("(R4) seqNumber: %u, timestamp: %llu\n", rangingTable->R4.seqNumber, rangingTable->R4.timestamp.full);
    DEBUG_PRINT("(Rx) seqNumber: %u, timestamp: %llu\n", rangingTable->Rx.seqNumber, rangingTable->Rx.timestamp.full);

    DEBUG_PRINT("PTof = %f, EPTof = %f, flag = %s\n", rangingTable->PTof, rangingTable->EPTof, rangingTable->flag == true ? "true" : "false");
}

void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet) {
    DEBUG_PRINT("\n{rangingTableSet}\n");
    DEBUG_PRINT("counter: %u\n", rangingTableSet->counter);
    DEBUG_PRINT("localSeqNumber: %u\n", rangingTableSet->localSeqNumber);

    printPriorityQueue(rangingTableSet);

    printSendList(&rangingTableSet->sendList);

    DEBUG_PRINT("\n[lastRxtimestamp]\n");
    for(int i = 0; i < rangingTableSet->counter; i++) {
        DEBUG_PRINT("neighborAddress: %u, seqNumber: %u, timestamp: %llu\n", rangingTableSet->rangingTable[i].neighborAddress, rangingTableSet->lastRxtimestamp[i].seqNumber, rangingTableSet->lastRxtimestamp[i].timestamp.full);
    }

    DEBUG_PRINT("\n[rangingTable]\n");
    for(int i = 0; i < rangingTableSet->counter; i++) {
        printRangingTable(&rangingTableSet->rangingTable[i]);
    }
}

void printAssistedCalculateTuple(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tx, Timestamp_Tuple_t Rx, Timestamp_Tuple_t Tn, Timestamp_Tuple_t Rn) {
    DEBUG_PRINT("\nRp[seq=%u, ts=%llu] <--%llu--> Tx[seq=%u, ts=%llu] <--%llu--> Rn[seq=%u, ts=%llu]\n",
        Rp.seqNumber, Rp.timestamp.full, Tx.timestamp.full - Rp.timestamp.full, Tx.seqNumber, Tx.timestamp.full, Rn.timestamp.full - Tx.timestamp.full, Rn.seqNumber, Rn.timestamp.full);
    DEBUG_PRINT("\nTp[seq=%u, ts=%llu] <--%llu--> Rx[seq=%u, ts=%llu] <--%llu--> Tn[seq=%u, ts=%llu]\n",
        Tp.seqNumber, Tp.timestamp.full, Rx.timestamp.full - Tp.timestamp.full, Rx.seqNumber, Rx.timestamp.full, Tn.timestamp.full - Rx.timestamp.full, Tn.seqNumber, Tn.timestamp.full);
}