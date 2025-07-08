#include "ranging_defconfig.h"
#include "ranging_struct.h"


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
    rangingTable->Rr = nullTimestampTuple;

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

/* shift Ranging Table
               <-------------
        <-------------              <------    
    +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  | EPTof| PTof |
    +------+------+------+------+------+------+
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | flag |
    +------+------+------+------+------+------+
        <-------------
               <-------------
*/
void shiftRangingTable(Ranging_Table_t *rangingTable) {
    rangingTable->ETb = rangingTable->Tb;
    rangingTable->ERb = rangingTable->Rb;
    rangingTable->ETp = rangingTable->Tp;
    rangingTable->ERp = rangingTable->Rp;

    rangingTable->EPTof = rangingTable->PTof;

    rangingTable->Tb = nullTimestampTuple;
    rangingTable->Rb = nullTimestampTuple;
    rangingTable->Tp = nullTimestampTuple;
    rangingTable->Rp = nullTimestampTuple;
    rangingTable->Rr = nullTimestampTuple;

    rangingTable->PTof = NULL_TOF;
}

/* fill Ranging Table
                      <-- Tr <-- Rf        <-- PTof
    +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  | EPTof| PTof |
    +------+------+------+------+------+------+
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | flag |
    +------+------+------+------+------+------+
                      <-- Rr <-- Tf <-- Re <-- update
*/
void fillRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf, Timestamp_Tuple_t Re, float PTof) {
    rangingTable->Tb = Tr;
    rangingTable->Rb = Rr;
    rangingTable->Tp = Tf;
    rangingTable->Rp = Rf;
    rangingTable->Rr = Re;
    rangingTable->PTof = PTof;
    // whether the table is contiguous
    if(rangingTable->ERp.seqNumber + 1 == rangingTable->Rp.seqNumber && rangingTable->Rb.seqNumber + 1 == rangingTable->Rr.seqNumber) {
        if(rangingTable->flag == false) {
            rangingTable->flag = true;
            // correcting PTof
            Ranging_Table_t correctRangingTable;
            correctRangingTable.Tp = rangingTable->ETp;
            correctRangingTable.Rp = rangingTable->ERp;
            float correctPTof = assistedCalculatePTof(&correctRangingTable, rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp);
            rangingTable->PTof = correctPTof;
            DEBUG_PRINT("[correct PTof]: correct PTof = %f\n", correctPTof);
        }
    }
    else {
        rangingTable->flag = false;
    }
}

/* replace Ranging Table
                      <-- Tr <-- Rf        <-- PTof
    +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  | EPTof| PTof |
    +------+------+------+------+------+------+
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | flag |
    +------+------+------+------+------+------+
                      <-- Rr <-- Tf
*/
void replaceRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf, float PTof) {
    rangingTable->Tb = Tr;
    rangingTable->Rb = Rr;
    rangingTable->Tp = Tf;
    rangingTable->Rp = Rf;
    rangingTable->PTof = PTof;
    rangingTable->flag = false;
}

/* timestamps for assistedCalculatePTof
       Rp   <--Db-->    Tr      <--Rb-->       Rf

    Tp      <--Ra-->      Rr    <--Da-->    Tf
*/ 
float assistedCalculatePTof(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
    Timestamp_Tuple_t Tp, Rp;
    Tp = rangingTable->Tp;
    Rp = rangingTable->Rp;

    // check completeness
    if(Tp.seqNumber == NULL_SEQ || Rp.seqNumber == NULL_SEQ || Tr.seqNumber == NULL_SEQ
        || Rr.seqNumber == NULL_SEQ || Tf.seqNumber == NULL_SEQ || Rf.seqNumber == NULL_SEQ) {
        DEBUG_PRINT("Warning: Data calculation is not complete\n");
        return INCOMPLETE_SIGN;
    }

    // printAssistedCalculateTuple(Tp, Rp, Tr, Rr, Tf, Rf);

    // check orderliness
    if(!(Tp.timestamp.full < Rr.timestamp.full && Rr.timestamp.full < Tf.timestamp.full
        && Rp.timestamp.full < Tr.timestamp.full && Tr.timestamp.full < Rf.timestamp.full)) {
        DEBUG_PRINT("Warning: Data calculation is not in order\n");
        return MISORDER_SIGN;
    }

    int64_t Ra = (Rr.timestamp.full - Tp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da = (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb = (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db = (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    int64_t diffA = Ra - Da;
    int64_t diffB = Rb - Db;

    float PTof = (diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / (float)(Ra + Db + Rb + Da);

    return PTof;
}

/* calculate the Time of Flight (Tof) based on the timestamps in the ranging table
    +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |
    +------+------+------+------+------+------+
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |
    +------+------+------+------+------+------+       
*/
float calculatePTof(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf, CalculateState state) {
    // check completeness
    if(state == FIRST_CALCULATE) {
        if(rangingTable->Tb.seqNumber == NULL_SEQ || rangingTable->Rb.seqNumber == NULL_SEQ || rangingTable->Tp.seqNumber == NULL_SEQ || rangingTable->Rp.seqNumber == NULL_SEQ 
            || Tr.seqNumber == NULL_SEQ || Rr.seqNumber == NULL_SEQ || Tf.seqNumber == NULL_SEQ || Rf.seqNumber == NULL_SEQ) {
            DEBUG_PRINT("Warning: Data calculation is not complete\n");
            return INCOMPLETE_SIGN;
        }
    }
    else if(state == SECOND_CALCULATE) {
        if(rangingTable->Tb.seqNumber == NULL_SEQ || rangingTable->Rb.seqNumber == NULL_SEQ || rangingTable->Tp.seqNumber == NULL_SEQ 
            || rangingTable->Rp.seqNumber == NULL_SEQ || Tr.seqNumber == NULL_SEQ || Rr.seqNumber == NULL_SEQ) {
            DEBUG_PRINT("Warning: Data calculation is not complete\n");
            return INCOMPLETE_SIGN;
        }
    }

    // printCalculateTuple(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, Tr, Rr, Tf, Rf);

/*
first step:
    try
    Tb       <--Ra1-->       Rp    <--Da1-->    Tr

       Rb    <--Db1-->    Tp       <--Rb1-->       Rr

    retry
   ETb       <--Ra1-->      ERp    <--Da1-->    Tr

      ERb    <--Db1-->   ETp       <--Rb1-->       Rr
----------------------------------------------------------------------
    Tof23 = Tof4 + Tofx = (Ra1 * Rb1 - Da1 * Db1) / Db1 - (Rb1 * PTof) / Db1
*/
    float Tof23 = NULL_TOF;

    // check orderliness
    if(!(rangingTable->Rb.timestamp.full < rangingTable->Tp.timestamp.full && rangingTable->Tp.timestamp.full < Rr.timestamp.full
        && rangingTable->Tb.timestamp.full < rangingTable->Rp.timestamp.full && rangingTable->Rp.timestamp.full < Tr.timestamp.full)) {
        // should be in order
        assert("Warning: Should not be called\n");
    }

    int64_t Ra1 = (rangingTable->Rp.timestamp.full - rangingTable->Tb.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db1 = (rangingTable->Tp.timestamp.full - rangingTable->Rb.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb1 = (Rr.timestamp.full - rangingTable->Tp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da1 = (Tr.timestamp.full - rangingTable->Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    int64_t diffA1 = Ra1 - Da1;
    int64_t diffB1 = Rb1 - Db1;

    float Ra_Da_1 = Ra1 / (float)Da1;
    float Rb_Db_1 = Rb1 / (float)Db1;

    // meet the convergence condition
    if(Ra_Da_1 < CONVERGENCE_THRESHOLD || Rb_Db_1 < CONVERGENCE_THRESHOLD) {
        if(Ra_Da_1 < Rb_Db_1) {
            Tof23 = (float)((diffA1 * Rb1 + diffA1 * Db1 + diffB1 * Ra1 + diffB1 * Da1) / 2 - Ra1 * rangingTable->PTof) / (float)Da1;
        }
        else {
            Tof23 = (float)((diffA1 * Rb1 + diffA1 * Db1 + diffB1 * Ra1 + diffB1 * Da1) / 2 - Rb1 * rangingTable->PTof) / (float)Db1;
        }

        // abnormal result
        float D = (Tof23 * VELOCITY) / 2;
        if(D < LOWER_BOUND_DISTANCE || D > UPPER_BOUND_DISTANCE) {
            if(state == FIRST_CALCULATE) {
                DEBUG_PRINT("Warning: D = %f is out of range of(%d, %d), retry to calculate\n", D, LOWER_BOUND_DISTANCE, UPPER_BOUND_DISTANCE);
                #if defined(CLASSIC_TOF_ENABLE)
                    // fallback to classic PTof calculation
                    Tof23 = (diffA1 * Rb1 + diffA1 * Db1 + diffB1 * Ra1 + diffB1 * Da1) / (float)(Ra1 + Db1 + Rb1 + Da1);
                    goto POINT_1;
                #else
                    // fallback to second calling PTof calculation
                    Ranging_Table_t replicaTable;
                    replicaTable.Tb = rangingTable->ETb;
                    replicaTable.Rb = rangingTable->ERb;
                    replicaTable.Tp = rangingTable->ETp;
                    replicaTable.Rp = rangingTable->ERp;
                    replicaTable.PTof = rangingTable->EPTof;

                    Tof23 = calculatePTof(&replicaTable, Tr, Rr, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
                    goto POINT_1;
                #endif
            }
            else if(state == SECOND_CALCULATE) {
                return NULL_TOF;
            }
        }

        if(state == SECOND_CALCULATE) {
            return Tof23;
        }
    }

    // beyond the convergence condition
    else {
        if(state == FIRST_CALCULATE) {
            DEBUG_PRINT("Warning: Ra1/Da1 and Rb1/Db1 are both greater than CONVERGENCE_THRESHOLD(%f), Ra_Da_1 = %f, Rb_Db_1 = %f, retry to calculate\n", CONVERGENCE_THRESHOLD, Ra_Da_1, Rb_Db_1);
            #if defined(CLASSIC_TOF_ENABLE)
                // fallback to classic PTof calculation
                Tof23 = (diffA1 * Rb1 + diffA1 * Db1 + diffB1 * Ra1 + diffB1 * Da1) / (float)(Ra1 + Db1 + Rb1 + Da1);
                goto POINT_1;
            #else
                // fallback to second calling PTof calculation
                Ranging_Table_t replicaTable;
                replicaTable.Tb = rangingTable->ETb;
                replicaTable.Rb = rangingTable->ERb;
                replicaTable.Tp = rangingTable->ETp;
                replicaTable.Rp = rangingTable->ERp;
                replicaTable.PTof = rangingTable->EPTof;

                Tof23 = calculatePTof(&replicaTable, Tr, Rr, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
                goto POINT_1;
            #endif
        }
        else if(state == SECOND_CALCULATE) {
            return NULL_TOF;
        }
    }

POINT_1:

/*
second step:
    try
       Rp    <--Db2-->    Tr      <--Rb2-->       Rf

    Tp       <--Ra2-->      Rr    <--Da2-->    Tf

    retry
      ERp    <--Db2-->    Tb      <--Rb2-->       Rf

   ETp       <--Ra2-->      Rb    <--Da2-->    Tf
----------------------------------------------------------------------
    Tof34 = Tofx + Tofn = (Ra2 * Rb2 - Da2 * Db2) / Db2 - Rb2 * Tof23 / Db2
*/
    float Tof34 = NULL_TOF;

     // check orderliness
    if(!(rangingTable->Tp.timestamp.full < Rr.timestamp.full && Rr.timestamp.full < Tf.timestamp.full
        && rangingTable->Rp.timestamp.full < Tr.timestamp.full && Tr.timestamp.full < Rf.timestamp.full)) {
        DEBUG_PRINT("Warning: Data calculation is not in order\n");
        return MISORDER_SIGN;
    }

    int64_t Ra2 = (Rr.timestamp.full - rangingTable->Tp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db2 = (Tr.timestamp.full - rangingTable->Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb2 = (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da2 = (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    
    int64_t diffA2 = Ra2 - Da2;
    int64_t diffB2 = Rb2 - Db2;
    float Ra_Da_2 = Ra2 / (float)Da2;
    float Rb_Db_2 = Rb2 / (float)Db2;

    if(Ra_Da_2 < CONVERGENCE_THRESHOLD || Rb_Db_2 < CONVERGENCE_THRESHOLD) {
        if(Ra_Da_2 < Rb_Db_2) {
            Tof34 = (float)((diffA2 * Rb2 + diffA2 * Db2 + diffB2 * Ra2 + diffB2 * Da2) / 2 - Ra2 * Tof23) / (float)Da2;
        }
        else {
            Tof34 = (float)((diffA2 * Rb2 + diffA2 * Db2 + diffB2 * Ra2 + diffB2 * Da2) / 2 - Rb2 * Tof23) / (float)Db2;
        }

        // abnormal result
        float D = (Tof34 * VELOCITY) / 2;
        if(D < LOWER_BOUND_DISTANCE || D > UPPER_BOUND_DISTANCE) {
            if(state == FIRST_CALCULATE) {
                DEBUG_PRINT("Warning: D = %f is out of range of(%d, %d), retry to calculate\n", D, LOWER_BOUND_DISTANCE, UPPER_BOUND_DISTANCE);
                #if defined(CLASSIC_TOF_ENABLE)
                    // fallback to classic PTof calculation
                    Tof34 = (diffA2 * Rb2 + diffA2 * Db2 + diffB2 * Ra2 + diffB2 * Da2) / (float)(Ra2 + Db2 + Rb2 + Da2);
                    goto POINT_2;
                #else
                    // fallback to second calling PTof calculation
                    Ranging_Table_t replicaTable;
                    replicaTable.Tb = rangingTable->ETp;
                    replicaTable.Rb = rangingTable->ERp;
                    replicaTable.Tp = rangingTable->Tb;
                    replicaTable.Rp = rangingTable->Rb;
                    replicaTable.PTof = (rangingTable->PTof + rangingTable->EPTof) / 2;

                    Tof34 = calculatePTof(&replicaTable, Tf, Rf, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
                    goto POINT_2;
                #endif
            }
            else if(state == SECOND_CALCULATE) {
                assert("Warning: Should not be called\n");
            }
        }
    } 
    else {
        if(state == FIRST_CALCULATE) {
            DEBUG_PRINT("Warning: Ra2/Da2 and Rb2/Db2 are both greater than CONVERGENCE_THRESHOLD(%f), Ra_Da_2 = %f, Rb_Db_2 = %f, retry to calculate\n", CONVERGENCE_THRESHOLD, Ra_Da_1, Rb_Db_1);
            #if defined(CLASSIC_TOF_ENABLE)
                // fallback to classic PTof calculation
                Tof34 = (diffA2 * Rb2 + diffA2 * Db2 + diffB2 * Ra2 + diffB2 * Da2) / (float)(Ra2 + Db2 + Rb2 + Da2);
                goto POINT_2;
            #else
                // fallback to second calling PTof calculation
                Ranging_Table_t replicaTable;
                replicaTable.Tb = rangingTable->ETp;
                replicaTable.Rb = rangingTable->ERp;
                replicaTable.Tp = rangingTable->Tb;
                replicaTable.Rp = rangingTable->Rb;
                replicaTable.PTof = (rangingTable->PTof + rangingTable->EPTof) / 2;

                Tof34 = calculatePTof(&replicaTable, Tf, Rf, nullTimestampTuple, nullTimestampTuple, SECOND_CALCULATE);
                goto POINT_2;
            #endif
        }
        else if(state == SECOND_CALCULATE) {
            assert("Warning: Should not be called\n");
        }
    }

POINT_2:

    return Tof34;
}

void printRangingMessage(Ranging_Message_t *rangingMessage) {
    DEBUG_PRINT("\n{rangingMessage}\n");

    DEBUG_PRINT("srcAddress: %u\n", rangingMessage->header.srcAddress);
    DEBUG_PRINT("msgSequence: %u\n", rangingMessage->header.msgSequence);

    DEBUG_PRINT("[Tr]\n");
    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        DEBUG_PRINT("seqNumber: %u, timestamp: %llu\n", rangingMessage->header.Txtimestamps[i].seqNumber, rangingMessage->header.Txtimestamps[i].timestamp.full);
    }

    DEBUG_PRINT("[Rr]:\n");
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
    DEBUG_PRINT("(ETb) seqNumber: %u, timestamp: %llu\n", rangingTable->ETb.seqNumber, rangingTable->ETb.timestamp.full);
    DEBUG_PRINT("(ERb) seqNumber: %u, timestamp: %llu\n", rangingTable->ERb.seqNumber, rangingTable->ERb.timestamp.full);
    DEBUG_PRINT("(ETp) seqNumber: %u, timestamp: %llu\n", rangingTable->ETp.seqNumber, rangingTable->ETp.timestamp.full);
    DEBUG_PRINT("(ERp) seqNumber: %u, timestamp: %llu\n", rangingTable->ERp.seqNumber, rangingTable->ERp.timestamp.full);
    DEBUG_PRINT("(Tb) seqNumber: %u, timestamp: %llu\n", rangingTable->Tb.seqNumber, rangingTable->Tb.timestamp.full);
    DEBUG_PRINT("(Rb) seqNumber: %u, timestamp: %llu\n", rangingTable->Rb.seqNumber, rangingTable->Rb.timestamp.full);
    DEBUG_PRINT("(Tp) seqNumber: %u, timestamp: %llu\n", rangingTable->Tp.seqNumber, rangingTable->Tp.timestamp.full);
    DEBUG_PRINT("(Rp) seqNumber: %u, timestamp: %llu\n", rangingTable->Rp.seqNumber, rangingTable->Rp.timestamp.full);
    DEBUG_PRINT("(Rr) seqNumber: %u, timestamp: %llu\n", rangingTable->Rr.seqNumber, rangingTable->Rr.timestamp.full);

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

void printAssistedCalculateTuple(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
    DEBUG_PRINT("\nRp[seq=%u, ts=%llu] <--%llu--> Tr[seq=%u, ts=%llu] <--%llu--> Rf[seq=%u, ts=%llu]\n",
        Rp.seqNumber, Rp.timestamp.full, Tr.timestamp.full - Rp.timestamp.full, Tr.seqNumber, Tr.timestamp.full, Rf.timestamp.full - Tr.timestamp.full, Rf.seqNumber, Rf.timestamp.full);
    DEBUG_PRINT("\nTp[seq=%u, ts=%llu] <--%llu--> Rr[seq=%u, ts=%llu] <--%llu--> Tf[seq=%u, ts=%llu]\n",
        Tp.seqNumber, Tp.timestamp.full, Rr.timestamp.full - Tp.timestamp.full, Rr.seqNumber, Rr.timestamp.full, Tf.timestamp.full - Rr.timestamp.full, Tf.seqNumber, Tf.timestamp.full);
}

void printCalculateTuple(Timestamp_Tuple_t ETb, Timestamp_Tuple_t ERb, Timestamp_Tuple_t ETp, Timestamp_Tuple_t ERp, Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp) {
    DEBUG_PRINT("\nT1[seq=%u, ts=%llu] <--%llu--> ERp[seq=%u, ts=%llu] <--%llu--> Tb[seq=%u, ts=%llu] <--%llu--> Rp[seq=%u, ts=%llu]\n",
               ETb.seqNumber,ETb.timestamp.full,
                ERp.timestamp.full -ETb.timestamp.full, ERp.seqNumber, ERp.timestamp.full,
                Tb.timestamp.full - ERp.timestamp.full, Tb.seqNumber, Tb.timestamp.full,
                Rp.timestamp.full - Tb.timestamp.full, Rp.seqNumber, Rp.timestamp.full);
    DEBUG_PRINT("\nR1[seq=%u, ts=%llu] <--%llu-->ETp[seq=%u, ts=%llu] <--%llu--> Rb[seq=%u, ts=%llu] <--%llu--> Tp[seq=%u, ts=%llu]\n",
               ERb.seqNumber,ERb.timestamp.full,
               ETp.timestamp.full -ERb.timestamp.full,ETp.seqNumber,ETp.timestamp.full,
                Rb.timestamp.full -ETp.timestamp.full, Rb.seqNumber, Rb.timestamp.full,
                Tp.timestamp.full - Rb.timestamp.full, Tp.seqNumber, Tp.timestamp.full);
}