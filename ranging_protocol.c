#include <stdlib.h>

#include "ranging_protocol.h"


static uint16_t MY_UWB_ADDRESS;
Ranging_Table_Set_t *rangingTableSet;
float distanceReal;
float distanceCalculate;

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

// -------------------- State Machine Operation --------------------
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


// -------------------- Generate and Process --------------------
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