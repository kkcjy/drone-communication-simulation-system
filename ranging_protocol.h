#ifndef RANGING_PROTOCOL_H
#define RANGING_PROTOCOL_H


#include "ranging_defconfig.h"
#include "ranging_struct.h"
#include "local_host.h"


void rangingTableSetInit();
#ifdef COORDINATE_SEND_ENABLE
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple, Coordinate_Tuple_t coordinateTuple);
#else
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple);
#endif
typedef void (*EventHandlerTable)(Ranging_Table_t*);
Time_t generateMessage(Ranging_Message_t *rangingMessage);
void processMessage(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo);


#endif