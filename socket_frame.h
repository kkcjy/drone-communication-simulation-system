#ifndef SOCKET_FRAME_H
#define SOCKET_FRAME_H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>

#include "dynamic_swarm_ranging.h"


#define     MAX_NODES       3
#define     MESSAGE_SIZE    1024
#define     ID_SIZE         20
#define     DATA_SIZE       MESSAGE_SIZE - ID_SIZE - sizeof(size_t)
#define     CENTER_PORT     8888
#define     REJECT_INFO     "REJECT"
#define     READ_PERIOD     200


typedef enum {
    SENDER,             // sender in communication
    RECEIVER            // receiver in communication
} StatusType;

typedef struct {
    char drone_id[ID_SIZE]; // id of drone
    StatusType status;      // status of drone: TX / RX
    dwTime_t timeStamp;     // TX / RX timeStamp
} LineMessage;

typedef struct {
    char sender_id[ID_SIZE];
    char data[DATA_SIZE];
    size_t data_size;
} NodeMessage;

typedef struct {
    int socket;
    char node_id[ID_SIZE];
    int idx;
} Node;

typedef struct {
    Node droneNode[MAX_NODES];
    int node_count;
    pthread_mutex_t nodes_mutex;
} NodeSet;

#endif