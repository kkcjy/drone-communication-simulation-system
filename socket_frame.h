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

#include "ranging_protocol.h"

#define     MAX_NODES       10
#define     BUFFER_SIZE     1024
#define     ID_SIZE         20
#define     MESSAGE_SIZE    BUFFER_SIZE - ID_SIZE - sizeof(size_t)
#define     CENTER_PORT     8888
#define     REJECT_INFO     "REJECT"

typedef struct {
    char sender_id[ID_SIZE];
    char data[MESSAGE_SIZE];
    size_t data_size;
} NodeMessage;

typedef struct {
    int socket;
    char node_id[ID_SIZE];
} NodeInfo;

#endif