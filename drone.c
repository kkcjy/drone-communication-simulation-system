#define _POSIX_C_SOURCE 200809L 
#include "frame.h"


const char* localAddress;
#if defined(CLASSIC_RANGING_MODE)
extern Ranging_Table_Set_t rangingTableSet;
#elif defined(MODIFIED_RANGING_MODE)
extern Ranging_Table_Set_t *rangingTableSet;
#endif
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;      // mutex for synchronizing access to the rangingTableSet
extern dwTime_t lastTxTimestamp;                              
extern dwTime_t lastRxTimestamp;                              
extern dwTime_t TxTimestamp;                            // store timestamp from flightLog
extern dwTime_t RxTimestamp;                            // store timestamp from flightLog
#define MAX_LINE 512
#define DELIM ","
#define MAX_ROWS 7000
static char *csv_lines[MAX_ROWS];
static int total_rows = 0;
static int csv_pos = 0;


#if defined(IEEE_802_15_4Z)
#define     RANGING_MODE            "IEEE"
#elif defined(SWARM_RANGING_V1)
#define     RANGING_MODE            "SR_V1"
#elif defined(SWARM_RANGING_V2)
#define     RANGING_MODE            "SR_V2"
#elif defined(DYNAMIC_RANGING)
#define     RANGING_MODE            "DSR"
#elif defined(COMPENSATE_DYNAMIC_RANGING)
#define     RANGING_MODE            "CDSR"
#endif


int load_csv_to_memory(const char *filename) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        perror("open failed");
        return -1;
    }

    char line[MAX_LINE];
    total_rows = 0;

    if (!fgets(line, sizeof(line), fp)) {
        fclose(fp);
        return 0;
    }

    while (fgets(line, sizeof(line), fp) && total_rows < MAX_ROWS) {
        line[strcspn(line, "\r\n")] = 0;
        csv_lines[total_rows] = strdup(line);
        total_rows++;
    }

    fclose(fp);
    return total_rows;
}

uint64_t get_csv_cell_ll(int n, int m) {
    if (n < 0 || n >= total_rows) return 0;

    char *line_copy = strdup(csv_lines[n]);
    char *token = strtok(line_copy, DELIM);
    int col = 0;
    uint64_t result = 0;

    while (token) {
        if (col == m) {
            result = atoll(token);
            break;
        }
        token = strtok(NULL, DELIM);
        col++;
    }

    free(line_copy);
    return result;
}

void free_csv_memory() {
    for (int i = 0; i < total_rows; i++) free(csv_lines[i]);
    total_rows = 0;
    csv_pos = 0;
}

uint64_t get_next_RxTimestamp(uint16_t RxAddress, uint64_t RxTimestamp) {
    int matched = 0;

    for (int i = csv_pos; i < total_rows; i++) {
        uint16_t addr = (uint16_t)get_csv_cell_ll(i, 5); 
        uint64_t time = get_csv_cell_ll(i, 6);         

        if (!matched) {
            if (addr == RxAddress && time == RxTimestamp) {
                matched = 1;
            }
        } else {
            if (addr == RxAddress) {
                csv_pos = i; 
                return time;
            }
        }
    }

    csv_pos = total_rows;
    return NULL_TIMESTAMP;
}

void send_to_center(int center_socket, const char* address, const Ranging_Message_t *ranging_msg) {
    Simu_Message_t simu_msg;

    if(sizeof(Ranging_Message_t) > PAYLOAD_SIZE) {
        perror("Warning: Ranging_Message_t too large!\n");
    }

    snprintf(simu_msg.srcAddress, sizeof(simu_msg.srcAddress), "%s", address);
    memcpy(simu_msg.payload, ranging_msg, sizeof(Ranging_Message_t));
    simu_msg.size = sizeof(Ranging_Message_t);

    if (send(center_socket, &simu_msg, sizeof(Simu_Message_t), 0) < 0) {
        perror("Send failed");
    }
}

void response_to_center(int center_socket, const char* address) {
    Simu_Message_t simu_msg;

    snprintf(simu_msg.srcAddress, sizeof(simu_msg.srcAddress), "%s", address);
    simu_msg.size = sizeof(Line_Message_t);

    if (send(center_socket, &simu_msg, sizeof(Simu_Message_t), 0) < 0) {
        perror("Send failed");
    }
}

void TxCallBack(int center_socket, dwTime_t timestamp) {
    #if defined(CLASSIC_RANGING_MODE)
        Ranging_Message_t ranging_msg;

        generateRangingMessage(&ranging_msg);
        Timestamp_Tuple_t curTimeTuple = {
            .timestamp = timestamp,
            .seqNumber = ranging_msg.header.msgSequence
        };
        updateTfBuffer(curTimeTuple);

        send_to_center(center_socket, localAddress, &ranging_msg);
        
        // printf("Txcall, Txtimesatamp = %lu\n", timestamp.full);

        // reset of TxTimestamp in other place
    #elif defined(MODIFIED_RANGING_MODE)
        Ranging_Message_t ranging_msg;

        generateDSRMessage(&ranging_msg);
        Timestamp_Tuple_t curTimeTuple = {
            .timestamp = timestamp,
            .seqNumber = ranging_msg.header.msgSequence
        };
        updateSendList(&rangingTableSet->sendList, curTimeTuple);

        send_to_center(center_socket, localAddress, &ranging_msg);

        // printf("Txcall, Txtimesatamp = %lu\n", timestamp.full);
    
        // reset TxTimestamp after callback
        TxTimestamp.full = 0;
    #endif
}

void RxCallBack(int center_socket, Ranging_Message_t *rangingMessage, dwTime_t timestamp) {
    int randnum = rand() % 10000;
    if (randnum < (int)(PACKET_LOSS * 10000) || timestamp.full == 0) {
        response_to_center(center_socket, localAddress);
        return;
    }

    #if defined(CLASSIC_RANGING_MODE)
        Ranging_Message_With_Timestamp_t rangingMessageWithTimestamp;
        rangingMessageWithTimestamp.rangingMessage = *rangingMessage;
        rangingMessageWithTimestamp.rxTime = timestamp;
        
        processRangingMessage(&rangingMessageWithTimestamp);

        uint64_t next_RxTimestamp = get_next_RxTimestamp((uint16_t)strtoul(localAddress, NULL, 10), timestamp.full);
        uint64_t check_interval = ((next_RxTimestamp - timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP) / (CHECK_POINT + 1);
        for(int i = 1; i <= CHECK_POINT; i++) {
            uint64_t check_timestamp = (timestamp.full + check_interval * i) % UWB_MAX_TIMESTAMP;
            uint16_t neighborAddress = rangingMessage->header.srcAddress;
            int16_t distance = getDistance(neighborAddress);
            DEBUG_PRINT("[local_%u <- neighbor_%u]: %s dist = %d, time = %llu\n", (uint16_t)strtoul(localAddress, NULL, 10), neighborAddress, RANGING_MODE, distance, check_timestamp);
        }

        response_to_center(center_socket, localAddress);

        // printf("Rxcall, Rx timestamp = %lu\n", timestamp.full);

        // reset of RxTimestamp in other place
    #elif defined(MODIFIED_RANGING_MODE)
        Ranging_Message_With_Additional_Info_t rangingMessageWithAdditionalInfo;
        rangingMessageWithAdditionalInfo.rangingMessage = *rangingMessage;
        rangingMessageWithAdditionalInfo.timestamp = timestamp;

        processDSRMessage(&rangingMessageWithAdditionalInfo);

        uint16_t neighborAddress = rangingMessage->header.srcAddress;
        uint64_t next_RxTimestamp = get_next_RxTimestamp((uint16_t)strtoul(localAddress, NULL, 10), timestamp.full);
        if(next_RxTimestamp == NULL_TIMESTAMP) {
            for(int i = 1; i <= CHECK_POINT; i++) {
                double distance = getCurDistance(neighborAddress, NULL_TIMESTAMP);
                DEBUG_PRINT("[local_%u <- neighbor_%u]: %s dist = %f, time = %llu\n", (uint16_t)strtoul(localAddress, NULL, 10), neighborAddress, RANGING_MODE, distance, timestamp.full);
            }
        }
        else {
            uint64_t check_interval = ((next_RxTimestamp - timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP) / (CHECK_POINT + 1);
            for(int i = 1; i <= CHECK_POINT; i++) {
                uint64_t check_timestamp = (timestamp.full + check_interval * i) % UWB_MAX_TIMESTAMP;
                double distance = getCurDistance(neighborAddress, check_timestamp);
                DEBUG_PRINT("[local_%u <- neighbor_%u]: %s dist = %f, time = %llu\n", (uint16_t)strtoul(localAddress, NULL, 10), neighborAddress, RANGING_MODE, distance, check_timestamp);
            }
        }

        response_to_center(center_socket, localAddress);

        // printf("Rxcall, Rx timestamp = %lu\n", timestamp.full);

        // reset RxTimestamp after callback
        RxTimestamp.full = 0;
    #endif
}

void *receive_from_center(void *arg) {
    int center_socket = *(int*)arg;
    Simu_Message_t simu_msg;

    while(true) {
        ssize_t bytes_received = recv(center_socket, &simu_msg, sizeof(Simu_Message_t), 0);

        if(bytes_received <= 0) {
            printf("Disconnected from Control Center\n");
            exit(EXIT_SUCCESS);
            break;
        }

        // ignore the message from itself
        if(strcmp(simu_msg.srcAddress, localAddress) != 0) {
            // handle message of flightLog
            if(simu_msg.size == sizeof(Line_Message_t)) {
                Line_Message_t *line_message = (Line_Message_t*)simu_msg.payload;
                if(line_message->address == (uint16_t)strtoul(localAddress, NULL, 10)) {
                    // sender
                    if(line_message->status == TX) {
                        TxTimestamp.full = line_message->timestamp.full;
                        TxCallBack(center_socket, TxTimestamp);
                    }
                    // receiver
                    else if(line_message->status == RX) {
                        RxTimestamp.full = line_message->timestamp.full;
                        response_to_center(center_socket, localAddress);
                    }
                }
            }

            // handle message of rangingMessage
            else if(simu_msg.size == sizeof(Ranging_Message_t)) {
                Ranging_Message_t *ranging_msg = (Ranging_Message_t*)simu_msg.payload;
                RxCallBack(center_socket, ranging_msg, RxTimestamp);
            }
            else {
                printf("Received unknown message size: %zu\n", simu_msg.size);
                return NULL;
            }
        }
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: ./drone <localAddress>\n");
        return 1;
    }

    const char *center_ip = CENTER_IP;
    localAddress = argv[1];

    lastTxTimestamp.full = 0;
    lastRxTimestamp.full = 0;
    TxTimestamp.full = 0;
    RxTimestamp.full = 0;

    if (load_csv_to_memory("data/simulation_dep.csv") <= 0) {
        printf("Failed to load CSV\n");
        return 1;
    }

    #if defined(CLASSIC_RANGING_MODE)
        rangingTableSetInit(&rangingTableSet);
    #elif defined(MODIFIED_RANGING_MODE)
        rangingTableSetInit();
    #endif

    int center_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (center_socket < 0) {
        perror("Socket creation error");
        free_csv_memory();
        return -1;
    }

    struct sockaddr_in serv_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CENTER_PORT)
    };
    
    if (inet_pton(AF_INET, center_ip, &serv_addr.sin_addr) <= 0) {
        perror("Invalid address");
        close(center_socket); 
        free_csv_memory();
        return -1;
    }

    if (connect(center_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection Failed");
        close(center_socket); 
        free_csv_memory();
        return -1;
    }

    // Send drone ID first
    send(center_socket, localAddress, strlen(localAddress), 0);

    // Receive thread
    pthread_t receive_thread;
    if (pthread_create(&receive_thread, NULL, receive_from_center, &center_socket) != 0) {
        perror("Failed to create receive thread");
        close(center_socket);
        free_csv_memory();
        return -1;
    }

    printf("Node %s connected to center\n", localAddress);

    pthread_join(receive_thread, NULL);
    close(center_socket);

    free_csv_memory();

    return 0;
}