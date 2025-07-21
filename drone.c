#include "socket_frame.h"
#include "local_host.h"
#include "queue_task.h"
#include "dynamic_swarm_ranging.h"


const char *local_drone_id;            
extern Local_Host_t *localHost;     
extern Ranging_Table_Set_t* rangingTableSet;     
extern int RangingPeriod;
QueueTaskLock_t queueTaskLock;                  // lock for task
dwTime_t TxTimestamp;                           // store timestamp from broadcast_flight_Log
dwTime_t RxTimestamp;                           // store timestamp from broadcast_flight_Log


void send_to_center(int center_socket, const char* node_id, const Ranging_Message_t* ranging_msg) {
    NodeMessage msg;

    if(sizeof(Ranging_Message_t) > MESSAGE_SIZE) {
        printf("Warning: %ld > %ld, Ranging_Message_t too large!\n", sizeof(Ranging_Message_t), MESSAGE_SIZE);
    }

    snprintf(msg.sender_id, sizeof(msg.sender_id), "%s", node_id);  

    memcpy(msg.data, ranging_msg, sizeof(Ranging_Message_t));

    msg.data_size = sizeof(Ranging_Message_t);

    if (send(center_socket, &msg, sizeof(msg), 0) < 0) {
        perror("Send failed");
    }
}

// for Rx
void *receive_from_center(void *arg) {
    int center_socket = *(int*)arg;
    NodeMessage msg;

    while (1) {
        //  center_socket -> msg
        ssize_t bytes_received = recv(center_socket, &msg, sizeof(msg), 0);

        if (bytes_received <= 0) {
            printf("Disconnected from Control Center\n");
            break;
        }

        if (strcmp(msg.data, REJECT_INFO) == 0) {
            printf("Connection rejected: Maximum drones reached (%d)\n", MAX_NODES);
            exit(1);
        }

        // ignore the message from itself
        if (strcmp(msg.sender_id, local_drone_id) != 0 || strcmp(msg.sender_id, "CENTER") == 0) {
            // from broadcast_flight_Log
            if (msg.data_size == sizeof(LineMessage)) {
                LineMessage *line_msg = (LineMessage *)msg.data;
                if(strcmp(local_drone_id, line_msg->drone_id) == 0) {
                    StatusType status = line_msg->status;
                    // sender -> ready to send
                    if(status == SENDER) {
                        QueueTaskTx(&queueTaskLock, MESSAGE_SIZE, send_to_center, center_socket, local_drone_id);

                        TxTimestamp = line_msg->timeStamp;
                        Timestamp_Tuple_t curTimestamp;
                        curTimestamp.seqNumber = rangingTableSet->localSeqNumber;
                        curTimestamp.timestamp = TxTimestamp;

                        #ifdef COORDINATE_SEND_ENABLE
                            updateSendList(&rangingTableSet->sendList, curTimestamp, nullCoordinateTuple);
                        #endif

                        TxTimestamp.full = NULL_TIMESTAMP;
                    }
                    // receiver -> prepare to receive
                    else if(status == RECEIVER) {
                        RxTimestamp = line_msg->timeStamp;
                    }
                }
            }

            // from broadcast_to_nodes
            else if(msg.data_size == sizeof(Ranging_Message_t)) {
                while (RxTimestamp.full == NULL_TIMESTAMP) ;

                Ranging_Message_t *ranging_msg = (Ranging_Message_t*)msg.data;
                Ranging_Message_With_Additional_Info_t full_info;

                full_info.rangingMessage = *ranging_msg;
                full_info.timestamp = RxTimestamp;
                RxTimestamp.full = NULL_TIMESTAMP;
            
                // Rx
                // DEBUG_PRINT("[QueueTaskRx]: receive the message[%u] from %s at %llu\n", ranging_msg->header.msgSequence, msg.sender_id, curTime);
                QueueTaskRx(&queueTaskLock, &full_info, sizeof(full_info));
            }
            else {
                    printf("Receiving failed, size of data_size does not match\n");
                    return NULL;
            }            
        }
    }
    return NULL;
}

// for processing
void *process_messages(void *arg) {
    while (1) {
        processFromQueue(&queueTaskLock);
        local_sleep(10);                      
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        printf("Usage: %s <center_ip> <local_drone_id>\n", argv[0]);
        return 1;
    }

    const char *center_ip = argv[1];
    local_drone_id = argv[2];

    // init operation
    TxTimestamp.full = NULL_TIMESTAMP;
    RxTimestamp.full = NULL_TIMESTAMP;

    localInit(string_to_hash(local_drone_id));
    DEBUG_PRINT("[localInit]: localHost is ready: droneId = %s, localAddress = %u, x = %u, y = %u, z = %u, vx = %d, vy = %d, vz = %d, randOffTime = %llu\n", 
        local_drone_id, localHost->localAddress, localHost->location.x, localHost->location.y, localHost->location.z, localHost->velocity.x, localHost->velocity.y, localHost->velocity.z, localHost->randOffTime);
    initQueueTaskLock(&queueTaskLock);     
    DEBUG_PRINT("[initQueueTaskLock]: QueueTaskLock is ready\n");
    rangingTableSetInit();
    DEBUG_PRINT("[initRangingTableSet]: RangingTableSet is ready\n");

    int center_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (center_socket < 0) {
        perror("Socket creation error");
        return -1;
    }

    struct sockaddr_in serv_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CENTER_PORT)
    };
    
    if (inet_pton(AF_INET, center_ip, &serv_addr.sin_addr) <= 0) {
        perror("Invalid address");
        return -1;
    }

    if (connect(center_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection Failed");
        return -1;
    }

    // Send drone ID first
    send(center_socket, local_drone_id, strlen(local_drone_id), 0);

    // start process
    pthread_t process_thread;
    if (pthread_create(&process_thread, NULL, process_messages, NULL)) {
        perror("pthread_create for process thread failed");
        close(center_socket);
        return -1;
    }

    // start receive
    pthread_t receive_thread;
    if (pthread_create(&receive_thread, NULL, receive_from_center, &center_socket)) {
        perror("pthread_create for receive thread failed");
        close(center_socket);
        return -1;
    }

    printf("Node %s connected to center\n", local_drone_id);

    pthread_join(receive_thread, NULL);
    pthread_join(process_thread, NULL);
    close(center_socket);
    return 0;
}