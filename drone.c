#include "socket_frame.h"
#include "local_host.h"
#include "queue_task.h"
#include "dynamic_swarm_ranging.h"


const char *local_drone_id;            
extern Local_Host_t *localHost;     
extern Ranging_Table_Set_t* rangingTableSet;     
extern int RangingPeriod;
QueueTaskLock_t queueTaskLock;                  // lock for task


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

        // don't receive messages from essage dropped randomly
        if (strcmp(msg.sender_id, local_drone_id) != 0) {
            #ifdef PACKET_LOSS_ENABLE
                // for random
                int random_value = ((rand() * 2654435761U) / getpid()) % 100;

                if (random_value < PACKET_LOSS_RATE) {
                    Ranging_Message_t *ranging_msg = (Ranging_Message_t*)msg.data;
                    printf("[QueueTaskRx]: message[%d] from %s dropped(rate = %d%%)\n",ranging_msg->header.msgSequence , msg.sender_id, PACKET_LOSS_RATE);
                    continue;
                }
            #endif

            if (msg.data_size != sizeof(Ranging_Message_t)) {
                printf("Receiving failed, size of data_size does not match\n");
                return NULL;
            }

            Ranging_Message_t *ranging_msg = (Ranging_Message_t*)msg.data;

            // get curTime and localLocation
            #ifdef COORDINATE_SEND_ENABLE
                modifyLocation();
                Coordinate_Tuple_t localLocation = getCurrentLocation();
                Coordinate_Tuple_t remoteLocation = ranging_msg->header.TxCoordinate;

                // DEBUG_PRINT("[local]:  x = %d, y = %d, z = %d\n[remote]: x = %d, y = %d, z = %d\n", localLocation.x, localLocation.y, localLocation.z, remoteLocation.x, remoteLocation.y, remoteLocation.z);
                
                float distance = sqrt(pow((localLocation.x - remoteLocation.x), 2) + pow((localLocation.y - remoteLocation.y), 2) + pow((localLocation.z - remoteLocation.z), 2));
                float Tof = (distance / 1000) / VELOCITY;
                
                local_sleep(Tof);
            #endif

            uint64_t curTime = xTaskGetTickCount();

            Ranging_Message_With_Additional_Info_t full_info;

            full_info.rangingMessage = *ranging_msg;
            full_info.timestamp.full = curTime;
            full_info.RxCoordinate = localLocation;
            
            // Rx
            // DEBUG_PRINT("[QueueTaskRx]: receive the message[%u] from %s at %llu\n", ranging_msg->header.msgSequence, msg.sender_id, curTime);
            QueueTaskRx(&queueTaskLock, &full_info, sizeof(full_info));
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

    int64_t worldBaseTime;
    ssize_t bytes_received = recv(center_socket, &worldBaseTime, sizeof(worldBaseTime), 0);
    if (bytes_received != sizeof(worldBaseTime)) {
        perror("Failed to receive timestamp");
        close(center_socket);
        return -1;
    }

    localHost->baseTime = worldBaseTime;

    srand((unsigned int)(get_current_milliseconds()));

    // start receive
    pthread_t receive_thread;
    if (pthread_create(&receive_thread, NULL, receive_from_center, &center_socket)) {
        perror("pthread_create for receive thread failed");
        close(center_socket);
        return -1;
    }

    printf("Node %s connected to center\n", local_drone_id);

    // main send loop
    while (1) {
        // DEBUG_PRINT("[QueueTaskTx]: send the message[%u] from %s at %llu\n", rangingTableSet->localSeqNumber, local_drone_id, xTaskGetTickCount());
        Time_t time_delay = QueueTaskTx(&queueTaskLock, MESSAGE_SIZE, send_to_center, center_socket, local_drone_id);

        local_sleep(time_delay); 
    }

    pthread_join(receive_thread, NULL);
    pthread_join(process_thread, NULL);
    close(center_socket);
    return 0;
}