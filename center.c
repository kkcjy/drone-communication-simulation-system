#include "frame.h"


Drone_Node_Set_t *droneNodeSet;
pthread_mutex_t broadcast_rangingMessage_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t task_allocation_mutex = PTHREAD_MUTEX_INITIALIZER;


void droneNodeSet_init() {
    droneNodeSet = (Drone_Node_Set_t*)malloc(sizeof(Drone_Node_Set_t));
    memset(droneNodeSet->node, 0, sizeof(droneNodeSet->node));

    droneNodeSet->count = 0;

    if(pthread_mutex_init(&droneNodeSet->mutex, NULL) != 0) {
        perror("Mutex init failed");
        free(droneNodeSet);
        droneNodeSet = NULL;
        exit(EXIT_FAILURE);
    }
}

int count_rx_from_header(const char *header_line) {
    int rx_count = 0;
    char *copy = strdup(header_line);
    char *token = strtok(copy, ",");

    while (token != NULL) {
        if (strncmp(token, "Rx", 2) == 0 && strstr(token, "_addr")) {
            rx_count++;
        }
        token = strtok(NULL, ",");
    }

    free(copy);
    return rx_count;
}

void broadcast_rangingMessage(Simu_Message_t *simu_msg) {
    for (int i = 0; i < droneNodeSet->count; i++) {
        if (send(droneNodeSet->node[i].socket, simu_msg, sizeof(Simu_Message_t), 0) < 0) {
            perror("Failed to broadcast message");
        }
    }
}

void *broadcast_flightLog(void *arg) {
    FILE *fp = fopen(FILE_NAME, "r");
    if (!fp) {
        perror("Failed to open file");
        return NULL;
    }

    char line[MAX_LINE_LEN];

    if (!fgets(line, sizeof(line), fp)) {
        fprintf(stderr, "Empty file\n");
        fclose(fp);
        return NULL;
    }
    int rx_count = count_rx_from_header(line);
    printf("Detected Rx count: %d\n", rx_count);

    // Check if the number of drones matches the count from the file
    int drone_num = rx_count + 1;
    if(drone_num != droneNodeSet->count) {
        printf("Warning: droneNodeSet->count = %d, but drone_num read from file = %d\n", droneNodeSet->count, drone_num);
        exit(EXIT_FAILURE);
    }

    int line_count = 0;

    // Broadcast flight log to all drones
    while (fgets(line, sizeof(line), fp)) {
        if((line_count++ / NODES_NUM) % RANGING_PERIOD_RATE == 0) {
            usleep(READ_PERIOD * 1000); 

            if (*line == '\n' || *line == '\0') {
                break;
            }

            //wait for broadcast_rangingMessage
            pthread_mutex_lock(&task_allocation_mutex);
            pthread_mutex_lock(&broadcast_rangingMessage_mutex);

            // Tx task allocation
            Line_Message_t Tx_line_message;
            char *token = strtok(line, ",");
            token = strtok(NULL, ",");
            Tx_line_message.address = (uint16_t)strtoul(token, NULL, 10);
            Tx_line_message.status = TX;
            for (int i = 0; i < 3; i++) {
                token = strtok(NULL, ",");
            }
            Tx_line_message.timestamp.full = (uint64_t)strtoull(token, NULL, 10);

            for(int i = 0; i < droneNodeSet->count; i++) {
                if((uint16_t)strtoul(droneNodeSet->node[i].address, NULL, 10) == Tx_line_message.address) {
                    printf("[broadcast_flightLog]: Tx address = %d, Tx timestamp = %lu\n", Tx_line_message.address, Tx_line_message.timestamp.full);

                    Simu_Message_t simu_msg;
                    strncpy(simu_msg.srcAddress, CENTER_ADDRESS, ADDR_SIZE);
                    memcpy(simu_msg.payload, &Tx_line_message, sizeof(Line_Message_t));
                    simu_msg.size = sizeof(Line_Message_t);

                    if(send(droneNodeSet->node[i].socket, &simu_msg, sizeof(Simu_Message_t), 0) < 0) {
                        perror("Failed to send Tx message");
                    }
                }
            }

            // Rx task allocation
            for(int i = 0; i < rx_count; i++) {
                Line_Message_t Rx_line_message;
                token = strtok(NULL, ",");
                Rx_line_message.address = (uint16_t)strtoul(token, NULL, 10);
                Rx_line_message.status = RX;
                token = strtok(NULL, ",");
                Rx_line_message.timestamp.full = (uint64_t)strtoull(token, NULL, 10);

                // Skip packet loss cases
                if(Rx_line_message.timestamp.full == 0) {
                    continue;
                }

                for(int j = 0; j < droneNodeSet->count; j++) {
                    if((uint16_t)strtoul(droneNodeSet->node[j].address, NULL, 10) == Rx_line_message.address) {
                        printf("[broadcast_flightLog]: Rx address = %d, Rx timestamp = %lu\n", Rx_line_message.address, Rx_line_message.timestamp.full);

                        Simu_Message_t simu_msg;
                        strncpy(simu_msg.srcAddress, CENTER_ADDRESS, ADDR_SIZE);
                        memcpy(simu_msg.payload, &Rx_line_message, sizeof(Line_Message_t));
                        simu_msg.size = sizeof(Line_Message_t);

                        if(send(droneNodeSet->node[j].socket, &simu_msg, sizeof(Simu_Message_t), 0) < 0) {
                            perror("Failed to send Rx message");
                        }
                    }
                }
            }
            pthread_mutex_unlock(&broadcast_rangingMessage_mutex);
        }       
    }

    printf("Flight log broadcast completed.\n");
    fclose(fp);

    exit(EXIT_SUCCESS);
    return NULL;
}

void *handle_node_connection(void *arg) {
    int node_socket = *(int*)arg;
    free(arg);

    char node_address[ADDR_SIZE];
    ssize_t bytes_received = recv(node_socket, node_address, sizeof(node_address), 0);
    if (bytes_received <= 0) {
        close(node_socket);
        return NULL;
    }
    node_address[bytes_received] = '\0';
    
    if (droneNodeSet->count < NODES_NUM) {
        droneNodeSet->node[droneNodeSet->count].socket = node_socket;
        strncpy(droneNodeSet->node[droneNodeSet->count].address, node_address, ADDR_SIZE);
        droneNodeSet->count++;
        printf("New drone connected: %s\n", droneNodeSet->node[droneNodeSet->count - 1].address);
    }
    else {
        printf("Max nodes reached (%d), rejecting %s\n", NODES_NUM, node_address);
        close(node_socket);
        pthread_mutex_unlock(&droneNodeSet->mutex);
        return NULL;
    }
    pthread_mutex_unlock(&droneNodeSet->mutex);

    // Broadcast received message to all nodes
    Simu_Message_t simu_msg;
    while ((bytes_received = recv(node_socket, &simu_msg, sizeof(simu_msg), 0)) > 0) {
        Ranging_Message_t *ranging_msg = (Ranging_Message_t*)simu_msg.payload;
        // printf("[broadcast_rangingMessage]: address = %d, msgSeq = %d\n", ranging_msg->header.srcAddress, ranging_msg->header.msgSequence);
        
        // wait for task allocation
        pthread_mutex_lock(&broadcast_rangingMessage_mutex);
        broadcast_rangingMessage(&simu_msg);
        pthread_mutex_unlock(&broadcast_rangingMessage_mutex);
        pthread_mutex_unlock(&task_allocation_mutex);
    }

    // Handle disconnection
    pthread_mutex_lock(&droneNodeSet->mutex);
    for (int i = 0; i < droneNodeSet->count; i++) {
        // Find the node that matches the disconnected socket
        if (droneNodeSet->node[i].socket == node_socket) {
            printf("Node %s disconnected\n", droneNodeSet->node[i].address);
            for(int j = i + 1; j < droneNodeSet->count; j++) {
                droneNodeSet->node[j - 1] = droneNodeSet->node[j];
            }
            // Clear the last node
            memset(&droneNodeSet->node[droneNodeSet->count - 1], 0, sizeof(Drone_Node_t));
            droneNodeSet->count--;
            break;
        }
    }
    pthread_mutex_unlock(&droneNodeSet->mutex);

    close(node_socket);
    return NULL;
}

int main() {
    droneNodeSet_init();

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(CENTER_PORT)
    };

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, NODES_NUM) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    printf("Control Center started on port %d (Max drones: %d)\n", CENTER_PORT, NODES_NUM);
    printf("Waiting for drone connections...\n");

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addrlen = sizeof(client_addr);
        int *new_socket = malloc(sizeof(int));
        if (!new_socket) {
            perror("malloc failed");
            continue;
        }

        *new_socket = accept(server_fd, (struct sockaddr*)&client_addr, &addrlen);
        if (*new_socket < 0) {
            perror("accept");
            free(new_socket);
            continue;
        }

        pthread_mutex_lock(&droneNodeSet->mutex);
        pthread_t thread_id;
        if (pthread_create(&thread_id, NULL, handle_node_connection, new_socket) != 0) {
            perror("pthread_create");
            close(*new_socket);
            free(new_socket);
        }
        pthread_detach(thread_id);

        // make sure all drones connected before broadcast flightLog
        pthread_mutex_lock(&droneNodeSet->mutex);
        if (droneNodeSet->count == NODES_NUM) {
            pthread_mutex_unlock(&droneNodeSet->mutex);
            break;
        }
        pthread_mutex_unlock(&droneNodeSet->mutex);
    }

    printf("All drones connected, starting flight log broadcast...\n");

    // broadcast flightLog
    pthread_t broadcast_thread;
    pthread_create(&broadcast_thread, NULL, broadcast_flightLog, NULL);

    while (droneNodeSet->count);

    pthread_detach(broadcast_thread);
    close(server_fd);
    return 0;
}