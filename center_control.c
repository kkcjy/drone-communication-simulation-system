#include "socket_frame.h"
#include "dynamic_swarm_ranging.h"


NodeSet nodeSet;
long file_pos = 0;  


void nodeSetInit() {
    nodeSet.node_count = 0;
    pthread_mutex_init(&nodeSet.nodes_mutex);
}

int get_index_by_num(int number) {
    for (int i = 0; i < node_count; i++) {
        if (droneNode[i].number == number) {
            return i;
        }
    }
    return -1;
}

void broadcast_to_nodes(NodeMessage *msg) {
    pthread_mutex_lock(&nodes_mutex);
    for (int i = 0; i < node_count; i++) {
        if (send(droneNode[i].socket, msg, sizeof(NodeMessage), 0) < 0) {
            perror("Broadcast failed");
        }
    }
    pthread_mutex_unlock(&nodes_mutex);
}

void *handle_node_connection(void *arg) {
    int node_socket = *(int*)arg;
    free(arg);
    
    char node_id[ID_SIZE];
    ssize_t bytes_received = recv(node_socket, node_id, sizeof(node_id), 0);
    if (bytes_received <= 0) {
        close(node_socket);
        return NULL;
    }
    node_id[bytes_received] = '\0';

    pthread_mutex_lock(&nodes_mutex);
    if (node_count < MAX_NODES) {
        droneNode[node_count].socket = node_socket;
        strncpy(droneNode[node_count].node_id, node_id, sizeof(droneNode[node_count].node_id));
        droneNode[node_count].number = node_count;
        printf("Node %s connected\n", node_id);
    }
    else {
        printf("Max droneNode reached (%d), rejecting %s\n", MAX_NODES, node_id);
        NodeMessage reject_msg;
        strcpy(reject_msg.data, REJECT_INFO);
        reject_msg.data_size = strlen(reject_msg.data);
        send(node_socket, &reject_msg, sizeof(reject_msg), 0);
        close(node_socket);
        pthread_mutex_unlock(&nodes_mutex);
        return NULL;
    }
    pthread_mutex_unlock(&nodes_mutex);

    // broadcast tx from drone
    while (1) {
    NodeMessage msg;
        ssize_t bytes_received = recv(node_socket, &msg, sizeof(msg), 0);
        if (bytes_received <= 0) {
            break;
        }
        broadcast_to_nodes(&msg);
    }

    pthread_mutex_lock(&nodes_mutex);
    for (int i = 0; i < node_count; i++) {
        if (droneNode[i].socket == node_socket) {
            printf("Node %s disconnected\n", droneNode[i].node_id);
            droneNode[i] = droneNode[node_count - 1];
            node_count--;
            break;
        }
    }
    pthread_mutex_unlock(&nodes_mutex);

    close(node_socket);
    return NULL;
}

void *broadcast_flight_Log(void *arg) {
    while (1) {
        FILE *file = fopen("data/flight_Log.txt", "r");
        if (!file) {
            perror("Failed to open flight_Log.txt");
            sleep(1);
            continue;
        }

        fseek(file, file_pos, SEEK_SET);
        
        char line1[256], line2[256], line3[256];
        if (fgets(line1, sizeof(line1), file) &&
            fgets(line2, sizeof(line2), file) &&
            fgets(line3, sizeof(line3), file)) {

            file_pos = ftell(file);

            char *lines[3] = {line1, line2, line3};
            LineMessage line_msg;
            NodeMessage read_msg;

            for (int i = 0; i < 3; i++) {
                unsigned long long date_time;
                int drone_num;
                char status[3];

                memset(&line_msg, 0, sizeof(line_msg));
                memset(&read_msg, 0, sizeof(read_msg));

                if (sscanf(lines[i], "%llu %d %2s %lu", &date_time, &drone_num, status, &line_msg.timeStamp.full) == 4) {
                    int idx = get_index_by_num(drone_num);
                    if (idx < 0) {
                        continue;
                    }
                    snprintf(line_msg.drone_id, ID_SIZE, "%s", droneNode[idx].node_id);
                    line_msg.status = (strcmp(status, "TX") == 0) ? SENDER : RECEIVER;
                    memcpy(read_msg.data, &line_msg, sizeof(LineMessage));
                    strncpy(read_msg.sender_id, "CENTER", ID_SIZE);
                    read_msg.data_size = sizeof(LineMessage);
                    send(droneNode[idx].socket, &read_msg, sizeof(read_msg), 0);

                    printf("drone_num: %d, drone_id: %s, status: %s\n", drone_num, line_msg.drone_id, status);
                }
            }
        }
        fclose(file);
        local_sleep(READ_PERIOD);
    }
    return NULL;
}

int main() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
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

    if (listen(server_fd, MAX_NODES) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    printf("Control Center started on port %d (Max drones: %d)\n", CENTER_PORT, MAX_NODES);
    printf("Waiting for drone connections...\n");

    while (1) {
        int *new_socket = malloc(sizeof(int));
        socklen_t addrlen = sizeof(address);
        *new_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen);
        if (*new_socket < 0) {
            perror("accept");
            free(new_socket);
            continue;
        }

        pthread_t thread_id;
        if (pthread_create(&thread_id, NULL, handle_node_connection, new_socket)) {
            perror("pthread_create");
            close(*new_socket);
            free(new_socket);
        }
        pthread_detach(thread_id);

        // three drones connected
        pthread_mutex_lock(&nodes_mutex);
        if (node_count == MAX_NODES) {
            pthread_mutex_unlock(&nodes_mutex);
            break;
        }
        pthread_mutex_unlock(&nodes_mutex);
    }

    pthread_t broadcast_thread;
    pthread_create(&broadcast_thread, NULL, broadcast_flight_Log, NULL);

    while (1);

    pthread_detach(broadcast_thread);
    close(server_fd);
    return 0;
}