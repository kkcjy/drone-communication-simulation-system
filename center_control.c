#include "socket_frame.h"
#include "modified_ranging.h"

NodeInfo nodes[MAX_NODES];
int node_count = 0;
pthread_mutex_t nodes_mutex = PTHREAD_MUTEX_INITIALIZER;
uint64_t worldBaseTime = 0;

void broadcast_to_nodes(NodeMessage *msg) {
    pthread_mutex_lock(&nodes_mutex);
    for (int i = 0; i < node_count; i++) {
        if (send(nodes[i].socket, msg, sizeof(NodeMessage), 0) < 0) {
            perror("Broadcast failed");
        }
    }
    pthread_mutex_unlock(&nodes_mutex);
}

void *handle_node_connection(void *arg) {
    int node_socket = *(int *)arg;
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
        if(node_count == 0) {
            worldBaseTime = get_current_milliseconds();
            printf("set worldBaseTime = %ld\n", worldBaseTime);
        }
        nodes[node_count].socket = node_socket;
        strncpy(nodes[node_count].node_id, node_id, sizeof(nodes[node_count].node_id));
        node_count++;
        printf("Node %s connected\n", node_id);

        // send worldBaseTime to drone
        send(node_socket, &worldBaseTime, sizeof(worldBaseTime), 0);
    }
    else {
        printf("Max nodes reached (%d), rejecting %s\n", MAX_NODES, node_id);
        NodeMessage reject_msg;
        strcpy(reject_msg.data, REJECT_INFO);
        reject_msg.data_size = strlen(reject_msg.data);
        send(node_socket, &reject_msg, sizeof(reject_msg), 0);
        close(node_socket);
        pthread_mutex_unlock(&nodes_mutex);
        return NULL;
    }
    pthread_mutex_unlock(&nodes_mutex);

    NodeMessage msg;
    while ((bytes_received = recv(node_socket, &msg, sizeof(msg), 0)) > 0) {
        // Print function
        MessageWithLocation *modified_msg = (MessageWithLocation*)msg.data; 
        Ranging_Message_t *rangingMessage = &modified_msg->rangingMessage;
        // printf("\n********************[%s]********************\n", node_id);
        // printRangingMessage(rangingMessage);
        // printf("********************[%s]********************\n", node_id);
        printf("broadcast [%s], address = %d, msgSeq = %d, time = %ld\n", node_id, rangingMessage->header.srcAddress, rangingMessage->header.msgSequence, get_current_milliseconds() - worldBaseTime);

        // Immediately broadcast received message to all nodes
        broadcast_to_nodes(&msg);
    }

    pthread_mutex_lock(&nodes_mutex);
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].socket == node_socket) {
            printf("Node %s disconnected\n", nodes[i].node_id);
            nodes[i] = nodes[node_count - 1];
            node_count--;
            break;
        }
    }
    pthread_mutex_unlock(&nodes_mutex);

    close(node_socket);
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

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
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
        *new_socket = accept(server_fd, (struct sockaddr *)&address, &addrlen);
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
    }

    close(server_fd);
    return 0;
}