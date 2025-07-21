#include "queue_task.h"


extern Ranging_Table_Set_t* rangingTableSet;     
extern Local_Host_t* localHost;


void initQueueTaskLock(QueueTaskLock_t *queue) {
    pthread_mutex_init(&queue->mutex, NULL);
    pthread_mutex_init(&queue->countMutex, NULL);
    for(int i = 0; i < QUEUE_TASK_LENGTH; i++) {
        queue->queueTask[i].data = NULL;
        queue->queueTask[i].data_size = 0;
    }
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

// generate data and Tx
Time_t QueueTaskTx(QueueTaskLock_t *queue, int msgSize, SendFunction send_to_center, int centerSocket, const char* droneId) {
    pthread_mutex_lock(&queue->mutex);

    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)malloc(msgSize);
    if (!rangingMessage) {
        DEBUG_PRINT("[QueueTaskTx]: malloc failed\n");
        pthread_mutex_unlock(&queue->mutex);
        return NULL_TIMESTAMP;
    }

    Time_t timeDelay = generateMessage(rangingMessage);

    pthread_mutex_unlock(&queue->mutex);

    if (send_to_center) {
        send_to_center(centerSocket, droneId, rangingMessage);
    }

    free(rangingMessage);
    rangingMessage = NULL; 

    return timeDelay;
}

// Rx
void QueueTaskRx(QueueTaskLock_t *queue, void *data, size_t data_size) {
    if(data_size == 0) {
        DEBUG_PRINT("[QueueTaskRx]: receive empty message\n");
        return;
    }

    pthread_mutex_lock(&queue->countMutex);
    if(queue->count == QUEUE_TASK_LENGTH) {
        DEBUG_PRINT("[QueueTaskRx]: queueTask is full, message missed\n");
        pthread_mutex_unlock(&queue->countMutex);
        return;
    }

    // receive data
    queue->queueTask[queue->tail].data = malloc(data_size);
    if (queue->queueTask[queue->tail].data == NULL) {
        DEBUG_PRINT("[QueueTaskRx]: malloc failed\n");
        pthread_mutex_unlock(&queue->countMutex);
        return;
    }
    queue->queueTask[queue->tail].data_size = data_size;
    memcpy(queue->queueTask[queue->tail].data, data, data_size);

    queue->tail = (queue->tail + 1) % QUEUE_TASK_LENGTH;
    queue->count++;

    pthread_mutex_unlock(&queue->countMutex);
}

// process
void processFromQueue(QueueTaskLock_t *queue) {
GET:

    pthread_mutex_lock(&queue->mutex);
    pthread_mutex_lock(&queue->countMutex);
    if(queue->count != 0) {
        goto PROCESS;
    }

    pthread_mutex_unlock(&queue->mutex);
    pthread_mutex_unlock(&queue->countMutex);
    local_sleep(10); 
    goto GET;

PROCESS:
    Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo = (Ranging_Message_With_Additional_Info_t*)queue->queueTask[queue->head].data;

    processMessage(rangingMessageWithAdditionalInfo);

    free(queue->queueTask[queue->head].data);
    queue->queueTask[queue->head].data = NULL;
    queue->head = (queue->head + 1) % QUEUE_TASK_LENGTH;
    queue->count--;

    pthread_mutex_unlock(&queue->countMutex);

    pthread_mutex_unlock(&queue->mutex);
}