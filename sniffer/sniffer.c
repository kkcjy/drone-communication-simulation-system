#include <libusb-1.0/libusb.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../adhocuwb_dynamic_swarm_ranging.h"


#define     LISTENED_DRONES 2
#define     FILENAME_SIZE   32
#define     MAX_PACKET_SIZE 256
#define     MAGIC_MATCH     0xBB88
#define     VENDOR_ID       0x0483
#define     PRODUCT_ID      0x5740


char filename[FILENAME_SIZE];
volatile sig_atomic_t keep_running = 1;     // used for the interruption caused by Ctrl+C
int ignore_lines = 30;
int listen_lines = 0;


typedef union {
    uint8_t raw[18];
    struct {
        uint32_t magic;
        uint16_t senderAddress;
        uint16_t seqNumber;
        uint16_t msgLength;
        uint64_t rxTime;
    } __attribute__((packed));
} __attribute__((packed)) Sniffer_Meta_t;


void handle_sigint(int sigint) {
    keep_running = 0;
}

void generate_filename(char *buffer, size_t buffer_size) {
    time_t rawtime;
    struct tm *curtime;
    time(&rawtime);
    curtime = localtime(&rawtime);
    strftime(buffer, buffer_size, "../data/raw_sensor_data.csv", curtime);
}

uint64_t get_system_time() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
}

void fprintRangingMessaage(FILE* file, libusb_device_handle *device_handle) {
    uint8_t buffer[MAX_PACKET_SIZE];
    uint8_t payload[MAX_PACKET_SIZE];
    uint8_t endpoint = 0x81; 
    int transferred;

    while (keep_running) {
        // first reception
        int response = libusb_bulk_transfer(device_handle, endpoint, buffer, MAX_PACKET_SIZE, &transferred, 5000);
        
        // success
        if(response == 0 && transferred <= MAX_PACKET_SIZE) {
            listen_lines++;
            if(listen_lines == ignore_lines) {
                fclose(file);
                file = fopen(filename, "w");
                if (file == NULL) {
                    perror("Open file error");
                    return;
                }

                fprintf(file, "system_time,src_addr,msg_seq,msg_len,filter,");
                for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
                    fprintf(file, "Tx%d_time,Tx%d_seq,", i, i);
                }
                for(int i = 0; i < MESSAGE_BODYUNIT_SIZE && i < LISTENED_DRONES - 1; i++) {
                    fprintf(file, "Rx%d_addr,Rx%d_time,Rx%d_seq,", i, i, i);
                }
                fprintf(file, "\n");

                printf("ignore %d lines...\n", listen_lines);
            }
            else if(listen_lines > ignore_lines && (listen_lines - ignore_lines) % 10 == 0) {
                printf("listen %d lines...\n", listen_lines - ignore_lines);
            }

            Sniffer_Meta_t *meta = (Sniffer_Meta_t *)buffer;
            uint64_t system_time = get_system_time();
            fprintf(file, "%lu,", system_time);

            if(meta->magic == MAGIC_MATCH && meta->msgLength <= 256) {
                response = libusb_bulk_transfer(device_handle, endpoint, payload, meta->msgLength, &transferred, 5000);
                if (response == 0 && transferred == meta->msgLength) {
                    Ranging_Message_t rangingMessage;
                    memcpy(&rangingMessage, payload, sizeof(Ranging_Message_t));

                    // header
                    fprintf(file, "%u,%u,%u,%u,", rangingMessage.header.srcAddress, rangingMessage.header.msgSequence, rangingMessage.header.msgLength, rangingMessage.header.filter);

                    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
                        fprintf(file, "%lu,%u,", rangingMessage.header.Txtimestamps[i].timestamp.full % UWB_MAX_TIMESTAMP, rangingMessage.header.Txtimestamps[i].seqNumber);
                    }

                    // bodyunit
                    for(int i = 0; i < MESSAGE_BODYUNIT_SIZE && i < LISTENED_DRONES - 1; i++) {
                        fprintf(file, "%u,%lu,%u,", rangingMessage.bodyUnits[i].address, rangingMessage.bodyUnits[i].timestamp.full % UWB_MAX_TIMESTAMP, rangingMessage.bodyUnits[i].seqNumber);
                    }
                    fprintf(file, "\n");
                }
                // fail
                else if(transferred > MAX_PACKET_SIZE) {
                    printf("MAX_PACKET_SIZE is small\n");
                    exit(EXIT_FAILURE);
                }
                else {
                    printf("Bulk transfer failed: %s\n", libusb_strerror(response));
                    exit(EXIT_FAILURE);
                }
            }
        }
        // fail
        else if(transferred > MAX_PACKET_SIZE) {
            printf("MAX_PACKET_SIZE is small\n");
            exit(EXIT_FAILURE);
        }
        else {
            printf("Bulk transfer failed: %s\n", libusb_strerror(response));
            exit(EXIT_FAILURE);
        }
    }
}

int main() {
    libusb_device_handle *device_handle = NULL;
    libusb_context *context = NULL;
    FILE *log_file;

    signal(SIGINT, handle_sigint);

    int response = libusb_init(&context);
    if (response < 0) {
        fprintf(stderr, "libusb init error\n");
        return 1;
    }

    device_handle = libusb_open_device_with_vid_pid(context, VENDOR_ID, PRODUCT_ID);
    if (!device_handle) {
        fprintf(stderr, "cannot find USB device\n");
        return 1;
    }

    libusb_set_auto_detach_kernel_driver(device_handle, 1);
    libusb_claim_interface(device_handle, 0);

    generate_filename(filename, sizeof(filename));
    log_file = fopen(filename, "w");
    if (!log_file) {
        perror("open log file");
        return 1;
    }

    /* Populating message details */
    fprintRangingMessaage(log_file, device_handle);

    fclose(log_file);
    libusb_release_interface(device_handle, 0);
    libusb_close(device_handle);
    libusb_exit(context);

    printf("Logging finished. Data saved to %s\n", filename);

    return 0;
}