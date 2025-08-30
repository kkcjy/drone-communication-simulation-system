# C Language UWB Sniffer Program User Guide

## Overview
This program uses a drone connected to a computer as a sniffer to collect UWB ranging data from other drones and record it into CSV files. It supports two ranging modes: `dynamic_swarm_ranging` and `swarm_ranging`, which can be configured according to actual needs.

## Prerequisites

1. **Hardware Connection**
   - Connect the drone acting as the sniffer to the computer via a USB data cable

2. **Firmware Configuration**
   - Open the `adhocuwb_init.h` file
   - Ensure the `SNIFFER_ENABLE` macro is defined (uncomment or add this macro definition)

3. **Firmware Compilation and Flashing**
   - Compile the drone firmware (including sniffer functionality)
   - Flash the compiled firmware onto the drone that will serve as the sniffer
   - Power on the drone and ensure it operates normally

## Computer-Side Program Compilation

1. **Configure Makefile**
   
   Modify `CFLAGS` and `LDFLAGS` according to your operating system:
     - **Linux System**: Use the default configuration
     - **macOS System**: Comment out Linux-related configurations and uncomment macOS configurations

2. **Compile the Program**
   ```bash
   make
   ```

## Running the Program

1. **Start the Sniffer Program**
   ```bash
   ./sniffer
   ```

2. **Program Operation Description**
   - After startup, the program automatically creates a CSV file named `sniffer_Log.csv`
   - Data is written to the CSV file in real-time, including timestamp, source address, sequence number, ranging time, etc.
   - Press `Ctrl+C` to terminate the program, which will automatically save data before exiting

## Mode Configuration

1. **Mode Selection**
   - Select the ranging mode in `sniffer.c` via macro definitions:
     
     `#define SWARM_RANGING_MODE`
     
     `#define DYNAMIC_SWARM_RANGING_MODE`
   - Note: Only one mode can be selected at a time; the other must be commented out

2. **Listening Quantity Configuration**
   - Modify the `LISTENED_DRONES` macro definition in `sniffer.c` to match the actual number of drones to be monitored:
     ```c
     #define LISTENED_DRONES ...
     ```

## Data File
- Data files are stored in the `data` directory, with filenames based on timestamps
- CSV files contain the following main fields:
  - Sniffer reception time, source address, message sequence number, message length, filter
  - Transmission time and sequence number of each sending node
  - Address, reception time, and sequence number of each receiving node

## Cleaning Compiled Files
To recompile, first clean existing compiled files:
```bash
make clean
```

## Troubleshooting
- If the program fails to find the USB device, check:
  - USB connection stability
  - Correctness of `VENDOR_ID` and `PRODUCT_ID` in `sniffer.c`
  - Proper USB driver installation
- If bulk transfer errors occur, verify:
  - Drone firmware is correctly flashed and running
  - USB cable integrity
  - Sufficient permissions for USB device access

## Notes
- If real-time actual data is needed, the `vicon.py` script should be used.
- Ensure that all drone addresses are set to values other than 0
- Ensure the `data` directory exists in the program folder before running, or the program may fail to create log files
- The number of monitored drones must not exceed the maximum limits defined by `RANGING_MAX_BODY_UNIT` (for swarm mode) or `MESSAGE_BODYUNIT_SIZE` (for dynamic mode)
- The CSV file format automatically adjusts based on the selected ranging mode