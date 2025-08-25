# Drone Simulation System User Guide

## Overview
This system simulates drone communication scenarios through a central controller (`center`) and drone nodes (`drone`). It processes sniffer data into a usable format, then coordinates communication between nodes by broadcasting flight logs and forwarding ranging messages. It supports two ranging modes: `dynamic_swarm_ranging` and `swarm_ranging`, which can be configured according to actual needs.

## Process Steps

### 1. Data Processing
1. Open `data_process.py`
2. Modify parameters according to your data:
   - `FILE_NAME`: Name of the sniffer data file (located in `../sniffer/data/` directory)
   - `DRONE_NUM`: Actual number of drones in the simulation
   - `INTEGRITY_FILTER`: Whether to perform integrity check
3. Run the data processing script:
   ```bash
   python3 data_process.py
   ```
4. The script will:
   - Read the original CSV from `../sniffer/data/`
   - Generate a processed CSV in `./data/` with format:
     ```
     src_addr,msg_seq,filter,Tx_time,Rx0_addr,Rx0_time,Rx1_addr,Rx1_time,...
     ```
   - Output processing statistics (total lines processed and valid records generated)

### 2. Configuration Parameter Modification
1. Open `frame.h`
2. Modify the `NODES_NUM` macro to match `DRONE_NUM` from step 1
   ```c
   #define     NODES_NUM   <number of drones>
   ```
4. Other key configurable parameters in `frame.h`(No changes are necessary generally):
   - `CENTER_PORT`: Communication port (default: 8888)
   - `READ_PERIOD`: Interval between log broadcasts (default: 200ms)
   - `ADDR_SIZE`: Maximum length of node addresses (default: 20)
---
5. Open `support`
6. Select the appropriate mode based on the protocol: `SWARM_RANGING_MODE` or `DYNAMIC_SWARM_RANGING_MODE`

### 3. Compile the Program
1. Use `make mode` to check whether the current compilation mode matches the expected one:
```bash
make mode
```
2. Execute the compilation command in the project root directory:
```bash
make
```
3. Two executable files will be generated:
- `center`: Central controller program
- `drone`: Drone node simulation program

### 4. Run the System
1. **Start the central controller**:
   ```bash
   ./center
   ```
   - The controller will start on `CENTER_PORT` (default 8888)
   - It will wait until all `NODES_NUM` drones connect

2. **Start drone nodes** (open a new terminal for each node):
   ```bash
   ./drone <drone_address>
   ```
   - `<drone_address>`: Unique numeric address for the drone (e.g., 1, 2, etc.), the address must be consistent with the setting on cfclient.

3. **System operation**:
   - Once all drones connect, the controller starts broadcasting flight logs
   - Drones receive log data and exchange ranging messages through the controller
   - The controller forwards ranging messages between all connected drones

### 5. Script
   Two Python scripts are provided for analyzing the results:
1. **data_process.py**
   This script reads data collected by the sniffer and organizes it into a format suitable for simulation, saving the results in the simulation/data directory.

2. **evaluation.py**
   This script integrates the processed SR and DSR data, aligns them with the VICON timestamps, and then evaluates the data.

3. **optimize.py**
   This script reads data from ranging log.csv and adjusts the compensation coefficient appropriately to optimize ranging accuracy, make sure COMPENSATE_ENABLE is closed.

4. **vicon.py**
   This script receives rigid body motion data in real-time from the host of the Vicon motion capture system via a network connection. 

## Key Components

### Center Program
- **Node Management**: Tracks connected drones using a `Drone_Node_Set_t` structure
- **Flight Log Broadcasting**: Reads processed CSV data and sends time-stamped messages to corresponding drones
- **Message Routing**: Forwards ranging messages between drones using a broadcast mechanism
- **Concurrency**: Uses pthread mutexes for thread-safe access to shared data and handles each drone connection in a separate thread

### Drone Program
- **Communication**: Maintains a connection to the central controller
- **Message Handling**: 
  - Receives flight log data (Tx/Rx timestamps)
  - Processes ranging messages using `processDSRMessage()`
  - Generates responses using `generateDSRMessage()`
- **Callbacks**: Uses `TxCallBack()` and `RxCallBack()` to handle message processing

### Data Flow
1. Sniffer data → `data_process.py` → Processed CSV
2. Controller reads CSV → Sends Tx/Rx timestamps to respective drones
3. Drones generate ranging messages → Sent to controller
4. Controller broadcasts ranging messages to all drones
5. Drones process received ranging messages with timestamps

## Notes
- Ensure that all drone addresses are set to values other than 0
- Ensure COMPENSATE_ENABLE is closed while using optimize.py
- Ensure the original sniffer data file exists in `../sniffer/data/` before processing
- Drone addresses must be unique and match those in the processed data file
- Use `Ctrl+C` to terminate any running component

## Troubleshooting
- **Connection Issues**: Verify `CENTER_PORT` is not blocked and controller is running before starting drones
- **Data Mismatch**: Check that `NODES_NUM` matches `drone_num` and addresses match data file
- **Compilation Errors**: Ensure all dependencies are installed
- **Timeout Errors**: Verify controller is broadcasting data and network connectivity is working