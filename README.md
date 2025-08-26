# Drone Simulation System

## Project Introduction

This project is a drone communication simulation system that simulates communication scenarios between multiple drones through a central controller (`center`) and drone nodes (`drone`). The system focuses on data processing and protocol implementation, providing convenient with convenient data processing functions and supports multiple ranging modes, which can be used to test and evaluate communication and ranging algorithms for drone swarms.

> Note: This project only includes the protocol processing part, and does not contain sniffer hardware-related code or firmware code that can be directly implanted into drones.

## Core Functions

- Simulate communication scenarios between multiple drone nodes
- Process raw data collected by sniffer and convert it into a format usable for simulation
- Support four ranging modes, which can be flexibly configured according to requirements (**only one of the four modes can be enabled**)
- Provide data evaluation and parameter optimization tools
- Implement node management and message forwarding through a central controller

## Supported Ranging Modes

The system supports the following four ranging mode configurations in `support.h` (**only one mode can be enabled at a time**):

- `IEEE_802_15_4Z`: Ranging mode based on the 802.15.4z standard
- `SWARM_RANGING_V1`: Swarm ranging version 1
- `SWARM_RANGING_V2`: Swarm ranging version 2
- `DYNAMIC_RANGING_MODE`: Dynamic ranging mode

You can check the currently enabled ranging mode using the `make mode` command.

## Quick Start

### 1. Data Preparation and Processing

#### Obtain the Data Processing Script
The data processing script `data_process.py` is located in the drone firmware project, which can be obtained from the following address:
[https://github.com/SEU-NetSI/crazyflie-firmware-adhocuwb](https://github.com/SEU-NetSI/crazyflie-firmware-adhocuwb)

#### Data Processing Steps

1. Place the raw data file collected by the sniffer into the `sniffer/data/` directory
2. Edit `data_process.py` and modify the following parameters:
   - `FILE_NAME`: Name of the sniffer data file
   - `DRONE_NUM`: Number of drones in the simulation
   - `INTEGRITY_FILTER`: Whether to enable integrity check (True/False)

3. Run the data processing script:
   ```bash
   python3 data_process.py
   ```

4. After the script executes, it will:
   - Read the original CSV file from `sniffer/data/`
   - Generate a processed CSV file in the `data/` directory with the following format:
     ```
     src_addr,msg_seq,filter,Tx_time,Rx0_addr,Rx0_time,Rx1_addr,Rx1_time,...
     ```
   - Output processing statistics (total number of lines processed and valid records)

### 2. Configure System Parameters

1. Open the `support.h` file
2. Select the desired ranging mode (**uncomment only one macro**):
   ```c
   #define IEEE_802_15_4Z                       // Enable 802.15.4z mode
   // #define SWARM_RANGING_V1                  // Enable swarm ranging version 1
   // #define SWARM_RANGING_V2                  // Enable swarm ranging version 2
   // #define DYNAMIC_RANGING_MODE              // Enable dynamic ranging mode
   ```

3. Modify core parameters according to simulation requirements:
   ```c
   #define     NODES_NUM               2        // Total number of drones in the system
   #define     PACKET_LOSS             0        // Simulated communication link packet loss rate
   #define     RANGING_PERIOD_RATE     1        // Multiplier for ranging data transmission period
   ```

### 3. Compile the Program

1. Check the current compilation mode (confirm the enabled ranging mode):
   ```bash
   make mode
   ```

2. Execute the compilation command in the project root directory:
   ```bash
   make
   ```

3. After successful compilation, two executable files will be generated:
   - `center`: Central controller program
   - `drone`: Drone node simulation program

### 4. Run the System

#### Start the Central Controller
```bash
./center
```
- The controller will start on the default port 8888
- After starting, it will wait for all configured drone nodes (`NODES_NUM`) to connect

#### Start Drone Nodes
Open a new terminal for each drone node and execute:
```bash
./drone <drone_address>
```
- `<drone_address>`: Unique numeric address for the drone (e.g., 1, 2, etc.)
- The address must be consistent with the settings on cfclient

#### System Operation Process
1. After all drone nodes are successfully connected, the controller starts broadcasting flight logs
2. Drones receive log data and exchange ranging messages through the controller
3. The controller is responsible for forwarding ranging messages between all connected drones

## Data Analysis Tools

The project provides two Python scripts for result analysis:

1. **evaluation.py**
   - Function: Integrates processed data from four modes, aligns with VICON timestamps, and performs evaluation
   - Purpose: Analyze the performance of different ranging modes

2. **optimize.py**
   - Function: Reads data from ranging result files and adjusts compensation coefficients to optimize ranging accuracy
   - Note: Ensure `COMPENSATE_ENABLE` is turned off when using

## System Components

### Central Controller (center)

- **Node Management**: Tracks all connected drones using the `Drone_Node_Set_t` structure
- **Flight Log Broadcasting**: Reads processed CSV data and sends time-stamped messages to corresponding drones
- **Message Routing**: Uses a broadcast mechanism to forward ranging messages between drones
- **Concurrent Processing**: Uses pthread mutexes to ensure thread safety of shared data, creating separate processing threads for each drone connection

### Drone Node (drone)

- **Communication Management**: Maintains a continuous connection with the central controller
- **Message Processing**:
  - Receives flight log data (including Tx/Rx timestamps)
  - Processes received ranging messages using `processDSRMessage()`
  - Generates response messages using `generateDSRMessage()`
- **Callback Mechanism**: Handles subsequent operations triggered by messages through `TxCallBack()` and `RxCallBack()`

## Data Flow

1. Sniffer data → Processed by `data_process.py` → Generate standardized CSV
2. Controller reads CSV data → Sends Tx/Rx timestamps to corresponding drones
3. Drones generate ranging messages → Send to central controller
4. Controller broadcasts ranging messages to all drone nodes
5. Drone nodes process received ranging messages with timestamps

## Notes

- All drone addresses must be set to non-zero values
- Drone addresses must be unique and match the addresses in the processed data file
- When using `optimize.py`, ensure `COMPENSATE_ENABLE` is turned off
- Before data processing, confirm that the original sniffer data file exists in the `sniffer/data/` directory
- Only one of the four ranging modes can be enabled, and you can check the currently enabled mode using the `make mode` command
- You can use `Ctrl+C` to terminate any running component

## Troubleshooting

- **Connection Issues**: Check if `CENTER_PORT` (default 8888) is blocked, and ensure the controller is running before starting drones
- **Data Mismatch**: Confirm that `NODES_NUM` is consistent with `drone_num`, and that addresses match the data file
- **Compilation Errors**: Ensure all dependencies are correctly installed, and check if multiple ranging modes are enabled simultaneously
- **Mode Errors**: If you need to switch ranging modes, recompile after modifying `support.h` and confirm using `make mode`
- **Timeout Errors**: Verify that the controller is broadcasting data and that the network connection is normal