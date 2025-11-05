# Drone Communication Simulation System

## Project Overview

This project is a communication simulation system designed for multi-drone collaborative scenarios, leveraging a "Central Controller (center) + Drone Node (drone)" architecture to accurately model communication interactions among multiple drones. The core value lies in addressing the "black-box nature" of drone communication—by collecting real-time transmission packets and integrating high-precision positioning data (with inter-node distance information) from the VICON system, it fully reproduces the entire "transmit-receive" communication process, making every step of the communication chain traceable and analyzable.

The system features lightweight data processing capabilities, supports multiple mainstream ranging protocols, and can be directly used for functional testing and performance evaluation of drone swarm communication algorithms and ranging algorithms. Specifically developed to provide supporting simulation for the open-source project [crazyflie-firmware-adhocuwb](https://github.com/SEU-NetSI/crazyflie-firmware-adhocuwb), this system requires synchronous adaptation and maintenance if the communication protocol of the open-source project is updated, ensuring consistency between simulation and real hardware scenarios.

> **Note**: This project includes only core modules for data collection and protocol processing to realize simulation and reproduction of communication processes. It does not contain Sniffer hardware driver-related code or firmware code directly flashable to drone hardware.


## Core Features

- **Multi-node Communication Scenario Simulation**: Constructs a distributed interaction architecture between the central controller and multiple drone nodes, reproducing communication topologies (e.g., star, mesh) of drone swarms in real environments with support for custom node counts.
- **Raw Data Collection and Conversion**: Captures real-time communication packets from drones via Sniffer tools, automatically filters invalid data (e.g., interference frames from initial USB transmission), and converts it into a standard format parsable by the simulation system.
- **High-precision Distance Information Acquisition**: Deeply integrates with the VICON positioning system to collect real-time 3D coordinates of drones, automatically calculating linear distances between nodes to provide a "ground truth benchmark" for ranging algorithm evaluation.
- **Flexible Switching of Multiple Ranging Modes**: Embeds 4 ranging modes (only one enabled at a time), supporting quick switching via configuration files to meet testing needs of different protocols and scenarios.
- **Data Evaluation and Parameter Optimization**: Provides an integrated analysis toolchain supporting ranging error calculation, performance comparison, and automatic adjustment of compensation coefficients to facilitate algorithm iteration.
- **Node Management and Message Forwarding**: The central controller uniformly manages connection status and data caching of all drone nodes, responsible for message routing and forwarding to ensure communication timing accuracy.


## Supported Ranging Modes

The system configures ranging modes via the `support.h` header file. Ensure only one macro definition is uncommented to avoid mode conflicts. Detailed mode descriptions are as follows:

| Ranging Mode                     | Macro Definition                  | Core Features & Application Scenarios                                                                                                                                                                                                 |
|----------------------------------|-----------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| IEEE 802.15.4z Standard Mode     | `#define IEEE_802_15_4Z`          | Complies with the IEEE 802.15.4z standard, supports UWB ranging, and provides basic high-precision ranging capabilities.                                                                                                               |
| Swarm Ranging V1                 | `#define SWARM_RANGING_V1`        | Optimized for low-latency requirements of drone swarms, uses a lightweight protocol, and is suitable for simple coordination scenarios in small-scale clusters.                                                                         |
| Swarm Ranging V2                 | `#define SWARM_RANGING_V2`        | An enhanced version of V1 with packet loss retransmission mechanisms and anti-interference capabilities, improving communication reliability in complex environments. Suitable for medium to large-scale clusters.                        |
| Dynamic Ranging Mode             | `#define DYNAMIC_RANGING`         | Specifically optimized for scenarios with rapid relative movement of drones. Dynamically adjusts parameters to ensure a 0% computational failure rate, guaranteeing stable ranging under high-speed motion conditions.                  |
| Compensated Dynamic Ranging Mode | `#define COMPENSATE_DYNAMIC_RANGING` | Builds upon dynamic ranging by integrating advanced motion compensation algorithms. Effectively reduces ranging lag errors caused by relative motion through prediction and real-time compensation. Ideal for high-speed drone formations and complex applications requiring precise collaborative control. |

**Mode Check Command**: Execute `make mode` in the project root directory to quickly confirm the currently enabled ranging mode.


## Simulation Workflow

### 1. Preparatory Step: Data Acquisition

Acquire two types of core data (positioning + packet data) and ensure global consistency of drone addresses (subsequent steps rely on address-based data association).

#### (1) Address Pre-configuration (Critical Prerequisite)
- In `cfclient` (drone configuration tool), set a unique numeric address (e.g., 1, 2, 3; **0 is prohibited**) for each drone.
- When creating drone rigid bodies in the VICON system, use naming consistent with `cfclient` addresses (e.g., 1, 2, 3) to ensure unambiguous data association.
- You need to modify the `mc = motioncapture.connect` section in the `vicon.py` script. Make sure that the Vicon system and the host connected to the sniffer are on the same local network.

#### (2) VICON Positioning Data Acquisition
- Run the `vicon.py` script, which first scans for drone rigid bodies in the environment and lists identified nodes (e.g., 1, 2, 3).
- After manual confirmation, the script records real-time coordinates and inter-node distances, generating `data/vicon.txt`.

#### (3) Sniffer Packet Data Acquisition
- Enter the `sniffer` folder and run the executable `sniffer` (compile `sniffer.c` first). The script initially ignores the first 30–50 packets to filter out USB transmission interference.
- After execution, a file `data/raw_sensor_data.csv` is generated, which contains the raw communication packets (source address, destination address, transmission timestamp, etc.).

### 2. Data Preparation and Processing

Convert raw data to a standard format readable by the simulation system using `data_process.py`:

#### (1) Parameter Configuration
Open `data_process.py` and modify:
- `DRONE_NUM`: Number of simulated drones (exclude the Sniffer device, count only communicating drones).

#### (2) Execute Data Processing
```bash
python3 data_process.py
```

#### (3) Processing Output
- Generated file: `simulation_dep.csv` in the `data/` directory with the format:
  ```plaintext
  src_addr,msg_seq,filter,Tx_time,Rx0_addr,Rx0_time,Rx1_addr,Rx1_time,...
  ```
- Statistics: Terminal outputs total lines and valid records (with complete Tx/Rx timestamps) for data validation.

### 3. System Parameter Configuration

Modify `support.h` for core configurations:

#### (1) Select Ranging Mode
Example for enabling IEEE 802.15.4z mode:
```c
#define IEEE_802_15_4Z                       // Enable 802.15.4z mode
// #define SWARM_RANGING_V1                  // Enable Swarm Ranging V1
// #define SWARM_RANGING_V2                  // Enable Swarm Ranging V2
// #define DYNAMIC_RANGING              // Enable Dynamic Ranging Mode
// #define COMPENSATE_DYNAMIC_RANGING   // Enable Dynamic Ranging Mode
```

#### (2) Modify Core Simulation Parameters
```c
#define     NODES_NUM               2        // Total drones (must match DRONE_NUM)
#define     PACKET_LOSS             0        // Communication packet loss rate (0-100%, 0=none)
#define     RANGING_PERIOD_RATE     1        // Ranging data transmission period multiplier (1=default)
```

### 4. Program Compilation

#### (1) Verify Current Mode
```bash
make mode
```
Output example: `Current Ranging Mode: IEEE_802_15_4Z`

#### (2) Compile
```bash
make
```

#### (3) Compilation Results
- `center`: Central controller for node management and message forwarding.
- `drone`: Drone node simulator for single-drone communication behavior.

### 5. System Operation

#### (1) Start Central Controller
```bash
./center
```
Listens on port 8888, waits for drones with prompt: `Waiting for drones to connect...`

#### (2) Start Drone Nodes
Open a new terminal for each drone:
```bash
./drone <drone_address>
```
Example: `./drone 1`

#### (3) System Operation Logic
- Upon all nodes connecting, the controller reads `data/simulation_dep.csv`.
- Asynchronous processing divides into "task allocation" (log delivery) and "packet transmission" (message exchange via controller).
- Drones receive logs, generate ranging messages, send to the controller, which broadcasts to all nodes for multi-node communication simulation.


## Data Analysis Tools(evaluation.py)

### Core Function
- Integrates processed data from 5 modes, aligns with VICON ground truth, calculates errors (absolute, RMSE), latency, stability, and generates visual reports.
- Initial usage saves data to a CSV file, enabling subsequent direct access via the read function without maintaining log files(RESULT_REPRODUCTION = True).

### Key Parameters (Modify in script)
- `local_address`: Local drone address (e.g., 1, reference node).
- `neighbor_address`: Neighbor drone address (e.g., 2, target node).
- `leftbound`: Timestamp start (e.g., 1620000000).
- `rightbound`: Timestamp end (e.g., 1620001000).


## System Components

### 1. Central Controller (center)

| Module          | Core Function                                                                 |
|-----------------|-----------------------------------------------------------------------------|
| Node Management | Maintains node status via `Drone_Node_Set_t`, detects offline/reconnect events. |
| Log Broadcasting| Delivers `data/simulation_dep.csv` logs to corresponding drones by timestamp. |
| Message Routing | Forwards received ranging messages to all nodes with address-based filtering. |
| Concurrency     | Uses `pthread` with semaphores for asynchronous processing, avoiding data races. |

### 2. Drone Node (drone)

| Module          | Core Function                                                                 |
|-----------------|-----------------------------------------------------------------------------|
| Communication   | Establishes TCP connection with heartbeats, receives logs, sends ranging messages. |
| Message Processing | Parses logs (Tx/Rx timestamps) and neighbor messages via `processDSRMessage`; generates responses via `generateDSRMessage`. |
| Callbacks       | `TxCallBack()` records transmission timestamps; `RxCallBack()` calculates errors and applies compensation (if enabled). |


## Data Flow

1. **Acquisition**: Sniffer generates `raw_sensor_data.csv`; VICON generates `vicon.txt`.
2. **Processing**: `data_process.py` filters and converts data to `simulation_dep.csv`.
3. **Simulation**: Controller reads logs, drones exchange messages via the controller.
4. **Analysis**: `evaluation.py` compares results with VICON; `optimize.py` adjusts compensation coefficients.


## Notes

- **Address Uniqueness**: Non-zero, globally unique, consistent with `cfclient`, VICON, and data files.
- **Mode Configuration**: Recompile with `make clean && make` after modifying `support.h`; verify mode with `make mode`.
- **Tool Restrictions**: Comment `COMPENSATE_ENABLE` in `support.h` before running `optimize.py`.
- **Data Path**: Ensure `raw_sensor_data.csv` exists in `sniffer/data/`.
- **Process Termination**: Stop drones first with `Ctrl+C`, then the controller.


## Troubleshooting

### 1. Connection Issues

| Error            | Cause                          | Solution                                                                 |
|------------------|------------------------------|-------------------------------------------------------------------------|
| `drone: connect failed` | Controller not started or port blocked | Start `./center` first; check firewall rules for port 8888.              |
| `center: accept error` | Excessive nodes (>10)          | Reduce node count; increase file descriptors with `ulimit -n 65535`.      |
| `timeout: no data recv` | High network latency (>500ms)  | Switch to LAN; decrease `RANGING_PERIOD_RATE` to 0.5.                    |

### 2. Data Issues

| Error            | Troubleshooting Steps          | Solution                                                               |
|------------------|------------------------------|-------------------------------------------------------------------------|
| `data_process.py: error` | Invalid CSV format (missing fields) | Re-run Sniffer; check `raw_sensor_data.csv` header.                     |
| `distance error >1m` | VICON calibration error or invalid compensation | Re-calibrate VICON; run `optimize.py`; check `support.h` compensation.  |
| `msg_seq not continuous` | High packet loss or data collection interruption | Decrease `PACKET_LOSS` to 5%; ensure Sniffer runs without USB issues.    |

### 3. Compilation Issues

| Error Message        | Cause                          | Solution                                                                 |
|----------------------|------------------------------|-------------------------------------------------------------------------|
| `undefined reference to pthread` | Missing pthread library        | `sudo apt install libpthread-stubs0-dev`                                |
| `multiple definitions of mode` | Multiple modes enabled         | Check `support.h`; only one mode macro uncommented.                      |
| `make: *** No rule to make target` | Missing source files           | `git pull` or re-clone the project.                                      |

### 4. Advanced Debugging

- **Log Level**: Modify `LOG_LEVEL` in `config.h` (0=ERROR, 1=WARN, 2=INFO, 3=DEBUG).
- **Real-time Packet Capture**: `tcpdump -i lo port 8888 -w debug.pcap`.
- **Error Visualization**: `gnuplot -e "plot 'error_data.txt' using 1:2 with lines"`.