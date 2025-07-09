# DSR Local Test Without Crazyflie Support

## 🌟 Overview
This project provides a local test environment for modified ranging without the need for Crazyflie hardware support. It consists of a drone simulator and a control center, allowing you to simulate the communication and ranging process between drones. Additionally, there is a Python script for data processing.

## 📋 Prerequisites
- **Operating System**: A Unix-like operating system (e.g., Linux, macOS)
- **Compiler**: GCC compiler for C code compilation
- **Python**: Python 3 installed for data processing

## 🔨 Building the Project

### Build All Components
To build both the drone simulator and the control center, run the following command in the project root directory:
```sh
make
```

### Build Specific Components
- **Build only the drone simulator**:
```sh
make drone
```
- **Build only the control center**:
```sh
make center_control
```

## 🚀 Running the System

### Start the Control Center
After building the project, start the control center by running the following command:
```sh
./center_control
```
The control center will start listening on the specified port (`8888` by default) and wait for drone connections.

### Start the Drone Simulator
To start the drone simulator, run the following command, replacing `<port>` with the IP address of the control center and `<drone_name>` with a unique identifier for the drone:
```sh
./drone <port> <drone_name>
```
**Example**:
```sh
./drone 127.0.0.1 drone1
./drone 127.0.0.1 drone2
```
You can start multiple drone simulators to simulate a multi - drone environment.

### Process Data
After the simulation is completed, you can use the Python script to process the data. Run the following command:
```sh
python3 data_process.py
```
This script will generate a plot of the adjusted data and save it as `data.png`.

## 📁 Code Structure
- **`local_host`**: Initializes the local host, including address, base time, location, and velocity, providing the system’s basic configuration.
- **`ranging_struct`**: Defines all necessary types and data structures used throughout the project.
- **`ranging_protocol`**: Core module for ranging, responsible for table initialization, updating, running the ranging algorithm, and handling related logic.
- **`socket_frame`**: Implements the socket communication framework, including message structures and constants, enabling data exchange between the drone simulator and the control center.
- **`drone`**: Simulates drone behavior, handling message sending, receiving, and processing using the socket framework.
- **`center_control`**: Manages the control center, including drone connections, message broadcasting, and drone list management, overseeing the entire system.
- **`data_process`**: Processes and visualizes simulation data by reading files, calculating offsets, and generating plots to help analyze algorithm performance.
- **`lock`**: Provides a thread-safe locking mechanism for message processing to ensure data consistency in multithreaded environments.
- **`debug`**: Defines the `DEBUG_PRINT` function to assist with debugging and development.

## ⚙️ Process

### 1. Kind of Message
- `Tx, Rx, Rx_NO`

### 2. State Transition
```plaintext
            +------+------+------+------+------+            
            |  Tb  |  Rp  |      |      |      |   ----+
    +--->   +------+------+------+------+------+       |   (2) Rx(impossible) / (3) Rx_NO
    |       |  Rb  |  Tp  |  Rr  |      |      |   <---+
    |       +------+------+------+------+------+   ----+        
    |                                                  |   (1) Tx
    |       +------+------+------+------+------+   <---+                                +------+------+------+------+------+
    |       |  Tb  |  Rp  |      |      |      |   ----+----------------------------+   |  Tb  |  Rp  | [Tr] |      |      |
    |       +------+------+------+------+------+       |   (1) Tx       (3) Rx_NO   |   +------+------+------+------+------+
    |       |  Rb  |  Tp  |  Rr  | [Tf] |      |   <---+----------------------------+   |  Rb  |  Tp  |  Rr  | [Tf] | [Re] |
    |       +------+------+------+------+------+   ----+                                +------+------+------+------+------+
    |                                                  |   (2) Rx
    |       +------+------+------+------+------+   <---+
    |       |  Tb  |  Rp  | [Tr] | [Rf] |      |    
    +----   +------+------+------+------+------+
            |  Rb  |  Tp  |  Rr  | [Tf] | [Re] |
            +------+------+------+------+------+
```

### 3. Initialize
- **Normal**: 
`{Tp, Rp, Tr, Rr, Tf, Rf}`
```plaintext
+------+------+------+------+------+
| -Tr- | -Rf- |      |      |      |
+------+------+------+------+------+
| -Rr- | -Tf- | -Re- |      |      |
+------+------+------+------+------+
```
- **Initialization**
```plaintext
+------+------+------+------+------+
| -Tr- | -Rf- |      |      |      |
+------+------+------+------+------+
| -Rr- | -Tf- | -Re- |      |      |
+------+------+------+------+------+
```
- **Orderliness**
  - **Backup** 
    -  `{Tp, Rp, backupTr, backupRr, Tf, Rf}`
  - **Extra Node**
    1. `Tr < Rp  =>  {ETp, ERp, Tr, Rr, Tf, Rf}`
    2. `Tf < Rr  =>  {Tb, Rb, Tp, Rp, Tr, Rr}`
```plaintext
Back up:
Tb           Rp     Tr           Rf
    
   Rb     Tp           Tf     Rr
+------+------+------+------+------+
|-bkTr-| -Rf- |      |      |      |
+------+------+------+------+------+
|-bkRr-| -Tf- | -Re- |      |      |
+------+------+------+------+------+

Extra Node(Tr < Rp):
Tb           Tr     Rp           Rf                 

   Rb     Tp           Rr     Tf                    
+------+------+------+------+------+                
| -Tr- | -Rf- |      |      |      |                
+------+------+------+------+------+               
| -Rr- | -Tf- | -Re- |      |      |                
+------+------+------+------+------+               

Extra Node(Tf < Rr):
Tb           Rp     Tr           Rf
   Rb     Tp           Tf     Rr          
+------+------+------+------+------+
|  Tb  | -Rf- |      |      |      |
+------+------+------+------+------+
|  Rb  | -Tf- | -Re- |      |      |
+------+------+------+------+------+
```
- **Lossing Packet**
  1. `Tf and Rf are full  =>  {ETp, ERp, Tb, Rb, Tf, Rf}`
  2. `Tr and Rr are full  =>  {Tb, Rb, Tp, Rp, Tr, Rr}`
```plaintext
+------+------+------+------+------+
|  Tb  |  Rp  |      |      |      |
+------+------+------+------+------+
|  Rb  |  Tp  |  Rr  |      |      |
+------+------+------+------+------+
```

### 4. Calculate
- **Normal**: 
`{Tb, Rb, Tp, Rp, Tr, Rr, Tf, Rf}`
```plaintext
+------+------+------+------+------+
| -Tr- | -Rf- |      |      |      |
+------+------+------+------+------+
| -Rr- | -Tf- | -Re- |      |      |
+------+------+------+------+------+
```
- **Orderliness**
  - **Backup**
    - `{Tb, Rb, Tp, Rp, backupTr, backupRr, Tf, Rf}`
  - **Extra Node**
    1. `Tr < Rp  =>  {ETp, ERp, Tr, Rr, Tf, Rf}`
    2. `Tf < Rr  =>  {Tb, Rb, Tp, Rp, Tr, Rr}`
```plaintext
Back up:
Tb           Rp     Tr           Rf
    
   Rb     Tp           Tf     Rr
+------+------+------+------+------+
|-bkTr-| -Rf- |      |      |      |
+------+------+------+------+------+
|-bkRr-| -Tf- | -Re- |      |      |
+------+------+------+------+------+

Extra Node(Tr < Rp):
Tb           Tr     Rp           Rf

   Rb     Tp           Rr     Tf
+------+------+------+------+------+
|  Tr  |  Rf  |      |      |      |
+------+------+------+------+------+
|  Rr  |  Tf  |  Re  |      |      |
+------+------+------+------+------+

Extra Node(Tf < Rr):
Tb           Rp     Tr           Rf
    
   Rb     Tp           Tf     Rr
+------+------+------+------+------+
|  Tb  |  Rf  |      |      |      |
+------+------+------+------+------+
|  Rb  |  Tf  |  Re  |      |      |
+------+------+------+------+------+
```
- **Lossing Packet**
  - `Tf and Rf are full  =>  {ETp, ERp, Tb, Rb, Tf, Rf}`
  - `Tr and Rr are full  =>  {Tb, Rb, Tp, Rp, Tr, Rr}`
```plaintext
+------+------+------+------+------+
|  Tb  |  Rp  |      |      |      |
+------+------+------+------+------+
|  Rb  |  Tp  |  Rr  |      |      |
+------+------+------+------+------+
```

## ⚙️ Configuration
The project can be configured through preprocessor directives in the source code. For example, you can enable or disable features such as dynamic ranging frequency, packet loss simulation, and position sending by defining or undefining the corresponding macros in the source files.

## 🛠️ Troubleshooting
- **Compilation errors**: Make sure you have the necessary dependencies installed and that your compiler is configured correctly.
- **Connection issues**: Check that the IP address and port specified when starting the drone simulator match the settings of the control center.
- **Data processing errors**: Ensure that the input data format is correct and that Python 3 and the required libraries (e.g., `matplotlib, numpy`) are installed.