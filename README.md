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
- **`queue_task`**: Provides a thread-safe locking mechanism for message processing to ensure data consistency in multithreaded environments.
- **`ranging_local_support`**: Construct an environment and functions suitable for local execution.
- **`socket_frame`**: Implements the socket communication framework, including message structures and constants, enabling data exchange between the drone simulator and the control center.
- **`drone`**: Simulates drone behavior, handling message sending, receiving, and processing using the socket framework.
- **`center_control`**: Manages the control center, including drone connections, message broadcasting, and drone list management, overseeing the entire system.
- **`data_process`**: Processes and visualizes simulation data by reading files, calculating offsets, and generating plots to help analyze algorithm performance.

## ⚙️ State Transition

### 1. Kind of Message
- `TX, RX, RX_NO`

### 2. Simplified version
```plaintext
            +------+------+------+------+------+            
            |      |      |      |      |  S1  |   ----+
            +------+------+------+------+------+       |   (2) RX(impossible) / (3) RX_NO
            |      |      |      |      |      |   <---+
            +------+------+------+------+------+   ----+        
                                                       |   (1) TX
            +------+------+------+------+------+   <---+                                +------+------+------+------+------+
            |      |      |      |      |  S2  |   ----+----------------------------+   |      |      |      |      |  S3  |
            +------+------+------+------+------+       |   (1) TX       (3) RX_NO   |   +------+------+------+------+------+
            |      |      |      | [Tf] |      |   <---+----------------------------+   |      |      |      | [Tf] | [Re] |
            +------+------+------+------+------+   ----+                                +------+------+------+------+------+
                                                       |   (2) RX
            +------+------+------+------+------+   <---+
            |      |      |      | [Rf] |  S3  |
            +------+------+------+------+------+
            |      |      |      | [Tf] | [Re] |
            +------+------+------+------+------+   ----+
                                                       |   shift
            +------+------+------+------+------+   <---+         
            |      |  Rp  |      |      |  S4  |   ----+
            +------+------+------+------+------+       |   (2) RX(impossible) / (3) RX_NO
            |      |  Tp  |  Rr  |      |      |   <---+
            +------+------+------+------+------+   ----+
                                                       |   (1) TX
            +------+------+------+------+------+   <---+                                +------+------+------+------+------+
            |      |  Rp  |      |      |  S5  |   ----+----------------------------+   |      |  Rp  | [Tr] |      |  S6  |
            +------+------+------+------+------+       |   (1) TX       (3) RX_NO   |   +------+------+------+------+------+
            |      |  Tp  |  Rr  | [Tf] |      |   <---+----------------------------+   |      |  Tp  |  Rr  | [Tf] | [Re] |
            +------+------+------+------+------+   ----+                                +------+------+------+------+------+
                                                       |   (2) RX
            +------+------+------+------+------+   <---+
            |      |  Rp  | [Tr] | [Rf] |  S6  |
            +------+------+------+------+------+
            |      |  Tp  |  Rr  | [Tf] | [Re] |
            +------+------+------+------+------+   ----+
                                                       |   calculate and shift
            +------+------+------+------+------+   <---+
            |  Tb  |  Rp  |      |      |  S4  |   ----+
    +--->   +------+------+------+------+------+       |   (2) RX(impossible) / (3) RX_NO
    |       |  Rb  |  Tp  |  Rr  |      |      |   <---+
    |       +------+------+------+------+------+   ----+
    |                                                  |   (1) TX
    |       +------+------+------+------+------+   <---+                                +------+------+------+------+------+
    |       |  Tb  |  Rp  |      |      |  S5  |   ----+----------------------------+   |  Tb  |  Rp  | [Tr] |      |  S6  |
    |       +------+------+------+------+------+       |   (1) TX       (3) RX_NO   |   +------+------+------+------+------+
    |       |  Rb  |  Tp  |  Rr  | [Tf] |      |   <---+----------------------------+   |  Rb  |  Tp  |  Rr  | [Tf] | [Re] |
    |       +------+------+------+------+------+   ----+                                +------+------+------+------+------+
    |                                                  |   (2) RX
    |       +------+------+------+------+------+   <---+
    |       |  Tb  |  Rp  | [Tr] | [Rf] |  S6  |
    +----   +------+------+------+------+------+
            |  Rb  |  Tp  |  Rr  | [Tf] | [Re] |
            +------+------+------+------+------+
```
### 2. Full version
```
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |      | [Tr] | [Rf] |  S1  |            |      |      |      |      |      |      |  S1  |            |      |      |      |      | [Tr] |      |  S1  |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |      |      |      | [Re] |    <==>    |      |      |      |      |      |      |      |    <==>    |      |      |      |      |      |      | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
                                                                                          |
                                                                                          |  Tx
                                                                                          |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |      |      |      |  S2  |            |      |      |      |      |      |      |  S2  |            |      |      |      |      | [Tr] |      |  S2  |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |      |      | [Tf] |      |    <==>    |      |      |      |      |      | [Tf] |      |    <==>    |      |      |      |      |      |  Tf  | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
                                                                                          |
                                                                                          |  Rx
                                                                                          |
                                                              +------+------+------+------+------+------+------+
                                                              |      |      |      |      | [Tr] | [Rf] |  S3  |
                                                              +------+------+------+------+------+------+------+
                                                              |      |      |      |      |      |  Tf  | [Re] |
                                                              +------+------+------+------+------+------+------+
                                                              |             |             |
                                                              +------+------+------+------+
                                                                                          |
                                                                                          |  shift
                                                                                          |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |  Rp  | [Tr] | [Rf] | S4.1 |            |      |      |      |  Rp  |      |      | S4.1 |            |      |      |      |  Rp  | [Tr] |      | S4.1 |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |  Tp  |  Rr  |      | [Re] |    <==>    |      |      |      |  Tp  |  Rr  |      |      |    <==>    |      |      |      |  Tp  |  Rr  |      | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+    
                                                                                          |
                                                                                          |  Tx
                                                                                          |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  | [Tr] |      | S5.1 |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |  Tp  |  Rr  | [Tf] |      |    <==>    |      |      |      |  Tp  |  Rr  | [Tf] |      |    <==>    |      |      |      |  Tp  |  Rr  | [Tf] | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+    
                                                                                          |
                                                                                          |  Rx
                                                                                          |
                                                              +------+------+------+------+------+------+------+          
                                                              |      |      |      |  Rp  | [Tr] | [Rf] | S6.1 |         
                                                              +------+------+------+------+------+------+------+  
                                                              |      |      |      |  Tp  |  Rr  |  Tf  | [Re] |  
                                                              +------+------+------+------+------+------+------+         
                                                              |             |             |                               
                                                              +------+------+------+------+                             
                                                                                          |
                                                                                          |  shift
                                                                                          |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.2 |            |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S4.2 |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |    PTof     |                                 |             |    PTof     |                                 |             |    PTof     |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
                                                                                          |
                                                                                          |  Tx
                                                                                          |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      | ERp  |  Tb  |  Rp  |      |      | S5.2 |            |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S5.2 |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |    PTof     |                                 |             |    PTof     |                                 |             |    PTof     |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+    
                                                                                          |
                                                                                          |  Rx
                                                                                          |
                                                              +------+------+------+------+------+------+------+
                                                              |      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.2 |
                                                              +------+------+------+------+------+------+------+
                                                              |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |
                                                              +------+------+------+------+------+------+------+ 
                                                              |             |    PTof     |                            
                                                              +------+------+------+------+
                                                                                          |
                                                                                          |  shift                  +-------------------------------------------------------------+
                                                                                          |                         |                                                             |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+<---+       +------+------+------+------+------+------+------+    |
| ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S4.3 |    |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+    |
| ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |    |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                         |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+                         |
                                                                                          |                                                                                       |
                                                                                          |  Tx                                                                                   |
                                                                                          |                                                                                       |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
| ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S5.3 |    |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+    |
| ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |    |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|    EPTof    |    PTof     |                                 |             |    PTof     |                                 |    EPTof    |    PTof     |                         |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+                         |
                                                                                          |                                                                                       | 
                                                                                          |  Rx                                                                                   |
                                                                                          |                                                                                       |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              | ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.3 |                                                                  |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |                                                                  |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              |    EPTof    |    PTof     |                                                                                       |
                                                              +------+------+------+------+                                                                                       |
                                                                                          |  shift                                                                                |
                                                                                          +---------------------------------------------------------------------------------------+ 
```

## ⚙️ Configuration
The project can be configured through preprocessor directives in the source code. For example, you can enable or disable features such as dynamic ranging frequency, packet loss simulation, and position sending by defining or undefining the corresponding macros in the source files.

## 🛠️ Troubleshooting
- **Compilation errors**: Make sure you have the necessary dependencies installed and that your compiler is configured correctly.
- **Connection issues**: Check that the IP address and port specified when starting the drone simulator match the settings of the control center.
- **Data processing errors**: Ensure that the input data format is correct and that Python 3 and the required libraries (e.g., `matplotlib, numpy`) are installed.