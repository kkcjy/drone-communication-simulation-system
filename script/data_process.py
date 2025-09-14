import re
import numpy as np
import pandas as pd


# This script reads data collected by the sniffer and organizes it into a format suitable for simulation, saving the results in the simulation/data directory.


# Number of drones in the simulation.
DRONE_NUM = 2


# get number of Txi and Rxi from the header of the sniffer data file by seq
def count_tx_rx_from_header(file_path):
    with open(file_path, 'r') as f:
        header_line = f.readline().strip()
        headers = header_line.split(',')

        tx_count = 0
        rx_count = 0
        tx_pattern = re.compile(r'Tx(\d+)_seq')
        rx_pattern = re.compile(r'Rx(\d+)_seq')

        for header in headers:
            tx_match = tx_pattern.match(header)
            rx_match = rx_pattern.match(header)
            if tx_match:
                tx_num = int(tx_match.group(1))
                tx_count = max(tx_count, tx_num + 1)
            elif rx_match:
                rx_num = int(rx_match.group(1))
                rx_count = max(rx_count, rx_num + 1)

        return tx_count, rx_count

def get_drones_addr(file_path):
    search_lines = 50
    addr_set = []
    with open(file_path, 'r') as f:
        next(f)
        for _ in range(search_lines):
            line = f.readline().strip()
            parts = line.split(',')
            addr = int(parts[1])
            if addr not in addr_set:
                addr_set.append(addr)
    return addr_set


if __name__ == '__main__':
    buffer_size = 5 * DRONE_NUM
    buffer_pos = 0
    sniffer_data_path = '../data/raw_sensor_data.csv'
    processed_data_path = '../data/simulation_dep.csv'

    print("Starting to Read and Integrate Sniffer-Collected Data...")

    tx_count, rx_count = count_tx_rx_from_header(sniffer_data_path)
    print(f"Detected {tx_count} Tx and {rx_count} Rx in the sniffer data.")

    addr_set = get_drones_addr(sniffer_data_path)
    print(f"Detected drone addresses: {addr_set}")

    system_time = np.zeros(buffer_size, dtype=np.uint64)
    src_addr = np.zeros(buffer_size, dtype=np.uint16)
    msg_seq = np.zeros(buffer_size, dtype=np.uint16)
    filter = np.zeros(buffer_size, dtype=np.uint16)
    Tx_time = np.zeros(buffer_size, dtype=np.uint64)
    Rx_addr = np.zeros((buffer_size, DRONE_NUM - 1), dtype=np.uint16)
    Rx_time = np.zeros((buffer_size, DRONE_NUM - 1), dtype=np.uint64)

    with open(processed_data_path, 'w') as out_file:
        out_file.write("system_time,src_addr,msg_seq,filter,Tx_time")
        for i in range(DRONE_NUM - 1):
            out_file.write(f",Rx{i}_addr,Rx{i}_time")
        out_file.write('\n')

    processed_count = 0
    output_count = 0

    with open(sniffer_data_path, 'r') as f:
        f.readline()
        while True:
            line = f.readline().strip()
            if not line:
                break

            parts = line.split(',')
            processed_count += 1

            # system_time, src_addr, msg_seq, filter, Rx_addr
            system_time[buffer_pos] = int(parts[0])
            src_addr[buffer_pos] = int(parts[1])
            msg_seq[buffer_pos] = int(parts[2])
            filter[buffer_pos] = int(parts[4])

            addr_pos = 0
            for i in range(DRONE_NUM - 1):
                if addr_set[addr_pos] == src_addr[buffer_pos]:
                    addr_pos += 1
                Rx_addr[buffer_pos, i] = addr_set[addr_pos]
                addr_pos += 1

            # Tx time
            last_Tx_time = int(parts[5])
            last_Tx_seq = int(parts[6])
            for i in range(buffer_size):
                if src_addr[i] == src_addr[buffer_pos] and msg_seq[i] == last_Tx_seq:
                    Tx_time[i] = last_Tx_time

            # Rx time
            for rx_idx in range(rx_count):
                last_Tx_addr = int(parts[5 + 2 * tx_count + 3 * rx_idx])
                last_Rx_time = int(parts[6 + 2 * tx_count + 3 * rx_idx])
                last_Rx_seq = int(parts[7 + 2 * tx_count + 3 * rx_idx])

                for i in range(buffer_size):
                    if src_addr[i] == last_Tx_addr and msg_seq[i] == last_Rx_seq:
                        for j in range(DRONE_NUM - 1):
                            if Rx_addr[i, j] == src_addr[buffer_pos]:
                                Rx_time[i, j] = last_Rx_time

            # write to file
            if (buffer_pos + 1) % DRONE_NUM == 0 and src_addr[2 * DRONE_NUM - 1] != 0:
                for i in range(DRONE_NUM):
                    current_pos = (buffer_pos + 1 + i) % buffer_size
                    with open(processed_data_path, 'a') as outfile:
                        if Tx_time[current_pos] == 0:
                            continue
                        outfile.write(
                            f"{system_time[current_pos]},{src_addr[current_pos]},"
                            f"{msg_seq[current_pos]},{filter[current_pos]},"
                            f"{Tx_time[current_pos]}"
                        )
                        for j in range(DRONE_NUM - 1):
                            outfile.write(f",{Rx_addr[current_pos, j]},{Rx_time[current_pos, j]}")
                        outfile.write('\n')
                    output_count += 1

                    # clean up
                    system_time[current_pos] = 0
                    src_addr[current_pos] = 0
                    msg_seq[current_pos] = 0
                    filter[current_pos] = 0
                    Tx_time[current_pos] = 0
                    for j in range(DRONE_NUM - 1):
                        Rx_addr[current_pos, j] = 0
                        Rx_time[current_pos, j] = 0

            buffer_pos = (buffer_pos + 1) % buffer_size

    print(f"Processing completed! Processed {processed_count} lines of data, "
          f"output {output_count} valid records")
    print(f"Output file: {processed_data_path}")
