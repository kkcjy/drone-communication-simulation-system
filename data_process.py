import re
import csv
import matplotlib
import numpy as np
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt

def read_modified_data(log_path, pairs):
    dis_list = [[] for _ in range(3)]
    time_list = [[] for _ in range(3)]
    pattern = re.compile(r"\[(\d)-(\d)\]: ModifiedD = ([\d\.]+), time = (\d+)")
    for idx, (a, b) in enumerate(pairs):
        pass 
    with open(log_path, 'r') as f:
        for line in f:
            m = pattern.search(line)
            if m:
                src, dst = int(m.group(1)), int(m.group(2))
                for i, (a, b) in enumerate(pairs):
                    if (src, dst) == (a, b) or (src, dst) == (b, a):
                        dis = float(m.group(3))
                        t = int(m.group(4))
                        dis_list[i].append(dis)
                        time_list[i].append(t)
                        break
    max_len = max(len(d) for d in dis_list)
    dis_arr = np.full((3, max_len), np.nan)
    time_arr = np.full((3, max_len), np.nan)
    for i in range(3):
        l = len(dis_list[i])
        dis_arr[i, :l] = dis_list[i]
        time_arr[i, :l] = time_list[i]
    return dis_arr, time_arr

# replace the timestamp in the modified log with the corresponding time in the flight log
def modify_realTime(timestamp, flight_log_path):
    ts2hms = {}
    with open(flight_log_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 4:
                ts = parts[3]
                hmsms = parts[0][8:17]  
                ts2hms[ts] = hmsms
    real_timestamp = timestamp.astype('U32')
    it = np.nditer(real_timestamp, flags=['multi_index', 'refs_ok'], op_flags=['readwrite'])
    while not it.finished:
        t = it[0]
        try:
            if t == '' or t == 'nan':
                hmsms = '---------'
            else:
                t_str = str(int(float(t)))
                hmsms = ts2hms.get(t_str, '---------')
        except Exception:
            hmsms = '---------'
        it[0][...] = hmsms
        it.iternext()
    return real_timestamp

def read_swarm_data(log_path, pairs):
    dis_list = [[] for _ in range(3)]
    time_list = [[] for _ in range(3)]
    for idx, (a, b) in enumerate(pairs):
        pass 
    with open(log_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 5:
                continue
            hmsms = parts[0][8:17]
            for i in [1, 3]:
                seq = parts[i]
                dis = float(parts[i+1])
                if '-' in seq:
                    src, dst = map(int, seq.split('-'))
                    for j, (a, b) in enumerate(pairs):
                        if (src, dst) == (a, b) or (src, dst) == (b, a):
                            dis_list[j].append(dis)
                            time_list[j].append(hmsms)
                            break
    max_len = max(len(d) for d in dis_list)
    dis_arr = np.full((3, max_len), np.nan)
    time_arr = np.full((3, max_len), '', dtype='U9') 
    for i in range(3):
        l = len(dis_list[i])
        dis_arr[i, :l] = dis_list[i]
        time_arr[i, :l] = time_list[i]
    return dis_arr, time_arr

def read_true_data(log_path):
    dis_list = [[] for _ in range(3)]
    time_list = [[] for _ in range(3)]
    with open(log_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader) 
        for row in reader:
            if len(row) < 4:
                continue
            ts = row[0]
            try:
                hms, ms = ts.split()[1].split('.')
                hms_str = hms.replace(':', '') + ms 
            except Exception:
                hms_str = ''
            distances = row[1:4]
            for i in range(3):
                try:
                    dis = float(distances[i]) * 100  
                except Exception:
                    dis = np.nan
                dis_list[i].append(dis)
                time_list[i].append(hms_str)
    max_len = max(len(d) for d in dis_list)
    dis_arr = np.full((3, max_len), np.nan)
    time_arr = np.full((3, max_len), '', dtype='U9') 
    for i in range(3):
        l = len(dis_list[i])
        dis_arr[i, :l] = dis_list[i]
        time_arr[i, :l] = time_list[i]
    return dis_arr, time_arr

def hms_to_seconds(hms_arr):
    sec_arr = []
    for hms in hms_arr:
        if isinstance(hms, str) and len(hms) == 9 and hms[:6].isdigit() and hms[6:].isdigit():
            h, m, s = int(hms[:2]), int(hms[2:4]), int(hms[4:6])
            ms = int(hms[6:])  
            sec_arr.append(h*3600 + m*60 + s + ms/1000)
        else:
            sec_arr.append(np.nan)
    return np.array(sec_arr)

def seconds_to_hms(sec):
    h = int(sec // 3600)
    m = int((sec % 3600) // 60)
    s = int(sec % 60)
    ms = int(round((sec - int(sec)) * 1000))
    return f"{h:02d}:{m:02d}:{s:02d}.{ms:03d}"

def plot_multi_ranging(modified_dis, modified_time, swarm_dis, swarm_time, true_dis, true_time):
    plt.figure(figsize=(18, 9))

    true_colors = ["#E0D4D4", "#E0D4D4", "#E0D4D4"]
    true_markers = ['o', 'o', 'o']
    true_labels = ['drone1-drone2 (true)', 'drone1-drone3 (true)', 'drone2-drone3 (true)']

    swarm_colors = ["#00B27A", "#D52700", "#75d8d1"]
    swarm_markers = ['*', '*', '*'] 
    swarm_labels = ['drone1-drone2 (swarm)', 'drone1-drone3 (swarm)', 'drone2-drone3 (swarm)']

    modified_colors = ["#9E5409", "#3BB1F5", "#8C6EF8"]
    modified_markers = ['x', 'x', 'x'] 
    modified_labels = ['drone1-drone2 (modified)', 'drone1-drone3 (modified)', 'drone2-drone3 (modified)']
    
    for i in range(3):
        # true
        x_true = hms_to_seconds(true_time[i])
        y_true = true_dis[i]
        plt.plot(x_true, y_true, linestyle='--', color=true_colors[i], 
                 marker=true_markers[i], label=true_labels[i], linewidth=1, markersize=4)

        # swarm
        x_swarm = hms_to_seconds(swarm_time[i])
        y_swarm = swarm_dis[i]
        plt.plot(x_swarm, y_swarm, linestyle='-', color=swarm_colors[i], 
                 marker=swarm_markers[i], label=swarm_labels[i], linewidth=1, markersize=4)
        
        # modified
        x_mod = hms_to_seconds(modified_time[i])
        y_mod = modified_dis[i]
        plt.plot(x_mod, y_mod,  linestyle=':', color=modified_colors[i], 
                 marker=modified_markers[i], label=modified_labels[i], linewidth=3, markersize=5)

    all_x = np.concatenate([hms_to_seconds(modified_time[i]) for i in range(3)])
    all_x = all_x[~np.isnan(all_x)]
    all_x = np.sort(np.unique(all_x))
    step = max(1, len(all_x)//15)
    xticks = all_x[::step]
    xticklabels = [seconds_to_hms(sec) for sec in xticks]

    plt.xticks(xticks, xticklabels, rotation=30)
    plt.xlabel('Time', fontsize=16)
    plt.ylabel('Distance (m)', fontsize=16)
    plt.title('Multi-Drones Distance Comparison', fontsize=18)
    plt.legend(fontsize=12)
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    modified_Log_path = 'data/modified_Log.txt'
    swarm_Log_path = 'data/swarm_Log.txt'
    true_Log_path = 'data/true_Log.csv'
    flight_Log_path = 'data/flight_Log.txt'

    pairs = [(1, 0), (2, 0), (1, 2)]

    modified_dis, modified_time = read_modified_data(modified_Log_path, pairs)
    modified_time = modify_realTime(modified_time, flight_Log_path)

    swarm_dis, swarm_time = read_swarm_data(swarm_Log_path, pairs)

    true_dis, true_time = read_true_data(true_Log_path)

    plot_multi_ranging(modified_dis, modified_time, swarm_dis, swarm_time, true_dis, true_time)