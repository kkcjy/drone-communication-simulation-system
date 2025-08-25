import re
import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')


# This script integrates the processed SR and DSR data, aligns them with the VICON timestamps, and then evaluates the data.


# Set the active address and use the target address’s time range as the alignment reference
local_address = 2
neighbor_address = 3
leftbound = 1409700
rightbound = 1423480
invalid_sign = -1

sys_path = "../data/processed_Log.csv"
dsr_path = "../data/dynamic_swarm_ranging.txt"
sr_path = "../data/swarm_ranging.txt"
vicon_path = "../data/vicon.txt"
ranging_Log_path = "../data/ranging_Log.csv"


def align_sys_time(time_list):
    sys_time = []
    rx_time = []
    align_sys_time = []
    with open(sys_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            sys_time.append(int(row['system_time']))
            rx_time.append(int(row['Rx0_time']))

    index = 0
    length = len(time_list)
    for i in range(len(rx_time)):
        if index >= length:
            break
        if rx_time[i] == time_list[index]:
            align_sys_time.append(sys_time[i])
            index += 1
        
    return align_sys_time

def read_vicon_Log(): 
    vicon_value = []
    vicon_time = []
    pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: vicon dist = (-?\d+\.\d+), time = (\d+)")
    
    with open(vicon_path, "r", encoding="utf-8") as f:
        for line in f:
            if (match := pattern.search(line)):
                vicon_value.append(float(match.group(1)))
                vicon_time.append(int(match.group(2)))
    
    return vicon_value, vicon_time

def read_dsr_Log():
    dsr_value = []
    dsr_time = []
    pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: DSR dist = (-?\d+\.\d+), time = (\d+)")

    with open(dsr_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            if i < 3:
                continue
            if (match := pattern.search(line)):
                dsr_value.append(float(match.group(1)))
                dsr_time.append(int(match.group(2)))

    dsr_sys_time = align_sys_time(dsr_time)
    return dsr_value, dsr_time, dsr_sys_time

def read_sr_Log():
    sr_value = []
    sr_time = []
    pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: SR dist = (-?\d+), time = (\d+)")

    with open(sr_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            if i < 3:
                continue
            if (match := pattern.search(line)):
                sr_value.append(int(match.group(1)))
                sr_time.append(int(match.group(2)))

    sr_sys_time = align_sys_time(sr_time)
    return sr_value, sr_time, sr_sys_time

def write_ranging_Log(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time):
    sr_sys_time = np.array(sr_sys_time)
    sr = np.array(sr)
    dsr_sys_time = np.array(dsr_sys_time)
    dsr = np.array(dsr)
    vicon_sys_time = np.array(vicon_sys_time)
    vicon = np.array(vicon)

    if len(sr_sys_time) == len(dsr_sys_time) and np.all(sr_sys_time == dsr_sys_time):
        with open(ranging_Log_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["DSR", "SR", "VICON", "TIME"])

            count = 0
            for i in range(len(sr_sys_time)):
                t = sr_sys_time[i]
                idx = np.argmin(np.abs(vicon_sys_time - t))

                writer.writerow([dsr[i], sr[i], vicon[idx], sr_sys_time[i]])
                count += 1

        print(f"Ranging log saved to {ranging_Log_path}, total {count} records.")
    else:
        print("Error: sr_sys_time and dsr_sys_time are not identical. Cannot write log.")

def get_align_data(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time):
    def get_diff_dis(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time):
        left_idx_sr = np.searchsorted(sr_sys_time, leftbound, side='left')
        right_idx_sr = np.searchsorted(sr_sys_time, rightbound, side='right')
        sr_slice = sr[left_idx_sr : right_idx_sr]
        sr_slice = sr_slice[sr_slice != invalid_sign]
        sr_mean = np.mean(sr_slice) if sr_slice.size > 0 else np.nan

        left_idx_dsr = np.searchsorted(dsr_sys_time, leftbound, side='left')
        right_idx_dsr = np.searchsorted(dsr_sys_time, rightbound, side='right')
        dsr_slice = dsr[left_idx_dsr : right_idx_dsr]
        dsr_slice = dsr_slice[dsr_slice != invalid_sign]
        dsr_mean = np.mean(dsr_slice) if dsr_slice.size > 0 else np.nan

        left_idx_vicon = np.searchsorted(vicon_sys_time, leftbound, side='left')
        right_idx_vicon = np.searchsorted(vicon_sys_time, rightbound, side='right')
        vicon_slice = vicon[left_idx_vicon : right_idx_vicon]
        vicon_slice = vicon_slice[vicon_slice != invalid_sign]
        vicon_mean = np.mean(vicon_slice) if vicon_slice.size > 0 else np.nan

        sr_diff = vicon_mean - sr_mean
        dsr_diff = vicon_mean - dsr_mean

        avg_diff = (sr_diff + dsr_diff) / 2

        return avg_diff
    
    sr = np.array(sr)
    dsr = np.array(dsr)
    vicon = np.array(vicon)
    sr_sys_time = np.array(sr_sys_time)
    dsr_sys_time = np.array(dsr_sys_time)
    vicon_sys_time = np.array(vicon_sys_time)

    avg_diff = get_diff_dis(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time)

    align_sr = sr + avg_diff
    align_dsr = dsr + avg_diff
    align_vicon = vicon

    return align_sr, align_dsr, align_vicon, avg_diff

def evaluation_data(align_sr, sr_sys_time, align_dsr, dsr_sys_time, align_vicon, vicon_sys_time, avg_diff):
    def compute_error_metrics(predicted, ground_truth):
        if len(predicted) == 0 or len(ground_truth) == 0:
            return np.nan, np.nan, np.nan, np.nan
        ae = np.abs(predicted - ground_truth)
        mean_ae = np.mean(ae)
        max_ae = np.max(ae)
        rmse = np.sqrt(np.mean((predicted - ground_truth) ** 2))
        mean_re = np.mean(np.abs(predicted - ground_truth) / ground_truth) * 100
        return mean_ae, max_ae, rmse, mean_re

    align_sr = np.array(align_sr)
    sr_sys_time = np.array(sr_sys_time)
    align_dsr = np.array(align_dsr)
    dsr_sys_time = np.array(dsr_sys_time)
    align_vicon = np.array(align_vicon)
    vicon_sys_time = np.array(vicon_sys_time)

    sr_filtered = []
    vicon_for_sr = []
    for i, t in enumerate(sr_sys_time):
        if align_sr[i] == avg_diff + invalid_sign:
            continue
        idx = np.argmin(np.abs(vicon_sys_time - t))
        sr_filtered.append(align_sr[i])
        vicon_for_sr.append(align_vicon[idx])
    sr_filtered = np.array(sr_filtered)
    vicon_for_sr = np.array(vicon_for_sr)
    invalid_rate_sr = (len(sr_sys_time) - len(sr_filtered)) / len(sr_sys_time) * 100 if len(sr_sys_time) > 0 else np.nan

    dsr_filtered = []
    vicon_for_dsr = []
    for i, t in enumerate(dsr_sys_time):
        if align_dsr[i] == avg_diff + invalid_sign:
            continue
        idx = np.argmin(np.abs(vicon_sys_time - t))
        dsr_filtered.append(align_dsr[i])
        vicon_for_dsr.append(align_vicon[idx])
    dsr_filtered = np.array(dsr_filtered)
    vicon_for_dsr = np.array(vicon_for_dsr)
    invalid_rate_dsr = (len(dsr_sys_time) - len(dsr_filtered)) / len(dsr_sys_time) * 100 if len(dsr_sys_time) > 0 else np.nan

    mean_ae_sr, max_ae_sr, rmse_sr, mre_sr = compute_error_metrics(sr_filtered, vicon_for_sr)
    mean_ae_dsr, max_ae_dsr, rmse_dsr, mre_dsr = compute_error_metrics(dsr_filtered, vicon_for_dsr)

    print("==== Error Metrics ====")
    print(f"SR  : Mean AE(平均绝对误差) = {mean_ae_sr:.3f} cm, "
          f"Max AE(最大绝对误差) = {max_ae_sr:.3f} cm, "
          f"RMSE(均方根误差) = {rmse_sr:.3f} cm, "
          f"MRE(平均相对误差) = {mre_sr:.3f}%, "
          f"Invalid Rate = {invalid_rate_sr:.2f}%")
    print(f"DSR : Mean AE(平均绝对误差) = {mean_ae_dsr:.3f} cm, "
          f"Max AE(最大绝对误差) = {max_ae_dsr:.3f} cm, "
          f"RMSE(均方根误差) = {rmse_dsr:.3f} cm, "
          f"MRE(平均相对误差) = {mre_dsr:.3f}%, "
          f"Invalid Rate = {invalid_rate_dsr:.2f}%")

def ranging_plot(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time):
    plt.plot(sr_sys_time, sr, color='#4A90E2', label='SR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(dsr_sys_time, dsr, color="#E4491E", label='DSR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(vicon_sys_time, vicon, color="#9DF423", label='VICON', alpha=0.8, linestyle='-', marker='o', markersize=4, linewidth=2)
    plt.xlabel('Time (ms)') 
    plt.ylabel('Distance Measurement')
    plt.title('SR vs DSR vs VICON Distance Measurements Over Absolute Time')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    sr, sr_time, sr_sys_time = read_sr_Log()
    dsr, dsr_time, dsr_sys_time = read_dsr_Log()
    vicon, vicon_sys_time = read_vicon_Log()
    
    write_ranging_Log(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time)

    # align_sr, align_dsr, align_vicon, avg_diff = get_align_data(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time)

    # evaluation_data(align_sr, sr_sys_time, align_dsr, dsr_sys_time, align_vicon, vicon_sys_time, avg_diff)

    # ranging_plot(align_sr, sr_sys_time, align_dsr, dsr_sys_time, align_vicon, vicon_sys_time)