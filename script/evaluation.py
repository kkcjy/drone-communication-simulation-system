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
leftbound = 2056100
rightbound = 2066600
invalid_sign = -1

sys_path = "data/raw_sensor_data.csv"
ieee_path = "data/log/802_15_4z.txt"
sr_v1_path = "data/log/swarm_v1.txt"
sr_v2_path = "data/log/swarm_v2.txt"
dsr_path = "data/log/dynamic.txt"
vicon_path = "data/vicon.txt"
output_path = "data/ranging_log.csv"


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

def read_ieee_log():
    ieee_value = []
    ieee_time = []
    pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: IEEE dist = (-?\d+(?:\.\d+)?), time = (\d+)")

    with open(ieee_path, "r", encoding="utf-8") as f:
        for line in f:
            if (match := pattern.search(line)):
                ieee_value.append(float(match.group(1)))
                ieee_time.append(int(match.group(2)))

    ieee_sys_time = align_sys_time(ieee_time)
    return ieee_value, ieee_time, ieee_sys_time

def read_sr_v1_Log():
    sr_v1_value = []
    sr_v1_time = []
    pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: SR_V1 dist = (-?\d+), time = (\d+)")

    with open(sr_v1_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            if i < 3:
                continue
            if (match := pattern.search(line)):
                sr_v1_value.append(int(match.group(1)))
                sr_v1_time.append(int(match.group(2)))

    sr_v1_sys_time = align_sys_time(sr_v1_time)
    return sr_v1_value, sr_v1_time, sr_v1_sys_time

def read_sr_v2_Log():
    sr_v2_value = []
    sr_v2_time = []
    pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: SR_V2 dist = (-?\d+), time = (\d+)")

    with open(sr_v2_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            if i < 3:
                continue
            if (match := pattern.search(line)):
                sr_v2_value.append(int(match.group(1)))
                sr_v2_time.append(int(match.group(2)))

    sr_v2_sys_time = align_sys_time(sr_v2_time)
    return sr_v2_value, sr_v2_time, sr_v2_sys_time

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

def write_ranging_Log(ieee, ieee_sys_time, sr_v1, sr_v1_sys_time, sr_v2, sr_v2_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time):
    ieee = np.array(ieee)
    ieee_sys_time = np.array(ieee_sys_time)
    sr_v1 = np.array(sr_v1)
    sr_v1_sys_time = np.array(sr_v1_sys_time)
    sr_v2 = np.array(sr_v2)
    sr_v2_sys_time = np.array(sr_v2_sys_time)
    dsr = np.array(dsr)
    dsr_sys_time = np.array(dsr_sys_time)
    vicon = np.array(vicon)
    vicon_sys_time = np.array(vicon_sys_time)

    common_time = set(ieee_sys_time) & set(sr_v1_sys_time) & set(sr_v2_sys_time) & set(dsr_sys_time)
    common_time = sorted(list(common_time))

    ieee_idx = {t: i for i, t in enumerate(ieee_sys_time)}
    sr_v1_idx = {t: i for i, t in enumerate(sr_v1_sys_time)}
    sr_v2_idx = {t: i for i, t in enumerate(sr_v2_sys_time)}
    dsr_idx = {t: i for i, t in enumerate(dsr_sys_time)}

    filtered_ieee = []
    filtered_sr_v1 = []
    filtered_sr_v2 = []
    filtered_dsr = []
    filtered_vicon = []
    filtered_time = []

    with open(output_path, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["IEEE", "SR_V1", "SR_V2", "DSR", "VICON", "TIME"])

        count = 0
        for t in common_time:
            idx_vicon = np.argmin(np.abs(vicon_sys_time - t))
            filtered_ieee.append(ieee[ieee_idx[t]])
            filtered_sr_v1.append(sr_v1[sr_v1_idx[t]])
            filtered_sr_v2.append(sr_v2[sr_v2_idx[t]])
            filtered_dsr.append(dsr[dsr_idx[t]])
            filtered_vicon.append(vicon[idx_vicon])
            filtered_time.append(t)
            writer.writerow([ieee[ieee_idx[t]], sr_v1[sr_v1_idx[t]], sr_v2[sr_v2_idx[t]], dsr[dsr_idx[t]], vicon[idx_vicon], t])
            count += 1
    print(f"Ranging log saved to {output_path}, total {count} records.")

    return (np.array(filtered_ieee), np.array(filtered_sr_v1), np.array(filtered_sr_v2), np.array(filtered_dsr), np.array(filtered_time))

def get_align_data(ieee, sr_v1, sr_v2, dsr, sys_time, vicon, vicon_sys_time):
    def get_diff_dis(ieee, sr_v1, sr_v2, dsr, sys_time, vicon, vicon_sys_time):
        def mean_in_window(arr, arr_time):
            idx_left = np.searchsorted(arr_time, leftbound, side='left')
            idx_right = np.searchsorted(arr_time, rightbound, side='right')
            slice_ = arr[idx_left:idx_right]
            slice_ = slice_[slice_ != invalid_sign]
            return np.mean(slice_) if slice_.size > 0 else np.nan

        ieee_mean = mean_in_window(ieee, sys_time)
        sr_v1_mean = mean_in_window(sr_v1, sys_time)
        sr_v2_mean = mean_in_window(sr_v2, sys_time)
        dsr_mean = mean_in_window(dsr, sys_time)
        vicon_mean = mean_in_window(vicon, vicon_sys_time)

        avg_diff = np.nanmean([vicon_mean - ieee_mean, vicon_mean - sr_v1_mean, vicon_mean - sr_v2_mean, vicon_mean - dsr_mean])
        return avg_diff

    ieee = np.array(ieee)
    sr_v1 = np.array(sr_v1)
    sr_v2 = np.array(sr_v2)
    dsr = np.array(dsr)
    vicon = np.array(vicon)
    sys_time = np.array(sys_time)

    avg_diff = get_diff_dis(ieee, sr_v1, sr_v2, dsr, sys_time, vicon, vicon_sys_time)

    align_ieee = ieee + avg_diff
    align_sr_v1 = sr_v1 + avg_diff
    align_sr_v2 = sr_v2 + avg_diff
    align_dsr = dsr + avg_diff
    align_vicon = vicon

    return align_ieee, align_sr_v1, align_sr_v2, align_dsr, align_vicon, avg_diff

def evaluation_data(align_ieee, ieee_sys_time, align_sr_v1, sr_v1_sys_time, align_sr_v2, sr_v2_sys_time, align_dsr, dsr_sys_time, align_vicon, vicon_sys_time, avg_diff):
    def compute_error_metrics(predicted, ground_truth):
        if len(predicted) == 0 or len(ground_truth) == 0:
            return np.nan, np.nan, np.nan, np.nan
        ae = np.abs(predicted - ground_truth)
        mean_ae = np.mean(ae)
        max_ae = np.max(ae)
        rmse = np.sqrt(np.mean((predicted - ground_truth) ** 2))
        mean_re = np.mean(np.abs(predicted - ground_truth) / ground_truth) * 100
        return mean_ae, max_ae, rmse, mean_re

    def valid_filter_and_metrics(align_data, sys_time, align_vicon, vicon_sys_time, avg_diff):
        filtered, vicon_for_data = [], []
        for i, t in enumerate(sys_time):
            if align_data[i] == avg_diff + invalid_sign:
                continue
            idx = np.argmin(np.abs(vicon_sys_time - t))
            filtered.append(align_data[i])
            vicon_for_data.append(align_vicon[idx])
        filtered = np.array(filtered)
        vicon_for_data = np.array(vicon_for_data)
        invalid_rate = (len(sys_time) - len(filtered)) / len(sys_time) * 100 if len(sys_time) > 0 else np.nan
        return filtered, vicon_for_data, invalid_rate

    def common_valid_filter_and_metrics(ieee, sr_v1, sr_v2, dsr, sys_time, align_vicon, vicon_sys_time, avg_diff):
        f_ieee, f_sr_v1, f_sr_v2, f_dsr, f_vicon, f_time = [], [], [], [], [], []
        for i, t in enumerate(sys_time):
            if (ieee[i] == avg_diff + invalid_sign or sr_v1[i] == avg_diff + invalid_sign
                or sr_v2[i] == avg_diff + invalid_sign or dsr[i] == avg_diff + invalid_sign):
                continue
            idx_vicon = np.argmin(np.abs(vicon_sys_time - t))
            f_ieee.append(ieee[i])
            f_sr_v1.append(sr_v1[i])
            f_sr_v2.append(sr_v2[i])
            f_dsr.append(dsr[i])
            f_vicon.append(align_vicon[idx_vicon])
            f_time.append(t)
        filtered_time = np.array(f_time)
        invalid_rate = (len(sys_time) - len(filtered_time)) / len(sys_time) * 100 if len(sys_time) > 0 else np.nan
        return (np.array(f_ieee), np.array(f_sr_v1), np.array(f_sr_v2),
                np.array(f_dsr), np.array(f_vicon), filtered_time, invalid_rate)

    align_ieee = np.array(align_ieee)
    ieee_sys_time = np.array(ieee_sys_time)
    align_sr_v1 = np.array(align_sr_v1)
    sr_v1_sys_time = np.array(sr_v1_sys_time)
    align_sr_v2 = np.array(align_sr_v2)
    sr_v2_sys_time = np.array(sr_v2_sys_time)
    align_dsr = np.array(align_dsr)
    dsr_sys_time = np.array(dsr_sys_time)
    align_vicon = np.array(align_vicon)
    vicon_sys_time = np.array(vicon_sys_time)

    ieee_filtered, vicon_for_ieee, invalid_rate_ieee = valid_filter_and_metrics(align_ieee, ieee_sys_time, align_vicon, vicon_sys_time, avg_diff)
    sr_v1_filtered, vicon_for_sr_v1, invalid_rate_sr_v1 = valid_filter_and_metrics(align_sr_v1, sr_v1_sys_time, align_vicon, vicon_sys_time, avg_diff)
    sr_v2_filtered, vicon_for_sr_v2, invalid_rate_sr_v2 = valid_filter_and_metrics(align_sr_v2, sr_v2_sys_time, align_vicon, vicon_sys_time, avg_diff)
    dsr_filtered, vicon_for_dsr, invalid_rate_dsr = valid_filter_and_metrics(align_dsr, dsr_sys_time, align_vicon, vicon_sys_time, avg_diff)

    mean_ae_ieee, max_ae_ieee, rmse_ieee, mre_ieee = compute_error_metrics(ieee_filtered, vicon_for_ieee)
    mean_ae_sr_v1, max_ae_sr_v1, rmse_sr_v1, mre_sr_v1 = compute_error_metrics(sr_v1_filtered, vicon_for_sr_v1)
    mean_ae_sr_v2, max_ae_sr_v2, rmse_sr_v2, mre_sr_v2 = compute_error_metrics(sr_v2_filtered, vicon_for_sr_v2)
    mean_ae_dsr, max_ae_dsr, rmse_dsr, mre_dsr = compute_error_metrics(dsr_filtered, vicon_for_dsr)

    print("==== Error metrics for all valid data ====")
    print(f"IEEE : Mean AE(平均绝对误差) = {mean_ae_ieee:.3f} cm, Max AE(最大绝对误差) = {max_ae_ieee:.3f} cm, RMSE(均方根误差) = {rmse_ieee:.3f} cm, MRE(平均相对误差) = {mre_ieee:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_ieee:.2f}%")
    print(f"SR_V1: Mean AE(平均绝对误差) = {mean_ae_sr_v1:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr_v1:.3f} cm, RMSE(均方根误差) = {rmse_sr_v1:.3f} cm, MRE(平均相对误差) = {mre_sr_v1:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_sr_v1:.2f}%")
    print(f"SR_V2: Mean AE(平均绝对误差) = {mean_ae_sr_v2:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr_v2:.3f} cm, RMSE(均方根误差) = {rmse_sr_v2:.3f} cm, MRE(平均相对误差) = {mre_sr_v2:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_sr_v2:.2f}%")
    print(f"DSR  : Mean AE(平均绝对误差) = {mean_ae_dsr:.3f} cm, Max AE(最大绝对误差) = {max_ae_dsr:.3f} cm, RMSE(均方根误差) = {rmse_dsr:.3f} cm, MRE(平均相对误差) = {mre_dsr:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_dsr:.2f}%")

    ieee_c, sr_v1_c, sr_v2_c, dsr_c, vicon_c, time_c, invalid_rate_common = common_valid_filter_and_metrics(align_ieee, align_sr_v1, align_sr_v2, align_dsr, ieee_sys_time, align_vicon, vicon_sys_time, avg_diff)

    mean_ae_ieee_c, max_ae_ieee_c, rmse_ieee_c, mre_ieee_c = compute_error_metrics(ieee_c, vicon_c)
    mean_ae_sr_v1_c, max_ae_sr_v1_c, rmse_sr_v1_c, mre_sr_v1_c = compute_error_metrics(sr_v1_c, vicon_c)
    mean_ae_sr_v2_c, max_ae_sr_v2_c, rmse_sr_v2_c, mre_sr_v2_c = compute_error_metrics(sr_v2_c, vicon_c)
    mean_ae_dsr_c, max_ae_dsr_c, rmse_dsr_c, mre_dsr_c = compute_error_metrics(dsr_c, vicon_c)

    print("==== Error Metrics for Common Valid Data ====")
    print(f"IEEE : Mean AE(平均绝对误差) = {mean_ae_ieee_c:.3f} cm, Max AE(最大绝对误差) = {max_ae_ieee_c:.3f} cm, RMSE(均方根误差) = {rmse_ieee_c:.3f} cm, MRE(平均相对误差) = {mre_ieee_c:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_common:.2f}%")
    print(f"SR_V1: Mean AE(平均绝对误差) = {mean_ae_sr_v1_c:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr_v1_c:.3f} cm, RMSE(均方根误差) = {rmse_sr_v1_c:.3f} cm, MRE(平均相对误差) = {mre_sr_v1_c:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_common:.2f}%")
    print(f"SR_V2: Mean AE(平均绝对误差) = {mean_ae_sr_v2_c:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr_v2_c:.3f} cm, RMSE(均方根误差) = {rmse_sr_v2_c:.3f} cm, MRE(平均相对误差) = {mre_sr_v2_c:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_common:.2f}%")
    print(f"DSR  : Mean AE(平均绝对误差) = {mean_ae_dsr_c:.3f} cm, Max AE(最大绝对误差) = {max_ae_dsr_c:.3f} cm, RMSE(均方根误差) = {rmse_dsr_c:.3f} cm, MRE(平均相对误差) = {mre_dsr_c:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_common:.2f}%")


def ranging_plot(sr, sr_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time):
    plt.plot(sr_sys_time, sr, color='#4A90E2', label='SR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(dsr_sys_time, dsr, color="#E4491E", label='DSR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(vicon_sys_time, vicon, color="#9DF423", label='VICON', alpha=0.8, linestyle='-', marker='o', markersize=4, linewidth=2)
    plt.xlabel('Time (ms)') 
    plt.ylabel('Distance Measurement')
    plt.title('SR_V2 vs DSR vs VICON Distance Measurements Over Absolute Time')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    ieee, ieee_time, ieee_sys_time = read_ieee_log()
    sr_v1, sr_v1_time, sr_v1_sys_time = read_sr_v1_Log()
    sr_v2, sr_v2_time, sr_v2_sys_time = read_sr_v2_Log()
    dsr, dsr_time, dsr_sys_time = read_dsr_Log()
    vicon, vicon_sys_time = read_vicon_Log()

    ieee, sr_v1, sr_v2, dsr, sys_time = write_ranging_Log(ieee, ieee_sys_time, sr_v1, sr_v1_sys_time, sr_v2, sr_v2_sys_time, dsr, dsr_sys_time, vicon, vicon_sys_time)

    align_ieee, align_sr_v1, align_sr_v2, align_dsr, align_vicon, avg_diff = get_align_data(ieee, sr_v1, sr_v2, dsr, sys_time, vicon, vicon_sys_time)

    evaluation_data(align_ieee, sys_time, align_sr_v1, sys_time, align_sr_v2, sys_time, align_dsr, sys_time, align_vicon, vicon_sys_time, avg_diff)

    ranging_plot(align_sr_v2, sr_v2_sys_time, align_dsr, dsr_sys_time, align_vicon, vicon_sys_time)