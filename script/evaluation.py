import re
import csv
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')


# This script integrates the processed SR and DSR data, aligns them with the VICON timestamps, and then evaluates the data.


# Set the active address and use the target address’s time range as the alignment reference
RESULT_REPRODUCTION = False
CHECK_POINT = 2
local_address = 2
neighbor_address = 3
leftbound = 1409700
rightbound = 1423480
# leftbound = 1719676
# rightbound = 1725063
# leftbound = 2056100
# rightbound = 2066600
# leftbound = 562850
# rightbound = 574910
invalid_sign = -1

sys_path = "../data/simulation_dep.csv"
ieee_path = "../data/log/ieee.txt"
sr_v1_path = "../data/log/swarm_v1.txt"
sr_v2_path = "../data/log/swarm_v2.txt"
dsr_path = "../data/log/dynamic.txt"
cdsr_path = "../data/log/compensate.txt"
vicon_path = "../data/vicon.txt"
output_path = "../data/ranging_log.csv"


def align_sys_time(time_list):
    sys_time = []
    rx_time = []
    align_sys_time = []
    with open(sys_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if int(row['Rx0_addr']) == local_address:
                sys_time.append(int(row['system_time']))
                rx_time.append(int(row['Rx0_time']))

    index = 0
    check_interval = 0
    length = len(time_list)
    for i in range(len(rx_time)):
        if rx_time[i] == time_list[index]:
            align_sys_time.append(sys_time[i])
            if index < length - CHECK_POINT - 1:
                check_interval = (sys_time[i + 1] - sys_time[i]) / (CHECK_POINT + 1)
                for j in range(1, CHECK_POINT + 1):
                    align_sys_time.append(sys_time[i] + check_interval * j)
            else:
                for j in range(1, CHECK_POINT + 1):
                    align_sys_time.append(sys_time[i] + check_interval * j)
                break
            index += 3
        
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

def read_sr_v1_log():
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

def read_sr_v2_log():
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

def read_dsr_log():
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

def read_cdsr_log():
    cdsr_value = []
    cdsr_time = []
    pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: CDSR dist = (-?\d+\.\d+), time = (\d+)")

    with open(cdsr_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            if i < 3:
                continue
            if (match := pattern.search(line)):
                cdsr_value.append(float(match.group(1)))
                cdsr_time.append(int(match.group(2)))

    cdsr_sys_time = align_sys_time(cdsr_time)
    return cdsr_value, cdsr_time, cdsr_sys_time

def read_vicon_log(): 
    vicon_value = []
    vicon_time = []
    pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: vicon dist = (-?\d+\.\d+), time = (\d+)")
    
    with open(vicon_path, "r", encoding="utf-8") as f:
        for line in f:
            if (match := pattern.search(line)):
                vicon_value.append(float(match.group(1)))
                vicon_time.append(int(match.group(2)))
    
    return vicon_value, vicon_time

def write_ranging_log(ieee, ieee_sys_time, sr_v1, sr_v1_sys_time, sr_v2, sr_v2_sys_time, dsr, dsr_sys_time, cdsr, cdsr_sys_time, vicon, vicon_sys_time):
    ieee = np.array(ieee)
    ieee_sys_time = np.array(ieee_sys_time)
    sr_v1 = np.array(sr_v1)
    sr_v1_sys_time = np.array(sr_v1_sys_time)
    sr_v2 = np.array(sr_v2)
    sr_v2_sys_time = np.array(sr_v2_sys_time)
    dsr = np.array(dsr)
    dsr_sys_time = np.array(dsr_sys_time)
    cdsr = np.array(cdsr)
    cdsr_sys_time = np.array(cdsr_sys_time)
    vicon = np.array(vicon)
    vicon_sys_time = np.array(vicon_sys_time)

    common_time = set(ieee_sys_time) & set(sr_v1_sys_time) & set(sr_v2_sys_time) & set(dsr_sys_time) & set(cdsr_sys_time)
    common_time = sorted(list(common_time))

    ieee_idx = {t: i for i, t in enumerate(ieee_sys_time)}
    sr_v1_idx = {t: i for i, t in enumerate(sr_v1_sys_time)}
    sr_v2_idx = {t: i for i, t in enumerate(sr_v2_sys_time)}
    dsr_idx = {t: i for i, t in enumerate(dsr_sys_time)}
    cdsr_idx = {t: i for i, t in enumerate(cdsr_sys_time)}

    filtered_ieee = []
    filtered_sr_v1 = []
    filtered_sr_v2 = []
    filtered_dsr = []
    filtered_cdsr = []
    filtered_vicon = []
    filtered_time = []

    invalid_ignore = True

    with open(output_path, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["IEEE", "SR_V1", "SR_V2", "DSR", "CDSR", "VICON", "TIME"])

        count = 0
        for t in common_time:
            if invalid_ignore and (ieee[ieee_idx[t]] == invalid_sign and sr_v1[sr_v1_idx[t]] == invalid_sign and sr_v2[sr_v2_idx[t]] == invalid_sign
                                   and dsr[dsr_idx[t]] == invalid_sign and cdsr[cdsr_idx[t]] == invalid_sign):
                continue
            else:
                invalid_ignore = False
            idx_vicon = np.argmin(np.abs(vicon_sys_time - t))
            filtered_ieee.append(ieee[ieee_idx[t]])
            filtered_sr_v1.append(sr_v1[sr_v1_idx[t]])
            filtered_sr_v2.append(sr_v2[sr_v2_idx[t]])
            filtered_dsr.append(dsr[dsr_idx[t]])
            filtered_cdsr.append(cdsr[cdsr_idx[t]])
            filtered_vicon.append(vicon[idx_vicon])
            filtered_time.append(t)
            writer.writerow([f"{ieee[ieee_idx[t]]:.1f}", f"{sr_v1[sr_v1_idx[t]]:.1f}", f"{sr_v2[sr_v2_idx[t]]:.1f}", f"{dsr[dsr_idx[t]]:.1f}", f"{cdsr[cdsr_idx[t]]:.1f}", f"{vicon[idx_vicon]:.4f}", f"{t}"])
            count += 1
    print(f"Ranging log saved to {output_path}, total {count} records.")

    return (np.array(filtered_ieee), np.array(filtered_sr_v1), np.array(filtered_sr_v2), np.array(filtered_dsr), np.array(filtered_cdsr), np.array(filtered_time))

def read_ranging_log():
    data = pd.read_csv(output_path)
    return (data['IEEE'].values, data['SR_V1'].values, data['SR_V2'].values, data['DSR'].values, data['CDSR'].values, data['VICON'].values, data['TIME'].values)

def get_align_data(ieee, sr_v1, sr_v2, dsr, cdsr, sys_time, vicon, vicon_sys_time):
    def get_diff_dis(ieee, sr_v1, sr_v2, dsr, cdsr, sys_time, vicon, vicon_sys_time):
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
        cdsr_mean = mean_in_window(cdsr, sys_time)
        vicon_mean = mean_in_window(vicon, vicon_sys_time)

        avg_diff = np.nanmean([vicon_mean - ieee_mean, vicon_mean - sr_v1_mean, vicon_mean - sr_v2_mean, vicon_mean - dsr_mean, vicon_mean - cdsr_mean])
        return avg_diff

    ieee = np.array(ieee)
    sr_v1 = np.array(sr_v1)
    sr_v2 = np.array(sr_v2)
    dsr = np.array(dsr)
    cdsr = np.array(cdsr)
    vicon = np.array(vicon)
    sys_time = np.array(sys_time)

    avg_diff = get_diff_dis(ieee, sr_v1, sr_v2, dsr, cdsr, sys_time, vicon, vicon_sys_time)

    align_ieee = ieee + avg_diff
    align_sr_v1 = sr_v1 + avg_diff
    align_sr_v2 = sr_v2 + avg_diff
    align_dsr = dsr + avg_diff
    align_cdsr = cdsr + avg_diff
    align_vicon = vicon

    return align_ieee, align_sr_v1, align_sr_v2, align_dsr, align_cdsr, align_vicon, avg_diff

def evaluation_data(align_ieee, ieee_sys_time, align_sr_v1, sr_v1_sys_time, align_sr_v2, sr_v2_sys_time, align_dsr, dsr_sys_time, align_cdsr, cdsr_sys_time, align_vicon, vicon_sys_time, avg_diff):
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

    align_ieee = np.array(align_ieee)
    ieee_sys_time = np.array(ieee_sys_time)
    align_sr_v1 = np.array(align_sr_v1)
    sr_v1_sys_time = np.array(sr_v1_sys_time)
    align_sr_v2 = np.array(align_sr_v2)
    sr_v2_sys_time = np.array(sr_v2_sys_time)
    align_dsr = np.array(align_dsr)
    dsr_sys_time = np.array(dsr_sys_time)
    align_cdsr = np.array(align_cdsr)
    cdsr_sys_time = np.array(cdsr_sys_time)
    align_vicon = np.array(align_vicon)
    vicon_sys_time = np.array(vicon_sys_time)

    ieee_f, vicon_for_ieee, invalid_rate_ieee = valid_filter_and_metrics(align_ieee, ieee_sys_time, align_vicon, vicon_sys_time, avg_diff)
    sr_v1_f, vicon_for_sr_v1, invalid_rate_sr_v1 = valid_filter_and_metrics(align_sr_v1, sr_v1_sys_time, align_vicon, vicon_sys_time, avg_diff)
    sr_v2_f, vicon_for_sr_v2, invalid_rate_sr_v2 = valid_filter_and_metrics(align_sr_v2, sr_v2_sys_time, align_vicon, vicon_sys_time, avg_diff)
    dsr_f, vicon_for_dsr, invalid_rate_dsr = valid_filter_and_metrics(align_dsr, dsr_sys_time, align_vicon, vicon_sys_time, avg_diff)
    cdsr_f, vicon_for_cdsr, invalid_rate_cdsr = valid_filter_and_metrics(align_cdsr, cdsr_sys_time, align_vicon, vicon_sys_time, avg_diff)

    mean_ae_ieee, max_ae_ieee, rmse_ieee, mre_ieee = compute_error_metrics(ieee_f, vicon_for_ieee)
    mean_ae_sr_v1, max_ae_sr_v1, rmse_sr_v1, mre_sr_v1 = compute_error_metrics(sr_v1_f, vicon_for_sr_v1)
    mean_ae_sr_v2, max_ae_sr_v2, rmse_sr_v2, mre_sr_v2 = compute_error_metrics(sr_v2_f, vicon_for_sr_v2)
    mean_ae_dsr, max_ae_dsr, rmse_dsr, mre_dsr = compute_error_metrics(dsr_f, vicon_for_dsr)
    mean_ae_cdsr, max_ae_cdsr, rmse_cdsr, mre_cdsr = compute_error_metrics(cdsr_f, vicon_for_cdsr)

    print("==== Error metrics for all valid data ====")
    print(f"IEEE : Mean AE(平均绝对误差) = {mean_ae_ieee:.3f} cm, Max AE(最大绝对误差) = {max_ae_ieee:.3f} cm, RMSE(均方根误差) = {rmse_ieee:.3f} cm, MRE(平均相对误差) = {mre_ieee:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_ieee:.2f}%")
    # print(f"SR_V1: Mean AE(平均绝对误差) = {mean_ae_sr_v1:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr_v1:.3f} cm, RMSE(均方根误差) = {rmse_sr_v1:.3f} cm, MRE(平均相对误差) = {mre_sr_v1:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_sr_v1:.2f}%")
    # print(f"SR_V2: Mean AE(平均绝对误差) = {mean_ae_sr_v2:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr_v2:.3f} cm, RMSE(均方根误差) = {rmse_sr_v2:.3f} cm, MRE(平均相对误差) = {mre_sr_v2:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_sr_v2:.2f}%")
    print(f"SR: Mean AE(平均绝对误差) = {mean_ae_sr_v2:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr_v2:.3f} cm, RMSE(均方根误差) = {rmse_sr_v2:.3f} cm, MRE(平均相对误差) = {mre_sr_v2:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_sr_v2:.2f}%")
    print(f"DSR  : Mean AE(平均绝对误差) = {mean_ae_dsr:.3f} cm, Max AE(最大绝对误差) = {max_ae_dsr:.3f} cm, RMSE(均方根误差) = {rmse_dsr:.3f} cm, MRE(平均相对误差) = {mre_dsr:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_dsr:.2f}%")
    print(f"CDSR : Mean AE(平均绝对误差) = {mean_ae_cdsr:.3f} cm, Max AE(最大绝对误差) = {max_ae_cdsr:.3f} cm, RMSE(均方根误差) = {rmse_cdsr:.3f} cm, MRE(平均相对误差) = {mre_cdsr:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_cdsr:.2f}%")

def ranging_plot(ranging1, ranging1_sys_time, ranging2, ranging2_sys_time, ranging3, ranging3_sys_time, vicon, vicon_sys_time, name1="RANGING1", name2="RANGING2", name3="RANGING3"):
    plt.plot(ranging1_sys_time, ranging1, color='#4A90E2', label=name1, linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(ranging2_sys_time, ranging2, color="#E4491E", label=name2, linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(ranging3_sys_time, ranging3, color="#FF7B00", label=name3, linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(vicon_sys_time, vicon, color="#9DF423", label='VICON', alpha=0.8, linestyle='-', marker='o', markersize=4, linewidth=1.5)
    plt.xlabel('Time (ms)', fontsize=20) 
    plt.ylabel('Distance Measurement',  fontsize=20)
    plt.title(f'{name1} vs {name2} vs {name3} vs VICON Distance Measurements Over Absolute Time',  fontsize=20)
    plt.legend(fontsize=16) 
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    if RESULT_REPRODUCTION:
        ieee, sr_v1, sr_v2, dsr, cdsr, _, sys_time = read_ranging_log()
        vicon, vicon_sys_time = read_vicon_log()

    else:
        ieee, ieee_time, ieee_sys_time = read_ieee_log()
        sr_v1, sr_v1_time, sr_v1_sys_time = read_sr_v1_log()
        sr_v2, sr_v2_time, sr_v2_sys_time = read_sr_v2_log()
        dsr, dsr_time, dsr_sys_time = read_dsr_log()
        cdsr, cdsr_time, cdsr_sys_time = read_cdsr_log()
        vicon, vicon_sys_time = read_vicon_log()

        ieee, sr_v1, sr_v2, dsr, cdsr, sys_time = write_ranging_log(ieee, ieee_sys_time, sr_v1, sr_v1_sys_time, sr_v2, sr_v2_sys_time, dsr, dsr_sys_time, cdsr, cdsr_sys_time, vicon, vicon_sys_time)

    align_ieee, align_sr_v1, align_sr_v2, align_dsr, align_cdsr, align_vicon, avg_diff = get_align_data(ieee, sr_v1, sr_v2, dsr, cdsr, sys_time, vicon, vicon_sys_time)

    evaluation_data(align_ieee, sys_time, align_sr_v1, sys_time, align_sr_v2, sys_time, align_dsr, sys_time, align_cdsr, sys_time, align_vicon, vicon_sys_time, avg_diff)

    # ranging_plot(align_sr_v2, sys_time, align_dsr, sys_time, align_cdsr, sys_time, align_vicon, vicon_sys_time, name1="SR_V2", name2="DSR", name3="CDSR")
    ranging_plot(align_sr_v2, sys_time, align_dsr, sys_time, align_cdsr, sys_time, align_vicon, vicon_sys_time, name1="SR", name2="DSR", name3="CDSR")