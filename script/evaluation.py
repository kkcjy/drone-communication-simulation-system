import re
import csv
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')
from scipy.stats import gaussian_kde

# This script integrates the processed SR and DSR data, aligns them with the VICON timestamps, and then evaluates the data.


# Set the active address and use the target address’s time range as the alignment reference
RESULT_REPRODUCTION = False
REAL_TIME_ENABLE = True
CHECK_POINT = 8
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
ieee_path = "../data/log_real_8/ieee.txt"
sr_v1_path = "../data/log_real_8/swarm_v1.txt"
sr_v2_path = "../data/log_real_8/swarm_v2.txt"
dsr_path = "../data/log_real_8/dynamic.txt"
cdsr_path = "../data/log_real_8/compensate.txt"
vicon_path = "../data/vicon.txt"
output_path = "../data/ranging_log.csv"

sr_color = '#62b2ff'            # 蓝色
dsr_ic_color  = '#ff937f'       # 红色
dsr_color = '#ffc04d'           # 黄色
vicon_color = '#78c99c'         # 绿色
sr_error_color = '#a7d3ff'      # 蓝色浅色
dsr_ic_error_color  = '#ffc0b7' # 红色浅色
dsr_error_color  = '#fedb9b'    # 黄色浅色

def align_sys_time(time_list):
    if REAL_TIME_ENABLE:
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
                index += CHECK_POINT + 1
        return align_sys_time
    
    else:
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
    # pattern = re.compile(rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: SR_V2 dist = (-?\d+), time = (\d+)")
    pattern = re.compile(rf"\[local_{local_address} <- neighbor_{neighbor_address}\]: SR_V2 dist = (-?\d+(?:\.\d+)?), time = (\d+)")

    with open(sr_v2_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            if i < 3:
                continue
            if (match := pattern.search(line)):
                sr_v2_value.append(float(match.group(1)))
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
            if invalid_ignore and (ieee[ieee_idx[t]] == invalid_sign or sr_v1[sr_v1_idx[t]] == invalid_sign or sr_v2[sr_v2_idx[t]] == invalid_sign
                                   or dsr[dsr_idx[t]] == invalid_sign or cdsr[cdsr_idx[t]] == invalid_sign):
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

def evaluation_data_motion_static(align_sr_v2, sr_v2_sys_time, align_cdsr, cdsr_sys_time, align_vicon, vicon_sys_time, avg_diff):
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

    def classify_motion_static(sr_v2_f, cdsr_f, vicon_for_sr_v2, sr_v2_sys_time, window=7):
        n = len(vicon_for_sr_v2)
        motion_mask = np.zeros(n, dtype=bool)

        for i in range(window-1, n):
            window_data = vicon_for_sr_v2[i-(window-1):i+1] 
            diffs = np.diff(window_data)
            avg_diff = np.mean(diffs)
            if np.abs(avg_diff) > 3:
                motion_mask[i] = True

        static_mask = ~motion_mask

        data = {
            "sr_v2_motion": sr_v2_f[motion_mask],
            "cdsr_motion": cdsr_f[motion_mask],
            "vicon_motion": vicon_for_sr_v2[motion_mask],
            "time_motion": sr_v2_sys_time[motion_mask],
            "sr_v2_static": sr_v2_f[static_mask],
            "cdsr_static": cdsr_f[static_mask],
            "vicon_static": vicon_for_sr_v2[static_mask],
            "time_static": sr_v2_sys_time[static_mask],
        }
        return data

    align_sr_v2 = np.array(align_sr_v2)
    sr_v2_sys_time = np.array(sr_v2_sys_time)
    align_cdsr = np.array(align_cdsr)
    cdsr_sys_time = np.array(cdsr_sys_time)
    align_vicon = np.array(align_vicon)
    vicon_sys_time = np.array(vicon_sys_time)

    sr_v2_f, vicon_for_sr_v2, invalid_rate_sr_v2 = valid_filter_and_metrics(align_sr_v2, sr_v2_sys_time, align_vicon, vicon_sys_time, avg_diff)
    cdsr_f, vicon_for_cdsr, invalid_rate_cdsr = valid_filter_and_metrics(align_cdsr, cdsr_sys_time, align_vicon, vicon_sys_time, avg_diff)

    sr_v2_motion, cdsr_motion, vicon_motion, time_motion, sr_v2_static, cdsr_static, vicon_static, time_static = classify_motion_static(sr_v2_f, cdsr_f, vicon_for_sr_v2, sr_v2_sys_time).values()

    print(len(sr_v2_motion), len(cdsr_motion), len(sr_v2_static), len(cdsr_static))

    mean_ae_sr_v2_motion, max_ae_sr_v2_motion, rmse_sr_v2_motion, mre_sr_v2_motion = compute_error_metrics(sr_v2_motion, vicon_motion)
    mean_ae_cdsr_motion, max_ae_cdsr_motion, rmse_cdsr_motion, mre_cdsr_motion = compute_error_metrics(cdsr_motion, vicon_motion)
    mean_ae_sr_v2_static, max_ae_sr_v2_static, rmse_sr_v2_static, mre_sr_v2_static = compute_error_metrics(sr_v2_static, vicon_static)
    mean_ae_cdsr_static, max_ae_cdsr_static, rmse_cdsr_static, mre_cdsr_static = compute_error_metrics(cdsr_static, vicon_static)

    print("==== Error metrics for motion data ====")
    print(f"SR-V2 : Mean AE = {mean_ae_sr_v2_motion:.3f} cm, Max AE = {max_ae_sr_v2_motion:.3f} cm, "
        f"RMSE = {rmse_sr_v2_motion:.3f} cm, MRE = {mre_sr_v2_motion:.3f}%")
    print(f"CDSR  : Mean AE = {mean_ae_cdsr_motion:.3f} cm, Max AE = {max_ae_cdsr_motion:.3f} cm, "
        f"RMSE = {rmse_cdsr_motion:.3f} cm, MRE = {mre_cdsr_motion:.3f}%")

    print("==== Error metrics for static data ====")
    print(f"SR-V2 : Mean AE = {mean_ae_sr_v2_static:.3f} cm, Max AE = {max_ae_sr_v2_static:.3f} cm, "
        f"RMSE = {rmse_sr_v2_static:.3f} cm, MRE = {mre_sr_v2_static:.3f}%")
    print(f"CDSR  : Mean AE = {mean_ae_cdsr_static:.3f} cm, Max AE = {max_ae_cdsr_static:.3f} cm, "
        f"RMSE = {rmse_cdsr_static:.3f} cm, MRE = {mre_cdsr_static:.3f}%")

    def plot_motion_static_scatter(sr_v2_motion, cdsr_motion, vicon_motion, time_motion,
                                sr_v2_static, cdsr_static, vicon_static, time_static):
        plt.figure(figsize=(14, 6))

        # Motion 数据 (实心圆点)
        plt.plot(time_motion, vicon_motion, 'green', label='VICON Motion')
        plt.scatter(time_motion, sr_v2_motion, color='blue', marker='o', s=20, label='SR-V2 Motion')

        # Static 数据 (空心圆点)
        plt.plot(time_static, vicon_static, 'green', label='VICON Static')
        plt.scatter(time_static, sr_v2_static, facecolors='none', edgecolors='yellow', marker='o', s=20, label='SR-V2 Static')

        plt.xlabel("Time (ms)")
        plt.ylabel("Distance (cm)")
        plt.title("Distance Measurements: Motion vs Static (Scatter)")
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.legend()
        plt.tight_layout()
        plt.show()

    plot_motion_static_scatter(sr_v2_motion, cdsr_motion, vicon_motion, time_motion, sr_v2_static, cdsr_static, vicon_static, time_static)


def sr_vicon_plot(sr, sr_sys_time, vicon, vicon_sys_time, sr_name="DS-TWR"):
    fig, ax1 = plt.subplots(figsize=(24, 14))
    
    ax1.set_xlabel('Time (ms)', fontsize=40, labelpad=40)
    ax1.set_ylabel('Distance (cm)', fontsize=40, labelpad=40)
    
    l1, = ax1.plot(sr_sys_time, sr, color=sr_color, linestyle='-', linewidth=4, marker='o', markersize=6, label='DS-TWR')
    l2, = ax1.plot(vicon_sys_time, vicon, color=vicon_color, linestyle='-', linewidth=4, label='VICON')
    
    ax1.tick_params(axis='x', labelsize=35)
    ax1.tick_params(axis='y', labelsize=35)
    ax1.grid(True, linestyle='--', alpha=0.3)

    ax2 = ax1.twinx()
    ax2.set_ylim(-65, 120)
    ax2.set_ylabel('Absolute Error (cm)', fontsize=40, labelpad=40)
    
    align_vicon_interp = np.interp(sys_time, vicon_sys_time, vicon)
    sr_error = np.abs(sr - align_vicon_interp)
    
    l3, = ax2.plot(sys_time, sr_error, color=sr_error_color, linestyle='-.', linewidth=4, label='Error (DS-TWR)')
    
    ax2.tick_params(axis='y', labelsize=35)
    
    lines = [l1, l2, l3]
    labels = [line.get_label() for line in lines]
    ax1.legend(lines, labels, fontsize=30, loc='upper left', frameon=False, ncol=2, columnspacing=3)
    plt.tight_layout()
    plt.show()

def dsr_sr_vicon_plot(dsr, sr, sys_time, align_vicon, vicon_sys_time, dsr_std=None, sr_std=None):
    fig, ax1 = plt.subplots(figsize=(24, 14))
    
    ax1.set_xlabel('Time (ms)', fontsize=40, labelpad=40)
    ax1.set_ylabel('Distance (cm)', fontsize=40, labelpad=40)
    
    l1, = ax1.plot(sys_time, sr, color=sr_color, linestyle='-', linewidth=4, label='DSR-IC (Divergent)')
    l2, = ax1.plot(sys_time, dsr, color=dsr_ic_color, linestyle='-', linewidth=4, label='DSR-IC (Convergent)')
    l3, = ax1.plot(vicon_sys_time, align_vicon, color=vicon_color, linestyle='-', linewidth=4, label='VICON')
    
    ax1.tick_params(axis='x', labelsize=35)
    ax1.tick_params(axis='y', labelsize=35)
    ax1.grid(True, linestyle='--', alpha=0.3)

    ax2 = ax1.twinx()
    ax2.set_ylabel('Absolute Error (cm)', fontsize=40, labelpad=40)
    
    align_vicon_interp = np.interp(sys_time, vicon_sys_time, align_vicon)
    dsr_error = np.abs(dsr - align_vicon_interp)
    sr_error = np.abs(sr - align_vicon_interp)
    
    l4, = ax2.plot(sys_time, sr_error, color=sr_error_color, linestyle='-.', linewidth=4, label='Error (Divergent)')
    l5, = ax2.plot(sys_time, dsr_error, color=dsr_ic_error_color, linestyle='-.', linewidth=4, label='Error (Convergent)')
    
    ax2.tick_params(axis='y', labelsize=35)
    
    lines = [l1, l2, l3, l4, l5]
    labels = [line.get_label() for line in lines]
    ax1.legend(lines, labels, fontsize=30, loc='upper left', frameon=False, ncol=2, columnspacing=3)
    plt.tight_layout()
    plt.show()

def ranging_plot(ranging1, ranging1_sys_time, ranging2, ranging2_sys_time, ranging3, ranging3_sys_time, vicon, vicon_sys_time, name1="RANGING1", name2="RANGING2", name3="RANGING3"):
    fig, ax1 = plt.subplots(figsize=(24, 14))
    
    ax1.set_xlabel('Time (ms)', fontsize=40, labelpad=40)
    ax1.set_ylabel('Distance (cm)', fontsize=40, labelpad=40)
    
    l1, = ax1.plot(ranging1_sys_time, ranging1, color=sr_color, linestyle='-', linewidth=4, label='DS-TWR')
    l2, = ax1.plot(ranging2_sys_time, ranging2, color=dsr_ic_color, linestyle='-', linewidth=4, label='DSR-IC')
    l3, = ax1.plot(ranging3_sys_time, ranging3, color=dsr_color, linestyle='-', linewidth=4, label='DSR')
    l4, = ax1.plot(vicon_sys_time, vicon, color=vicon_color, linestyle='-', linewidth=4, label='VICON')
    
    ax1.tick_params(axis='x', labelsize=35)
    ax1.tick_params(axis='y', labelsize=35)
    ax1.grid(True, linestyle='--', alpha=0.3)

    ax2 = ax1.twinx()
    ax2.set_ylim(-20, 120)
    ax2.set_ylabel('Absolute Error (cm)', fontsize=40, labelpad=40)
    
    align_vicon_interp = np.interp(sys_time, vicon_sys_time, align_vicon)
    sr_error = np.abs(ranging1 - align_vicon_interp)
    dsr_ic_error = np.abs(ranging2 - align_vicon_interp)
    dsr_error = np.abs(ranging3 - align_vicon_interp)
    
    l5, = ax2.plot(sys_time, sr_error, color=sr_error_color, linestyle='-.', linewidth=4, label='Error (DS-TWR)')
    l6, = ax2.plot(sys_time, dsr_ic_error, color=dsr_ic_error_color, linestyle='-.', linewidth=4, label='Error (DSR-IC)')
    l7, = ax2.plot(sys_time, dsr_error, color=dsr_error_color, linestyle='-.', linewidth=4, label='Error (DSR)')
    
    ax2.tick_params(axis='y', labelsize=35)
    
    lines = [l1, l2, l3, l4, l5, l6, l7]
    labels = [line.get_label() for line in lines]
    ax1.legend(lines, labels, fontsize=30, loc='upper left', frameon=False, ncol=2, columnspacing=4)
    plt.tight_layout()
    plt.show()

def evaluation_relative_move(align_sr_v2, align_dsr, align_cdsr, align_vicon, sys_time, vicon_sys_time, left, right):
    aligned_vicon = np.zeros_like(sys_time, dtype=float)
    for i, t in enumerate(sys_time):
        idx = np.argmin(np.abs(vicon_sys_time - t))
        aligned_vicon[i] = align_vicon[idx]

    mask = (sys_time >= left) & (sys_time <= right)
    if not np.any(mask):
        return None

    sr = align_sr_v2[mask]
    dsr = align_dsr[mask]
    cdsr = align_cdsr[mask]
    vicon = aligned_vicon[mask]

    def compute_error_metrics(predicted, ground_truth):
        ae = np.abs(predicted - ground_truth)
        mean_ae = np.mean(ae)
        max_ae = np.max(ae)
        rmse = np.sqrt(np.mean((predicted - ground_truth) ** 2))
        mre = np.mean(ae / np.maximum(ground_truth, 1e-6)) * 100
        return mean_ae, max_ae, rmse, mre

    mean_ae_sr, max_ae_sr, rmse_sr, mre_sr = compute_error_metrics(sr, vicon)
    mean_ae_dsr, max_ae_dsr, rmse_dsr, mre_dsr = compute_error_metrics(dsr, vicon)
    mean_ae_cdsr, max_ae_cdsr, rmse_cdsr, mre_cdsr = compute_error_metrics(cdsr, vicon)

    print(f"\n==== Relative Move Error in Range [{left}, {right}], length = {len(vicon)} ====")
    print(f"SR   : Mean AE = {mean_ae_sr:.3f} cm, Max AE = {max_ae_sr:.3f} cm, RMSE = {rmse_sr:.3f} cm, MRE = {mre_sr:.3f}%")
    print(f"DSR  : Mean AE = {mean_ae_dsr:.3f} cm, Max AE = {max_ae_dsr:.3f} cm, RMSE = {rmse_dsr:.3f} cm, MRE = {mre_dsr:.3f}%")
    print(f"CDSR : Mean AE = {mean_ae_cdsr:.3f} cm, Max AE = {max_ae_cdsr:.3f} cm, RMSE = {rmse_cdsr:.3f} cm, MRE = {mre_cdsr:.3f}%")

def evaluation_error(align_sr_v2, align_dsr, align_cdsr, align_vicon, sys_time, vicon_sys_time, bin_width=2, max_range=20):
    def get_error_for_eval(align_data, sys_time, align_vicon, vicon_sys_time):
        filtered, vicon_for_data = [], []
        for i, t in enumerate(sys_time):
            idx = np.argmin(np.abs(vicon_sys_time - t))
            filtered.append(align_data[i])
            vicon_for_data.append(align_vicon[idx])
        filtered = np.array(filtered)
        vicon_for_data = np.array(vicon_for_data)
        err = np.abs(filtered - vicon_for_data)
        return err

    err1 = get_error_for_eval(align_sr_v2, sys_time, align_vicon, vicon_sys_time)
    err2 = get_error_for_eval(align_dsr, sys_time, align_vicon, vicon_sys_time)
    err3 = get_error_for_eval(align_cdsr, sys_time, align_vicon, vicon_sys_time)

    bins = np.arange(bin_width, max_range + bin_width, bin_width)

    def calc_cdf(err, bins):
        cdf = []
        for b in bins:
            cdf.append(np.sum(err <= b) / len(err) * 100)
        return np.array(cdf)

    cdf_sr = calc_cdf(err1, bins)
    cdf_dsr = calc_cdf(err2, bins)
    cdf_cdsr = calc_cdf(err3, bins)

    print(f"\n{'Threshold (cm)':<15}{'SR':>12}{'DSR-IC':>12}{'DSR-IC+DS-REC':>18}")
    for i, b in enumerate(bins):
        print(f"{b:<15}{cdf_sr[i]:>12.2f}{cdf_dsr[i]:>12.2f}{cdf_cdsr[i]:>18.2f}")

# def error_histogram_plot(align_sr_v2, align_dsr, align_cdsr, align_vicon, sys_time, vicon_sys_time, 
#                          name1="DS-TWR", name2="DSR-IC", name3="DSR", bin_width=2, max_range=20, quantile=90):
#     def get_error_for_hist(align_data, sys_time, align_vicon, vicon_sys_time):
#         filtered, vicon_for_data = [], []
#         for i, t in enumerate(sys_time):
#             idx = np.argmin(np.abs(vicon_sys_time - t))
#             filtered.append(align_data[i])
#             vicon_for_data.append(align_vicon[idx])
#         filtered = np.array(filtered)
#         vicon_for_data = np.array(vicon_for_data)
#         err = np.abs(filtered - vicon_for_data)
#         return err

#     err1 = get_error_for_hist(align_sr_v2, sys_time, align_vicon, vicon_sys_time)
#     err2 = get_error_for_hist(align_dsr, sys_time, align_vicon, vicon_sys_time)
#     err3 = get_error_for_hist(align_cdsr, sys_time, align_vicon, vicon_sys_time)

#     bins = np.arange(0, max_range + bin_width, bin_width)
#     hist1, _ = np.histogram(err1, bins=bins)
#     hist2, _ = np.histogram(err2, bins=bins)
#     hist3, _ = np.histogram(err3, bins=bins)
#     hist1 = hist1 / len(err1) * 100
#     hist2 = hist2 / len(err2) * 100
#     hist3 = hist3 / len(err3) * 100

#     width = bin_width / 4
#     plt.figure(figsize=(12, 7))

#     plt.bar(bins[:-1], hist1, width=width, align="edge", alpha=0.8, label=name1, color='tab:blue')
#     plt.bar(bins[:-1] + width, hist2, width=width, align="edge", alpha=0.8, label=name2, color='tab:orange')
#     plt.bar(bins[:-1] + 2*width, hist3, width=width, align="edge", alpha=0.8, label=name3, color='tab:green')

#     q1 = np.percentile(err1, quantile)
#     q2 = np.percentile(err2, quantile)
#     q3 = np.percentile(err3, quantile)

#     idx1 = np.searchsorted(bins, q1) - 1
#     idx2 = np.searchsorted(bins, q2) - 1
#     idx3 = np.searchsorted(bins, q3) - 1

#     x1 = bins[idx1] + width/2
#     x2 = bins[idx2] + 1.5*width
#     x3 = bins[idx3] + 2.5*width

#     height1 = hist1[idx1]
#     height2 = hist2[idx2]
#     height3 = hist3[idx3]

#     unified_height = 2 * max(height1, height2, height3)
#     plt.vlines(x1, 0, unified_height, color='tab:blue', linestyle='--', linewidth=2)
#     plt.vlines(x2, 0, unified_height+2, color='tab:orange', linestyle='--', linewidth=2)
#     plt.vlines(x3, 0, unified_height+4, color='tab:green', linestyle='--', linewidth=2)

#     offset = unified_height * 0.05
#     plt.text(x1, unified_height + offset, f'{q1:.1f}cm', color='tab:blue', fontsize=25, ha='center')
#     plt.text(x2, unified_height + offset+2, f'{q2:.1f}cm', color='tab:orange', fontsize=25, ha='center')
#     plt.text(x3, unified_height + offset+4, f'{q3:.1f}cm', color='tab:green', fontsize=25, ha='center')

#     tick_positions = bins[:-1] + 1.5 * width
#     tick_labels = [f"{b}-{b + bin_width}" for b in bins[:-1]]
#     plt.xticks(tick_positions, tick_labels, fontsize=22)
#     plt.yticks(fontsize=22)
#     plt.xlabel("Absolute Error (cm)", fontsize=28, labelpad=20)
#     plt.ylabel("Percentage (%)", fontsize=28, labelpad=20)
#     plt.title(f"Error Distribution Histogram with 90% Quantile Lines", fontsize=30, pad=25)
#     plt.grid(True, linestyle="--", alpha=0.7)
#     plt.legend(fontsize=22)
#     plt.tight_layout()
#     plt.show()

def error_hill_plot(align_sr_v2, align_dsr, align_cdsr, align_vicon, sys_time, vicon_sys_time, name1="DS-TWR", name2="DSR-IC", name3="DSR", max_range=30, quantile=90):
    def get_error_for_hist(align_data, sys_time, align_vicon, vicon_sys_time):
        filtered, vicon_for_data = [], []
        for i, t in enumerate(sys_time):
            idx = np.argmin(np.abs(vicon_sys_time - t))
            filtered.append(align_data[i])
            vicon_for_data.append(align_vicon[idx])
        return np.abs(np.array(filtered) - np.array(vicon_for_data))

    err1 = get_error_for_hist(align_sr_v2, sys_time, align_vicon, vicon_sys_time)
    err2 = get_error_for_hist(align_dsr, sys_time, align_vicon, vicon_sys_time)
    err3 = get_error_for_hist(align_cdsr, sys_time, align_vicon, vicon_sys_time)

    c1 = "#88c9a4"
    c2 = "#7fc6d4"
    c3 = "#fab778"

    xs = np.linspace(0, max_range, 400)

    kde1 = gaussian_kde(err1)
    kde2 = gaussian_kde(err2)
    kde3 = gaussian_kde(err3)

    y1 = kde1(xs)
    y2 = kde2(xs)
    y3 = kde3(xs)

    plt.figure(figsize=(24, 14))

    plt.plot(xs, y1, color=c1, linewidth=3, label=name1)
    plt.plot(xs, y2, color=c2, linewidth=3, label=name2)
    plt.plot(xs, y3, color=c3, linewidth=3, label=name3)

    plt.fill_between(xs, y1, color=c1, alpha=0.25)
    plt.fill_between(xs, y2, color=c2, alpha=0.25)
    plt.fill_between(xs, y3, color=c3, alpha=0.25)

    q1 = np.percentile(err1, quantile)
    q2 = np.percentile(err2, quantile)
    q3 = np.percentile(err3, quantile)

    ymax = max(np.max(y1), np.max(y2), np.max(y3))

    def qline(q, color):
        plt.axvline(q, 0, 1, color=color, linestyle="--", linewidth=2)
        plt.text(q - 0.1, ymax * (3)/4, f"{q:.1f} cm", color=color, fontsize=20, ha="right", fontweight="bold")

    qline(q1, c1)
    qline(q2, c2)
    qline(q3, c3)

    # Labels
    plt.xlabel("Absolute Error (cm)", fontsize=40, labelpad=40)
    plt.ylabel("Percentage (%)", fontsize=40, labelpad=40)

    # Styling
    plt.xticks(fontsize=35)
    plt.yticks(fontsize=35)
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend(fontsize=30, frameon=False)

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

    # evaluation_data(align_ieee, sys_time, align_sr_v1, sys_time, align_sr_v2, sys_time, align_dsr, sys_time, align_cdsr, sys_time, align_vicon, vicon_sys_time, avg_diff)

    # sr_vicon_plot(align_sr_v2, sys_time, align_vicon, vicon_sys_time, sr_name="DS-TWR")

    # sr_vicon_plot(align_dsr, sys_time, align_vicon, vicon_sys_time, sr_name="DSR-IC")

    # dsr_sr_vicon_plot(align_cdsr, align_sr_v2, sys_time, align_vicon, vicon_sys_time)

    ranging_plot(align_sr_v2, sys_time, align_dsr, sys_time, align_cdsr, sys_time, align_vicon, vicon_sys_time, name1="DS-TWR", name2="DSR-IC", name3="DSR")

    # evaluation_relative_move(align_sr_v2, align_dsr, align_cdsr, align_vicon, sys_time, vicon_sys_time, left = 1537080, right = 1538579)

    # evaluation_error(align_sr_v2, align_dsr, align_cdsr, align_vicon, sys_time, vicon_sys_time)

    # error_histogram_plot(align_sr_v2, align_dsr, align_cdsr, align_vicon, sys_time, vicon_sys_time, name1="DS-TWR", name2="DSR-IC", name3="DSR")

    # error_hill_plot(align_sr_v2, align_dsr, align_cdsr, align_vicon, sys_time, vicon_sys_time, name1="DS-TWR", name2="DSR-IC", name3="DSR")

    # evaluation_data_motion_static(align_sr_v2, sr_v2_sys_time, align_cdsr, cdsr_sys_time, align_vicon, vicon_sys_time, avg_diff)