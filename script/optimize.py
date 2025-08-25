import re
from joblib import Parallel, delayed
import pandas as pd
from tqdm import tqdm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
matplotlib.use('TkAgg')


# This script reads data from ranging log.csv and adjusts the compensation coefficient appropriately to optimize ranging accuracy, make sure COMPENSATE_ENABLE is closed.


local_address = 2
neighbor_address = 3
leftbound = 1409700
rightbound = 1423480
# leftbound = 1719676
# rightbound = 1725063
# leftbound = 2056100
# rightbound = 2066600
invalid_sign = -1

# ---temp
ranging_Log_path = '../data/ranging_Log.csv'
vicon_path = "../data/vicon.txt"
# ---processed
# file_num = "5"
# ranging_Log_path = "../../../../../../data/processed/" + file_num + ".csv"
# vicon_path = "../../../../../../data/processed/" + file_num + ".txt"
# ---packed loss
# csv_num = "1_0"
# txt_num = "1"
# ranging_Log_path = "../../../../../../data/packedloss/" + csv_num + ".csv"
# vicon_path = "../../../../../../data/packedloss/" + txt_num + ".txt"
# ---period
# csv_num = "1_400"
# txt_num = "1"
# ranging_Log_path = "../../../../../../data/period/" + csv_num + ".csv"
# vicon_path = "../../../../../../data/period/" + txt_num + ".txt"


def read_log():  
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
    
    data = pd.read_csv(ranging_Log_path)
    data['DSR'] = pd.to_numeric(data['DSR'], errors='coerce')
    data['SR'] = pd.to_numeric(data['SR'], errors='coerce')
    data['VICON'] = pd.to_numeric(data['VICON'], errors='coerce')
    data['TIME'] = pd.to_numeric(data['TIME'], errors='coerce')
    data.dropna(inplace=True)

    dsr = data['DSR'].to_numpy(dtype=float)
    sr = data['SR'].to_numpy(dtype=float)
    vicon_sample = data['VICON'].to_numpy(dtype=float)
    sys_time = data['TIME'].to_numpy(dtype=float)
    vicon, vicon_sys_time = read_vicon_Log()

    return dsr, sr, vicon_sample, sys_time, vicon, vicon_sys_time

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

    return align_sr, align_dsr, avg_diff

def evaluation_data(dsr, sr, S_cdsr, D_cdsr, vicon, avg_diff):
    def compute_error_metrics(predicted, ground_truth):
        if len(predicted) == 0 or len(ground_truth) == 0:
            return np.nan, np.nan, np.nan, np.nan
        ae = np.abs(predicted - ground_truth)
        mean_ae = np.mean(ae)
        max_ae = np.max(ae)
        rmse = np.sqrt(np.mean((predicted - ground_truth) ** 2))
        mean_re = np.mean(np.abs(predicted - ground_truth) / ground_truth) * 100
        return mean_ae, max_ae, rmse, mean_re

    sr_filtered = []
    vicon_for_sr = []
    for i in range(len(sr)):
        if sr[i] == avg_diff + invalid_sign:
            continue
        sr_filtered.append(sr[i])
        vicon_for_sr.append(vicon[i])
    sr_filtered = np.array(sr_filtered)
    vicon_for_sr = np.array(vicon_for_sr)
    invalid_rate_sr = (len(sr) - len(sr_filtered)) / len(sr) * 100 if len(sr) > 0 else np.nan

    dsr_filtered = []
    vicon_for_dsr = []
    for i in range(len(dsr)):
        if dsr[i] == avg_diff + invalid_sign:
            continue
        dsr_filtered.append(dsr[i])
        vicon_for_dsr.append(vicon[i])
    dsr_filtered = np.array(dsr_filtered)
    vicon_for_dsr = np.array(vicon_for_dsr)
    invalid_rate_dsr = (len(dsr) - len(dsr_filtered)) / len(dsr) * 100 if len(dsr) > 0 else np.nan

    mean_ae_sr, max_ae_sr, rmse_sr, mre_sr = compute_error_metrics(sr_filtered, vicon_for_sr)
    mean_ae_dsr, max_ae_dsr, rmse_dsr, mre_dsr = compute_error_metrics(dsr_filtered, vicon_for_dsr)
    mean_ae_s_cdsr, max_ae_s_cdsr, rmse_s_cdsr, mre_s_cdsr = compute_error_metrics(S_cdsr, vicon_for_dsr)
    mean_ae_d_cdsr, max_ae_d_cdsr, rmse_d_cdsr, mre_d_cdsr = compute_error_metrics(D_cdsr, vicon_for_dsr)

    print(f"SR:     Mean AE(平均绝对误差) = {mean_ae_sr:.3f} cm, "
          f"Max AE(最大绝对误差) = {max_ae_sr:.3f} cm, "
          f"RMSE(均方根误差) = {rmse_sr:.3f} cm, "
          f"MRE(平均相对误差) = {mre_sr:.3f}%, "
          f"Invalid Rate(计算失败率) = {invalid_rate_sr:.2f}%")
    print(f"DSR:    Mean AE(平均绝对误差) = {mean_ae_dsr:.3f} cm, "
          f"Max AE(最大绝对误差) = {max_ae_dsr:.3f} cm, "
          f"RMSE(均方根误差) = {rmse_dsr:.3f} cm, "
          f"MRE(平均相对误差) = {mre_dsr:.3f}%, "
          f"Invalid Rate(计算失败率) = {invalid_rate_dsr:.2f}%")
    print(f"S-CDSR: Mean AE(平均绝对误差) = {mean_ae_s_cdsr:.3f} cm, "
          f"Max AE(最大绝对误差) = {max_ae_s_cdsr:.3f} cm, "
          f"RMSE(均方根误差) = {rmse_s_cdsr:.3f} cm, "
          f"MRE(平均相对误差) = {mre_s_cdsr:.3f}%, "
          f"Invalid Rate(计算失败率) = {invalid_rate_dsr:.2f}%")
    print(f"D-CDSR: Mean AE(平均绝对误差) = {mean_ae_d_cdsr:.3f} cm, "
          f"Max AE(最大绝对误差) = {max_ae_d_cdsr:.3f} cm, "
          f"RMSE(均方根误差) = {rmse_d_cdsr:.3f} cm, "
          f"MRE(平均相对误差) = {mre_d_cdsr:.3f}%, "
          f"Invalid Rate(计算失败率) = {invalid_rate_dsr:.2f}%")

def static_compensation_algorithm(distance_List, compensate_rate, deceleration_bound):
    last_dis_Calculate = None
    compensate_unit = None
    distance_List_Processed = []

    for i in range(len(distance_List)):
        dis_Calculate = distance_List[i]

        if last_dis_Calculate is None:
            last_dis_Calculate = dis_Calculate
            distance = dis_Calculate
        elif compensate_unit is None:
            compensate_unit = dis_Calculate - last_dis_Calculate
            last_dis_Calculate = dis_Calculate
            distance = dis_Calculate
        else:
            dis_Compensate = compensate_unit
            compensate_unit = dis_Calculate - last_dis_Calculate
            last_dis_Calculate = dis_Calculate

            if abs(dis_Compensate) - abs(compensate_unit) > deceleration_bound or dis_Compensate * compensate_unit <= 0:
                distance = dis_Calculate
            else:
                distance = dis_Calculate + dis_Compensate * compensate_rate
        distance_List_Processed.append(distance)

    return np.array(distance_List_Processed)

def dynamic_compensation_algorithm(distance_List, compensate_rate_low, deceleration_bound_low, compensate_rate_high, deceleration_bound_high, motion_threshold):
    last_dis_Calculate = None
    compensate_unit = None
    distance_List_Processed = []

    for i in range(len(distance_List)):
        dis_Calculate = distance_List[i]

        if last_dis_Calculate is None:
            last_dis_Calculate = dis_Calculate
            distance = dis_Calculate
        elif compensate_unit is None:
            compensate_unit = dis_Calculate - last_dis_Calculate
            last_dis_Calculate = dis_Calculate
            distance = dis_Calculate
        else:
            dis_Compensate = compensate_unit
            compensate_unit = dis_Calculate - last_dis_Calculate
            last_dis_Calculate = dis_Calculate

            avg_compensate_unit = (dis_Compensate + compensate_unit) / 2
            deceleration_bound = deceleration_bound_low if abs(avg_compensate_unit) < motion_threshold else deceleration_bound_high
            compensate_rate = compensate_rate_low if abs(avg_compensate_unit) < motion_threshold else compensate_rate_high

            if abs(dis_Compensate) - abs(compensate_unit) > deceleration_bound or dis_Compensate * compensate_unit <= 0:
                distance = dis_Calculate
            else:
                distance = dis_Calculate + dis_Compensate * compensate_rate
        distance_List_Processed.append(distance)

    return np.array(distance_List_Processed)

def ranging_plot(cdsr, dsr, sr, sys_time, vicon, vicon_sys_time):
    plt.figure(figsize=(12, 6))
    plt.plot(sys_time, sr, color="#4A90E2", label='SR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(sys_time, dsr, color="#E4491E", label='DSR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(sys_time, cdsr, color="#FF7B00", label='CDSR', linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(vicon_sys_time, vicon, color="#9DF423", label='VICON', alpha=0.8, linestyle='-', marker='o', markersize=4, linewidth=2)
    plt.title('Ranging Comparison Over Time')
    plt.xlabel('Sample Index')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def static_set_param(COMPENSATE_RATE, DECELERATION_BOUND, dsr, sr, sys_time, vicon, vicon_sys_time):
    S_cdsr = static_compensation_algorithm(dsr, COMPENSATE_RATE, DECELERATION_BOUND)
    # ranging_plot(S_cdsr, dsr, sr, sys_time, vicon, vicon_sys_time)
    return S_cdsr

def dynamic_set_param(COMPENSATE_RATE_LOW, DECELERATION_BOUND_LOW, COMPENSATE_RATE_HIGH, DECELERATION_BOUND_HIGH, MOTION_THRESHOLD, dsr, sr, sys_time, vicon, vicon_sys_time):
    D_cdsr = dynamic_compensation_algorithm(dsr, COMPENSATE_RATE_LOW, DECELERATION_BOUND_LOW, COMPENSATE_RATE_HIGH, DECELERATION_BOUND_HIGH, MOTION_THRESHOLD)
    # ranging_plot(D_cdsr, dsr, sr, sys_time, vicon, vicon_sys_time)
    return D_cdsr

def static_evaluate_params(dsr, sr, vicon_sample, sys_time, vicon, vicon_sys_time):
    def evaluate_static_params(param_compensate, param_deceleration, dsr, vicon_sample):
        curcdsr = static_compensation_algorithm(dsr, param_compensate, param_deceleration)
        cur_mae = np.mean(np.abs(curcdsr - vicon_sample))
        return cur_mae, param_compensate, param_deceleration, curcdsr

    compensate_rate_range = np.arange(0, 1.01, 0.01)
    deceleration_bound_range = np.arange(0, 20.1, 0.1)

    tasks = [(c, d) for c in compensate_rate_range for d in deceleration_bound_range]

    results = Parallel(n_jobs=5, verbose=0)(
        delayed(evaluate_static_params)(c, d, dsr, vicon_sample)
        for c, d in tqdm(tasks, desc="Evaluating static parameters", ncols=100)
    )

    best_result = min(results, key=lambda x: x[0])
    best_mae, best_c, best_d, S_cdsr = best_result

    print(f"\nBest parameters found: COMPENSATE_RATE = {best_c:.2f}, DECELERATION_BOUND = {best_d:.2f}")
    # ranging_plot(S_cdsr, dsr, sr, sys_time, vicon, vicon_sys_time)
    return S_cdsr


def dynamic_evaluate_params(dsr, sr, vicon_sample, sys_time, vicon, vicon_sys_time):
    def evaluate_dynamic_params(param_compensate_low, param_deceleration_low,
                                param_compensate_high, param_deceleration_high,
                                param_motion_threshold, dsr, vicon_sample):
        curcdsr = dynamic_compensation_algorithm(dsr, param_compensate_low, param_deceleration_low,
                                                 param_compensate_high, param_deceleration_high,
                                                 param_motion_threshold)
        cur_mae = np.mean(np.abs(curcdsr - vicon_sample))
        return cur_mae, param_compensate_low, param_deceleration_low, param_compensate_high, param_deceleration_high, param_motion_threshold, curcdsr

    compensate_rate_range_low = np.arange(0, 1.1, 0.1)
    deceleration_bound_range_low = np.arange(0, 11, 1)
    compensate_rate_range_high = np.arange(0, 1.1, 0.1)
    deceleration_bound_range_high = np.arange(0, 21, 1)
    motion_threshold = np.arange(0, 5, 1)

    tasks = [(cl, dl, ch, dh, mt)
             for cl in compensate_rate_range_low
             for dl in deceleration_bound_range_low
             for ch in compensate_rate_range_high
             for dh in deceleration_bound_range_high
             for mt in motion_threshold]

    results = Parallel(n_jobs=5, verbose=0)(
        delayed(evaluate_dynamic_params)(cl, dl, ch, dh, mt, dsr, vicon_sample)
        for cl, dl, ch, dh, mt in tqdm(tasks, desc="Evaluating dynamic parameters", ncols=100)
    )

    best_result = min(results, key=lambda x: x[0])
    best_mae, cl, dl, ch, dh, mt, D_cdsr = best_result

    print(f"\nBest parameters found: COMPENSATE_RATE_LOW = {cl:.2f}, DECELERATION_BOUND_LOW = {dl:.2f}, "
          f"COMPENSATE_RATE_HIGH = {ch:.2f}, DECELERATION_BOUND_HIGH = {dh:.2f}, MOTION_THRESHOLD = {mt:.2f}")
    # ranging_plot(D_cdsr, dsr, sr, sys_time, vicon, vicon_sys_time)
    return D_cdsr


if __name__ == '__main__':
    dsr, sr, vicon_sample, sys_time, vicon, vicon_sys_time = read_log()

    align_sr, align_dsr, avg_diff = get_align_data(sr, sys_time, dsr, sys_time, vicon, vicon_sys_time)

    COMPENSATE_RATE = 0.7
    DECELERATION_BOUND = 15
    S_cdsr = static_set_param(COMPENSATE_RATE, DECELERATION_BOUND, align_dsr, align_sr, sys_time, vicon, vicon_sys_time)
    COMPENSATE_RATE_LOW = 0.1
    DECELERATION_BOUND_LOW = 1
    COMPENSATE_RATE_HIGH = 0.7
    DECELERATION_BOUND_HIGH = 15
    MOTION_THRESHOLD = 3
    D_cdsr = dynamic_set_param(COMPENSATE_RATE_LOW, DECELERATION_BOUND_LOW, COMPENSATE_RATE_HIGH, DECELERATION_BOUND_HIGH, MOTION_THRESHOLD, align_dsr, align_sr, sys_time, vicon, vicon_sys_time)
    evaluation_data(align_dsr, align_sr, S_cdsr, D_cdsr, vicon_sample, avg_diff)

    # S_cdsr = static_evaluate_params(align_dsr, align_sr, vicon_sample, sys_time, vicon, vicon_sys_time)
    # D_cdsr = dynamic_evaluate_params(align_dsr, align_sr, vicon_sample, sys_time, vicon, vicon_sys_time)
    # evaluation_data(align_dsr, align_sr, S_cdsr, D_cdsr, vicon_sample, avg_diff)