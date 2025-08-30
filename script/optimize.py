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
# leftbound = 1409700
# rightbound = 1423480
# leftbound = 1719676
# rightbound = 1725063
# leftbound = 2056100
# rightbound = 2066600
leftbound = 562850
rightbound = 574910
invalid_sign = -1

ranging_log_path = '../data/ranging_log.csv'
vicon_path = "../data/vicon.txt"


def read_ranging_log():
    def read_vicon_log():
        vicon_value, vicon_time = [], []
        pattern = re.compile(
            rf"\[local_(?:{local_address}) <- neighbor_(?:{neighbor_address})\]: "
            r"vicon dist = (-?\d+\.\d+), time = (\d+)"
        )
        with open(vicon_path, "r", encoding="utf-8") as f:
            for line in f:
                if (match := pattern.search(line)):
                    vicon_value.append(float(match.group(1)))
                    vicon_time.append(int(match.group(2)))
        return vicon_value, vicon_time

    cols = ["IEEE", "SR_V1", "SR_V2", "DSR", "VICON", "TIME"]
    data = pd.read_csv(ranging_log_path)
    data[cols] = data[cols].apply(pd.to_numeric, errors="coerce")
    data.dropna(inplace=True)
    ieee, sr_v1, sr_v2, dsr, vicon, sys_time = [data[c].to_numpy(float) for c in cols]
    vicon_full, vicon_full_sys_time = read_vicon_log()
    return ieee, sr_v1, sr_v2, dsr, vicon, sys_time, vicon_full, vicon_full_sys_time

def get_align_data(ieee, sr_v1, sr_v2, dsr, vicon, sys_time, vicon_full, vicon_full_sys_time):
    def get_diff_dis(ieee, sr_v1, sr_v2, dsr, sys_time, vicon_full, vicon_full_sys_time):
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
        vicon_mean = mean_in_window(vicon_full, vicon_full_sys_time)
        avg_diff = np.nanmean([vicon_mean - ieee_mean, vicon_mean - sr_v1_mean, vicon_mean - sr_v2_mean, vicon_mean - dsr_mean])
        return avg_diff

    ieee = np.array(ieee)
    sr_v1 = np.array(sr_v1)
    sr_v2 = np.array(sr_v2)
    dsr = np.array(dsr)
    vicon = np.array(vicon)
    sys_time = np.array(sys_time)
    vicon_full = np.array(vicon_full)
    vicon_full_sys_time = np.array(vicon_full_sys_time)
    avg_diff = get_diff_dis(ieee, sr_v1, sr_v2, dsr, sys_time, vicon_full, vicon_full_sys_time)

    align_ieee = ieee + avg_diff
    align_sr_v1 = sr_v1 + avg_diff
    align_sr_v2 = sr_v2 + avg_diff
    align_dsr = dsr + avg_diff
    align_vicon = vicon
    return align_ieee, align_sr_v1, align_sr_v2, align_dsr, align_vicon, avg_diff

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

def static_evaluate_params(dsr, vicon):
    def evaluate_static_params(param_compensate, param_deceleration, dsr, vicon):
        curcdsr = static_compensation_algorithm(dsr, param_compensate, param_deceleration)
        cur_mae = np.mean(np.abs(curcdsr - vicon))
        return cur_mae, param_compensate, param_deceleration, curcdsr

    compensate_rate_range = np.arange(0, 1.01, 0.01)
    deceleration_bound_range = np.arange(0, 20.1, 0.1)
    tasks = [(c, d) for c in compensate_rate_range for d in deceleration_bound_range]
    results = Parallel(n_jobs=5, verbose=0)(
        delayed(evaluate_static_params)(c, d, dsr, vicon)
        for c, d in tqdm(tasks, desc="Evaluating static parameters", ncols=100)
    )

    best_result = min(results, key=lambda x: x[0])
    best_mae, best_c, best_d, S_cdsr = best_result
    print(f"Best parameters found: COMPENSATE_RATE = {best_c:.2f}, DECELERATION_BOUND = {best_d:.2f}\n")
    return S_cdsr

def dynamic_evaluate_params(dsr, vicon):
    def evaluate_dynamic_params(param_compensate_low, param_deceleration_low,
                                param_compensate_high, param_deceleration_high,
                                param_motion_threshold, dsr, vicon):
        curcdsr = dynamic_compensation_algorithm(dsr, param_compensate_low, param_deceleration_low,
                                                 param_compensate_high, param_deceleration_high,
                                                 param_motion_threshold)
        cur_mae = np.mean(np.abs(curcdsr - vicon))
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
    results = Parallel(n_jobs=4, verbose=0)(
        delayed(evaluate_dynamic_params)(cl, dl, ch, dh, mt, dsr, vicon)
        for cl, dl, ch, dh, mt in tqdm(tasks, desc="Evaluating dynamic parameters", ncols=100)
    )

    best_result = min(results, key=lambda x: x[0])
    best_mae, cl, dl, ch, dh, mt, D_cdsr = best_result
    print(f"Best parameters found: COMPENSATE_RATE_LOW = {cl:.2f}, DECELERATION_BOUND_LOW = {dl:.2f}, "
          f"COMPENSATE_RATE_HIGH = {ch:.2f}, DECELERATION_BOUND_HIGH = {dh:.2f}, MOTION_THRESHOLD = {mt:.2f}\n")
    return D_cdsr

def generate_cdsr(dsr):
    COMPENSATE_RATE = 0.7
    DECELERATION_BOUND = 15
    S_cdsr = static_compensation_algorithm(dsr, COMPENSATE_RATE, DECELERATION_BOUND)
    COMPENSATE_RATE_LOW = 0.1
    DECELERATION_BOUND_LOW = 1
    COMPENSATE_RATE_HIGH = 0.7
    DECELERATION_BOUND_HIGH = 15
    MOTION_THRESHOLD = 3  
    D_cdsr = dynamic_compensation_algorithm(dsr, COMPENSATE_RATE_LOW, DECELERATION_BOUND_LOW, COMPENSATE_RATE_HIGH, DECELERATION_BOUND_HIGH, MOTION_THRESHOLD)
    return S_cdsr, D_cdsr

def generate_best_cdsr(dsr, vicon):
    S_cdsr = static_evaluate_params(dsr, vicon)
    D_cdsr = dynamic_evaluate_params(dsr, vicon)
    return S_cdsr, D_cdsr

def evaluation_data(align_ieee, align_sr_v1, align_sr_v2, align_dsr, S_cdsr, D_cdsr, align_vicon, avg_diff):
    def compute_error_metrics(predicted, ground_truth):
        if len(predicted) == 0 or len(ground_truth) == 0:
            return np.nan, np.nan, np.nan, np.nan
        ae = np.abs(predicted - ground_truth) 
        mean_ae = np.mean(ae)                  
        max_ae = np.max(ae)                    
        rmse = np.sqrt(np.mean((predicted - ground_truth) ** 2))  
        ground_truth_safe = np.where(ground_truth == 0, np.finfo(float).eps, ground_truth)
        mean_re = np.mean(ae / ground_truth_safe) * 100 
        return mean_ae, max_ae, rmse, mean_re

    def single_valid_filter_and_metrics(align_data, align_vicon, avg_diff, invalid_sign):
        valid_indices = align_data != (avg_diff + invalid_sign)
        filtered_data = align_data[valid_indices]
        filtered_vicon = align_vicon[valid_indices]
        total_count = len(align_data)
        invalid_rate = ((total_count - len(filtered_data)) / total_count) * 100 if total_count > 0 else np.nan
        return filtered_data, filtered_vicon, invalid_rate

    def common_valid_filter_and_metrics(ieee, sr_v1, sr_v2, dsr, S_cdsr, D_cdsr, align_vicon, avg_diff, invalid_sign):
        common_valid_indices = (ieee != (avg_diff + invalid_sign)) & (sr_v1 != (avg_diff + invalid_sign)) & (sr_v2 != (avg_diff + invalid_sign)) & (dsr != (avg_diff + invalid_sign)) & (S_cdsr != (avg_diff + invalid_sign)) & (D_cdsr != (avg_diff + invalid_sign))
        f_ieee = ieee[common_valid_indices]
        f_sr_v1 = sr_v1[common_valid_indices]
        f_sr_v2 = sr_v2[common_valid_indices]
        f_dsr = dsr[common_valid_indices]
        f_S_cdsr = S_cdsr[common_valid_indices]
        f_D_cdsr = D_cdsr[common_valid_indices]
        f_vicon = align_vicon[common_valid_indices]
        total_count = len(ieee)
        invalid_rate = ((total_count - len(f_ieee)) / total_count) * 100 if total_count > 0 else np.nan
        return f_ieee, f_sr_v1, f_sr_v2, f_dsr, f_S_cdsr, f_D_cdsr, f_vicon, invalid_rate

    ieee_f, vicon_for_ieee, invalid_rate_ieee = single_valid_filter_and_metrics(align_ieee, align_vicon, avg_diff, invalid_sign)
    sr_v1_f, vicon_for_sr_v1, invalid_rate_sr_v1 = single_valid_filter_and_metrics(align_sr_v1, align_vicon, avg_diff, invalid_sign)
    sr_v2_f, vicon_for_sr_v2, invalid_rate_sr_v2 = single_valid_filter_and_metrics(align_sr_v2, align_vicon, avg_diff, invalid_sign)
    dsr_f, vicon_for_dsr, invalid_rate_dsr = single_valid_filter_and_metrics(align_dsr, align_vicon, avg_diff, invalid_sign)
    S_cdsr_f, vicon_for_S_cdsr, invalid_rate_S_cdsr = single_valid_filter_and_metrics(S_cdsr, align_vicon, avg_diff, invalid_sign)
    D_cdsr_f, vicon_for_D_cdsr, invalid_rate_D_cdsr = single_valid_filter_and_metrics(D_cdsr, align_vicon, avg_diff, invalid_sign)

    mean_ae_ieee, max_ae_ieee, rmse_ieee, mre_ieee = compute_error_metrics(ieee_f, vicon_for_ieee)
    mean_ae_sr_v1, max_ae_sr_v1, rmse_sr_v1, mre_sr_v1 = compute_error_metrics(sr_v1_f, vicon_for_sr_v1)
    mean_ae_sr_v2, max_ae_sr_v2, rmse_sr_v2, mre_sr_v2 = compute_error_metrics(sr_v2_f, vicon_for_sr_v2)
    mean_ae_dsr, max_ae_dsr, rmse_dsr, mre_dsr = compute_error_metrics(dsr_f, vicon_for_dsr)
    mean_ae_S_cdsr, max_ae_S_cdsr, rmse_S_cdsr, mre_S_cdsr = compute_error_metrics(S_cdsr_f, vicon_for_S_cdsr)
    mean_ae_D_cdsr, max_ae_D_cdsr, rmse_D_cdsr, mre_D_cdsr = compute_error_metrics(D_cdsr_f, vicon_for_D_cdsr)

    print("==== Error metrics for all valid data ====")
    print(f"IEEE  : Mean AE(平均绝对误差) = {mean_ae_ieee:.3f} cm, Max AE(最大绝对误差) = {max_ae_ieee:.3f} cm, RMSE(均方根误差) = {rmse_ieee:.3f} cm, MRE(平均相对误差) = {mre_ieee:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_ieee:.2f}%")
    print(f"SR_V1 : Mean AE(平均绝对误差) = {mean_ae_sr_v1:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr_v1:.3f} cm, RMSE(均方根误差) = {rmse_sr_v1:.3f} cm, MRE(平均相对误差) = {mre_sr_v1:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_sr_v1:.2f}%")
    print(f"SR_V2 : Mean AE(平均绝对误差) = {mean_ae_sr_v2:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr_v2:.3f} cm, RMSE(均方根误差) = {rmse_sr_v2:.3f} cm, MRE(平均相对误差) = {mre_sr_v2:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_sr_v2:.2f}%")
    print(f"DSR   : Mean AE(平均绝对误差) = {mean_ae_dsr:.3f} cm, Max AE(最大绝对误差) = {max_ae_dsr:.3f} cm, RMSE(均方根误差) = {rmse_dsr:.3f} cm, MRE(平均相对误差) = {mre_dsr:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_dsr:.2f}%")
    print(f"S-CDSR: Mean AE(平均绝对误差) = {mean_ae_S_cdsr:.3f} cm, Max AE(最大绝对误差) = {max_ae_S_cdsr:.3f} cm, RMSE(均方根误差) = {rmse_S_cdsr:.3f} cm, MRE(平均相对误差) = {mre_S_cdsr:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_S_cdsr:.2f}%")
    print(f"D-CDSR: Mean AE(平均绝对误差) = {mean_ae_D_cdsr:.3f} cm, Max AE(最大绝对误差) = {max_ae_D_cdsr:.3f} cm, RMSE(均方根误差) = {rmse_D_cdsr:.3f} cm, MRE(平均相对误差) = {mre_D_cdsr:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_D_cdsr:.2f}%")

    ieee_c, sr_v1_c, sr_v2_c, dsr_c, S_cdsr_c, D_cdsr_c, vicon_c, invalid_rate_common = common_valid_filter_and_metrics(align_ieee, align_sr_v1, align_sr_v2, align_dsr, S_cdsr, D_cdsr, align_vicon, avg_diff, invalid_sign)

    mean_ae_sr_v2_c, max_ae_sr_v2_c, rmse_sr_v2_c, mre_sr_v2_c = compute_error_metrics(sr_v2_c, vicon_c)
    mean_ae_dsr_c, max_ae_dsr_c, rmse_dsr_c, mre_dsr_c = compute_error_metrics(dsr_c, vicon_c)
    mean_ae_S_cdsr_c, max_ae_S_cdsr_c, rmse_S_cdsr_c, mre_S_cdsr_c = compute_error_metrics(S_cdsr_c, vicon_c)
    mean_ae_D_cdsr_c, max_ae_D_cdsr_c, rmse_D_cdsr_c, mre_D_cdsr_c = compute_error_metrics(D_cdsr_c, vicon_c)

    print("\n==== Error metrics for common valid data ====")
    print(f"CLASSIC: Mean AE(平均绝对误差) = {mean_ae_sr_v2_c:.3f} cm, Max AE(最大绝对误差) = {max_ae_sr_v2_c:.3f} cm, RMSE(均方根误差) = {rmse_sr_v2_c:.3f} cm, MRE(平均相对误差) = {mre_sr_v2_c:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_common:.2f}%")
    print(f"DSR    : Mean AE(平均绝对误差) = {mean_ae_dsr_c:.3f} cm, Max AE(最大绝对误差) = {max_ae_dsr_c:.3f} cm, RMSE(均方根误差) = {rmse_dsr_c:.3f} cm, MRE(平均相对误差) = {mre_dsr_c:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_common:.2f}%")
    print(f"S-CDSR : Mean AE(平均绝对误差) = {mean_ae_S_cdsr_c:.3f} cm, Max AE(最大绝对误差) = {max_ae_S_cdsr_c:.3f} cm, RMSE(均方根误差) = {rmse_S_cdsr_c:.3f} cm, MRE(平均相对误差) = {mre_S_cdsr_c:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_common:.2f}%")
    print(f"D-CDSR : Mean AE(平均绝对误差) = {mean_ae_D_cdsr_c:.3f} cm, Max AE(最大绝对误差) = {max_ae_D_cdsr_c:.3f} cm, RMSE(均方根误差) = {rmse_D_cdsr_c:.3f} cm, MRE(平均相对误差) = {mre_D_cdsr_c:.3f}%, Invalid Rate(计算失败率) = {invalid_rate_common:.2f}%")

def ranging_plot(ranging1, ranging1_sys_time, ranging2, ranging2_sys_time, ranging3, ranging3_sys_time, vicon, vicon_sys_time, name1="RANGING1", name2="RANGING2", name3="RANGING3"):
    plt.plot(ranging1_sys_time, ranging1, color='#4A90E2', label=name1, linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(ranging2_sys_time, ranging2, color="#E4491E", label=name2, linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(ranging3_sys_time, ranging3, color="#FF7B00", label=name3, linestyle='--', marker='x', markersize=4, linewidth=1.5)
    plt.plot(vicon_sys_time, vicon, color="#9DF423", label='VICON', alpha=0.8, linestyle='-', marker='o', markersize=4, linewidth=1.5)
    plt.xlabel('Time (ms)') 
    plt.ylabel('Distance Measurement')
    plt.title(f'{name1} vs {name2} vs VICON Distance Measurements Over Absolute Time')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    ieee, sr_v1, sr_v2, dsr, vicon, sys_time, vicon_full, vicon_full_sys_time = read_ranging_log()
    align_ieee, align_sr_v1, align_sr_v2, align_dsr, align_vicon, avg_diff = get_align_data(ieee, sr_v1, sr_v2, dsr, vicon, sys_time, vicon_full, vicon_full_sys_time)

    S_cdsr, D_cdsr = generate_cdsr(align_dsr)

    # S_cdsr, D_cdsr = generate_best_cdsr(align_dsr, align_vicon)

    evaluation_data(align_ieee, align_sr_v1, align_sr_v2, align_dsr, S_cdsr, D_cdsr, align_vicon, avg_diff)
    ranging_plot(align_sr_v2, sys_time, align_dsr, sys_time, S_cdsr, sys_time, vicon_full, vicon_full_sys_time, "SR_V2", "DSR", "CDSR")