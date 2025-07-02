# import re
# import matplotlib
# matplotlib.use('TkAgg')  
# import numpy as np
# import matplotlib.pyplot as plt
# from collections import defaultdict

# def read_data(file_path, address=None):
#     data = []
#     current_entry = {}
#     patterns = {
#         'ModifiedD': r'ModifiedD\s*=\s*([\d.]+)',
#         'ClassicD': r'ClassicD\s*=\s*([\d.]+)',
#         'TrueD': r'TrueD\s*=\s*([\d.]+)',
#         'time': r'time\s*=\s*(\d+)'
#     }
#     address_pattern = r'\[current_(\d+)\]:'  

#     with open(file_path, 'r') as f:
#         for line in f:
#             line = line.strip()
            
#             if address is not None:
#                 addr_match = re.search(address_pattern, line)
#                 if not addr_match or int(addr_match.group(1)) != address:
#                     continue
            
#             for key, pattern in patterns.items():
#                 match = re.search(pattern, line)
#                 if match:
#                     value = float(match.group(1)) if key != 'time' else int(match.group(1))
#                     current_entry[key] = value
#                     if len(current_entry) == 4:
#                         data.append(tuple(current_entry.values()))
#                         current_entry = {}
#     return np.array(data)

# def find_most_common_equal_values(data, precision=2):
#     rounded_data = np.round(data, precision)
#     value_counts = defaultdict(int)
    
#     for value in rounded_data:
#         value_counts[value] += 1
    
#     if not value_counts:
#         return None, 0
    
#     most_common_value = max(value_counts.items(), key=lambda x: x[1])[0]
#     return most_common_value, value_counts[most_common_value]

# def calculate_offsets(data, left_idx, right_idx, mode=1):
#     if left_idx < 0 or right_idx >= len(data) or left_idx > right_idx:
#         raise ValueError(f"Invalid index range: left={left_idx}, right={right_idx}")
    
#     selected_data = data[left_idx:right_idx+1]
#     mod_data = selected_data[:, 0]
#     cls_data = selected_data[:, 1]
#     tru_data = selected_data[:, 2]
    
#     if mode == 0:
#         mod_mean = np.mean(mod_data)
#         cls_mean = np.mean(cls_data)
#         tru_mean = np.mean(tru_data)
        
#         diffs = {
#             'mod': abs(mod_mean - tru_mean),
#             'cls': abs(cls_mean - tru_mean),
#             'tru': 0  
#         }
#         selected_base = min(diffs, key=diffs.get)
        
#         if selected_base == 'mod':
#             base_value = mod_mean
#             mod_offset = 0
#             cls_offset = cls_mean - mod_mean
#             tru_offset = tru_mean - mod_mean
#         elif selected_base == 'cls':
#             base_value = cls_mean
#             mod_offset = mod_mean - cls_mean
#             cls_offset = 0
#             tru_offset = tru_mean - cls_mean
#         else:  
#             base_value = tru_mean
#             mod_offset = mod_mean - tru_mean
#             cls_offset = cls_mean - tru_mean
#             tru_offset = 0
            
#         base_type = f"Mean ({selected_base})"
        
#     elif mode == 1:

#         mod_common, mod_count = find_most_common_equal_values(mod_data)
#         cls_common, cls_count = find_most_common_equal_values(cls_data)
#         tru_common, tru_count = find_most_common_equal_values(tru_data)
        
#         counts = {
#             'mod': mod_count,
#             'cls': cls_count,
#             'tru': tru_count
#         }
#         selected_base = max(counts.items(), key=lambda x: x[1])[0]
        
#         if selected_base == 'mod':
#             base_value = mod_common
#             mod_offset = 0
#             cls_offset = cls_common - mod_common
#             tru_offset = tru_common - mod_common
#         elif selected_base == 'cls':
#             base_value = cls_common
#             mod_offset = mod_common - cls_common
#             cls_offset = 0
#             tru_offset = tru_common - cls_common
#         else: 
#             base_value = tru_common
#             mod_offset = mod_common - tru_common
#             cls_offset = cls_common - tru_common
#             tru_offset = 0
            
#         base_type = f"Most Common ({selected_base})"
        
#     else:
#         raise ValueError(f"Unsupported alignment mode: {mode}")
    
#     print(f"Selected base: {base_type} (value={base_value:.4f})")
#     print(f"Alignment parameters: ModifiedD Offset = {mod_offset:.4f}, ClassicD Offset = {cls_offset:.4f}")
#     return mod_offset, cls_offset, base_value, base_type

# def plot_adjustment(data, left_idx, right_idx, mode=1):
#     plt.figure(figsize=(12, 8))
    
#     time = data[:, 3]
#     mod_data = data[:, 0]
#     cls_data = data[:, 1]
#     tru_data = data[:, 2]
    
#     mod_offset, cls_offset, base_value, base_type = calculate_offsets(data, left_idx, right_idx, mode)
    
#     adjusted_mod = mod_data - mod_offset
#     adjusted_cls = cls_data - cls_offset
    
#     plt.plot(time, tru_data, 'g-^', label='TrueD', alpha=0.8, linewidth=1.5)
#     plt.plot(time, adjusted_mod, 'b-o', label=f'ModifiedD (Offset={mod_offset:.2f})', alpha=0.8)
#     plt.plot(time, adjusted_cls, 'r-s', label=f'ClassicD (Offset={cls_offset:.2f})', alpha=0.8)
    
#     plt.axhline(y=base_value, color='purple', linestyle='--', 
#                 label=f'Base Value ({base_type}: {base_value:.2f})')
#     plt.axvspan(time[left_idx], time[right_idx], color='yellow', 
#                 alpha=0.2, label='Alignment Region')
    
#     mode_text = "Mean Alignment" if mode == 0 else "Most Common Value Alignment"
#     plt.title(f'Vertical Alignment Using {mode_text}')
#     plt.xlabel('Time')
#     plt.ylabel('Distance')
#     plt.legend(loc='upper left')
#     plt.grid(True, linestyle='--', alpha=0.7)
#     plt.tight_layout()
    
#     filename = 'data/data.png' 
#     plt.savefig(filename)
#     plt.show()

# if __name__ == "__main__":

#     file_path = 'data/dataLog.txt'  
#     LEFT_IDX = 0                   # Alignment interval start index
#     RIGHT_IDX = 35                 # Alignment interval end index
#     ADDRESS = 34698                # Device address
#     MODE = 0                       # Alignment mode: 0=Mean alignment, 1=Mode alignment
    
#     data = read_data(file_path, address=ADDRESS)
#     if len(data) == 0:
#         print("Warning: No valid data found!")
#         exit()
    
#     RIGHT_IDX = min(RIGHT_IDX, len(data)-1)
#     LEFT_IDX = max(LEFT_IDX, 0)
    
#     if LEFT_IDX > RIGHT_IDX:
#         print(f"Warning: Start index ({LEFT_IDX}) > end index ({RIGHT_IDX}), swapping...")
#         LEFT_IDX, RIGHT_IDX = RIGHT_IDX, LEFT_IDX
    
#     plot_adjustment(data, LEFT_IDX, RIGHT_IDX, MODE)

import re
import matplotlib
matplotlib.use('TkAgg')  
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

def read_data(file_path, address=None):
    data = []
    current_entry = {}
    patterns = {
        'ModifiedD': r'ModifiedD\s*=\s*([\d.]+)',
        'TrueD': r'TrueD\s*=\s*([\d.]+)',
        'time': r'time\s*=\s*(\d+)'
    }
    address_pattern = r'\[current_(\d+)\]:'  

    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            
            if address is not None:
                addr_match = re.search(address_pattern, line)
                if not addr_match or int(addr_match.group(1)) != address:
                    continue
            
            for key, pattern in patterns.items():
                match = re.search(pattern, line)
                if match:
                    value = float(match.group(1)) if key != 'time' else int(match.group(1))
                    current_entry[key] = value
                    if len(current_entry) == 3: 
                        data.append((current_entry['ModifiedD'], current_entry['TrueD'], current_entry['time']))
                        current_entry = {}
    return np.array(data)

def find_most_common_equal_values(data, precision=2):
    rounded_data = np.round(data, precision)
    value_counts = defaultdict(int)
    
    for value in rounded_data:
        value_counts[value] += 1
    
    if not value_counts:
        return None, 0
    
    most_common_value = max(value_counts.items(), key=lambda x: x[1])[0]
    return most_common_value, value_counts[most_common_value]

def calculate_offsets(data, left_idx, right_idx, mode=1):
    if left_idx < 0 or right_idx >= len(data) or left_idx > right_idx:
        raise ValueError(f"Invalid index range: left={left_idx}, right={right_idx}")
    
    selected_data = data[left_idx:right_idx+1]
    mod_data = selected_data[:, 0]  
    tru_data = selected_data[:, 1]
    
    if mode == 0:
        mod_mean = np.mean(mod_data)
        tru_mean = np.mean(tru_data)
        
        diffs = {
            'mod': abs(mod_mean - tru_mean),
            'tru': 0  
        }
        selected_base = min(diffs, key=diffs.get)
        
        if selected_base == 'mod':
            base_value = mod_mean
            mod_offset = 0
            tru_offset = tru_mean - mod_mean
        else:  
            base_value = tru_mean
            mod_offset = mod_mean - tru_mean
            tru_offset = 0
            
        base_type = f"Mean ({selected_base})"
        
    elif mode == 1:
        mod_common, mod_count = find_most_common_equal_values(mod_data)
        tru_common, tru_count = find_most_common_equal_values(tru_data)
        
        counts = {
            'mod': mod_count,
            'tru': tru_count
        }
        selected_base = max(counts.items(), key=lambda x: x[1])[0]
        
        if selected_base == 'mod':
            base_value = mod_common
            mod_offset = 0
            tru_offset = tru_common - mod_common
        else: 
            base_value = tru_common
            mod_offset = mod_common - tru_common
            tru_offset = 0
            
        base_type = f"Most Common ({selected_base})"
        
    else:
        raise ValueError(f"Unsupported alignment mode: {mode}")
    
    print(f"Selected base: {base_type} (value={base_value:.4f})")
    print(f"Alignment parameters: ModifiedD Offset = {mod_offset:.4f}")
    return mod_offset, tru_offset, base_value, base_type

def plot_adjustment(data, left_idx, right_idx, mode=1):
    plt.figure(figsize=(12, 8))
    
    time = data[:, 2]
    mod_data = data[:, 0] 
    tru_data = data[:, 1] 
    
    mod_offset, tru_offset, base_value, base_type = calculate_offsets(data, left_idx, right_idx, mode)
    
    adjusted_mod = mod_data - mod_offset
    
    plt.plot(time, tru_data, 'g-o', label='TrueD', alpha=0.8, linewidth=1.5)
    plt.plot(time, adjusted_mod, 'r-o', label=f'ModifiedD (Offset={mod_offset:.2f})', alpha=0.8)
    
    plt.axhline(y=base_value, color='purple', linestyle='--', 
                label=f'Base Value ({base_type}: {base_value:.2f})')
    plt.axvspan(time[left_idx], time[right_idx], color='yellow', 
                alpha=0.2, label='Alignment Region')
    
    mode_text = "Mean Alignment" if mode == 0 else "Most Common Value Alignment"
    plt.title(f'Vertical Alignment Using {mode_text}')
    plt.xlabel('Time')
    plt.ylabel('Distance')
    plt.legend(loc='upper left')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    
    filename = 'data/data.png' 
    plt.savefig(filename)
    plt.show()

if __name__ == "__main__":
    file_path = 'data/dataLog.txt'  
    LEFT_IDX = 0                   # Alignment interval start index
    RIGHT_IDX = 35                 # Alignment interval end index
    ADDRESS = 34698                # Device address
    MODE = 0                       # Alignment mode: 0=Mean alignment, 1=Mode alignment
    
    data = read_data(file_path, address=ADDRESS)
    if len(data) == 0:
        print("Warning: No valid data found!")
        exit()
    
    RIGHT_IDX = min(RIGHT_IDX, len(data)-1)
    LEFT_IDX = max(LEFT_IDX, 0)
    
    if LEFT_IDX > RIGHT_IDX:
        print(f"Warning: Start index ({LEFT_IDX}) > end index ({RIGHT_IDX}), swapping...")
        LEFT_IDX, RIGHT_IDX = RIGHT_IDX, LEFT_IDX
    
    plot_adjustment(data, LEFT_IDX, RIGHT_IDX, MODE)