import re
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg') 

plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 28
plt.rcParams['axes.linewidth'] = 3.0

def extract_rates(filename):
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()
            pattern = r'rate\s*=\s*(\d+\.\d+)'
            matches = re.findall(pattern, content)
            rates = [float(x) for x in matches]
            if not rates:
                print("No 'rate' values found in the file.")
                return None
            return rates
    except Exception as e:
        print(f"Error reading file: {e}")
        return None

def plot_rate_histogram(rates, bar_cmap='viridis', edge_color="white", 
                        avg_line_color='#FF6B6B', shade=0.7, text_color='#2a2a2a'):
    rates_array = np.array(rates)
    average = np.mean(rates_array)
    std_dev = np.std(rates_array)
    
    fig, ax = plt.subplots(figsize=(18, 14))
    
    counts, bins, patches = plt.hist(rates_array, bins=20, edgecolor=edge_color, 
                                    linewidth=5.0, alpha=0.9, density=False)
    
    cmap = plt.colormaps[bar_cmap]
    norm = matplotlib.colors.Normalize(vmin=min(counts), vmax=max(counts))
    for count, patch in zip(counts, patches):
        patch.set_facecolor(cmap(norm(count) * shade))
    
    plt.axvline(average, color=avg_line_color, linestyle='-.', linewidth=5, 
                alpha=0.9, label=f'Mean = {average:.4f} ± {std_dev:.4f}')
    
    for count, bin_edge in zip(counts, bins):
        if count > 0:
            plt.text(bin_edge + (bins[1]-bins[0])/2, count + max(counts)*0.01, 
                     f'{int(count)}', ha='center', va='bottom', 
                     fontsize=26)
    
    ax.spines[['top', 'right']].set_visible(False)
    ax.spines[['left', 'bottom']].set_color(text_color)
    ax.tick_params(colors=text_color, labelsize=26)
    
    plt.title("Compensation Factor Distribution", fontsize=32, pad=40)
    plt.xlabel("Compensation Factor Value", fontsize=30, labelpad=30)
    plt.ylabel("Frequency", fontsize=30, labelpad=30)
    
    ax.grid(True, alpha=0.3, linestyle='--', color='#666666')
    
    legend = plt.legend(frameon=True, fancybox=True, shadow=True, framealpha=0.95, fontsize=26, loc='upper right')
    legend.get_frame().set_edgecolor('#444444')
    legend.get_frame().set_linewidth(3.0)
    legend.get_frame().set_facecolor('#f8f8f8')
    
    ax.set_facecolor('#fafafa')
    fig.patch.set_facecolor('#ffffff')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    filename = "../data/log/rate.txt"
    rates = extract_rates(filename)
    if rates:
        print(f"Extracted {len(rates)} rate values.")
        print(f"Average rate = {np.mean(rates):.6f}")
        print(f"Standard deviation = {np.std(rates):.6f}")
        
        plot_rate_histogram(
            rates,
            bar_cmap='viridis',
            edge_color="#ffffff",
            avg_line_color='#FF6B6B',
            shade=0.85,
            text_color='#333333'
        )