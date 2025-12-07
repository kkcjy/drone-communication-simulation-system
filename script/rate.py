import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde
from matplotlib.ticker import PercentFormatter

def extract_rates(filename):
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()
            pattern = r'rate\s*=\s*([\d\.]+)'
            matches = re.findall(pattern, content)
            rates = [float(x) for x in matches]
            if not rates:
                print("No 'rate' values found in the file.")
                return None
            return rates
    except Exception as e:
        print(f"Error reading file: {e}")
        return None

def error_hill_plot(rates):
    rates_array = np.array(rates)
    mean = np.mean(rates_array)
    std = np.std(rates_array)

    main_color = '#ffc0b7'    # KDE 曲线 + ±1σ 填充
    sigma_color = "#e3ad78"   # ±1σ 区域填充
    mean_color = "#a792c9"    # 均值标记

    xs_min = max(0, mean - 3*std)
    xs_max = mean + 3*std
    xs = np.linspace(xs_min, xs_max, 400)

    kde = gaussian_kde(rates_array)
    y = kde(xs)

    plt.figure(figsize=(10, 6))
    # 整体曲线下方填充
    plt.fill_between(xs, 0, y, color=main_color, alpha=0.2)
    # ±1σ 区域填充
    plt.fill_between(xs, 0, y, where=(xs >= mean - std) & (xs <= mean + std),
                     color=sigma_color, alpha=0.4, label=f'±1σ = {std:.4f}')
    # KDE 曲线
    plt.plot(xs, y, color=main_color, linewidth=2.5)
    # 均值标记
    plt.axvline(mean, color=mean_color, linestyle='--', linewidth=2, label=f'Mean = {mean:.4f}')

    # 坐标轴标签
    plt.xlabel("Compensation Factor", fontsize=16, fontname='Times New Roman')
    plt.ylabel("Percentage (%)", fontsize=16, fontname='Times New Roman')

    # 网格和边框
    plt.grid(True, linestyle='--', alpha=0.5)
    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_linewidth(1.2)
    ax.spines['bottom'].set_linewidth(1.2)

    ax.yaxis.set_major_formatter(PercentFormatter())

    plt.legend(fontsize=12, frameon=False)
    plt.tight_layout()
    plt.ylim(0, 20)
    plt.show()

if __name__ == "__main__":
    filename = "../data/rate.txt"
    rates = extract_rates(filename)
    error_hill_plot(rates)
