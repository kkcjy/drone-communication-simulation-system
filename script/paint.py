import matplotlib.pyplot as plt

# X轴：每个测距周期的计算次数（采样频率）
x = [1, 3, 5, 7, 9]

# DS-TWR 四个指标
ds_twr_mean_ae = [6.287, 7.482, 7.721, 7.827, 7.888]
ds_twr_max_ae  = [26.555, 32.812, 33.561, 33.562, 33.562]
ds_twr_rmse    = [7.860, 9.307, 9.588, 9.717, 9.785]
ds_twr_mre     = [3.483, 4.063, 4.178, 4.230, 4.259]

# DSR 四个指标
dsr_mean_ae = [4.438, 4.890, 4.985, 5.027, 5.051]
dsr_max_ae  = [26.520, 29.336, 30.261, 30.843, 30.800]
dsr_rmse    = [5.768, 6.402, 6.534, 6.594, 6.626]
dsr_mre     = [2.626, 2.897, 2.953, 2.979, 2.993]

# 设置字体
plt.rcParams['font.family'] = 'Times New Roman'

# 创建 2x2 子图
fig, axs = plt.subplots(2, 2, figsize=(12, 8))

# 配色方案
colors = ['#97c3dd', '#fc945d']  # DS-TWR 蓝, DSR 橙
markers = ['o', 's']
linestyles = ['-', '-']

# 绘图函数
def plot_metric(ax, y_twr, y_dsr, ylabel):
    ax.plot(x, y_twr, color=colors[0], linestyle=linestyles[0], marker=markers[0], linewidth=3, markersize=6, label='DS-TWR')
    ax.plot(x, y_dsr, color=colors[1], linestyle=linestyles[1], marker=markers[1], linewidth=3, markersize=6, label='DSR')
    ax.set_xlabel('Calculation Frequency per Cycle', fontsize=15, labelpad=8)
    ax.set_ylabel(ylabel, fontsize=15, labelpad=8)
    ax.set_xticks(x)
    ax.tick_params(axis='x', labelsize=10)
    ax.tick_params(axis='y', labelsize=10)
    
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.legend(fontsize=12, loc='upper left', framealpha=0)

plot_metric(axs[0, 0], ds_twr_mean_ae, dsr_mean_ae, 'Mean AE(cm)')
plot_metric(axs[0, 1], ds_twr_max_ae, dsr_max_ae, 'Max AE(cm)')
plot_metric(axs[1, 0], ds_twr_rmse, dsr_rmse, 'RMSE(cm)')
plot_metric(axs[1, 1], ds_twr_mre, dsr_mre, 'MRE(%)')

plt.tight_layout(pad=3.0)
plt.show()
