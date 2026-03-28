import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

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

# ------------------ 下降率计算 (%) ------------------
mean_ae_rr = [(a - b) / a * 100 for a, b in zip(ds_twr_mean_ae, dsr_mean_ae)]
max_ae_rr  = [(a - b) / a * 100 for a, b in zip(ds_twr_max_ae,  dsr_max_ae)]
rmse_rr    = [(a - b) / a * 100 for a, b in zip(ds_twr_rmse,    dsr_rmse)]
mre_rr     = [(a - b) / a * 100 for a, b in zip(ds_twr_mre,     dsr_mre)]

# 字体
plt.rcParams['font.family'] = 'Times New Roman'

# 创建子图
fig, axs = plt.subplots(2, 2, figsize=(15, 10))

# 配色
colors = ['#97c3dd', '#fc945d']   # DS-TWR, DSR
rr_color = '#78ba5c'              # Reduction Rate

# ------------------ 绘图函数（无 legend） ------------------
def plot_metric(ax, y_twr, y_dsr, y_rr, ylabel):
    # 左轴
    ax.plot(x, y_twr, color=colors[0], marker='o',
            linewidth=3, markersize=6)
    ax.plot(x, y_dsr, color=colors[1], marker='s',
            linewidth=3, markersize=6)

    ax.set_xlabel('Calculation Frequency per Interval', fontsize=20)
    ax.set_ylabel(ylabel, fontsize=20)
    ax.set_xticks(x)
    ax.tick_params(axis='both', labelsize=20)
    ax.grid(True, linestyle='--', alpha=0.3)

    # 右轴
    ax_r = ax.twinx()
    ax_r.plot(x, y_rr, color=rr_color, linestyle='--', marker='^',
              linewidth=2.5, markersize=6)
    ax_r.set_ylabel('Reduction Rate (%)', fontsize=20)
    ax_r.tick_params(axis='y', labelsize=20)
    ax_r.set_ylim(0, max(y_rr) * 1.3)

# ------------------ 绘制 ------------------
plot_metric(axs[0, 0], ds_twr_mean_ae, dsr_mean_ae, mean_ae_rr, 'Mean AE (cm)')
plot_metric(axs[0, 1], ds_twr_max_ae,  dsr_max_ae,  max_ae_rr,  'Max AE (cm)')
plot_metric(axs[1, 0], ds_twr_rmse,    dsr_rmse,    rmse_rr,    'RMSE (cm)')
plot_metric(axs[1, 1], ds_twr_mre,     dsr_mre,     mre_rr,     'MRE (%)')

# ------------------ 全局 legend（关键） ------------------
legend_elements = [
    Line2D([0], [0], color=colors[0], marker='o',
           linewidth=3, markersize=6, label='SRv2'),
    Line2D([0], [0], color=colors[1], marker='s',
           linewidth=3, markersize=6, label='DSR'),
    Line2D([0], [0], color=rr_color, marker='^',
           linestyle='--', linewidth=2.5, markersize=6,
           label='Reduction Rate')
]

fig.legend(handles=legend_elements,
           loc='upper center',
           ncol=3,
           fontsize=20,
           frameon=False,
           bbox_to_anchor=(0.5, 0.985))

fig.subplots_adjust(
    left=0.07,   # 左边距
    right=0.92,  # 右边距
    top=0.88,    # 上边距，留图例位置
    bottom=0.08, # 下边距
    hspace=0.4,  # 上下子图间距
    wspace=0.4   # 左右子图间距
)

plt.show()