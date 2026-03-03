import matplotlib
matplotlib.use('TkAgg')

import re
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

with open("log.txt", "r", encoding="utf-8") as f:
    text = f.read()

colors = {
    "Mean AE": {
        "SRv2": "#9bbf8a",
        "IEEE": "#add3e2"
    },
    "Max AE": {
        "SRv2": "#82afda",
        "IEEE": "#ffbe7a"
    },
    "RMSE": {
        "SRv2": "#f79059",
        "IEEE": "#fa8878"
    }
}

rates = []
metrics = {
    "Mean AE": {"IEEE": [], "SRv2": [], "DSR": []},
    "Max AE": {"IEEE": [], "SRv2": [], "DSR": []},
    "RMSE": {"IEEE": [], "SRv2": [], "DSR": []},
}

blocks = re.split(r"rate\s*=\s*", text)[1:]

for block in blocks:
    rate = int(block.strip().splitlines()[0])
    rates.append(rate)

    ieee = re.search(r"IEEE\s*:.*?=\s*([\d.]+).*?=\s*([\d.]+).*?=\s*([\d.]+)", block)
    sr = re.search(r"SRv2:.*?=\s*([\d.]+).*?=\s*([\d.]+).*?=\s*([\d.]+)", block)
    dsr = re.search(r"DSR\s*:.*?=\s*([\d.]+).*?=\s*([\d.]+).*?=\s*([\d.]+)", block)

    metrics["Mean AE"]["IEEE"].append(float(ieee.group(1)))
    metrics["Max AE"]["IEEE"].append(float(ieee.group(2)))
    metrics["RMSE"]["IEEE"].append(float(ieee.group(3)))

    metrics["Mean AE"]["SRv2"].append(float(sr.group(1)))
    metrics["Max AE"]["SRv2"].append(float(sr.group(2)))
    metrics["RMSE"]["SRv2"].append(float(sr.group(3)))

    metrics["Mean AE"]["DSR"].append(float(dsr.group(1)))
    metrics["Max AE"]["DSR"].append(float(dsr.group(2)))
    metrics["RMSE"]["DSR"].append(float(dsr.group(3)))

reduction_sr = {
    k: [(s - d) / s * 100.0 for s, d in zip(v["SRv2"], v["DSR"])]
    for k, v in metrics.items()
}

reduction_ieee = {
    k: [(i - d) / i * 100.0 for i, d in zip(v["IEEE"], v["DSR"])]
    for k, v in metrics.items()
}

fig, ax = plt.subplots(figsize=(24, 14))

markers = {
    "Mean AE": "o",
    "Max AE": "s",
    "RMSE": "^",
}

for name, values in reduction_sr.items():
    ax.plot(
        rates,
        values,
        marker=markers[name],
        linewidth=4,
        markersize=14,
        color=colors[name]["SRv2"],
        markerfacecolor=colors[name]["SRv2"],
        markeredgecolor=colors[name]["SRv2"],
        label=f"{name} (DSR vs SRv2)"
    )

for name, values in reduction_ieee.items():
    ax.plot(
        rates,
        values,
        marker=markers[name],
        linewidth=4,
        markersize=14,
        linestyle="--",
        color=colors[name]["IEEE"],
        markerfacecolor=colors[name]["IEEE"],
        markeredgecolor=colors[name]["IEEE"],
        label=f"{name} (DSR vs IEEE)"
    )

ax.set_xlabel(
    "Ratio of ranging neighbors to bodyUnits in rangingMessage",
    fontsize=40,
    labelpad=40
)

ax.set_ylabel(
    "Error reduction rate",
    fontsize=40,
    labelpad=40
)

ax.set_xticks(rates)
ax.tick_params(axis='x', labelsize=32)
ax.tick_params(axis='y', labelsize=32)

ax.yaxis.set_major_formatter(FuncFormatter(lambda y, _: f"{y:.0f}%"))

ax.grid(True, linestyle='--', alpha=0.3)

ax.legend(
    fontsize=24,
    frameon=False,
    ncol=2,
    columnspacing=4,
    loc='upper left'
)

plt.tight_layout()
plt.show()
