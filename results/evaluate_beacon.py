#!/usr/bin/env python3
"""
Beacon detection evaluation — color accuracy, blink accuracy, and combined accuracy
per target color and overall. Produces a visual dashboard.
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import numpy as np
from pathlib import Path

# ── Configuration ─────────────────────────────────────────────────────────────
BASE_DIR = Path(__file__).parent

METRIC_PALETTE = {
    "Color":  "#4C72B0",
    "Blink":  "#DD8452",
    "Both":   "#55A868",
}

BEACON_PALETTE = {
    "red":     "#E05C5C",
    "green":   "#4CAF50",
    "blue":    "#5B8FD4",
    "overall": "#9B59B6",
}

# ── Load CSVs ─────────────────────────────────────────────────────────────────
csv_files = sorted(BASE_DIR.glob("beacon_log_*.csv"))
if not csv_files:
    raise FileNotFoundError(f"No beacon_log_*.csv found in {BASE_DIR}")

frames = []
for f in csv_files:
    df = pd.read_csv(f)
    df["source_file"] = f.name
    frames.append(df)

data = pd.concat(frames, ignore_index=True)
print(f"Loaded {len(csv_files)} file(s)  |  {len(data):,} total rows\n")

# ── Parse boolean-like columns ────────────────────────────────────────────────
def to_bool(val):
    """Return True/False, or None if value is missing/unknown."""
    s = str(val).strip().lower()
    if s in ("", "nan", "none", "unknown"):
        return None
    return s == "true"

data["blink_detected"]     = data["blink_is_blinking"].apply(to_bool)
data["target_blinking_b"]  = data["target_blinking"].apply(to_bool)
data["target_color"]       = data["target_color"].str.strip().str.lower()
data["color"]              = data["color"].str.strip().str.lower()

# ── Correctness flags ─────────────────────────────────────────────────────────
# When a blinking beacon is in its OFF phase the detector returns "unknown" —
# this is physically correct behaviour, not a classification error.
data["beacon_off"]     = (data["color"] == "unknown") & (data["target_blinking_b"] == True)
data["color_correct"]  = (data["color"] == data["target_color"]) | data["beacon_off"]
data["blink_eval"]     = data["blink_detected"].notna()        # rows with a real blink reading
data["blink_correct"]  = data.apply(
    lambda r: r["blink_detected"] == r["target_blinking_b"] if r["blink_eval"] else None, axis=1
)
data["both_correct"]   = data.apply(
    lambda r: bool(r["color_correct"]) and bool(r["blink_correct"]) if r["blink_eval"] else None,
    axis=1,
)

# ── Aggregate helper ──────────────────────────────────────────────────────────
def agg(df: pd.DataFrame) -> dict:
    n       = len(df)
    bdf     = df[df["blink_eval"]]
    nb      = len(bdf)
    ca      = df["color_correct"].mean()
    ba      = bdf["blink_correct"].mean() if nb else float("nan")
    boa     = bdf["both_correct"].mean()  if nb else float("nan")
    return dict(
        n_total=n,
        color_correct=int(df["color_correct"].sum()),
        color_acc=ca,
        n_blink_eval=nb,
        blink_correct=int(bdf["blink_correct"].sum()) if nb else 0,
        blink_acc=ba,
        both_correct=int(bdf["both_correct"].sum()) if nb else 0,
        both_acc=boa,
    )

target_colors = sorted(data["target_color"].unique())
stats = {c: agg(data[data["target_color"] == c]) for c in target_colors}
stats["overall"] = agg(data)

# ── Console summary ───────────────────────────────────────────────────────────
header = f"{'Target':<10}  {'N':>7}  {'Color Acc':>10}  {'Blink Acc*':>10}  {'Both Acc*':>10}  {'N(blink)':>9}"
print(header)
print("-" * len(header))
for key, s in stats.items():
    ba  = f"{s['blink_acc']:.1%}"  if not np.isnan(s["blink_acc"])  else "—"
    boa = f"{s['both_acc']:.1%}"   if not np.isnan(s["both_acc"])   else "—"
    print(f"{key:<10}  {s['n_total']:>7,}  {s['color_acc']:>10.1%}  {ba:>10}  {boa:>10}  {s['n_blink_eval']:>9,}")
n_beacon_off = int(data["beacon_off"].sum())
print(f"\n* Blink / Both accuracy computed only over rows where blink was detected (not 'unknown')")
print(f"  Color correction applied: {n_beacon_off} 'unknown' detections on blinking beacons counted as correct (beacon OFF phase)\n")

# ═══════════════════════════════════════════════════════════════════════════════
# VISUALISATION
# ═══════════════════════════════════════════════════════════════════════════════
groups    = target_colors + ["overall"]
n_groups  = len(groups)
metrics   = ["Color", "Blink", "Both"]
n_metrics = len(metrics)

x = np.arange(n_groups)
bar_w = 0.22
offsets = np.linspace(-(n_metrics - 1) / 2, (n_metrics - 1) / 2, n_metrics) * bar_w

fig = plt.figure(figsize=(16, 11), facecolor="#F5F7FA")
fig.suptitle("Beacon Detection Accuracy Dashboard", fontsize=18, fontweight="bold",
             color="#1A1A2E", y=0.98)

gs = gridspec.GridSpec(
    2, 4,
    figure=fig,
    height_ratios=[1.6, 1],
    hspace=0.45,
    wspace=0.35,
    left=0.06, right=0.97,
    top=0.93, bottom=0.07,
)

ax_main  = fig.add_subplot(gs[0, :])          # full-width bar chart
ax_c     = fig.add_subplot(gs[1, 0])          # donut – color
ax_b     = fig.add_subplot(gs[1, 1])          # donut – blink
ax_bo    = fig.add_subplot(gs[1, 2])          # donut – both
ax_tbl   = fig.add_subplot(gs[1, 3])          # summary table

# ── Panel colour ──────────────────────────────────────────────────────────────
for ax in [ax_main, ax_c, ax_b, ax_bo, ax_tbl]:
    ax.set_facecolor("#FFFFFF")
    for spine in ax.spines.values():
        spine.set_edgecolor("#DDDDDD")

# ──────────────────────────────────────────────────────────────────────────────
# 1. Main grouped bar chart
# ──────────────────────────────────────────────────────────────────────────────
acc_keys = {"Color": "color_acc", "Blink": "blink_acc", "Both": "both_acc"}

for i, metric in enumerate(metrics):
    acc_key = acc_keys[metric]
    vals = [stats[g][acc_key] for g in groups]
    # Replace NaN with 0 for plotting
    vals_plot = [v if not (isinstance(v, float) and np.isnan(v)) else 0 for v in vals]

    bars = ax_main.bar(
        x + offsets[i], vals_plot,
        width=bar_w,
        color=METRIC_PALETTE[metric],
        label=f"{metric} Accuracy",
        edgecolor="white",
        linewidth=0.8,
        alpha=0.92,
        zorder=3,
    )

    for bar, val in zip(bars, vals):
        if isinstance(val, float) and np.isnan(val):
            ax_main.text(bar.get_x() + bar.get_width() / 2, 0.01, "—",
                         ha="center", va="bottom", fontsize=8, color="#888")
        else:
            ax_main.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 0.012,
                f"{val:.0%}",
                ha="center", va="bottom", fontsize=8.5, fontweight="bold",
                color="#333333",
            )

# Vertical separator before "overall"
ax_main.axvline(n_groups - 1 - 0.5, color="#BBBBBB", linewidth=1.2, linestyle="--", zorder=2)

# Color-coded x-tick labels
ax_main.set_xticks(x)
tick_labels = ax_main.set_xticklabels(
    [g.capitalize() for g in groups],
    fontsize=12, fontweight="bold"
)
for label, g in zip(tick_labels, groups):
    label.set_color(BEACON_PALETTE.get(g, "#333333"))

ax_main.set_ylim(0, 1.18)
ax_main.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0%}"))
ax_main.set_ylabel("Accuracy", fontsize=11)
ax_main.set_title("Accuracy by Target Color & Metric", fontsize=13, pad=8, color="#333333")
ax_main.legend(loc="upper left", framealpha=0.9, fontsize=10)
ax_main.yaxis.grid(True, color="#EEEEEE", zorder=0)
ax_main.set_axisbelow(True)

# ──────────────────────────────────────────────────────────────────────────────
# 2. Helper: donut chart
# ──────────────────────────────────────────────────────────────────────────────
def draw_donut(ax, correct, total, title, metric_color):
    wrong = total - correct
    if total == 0:
        ax.text(0.5, 0.5, "No data", ha="center", va="center", transform=ax.transAxes)
        ax.set_title(title, fontsize=11, fontweight="bold", color="#333")
        ax.axis("off")
        return

    wedges, _ = ax.pie(
        [correct, wrong],
        colors=[metric_color, "#E8E8E8"],
        startangle=90,
        wedgeprops=dict(width=0.45, edgecolor="white", linewidth=2),
        counterclock=False,
    )
    pct = correct / total
    ax.text(0, 0, f"{pct:.1%}", ha="center", va="center",
            fontsize=17, fontweight="bold", color="#1A1A2E")
    ax.text(0, -0.28, f"{correct:,}/{total:,}", ha="center", va="center",
            fontsize=9, color="#666")
    ax.set_title(title, fontsize=11, fontweight="bold", color="#333", pad=8)

ov = stats["overall"]

draw_donut(ax_c, ov["color_correct"], ov["n_total"],
           "Overall\nColor Accuracy", METRIC_PALETTE["Color"])

draw_donut(ax_b, ov["blink_correct"], ov["n_blink_eval"],
           "Overall\nBlink Accuracy*", METRIC_PALETTE["Blink"])

draw_donut(ax_bo, ov["both_correct"], ov["n_blink_eval"],
           "Overall\nBoth Correct*", METRIC_PALETTE["Both"])

# ──────────────────────────────────────────────────────────────────────────────
# 3. Summary table
# ──────────────────────────────────────────────────────────────────────────────
ax_tbl.axis("off")

col_labels = ["Target", "N", "Color", "Blink*", "Both*"]
row_data = []
for g in groups:
    s = stats[g]
    ba  = f"{s['blink_acc']:.1%}" if not np.isnan(s["blink_acc"])  else "—"
    boa = f"{s['both_acc']:.1%}"  if not np.isnan(s["both_acc"])   else "—"
    row_data.append([
        g.capitalize(),
        f"{s['n_total']:,}",
        f"{s['color_acc']:.1%}",
        ba, boa,
    ])

tbl = ax_tbl.table(
    cellText=row_data,
    colLabels=col_labels,
    cellLoc="center",
    loc="center",
    bbox=[0, 0.15, 1, 0.80],
)
tbl.auto_set_font_size(False)
tbl.set_fontsize(10)

# Style header
for j in range(len(col_labels)):
    cell = tbl[0, j]
    cell.set_facecolor("#1A1A2E")
    cell.set_text_props(color="white", fontweight="bold")

# Style data rows
for i, g in enumerate(groups, start=1):
    row_color = "#FAFAFA" if i % 2 == 0 else "#FFFFFF"
    highlight = g == "overall"
    for j in range(len(col_labels)):
        cell = tbl[i, j]
        cell.set_facecolor("#EDE7F6" if highlight else row_color)
        if highlight:
            cell.set_text_props(fontweight="bold")

# Color the "Target" column
for i, g in enumerate(groups, start=1):
    tbl[i, 0].set_text_props(color=BEACON_PALETTE.get(g, "#333"), fontweight="bold")

ax_tbl.set_title("Summary Table", fontsize=11, fontweight="bold", color="#333", pad=4)
ax_tbl.text(0.5, 0.06, "* over rows with detected blink state",
            ha="center", va="center", transform=ax_tbl.transAxes,
            fontsize=8, color="#888", style="italic")

# ──────────────────────────────────────────────────────────────────────────────
# Footer
# ──────────────────────────────────────────────────────────────────────────────
n_files = len(csv_files)
fig.text(0.5, 0.005,
         f"Source: {n_files} CSV file(s) from {BASE_DIR.name}  |  "
         f"{len(data):,} detection rows total  |  "
         f"{ov['n_blink_eval']:,} rows with blink state available  |  "
         f"Color correction: {n_beacon_off} 'unknown' detections on blinking beacons treated as correct (beacon OFF phase)",
         ha="center", fontsize=8.5, color="#888", style="italic")

plt.savefig(BASE_DIR / "beacon_accuracy_dashboard.png", dpi=150, bbox_inches="tight")
print("Dashboard saved -> beacon_accuracy_dashboard.png")
plt.show()
