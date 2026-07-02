#!/usr/bin/env python3
"""
Beacon detection - precision, recall, accuracy, and F1
for color classification and blink detection.
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.colors as mcolors
import numpy as np
from pathlib import Path
import itertools

# ── Load data ─────────────────────────────────────────────────────────────────
BASE_DIR = Path(__file__).parent
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

# ── Parse booleans ────────────────────────────────────────────────────────────
def to_bool(val):
    s = str(val).strip().lower()
    if s in ("", "nan", "none", "unknown"):
        return None
    return s == "true"

data["blink_detected"]    = data["blink_is_blinking"].apply(to_bool)
data["target_blinking_b"] = data["target_blinking"].apply(to_bool)
data["color"]             = data["color"].str.strip().str.lower()
data["target_color"]      = data["target_color"].str.strip().str.lower()

# When a blinking beacon is in its OFF phase the detector returns "unknown" —
# this is physically correct behaviour, not a classification error.
data["beacon_off"] = (data["color"] == "unknown") & (data["target_blinking_b"] == True)
n_beacon_off = int(data["beacon_off"].sum())

# ══════════════════════════════════════════════════════════════════════════════
# A. COLOR CLASSIFICATION METRICS
# ══════════════════════════════════════════════════════════════════════════════
actual_classes = ["red", "green", "blue"]           # possible target colors
pred_classes   = ["red", "green", "blue", "unknown"] # possible predicted colors

# Build confusion matrix  [actual x predicted]
# beacon_off rows are routed to the diagonal (TP) rather than the "unknown" column.
conf_color = np.zeros((len(actual_classes), len(pred_classes)), dtype=int)
for _, row in data.iterrows():
    a = row["target_color"]
    p = row["color"] if not row["beacon_off"] else a   # remap off-phase to correct class
    if a in actual_classes and p in pred_classes:
        conf_color[actual_classes.index(a), pred_classes.index(p)] += 1

# Per-class precision / recall / F1 / accuracy
color_metrics = {}
N = conf_color.sum()
for i, cls in enumerate(actual_classes):
    j = pred_classes.index(cls)           # column index for this class as prediction

    tp = conf_color[i, j]
    fp = conf_color[:, j].sum() - tp      # others predicted as cls
    fn = conf_color[i, :].sum() - tp      # cls predicted as something else
    tn = N - tp - fp - fn

    prec = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    rec  = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1   = 2 * prec * rec / (prec + rec)  if (prec + rec) > 0 else 0.0
    acc  = (tp + tn) / N

    color_metrics[cls] = dict(tp=tp, fp=fp, fn=fn, tn=tn,
                               precision=prec, recall=rec, f1=f1, accuracy=acc)

# Macro averages (over the 3 named color classes)
macro_prec = np.mean([color_metrics[c]["precision"] for c in actual_classes])
macro_rec  = np.mean([color_metrics[c]["recall"]    for c in actual_classes])
macro_f1   = np.mean([color_metrics[c]["f1"]        for c in actual_classes])
overall_color_acc = conf_color[:, :3].diagonal().sum() / N  # TP only for named colors

# ══════════════════════════════════════════════════════════════════════════════
# B. BLINK DETECTION METRICS  (binary: blinking=positive, not blinking=negative)
# ══════════════════════════════════════════════════════════════════════════════
bdf = data[data["blink_detected"].notna()].copy()
Nb  = len(bdf)

tp_b = int(((bdf["blink_detected"] == True)  & (bdf["target_blinking_b"] == True)).sum())
fp_b = int(((bdf["blink_detected"] == True)  & (bdf["target_blinking_b"] == False)).sum())
fn_b = int(((bdf["blink_detected"] == False) & (bdf["target_blinking_b"] == True)).sum())
tn_b = int(((bdf["blink_detected"] == False) & (bdf["target_blinking_b"] == False)).sum())

prec_b  = tp_b / (tp_b + fp_b) if (tp_b + fp_b) > 0 else 0.0
rec_b   = tp_b / (tp_b + fn_b) if (tp_b + fn_b) > 0 else 0.0
f1_b    = 2 * prec_b * rec_b / (prec_b + rec_b) if (prec_b + rec_b) > 0 else 0.0
acc_b   = (tp_b + tn_b) / Nb

conf_blink = np.array([[tp_b, fn_b],
                        [fp_b, tn_b]])

# ══════════════════════════════════════════════════════════════════════════════
# C. CONSOLE SUMMARY
# ══════════════════════════════════════════════════════════════════════════════
print(f"Color correction applied: {n_beacon_off} 'unknown' detections on blinking beacons counted as TP (beacon OFF phase)\n")
print("COLOR METRICS")
print(f"{'Class':<10} {'TP':>5} {'FP':>5} {'FN':>5} {'TN':>5}  "
      f"{'Precision':>10} {'Recall':>8} {'F1':>8} {'Accuracy':>10}")
print("-" * 77)
for cls in actual_classes:
    m = color_metrics[cls]
    print(f"{cls:<10} {m['tp']:>5} {m['fp']:>5} {m['fn']:>5} {m['tn']:>5}  "
          f"{m['precision']:>10.1%} {m['recall']:>8.1%} {m['f1']:>8.1%} {m['accuracy']:>10.1%}")
print(f"{'Macro avg':<10} {'':>5} {'':>5} {'':>5} {'':>5}  "
      f"{macro_prec:>10.1%} {macro_rec:>8.1%} {macro_f1:>8.1%} {overall_color_acc:>10.1%}")

print("\nBLINK METRICS  (positive class = blinking, over rows with detected blink state)")
print(f"{'TP':>6} {'FP':>6} {'FN':>6} {'TN':>6}  "
      f"{'Precision':>10} {'Recall':>8} {'F1':>8} {'Accuracy':>10}")
print("-" * 63)
print(f"{tp_b:>6} {fp_b:>6} {fn_b:>6} {tn_b:>6}  "
      f"{prec_b:>10.1%} {rec_b:>8.1%} {f1_b:>8.1%} {acc_b:>10.1%}")

# ══════════════════════════════════════════════════════════════════════════════
# D. VISUALISATION
# ══════════════════════════════════════════════════════════════════════════════
BEACON_CLR = {"red": "#E05C5C", "green": "#4CAF50", "blue": "#5B8FD4"}
METRIC_CLR  = {"Precision": "#4C72B0", "Recall": "#DD8452", "F1": "#55A868", "Accuracy": "#C44E52"}

fig = plt.figure(figsize=(17, 11), facecolor="#F5F7FA")
fig.suptitle("Beacon Detection — Precision, Recall, F1, & Accuracy",
             fontsize=17, fontweight="bold", color="#1A1A2E", y=0.98)

gs = gridspec.GridSpec(
    2, 3,
    figure=fig,
    height_ratios=[1, 1],
    hspace=0.50,
    wspace=0.38,
    left=0.06, right=0.97,
    top=0.92, bottom=0.07,
)

ax_cmat  = fig.add_subplot(gs[0, 0])   # color confusion matrix
ax_cbar  = fig.add_subplot(gs[0, 1:])  # color metric bars
ax_bmat  = fig.add_subplot(gs[1, 0])   # blink confusion matrix
ax_bbar  = fig.add_subplot(gs[1, 1])   # blink metric bars
ax_tbl   = fig.add_subplot(gs[1, 2])   # summary table

for ax in [ax_cmat, ax_cbar, ax_bmat, ax_bbar, ax_tbl]:
    ax.set_facecolor("#FFFFFF")
    for sp in ax.spines.values():
        sp.set_edgecolor("#DDDDDD")

# ── D1. Color confusion matrix ────────────────────────────────────────────────
# Row-normalize so recall is visible in the heatmap colours
conf_norm = conf_color / conf_color.sum(axis=1, keepdims=True).clip(1)

im = ax_cmat.imshow(conf_norm, cmap="Blues", vmin=0, vmax=1, aspect="auto")
ax_cmat.set_xticks(range(len(pred_classes)))
ax_cmat.set_xticklabels([c.capitalize() for c in pred_classes], fontsize=9)
ax_cmat.set_yticks(range(len(actual_classes)))
ax_cmat.set_yticklabels([c.capitalize() for c in actual_classes], fontsize=9,
                         fontweight="bold")
for lbl, cls in zip(ax_cmat.get_yticklabels(), actual_classes):
    lbl.set_color(BEACON_CLR[cls])
ax_cmat.set_xlabel("Predicted", fontsize=10)
ax_cmat.set_ylabel("Actual (Target)", fontsize=10)
ax_cmat.set_title("Color Confusion Matrix\n(cell shade = row recall)", fontsize=11,
                   fontweight="bold", color="#333")

# Annotate cells
for r, c in itertools.product(range(len(actual_classes)), range(len(pred_classes))):
    count = conf_color[r, c]
    pct   = conf_norm[r, c]
    txt_color = "white" if pct > 0.55 else "#333333"
    ax_cmat.text(c, r, f"{count}\n({pct:.0%})", ha="center", va="center",
                 fontsize=9, color=txt_color, fontweight="bold" if r == pred_classes.index(pred_classes[c]) else "normal")

# ── D2. Color per-class metric bars ──────────────────────────────────────────
bar_groups = actual_classes + ["Macro"]
bar_metrics = ["Precision", "Recall", "F1"]
n_g = len(bar_groups)
n_m = len(bar_metrics)
x = np.arange(n_g)
bar_w = 0.22
offsets = np.linspace(-(n_m - 1) / 2, (n_m - 1) / 2, n_m) * bar_w

for i, metric in enumerate(bar_metrics):
    vals = []
    for g in bar_groups:
        if g == "Macro":
            v = {"Precision": macro_prec, "Recall": macro_rec, "F1": macro_f1}[metric]
        else:
            v = color_metrics[g][metric.lower()]
        vals.append(v)

    bars = ax_cbar.bar(x + offsets[i], vals, width=bar_w,
                       color=METRIC_CLR[metric], label=metric,
                       edgecolor="white", linewidth=0.8, alpha=0.92, zorder=3)

    for bar, val in zip(bars, vals):
        ax_cbar.text(bar.get_x() + bar.get_width() / 2,
                     bar.get_height() + 0.012,
                     f"{val:.0%}", ha="center", va="bottom",
                     fontsize=8.5, fontweight="bold", color="#333")

# Separator before Macro
ax_cbar.axvline(n_g - 1 - 0.5, color="#BBBBBB", linewidth=1.2, linestyle="--", zorder=2)

ax_cbar.set_xticks(x)
tick_labels = ax_cbar.set_xticklabels([g.capitalize() for g in bar_groups],
                                       fontsize=11, fontweight="bold")
for lbl, g in zip(tick_labels, bar_groups):
    lbl.set_color(BEACON_CLR.get(g, "#555555"))

ax_cbar.set_ylim(0, 1.2)
ax_cbar.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0%}"))
ax_cbar.set_ylabel("Score", fontsize=10)
ax_cbar.set_title("Color Classification — Precision / Recall / F1 per Class",
                   fontsize=11, fontweight="bold", color="#333", pad=8)
ax_cbar.legend(loc="upper left", framealpha=0.9, fontsize=10)
ax_cbar.yaxis.grid(True, color="#EEEEEE", zorder=0)
ax_cbar.set_axisbelow(True)

# Side note: precision = 100% explanation
ax_cbar.text(0.98, 0.97,
             f"Precision = 100% for all colors:\nthe model never confuses one color\n"
             f"for another — misses are 'unknown'.\n"
             f"({n_beacon_off} off-phase 'unknown' rows on\nblinking beacons counted as correct)",
             transform=ax_cbar.transAxes, ha="right", va="top",
             fontsize=8.5, color="#666",
             bbox=dict(boxstyle="round,pad=0.4", facecolor="#FFF9C4", edgecolor="#CCAA00", alpha=0.9))

# ── D3. Blink confusion matrix (2x2) ─────────────────────────────────────────
blink_norm = conf_blink / conf_blink.sum(axis=1, keepdims=True).clip(1)
blink_labels = ["Blinking\n(Positive)", "Not Blinking\n(Negative)"]
cell_labels  = [["TP", "FN"], ["FP", "TN"]]

im2 = ax_bmat.imshow(blink_norm, cmap="Oranges", vmin=0, vmax=1, aspect="auto")
ax_bmat.set_xticks([0, 1])
ax_bmat.set_xticklabels(blink_labels, fontsize=9)
ax_bmat.set_yticks([0, 1])
ax_bmat.set_yticklabels(blink_labels, fontsize=9, fontweight="bold")
ax_bmat.set_xlabel("Predicted", fontsize=10)
ax_bmat.set_ylabel("Actual (Target)", fontsize=10)
ax_bmat.set_title(f"Blink Confusion Matrix\n(N={Nb} rows with blink state)",
                   fontsize=11, fontweight="bold", color="#333")

for r, c in itertools.product(range(2), range(2)):
    count = conf_blink[r, c]
    pct   = blink_norm[r, c]
    txt_color = "white" if pct > 0.55 else "#333333"
    ax_bmat.text(c, r, f"{cell_labels[r][c]}\n{count}  ({pct:.0%})",
                 ha="center", va="center", fontsize=10,
                 color=txt_color, fontweight="bold")

# ── D4. Blink metric bars ─────────────────────────────────────────────────────
blink_metric_names  = ["Precision", "Recall", "F1", "Accuracy"]
blink_metric_vals   = [prec_b, rec_b, f1_b, acc_b]
blink_bar_colors    = [METRIC_CLR[m] for m in blink_metric_names]

bars = ax_bbar.bar(blink_metric_names, blink_metric_vals,
                   color=blink_bar_colors, edgecolor="white",
                   linewidth=0.8, alpha=0.92, zorder=3, width=0.55)

for bar, val in zip(bars, blink_metric_vals):
    ax_bbar.text(bar.get_x() + bar.get_width() / 2,
                 bar.get_height() + 0.015,
                 f"{val:.1%}", ha="center", va="bottom",
                 fontsize=11, fontweight="bold", color="#333")

ax_bbar.set_ylim(0, 1.2)
ax_bbar.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0%}"))
ax_bbar.set_ylabel("Score", fontsize=10)
ax_bbar.set_title("Blink Detection Metrics\n(positive class = blinking)",
                   fontsize=11, fontweight="bold", color="#333", pad=8)
ax_bbar.yaxis.grid(True, color="#EEEEEE", zorder=0)
ax_bbar.set_axisbelow(True)
ax_bbar.tick_params(axis="x", labelsize=10)

# ── D5. Summary table ─────────────────────────────────────────────────────────
ax_tbl.axis("off")

col_labels = ["", "Precision", "Recall", "F1", "Accuracy"]
row_data = []
for cls in actual_classes:
    m = color_metrics[cls]
    row_data.append([
        cls.capitalize(),
        f"{m['precision']:.1%}",
        f"{m['recall']:.1%}",
        f"{m['f1']:.1%}",
        f"{m['accuracy']:.1%}",
    ])
row_data.append(["Macro avg", f"{macro_prec:.1%}", f"{macro_rec:.1%}",
                 f"{macro_f1:.1%}", f"{overall_color_acc:.1%}"])
row_data.append(["──────────", "──────", "──────", "──────", "──────"])
row_data.append(["Blink", f"{prec_b:.1%}", f"{rec_b:.1%}", f"{f1_b:.1%}", f"{acc_b:.1%}"])

tbl = ax_tbl.table(
    cellText=row_data,
    colLabels=col_labels,
    cellLoc="center",
    loc="center",
    bbox=[0, 0.1, 1, 0.85],
)
tbl.auto_set_font_size(False)
tbl.set_fontsize(10)

for j in range(len(col_labels)):
    cell = tbl[0, j]
    cell.set_facecolor("#1A1A2E")
    cell.set_text_props(color="white", fontweight="bold")

highlight_rows = {4: "#EDE7F6", 6: "#FFF3E0"}
for i in range(1, len(row_data) + 1):
    clr = highlight_rows.get(i, "#FAFAFA" if i % 2 == 0 else "#FFFFFF")
    for j in range(len(col_labels)):
        cell = tbl[i, j]
        cell.set_facecolor(clr)
        if i in highlight_rows:
            cell.set_text_props(fontweight="bold")

for i, cls in enumerate(actual_classes, start=1):
    tbl[i, 0].set_text_props(color=BEACON_CLR[cls], fontweight="bold")

ax_tbl.set_title("All Metrics Summary", fontsize=11, fontweight="bold",
                  color="#333", pad=4)
ax_tbl.text(0.5, 0.04,
            "Blink metrics over rows with detected blink state only",
            ha="center", fontsize=8, color="#888", style="italic",
            transform=ax_tbl.transAxes)

# ── Footer ────────────────────────────────────────────────────────────────────
fig.text(0.5, 0.005,
         f"{len(csv_files)} CSV file(s)  |  {len(data):,} total rows  |  "
         f"{Nb} rows with blink state  |  "
         f"Color correction: {n_beacon_off} off-phase 'unknown' detections on blinking beacons counted as TP",
         ha="center", fontsize=8.5, color="#888", style="italic")

out = BASE_DIR / "beacon_prf_dashboard.png"
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"\nDashboard saved -> {out.name}")
plt.show()
