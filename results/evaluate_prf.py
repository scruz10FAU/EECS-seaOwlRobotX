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

import argparse


# ── Load data ─────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description = "comprea precision, recall, accuracy and F1")

parser.add_argument("--filepath", "-f", type=str, help="Path to csv files")
parser.add_argument("--split", "-s", action="store_true",
                    help="Save each chart as a separate PNG (default: single combined dashboard)")

args = parser.parse_args()
# ── Load data ─────────────────────────────────────────────────────────────────
if args.filepath is not None:
    BASE_DIR = Path(args.filepath)
else:
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
# B. LATENCY  (inter-detection interval per file, then aggregated)
# ══════════════════════════════════════════════════════════════════════════════
latency_diffs = []
latency_per_file = {}   # fname -> mean latency (s)
for fname, grp in data.groupby("source_file"):
    ts = grp["timestamp"].sort_values().reset_index(drop=True)
    diffs = ts.diff().dropna()
    diffs = diffs[diffs > 0]        # drop same-timestamp duplicates
    if len(diffs):
        latency_diffs.append(diffs)
        latency_per_file[fname] = diffs.mean()

latency_all   = pd.concat(latency_diffs) if latency_diffs else pd.Series([], dtype=float)
lat_mean = latency_all.mean()   # seconds
lat_std  = latency_all.std()
lat_min  = latency_all.min()
lat_max  = latency_all.max()

# ══════════════════════════════════════════════════════════════════════════════
# C. BLINK DETECTION METRICS  (binary: blinking=positive, not blinking=negative)
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
# D. AVERAGE PRECISION  (area under PR curve, one-vs-rest per color class)
# ══════════════════════════════════════════════════════════════════════════════
def _compute_ap(y_true, y_score):
    """Trapezoid area under the precision-recall curve."""
    order  = np.argsort(y_score)[::-1]
    yt     = np.asarray(y_true, dtype=int)[order]
    n_pos  = yt.sum()
    if n_pos == 0:
        return 0.0
    tp_cum = np.cumsum(yt)
    prec   = tp_cum / np.arange(1, len(yt) + 1)
    rec    = tp_cum / n_pos
    prec   = np.concatenate([[1.0], prec])
    rec    = np.concatenate([[0.0], rec])
    return float(np.trapezoid(prec, rec))

ap_color = {}
for cls in actual_classes:
    y_true = (data["target_color"] == cls).values
    vcol   = f"vote_{cls}"
    if vcol in data.columns:
        y_score = data[vcol].fillna(0).values
    else:
        # proxy: model confidence when it picks this class, inverse otherwise
        y_score = np.where(data["color"] == cls,
                           data["det_confidence"].values,
                           1.0 - data["det_confidence"].values)
    ap_color[cls] = _compute_ap(y_true, y_score)

map_color = float(np.mean(list(ap_color.values())))

# Blink: no continuous confidence score, so AP equals precision at the single threshold
ap_blink = prec_b

# ══════════════════════════════════════════════════════════════════════════════
# E. CONSOLE SUMMARY
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

print(f"\nLATENCY  (inter-detection interval across {len(csv_files)} file(s))")
print(f"  Mean : {lat_mean*1000:.1f} ms   Std : {lat_std*1000:.1f} ms   "
      f"Min : {lat_min*1000:.1f} ms   Max : {lat_max*1000:.1f} ms")
for fname, lmean in latency_per_file.items():
    print(f"  {fname:<45}  {lmean*1000:.1f} ms avg")

print("\nBLINK METRICS  (positive class = blinking, over rows with detected blink state)")
print(f"{'TP':>6} {'FP':>6} {'FN':>6} {'TN':>6}  "
      f"{'Precision':>10} {'Recall':>8} {'F1':>8} {'Accuracy':>10}")
print("-" * 63)
print(f"{tp_b:>6} {fp_b:>6} {fn_b:>6} {tn_b:>6}  "
      f"{prec_b:>10.1%} {rec_b:>8.1%} {f1_b:>8.1%} {acc_b:>10.1%}")

# ══════════════════════════════════════════════════════════════════════════════
# E. VISUALISATION
# ══════════════════════════════════════════════════════════════════════════════
BEACON_CLR = {"red": "#E05C5C", "green": "#4CAF50", "blue": "#5B8FD4"}
METRIC_CLR  = {"Precision": "#4C72B0", "Recall": "#DD8452", "F1": "#55A868",
               "Accuracy": "#C44E52", "AP": "#9C27B0"}

# Pre-computed values shared by drawing helpers
conf_norm  = conf_color / conf_color.sum(axis=1, keepdims=True).clip(1)
blink_norm = conf_blink / conf_blink.sum(axis=1, keepdims=True).clip(1)
bar_groups  = actual_classes + ["Macro"]
bar_metrics = ["Precision", "Recall", "F1", "Accuracy", "AP"]
_n_g = len(bar_groups)
_x   = np.arange(_n_g)
_bw  = 0.13
_off = np.linspace(-(len(bar_metrics) - 1) / 2,
                    (len(bar_metrics) - 1) / 2,
                    len(bar_metrics)) * _bw

def _style(ax):
    ax.set_facecolor("#FFFFFF")
    for sp in ax.spines.values():
        sp.set_edgecolor("#DDDDDD")

def _draw_color_confusion(ax):
    im = ax.imshow(conf_norm, cmap="Blues", vmin=0, vmax=1, aspect="auto")
    ax.set_xticks(range(len(pred_classes)))
    ax.set_xticklabels([c.capitalize() for c in pred_classes], fontsize=9)
    ax.set_yticks(range(len(actual_classes)))
    ax.set_yticklabels([c.capitalize() for c in actual_classes], fontsize=9,
                       fontweight="bold")
    for lbl, cls in zip(ax.get_yticklabels(), actual_classes):
        lbl.set_color(BEACON_CLR[cls])
    ax.set_xlabel("Predicted", fontsize=10)
    ax.set_ylabel("Actual (Target)", fontsize=10)
    ax.set_title("Color Confusion Matrix\n(cell shade = row recall)",
                 fontsize=11, fontweight="bold", color="#333")
    for r, c in itertools.product(range(len(actual_classes)), range(len(pred_classes))):
        count = conf_color[r, c]
        pct   = conf_norm[r, c]
        txt_color = "white" if pct > 0.55 else "#333333"
        ax.text(c, r, f"{count}\n({pct:.0%})", ha="center", va="center",
                fontsize=9, color=txt_color,
                fontweight="bold" if r == pred_classes.index(pred_classes[c]) else "normal")
    ax.get_figure().colorbar(im, ax=ax, fraction=0.046, pad=0.04)

def _draw_color_metrics(ax):
    for i, metric in enumerate(bar_metrics):
        vals = []
        for g in bar_groups:
            if g == "Macro":
                v = {"Precision": macro_prec, "Recall": macro_rec, "F1": macro_f1,
                     "Accuracy": overall_color_acc, "AP": map_color}[metric]
            else:
                v = color_metrics[g][metric.lower()] if metric != "AP" else ap_color[g]
            vals.append(v)
        bars = ax.bar(_x + _off[i], vals, width=_bw,
                      color=METRIC_CLR[metric], label=metric,
                      edgecolor="white", linewidth=0.8, alpha=0.92, zorder=3)
        for bar, val in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() + 0.012, f"{val:.0%}",
                    ha="center", va="bottom",
                    fontsize=7.5, fontweight="bold", color="#333", rotation=45)
    ax.axvline(_n_g - 1 - 0.5, color="#BBBBBB", linewidth=1.2, linestyle="--", zorder=2)
    ax.set_xticks(_x)
    tick_labels = ax.set_xticklabels([g.capitalize() for g in bar_groups],
                                     fontsize=11, fontweight="bold")
    for lbl, g in zip(tick_labels, bar_groups):
        lbl.set_color(BEACON_CLR.get(g, "#555555"))
    ax.set_ylim(0, 1.2)
    ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0%}"))
    ax.set_ylabel("Score", fontsize=10)
    ax.set_title("Color Classification — Precision / Recall / F1 / Accuracy / AP",
                 fontsize=11, fontweight="bold", color="#333", pad=8)
    ax.legend(loc="lower left", framealpha=0.9, fontsize=10)
    ax.yaxis.grid(True, color="#EEEEEE", zorder=0)
    ax.set_axisbelow(True)

def _draw_blink_confusion(ax):
    blink_labels = ["Blinking\n(Positive)", "Not Blinking\n(Negative)"]
    cell_labels  = [["TP", "FN"], ["FP", "TN"]]
    ax.imshow(blink_norm, cmap="Oranges", vmin=0, vmax=1, aspect="auto")
    ax.set_xticks([0, 1]); ax.set_xticklabels(blink_labels, fontsize=9)
    ax.set_yticks([0, 1]); ax.set_yticklabels(blink_labels, fontsize=9, fontweight="bold")
    ax.set_xlabel("Predicted", fontsize=10)
    ax.set_ylabel("Actual (Target)", fontsize=10)
    ax.set_title(f"Blink Confusion Matrix\n(N={Nb} rows with blink state)",
                 fontsize=11, fontweight="bold", color="#333")
    for r, c in itertools.product(range(2), range(2)):
        count = conf_blink[r, c]
        pct   = blink_norm[r, c]
        txt_color = "white" if pct > 0.55 else "#333333"
        ax.text(c, r, f"{cell_labels[r][c]}\n{count}  ({pct:.0%})",
                ha="center", va="center", fontsize=10,
                color=txt_color, fontweight="bold")

def _draw_blink_metrics(ax):
    names  = ["Precision", "Recall", "F1", "Accuracy", "AP"]
    vals   = [prec_b, rec_b, f1_b, acc_b, ap_blink]
    colors = [METRIC_CLR[m] for m in names]
    bars = ax.bar(names, vals, color=colors, edgecolor="white",
                  linewidth=0.8, alpha=0.92, zorder=3, width=0.55)
    for bar, val in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.015,
                f"{val:.1%}", ha="center", va="bottom",
                fontsize=11, fontweight="bold", color="#333")
    ax.set_ylim(0, 1.2)
    ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0%}"))
    ax.set_ylabel("Score", fontsize=10)
    ax.set_title("Blink Detection Metrics\n(positive class = blinking)",
                 fontsize=11, fontweight="bold", color="#333", pad=8)
    ax.yaxis.grid(True, color="#EEEEEE", zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(axis="x", labelsize=10)

def _draw_summary(ax, fig):
    ax.axis("off")
    col_labels = ["", "Precision", "Recall", "F1", "Accuracy"]
    row_data = []
    for cls in actual_classes:
        m = color_metrics[cls]
        row_data.append([cls.capitalize(), f"{m['precision']:.1%}",
                         f"{m['recall']:.1%}", f"{m['f1']:.1%}", f"{m['accuracy']:.1%}"])
    row_data.append(["Macro avg", f"{macro_prec:.1%}", f"{macro_rec:.1%}",
                     f"{macro_f1:.1%}", f"{overall_color_acc:.1%}"])
    row_data.append(["──────────", "──────", "──────", "──────", "──────"])
    row_data.append(["Blink", f"{prec_b:.1%}", f"{rec_b:.1%}", f"{f1_b:.1%}", f"{acc_b:.1%}"])

    tbl = ax.table(cellText=row_data, colLabels=col_labels,
                   cellLoc="center", loc="center", bbox=[0, 0.18, 1, 0.78])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    for j in range(len(col_labels)):
        tbl[0, j].set_facecolor("#1A1A2E")
        tbl[0, j].set_text_props(color="white", fontweight="bold")
    highlight_rows = {4: "#EDE7F6", 6: "#FFF3E0"}
    for i in range(1, len(row_data) + 1):
        clr = highlight_rows.get(i, "#FAFAFA" if i % 2 == 0 else "#FFFFFF")
        for j in range(len(col_labels)):
            tbl[i, j].set_facecolor(clr)
            if i in highlight_rows:
                tbl[i, j].set_text_props(fontweight="bold")
    for i, cls in enumerate(actual_classes, start=1):
        tbl[i, 0].set_text_props(color=BEACON_CLR[cls], fontweight="bold")

    ax.set_title("All Metrics Summary", fontsize=11, fontweight="bold", color="#333", pad=4)
    ax.text(0.5, 0.13, "Blink: rows with detected blink state only",
            ha="center", fontsize=7.5, color="#888", style="italic",
            transform=ax.transAxes)
    ax.text(0.5, 0.06,
            f"Avg latency: {lat_mean*1000:.0f} ms  "
            f"(±{lat_std*1000:.0f} ms std,  "
            f"{lat_min*1000:.0f}–{lat_max*1000:.0f} ms range)",
            ha="center", fontsize=9, color="#444", fontweight="bold",
            transform=ax.transAxes)
    ax.text(0.5, 0.01, "Latency = mean inter-detection interval across all files",
            ha="center", fontsize=7.5, color="#888", style="italic",
            transform=ax.transAxes)
    fig.text(0.5, 0.005,
             f"{len(csv_files)} CSV file(s)  |  {len(data):,} total rows  |  "
             f"{Nb} rows with blink state  |  "
             f"Color correction: {n_beacon_off} off-phase 'unknown' counted as TP",
             ha="center", fontsize=7.5, color="#888", style="italic")

# ── Dispatch ──────────────────────────────────────────────────────────────────
saved = []

if args.split:
    specs = [
        ("beacon_prf_color_confusion.png", (6, 5),   _draw_color_confusion),
        ("beacon_prf_color_metrics.png",   (10, 6),  _draw_color_metrics),
        ("beacon_prf_blink_confusion.png", (6, 5),   _draw_blink_confusion),
        ("beacon_prf_blink_metrics.png",   (7, 5),   _draw_blink_metrics),
    ]
    for fname, size, draw_fn in specs:
        fig, ax = plt.subplots(figsize=size, facecolor="#F5F7FA")
        _style(ax)
        draw_fn(ax)
        out = BASE_DIR / fname
        fig.savefig(out, dpi=150, bbox_inches="tight")
        plt.close(fig)
        saved.append(out.name)

    fig, ax = plt.subplots(figsize=(7, 5), facecolor="#F5F7FA")
    _draw_summary(ax, fig)
    out = BASE_DIR / "beacon_prf_summary.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    saved.append(out.name)

else:
    fig = plt.figure(figsize=(17, 11), facecolor="#F5F7FA")
    fig.suptitle("Beacon Detection — Precision, Recall, F1, & Accuracy",
                 fontsize=17, fontweight="bold", color="#1A1A2E", y=0.98)
    gs = gridspec.GridSpec(2, 3, figure=fig,
                           height_ratios=[1, 1], hspace=0.50, wspace=0.38,
                           left=0.06, right=0.97, top=0.92, bottom=0.07)
    panels = {
        "cmat": fig.add_subplot(gs[0, 0]),
        "cbar": fig.add_subplot(gs[0, 1:]),
        "bmat": fig.add_subplot(gs[1, 0]),
        "bbar": fig.add_subplot(gs[1, 1]),
        "tbl":  fig.add_subplot(gs[1, 2]),
    }
    for ax in panels.values():
        _style(ax)
    _draw_color_confusion(panels["cmat"])
    _draw_color_metrics(panels["cbar"])
    _draw_blink_confusion(panels["bmat"])
    _draw_blink_metrics(panels["bbar"])
    _draw_summary(panels["tbl"], fig)

    out = BASE_DIR / "beacon_prf_dashboard.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    saved.append(out.name)

print()
for name in saved:
    print(f"  Saved -> {name}")
