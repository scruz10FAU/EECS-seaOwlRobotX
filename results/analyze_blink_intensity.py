"""
Hybrid blink detection evaluation:
  - Blue (no red/green mix) → intensity oscillation
  - Red or Green            → color-change frequency (dominant color = ON, other = OFF)
  - Mixed/unknown dominant  → intensity (fallback)

Usage:
    python analyze_blink_intensity.py -d path/to/beacon_debug
    python analyze_blink_intensity.py -d seabird_dataset7_27/beacon_debug --split
    python analyze_blink_intensity.py -d seabird_dataset7_27/beacon_debug --out results.png
"""
import argparse
import os
import glob
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

# ── Strategy parameters ───────────────────────────────────────────────────────
# Intensity strategy uses per-file mean as the adaptive threshold (no fixed constant)
MIN_TRANSITIONS           = 3     # minimum transitions for color-change strategy
MIN_TRANSITIONS_INTENSITY = 6     # higher bar for intensity crossings (guards against noise)
MIN_AMPLITUDE_SWING       = 0.20  # non-zero intensity max−min must exceed this
MIN_PERIOD_S              = 0.5
MAX_PERIOD_S              = 5.0
MAX_CV                    = 0.40  # coefficient of variation threshold

BG       = "#F5F7FA"
CORRECT  = "#4CAF50"
WRONG    = "#E05C5C"
ON_CLR   = "#5B8FD4"
OFF_CLR  = "#9E9E9E"
TKW2     = dict(fontsize=10, fontweight="bold", color="#333", pad=6)

# One color per detection strategy shown in charts
STRATEGY_CLR   = {"intensity": "#9C27B0", "color": "#FF9800"}
STRATEGY_LABEL = {
    "intensity": "Intensity oscillation",
    "color":     "Color-change frequency",
}

FOOTER = (
    f"Blue→mean-intensity crossings (non-zero mean, amplitude≥{MIN_AMPLITUDE_SWING}, ≥{MIN_TRANSITIONS_INTENSITY} trans) | "
    f"Red/Green→color-change frequency (dominant=ON, other=OFF, ≥{MIN_TRANSITIONS} trans) | "
    f"period [{MIN_PERIOD_S}–{MAX_PERIOD_S}]s | CV≤{MAX_CV}"
)


# ── Detection strategies ──────────────────────────────────────────────────────

def _detect_intensity(df):
    """Mean-intensity crossing — used for stable blue beacons.
    Threshold and amplitude are derived from non-zero frames only, so measurement
    failures (zero intensity) cannot inflate the amplitude check. Zero frames still
    participate in ON/OFF state assignment via the same threshold — they are not
    excluded, they just land wherever the threshold places them."""
    rows = df[["timestamp", "intensity"]].sort_values("timestamp").reset_index(drop=True)

    non_zero = rows.loc[rows["intensity"] > 0, "intensity"]
    if non_zero.empty:
        return False, 0.0, 0, np.nan, np.nan

    threshold = float(non_zero.mean())

    # Amplitude across all frames: zeros count toward the swing range
    amplitude = float(rows["intensity"].max() - rows["intensity"].min())
    if amplitude < MIN_AMPLITUDE_SWING:
        return False, 0.0, 0, np.nan, np.nan

    rows["state"] = np.where(rows["intensity"] >= threshold, "on", "off")
    rows["prev"]  = rows["state"].shift(1)

    n_trans = int((rows["state"] != rows["prev"]).iloc[1:].sum())
    on_ts   = rows.loc[rows["state"] == "on", "timestamp"].values

    if len(on_ts) < 2 or n_trans < MIN_TRANSITIONS_INTENSITY:
        return False, 0.0, n_trans, np.nan, np.nan

    intervals = np.diff(on_ts)
    median_iv = np.median(intervals)
    short     = intervals[intervals <= 2.0 * median_iv]
    if len(short) == 0:
        return False, 0.0, n_trans, np.nan, np.nan

    mean_iv = short.mean()
    cv      = short.std() / mean_iv if mean_iv > 0 else 99.0
    hz      = 1.0 / mean_iv
    ok      = (MIN_PERIOD_S <= mean_iv <= MAX_PERIOD_S) and (cv <= MAX_CV)
    return ok, hz, n_trans, mean_iv, cv


def _detect_color(df):
    """Color-change frequency — used for red/green beacons and blue fallback.
    Dominant non-unknown color = ON; any other color (including unknown) = OFF.
    Counts how often the detected color changes, then checks period consistency."""
    dom   = _dominant_color(df)
    rows  = df[["timestamp", "color"]].sort_values("timestamp").reset_index(drop=True)
    rows["prev"] = rows["color"].shift(1)

    n_trans = int((rows["color"] != rows["prev"]).iloc[1:].sum())
    on_ts   = rows.loc[rows["color"] == dom, "timestamp"].values

    if len(on_ts) < 2 or n_trans < MIN_TRANSITIONS:
        return False, 0.0, n_trans, np.nan, np.nan

    intervals = np.diff(on_ts)
    median_iv = np.median(intervals)
    short     = intervals[intervals <= 2.0 * median_iv]
    if len(short) == 0:
        return False, 0.0, n_trans, np.nan, np.nan

    mean_iv = short.mean()
    cv      = short.std() / mean_iv if mean_iv > 0 else 99.0
    hz      = 1.0 / mean_iv if mean_iv > 0 else 0.0
    ok      = (MIN_PERIOD_S <= mean_iv <= MAX_PERIOD_S) and (cv <= MAX_CV)
    return ok, hz, n_trans, mean_iv, cv


def _dominant_color(df):
    """Most common non-unknown color."""
    known = df[~df["color"].isin(["unknown"])]
    if known.empty:
        return "unknown"
    return known["color"].value_counts().index[0]


def _is_stable_blue(df):
    """True when red/green frames are fewer than 10% of total (noise tolerance)."""
    other = df["color"].isin(["red", "green"]).sum()
    return other / len(df) < 0.10


def detect_blinking(df, method="hybrid"):
    """Select and run a detection strategy.
    Returns (pred, hz, n_trans, period, cv, strategy).

    Blue + stable (no red/green mix):
        Intensity oscillation first; if no intensity signal found, fall back to
        color-change frequency (e.g. blue beacon whose off-phase stays bright).
    Red or Green:
        Color-change frequency — counts dominant↔other transitions.
    """
    dom = _dominant_color(df)

    if (dom == "blue" and _is_stable_blue(df) and method=="hybrid") or method == "intensity":
        # Stable blue (no red/green mix): intensity only.
        # Zero dips → steady; consistent dips → blinking.
        return (*_detect_intensity(df), "intensity")

    # Red, green, or blue mixed with other primary colors: color-change frequency.
    return (*_detect_color(df), "color")


# ── Drawing helpers ───────────────────────────────────────────────────────────

def _style_ax(ax):
    ax.set_facecolor("#FFFFFF")
    for sp in ax.spines.values():
        sp.set_edgecolor("#DDDDDD")


def _draw_traces(fig, gs, results):
    """Row 0: time series. Row 1: intensity histogram."""
    for col, r in enumerate(results):
        df   = r["df"].sort_values("timestamp").reset_index(drop=True)
        ts   = df["timestamp"] - df["timestamp"].iloc[0]
        intv = df["intensity"]
        strat_clr = STRATEGY_CLR[r["strategy"]]

        # ── time series ──────────────────────────────────────────────────────
        ax = fig.add_subplot(gs[0, col])
        _style_ax(ax)

        if r["strategy"] == "intensity":
            thr   = r["intv_threshold"]
            state = np.where(intv >= thr, "on", "off")
            for i in range(len(ts) - 1):
                ax.axvspan(ts.iloc[i], ts.iloc[i + 1],
                           alpha=0.12, color=ON_CLR if state[i] == "on" else OFF_CLR, lw=0)
            # Shade the minimum required amplitude band centered on the mean.
            # Signal must span the full height of this band (max−min ≥ MIN_AMPLITUDE_SWING).
            half = MIN_AMPLITUDE_SWING / 2
            ax.axhspan(thr - half, thr + half,
                       alpha=0.18, color="#E05C5C", zorder=1)
            ax.axhline(thr - half, color="#E05C5C", lw=0.7, ls=":", alpha=0.6)
            ax.axhline(thr + half, color="#E05C5C", lw=0.7, ls=":", alpha=0.6)
            ax.plot(ts, intv, color=strat_clr, lw=1.4, zorder=3)
            ax.axhline(thr, color="#E05C5C", lw=1, ls="--", alpha=0.8,
                       label=f"mean={thr:.2f}")
        else:
            # Shade by color column: dominant = ON tint, other = OFF tint
            dom = _dominant_color(df)
            for i in range(len(ts) - 1):
                is_on = df["color"].iloc[i] == dom
                ax.axvspan(ts.iloc[i], ts.iloc[i + 1],
                           alpha=0.18, color=strat_clr if is_on else OFF_CLR, lw=0)
            ax.plot(ts, intv, color="#AAAAAA", lw=0.9, zorder=2, alpha=0.7)

        ax.set_ylim(-0.05, 1.05)
        ax.set_xlabel("t (s)", fontsize=7)
        if col == 0:
            ax.set_ylabel("Intensity", fontsize=7)
        ax.tick_params(labelsize=6)

        gt_lbl = "BLINK" if r["gt"]   else "STEADY"
        pr_lbl = "BLINK" if r["pred"] else "STEADY"
        ax.set_title(
            f"{r['name'][-6:]} [{r['dom_color']} | {STRATEGY_LABEL[r['strategy']]}]\n"
            f"GT={gt_lbl}  →  {pr_lbl}",
            fontsize=6.5, fontweight="bold",
            color=CORRECT if r["match"] else WRONG, pad=3)

        # ── histogram ────────────────────────────────────────────────────────
        ax2 = fig.add_subplot(gs[1, col])
        _style_ax(ax2)
        ax2.hist(intv, bins=20, color=strat_clr, alpha=0.70, edgecolor="none")
        if r["strategy"] == "intensity":
            ax2.axvline(r["intv_threshold"], color="#E05C5C", lw=1, ls="--")
            if r["pred"]:
                sub = f"Below mean: {r['n_off']}  ({r['n_trans']} crossings)"
            else:
                amp = r["amplitude"]
                if not np.isnan(amp) and amp < MIN_AMPLITUDE_SWING:
                    sub = f"Swing {amp:.2f} < {MIN_AMPLITUDE_SWING} (rejected)"
                elif r["n_trans"] < MIN_TRANSITIONS_INTENSITY:
                    sub = f"Crossings {r['n_trans']} < {MIN_TRANSITIONS_INTENSITY} (rejected)"
                else:
                    sub = f"Below mean: {r['n_off']} — STEADY"
        else:
            sub = f"Color changes: {r['n_trans']}"
        ax2.set_xlabel("Intensity", fontsize=7)
        if col == 0:
            ax2.set_ylabel("Count", fontsize=7)
        ax2.tick_params(labelsize=6)
        ax2.set_title(sub, fontsize=7, color="#555", pad=3)


def _draw_result_grid(ax, results):
    n = len(results)
    _style_ax(ax)
    ax.set_xlim(-0.5, 3.5); ax.set_ylim(-0.5, n - 0.5)
    ax.set_xticks([0, 1, 2, 3])
    ax.set_xticklabels(["GT", "Pred", "Match", "Detection"], fontsize=8)
    ax.set_yticks(range(n))
    ax.set_yticklabels([r["name"][-6:] for r in results], fontsize=7)
    ax.set_title("Per-file Results", **TKW2)
    for i, r in enumerate(results):
        gt_lbl   = "BLINK" if r["gt"]   else "STEADY"
        pred_lbl = "BLINK" if r["pred"] else "STEADY"
        ok_lbl   = "OK"    if r["match"] else "WRONG"
        strat_short = "intensity" if r["strategy"] == "intensity" else "color chg"
        for xp, lbl, clr in [
            (0, gt_lbl,     ON_CLR  if r["gt"]   else OFF_CLR),
            (1, pred_lbl,   ON_CLR  if r["pred"] else OFF_CLR),
            (2, ok_lbl,     CORRECT if r["match"] else WRONG),
            (3, strat_short, STRATEGY_CLR[r["strategy"]]),
        ]:
            ax.add_patch(plt.Rectangle((xp - 0.45, i - 0.4), 0.9, 0.8,
                         facecolor=clr, alpha=0.85, lw=0))
            ax.text(xp, i, lbl, ha="center", va="center",
                    fontsize=6.5, fontweight="bold", color="white")


def _draw_confusion(ax, tp, tn, fp, fn, prec, rec, f1):
    _style_ax(ax)
    cm      = np.array([[tp, fn], [fp, tn]])
    cm_norm = cm / max(cm.sum(), 1)
    ax.imshow(cm_norm, cmap="Blues", vmin=0, vmax=1, aspect="auto")
    ax.set_xticks([0, 1]); ax.set_xticklabels(["Pred\nBLINK", "Pred\nSTEADY"], fontsize=8)
    ax.set_yticks([0, 1]); ax.set_yticklabels(["GT\nBLINK", "GT\nSTEADY"], fontsize=8)
    for row, col_i, lbl in [(0, 0, "TP"), (0, 1, "FN"), (1, 0, "FP"), (1, 1, "TN")]:
        v = cm[row, col_i]
        ax.text(col_i, row, f"{lbl}\n{v}", ha="center", va="center",
                fontsize=12, fontweight="bold",
                color="white" if cm_norm[row, col_i] > 0.4 else "#333")
    ax.set_title("Confusion Matrix", **TKW2)
    ax.text(0.5, -0.20, f"Prec={prec:.0%}  Rec={rec:.0%}  F1={f1:.0%}",
            ha="center", fontsize=9, color="#555", transform=ax.transAxes)


def _draw_signal_count(ax, results):
    _style_ax(ax)
    names  = [r["name"][-6:] for r in results]
    # Use n_trans for all strategies — the actual count used in the blinking decision.
    # For intensity files rejected by amplitude, n_trans=0 (crossings never ran).
    counts = [r["n_trans"] for r in results]
    bar_c  = [STRATEGY_CLR[r["strategy"]] for r in results]
    bars   = ax.bar(names, counts, color=bar_c, edgecolor="white",
                    alpha=0.88, width=0.5, zorder=3)
    # Two separate threshold lines: one per strategy
    ax.axhline(MIN_TRANSITIONS_INTENSITY, color=STRATEGY_CLR["intensity"],
               lw=1.4, ls="--", alpha=0.85, zorder=4)
    ax.axhline(MIN_TRANSITIONS, color=STRATEGY_CLR["color"],
               lw=1.4, ls="--", alpha=0.85, zorder=4)
    for bar, v in zip(bars, counts):
        ax.text(bar.get_x() + bar.get_width() / 2, v + 0.2,
                str(v), ha="center", va="bottom", fontsize=8, fontweight="bold")
    ax.set_ylabel("Transitions / mean-crossings detected", fontsize=8)
    ax.set_title("Signal Count per File", **TKW2)
    from matplotlib.lines import Line2D
    ax.legend(handles=[
        mpatches.Patch(color=STRATEGY_CLR["intensity"], label="Intensity oscillation"),
        mpatches.Patch(color=STRATEGY_CLR["color"],     label="Color-change frequency"),
        Line2D([0], [0], color=STRATEGY_CLR["intensity"], lw=1.4, ls="--",
               label=f"Intensity min={MIN_TRANSITIONS_INTENSITY}"),
        Line2D([0], [0], color=STRATEGY_CLR["color"],     lw=1.4, ls="--",
               label=f"Color min={MIN_TRANSITIONS}"),
    ], fontsize=7.5)
    ax.yaxis.grid(True, color="#EEE", zorder=0); ax.set_axisbelow(True)
    ax.tick_params(axis="x", labelsize=7, rotation=15)


def _draw_color_timeline(fig, gs, results):
    """One subplot per file: horizontal color band showing detected color each frame."""
    COLOR_MAP = {
        "red":     "#E05C5C",
        "green":   "#4CAF50",
        "blue":    "#5B8FD4",
        "white":   "#E8E8E8",
        "unknown": "#BBBBBB",
    }
    for col, r in enumerate(results):
        df  = r["df"].sort_values("timestamp").reset_index(drop=True)
        ts  = df["timestamp"] - df["timestamp"].iloc[0]
        clrs = df["color"].astype(str)

        ax = fig.add_subplot(gs[0, col])
        _style_ax(ax)
        ax.set_ylim(0, 1)
        ax.set_yticks([])

        # Colored fill for each frame segment
        for i in range(len(ts) - 1):
            ax.axvspan(ts.iloc[i], ts.iloc[i + 1],
                       color=COLOR_MAP.get(clrs.iloc[i], "#CCCCCC"), lw=0, alpha=0.88)
        # Last frame
        if len(ts) > 0:
            span = float(ts.iloc[-1] - ts.iloc[-2]) if len(ts) > 1 else 1.0
            ax.axvspan(ts.iloc[-1], ts.iloc[-1] + span,
                       color=COLOR_MAP.get(clrs.iloc[-1], "#CCCCCC"), lw=0, alpha=0.88)

        # Tick marks at each color transition
        for i in range(1, len(clrs)):
            if clrs.iloc[i] != clrs.iloc[i - 1]:
                ax.axvline(ts.iloc[i], color="#333333", lw=0.6, alpha=0.5)

        ax.set_xlabel("t (s)", fontsize=7)
        ax.tick_params(labelsize=6)

        n_changes = int((clrs != clrs.shift(1)).iloc[1:].sum())
        gt_lbl = "BLINK" if r["gt"] else "STEADY"
        ax.set_title(f"{r['name'][-6:]}  [{gt_lbl}]  {n_changes} changes",
                     fontsize=7, fontweight="bold",
                     color=CORRECT if r["match"] else WRONG, pad=3)

    # Shared legend on the last axis
    ax.legend(handles=[
        mpatches.Patch(color=COLOR_MAP["red"],     label="red"),
        mpatches.Patch(color=COLOR_MAP["green"],   label="green"),
        mpatches.Patch(color=COLOR_MAP["blue"],    label="blue"),
        mpatches.Patch(color=COLOR_MAP["white"],   label="white"),
        mpatches.Patch(color=COLOR_MAP["unknown"], label="unknown"),
    ], fontsize=6.5, loc="upper right", framealpha=0.85)


def _draw_cv_scatter(ax, results):
    _style_ax(ax)
    for r in results:
        if np.isnan(r["cv"]): continue
        ax.scatter(r["n_trans"], r["cv"],
                   color=STRATEGY_CLR[r["strategy"]],
                   marker="o" if r["gt"] else "s",
                   s=90, edgecolors=CORRECT if r["match"] else WRONG,
                   linewidths=2.0, zorder=3)
        hz_lbl = f"  {r['hz']:.2f} Hz" if r["hz"] > 0 else ""
        ax.annotate(r["name"][-6:] + hz_lbl, (r["n_trans"], r["cv"]),
                    fontsize=6, xytext=(4, 3), textcoords="offset points", color="#555")
    ax.axhline(MAX_CV, color="#E05C5C", lw=1.4, ls="--")
    ax.set_xlabel("Transitions detected", fontsize=8)
    ax.set_ylabel("Period CV", fontsize=8)
    ax.set_title("Period Consistency (CV) vs Transitions", **TKW2)
    ax.legend(handles=[
        mpatches.Patch(color=STRATEGY_CLR["intensity"], label="Intensity oscillation"),
        mpatches.Patch(color=STRATEGY_CLR["color"],     label="Color-change frequency"),
        mpatches.Patch(color="#E05C5C",                 label=f"CV≤{MAX_CV}"),
    ], fontsize=7.5)


# ── Output modes ─────────────────────────────────────────────────────────────

def _add_footer(fig, y=0.01):
    fig.text(0.5, y, FOOTER, ha="center", fontsize=7.5, color="#888", style="italic")


def save_combined(results, out_png, tp, tn, fp, fn, prec, rec, f1, method):
    n = len(results)
    method = method.capitalize()
    fig = plt.figure(figsize=(max(14, 2.8 * n), 13), facecolor=BG)
    fig.suptitle(f"{method} Blink Detection — Evaluation",
                 fontsize=15, fontweight="bold", color="#1A1A2E", y=0.99)

    gs_top   = GridSpec(2, n, figure=fig,
                        top=0.93, bottom=0.58, hspace=0.55, wspace=0.30,
                        left=0.06, right=0.97)
    gs_color = GridSpec(1, n, figure=fig,
                        top=0.52, bottom=0.44, wspace=0.30,
                        left=0.06, right=0.97)
    gs_bot   = GridSpec(1, 4, figure=fig,
                        top=0.38, bottom=0.07, wspace=0.40,
                        left=0.06, right=0.97)

    _draw_traces(fig, gs_top, results)
    _draw_color_timeline(fig, gs_color, results)
    _draw_result_grid(fig.add_subplot(gs_bot[0, 0]), results)
    _draw_confusion(fig.add_subplot(gs_bot[0, 1]), tp, tn, fp, fn, prec, rec, f1)
    _draw_signal_count(fig.add_subplot(gs_bot[0, 2]), results)
    _draw_cv_scatter(fig.add_subplot(gs_bot[0, 3]), results)
    _add_footer(fig)

    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {out_png}")


def save_split(results, stem, tp, tn, fp, fn, prec, rec, f1, method):
    n = len(results)
    method = method.capitalize()
    fig = plt.figure(figsize=(max(10, 2.8 * n), 6), facecolor=BG)
    fig.suptitle(f"{method} Blink Detection — Time Series & Histograms",
                 fontsize=13, fontweight="bold", color="#1A1A2E", y=1.01)
    gs = GridSpec(2, n, figure=fig, hspace=0.55, wspace=0.30,
                  left=0.06, right=0.97, top=0.92, bottom=0.08)
    _draw_traces(fig, gs, results)
    _add_footer(fig, y=-0.01)
    path = stem + "_traces.png"
    fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close(fig)
    print(f"  -> {path}")

    fig = plt.figure(figsize=(max(10, 2.8 * n), 2.2), facecolor=BG)
    fig.suptitle("Color Detections Over Time",
                 fontsize=13, fontweight="bold", color="#1A1A2E", y=1.05)
    gs = GridSpec(1, n, figure=fig, wspace=0.30,
                  left=0.06, right=0.97, top=0.78, bottom=0.18)
    _draw_color_timeline(fig, gs, results)
    path = stem + "_color_timeline.png"
    fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close(fig)
    print(f"  -> {path}")

    fig, ax = plt.subplots(figsize=(5, max(3, 0.65 * n)), facecolor=BG)
    fig.suptitle("Per-file Results", fontsize=13, fontweight="bold", color="#1A1A2E")
    _draw_result_grid(ax, results)
    path = stem + "_results.png"
    fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close(fig)
    print(f"  -> {path}")

    fig, ax = plt.subplots(figsize=(4, 4), facecolor=BG)
    fig.suptitle("Confusion Matrix", fontsize=13, fontweight="bold", color="#1A1A2E")
    _draw_confusion(ax, tp, tn, fp, fn, prec, rec, f1)
    path = stem + "_confusion.png"
    fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close(fig)
    print(f"  -> {path}")

    fig, ax = plt.subplots(figsize=(max(5, 1.2 * n), 4), facecolor=BG)
    fig.suptitle("Signal Count per File", fontsize=13, fontweight="bold", color="#1A1A2E")
    _draw_signal_count(ax, results)
    path = stem + "_signal.png"
    fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close(fig)
    print(f"  -> {path}")

    fig, ax = plt.subplots(figsize=(5, 4), facecolor=BG)
    fig.suptitle("Period Consistency vs Transitions",
                 fontsize=13, fontweight="bold", color="#1A1A2E")
    _draw_cv_scatter(ax, results)
    path = stem + "_cv.png"
    fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close(fig)
    print(f"  -> {path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Hybrid blink detection: intensity (blue) / color-change frequency (red/green)"
    )
    parser.add_argument("--data-dir", "-d",
                        default=r"seabird_dataset7_27\beacon_debug",
                        help="Directory containing beacon_log_*.csv files")
    parser.add_argument("--out", "-o", default=None,
                        help="Output PNG (combined) or filename stem (--split). "
                             "Default: <data-dir>/blink_intensity_eval[_<chart>].png")
    parser.add_argument("--split", "-s", action="store_true",
                        help="Save each chart as a separate PNG")
    parser.add_argument("--method", "-m", choices=["intensity", "color", "hybrid"], default="hybrid",
                        help="Method to use per file, select from 'intensity' for intensity only, 'color' for color only, or 'hybrid' to let algorithm choose")

    args = parser.parse_args()

    data_dir = os.path.abspath(args.data_dir)
    files    = sorted(glob.glob(os.path.join(data_dir, "beacon_log_*.csv")))
    if not files:
        print(f"No beacon_log_*.csv files found in: {data_dir}")
        return

    print(f"Evaluating {len(files)} files in: {data_dir}\n")

    results = []
    for f in files:
        df   = pd.read_csv(f)
        name = os.path.basename(f).replace("beacon_log_", "").replace(".csv", "")
        gt_b = df["target_blinking"].astype(str).str.lower()
        gt   = (gt_b == "true").sum() > (gt_b == "false").sum()

        pred, hz, n_trans, period, cv, strategy = detect_blinking(df, args.method)
        dom = _dominant_color(df)
        if strategy == "intensity":
            non_zero_intv = df.loc[df["intensity"] > 0, "intensity"]
            intv_threshold = float(non_zero_intv.mean()) if not non_zero_intv.empty else float("nan")
            n_off = int((df["intensity"] < intv_threshold).sum()) if not np.isnan(intv_threshold) else 0
            amplitude = float(df["intensity"].max() - df["intensity"].min())
        else:
            intv_threshold = float("nan")
            n_off = 0
            amplitude = float("nan")
        results.append(dict(
            name=name, df=df, gt=gt, pred=pred, match=(pred == gt),
            hz=hz, n_trans=n_trans, period=period, cv=cv,
            strategy=strategy, dom_color=dom,
            intv_threshold=intv_threshold,
            n_off=n_off,
            amplitude=amplitude,
        ))

        gt_s   = "BLINK"  if gt   else "STEADY"
        pred_s = "BLINK"  if pred else "STEADY"
        cv_s   = f"{cv:.2f}"    if not np.isnan(cv) else "—"
        hz_s   = f"{hz:.2f} Hz" if hz > 0           else "—"
        print(f"{name}  [{dom:<5} | {STRATEGY_LABEL[strategy]:<26}]  "
              f"GT={gt_s:<6}  Pred={pred_s:<6}  {'OK' if pred == gt else 'WRONG'}  "
              f"trans={n_trans}  CV={cv_s}  {hz_s}")

    n_ok = sum(r["match"] for r in results)
    tp = sum(r["gt"] and r["pred"]         for r in results)
    tn = sum(not r["gt"] and not r["pred"] for r in results)
    fp = sum(not r["gt"] and r["pred"]     for r in results)
    fn = sum(r["gt"] and not r["pred"]     for r in results)
    prec = tp / (tp + fp) if (tp + fp) else 0.0
    rec  = tp / (tp + fn) if (tp + fn) else 0.0
    f1   = 2 * prec * rec / (prec + rec) if (prec + rec) else 0.0
    print(f"\nCorrect: {n_ok}/{len(results)}  TP={tp} TN={tn} FP={fp} FN={fn}")
    print(f"Precision={prec:.0%}  Recall={rec:.0%}  F1={f1:.0%}\n")

    if args.split:
        stem = args.out if args.out else os.path.join(data_dir, f"blink_intensity_eval_{args.method}")
        print(f"STEM: {stem}")
        if stem.endswith(".png"):
            stem = stem[:-4]
        print("Saving split charts:")
        save_split(results, stem, tp, tn, fp, fn, prec, rec, f1, args.method)
    else:
        out_png = args.out if args.out else os.path.join(data_dir, f"blink_intensity_eval_{args.method}.png")
        print(f"OUT: {out_png}")
        print("Saving combined dashboard:")
        save_combined(results, out_png, tp, tn, fp, fn, prec, rec, f1, args.method)


if __name__ == "__main__":
    main()
