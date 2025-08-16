#!/usr/bin/env python3
import os, re
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

BASE = os.path.dirname(os.path.abspath(__file__))  # .../evaluation/payload
OUTDIR = os.path.join(BASE, "out_plots")
os.makedirs(OUTDIR, exist_ok=True)

CANDIDATES = [
    ("static_ref_data.txt", "Reference"),
    ("payload_200g.txt",    "200 g"),
    # ("payload_2kg.txt",     "2 kg"),
    # ("payload_3kg.txt",     "3 kg"),
    # ("payload_4kg.txt",     "4 kg"),
    ("payload_6kg.txt",     "6 kg"),
    # ("payload_9kg.txt",     "9 kg"),
    ("payload_14kg.txt",    "14 kg"),
]

# Per-curve time shifts (seconds) and windows (seconds AFTER offset)
OFFSETS = {
    "Reference": -125,
    "200 g": -100,
    # "2 kg": -30,
    #"3 kg": -50,
    # "4 kg": -25,
    "6 kg": -60,
    #"9 kg": -40,
    "14 kg": -60,
}
WINDOWS = {
    "Reference": (0,20),
    "200 g": (0, 20),
    # "2 kg": (0, 20),
    # "3 kg": (0, 20),
    # "4 kg": (0, 20),
    "6 kg": (0, 20),
    # "9 kg": (0, 20),
    "14 kg": (0, 20),
}

# Optional global axis locks for individual plots
LOCK_X_RANGE = None  # e.g., (0, 175) to force same x-limits
LOCK_Y_RANGE = None  # e.g., (-1.0, 1.0) to force same y-limits

stability_pattern = re.compile(
    r"\[CONTROLLER\]\s+(\d+)\s*\|\s*stability_controller\s*\|\s*\["
    r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*,"
    r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*,"
    r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*\]"
)

def parse_file(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = stability_pattern.search(line)
            if not m:
                continue
            t   = int(m.group(1))
            tgt = float(m.group(2))
            pit = float(m.group(3))
            acc = float(m.group(4))
            rows.append((t, tgt, pit, acc))
    if not rows:
        return pd.DataFrame(columns=["Time", "Target_Angle", "Pitch", "Acc_Cmd"])
    df = pd.DataFrame(rows, columns=["t", "Target_Angle", "Pitch", "Acc_Cmd"])
    t0 = df["t"].min()
    df["Time"] = (df["t"] - t0) / 1000.0  # seconds from 0 per file
    return df[["Time", "Target_Angle", "Pitch", "Acc_Cmd"]]

def apply_offset_and_windows(df, label):
    df2 = df.copy()
    offset = OFFSETS.get(label, 0.0) or 0.0
    df2["Time"] = df2["Time"] + offset

    win = WINDOWS.get(label, None)
    if win is None:
        return df2

    def crop_once(d, lo, hi):
        return d[(d["Time"] >= lo) & (d["Time"] <= hi)]

    if isinstance(win, (tuple, list)) and win and isinstance(win[0], (int, float)):
        dsel = crop_once(df2, win[0], win[1])
    elif isinstance(win, (list, tuple)):
        parts = []
        for seg in win:
            if isinstance(seg, (list, tuple)) and len(seg) == 2:
                parts.append(crop_once(df2, seg[0], seg[1]))
        dsel = pd.concat(parts, ignore_index=True) if parts else df2.iloc[0:0]
    else:
        dsel = df2

    return dsel

def safename(label):
    return "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in label)

def plot_combined(datasets):
    plt.figure(figsize=(10, 5))
    all_tmin, all_tmax = None, None
    for label, df in datasets:
        plt.plot(df["Time"], df["Pitch"], label=label)
        tmin, tmax = df["Time"].min(), df["Time"].max()
        all_tmin = tmin if all_tmin is None else min(all_tmin, tmin)
        all_tmax = tmax if all_tmax is None else max(all_tmax, tmax)

    plt.xlabel("Time (s)", fontsize=14)
    plt.ylabel("Angle (°)", fontsize=14)
    plt.ylim(0, 0.8)
    plt.grid(True, which="both", linestyle=":", linewidth=0.8)
    plt.legend(loc="best", ncol=2, frameon=True)
    plt.tight_layout()
    if all_tmin is not None and all_tmax is not None:
        plt.xlim(all_tmin, all_tmax)
    out_pdf = os.path.join(OUTDIR, "max_payload_combined.pdf")
    plt.savefig(out_pdf)
    plt.close()

def plot_individual(datasets):
    for label, df in datasets:
        fig = plt.figure(figsize=(8, 4.5))
        plt.plot(df["Time"], df["Pitch"], label="Pitch")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (deg)")
        plt.title(f"{label}")
        plt.ylim(0, 0.8)
        plt.grid(True, which="both", linestyle=":", linewidth=0.8)
        plt.legend(loc="best", frameon=True)
        plt.tight_layout()

        base = safename(label)
        pdf_path = os.path.join(OUTDIR, f"{base}_pitch.pdf")
        plt.savefig(pdf_path)  
        plt.close(fig)

def main():
    datasets = []
    for fname, label in CANDIDATES:
        path = os.path.join(BASE, fname)
        if not os.path.exists(path):
            continue
        df = parse_file(path)
        if df.empty:
            continue
        df2 = apply_offset_and_windows(df, label)
        if df2.empty:
            print(f"[WARN] After offset/window, '{label}' has no data; skipping.")
            continue
        datasets.append((label, df2))

    if not datasets:
        raise SystemExit("No data found after offset/window processing.")

    plot_combined(datasets)
    plot_individual(datasets)

if __name__ == "__main__":
    main()
