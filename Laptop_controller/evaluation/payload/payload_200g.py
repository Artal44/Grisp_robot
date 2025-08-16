import pandas as pd
import matplotlib.pyplot as plt
import re
import numpy as np
import os

logfile = './evaluation/payload/payload_200g.txt'

# STABILITY: [Target_Angle, Pitch, Acc/Command]
stability_pattern = re.compile(
    r"\[CONTROLLER\]\s+(\d+)\s*\|\s*stability_controller\s*\|\s*\["
    r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*,"
    r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*,"
    r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*\]"
)

# === Parse once ===
speed_rows = []      # (t, adv_v_ref, speed_meas)
stability_rows = []  # (t, target_angle, pitch, acc_cmd)
sonar_rows = []      # (t, sensor_name, distance_m)

with open(logfile, 'r') as f:
    for line in f:
        m = stability_pattern.search(line)
        if m:
            t = int(m.group(1))
            target_angle = float(m.group(2))
            pitch = float(m.group(3))
            acc_cmd = float(m.group(4))
            stability_rows.append((t, target_angle, pitch, acc_cmd))
            continue
        
# === Time base ===
all_ts = []
if stability_rows: all_ts.extend([r[0] for r in stability_rows])

if not all_ts:
    raise RuntimeError("No matching log entries found.")

t0 = min(all_ts)

# === DataFrames ===
# Stability DF
df_stab = pd.DataFrame(stability_rows, columns=['t', 'Target_Angle', 'Pitch', 'Acc_Cmd'])
if not df_stab.empty:
    df_stab['Time'] = (df_stab['t'] - t0) / 1000.0

def plot_stability_only(df_stab):
    if df_stab.empty:
        return
    plt.figure(figsize=(6, 4))
    df_stab['Time'] -= 100
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Angle (°)', fontsize=12)
    plt.plot(df_stab['Time'], df_stab['Pitch'] -0.25, '-', label='Pitch angle')
    plt.axhline(0, color='black', linestyle=':', linewidth=0.8)
    plt.legend(loc='upper right', fontsize=11)
    plt.grid(True)
    plt.xlim(0, 20)
    plt.ylim(0, 0.8)
    plt.tight_layout()
    plt.savefig('./evaluation/payload/payload_200g.pdf')
    plt.close()

plot_stability_only(df_stab)

