import pandas as pd
import matplotlib.pyplot as plt
import re
import numpy as np
import os

logfile = './robot_debug.txt'

# === Regex ===
NUM = r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)"
speed_pattern = re.compile(
    rf"\[CONTROLLER\]\s+(\d+)\s*\|\s*speed_controller\s*\|\s*\["
    rf"\s*{NUM}\s*,\s*{NUM}\s*,\s*{NUM}(?:\s*,\s*{NUM})?\s*\]",
    re.IGNORECASE
)

# STABILITY: [Target_Angle, Pitch, Acc/Command]
stability_pattern = re.compile(
    r"\[CONTROLLER\]\s+(\d+)\s*\|\s*stability_controller\s*\|\s*\["
    r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*,"
    r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*,"
    r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*\]"
)

# SONAR: robot_main , [MAIN_LOOP] 567993734160 | new_sonar_measure | [robot_front_right,0.4572]
sonar_pattern = re.compile(
    r"\[MAIN_LOOP\]\s+(\d+)\s*\|\s*new_sonar_measure\s*\|\s*\["
    r"\s*(robot_[a-z_]+)\s*,\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s*\]"
)

# === Parse once ===
speed_rows = []      # (t, adv_v_ref, speed_meas)
stability_rows = []  # (t, target_angle, pitch, acc_cmd)
sonar_rows = []      # (t, sensor_name, distance_m)

with open(logfile, 'r') as f:
    for line in f:
        m = speed_pattern.search(line)
        if m:
            t = int(m.group(1))
            adv_v_ref = float(m.group(2))  # 1st numeric in [...]
            # 2nd numeric is turn ref (m.group(3)) if you need it later
            speed_meas = float(m.group(4)) # 3rd numeric in [...]
            # optional target_angle would be m.group(5) if present
            speed_rows.append((t, adv_v_ref, speed_meas))
            continue

        m = stability_pattern.search(line)
        if m:
            t = int(m.group(1))
            target_angle = float(m.group(2))
            pitch = float(m.group(3))
            acc_cmd = float(m.group(4))
            stability_rows.append((t, target_angle, pitch, acc_cmd))
            continue

        m = sonar_pattern.search(line)
        if m:
            t = int(m.group(1))
            sensor = m.group(2)
            distance_m = float(m.group(3))
            sonar_rows.append((t, sensor, distance_m))
            continue

# === Time base ===
all_ts = []
if speed_rows:     all_ts.extend([r[0] for r in speed_rows])
if stability_rows: all_ts.extend([r[0] for r in stability_rows])
if sonar_rows:     all_ts.extend([r[0] for r in sonar_rows])

if not all_ts:
    raise RuntimeError("No matching log entries found.")

t0 = min(all_ts)

# === DataFrames ===
# Speed DF (keep your sign convention later in plotting)
df_speed = pd.DataFrame(speed_rows, columns=['t', 'Adv_V_Ref_New', 'Speed'])
df_speed['Time'] = (df_speed['t'] - t0) / 1000.0

# Stability DF
df_stab = pd.DataFrame(stability_rows, columns=['t', 'Target_Angle', 'Pitch', 'Acc_Cmd'])
if not df_stab.empty:
    df_stab['Time'] = (df_stab['t'] - t0) / 1000.0

# Sonar DF -> pivot to columns per sensor, forward fill for continuous lines
df_sonar = pd.DataFrame(sonar_rows, columns=['t', 'Sensor', 'Distance_m'])
if not df_sonar.empty:
    df_sonar['Time'] = (df_sonar['t'] - t0) / 1000.0
    df_sonar['Time'] 
    # meters -> centimeters for readability next to speed (cm/s)
    df_sonar['Distance_cm'] = df_sonar['Distance_m'] * 100.0

    sonar_wide = (
        df_sonar
        .sort_values('Time')
        .pivot_table(index='Time', columns='Sensor', values='Distance_cm', aggfunc='last')
        .sort_index()
        .ffill()
    )
else:
    sonar_wide = pd.DataFrame()

# === Make output folder ===
os.makedirs('./graphs/controllers', exist_ok=True)

# === Figure 1: Sonars (top) + Speed (bottom) ===
fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

# Top: Sonar distances
if not sonar_wide.empty:
    for col in sonar_wide.columns:
        if col.startswith('robot_main'):
            ax_top.plot(sonar_wide.index, sonar_wide[col], label=col.replace('robot_', '')+ " sonar")
    ax_top.set_ylabel('Distance (cm)', fontsize=12)
    ax_top.axhline(65, linestyle='--', linewidth=0.8, color='green', label='braking threshold')
    ax_top.axhline(30, linestyle='--', linewidth=0.8, color='red', label='emergency threshold')
    ax_top.grid(True, linestyle=':')
    ax_top.legend(loc='best', fontsize=10)
    ax_top.set_xlim(0, 15)
    ax_top.set_ylim(0, 300)
else:
    ax_top.text(0.5, 0.5, 'No sonar data', transform=ax_top.transAxes, ha='center', va='center')

# Bottom: Speed
if not df_speed.empty:
    ax_bot.set_ylabel('Speed (cm/s)', fontsize=12)
    # Keep your original sign convention (negating for your frame choice)
    df_speed['Time'] 
    if 'Adv_V_Ref_New' in df_speed:
        ax_bot.plot(df_speed['Time'], df_speed['Adv_V_Ref_New'], '--', label='Reference speed')
    ax_bot.plot(df_speed['Time'], df_speed['Speed'], '-', label='Robot speed', color='red')
    ax_bot.axhline(0, linestyle=':', linewidth=0.8, color='black')
    ax_bot.grid(True, linestyle=':')
    ax_bot.legend(loc='best', fontsize=10)
    ax_bot.set_xlim(0, 15)
    ax_bot.set_ylim(-10, 40)
else:
    ax_bot.text(0.5, 0.5, 'No speed data', transform=ax_bot.transAxes, ha='center', va='center')

ax_bot.set_xlabel('Time (s)', fontsize=12)

plt.tight_layout()
plt.savefig('./graphs/controllers/SONAR_and_SPEED.pdf')
plt.close()

# === Figure 2: Speed & Stability (two subplots), like your previous ===
def generate_speed_stability_plots(df_speed, df_stab):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    # --- SPEED subplot ---
    if not df_speed.empty:
        ax1.set_ylabel('Speed (cm/s)', fontsize=12)
        if 'Adv_V_Ref_New' in df_speed:
            ax1.plot(df_speed['Time'], -df_speed['Adv_V_Ref_New'], '--', label='Reference speed')
        ax1.plot(df_speed['Time'], -df_speed['Speed'], '-', label='Robot speed')
        ax1.axhline(0, color='black', linestyle=':', linewidth=0.8)
        ax1.legend(loc='upper right', fontsize=11)
        ax1.grid(True)

    # --- STABILITY subplot ---
    if not df_stab.empty:
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Angle (°)', fontsize=12)
        ax2.plot(df_stab['Time'], df_stab['Pitch'], '-', label='Pitch angle')
        ax2.axhline(0, color='black', linestyle=':', linewidth=0.8)
        ax2.legend(loc='upper right', fontsize=11)
        ax2.grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig('./graphs/controllers/SPEED_STABILITY_separate_subplots.pdf')
    plt.close()

# === Figure 3: Stability-only (like your previous STABILITY.pdf) ===
def plot_stability_only(df_stab):
    if df_stab.empty:
        return
    plt.figure(figsize=(8, 3.5))
    df_stab['Time'] 
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Angle (°)', fontsize=12)
    plt.plot(df_stab['Time'], df_stab['Pitch'] -0.2, '-', label='Pitch angle')
    plt.axhline(0, color='black', linestyle=':', linewidth=0.8)
    plt.legend(loc='upper right', fontsize=11)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('./graphs/controllers/STABILITY.pdf')
    plt.close()


plot_stability_only(df_stab)

