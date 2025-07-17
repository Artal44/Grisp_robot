import pandas as pd
import matplotlib.pyplot as plt
import re
import numpy as np

logfile = './robot_debug.txt'

# Temporary buffer to hold log entries for sorting
raw_entries = []

# New regex for format:
# [CONTROLLER] 567993623559 | stability_controller | [0.0,-32.325138448106095,600.0]
log_pattern = re.compile(
    r"\[CONTROLLER\] (\d+) \| (\w+_controller) \| \[([-.\d]+),([-.\d]+),([-.\d]+)\]"
)

with open(logfile, 'r') as f:
    for line in f:
        match = log_pattern.search(line)
        if match:
            t = int(match.group(1))  # Keep raw timestamp for sorting
            ctrl_raw = match.group(2)
            s = float(match.group(3))
            i = float(match.group(4))
            o = float(match.group(5))

            ctrl = ctrl_raw.replace('_controller', '').upper()

            # Convert units
            if ctrl == 'SPEED':
                s /= 100.0
                i /= 100.0
            elif ctrl == 'STABILITY':
                o /= 100.0

            raw_entries.append((t, ctrl, s, i, o))

# Sort entries by timestamp
raw_entries.sort()

# Normalize timestamps to start from zero
timestamps = [t for (t, _, _, _, _) in raw_entries]
t0 = timestamps[0] if timestamps else 0

# Create DataFrame from sorted entries
df = pd.DataFrame({
    'Controller': [ctrl for (_, ctrl, _, _, _) in raw_entries],
    'Time': [(t - t0) / 1000.0 for (t, _, _, _, _) in raw_entries],
    'Setpoint': [s for (_, _, s, _, _) in raw_entries],
    'Input': [i for (_, _, _, i, _) in raw_entries],
    'Output': [o for (_, _, _, _, o) in raw_entries]
})

# === Plotting by controller type ===
for ctrl in df['Controller'].unique():
    subset = df[df['Controller'] == ctrl].reset_index(drop=True)

    # Define labels
    if ctrl == 'SPEED':
        ylabel = 'Speed (m/s) & Target Angle (deg)'
        setpoint_label = 'Adv_V_Ref_New (Speed Setpoint)'
        input_label = 'Speed (Measured)'
        output_label = 'Target Angle (deg)'
    elif ctrl == 'STABILITY':
        ylabel = 'Angle (deg) & Acceleration (m/s²)'
        setpoint_label = 'Target Angle (deg)'
        input_label = 'Angle (Kalman)'
        output_label = 'Acc (Motor Command)'
    else:
        ylabel = 'Value'
        setpoint_label = 'Setpoint'
        input_label = 'Input'
        output_label = 'Output'

    # Plot
    plt.figure(figsize=(10, 6))
    plt.plot(subset['Time'], subset['Setpoint'], label=setpoint_label, linestyle='--', linewidth=2)
    plt.plot(subset['Time'], subset['Input'], label=input_label, linestyle='-', linewidth=1.5)
    plt.plot(subset['Time'], subset['Output'], label=output_label, linestyle='-.', linewidth=1.5)

    plt.axhline(0, color='black', linestyle=':', linewidth=0.8)
    plt.title(f'{ctrl} Controller Response Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.ylim(-5, 5)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f'./graphs/{ctrl}_data_plot.pdf')
    plt.close()
