import matplotlib.pyplot as plt
import numpy as np
import re
from collections import defaultdict

# Initialisation
data = defaultdict(list)

# Lecture du fichier
with open("robot_debug.txt", "r") as f:
    for line in f:
        match = re.search(r'\[MAIN_LOOP\] (\d+).*?\[(robot_.*?),([\d.]+)\]', line)
        if match:
            timestamp = int(match.group(1))
            role = match.group(2)
            value = float(match.group(3))
            data[role].append((timestamp, value))

# Tri et alignement des timestamps
for role in data:
    data[role].sort()
    t0 = data[role][0][0]
    data[role] = [(ts - t0, dist) for ts, dist in data[role]]

# --- PLOT TEMPOREL ---
plt.figure(figsize=(12, 6))
for role in data:
    ts, vals = zip(*data[role])
    plt.plot(np.array(ts)/1000, vals, label=role)  # conversion ms -> s

plt.title("Sonar Distances Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("./graphs/sonar_time.pdf")

# --- PLOT FOURIER ---
plt.figure(figsize=(12, 6))
for role in data:
    _, vals = zip(*data[role])
    vals = np.array(vals)
    fft_vals = np.fft.fft(vals - np.mean(vals))
    fft_freqs = np.fft.fftfreq(len(vals), d=0.03)  # ≈ 33 Hz sampling if 30 ms loop
    fft_magnitudes = np.abs(fft_vals)

    # On trace uniquement les fréquences positives
    pos_mask = fft_freqs > 0
    plt.plot(fft_freqs[pos_mask], fft_magnitudes[pos_mask], label=f"FFT {role}")

plt.title("Sonar Signal Frequency Spectrum (FFT)")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Amplitude")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("./graphs/sonar_fft.pdf")

