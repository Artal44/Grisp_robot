import re
import matplotlib.pyplot as plt

# Path to your log file
log_file = "robot_debug.txt"

timestamps = []
frequencies = []

# Regex to capture lines with frequency and timestamp
pattern = re.compile(r".*robot_frequency.*\[(.*?)\]")
timestamp_pattern = re.compile(r".*\[MAIN_LOOP\]\s+(\d+)")

with open(log_file, "r") as f:
    for line in f:
        if "robot_frequency" in line:
            # Extract the timestamp
            ts_match = timestamp_pattern.search(line)
            if ts_match:
                ts = int(ts_match.group(1)) / 1000.0  # convert from microseconds -> ms
            else:
                ts = len(timestamps)

            # Extract frequency values
            match = pattern.search(line)
            if match:
                values = match.group(1).split(",")
                freq = float(values[0])  # Instantaneous frequency
                frequencies.append(freq)
                timestamps.append(ts)

# Normalize timestamps (start at 0)
if timestamps:
    t0 = timestamps[0]
    timestamps = [t - t0 for t in timestamps]

# Plot
plt.figure(figsize=(10,5))
plt.plot(timestamps, frequencies, label="Frequency (Hz)", color='b')

plt.axhline(200, linestyle='--', color='r', label='Target 200Hz')
plt.xlabel("Time (ms)")
plt.ylabel("Frequency (Hz)")
plt.title("Robot Loop Frequency Over Time")
plt.legend()
plt.grid(True)
plt.savefig('./graphs/frequency_plot.pdf')
plt.show()
