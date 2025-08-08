import re
from collections import defaultdict
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

log_file = "robot_debug.txt"

# Read the whole file at once (so we can match across multiple lines)
with open(log_file, "r") as f:
    content = f.read()

# Regex: capture full TIMING block, even across lines
pattern = re.compile(r"\[TIMING\].*?\|\s*(\[[\s\S]*?\])")

# Data storage
timing_data = defaultdict(list)
all_iterations = []

# Find all TIMING blocks
matches = pattern.findall(content)
for match in matches:
    # Find all {"Name", value}
    entries = re.findall(r'\{"([^"]+)",\s*([0-9]+)\}', match)

    iteration = {}
    for name, value in entries:
        val = int(value)
        timing_data[name].append(val)
        iteration[name] = val
    all_iterations.append(iteration)

# Validate
if not timing_data:
    print("⚠️ No TIMING logs found! Check your log file content.")
    exit()

# Compute averages
averages = {name: sum(values) / len(values) for name, values in timing_data.items()}

# ---- Print average timings ----
print("\nAverage timing (microseconds):")
for name, avg in averages.items():
    print(f"{name:20s} {avg:.2f}")

# ---- Average timing bar chart ----
names = list(averages.keys())
values = [averages[n] for n in names]

plt.figure(figsize=(12, 6))
bars = plt.bar(names, values, color="skyblue")
plt.ylabel("Average Execution Time (µs)")
plt.title("Average Execution Time per Section in robot_loop")
plt.xticks(rotation=30, ha="right")
plt.grid(axis="y", linestyle="--", alpha=0.7)

for bar in bars:
    plt.text(bar.get_x() + bar.get_width() / 2,
             bar.get_height() + 50,
             f"{bar.get_height():.0f}",
             ha="center", va="bottom")

plt.tight_layout()
plt.savefig("./graphs/timing/timing_average.pdf")

# ---- Stacked chart for first N iterations ----
sections = [name for name in names if name != "Total"]
iterations_to_plot = all_iterations[:50]

fig, ax = plt.subplots(figsize=(14, 7))

# Generate distinct colors for each section using a colormap
cmap = plt.get_cmap('tab20', len(sections))
colors = [cmap(i) for i in range(len(sections))]

bottoms = np.zeros(len(iterations_to_plot))
for i, section in enumerate(sections):
    values = [it.get(section, 0) for it in iterations_to_plot]
    ax.bar(range(len(iterations_to_plot)), values, bottom=bottoms, label=section, color=colors[i])
    bottoms += np.array(values)

ax.set_xticks(range(len(iterations_to_plot)))
ax.set_xticklabels([f"{i+1}" for i in range(len(iterations_to_plot))], rotation=45)
ax.set_ylabel("Time (µs)")
ax.set_title(f"Timing Breakdown per Iteration (first {len(iterations_to_plot)} iterations)")
ax.legend(loc="upper left", bbox_to_anchor=(1, 1))  # Move legend outside
plt.tight_layout()
plt.savefig("./graphs/timing/timing_stacked.pdf")


