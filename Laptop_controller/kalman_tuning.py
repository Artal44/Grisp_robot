import re
import matplotlib.pyplot as plt
import numpy as np

# === Stockage temporaire pour tri ===
entries_kalman = []
entries_pred = []

# === Lecture du fichier ===
with open('./robot_debug.txt', 'r') as f:
    for line in f:
        # Mesure avec correction
        if 'kalman_comparison' in line and 'predict_only' not in line:
            m = re.search(
                r'\[\w+\]\s+(\d+)\s+\|\s+kalman_comparison\s+\|\s+\[([-\d.eE]+),\s*([-\d.eE]+)\]',
                line
            )
            if m:
                t = int(m.group(1))
                entries_kalman.append((t, float(m.group(2)), float(m.group(3))))

        # Prédiction uniquement
        elif 'kalman_comparison_predict_only' in line:
            m = re.search(
                r'\[\w+\]\s+(\d+)\s+\|\s+kalman_comparison_predict_only\s+\|\s+\[([-\d.eE]+)\]',
                line
            )
            if m:
                t = int(m.group(1))
                entries_pred.append((t, float(m.group(2))))

# === Tri par timestamp ===
entries_kalman.sort()
entries_pred.sort()

# === Initialisation des séries triées ===
timestamps = [(t - entries_kalman[0][0]) / 1000.0 for (t, _, _,) in entries_kalman] if entries_kalman else []
kalman_new = [x for (_, x, _) in entries_kalman]
measured =   [x for (_, _, x) in entries_kalman]

pred_timestamps = [(t - entries_kalman[0][0]) / 1000.0 for (t, _) in entries_pred] if entries_pred else []
pred_kalman_new = [x for (_, x) in entries_pred]

# === Graphique principal ===
fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

axs[0].plot(timestamps, kalman_new, label='NEW Kalman', color='tab:blue')
axs[0].plot(timestamps, measured, label='Measured', color='gray', linestyle='--', alpha=0.6)
axs[0].scatter(pred_timestamps, pred_kalman_new, label='Predicted only (NEW)', color='red', marker='x', s=30)
axs[0].set_ylabel('Angle (°)')
axs[0].set_title('NEW Kalman vs Measured')
axs[0].legend()
axs[0].grid(True, linestyle=':', linewidth=0.6, alpha=0.7)

plt.tight_layout()
plt.savefig('./graphs/kalman_comparison_split.pdf', bbox_inches='tight')

# === Graphique zoomé en Y ===
fig_zoom, axs_zoom = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

axs_zoom[0].plot(timestamps, kalman_new, label='NEW Kalman', color='tab:blue')
axs_zoom[0].plot(timestamps, measured, label='Measured', color='gray', linestyle='--', alpha=0.6)
axs_zoom[0].scatter(pred_timestamps, pred_kalman_new, label='Predicted only (NEW)', color='red', marker='x', s=30)
axs_zoom[0].set_ylabel('Angle (°)')
axs_zoom[0].set_title('NEW Kalman vs Measured (Zoom Y)')
axs_zoom[0].set_ylim([-5, 5])
axs_zoom[0].legend()
axs_zoom[0].grid(True, linestyle=':', linewidth=0.6, alpha=0.7)

plt.tight_layout()
plt.savefig('./graphs/kalman_comparison_split_zoom.pdf', bbox_inches='tight')

# === RMSE ===
rmse_new = np.sqrt(np.mean((np.array(kalman_new) - np.array(measured))**2)) if kalman_new else 0.0
print(f"NEW Kalman RMSE: {rmse_new:.2f}°")

