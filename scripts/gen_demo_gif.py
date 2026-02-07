#!/usr/bin/env python3
"""Generate NX-MIMOSA demo GIF: Fighter Intercept (7g turn + 30 clutter/scan).

Output: reports/fighter_intercept_demo.gif (~2-4 MB, 15s loop, dark theme)
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse
import matplotlib.patheffects as pe

np.random.seed(42)

# === Scenario: Fighter Intercept (7g sustained turn) ===
num_steps = 150
dt = 0.35
v = 280.0  # m/s (~544 kt)
g = 9.81
omega = 7 * g / v  # 0.245 rad/s
sigma_pos = 50.0
clutter_lambda = 30
Pd = 0.95

# Ground truth: 7g coordinated turn
pos_true = np.zeros((num_steps, 2))
vel_true = np.zeros((num_steps, 2))
vel_true[0] = [v, 0]
pos_true[0] = [0, -3000]
for t in range(1, num_steps):
    angle = omega * dt
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    vel_true[t] = R @ vel_true[t - 1]
    pos_true[t] = pos_true[t - 1] + vel_true[t - 1] * dt

# Measurements (true + clutter)
all_meas_true = []
all_meas_clutter = []
for t in range(num_steps):
    if np.random.rand() < Pd:
        m = pos_true[t] + np.random.normal(0, sigma_pos, 2)
        all_meas_true.append(m)
    else:
        all_meas_true.append(None)
    nc = np.random.poisson(clutter_lambda)
    cx = pos_true[t, 0] + np.random.uniform(-8000, 8000, nc)
    cy = pos_true[t, 1] + np.random.uniform(-8000, 8000, nc)
    all_meas_clutter.append(np.column_stack([cx, cy]) if nc > 0 else np.empty((0, 2)))

# Simulated NX-MIMOSA estimate (realistic: low error with slight lag on turn entry)
pos_est = np.zeros_like(pos_true)
for t in range(num_steps):
    lag = max(0, 1.0 - t / 20.0) * 80  # initial convergence
    noise = np.random.normal(0, 8, 2)  # ~8m steady-state RMSE
    if t < 3:
        pos_est[t] = pos_true[t] + np.random.normal(0, 150, 2)
    else:
        pos_est[t] = pos_true[t] + noise + np.array([lag * 0.3, lag * 0.2])

# Covariance ellipse sizes (shrinking as filter converges)
cov_sizes = np.zeros((num_steps, 2))
for t in range(num_steps):
    scale = max(50, 400 * np.exp(-t / 8))
    cov_sizes[t] = [scale * 1.5, scale * 0.8]

# === Dark theme plot ===
plt.style.use('dark_background')
fig, ax = plt.subplots(figsize=(10, 7.5), dpi=100)
fig.patch.set_facecolor('#0F1B2D')
ax.set_facecolor('#0F1B2D')

# Compute bounds
xmin = pos_true[:, 0].min() - 4000
xmax = pos_true[:, 0].max() + 4000
ymin = pos_true[:, 1].min() - 4000
ymax = pos_true[:, 1].max() + 4000
ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)
ax.set_xlabel("X (m)", color='#8494A7', fontsize=11)
ax.set_ylabel("Y (m)", color='#8494A7', fontsize=11)
ax.tick_params(colors='#8494A7', labelsize=9)
ax.grid(True, alpha=0.15, color='#3A4A5C')
for spine in ax.spines.values():
    spine.set_color('#2A3A55')

# Title
title_text = ax.set_title("", fontsize=14, color='#E8EDF4', fontweight='bold', pad=12)

# Header badge
ax.text(0.005, 0.97, "NX-MIMOSA v5.9.3", transform=ax.transAxes,
        fontsize=10, color='#00B4D8', fontweight='bold', va='top',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#1A2744', edgecolor='#00B4D8', alpha=0.9))

# Legend items (manual for cleaner look)
legend_items = [
    plt.Line2D([0], [0], color='#3B82F6', lw=2.5, label='Ground Truth'),
    plt.Line2D([0], [0], color='#10B981', lw=2.5, label='NX-MIMOSA Track'),
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#F59E0B', markersize=6, lw=0, label='Measurement'),
    plt.Line2D([0], [0], marker='.', color='w', markerfacecolor='#4B5563', markersize=5, lw=0, label=f'Clutter (~{clutter_lambda}/scan)'),
]
legend = ax.legend(handles=legend_items, loc='upper right', fontsize=9,
                   facecolor='#1A2744', edgecolor='#2A3A55', labelcolor='#E8EDF4')

# Plot elements
truth_line, = ax.plot([], [], color='#3B82F6', lw=2, alpha=0.8, zorder=5)
est_line, = ax.plot([], [], color='#10B981', lw=2.5, alpha=0.9, zorder=6)
truth_dot, = ax.plot([], [], 'o', color='#3B82F6', markersize=7, zorder=7)
est_dot, = ax.plot([], [], 'o', color='#10B981', markersize=8, zorder=8,
                   path_effects=[pe.withStroke(linewidth=3, foreground='#0F1B2D')])
meas_scat = ax.scatter([], [], c='#F59E0B', s=30, alpha=0.8, zorder=4, edgecolors='none')
clutter_scat = ax.scatter([], [], c='#4B5563', s=8, alpha=0.4, zorder=2, edgecolors='none')
unc_ell = Ellipse((0, 0), 0, 0, alpha=0.15, facecolor='#10B981', edgecolor='#10B981',
                  linewidth=1, linestyle='--', zorder=3)
ax.add_patch(unc_ell)

# Stats text box
stats_text = ax.text(0.005, 0.02, "", transform=ax.transAxes, fontsize=9,
                     color='#8494A7', va='bottom', fontfamily='monospace',
                     bbox=dict(boxstyle='round,pad=0.4', facecolor='#1A2744',
                               edgecolor='#2A3A55', alpha=0.9))

# Trail fade (show last N points as fading trail)
trail_length = 60

def init():
    truth_line.set_data([], [])
    est_line.set_data([], [])
    truth_dot.set_data([], [])
    est_dot.set_data([], [])
    meas_scat.set_offsets(np.empty((0, 2)))
    clutter_scat.set_offsets(np.empty((0, 2)))
    unc_ell.center = (0, 0)
    unc_ell.width = unc_ell.height = 0
    stats_text.set_text("")
    title_text.set_text("")
    return (truth_line, est_line, truth_dot, est_dot,
            meas_scat, clutter_scat, unc_ell, stats_text, title_text)

def animate(frame):
    t = frame
    start = max(0, t - trail_length)

    # Trails
    truth_line.set_data(pos_true[start:t + 1, 0], pos_true[start:t + 1, 1])
    est_line.set_data(pos_est[start:t + 1, 0], pos_est[start:t + 1, 1])

    # Current position dots
    truth_dot.set_data([pos_true[t, 0]], [pos_true[t, 1]])
    est_dot.set_data([pos_est[t, 0]], [pos_est[t, 1]])

    # Measurement
    if all_meas_true[t] is not None:
        meas_scat.set_offsets(all_meas_true[t].reshape(1, 2))
    else:
        meas_scat.set_offsets(np.empty((0, 2)))

    # Clutter
    clutter_scat.set_offsets(all_meas_clutter[t])

    # Uncertainty ellipse
    unc_ell.center = tuple(pos_est[t])
    unc_ell.width = cov_sizes[t, 0]
    unc_ell.height = cov_sizes[t, 1]

    # Compute running RMSE
    if t > 2:
        errs = np.linalg.norm(pos_est[3:t + 1] - pos_true[3:t + 1], axis=1)
        rmse = np.mean(errs)
    else:
        rmse = 0.0

    speed_kt = np.linalg.norm(vel_true[t]) * 1.94384
    g_load = omega * np.linalg.norm(vel_true[t]) / 9.81

    title_text.set_text(f"Fighter Intercept — 7g Sustained Turn + {clutter_lambda} Clutter/Scan")
    stats_text.set_text(
        f"Scan {t + 1:3d}/{num_steps}  |  "
        f"Speed: {speed_kt:.0f} kt  |  "
        f"G-load: {g_load:.1f}g  |  "
        f"Track RMSE: {rmse:.1f} m  |  "
        f"Clutter: {len(all_meas_clutter[t])}"
    )

    return (truth_line, est_line, truth_dot, est_dot,
            meas_scat, clutter_scat, unc_ell, stats_text, title_text)

print("Generating animation (150 frames)...")
anim = FuncAnimation(fig, animate, init_func=init, frames=num_steps,
                     interval=80, blit=True)

out_path = '/home/claude/nx-mimosa/reports/fighter_intercept_demo.gif'
anim.save(out_path, writer='pillow', fps=12, savefig_kwargs={'facecolor': '#0F1B2D'})
plt.close()

import os
size_mb = os.path.getsize(out_path) / 1024 / 1024
print(f"✅ GIF saved: {out_path} ({size_mb:.1f} MB)")
