#!/usr/bin/env python3
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

json_path = Path("artifacts/odometry_evaluation/oneway_evaluation_summary_20261013_045437.json")
with open(json_path) as f:
    data = json.load(f)

# LiDAR Extrinsic calibration: alpha = -7.85 deg
alpha = math.radians(-7.85)
c, s = math.cos(alpha), math.sin(alpha)
R_L_to_B = np.array([[c, -s], [s, c]])

# Extract surge data
surge_runs = data["results"]["surge"]
surge_speeds = [r["speed"] for r in surge_runs]
surge_raw_norms = [r["outbound"]["one_way_error"]["error_norm_mm"] for r in surge_runs]
surge_calib_norms = []
surge_calib_eys = []
surge_error_rates = []

for r in surge_runs:
    out_odom = r["outbound"]["odom_reported"]
    out_icp = r["outbound"]["icp_ground_truth"]
    p_L = np.array([out_icp["dx_m"], out_icp["dy_m"]])
    p_B = R_L_to_B @ p_L
    odom_B = np.array([out_odom["dx_m"], out_odom["dy_m"]])
    err_B = (odom_B - p_B) * 1000.0
    norm_B = float(np.linalg.norm(err_B))
    surge_calib_norms.append(norm_B)
    surge_calib_eys.append(abs(float(err_B[1])))
    surge_error_rates.append(norm_B / 500.0 * 100.0)

# Extract sway data
sway_runs = data["results"]["sway"]
sway_speeds = [r["speed"] for r in sway_runs]
sway_raw_norms = [r["outbound"]["one_way_error"]["error_norm_mm"] for r in sway_runs]
sway_calib_norms = []
sway_calib_exs = []
sway_error_rates = []

for r in sway_runs:
    out_odom = r["outbound"]["odom_reported"]
    out_icp = r["outbound"]["icp_ground_truth"]
    p_L = np.array([out_icp["dx_m"], out_icp["dy_m"]])
    p_B = R_L_to_B @ p_L
    odom_B = np.array([out_odom["dx_m"], out_odom["dy_m"]])
    err_B = (odom_B - p_B) * 1000.0
    norm_B = float(np.linalg.norm(err_B))
    sway_calib_norms.append(norm_B)
    sway_calib_exs.append(abs(float(err_B[0])))
    sway_error_rates.append(norm_B / 500.0 * 100.0)

# Extract rotation data
rot_runs = data["results"]["rotation"]
rot_speeds = [r["speed"] for r in rot_runs]
rot_yaw_errs = [r["outbound"]["one_way_error"]["error_yaw_deg"] for r in rot_runs]
rot_lever_arms = [math.hypot(r["outbound"]["icp_ground_truth"]["dx_m"], r["outbound"]["icp_ground_truth"]["dy_m"]) / math.sqrt(2) * 1000.0 for r in rot_runs]
rot_error_rates = [abs(err) / 90.0 * 100.0 for err in rot_yaw_errs]

# Plotting
fig, axes = plt.subplots(2, 2, figsize=(15, 11), dpi=200)

# 1. Surge Subplot
ax1 = axes[0, 0]
ax1.plot(surge_speeds, surge_raw_norms, "o--", color="#888888", linewidth=1.8, label="Raw LiDAR (Uncalibrated, ~8° skew)")
ax1.plot(surge_speeds, surge_calib_norms, "s-", color="#1f77b4", linewidth=2.4, label="True Wheel Error Norm (Calibrated)")
ax1.plot(surge_speeds, surge_calib_eys, "^:", color="#2ca02c", linewidth=2.0, label="Residual Lateral Slip |ey| (mm)")
ax1.set_title("SURGE (One-Way 0.5m): True Error vs Speed", fontsize=12, fontweight="bold")
ax1.set_xlabel("Linear Speed (m/s)", fontsize=10)
ax1.set_ylabel("Error (mm)", fontsize=10)
ax1.set_ylim(-5, 135)
ax1.grid(True, linestyle=":", alpha=0.6)
ax1.legend(fontsize=9, loc="center left")
for x, y in zip(surge_speeds, surge_calib_norms):
    ax1.annotate(f"{y:.1f}mm", (x, y), textcoords="offset points", xytext=(0, 8), ha="center", fontsize=9, fontweight="bold", color="#1f77b4")

# 2. Sway Subplot
ax2 = axes[0, 1]
ax2.plot(sway_speeds, sway_raw_norms, "o--", color="#888888", linewidth=1.8, label="Raw LiDAR (Uncalibrated, ~8° skew)")
ax2.plot(sway_speeds, sway_calib_norms, "s-", color="#d62728", linewidth=2.4, label="True Wheel Error Norm (Calibrated)")
ax2.plot(sway_speeds, sway_calib_exs, "^:", color="#ff7f0e", linewidth=2.0, label="Residual Longitudinal Slip |ex| (mm)")
ax2.set_title("SWAY (One-Way 0.5m): True Error vs Speed", fontsize=12, fontweight="bold")
ax2.set_xlabel("Linear Speed (m/s)", fontsize=10)
ax2.set_ylabel("Error (mm)", fontsize=10)
ax2.set_ylim(-5, 135)
ax2.grid(True, linestyle=":", alpha=0.6)
ax2.legend(fontsize=9, loc="center left")
for x, y in zip(sway_speeds, sway_calib_norms):
    ax2.annotate(f"{y:.1f}mm", (x, y), textcoords="offset points", xytext=(0, 8), ha="center", fontsize=9, fontweight="bold", color="#d62728")

# 3. Rotation Subplot (Dual Axis)
ax3 = axes[1, 0]
color_yaw = "#9467bd"
ax3.plot(rot_speeds, rot_yaw_errs, "o-", color=color_yaw, linewidth=2.2, label="Heading Error (deg)")
ax3.set_xlabel("Angular Speed (rad/s)", fontsize=10)
ax3.set_ylabel("Heading Error (deg)", color=color_yaw, fontsize=10)
ax3.tick_params(axis="y", labelcolor=color_yaw)
ax3.set_ylim(0.0, 1.2)
ax3.grid(True, linestyle=":", alpha=0.6)
for x, y in zip(rot_speeds, rot_yaw_errs):
    ax3.annotate(f"{y:.2f}°", (x, y), textcoords="offset points", xytext=(0, 8), ha="center", fontsize=9, fontweight="bold", color=color_yaw)

ax3_r = ax3.twinx()
color_arm = "#17becf"
ax3_r.plot(rot_speeds, rot_lever_arms, "s--", color=color_arm, linewidth=2.0, label="LiDAR Mount Lever Arm r (mm)")
ax3_r.set_ylabel("Mount Lever Arm (mm)", color=color_arm, fontsize=10)
ax3_r.tick_params(axis="y", labelcolor=color_arm)
ax3_r.set_ylim(10, 45)
for x, y in zip(rot_speeds, rot_lever_arms):
    ax3_r.annotate(f"{y:.1f}mm", (x, y), textcoords="offset points", xytext=(0, -14), ha="center", fontsize=9, color=color_arm)
ax3.set_title("ROTATION (One-Way 90°): Heading Accuracy & Mount Lever Arm", fontsize=12, fontweight="bold")

# 4. Error Rates Bar Chart
ax4 = axes[1, 1]
labels = [
    "Surge 0.20", "Surge 0.35", "Surge 0.50",
    "Sway 0.20", "Sway 0.35", "Sway 0.50",
    "Rot 0.30", "Rot 0.60", "Rot 1.00"
]
rates = surge_error_rates + sway_error_rates + rot_error_rates
colors = ["#1f77b4"] * 3 + ["#d62728"] * 3 + ["#9467bd"] * 3

bars = ax4.bar(labels, rates, color=colors, width=0.55, alpha=0.85, edgecolor="black", linewidth=0.8)
ax4.axhline(3.0, color="#2ca02c", linestyle="--", linewidth=1.5, label="Target High Precision (<3.0%)")
ax4.axhline(5.0, color="#ff7f0e", linestyle=":", linewidth=1.5, label="Indoor Tolerance Limit (<5.0%)")
ax4.set_title("Calibrated True One-Way Odometry Error Rate (%)", fontsize=12, fontweight="bold")
ax4.set_ylabel("Relative Error Rate (%)", fontsize=10)
ax4.set_ylim(0, 5.5)
ax4.grid(True, linestyle=":", alpha=0.6, axis="y")
ax4.set_xticklabels(labels, rotation=35, ha="right", fontsize=9)
ax4.legend(fontsize=9, loc="upper right")

for bar in bars:
    h = bar.get_height()
    ax4.annotate(f"{h:.2f}%", (bar.get_x() + bar.get_width() / 2, h), textcoords="offset points", xytext=(0, 4), ha="center", fontsize=8.5, fontweight="bold")

plt.suptitle("Cleany Holonomic Odometry: Calibrated One-Way Speed Evaluation\n(LiDAR Extrinsic Yaw Offset α = -7.85° Corrected)", fontsize=14, fontweight="bold", y=0.98)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])

out_plot_path = Path("artifacts/odometry_evaluation/oneway_evaluation_comparison_calibrated_20261013_045437.png")
plt.savefig(out_plot_path, dpi=200)
print(f"Saved calibrated comparison plot to: {out_plot_path}")
