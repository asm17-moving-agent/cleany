"""Response metrics and plots from measured wheel-odometry controller traces."""
from __future__ import annotations

import math
from pathlib import Path

from motion_patterns import PATTERN_DIRECTIONS

AXES = {"x": "surge", "y": "sway", "yaw": "rotation",
        "diagonal_left": "diagonal_left", "diagonal_right": "diagonal_right"}


def translation_direction(axis: str) -> tuple[float, float]:
    """Unit direction in the leg's initial body frame; speed is vector magnitude."""
    return PATTERN_DIRECTIONS[AXES[axis]]


def project_translation(x: float, y: float, axis: str) -> tuple[float, float]:
    ux, uy = translation_direction(axis)
    return ux*x + uy*y, -uy*x + ux*y


def endpoint_errors(result: dict, axis: str) -> dict:
    """Separate wheel-odom goal residual, ICP goal residual and odom-minus-ICP."""
    if axis == "yaw":
        return {}
    target = result["nominal_displacement"]
    output = {}
    for leg, sign in (("outbound", 1.), ("inbound", -1.)):
        data = result[leg]
        def components(pose):
            along, cross = project_translation(pose["dx_m"], pose["dy_m"], axis)
            return {"progress_m": sign*along, "shortfall_mm": (target-sign*along)*1000,
                    "cross_track_mm": sign*cross*1000, "yaw_drift_deg": pose["dyaw_deg"],
                    "goal_error_norm_mm": math.hypot(target-sign*along, cross)*1000}
        wheel = components(data["odom_reported"])
        entry = {"wheel_goal": wheel, "icp_valid": data["is_valid"],
                 "icp_goal": None, "odom_minus_icp": None}
        if data["is_valid"]:
            icp = components(data["calibrated_base_ground_truth"])
            entry["icp_goal"] = icp
            entry["odom_minus_icp"] = {
                "along_mm": (wheel["progress_m"] - icp["progress_m"])*1000,
                "cross_mm": wheel["cross_track_mm"] - icp["cross_track_mm"],
                **data["one_way_error"],
            }
        output[leg] = entry
    return output


def response_metrics(result: dict, axis: str) -> dict:
    trace = result.get("trace", [])
    if not trace or not result.get("is_valid"):
        return {"valid": False}
    cfg = result["motion_config"]
    rotation = axis == "yaw"
    tol = cfg["yaw_tolerance_rad" if rotation else "linear_tolerance_m"]
    stop_speed = cfg["stop_angular_speed_rad_s" if rotation else "stop_linear_speed_mps"]
    speed_key = "angular_speed_rad_s" if rotation else "linear_speed_mps"
    target = result["target_displacement"]
    displacement = [target - s["position_error"] for s in trace]
    times = [s["elapsed_s"] for s in trace]
    violations = [i for i, s in enumerate(trace)
                  if abs(s["position_error"]) > tol or abs(s[speed_key]) > stop_speed]
    settle_index = violations[-1] + 1 if violations else 0
    settling = times[settle_index] if settle_index < len(trace) else None
    first = lambda fraction: next((s["elapsed_s"] for s, pos in zip(trace, displacement)
                                    if pos >= fraction * target), None)
    t10, t90, t80 = first(.1), first(.9), first(.8)
    return {
        "valid": settling is not None and bool(result["reached_tolerance"]),
        "completion_s": times[-1], "settling_entry_s": settling,
        "rise_10_90_s": t90 - t10 if t10 is not None and t90 is not None else None,
        "last_20pct_s": settling - t80 if settling is not None and t80 is not None else None,
        "max_overshoot": result["max_overshoot"], "final_error": result["final_error"],
        "peak_odom_speed": max(abs(s[speed_key]) for s in trace),
        "units": "rad" if rotation else "m",
    }


def plot_responses(records: list[dict], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    axes = [axis for axis in AXES if any(r["axis"] == axis for r in records)]
    if not axes:
        return
    fig, axs = plt.subplots(3, len(axes), figsize=(5.7*len(axes), 11), dpi=150, squeeze=False)
    colors = plt.get_cmap("tab10")
    for col, axis in enumerate(axes):
        group = [r for r in records if r["axis"] == axis and r.get("result")]
        keys = sorted({(r["kp"], r["speed"]) for r in group})
        labelled = set()
        for record in group:
            color = colors(keys.index((record["kp"], record["speed"])) % 10)
            for leg, result in record["result"]["control_stopping_performance"].items():
                trace = result.get("trace", [])
                if not trace:
                    continue
                rotation = axis == "yaw"
                scale = 180 / math.pi if rotation else 1000
                target = result["target_displacement"]
                t = [s["elapsed_s"] for s in trace]
                pos = [(target-s["position_error"])*scale for s in trace]
                error = [s["position_error"]*scale for s in trace]
                speed = [abs(s["angular_speed_rad_s"]) if rotation else s["linear_speed_mps"]
                         for s in trace]
                cmd = [abs(s["command"][2]) if rotation else math.hypot(*s["command"][:2])
                       for s in trace]
                key = (record["kp"], record["speed"])
                label = f"Kp={record['kp']:g}, cap={record['speed']:g}" if key not in labelled else None
                labelled.add(key)
                style = "-" if leg == "outbound" else ":"
                axs[0, col].plot(t, pos, style, color=color, label=label, alpha=.85)
                axs[1, col].plot(t, error, style, color=color, alpha=.85)
                axs[2, col].plot(t, speed, style, color=color, alpha=.85)
                axs[2, col].plot(t, cmd, "--", color=color, alpha=.28)
        if group:
            result = group[0]["result"]["control_stopping_performance"]["outbound"]
            rotation = axis == "yaw"
            scale = 180 / math.pi if rotation else 1000
            tol = result["motion_config"]["yaw_tolerance_rad" if rotation else "linear_tolerance_m"] * scale
            target = result["target_displacement"] * scale
            axs[0, col].axhline(target, color="black", lw=1, ls="--")
            axs[0, col].axhspan(target-tol, target+tol, color="grey", alpha=.15)
            axs[1, col].axhspan(-tol, tol, color="grey", alpha=.15)
        axs[0, col].set_title(f"{axis.upper()} measured response")
        axs[0, col].set_ylabel("Progress (deg)" if axis == "yaw" else "Progress (mm)")
        axs[1, col].set_ylabel("Position error (deg)" if axis == "yaw" else "Position error (mm)")
        axs[2, col].set_ylabel("Speed magnitude (rad/s)" if axis == "yaw" else "Speed magnitude (m/s)")
        axs[2, col].set_xlabel("Time from leg initialization (s)")
        if group:
            axs[0, col].legend(fontsize=8, loc="lower right")
        for row in range(3):
            axs[row, col].grid(alpha=.25)
    fig.suptitle("Physical P response — wheel odometry feedback\n"
                 "Solid: outbound; dotted: inbound; faint dashed: commanded speed. "
                 "Post-completion dwell / ICP capture excluded.", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(output)
    plt.close(fig)
