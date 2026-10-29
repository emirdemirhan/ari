"""Pass/fail metrics, run artefacts (CSV log, JSON summary) and the plot sheet."""

from __future__ import annotations

import json
import math
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .config import Criterion, derived_vehicle_values
from .simulator import LOG_COLUMNS, RunResult

RAD2DEG = 180.0 / math.pi


def _mask(log, window):
    t = log["t"]
    if window is None:
        return np.ones_like(t, dtype=bool)
    return (t >= window[0]) & (t <= window[1])


def _estimate(log, estimator: str, name: str):
    if estimator == "kf":
        return log[f"kf_{name}"]
    if estimator == "eskf":
        return log[f"eskf_{name}"]
    use = log["use_eskf"] > 0.5
    return np.where(use, log[f"eskf_{name}"], log[f"kf_{name}"])


def _rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def compute_metric(res: RunResult, c: Criterion):
    """Returns the metric value, or None when it does not apply to this run."""
    log = res.log
    if c.metric == "no_crash":
        return 1.0 if res.crashed else 0.0
    m = _mask(log, c.window)
    if not m.any():
        return None
    err_h = log["agl"][m] - log["ref_alt"][m]
    if c.metric == "height_error_final_m":
        return abs(float(np.mean(err_h)))
    if c.metric == "height_rmse_m":
        return _rms(err_h)
    if c.metric == "height_max_m":
        return float(np.max(log["agl"][m]))
    if c.metric == "height_min_m":
        return float(np.min(log["agl"][m]))
    if c.metric == "height_overshoot_m":
        return float(np.max(err_h))
    if c.metric == "tilt_max_deg":
        tilt = np.arccos(np.clip(np.cos(log["roll"][m]) * np.cos(log["pitch"][m]), -1, 1))
        return float(np.max(tilt)) * RAD2DEG
    if c.metric == "est_tilt_error_max_deg":
        er = np.abs(_estimate(log, c.estimator, "roll")[m] - log["roll"][m])
        ep = np.abs(_estimate(log, c.estimator, "pitch")[m] - log["pitch"][m])
        return float(max(er.max(), ep.max())) * RAD2DEG
    if c.metric == "est_height_error_max_m":
        return float(np.max(np.abs(_estimate(log, c.estimator, "alt")[m] - log["agl"][m])))
    if c.metric == "attitude_tracking_rmse_deg":
        er = log["roll"][m] - log["ref_roll"][m]
        ep = log["pitch"][m] - log["ref_pitch"][m]
        return _rms(np.concatenate([er, ep])) * RAD2DEG
    if c.metric == "yaw_rate_tracking_rmse_dps":
        return _rms(log["r"][m] - log["ref_yaw_rate"][m]) * RAD2DEG
    if c.metric == "drift_xy_max_m":
        x, y = log["x"][m], log["y"][m]
        return float(np.max(np.hypot(x - x[0], y - y[0])))
    if c.metric == "exec_time_max_us":
        ex = log["exec_us"][m]
        return float(np.max(ex)) if np.any(ex > 0) else None      # SIL reports 0
    if c.metric == "motor_saturation_percent":
        mot = np.stack([log[f"m{i}"][m] for i in range(1, 5)])
        sat = np.any((mot <= 0.01) | (mot >= 99.99), axis=0)
        return 100.0 * float(np.mean(sat))
    if c.metric == "battery_min_v":
        return float(np.min(log["battery_v"][m]))
    raise ValueError(f"unknown metric '{c.metric}'")


def evaluate(res: RunResult) -> list:
    out = []
    criteria = list(res.scenario.criteria)
    if not any(c.metric == "no_crash" for c in criteria):
        criteria.insert(0, Criterion(metric="no_crash", max=0))
    for c in criteria:
        value = compute_metric(res, c)
        if value is None:
            passed = None
        else:
            passed = True
            if c.max is not None and not value <= c.max:
                passed = False
            if c.min is not None and not value >= c.min:
                passed = False
        out.append({"metric": c.metric, "estimator": c.estimator, "window": c.window, "value": value,
                     "max": c.max, "min": c.min, "passed": passed})
    return out


def write_artifacts(res: RunResult, results: list, out_dir: Path, plot: bool = True) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = np.column_stack([res.log[c] for c in LOG_COLUMNS])
    np.savetxt(out_dir / "log.csv", rows, delimiter=",", header=",".join(LOG_COLUMNS), comments="", fmt="%.6g")

    stats = dict(res.link_stats)
    rtt = stats.pop("rtt_ms", None)
    if rtt:
        stats["rtt_ms_mean"] = float(np.mean(rtt))
        stats["rtt_ms_p99"] = float(np.percentile(rtt, 99))
        stats["rtt_ms_max"] = float(np.max(rtt))
    sim_time = float(res.log["t"][-1]) if len(res.log["t"]) else 0.0
    summary = {
        "scenario": res.scenario.name,
        "description": res.scenario.description,
        "vehicle": res.vehicle.name,
        "link": res.link,
        "seed": res.scenario.seed,
        "passed": all(r["passed"] is not False for r in results),
        "crashed": res.crashed,
        "crash_reason": res.crash_reason,
        "crash_time_s": res.crash_time,
        "link_error": res.link_error,
        "sim_time_s": sim_time,
        "wall_time_s": res.wall_time_s,
        "realtime_factor": sim_time / res.wall_time_s if res.wall_time_s > 0 else None,
        "criteria": results,
        "faults_applied": res.faults,
        "link_stats": stats,
        "vehicle_derived": derived_vehicle_values(res.vehicle, res.scenario.gravity_mps2),
        "vehicle_params": asdict(res.vehicle),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=str))
    if plot:
        plot_run(res, results, out_dir / "report.png")
    return out_dir


def plot_run(res: RunResult, results: list, path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    L = res.log
    t = L["t"]
    d = RAD2DEG
    fig, axs = plt.subplots(4, 3, figsize=(18, 16), sharex=False)
    status = "PASS" if all(r["passed"] is not False for r in results) else "FAIL"
    crash = f" | CRASH: {res.crash_reason} at {res.crash_time:.2f} s" if res.crashed else ""
    if res.link_error:
        crash += " | LINK LOST"

    fig.suptitle(f"{res.scenario.name} [{res.link.upper()}] - {status}{crash}\n{res.scenario.description}",
                 fontsize=12)

    def panel(ax, title, ylabel, series):
        for y, label, style in series:
            ax.plot(t, y, style, label=label, linewidth=1)
        ax.set_title(title, fontsize=10)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="best")

    panel(axs[0, 0], "Height above terrain (rangefinder position)", "m",
          [(L["agl"], "true", "k-"), (L["ref_alt"], "reference", "r--"),
           (L["kf_alt"], "KF", "C0-"), (L["eskf_alt"], "ESKF", "C1-"), (L["terrain"], "terrain", "C7:")])
    panel(axs[0, 1], "Vertical speed", "m/s",
          [(L["vz"], "true", "k-"), (L["target_vz"], "target (outer loop)", "r--"),
           (L["kf_vz"], "KF", "C0-"), (L["eskf_vz"], "ESKF", "C1-")])
    panel(axs[0, 2], "Collective and motor commands", "%",
          [(L["total_thrust"], "collective", "k-")] + [(L[f"m{i}"], f"M{i}", f"C{i}-") for i in range(1, 5)])
    panel(axs[1, 0], "Roll", "deg",
          [(L["roll"] * d, "true", "k-"), (L["ref_roll"] * d, "reference", "r--"),
           (L["kf_roll"] * d, "KF", "C0-"), (L["eskf_roll"] * d, "ESKF", "C1-")])
    panel(axs[1, 1], "Pitch", "deg",
          [(L["pitch"] * d, "true", "k-"), (L["ref_pitch"] * d, "reference", "r--"),
           (L["kf_pitch"] * d, "KF", "C0-"), (L["eskf_pitch"] * d, "ESKF", "C1-")])
    panel(axs[1, 2], "Yaw", "deg",
          [(np.unwrap(L["yaw"]) * d, "true", "k-"), (np.unwrap(L["kf_yaw"]) * d, "KF", "C0-"),
           (np.unwrap(L["eskf_yaw"]) * d, "ESKF", "C1-")])

    tilt_err_kf = np.maximum(np.abs(L["kf_roll"] - L["roll"]), np.abs(L["kf_pitch"] - L["pitch"])) * d
    tilt_err_eskf = np.maximum(np.abs(L["eskf_roll"] - L["roll"]), np.abs(L["eskf_pitch"] - L["pitch"])) * d
    panel(axs[2, 0], "Attitude estimate error (max of roll, pitch)", "deg",
          [(tilt_err_kf, "KF", "C0-"), (tilt_err_eskf, "ESKF", "C1-")])
    panel(axs[2, 1], "Body rates", "deg/s",
          [(L["p"] * d, "p", "C0-"), (L["q"] * d, "q", "C1-"), (L["r"] * d, "r", "C2-"),
           (L["ref_yaw_rate"] * d, "yaw rate ref", "r--"),
           (L["roll_rate_ref"] * d, "roll rate ref", "C0:"), (L["pitch_rate_ref"] * d, "pitch rate ref", "C1:")])
    panel(axs[2, 2], "IMU accelerometer as sent (vibration + noise)", "m/s^2",
          [(L["accel_x"], "x", "C0-"), (L["accel_y"], "y", "C1-"), (L["accel_z"], "z", "C2-")])

    ax = axs[3, 0]
    ax.plot(L["x"], L["y"], "k-", linewidth=1)
    ax.plot(L["x"][:1], L["y"][:1], "go", label="start")
    ax.plot(L["x"][-1:], L["y"][-1:], "rs", label="end")
    for b in res.scenario.terrain:
        cx, cy = b.center_m
        sx, sy = b.size_m
        ax.add_patch(plt.Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, alpha=0.2, color="C7"))
    ax.set_title("Ground track (world x forward at yaw 0, y left)", fontsize=10)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)

    panel(axs[3, 1], "Velocity through the air, body frame (true vs ESKF)", "m/s",
          [(L["air_u"], "u true", "k-"), (L["eskf_u"], "u ESKF", "C1-"),
           (L["air_v"], "v true", "C7-"), (L["eskf_v"], "v ESKF", "C3-")])
    ax = axs[3, 2]
    ax.plot(t, L["battery_v"], "k-", linewidth=1, label="battery V")
    ax.set_ylabel("V")
    ax.grid(alpha=0.3)
    ax2 = ax.twinx()
    if np.any(L["exec_us"] > 0):
        ax2.plot(t, L["exec_us"], "C3-", linewidth=0.8, label="controller exec (us)")
        ax2.set_ylabel("exec us")
    else:
        ax2.plot(t, L["battery_a"], "C3-", linewidth=0.8, label="battery A")
        ax2.set_ylabel("A")
    ax.set_title("Battery / controller execution time", fontsize=10)
    ax.legend(fontsize=7, loc="upper left")
    ax2.legend(fontsize=7, loc="upper right")

    for ax in axs[:, :].flat:
        if ax is not axs[3, 0]:
            ax.set_xlabel("t (s)")

    lines = []
    for r in results:
        v = "n/a" if r["value"] is None else f"{r['value']:.3f}"
        lim = (f"<= {r['max']}" if r["max"] is not None else "") + (f" >= {r['min']}" if r["min"] is not None else "")
        mark = "n/a" if r["passed"] is None else ("PASS" if r["passed"] else "FAIL")
        est = f" [{r['estimator']}]" if r["metric"].startswith("est_") else ""
        win = f" {r['window'][0]}-{r['window'][1]} s" if r["window"] else ""
        lines.append(f"{mark:4s}  {r['metric']}{est}{win}: {v} {lim}")
    fig.text(0.01, 0.005, "\n".join(lines), family="monospace", fontsize=8, va="bottom")
    fig.tight_layout(rect=(0, 0.02 + 0.012 * len(lines), 1, 0.96))
    fig.savefig(path, dpi=90)
    plt.close(fig)


def format_results(res: RunResult, results: list) -> str:
    lines = []
    for r in results:
        v = "n/a" if r["value"] is None else f"{r['value']:.3f}"
        lim = (f"<= {r['max']:g}" if r["max"] is not None else "") + (f" >= {r['min']:g}" if r["min"] is not None else "")
        mark = " n/a" if r["passed"] is None else ("PASS" if r["passed"] else "FAIL")
        est = f" [{r['estimator']}]" if r["metric"].startswith("est_") else ""
        win = f" @{r['window'][0]:g}-{r['window'][1]:g}s" if r["window"] else ""
        lines.append(f"  {mark}  {r['metric']}{est}{win}: {v} {lim}")
    return "\n".join(lines)
