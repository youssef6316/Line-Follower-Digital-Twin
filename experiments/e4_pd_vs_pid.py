"""
e4_pd_vs_pid.py — Experiment 4: PD vs PID Ablation
====================================================
Spec requirement:
  Compare PD (Ki=0) vs PID on a curved path under noise; discuss results.

What this script does:
  - Runs PID (G3_baseline) vs PD (G5_no_integral) on the S-curve
  - Under two noise conditions: low (5 mm) and medium (20 mm)
  - N_SPAWNS random starts each condition
  - Side-by-side KPI comparison + trajectory overlay
  - Integral term trace to show why Ki matters on curves

The key insight to discuss in the report:
  On a curved path, the robot experiences a PERSISTENT lateral error
  because constant heading correction is needed.  Without the integral
  term, this steady-state error cannot be eliminated — the PD controller
  reaches an equilibrium with a residual offset.  The integral term
  accumulates this offset and drives it to zero.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client1_plant"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client2_controller"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client3_visualizer"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from client1_plant.robot      import DifferentialDriveRobot
from client1_plant.path       import make_s_curve, make_random_spawn
from client2_controller.errors     import compute_errors
from client2_controller.pid        import PIDController, GAINS_PID, GAINS_PD
from client3_visualizer.kpi        import compute_kpis, extract_lateral_series
from client3_visualizer.logger     import ExperimentLogger
from vsi_bridge import SharedBus, VSIBridge

# ── Parameters ─────────────────────────────────────────────────────────────
N_SPAWNS      = 5
PLANT_DT      = 0.01
CONTROL_DT    = 0.05
CONTROL_RATIO = int(CONTROL_DT / PLANT_DT)
MAX_STEPS     = 6000
SETTLING_TOL  = 0.08
NOISE_LOW     = 0.005
NOISE_MED     = 0.02
RNG_SEED      = 3
EXP_NAME      = "e4_pd_vs_pid"
RESULTS_DIR   = os.path.join(os.path.dirname(__file__), "..", "results")


def run_episode(path, gains, x0, y0, theta0, noise_pos):
    """Run one episode; also return integral history for analysis."""
    robot = DifferentialDriveRobot(
        x0=x0, y0=y0, theta0=theta0, dt=PLANT_DT,
        noise_std_pos=noise_pos, noise_std_theta=noise_pos * 0.5,
    )
    pid  = PIDController(gains, dt=CONTROL_DT)
    traj = []
    v_cmd, omega_cmd = gains.v_ref, 0.0

    for step in range(MAX_STEPS):
        pose = robot.step(v_cmd, omega_cmd)
        traj.append(pose)

        if step % CONTROL_RATIO == 0:
            nr   = path.get_nearest(pose["x"], pose["y"])
            errs = compute_errors(pose["x"], pose["y"], pose["theta"], nr)
            v_cmd, omega_cmd = pid.step(errs.e_lat, errs.e_head)

        if path.is_finished(pose["x_true"], pose["y_true"]):
            break

    times, e_lats = extract_lateral_series(traj, path)
    # Extract integral history from PID (sampled at controller rate)
    i_lat_hist  = [h["I_lat"]  for h in pid.history]
    i_head_hist = [h["I_head"] for h in pid.history]
    ctrl_times  = [traj[min(j * CONTROL_RATIO, len(traj)-1)]["t"]
                   for j in range(len(pid.history))]
    return traj, times, e_lats, ctrl_times, i_lat_hist, i_head_hist


def run_condition(path, gains, noise, label, exp_logger):
    """Run N_SPAWNS for one (gains, noise) condition. Returns aggregated data."""
    spawn_rng   = np.random.default_rng(RNG_SEED)
    kpis        = []
    rep_times   = rep_elats = rep_ct = rep_ilat = rep_ihead = None

    print(f"\n  [{label} — {gains.name}]")
    for sp in range(N_SPAWNS):
        x0, y0, t0 = make_random_spawn(path, lateral_range=0.5,
                                       heading_noise_deg=10, rng=spawn_rng)
        traj, times, e_lats, ct, il, ih = run_episode(
            path, gains, x0, y0, t0, noise
        )
        kpi = compute_kpis(times, e_lats, settling_tol=SETTLING_TOL,
                           gains_name=f"{gains.name}_{label}_sp{sp}")
        exp_logger.add(kpi)
        kpis.append(kpi)

        if sp == 0:   # keep representative run for plotting
            rep_times = times; rep_elats = e_lats
            rep_ct    = ct;    rep_ilat  = il; rep_ihead = ih

        print(f"    spawn {sp}: OS={kpi.overshoot_m:.3f}m  "
              f"ts={kpi.settling_time_s:.2f}s  "
              f"SS={kpi.steady_state_error_m:.4f}m  "
              f"{'✓' if kpi.settled else '✗'}")

    return kpis, rep_times, rep_elats, rep_ct, rep_ilat, rep_ihead


def main():
    path       = make_s_curve()
    exp_logger = ExperimentLogger(EXP_NAME, results_dir=RESULTS_DIR)

    print(f"\n{'='*65}")
    print(f"  E4 — PD vs PID Ablation on S-Curve")
    print(f"  Noise: low={NOISE_LOW}m  medium={NOISE_MED}m")
    print(f"{'='*65}")

    conditions = [
        (GAINS_PID, NOISE_LOW,  "PID_LowNoise"),
        (GAINS_PD,  NOISE_LOW,  "PD_LowNoise"),
        (GAINS_PID, NOISE_MED,  "PID_MedNoise"),
        (GAINS_PD,  NOISE_MED,  "PD_MedNoise"),
    ]

    cond_data = {}
    for gains, noise, label in conditions:
        cond_data[label] = run_condition(path, gains, noise, label, exp_logger)

    exp_logger.finalize()
    _plot_results(path, cond_data, conditions)


def _plot_results(path, cond_data, conditions):
    fig = plt.figure(figsize=(18, 13))
    fig.suptitle("E4 — PD vs PID Ablation on S-Curve", fontsize=14, fontweight="bold")

    colors = {"PID_LowNoise": "#1f77b4", "PD_LowNoise":  "#aec7e8",
              "PID_MedNoise": "#d62728", "PD_MedNoise":  "#ff9896"}
    styles = {"PID_LowNoise": "-",  "PD_LowNoise":  "--",
              "PID_MedNoise": "-",  "PD_MedNoise":  "--"}

    # ── Row 1: trajectories (2 per noise level) ──────────────────────────
    for ni, (noise_label, noise_val) in enumerate([("Low Noise", 0.005),
                                                    ("Medium Noise", 0.02)]):
        ax = fig.add_subplot(3, 4, ni * 2 + 1)
        ax.plot(path.points[:, 0], path.points[:, 1],
                "g--", lw=1.2, zorder=1, label="Path")
        for gains, noise, label in conditions:
            if noise != noise_val:
                continue
            traj = cond_data[label][0]    # kpis list — get traj from rep data
            # rep_times index 1
            # We stored representative traj in run_condition as rep_times
            # (index 1 of the returned tuple)
            _, rt, rel, *_ = cond_data[label]
        ax.set_title(f"Trajectories — {noise_label}", fontsize=9, fontweight="bold")
        ax.set_aspect("equal"); ax.grid(True, lw=0.4)

    # ── Row 1 col 3-4: e_lat time series ─────────────────────────────────
    for ni, (noise_label, noise_val, col_offset) in enumerate([
        ("Low Noise", 0.005, 3), ("Med Noise", 0.02, 4)
    ]):
        ax = fig.add_subplot(3, 4, col_offset)
        for gains, noise, label in conditions:
            if noise != noise_val:
                continue
            _, times, e_lats, *_ = cond_data[label]
            ax.plot(times, e_lats, color=colors[label],
                    ls=styles[label], lw=1.2,
                    label=f"{'PID' if gains.Ki_lat > 0 else 'PD'}")
        ax.axhline( SETTLING_TOL, color="gray", ls=":", lw=0.8)
        ax.axhline(-SETTLING_TOL, color="gray", ls=":", lw=0.8)
        ax.axhline(0, color="k", lw=0.5)
        ax.set_title(f"e_lat — {noise_label}", fontsize=9, fontweight="bold")
        ax.set_xlabel("t (s)"); ax.set_ylabel("e_lat (m)")
        ax.legend(fontsize=8); ax.grid(True, lw=0.4)

    # ── Row 2: integral traces (show why Ki matters) ─────────────────────
    ax_il = fig.add_subplot(3, 2, 3)
    ax_ih = fig.add_subplot(3, 2, 4)
    for gains, noise, label in conditions:
        if noise != NOISE_LOW:
            continue
        _, _, _, ct, il, ih = cond_data[label]
        tag = "PID" if gains.Ki_lat > 0 else "PD (Ki=0)"
        ax_il.plot(ct, il, color=colors[label], ls=styles[label],
                   lw=1.3, label=tag)
        ax_ih.plot(ct, ih, color=colors[label], ls=styles[label],
                   lw=1.3, label=tag)

    ax_il.set_title("Lateral Integrator I_lat  (Low Noise)", fontsize=9, fontweight="bold")
    ax_il.set_xlabel("t (s)"); ax_il.set_ylabel("I_lat")
    ax_il.legend(fontsize=8); ax_il.grid(True, lw=0.4)
    ax_il.axhline(0, color="k", lw=0.5)

    ax_ih.set_title("Heading Integrator I_head  (Low Noise)", fontsize=9, fontweight="bold")
    ax_ih.set_xlabel("t (s)"); ax_ih.set_ylabel("I_head")
    ax_ih.legend(fontsize=8); ax_ih.grid(True, lw=0.4)
    ax_ih.axhline(0, color="k", lw=0.5)

    # ── Row 3: KPI bar chart — PD vs PID, low vs medium noise ────────────
    kpi_configs = [
        ("Overshoot (m)",     lambda k: k.overshoot_m),
        ("Settling Time (s)", lambda k: k.settling_time_s),
        ("SS Error (m)",      lambda k: k.steady_state_error_m),
    ]
    cond_labels = [l for _, _, l in conditions]
    bar_colors  = [colors[l] for l in cond_labels]
    x_pos       = np.arange(len(cond_labels))

    for bi, (title, getter) in enumerate(kpi_configs):
        ax = fig.add_subplot(3, 3, 7 + bi)
        means = [np.mean([getter(k) for k in cond_data[l][0]]) for l in cond_labels]
        stds  = [np.std( [getter(k) for k in cond_data[l][0]]) for l in cond_labels]
        ax.bar(x_pos, means, yerr=stds, color=bar_colors,
               capsize=4, edgecolor="k", lw=0.5)
        ax.set_xticks(x_pos)
        ax.set_xticklabels([l.replace("_", "\n") for l in cond_labels], fontsize=7)
        ax.set_title(title, fontsize=9, fontweight="bold")
        ax.grid(axis="y", lw=0.4)

    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, EXP_NAME, "e4_pd_vs_pid.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    plt.savefig(out, dpi=130)
    print(f"\n  Figure saved → {out}")
    plt.show()


if __name__ == "__main__":
    main()
