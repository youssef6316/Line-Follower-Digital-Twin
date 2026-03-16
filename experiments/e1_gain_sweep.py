"""
e1_gain_sweep.py — Experiment 1: PID Gain Sweep
================================================
Spec requirement:
  Choose 3-5 gain sets; for each, run multiple random spawns on a
  straight path and report overshoot, settling time, and steady-state error.

What this script does:
  - Runs all 5 gain sets (G1–G5) × N_SPAWNS random starting poses
  - Computes KPIs for every run
  - Writes per-episode CSVs + one summary CSV
  - Produces a 3-panel comparison figure:
      Panel 1 — all trajectories overlaid per gain set
      Panel 2 — KPI bar charts (overshoot, settling time, SS error)
      Panel 3 — lateral error time series (one representative run each)
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client1_plant"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client2_controller"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client3_visualizer"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from client1_plant.robot      import DifferentialDriveRobot
from client1_plant.path       import make_s_curve, make_random_spawn, make_straight
from client2_controller.errors     import compute_errors
from client2_controller.pid          import PIDController, GAIN_SETS
from client3_visualizer.kpi        import compute_kpis, extract_lateral_series
from client3_visualizer.logger     import ExperimentLogger
from vsi_bridge   import SharedBus, VSIBridge

# ── Parameters ─────────────────────────────────────────────────────────────
N_SPAWNS      = 5
PATH_LENGTH   = 15.0
PLANT_DT      = 0.01
CONTROL_DT    = 0.05
CONTROL_RATIO = int(CONTROL_DT / PLANT_DT)
MAX_STEPS     = 4000
NOISE_POS     = 0.01
NOISE_THETA   = 0.005
SETTLING_TOL  = 0.05
RNG_SEED      = 0
EXP_NAME      = "e1_gain_sweep"
RESULTS_DIR   = os.path.join(os.path.dirname(__file__), "..", "results")


def run_episode(path, gains, x0, y0, theta0):
    """Single closed-loop episode. Returns (trajectory_list, times, e_lats)."""
    robot = DifferentialDriveRobot(
        x0=x0, y0=y0, theta0=theta0, dt=PLANT_DT,
        noise_std_pos=NOISE_POS, noise_std_theta=NOISE_THETA,
    )
    pid   = PIDController(gains, dt=CONTROL_DT)
    traj  = []
    v_cmd, omega_cmd = gains.v_ref, 0.0

    for step in range(MAX_STEPS):
        pose = robot.step(v_cmd, omega_cmd)
        traj.append(pose)

        if step % CONTROL_RATIO == 0:
            nr    = path.get_nearest(pose["x"], pose["y"])
            errs  = compute_errors(pose["x"], pose["y"], pose["theta"], nr)
            v_cmd, omega_cmd = pid.step(errs.e_lat, errs.e_head)

        if path.is_finished(pose["x_true"], pose["y_true"]):
            break

    times, e_lats = extract_lateral_series(traj, path)
    return traj, times, e_lats


def main():
    path       = make_straight(PATH_LENGTH)
    rng        = np.random.default_rng(RNG_SEED)
    exp_logger = ExperimentLogger(EXP_NAME, results_dir=RESULTS_DIR)

    # Storage for plotting
    all_results = {g.name: {"trajs": [], "times": [], "e_lats": [], "kpis": []}
                   for g in GAIN_SETS}

    print(f"\n{'='*65}")
    print(f"  E1 — PID Gain Sweep  |  {len(GAIN_SETS)} gain sets × {N_SPAWNS} spawns")
    print(f"{'='*65}")

    for gains in GAIN_SETS:
        print(f"\n  [{gains.name}]")
        spawn_rng = np.random.default_rng(RNG_SEED)   # same spawns for all gains

        for sp in range(N_SPAWNS):
            x0, y0, t0 = make_random_spawn(path, lateral_range=1.2,
                                           heading_noise_deg=15, rng=spawn_rng)
            traj, times, e_lats = run_episode(path, gains, x0, y0, t0)

            kpi = compute_kpis(times, e_lats,
                               settling_tol=SETTLING_TOL,
                               gains_name=f"{gains.name}_sp{sp}")
            exp_logger.add(kpi)

            all_results[gains.name]["trajs"].append(traj)
            all_results[gains.name]["times"].append(times)
            all_results[gains.name]["e_lats"].append(e_lats)
            all_results[gains.name]["kpis"].append(kpi)

            print(f"    spawn {sp}: OS={kpi.overshoot_m:.3f}m  "
                  f"ts={kpi.settling_time_s:.2f}s  "
                  f"SS={kpi.steady_state_error_m:.4f}m  "
                  f"{'✓' if kpi.settled else '✗'}")

    exp_logger.finalize()
    _plot_results(path, all_results)


def _plot_results(path, all_results):
    colors = cm.tab10(np.linspace(0, 0.6, len(GAIN_SETS)))
    names  = [g.name for g in GAIN_SETS]

    fig = plt.figure(figsize=(18, 13))
    fig.suptitle("E1 — PID Gain Sweep Results", fontsize=14, fontweight="bold")

    # ── Row 1: trajectories (one subplot per gain set) ──────────────────
    for col, (gains, color) in enumerate(zip(GAIN_SETS, colors)):
        ax = fig.add_subplot(3, 5, col + 1)
        ax.plot(path.points[:, 0], path.points[:, 1],
                "g--", lw=1.2, zorder=1, label="Path")
        res = all_results[gains.name]
        for traj in res["trajs"]:
            xs = [r["x_true"] for r in traj]
            ys = [r["y_true"] for r in traj]
            ax.plot(xs, ys, color=color, lw=1.0, alpha=0.7)
        ax.set_title(gains.name, fontsize=8, fontweight="bold")
        ax.set_aspect("equal"); ax.grid(True, lw=0.4)
        ax.set_xlabel("x (m)", fontsize=7); ax.set_ylabel("y (m)", fontsize=7)
        ax.tick_params(labelsize=6)

    # ── Row 2: lateral error time series (first spawn each) ─────────────
    for col, (gains, color) in enumerate(zip(GAIN_SETS, colors)):
        ax = fig.add_subplot(3, 5, col + 6)
        res   = all_results[gains.name]
        times = res["times"][0]
        elats = res["e_lats"][0]
        ax.plot(times, elats, color=color, lw=1.2)
        ax.axhline( 0.05, color="gray", ls=":", lw=0.8)
        ax.axhline(-0.05, color="gray", ls=":", lw=0.8)
        ax.axhline(0,     color="k",    lw=0.5)
        ax.set_title(f"{gains.name} — e_lat", fontsize=8)
        ax.set_xlabel("t (s)", fontsize=7); ax.set_ylabel("e_lat (m)", fontsize=7)
        ax.tick_params(labelsize=6); ax.grid(True, lw=0.4)

    # ── Row 3: KPI bar charts ────────────────────────────────────────────
    kpi_labels   = ["Overshoot (m)", "Settling Time (s)", "SS Error (m)"]
    kpi_getters  = [
        lambda k: k.overshoot_m,
        lambda k: k.settling_time_s,
        lambda k: k.steady_state_error_m,
    ]

    for bi, (label, getter) in enumerate(zip(kpi_labels, kpi_getters)):
        ax = fig.add_subplot(3, 3, 7 + bi)
        means = []
        stds  = []
        for gains in GAIN_SETS:
            vals = [getter(k) for k in all_results[gains.name]["kpis"]]
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        x_pos = np.arange(len(GAIN_SETS))
        bars  = ax.bar(x_pos, means, yerr=stds, color=colors,
                       capsize=4, edgecolor="k", linewidth=0.5)
        ax.set_xticks(x_pos)
        ax.set_xticklabels([g.name.replace("_", "\n") for g in GAIN_SETS],
                           fontsize=7)
        ax.set_title(label, fontsize=9, fontweight="bold")
        ax.set_ylabel(label, fontsize=8)
        ax.grid(axis="y", lw=0.4)

    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, EXP_NAME, "e1_gain_sweep.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    plt.savefig(out, dpi=130)
    print(f"\n  Figure saved → {out}")
    plt.show()


if __name__ == "__main__":
    main()
