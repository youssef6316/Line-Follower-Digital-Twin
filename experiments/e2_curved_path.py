"""
e2_curved_path.py — Experiment 2: Curved Path Robustness
=========================================================
Spec requirement:
  Repeat best controllers on a curved path; compare metrics and
  discuss curvature effects.

Strategy:
  - Take the top-2 performers from E1 (by mean settling time on straight)
  - Run them on the S-curve path with the same N_SPAWNS random starts
  - Also run all 5 for a full cross-comparison
  - Compare straight-path KPIs vs curved-path KPIs side by side
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client1_plant"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client2_controller"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client3_visualizer"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from client1_plant.robot     import DifferentialDriveRobot
from client1_plant.path         import make_straight, make_s_curve, make_random_spawn
from client2_controller.errors     import compute_errors
from client2_controller.pid        import PIDController, GAIN_SETS
from client3_visualizer.kpi       import compute_kpis, extract_lateral_series
from client3_visualizer.logger     import ExperimentLogger
from vsi_bridge import SharedBus, VSIBridge

# ── Parameters ─────────────────────────────────────────────────────────────
N_SPAWNS      = 5
PLANT_DT      = 0.01
CONTROL_DT    = 0.05
CONTROL_RATIO = int(CONTROL_DT / PLANT_DT)
MAX_STEPS     = 6000       # longer — curved path is ~23 m
NOISE_POS     = 0.01
NOISE_THETA   = 0.005
SETTLING_TOL  = 0.08       # slightly relaxed — curves are harder
RNG_SEED      = 1
EXP_NAME      = "e2_curved_path"
RESULTS_DIR   = os.path.join(os.path.dirname(__file__), "..", "results")


def run_episode(path, gains, x0, y0, theta0):
    robot = DifferentialDriveRobot(
        x0=x0, y0=y0, theta0=theta0, dt=PLANT_DT,
        noise_std_pos=NOISE_POS, noise_std_theta=NOISE_THETA,
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
    return traj, times, e_lats


def main():
    straight   = make_straight(15.0)
    scurve     = make_s_curve()
    exp_logger = ExperimentLogger(EXP_NAME, results_dir=RESULTS_DIR)

    print(f"\n{'='*65}")
    print(f"  E2 — Curved Path Robustness  |  {len(GAIN_SETS)} gains × {N_SPAWNS} spawns")
    print(f"  S-curve length: {scurve.total_length:.2f} m")
    print(f"{'='*65}")

    straight_kpis = {g.name: [] for g in GAIN_SETS}
    curved_kpis   = {g.name: [] for g in GAIN_SETS}
    curved_trajs  = {g.name: [] for g in GAIN_SETS}
    curved_elats  = {g.name: [] for g in GAIN_SETS}
    curved_times  = {g.name: [] for g in GAIN_SETS}

    for gains in GAIN_SETS:
        print(f"\n  [{gains.name}]")
        spawn_rng = np.random.default_rng(RNG_SEED)

        for sp in range(N_SPAWNS):
            # Straight run (for comparison)
            x0s, y0s, t0s = make_random_spawn(straight, lateral_range=1.2,
                                              heading_noise_deg=15, rng=np.random.default_rng(sp))
            _, ts, els = run_episode(straight, gains, x0s, y0s, t0s)
            kpi_s = compute_kpis(ts, els, settling_tol=SETTLING_TOL,
                                 gains_name=gains.name)
            straight_kpis[gains.name].append(kpi_s)

            # Curved run
            x0c, y0c, t0c = make_random_spawn(scurve, lateral_range=0.6,
                                              heading_noise_deg=10, rng=spawn_rng)
            traj, tc, elc = run_episode(scurve, gains, x0c, y0c, t0c)
            kpi_c = compute_kpis(tc, elc, settling_tol=SETTLING_TOL,
                                 gains_name=f"{gains.name}_curved_sp{sp}")
            exp_logger.add(kpi_c)

            curved_kpis[gains.name].append(kpi_c)
            curved_trajs[gains.name].append(traj)
            curved_elats[gains.name].append(elc)
            curved_times[gains.name].append(tc)

            print(f"    spawn {sp}  curved: OS={kpi_c.overshoot_m:.3f}m  "
                  f"ts={kpi_c.settling_time_s:.2f}s  "
                  f"SS={kpi_c.steady_state_error_m:.4f}m  "
                  f"{'✓' if kpi_c.settled else '✗'}")

    exp_logger.finalize()
    _plot_results(scurve, straight_kpis, curved_kpis, curved_trajs,
                  curved_elats, curved_times)


def _plot_results(scurve, straight_kpis, curved_kpis, trajs, elats, times):
    colors = cm.tab10(np.linspace(0, 0.6, len(GAIN_SETS)))
    names  = [g.name for g in GAIN_SETS]

    fig = plt.figure(figsize=(18, 12))
    fig.suptitle("E2 — Curved Path Robustness", fontsize=14, fontweight="bold")

    # ── Row 1: trajectory per gain set on S-curve ────────────────────────
    for col, (gains, color) in enumerate(zip(GAIN_SETS, colors)):
        ax = fig.add_subplot(3, 5, col + 1)
        ax.plot(scurve.points[:, 0], scurve.points[:, 1],
                "g--", lw=1.2, zorder=1)
        for traj in trajs[gains.name]:
            xs = [r["x_true"] for r in traj]
            ys = [r["y_true"] for r in traj]
            ax.plot(xs, ys, color=color, lw=0.9, alpha=0.7)
        ax.set_title(gains.name, fontsize=8, fontweight="bold")
        ax.set_aspect("equal"); ax.grid(True, lw=0.4)
        ax.set_xlabel("x (m)", fontsize=7); ax.set_ylabel("y (m)", fontsize=7)
        ax.tick_params(labelsize=6)

    # ── Row 2: e_lat on curved path (first spawn) ────────────────────────
    for col, (gains, color) in enumerate(zip(GAIN_SETS, colors)):
        ax = fig.add_subplot(3, 5, col + 6)
        ax.plot(times[gains.name][0], elats[gains.name][0],
                color=color, lw=1.2)
        ax.axhline( 0.08, color="gray", ls=":", lw=0.8)
        ax.axhline(-0.08, color="gray", ls=":", lw=0.8)
        ax.axhline(0,     color="k",    lw=0.5)
        ax.set_title(f"{gains.name}", fontsize=8)
        ax.set_xlabel("t (s)", fontsize=7); ax.set_ylabel("e_lat (m)", fontsize=7)
        ax.tick_params(labelsize=6); ax.grid(True, lw=0.4)

    # ── Row 3: straight vs curved KPI comparison ────────────────────────
    kpi_keys  = ["overshoot_m", "settling_time_s", "steady_state_error_m"]
    kpi_titles= ["Overshoot (m)", "Settling Time (s)", "SS Error (m)"]
    getters   = [lambda k: k.overshoot_m,
                 lambda k: k.settling_time_s,
                 lambda k: k.steady_state_error_m]

    x_pos = np.arange(len(GAIN_SETS))
    width = 0.35

    for bi, (title, getter) in enumerate(zip(kpi_titles, getters)):
        ax = fig.add_subplot(3, 3, 7 + bi)
        s_means = [np.mean([getter(k) for k in straight_kpis[g.name]]) for g in GAIN_SETS]
        c_means = [np.mean([getter(k) for k in curved_kpis[g.name]])   for g in GAIN_SETS]
        ax.bar(x_pos - width/2, s_means, width, label="Straight",
               color="steelblue", edgecolor="k", lw=0.5)
        ax.bar(x_pos + width/2, c_means, width, label="Curved",
               color="tomato",    edgecolor="k", lw=0.5)
        ax.set_xticks(x_pos)
        ax.set_xticklabels([g.name.replace("_", "\n") for g in GAIN_SETS], fontsize=7)
        ax.set_title(title, fontsize=9, fontweight="bold")
        ax.legend(fontsize=7); ax.grid(axis="y", lw=0.4)

    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, EXP_NAME, "e2_curved_path.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    plt.savefig(out, dpi=130)
    print(f"\n  Figure saved → {out}")
    plt.show()


if __name__ == "__main__":
    main()
