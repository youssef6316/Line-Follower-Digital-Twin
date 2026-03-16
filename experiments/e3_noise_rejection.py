"""
e3_noise_rejection.py — Experiment 3: Noise and Disturbance Rejection
======================================================================
Spec requirement:
  Add different noise levels and disturbances; analyze robustness
  and success rate.

What this script does:
  - Fixes the best gain set (G3_baseline) and the straight path
  - Sweeps 5 noise levels × N_SPAWNS random starts
  - Also injects an impulse lateral disturbance mid-episode
  - Reports success rate (% settled), mean KPIs, and degradation curves

Noise model:
  sigma_pos   — Gaussian std-dev on (x, y) published by plant (m)
  sigma_theta — 0.5 × sigma_pos on heading (rad)

Disturbance model:
  At t = DISTURB_TIME, a one-step lateral impulse is applied by
  temporarily offsetting the robot's true position perpendicular to
  the path.  This simulates a bump or slip event.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client1_plant"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client2_controller"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "client3_visualizer"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from client1_plant.robot       import DifferentialDriveRobot
from client1_plant.path        import make_straight, make_random_spawn
from client2_controller.errors     import compute_errors
from client2_controller.pid       import PIDController, GAINS_PID
from client3_visualizer.kpi        import compute_kpis, extract_lateral_series
from client3_visualizer.logger      import ExperimentLogger
from vsi_bridge import SharedBus, VSIBridge

# ── Parameters ─────────────────────────────────────────────────────────────
N_SPAWNS       = 5
PATH_LENGTH    = 15.0
PLANT_DT       = 0.01
CONTROL_DT     = 0.05
CONTROL_RATIO  = int(CONTROL_DT / PLANT_DT)
MAX_STEPS      = 4000
SETTLING_TOL   = 0.05
DISTURB_TIME   = 5.0     # seconds into episode when impulse hits
DISTURB_MAG    = 0.4     # lateral impulse magnitude (m)

NOISE_LEVELS   = [0.0, 0.005, 0.02, 0.05, 0.10]   # sigma_pos (m)
NOISE_LABELS   = ["No noise", "Low (5mm)", "Medium (20mm)",
                  "High (50mm)", "Extreme (100mm)"]

RNG_SEED       = 2
EXP_NAME       = "e3_noise_rejection"
RESULTS_DIR    = os.path.join(os.path.dirname(__file__), "..", "results")


def run_episode(path, gains, x0, y0, theta0,
                noise_pos, with_disturbance=False):
    """
    Run one episode with specified noise level.
    Optionally inject a lateral impulse at DISTURB_TIME.
    """
    robot = DifferentialDriveRobot(
        x0=x0, y0=y0, theta0=theta0, dt=PLANT_DT,
        noise_std_pos=noise_pos, noise_std_theta=noise_pos * 0.5,
    )
    pid  = PIDController(gains, dt=CONTROL_DT)
    traj = []
    v_cmd, omega_cmd = gains.v_ref, 0.0
    disturbed = False

    for step in range(MAX_STEPS):
        # Inject disturbance once at DISTURB_TIME
        if with_disturbance and not disturbed and robot.t >= DISTURB_TIME:
            robot.apply_disturbance(DISTURB_MAG)
            disturbed = True

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
    path       = make_straight(PATH_LENGTH)
    exp_logger = ExperimentLogger(EXP_NAME, results_dir=RESULTS_DIR)
    gains      = GAINS_PID   # G3_baseline

    print(f"\n{'='*65}")
    print(f"  E3 — Noise & Disturbance Rejection  |  {gains.name}")
    print(f"  Noise levels: {NOISE_LEVELS}")
    print(f"  Disturbance:  {DISTURB_MAG} m lateral impulse at t={DISTURB_TIME} s")
    print(f"{'='*65}")

    # Results storage
    results = {lvl: {"kpis": [], "times": [], "e_lats": [], "success": 0}
               for lvl in NOISE_LEVELS}
    disturb_results = {"kpis": [], "times": [], "e_lats": []}

    for noise, label in zip(NOISE_LEVELS, NOISE_LABELS):
        print(f"\n  [{label}]  sigma={noise} m")
        spawn_rng = np.random.default_rng(RNG_SEED)

        for sp in range(N_SPAWNS):
            x0, y0, t0 = make_random_spawn(path, lateral_range=1.0,
                                           heading_noise_deg=12, rng=spawn_rng)
            traj, times, e_lats = run_episode(path, gains, x0, y0, t0,
                                              noise_pos=noise)
            kpi = compute_kpis(times, e_lats, settling_tol=SETTLING_TOL,
                               gains_name=f"noise_{noise:.3f}_sp{sp}")
            exp_logger.add(kpi)

            results[noise]["kpis"].append(kpi)
            results[noise]["times"].append(times)
            results[noise]["e_lats"].append(e_lats)
            if kpi.settled:
                results[noise]["success"] += 1

            print(f"    spawn {sp}: OS={kpi.overshoot_m:.3f}m  "
                  f"ts={kpi.settling_time_s:.2f}s  "
                  f"SS={kpi.steady_state_error_m:.4f}m  "
                  f"{'✓' if kpi.settled else '✗'}")

    # Disturbance runs (medium noise + impulse)
    print(f"\n  [Disturbance — medium noise + {DISTURB_MAG}m impulse at t={DISTURB_TIME}s]")
    spawn_rng = np.random.default_rng(RNG_SEED)
    for sp in range(N_SPAWNS):
        x0, y0, t0 = make_random_spawn(path, lateral_range=1.0,
                                       heading_noise_deg=12, rng=spawn_rng)
        traj, times, e_lats = run_episode(path, gains, x0, y0, t0,
                                          noise_pos=0.02, with_disturbance=True)
        kpi = compute_kpis(times, e_lats, settling_tol=SETTLING_TOL,
                           gains_name=f"disturb_sp{sp}")
        exp_logger.add(kpi)
        disturb_results["kpis"].append(kpi)
        disturb_results["times"].append(times)
        disturb_results["e_lats"].append(e_lats)
        print(f"    spawn {sp}: OS={kpi.overshoot_m:.3f}m  "
              f"ts={kpi.settling_time_s:.2f}s  "
              f"SS={kpi.steady_state_error_m:.4f}m  "
              f"{'✓' if kpi.settled else '✗'}")

    exp_logger.finalize()
    _plot_results(results, disturb_results)


def _plot_results(results, disturb_results):
    colors  = cm.plasma(np.linspace(0.1, 0.85, len(NOISE_LEVELS)))
    n_lvls  = len(NOISE_LEVELS)

    fig = plt.figure(figsize=(18, 11))
    fig.suptitle("E3 — Noise & Disturbance Rejection", fontsize=14, fontweight="bold")

    # ── Row 1: e_lat time series per noise level (first spawn) ──────────
    for col, (noise, label, color) in enumerate(
            zip(NOISE_LEVELS, NOISE_LABELS, colors)):
        ax = fig.add_subplot(3, n_lvls, col + 1)
        ts  = results[noise]["times"][0]
        els = results[noise]["e_lats"][0]
        ax.plot(ts, els, color=color, lw=1.2)
        ax.axhline( SETTLING_TOL, color="gray", ls=":", lw=0.8)
        ax.axhline(-SETTLING_TOL, color="gray", ls=":", lw=0.8)
        ax.axhline(0, color="k", lw=0.5)
        ax.set_title(label, fontsize=8, fontweight="bold")
        ax.set_xlabel("t (s)", fontsize=7); ax.set_ylabel("e_lat (m)", fontsize=7)
        ax.tick_params(labelsize=6); ax.grid(True, lw=0.4)

    # ── Row 2: disturbance response (all spawns overlaid) ────────────────
    ax_d = fig.add_subplot(3, 2, 3)
    for i, (ts, els) in enumerate(zip(disturb_results["times"],
                                       disturb_results["e_lats"])):
        ax_d.plot(ts, els, lw=1.0, alpha=0.7, label=f"spawn {i}")
    ax_d.axvline(DISTURB_TIME, color="r", ls="--", lw=1.2,
                 label=f"Disturbance ({DISTURB_MAG}m)")
    ax_d.axhline( SETTLING_TOL, color="gray", ls=":", lw=0.8)
    ax_d.axhline(-SETTLING_TOL, color="gray", ls=":", lw=0.8)
    ax_d.set_title("Impulse Disturbance Response", fontsize=10, fontweight="bold")
    ax_d.set_xlabel("t (s)"); ax_d.set_ylabel("e_lat (m)")
    ax_d.legend(fontsize=7); ax_d.grid(True, lw=0.4)

    # ── Row 2: KPIs vs noise level ────────────────────────────────────────
    noise_arr   = np.array(NOISE_LEVELS)
    os_means    = [np.mean([k.overshoot_m          for k in results[n]["kpis"]]) for n in NOISE_LEVELS]
    ts_means    = [np.mean([k.settling_time_s       for k in results[n]["kpis"]]) for n in NOISE_LEVELS]
    ss_means    = [np.mean([k.steady_state_error_m  for k in results[n]["kpis"]]) for n in NOISE_LEVELS]
    success_pct = [100 * results[n]["success"] / N_SPAWNS                         for n in NOISE_LEVELS]

    for bi, (vals, title, color_line) in enumerate([
        (os_means,   "Mean Overshoot (m)",      "steelblue"),
        (ts_means,   "Mean Settling Time (s)",  "tomato"),
        (ss_means,   "Mean SS Error (m)",       "seagreen"),
    ]):
        ax = fig.add_subplot(3, 3, 7 + bi)
        ax.plot(NOISE_LEVELS, vals, "o-", color=color_line, lw=1.8, ms=6)
        ax.set_xlabel("Noise σ (m)", fontsize=8)
        ax.set_title(title, fontsize=9, fontweight="bold")
        ax.grid(True, lw=0.4)

    # Success rate on ax overlaid (secondary)
    ax_sr = fig.add_subplot(3, 2, 4)
    ax_sr.bar(NOISE_LABELS, success_pct, color=colors, edgecolor="k", lw=0.5)
    ax_sr.set_ylabel("Success rate (%)")
    ax_sr.set_title("Settling Success Rate vs Noise", fontsize=10, fontweight="bold")
    ax_sr.set_ylim(0, 110)
    for i, v in enumerate(success_pct):
        ax_sr.text(i, v + 2, f"{v:.0f}%", ha="center", fontsize=8)
    ax_sr.tick_params(axis="x", labelsize=7); ax_sr.grid(axis="y", lw=0.4)

    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, EXP_NAME, "e3_noise_rejection.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    plt.savefig(out, dpi=130)
    print(f"\n  Figure saved → {out}")
    plt.show()


if __name__ == "__main__":
    main()
