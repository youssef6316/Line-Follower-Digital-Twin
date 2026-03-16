"""
run_standalone.py — Full Closed-Loop Simulation (No VSI Required)
==================================================================
Wires Client 1 (Plant), Client 2 (Controller), and Client 3 (Visualizer)
together using the SharedBus from vsi_bridge.py.

When VSI is available, each client block below becomes a separate process
launched by VsiSim — the logic inside is identical.

Usage
-----
  python run_standalone.py                    # baseline run, straight path
  python run_standalone.py --path scurve      # S-curve path
  python run_standalone.py --gains G4_aggressive
  python run_standalone.py --no-plot          # headless (for batch runs)
"""

import sys, os, argparse
# sys.path.insert(0, "client1_plant")
# sys.path.insert(0, "client2_controller")
# sys.path.insert(0, "client3_visualizer")

import numpy as np

from client1_plant.robot                   import DifferentialDriveRobot
from client1_plant.path                    import make_straight, make_s_curve, make_random_spawn
from client2_controller.errors             import compute_errors
from client2_controller.pid                import PIDController, GAIN_SETS
from client3_visualizer.kpi                import extract_lateral_series, compute_kpis
from client3_visualizer.logger             import EpisodeLogger, ExperimentLogger
from client3_visualizer.visualizer_client  import VisualizerClient
from vsi_bridge                            import SharedBus, VSIBridge


# ── CLI args ───────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--path",   default="straight",
                    choices=["straight", "scurve"])
parser.add_argument("--gains",  default="G3_baseline",
                    choices=[g.name for g in GAIN_SETS])
parser.add_argument("--noise",  type=float, default=0.01,
                    help="Position noise std-dev (m)")
parser.add_argument("--seed",   type=int,   default=42)
parser.add_argument("--no-plot", action="store_true")
args = parser.parse_args()

# ── Configuration ──────────────────────────────────────────────────────────
PLANT_DT      = 0.01    # 100 Hz
CONTROL_DT    = 0.05    # 20  Hz
CONTROL_RATIO = int(CONTROL_DT / PLANT_DT)
VIZ_RATIO     = 5       # visualizer refresh every N plant steps
MAX_STEPS     = 4000

gains = next(g for g in GAIN_SETS if g.name == args.gains)
path  = make_straight(15.0) if args.path == "straight" else make_s_curve()
rng   = np.random.default_rng(args.seed)

print(f"\n{'='*60}")
print(f"  Line-Following Robot — Standalone Simulation")
print(f"  Path:  {args.path}  |  Gains: {gains.name}")
print(f"  Noise: pos={args.noise} m  |  Seed: {args.seed}")
print(f"{'='*60}\n")

# ── VSI Bridge (standalone mode) ───────────────────────────────────────────
bus            = SharedBus()
plant_bridge   = VSIBridge(mode="standalone", bus=bus, client_name="plant")
ctrl_bridge    = VSIBridge(mode="standalone", bus=bus, client_name="controller")
viz_bridge     = VSIBridge(mode="standalone", bus=bus, client_name="visualizer")

# ── Client 1: Plant ────────────────────────────────────────────────────────
x0, y0, t0 = make_random_spawn(path, lateral_range=1.2,
                                heading_noise_deg=15, rng=rng)
robot = DifferentialDriveRobot(
    x0=x0, y0=y0, theta0=t0, dt=PLANT_DT,
    noise_std_pos=args.noise, noise_std_theta=args.noise * 0.5,
)
print(f"  Spawn: x0={x0:.3f} m, y0={y0:.3f} m, θ0={np.rad2deg(t0):.1f}°\n")

# ── Client 2: Controller ───────────────────────────────────────────────────
pid = PIDController(gains, dt=CONTROL_DT)

# ── Client 3: Visualizer ───────────────────────────────────────────────────
exp_logger = ExperimentLogger("standalone_run", results_dir="results")
viz = VisualizerClient(
    path            = path,
    gains_name      = gains.name,
    exp_logger      = exp_logger,
    spawn_idx       = 0,
    update_interval = VIZ_RATIO,
    live_plot       = not args.no_plot,
    results_dir     = "results",
    experiment_name = "standalone_run",
)

# ── Main simulation loop ───────────────────────────────────────────────────
v_cmd, omega_cmd = gains.v_ref, 0.0
e_lat, e_head    = 0.0, 0.0

for step in range(MAX_STEPS):

    # ── CLIENT 1: advance plant, publish state ─────────────────────────
    pose = robot.step(v_cmd, omega_cmd)

    nr       = path.get_nearest(pose["x"], pose["y"])
    progress = nr.progress

    plant_bridge.publish_batch({
        "x":          pose["x"],
        "y":          pose["y"],
        "theta":      pose["theta"],
        "x_true":     pose["x_true"],
        "y_true":     pose["y_true"],
        "theta_true": pose["theta_true"],
        "t":          pose["t"],
        "progress":   progress,
    })

    # ── CLIENT 2: controller update (lower rate) ───────────────────────
    if step % CONTROL_RATIO == 0:
        state   = ctrl_bridge.read_plant_state()
        errors  = compute_errors(state["x"], state["y"], state["theta"], nr)
        e_lat   = errors.e_lat
        e_head  = errors.e_head
        v_cmd, omega_cmd = pid.step(e_lat, e_head)

        ctrl_bridge.publish_batch({
            "v":      v_cmd,
            "omega":  omega_cmd,
            "e_lat":  e_lat,
            "e_head": e_head,
        })

    # ── CLIENT 3: visualizer update ────────────────────────────────────
    viz.update(
        t          = pose["t"],
        x          = pose["x"],          y          = pose["y"],
        theta      = pose["theta"],
        x_true     = pose["x_true"],     y_true     = pose["y_true"],
        theta_true = pose["theta_true"],
        v          = v_cmd,              omega      = omega_cmd,
        e_lat      = e_lat,              e_head     = e_head,
        path_progress = progress,
    )

    # ── Termination ────────────────────────────────────────────────────
    if path.is_finished(pose["x_true"], pose["y_true"]):
        print(f"  ✓ Path finished at t={pose['t']:.2f} s  (step {step})")
        break
else:
    print(f"  ⚠ Max steps reached ({MAX_STEPS})")

# ── Finalise ───────────────────────────────────────────────────────────────
kpi = viz.finalize()
exp_logger.finalize()

print(f"\n  KPI Summary:")
print(f"    Overshoot      : {kpi.overshoot_m:.4f} m")
print(f"    Settling time  : {kpi.settling_time_s:.3f} s")
print(f"    Steady-state Δ : {kpi.steady_state_error_m:.4f} m")
print(f"    Settled        : {'YES' if kpi.settled else 'NO'}")
