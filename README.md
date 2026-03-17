# Line-Follower Digital Twin

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Repo](https://img.shields.io/badge/repo-youssef6316/Line--Follower--Digital--Twin-blue)](https://github.com/youssef6316/Line-Follower-Digital-Twin)

A compact, ready-to-run digital twin for a line-following differential-drive robot. The repository provides:
- a standalone closed-loop simulation (no external simulator required),
- modular client separation (Plant / Controller / Visualizer),
- KPI computation and experiment logging,
- a VSI abstraction layer so the same client code can be adapted to a real VSI backplane later.

This README is tailored to the actual code in the repository (see `run_standalone.py`, `vsi_bridge.py`, and the `client*_` packages).

Table of contents
- Quickstart
- Run examples (standalone)
- CLI options & common runs
- Project layout and important files
- Dependencies
- Results, logging & batch runs
- VSI mode (notes)
- Development & contributing
- License & contact

Quickstart (run a demo)
-----------------------
1. Clone:
```bash
git clone https://github.com/youssef6316/Line-Follower-Digital-Twin.git
cd Line-Follower-Digital-Twin
```

2. Create a Python virtual environment and install dependencies:
```bash
python3 -m venv .venv
source .venv/bin/activate        # Linux/macOS
# .venv\Scripts\activate         # Windows PowerShell
pip install --upgrade pip
# If there is a requirements.txt, use it:
# pip install -r requirements.txt
# Otherwise install the minimal dependencies used by the project:
pip install numpy matplotlib
```

3. Run the standalone closed-loop simulation:
```bash
python run_standalone.py
```

When the run finishes you'll see a KPI summary printed and the run artifacts will be saved into the `results/` folder (the Visualizer/Logger code uses `results/` by default).

Run examples and CLI options
---------------------------
The main entrypoint for experiments is `run_standalone.py`. Important CLI options:

- --path {straight, scurve}
  - `straight` (default) — long straight path
  - `scurve` — S-curve track

- --gains {G1..., G2..., G3_baseline, G4_aggressive, ...}
  - The controller gain sets are defined in `client2_controller/pid.py`.
  - Default: `G3_baseline`.

- --noise <float>
  - Position noise standard deviation (m). Default: `0.01`.

- --seed <int>
  - RNG seed for reproducible spawn and stochastic behavior. Default: `42`.

- --no-plot
  - Run headless (visualizer does not show live plots). Useful for batch runs.

Examples:
```bash
# baseline straight path, baseline gains
python run_standalone.py

# S-curve with aggressive gains and higher noise
python run_standalone.py --path scurve --gains G4_aggressive --noise 0.02

# headless (CI / batch) run:
python run_standalone.py --no-plot --seed 123
```

What the run prints / saves
- Terminal: run summary and the KPI summary (Overshoot, Settling time, Steady-state error).
- Results directory: experiment logs and JSON summaries produced by `client3_visualizer/logger.py` and `ExperimentLogger`.

Project layout (key files)
--------------------------
Top-level:
- run_standalone.py
  - Standalone runner that wires the three clients (plant, controller, visualizer) over an in-process SharedBus (see `vsi_bridge.py`).
  - Prints KPI summary at the end.
- vsi_bridge.py
  - VSI abstraction layer. Default `mode="standalone"` uses `SharedBus`.
  - Contains signal schema and convenience read/publish helpers. (VSI mode placeholder raises NotImplementedError until VSI API is available.)
- vsiBuildCommands
  - CAN / signal mapping used by VSI deployments (kept for reference).

Packages:
- client1_plant/
  - path.py — path generation utilities (straight, s-curve, spawn helpers).
  - robot.py — DifferentialDriveRobot plant model and history recording.
- client2_controller/
  - pid.py — PID controller and predefined gain sets used by the demo.
  - errors.py — functions to compute lateral & heading errors relative to the path.
- client3_visualizer/
  - visualizer_client.py ��� plotting, live visualization, KPI extraction calls.
  - kpi.py — KPI computation (overshoot, settling time, steady-state error).
  - logger.py — EpisodeLogger / ExperimentLogger for saving results to disk.

Results & experiments
---------------------
- `results/` — experiment outputs (created by the visualizer/logger).
- `experiments/` — (placeholder) for scenario/parameter sets you may add.
- The ExperimentLogger writes a run-level summary (KPI JSON) and per-episode logs. Use these artifacts for batch analysis.

Dependencies
------------
The repository primarily uses standard Python 3.8+ features plus:
- numpy
- matplotlib (visualizer)
If you add a `requirements.txt`, include these two as a minimum:
```
numpy
matplotlib
```

VSI / production integration
----------------------------
- The code is intentionally modular: the only file that must change to move from standalone to a VSI-based backplane is `vsi_bridge.py`.
- In `vsi_bridge.py` the `VSIBridge` class contains a `mode="vsi"` branch that currently raises a NotImplementedError. When you have access to the Innexis VSI Python API, replace the placeholder with:
  - import the vendor API (for example `import vsiCommonPythonApi as vsi_api`)
  - wire `self._vsi` / `self._vsi_signals` so `publish`/`read` map to the VSI API.
- The `vsiBuildCommands` file documents the CAN ID to signal mapping used by the VSI backplane (useful when you implement frame packing/unpacking).

Developer notes
---------------
- Controller tuning:
  - `client2_controller/pid.py` defines `GAIN_SETS`. Try `G3_baseline` first.
- To add a new controller:
  - Implement the control logic and expose a `step(e_lat, e_head)` function that returns `(v_cmd, omega_cmd)`, or follow the PID class pattern.
- Headless / CI runs:
  - Use `--no-plot` and a fixed `--seed` to run reproducible, non-interactive experiments.
  - Collect KPIs by parsing JSON outputs in `results/`.

Known limitations
-----------------
- VSI integration is not activated by default — `vsi_bridge.VSIBridge(mode="vsi")` is currently not implemented and will raise a NotImplementedError. The standalone mode is fully functional for development, tuning and batch experiments.
- There is no provided `requirements.txt` in the repository — add one to make installs reproducible.

Contributing
------------
1. Fork and create a feature branch.
2. Add tests (where applicable) and keep changes small.
3. Ensure any new dependency is added to a `requirements.txt`.
4. Open a pull request describing the change and the rationale.

License
-------
This repository includes an MIT license file. See LICENSE.

Contact / maintainer
--------------------
Maintainer: youssef6316 — https://github.com/youssef6316

Appendix: quick troubleshooting
------------------------------
- Import errors for `client*` packages: ensure you run from repository root and your virtualenv is active.
- Missing matplotlib error when running the visualizer: install it (pip install matplotlib) or run with `--no-plot`.
- If the experiment finishes immediately or KPIs are zero: check `--path` and `--seed` to make sure the robot spawn isn't already at the end of the path.

Notes
-----
This README reflects the repository's actual entry points and package structure (see `run_standalone.py`, `vsi_bridge.py`, `client1_plant/`, `client2_controller/`, `client3_visualizer/`). It is ready to commit as `README.md`. Replace or extend the dependency list and the `requirements.txt` if you add more packages for visualization, CLI helpers, or CI.
