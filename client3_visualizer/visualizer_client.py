"""
visualizer_client.py — Client 3: Visualizer / Logger
=====================================================
In standalone mode: receives trajectory data from the shared memory
bus (vsi_bridge.SharedBus) and renders it live.

In VSI mode: reads signals from the VSI backplane via python2DtCan
gateway and renders identically — only the data source changes.

Renders two live-updating subplots:
  Left  — 2D bird's-eye trajectory (robot path vs reference)
  Right — Lateral error e_lat(t) time series with settling band

The plot updates every `update_interval` steps to avoid blocking the
simulation loop.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from collections import deque
from client3_visualizer.kpi import compute_kpis, KPIResult
from client3_visualizer.logger import EpisodeLogger, ExperimentLogger


class VisualizerClient:
    """
    Real-time visualizer and KPI logger for one experiment episode.

    Parameters
    ----------
    path : BasePath
        Reference path object (for drawing and KPI extraction).
    gains_name : str
        Label for this episode's gain set.
    exp_logger : ExperimentLogger
        Shared experiment logger that accumulates KPIs across episodes.
    spawn_idx : int
        Index of this spawn (for naming the episode CSV).
    max_history : int
        Rolling window length for the live plots.
    update_interval : int
        Plot refresh every N simulation steps.
    settling_tol : float
        Settling band half-width (m) — drawn on error plot.
    live_plot : bool
        Set False to suppress the window (useful in batch experiments).
    results_dir : str
        Root directory for CSV output.
    """

    def __init__(
        self,
        path,
        gains_name:      str   = "unknown",
        exp_logger:      ExperimentLogger | None = None,
        spawn_idx:       int   = 0,
        max_history:     int   = 5000,
        update_interval: int   = 10,
        settling_tol:    float = 0.05,
        live_plot:       bool  = True,
        results_dir:     str   = "results",
        experiment_name: str   = "episode",
    ):
        self.path            = path
        self.gains_name      = gains_name
        self.exp_logger      = exp_logger
        self.spawn_idx       = spawn_idx
        self.update_interval = update_interval
        self.settling_tol    = settling_tol
        self.live_plot       = live_plot

        # Rolling data buffers
        self._xs      = deque(maxlen=max_history)
        self._ys      = deque(maxlen=max_history)
        self._times   = deque(maxlen=max_history)
        self._e_lats  = deque(maxlen=max_history)
        self._e_heads = deque(maxlen=max_history)

        self._step_count = 0

        # Episode CSV logger
        csv_path = os.path.join(
            results_dir, experiment_name,
            f"run_{gains_name}_spawn{spawn_idx}.csv"
        )
        self._ep_logger = EpisodeLogger(csv_path)

        # Matplotlib setup
        if self.live_plot:
            plt.ion()
            self._fig, self._axes = plt.subplots(1, 2, figsize=(14, 5))
            self._fig.suptitle(
                f"Live Simulation — {gains_name}", fontsize=12, fontweight="bold"
            )
            self._init_plots()

    # ------------------------------------------------------------------
    # Initialise static plot elements
    # ------------------------------------------------------------------

    def _init_plots(self):
        ax1, ax2 = self._axes

        # Left — trajectory
        ax1.plot(
            self.path.points[:, 0], self.path.points[:, 1],
            "g--", lw=1.5, label="Reference path", zorder=1
        )
        self._traj_line, = ax1.plot([], [], "b-", lw=1.5,
                                    label="Robot trajectory", zorder=2)
        self._start_dot, = ax1.plot([], [], "ro", ms=8, label="Start", zorder=3)
        ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
        ax1.set_title("Trajectory"); ax1.legend(fontsize=8)
        ax1.set_aspect("equal"); ax1.grid(True)

        # Right — lateral error
        self._err_line,  = ax2.plot([], [], "r-",  lw=1.2, label="e_lat (m)")
        self._ehead_line,= ax2.plot([], [], "b--", lw=1.0,
                                    alpha=0.6, label="e_head (rad)")
        ax2.axhline( self.settling_tol, color="gray", ls=":", lw=1)
        ax2.axhline(-self.settling_tol, color="gray", ls=":", lw=1,
                    label=f"±{self.settling_tol} m band")
        ax2.axhline(0, color="k", lw=0.5)
        ax2.set_xlabel("t (s)"); ax2.set_ylabel("Error")
        ax2.set_title("Lateral & Heading Error"); ax2.legend(fontsize=8)
        ax2.grid(True)

        plt.tight_layout()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(
        self,
        t:          float,
        x:          float,
        y:          float,
        theta:      float,
        x_true:     float,
        y_true:     float,
        theta_true: float,
        v:          float,
        omega:      float,
        e_lat:      float,
        e_head:     float,
        path_progress: float,
    ) -> None:
        """
        Ingest one simulation step.  Call this every plant timestep.
        Logs to CSV every step; updates plot every update_interval steps.
        """
        # Buffer
        self._xs.append(x_true)
        self._ys.append(y_true)
        self._times.append(t)
        self._e_lats.append(e_lat)
        self._e_heads.append(e_head)

        # CSV log
        self._ep_logger.log(
            t=t, x=x, y=y, theta=theta,
            x_true=x_true, y_true=y_true, theta_true=theta_true,
            v=v, omega=omega, e_lat=e_lat, e_head=e_head,
            path_progress=path_progress,
        )

        self._step_count += 1

        # Refresh plot
        if self.live_plot and self._step_count % self.update_interval == 0:
            self._refresh_plot()

    def finalize(self) -> KPIResult:
        """
        Close CSV, compute KPIs, add to experiment logger, show final plot.
        Returns the KPIResult for this episode.
        """
        self._ep_logger.close()

        kpi = compute_kpis(
            list(self._times),
            list(self._e_lats),
            settling_tol=self.settling_tol,
            gains_name=self.gains_name,
        )

        if self.exp_logger is not None:
            self.exp_logger.add(kpi)

        if self.live_plot:
            self._refresh_plot()
            self._axes[1].set_title(
                f"e_lat — OS={kpi.overshoot_m:.3f}m  "
                f"ts={kpi.settling_time_s:.2f}s  "
                f"SS={kpi.steady_state_error_m:.4f}m"
            )
            plt.ioff()
            plt.tight_layout()
            plt.savefig(
                self._ep_logger._filepath.replace(".csv", "_plot.png"), dpi=100
            )
            plt.show()

        return kpi

    # ------------------------------------------------------------------
    # Internal plot refresh
    # ------------------------------------------------------------------

    def _refresh_plot(self):
        xs     = list(self._xs)
        ys     = list(self._ys)
        ts     = list(self._times)
        e_lats = list(self._e_lats)
        e_heads= list(self._e_heads)

        if not xs:
            return

        ax1, ax2 = self._axes

        # Trajectory
        self._traj_line.set_data(xs, ys)
        if len(xs) == 1:
            self._start_dot.set_data([xs[0]], [ys[0]])
        _autoscale(ax1)

        # Error curves
        self._err_line.set_data(ts, e_lats)
        self._ehead_line.set_data(ts, e_heads)
        _autoscale(ax2)

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()


def _autoscale(ax):
    ax.relim()
    ax.autoscale_view()



