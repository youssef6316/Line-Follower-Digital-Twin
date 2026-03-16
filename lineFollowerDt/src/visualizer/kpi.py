"""
kpi.py — Key Performance Indicator Computation
================================================
Computes the three KPIs required by the capstone spec for each episode:

  1. Overshoot (m)
     Maximum lateral deviation PAST zero (opposite sign to initial error).
     Indicates whether the controller over-corrects.

     OS = max(0,  max(−sign(e0) * e_lat[t]))   for t > t_cross

  2. Settling Time (s)
     First time after which |e_lat| stays within a tolerance band
     [−tol, +tol] for a sustained window.

     ts = min{ t : |e_lat(τ)| ≤ tol  ∀ τ ∈ [t, t + T_window] }

  3. Steady-State Error (m)
     Mean |e_lat| over the final steady_frac fraction of the episode,
     after transients have died out.

     e_ss = mean(|e_lat[t_ss:]|)

All three are computed from the lateral error time-series produced by
the closed-loop simulation.  The functions accept plain Python lists or
numpy arrays, so they work both standalone and inside the VSI visualizer.
"""

import numpy as np
from dataclasses import dataclass
from typing import Sequence


@dataclass
class KPIResult:
    """
    KPI results for one simulation episode.

    Attributes
    ----------
    overshoot_m : float
        Peak lateral overshoot (m).  0.0 if no overshoot occurs.
    settling_time_s : float
        Time to enter and stay in tolerance band (s).
        Set to episode_duration if never settled.
    steady_state_error_m : float
        Mean absolute lateral error over final steady_frac of episode (m).
    settled : bool
        True if the robot settled within the episode duration.
    episode_duration_s : float
        Total simulated time of the episode (s).
    max_lateral_error_m : float
        Maximum |e_lat| over the entire episode (m).
    gains_name : str
        Label of the gain set used (for logging).
    """
    overshoot_m:          float
    settling_time_s:      float
    steady_state_error_m: float
    settled:              bool
    episode_duration_s:   float
    max_lateral_error_m:  float
    gains_name:           str = ""

    def as_dict(self) -> dict:
        return {
            "gains":               self.gains_name,
            "overshoot_m":         round(self.overshoot_m,          4),
            "settling_time_s":     round(self.settling_time_s,      4),
            "steady_state_err_m":  round(self.steady_state_error_m, 4),
            "settled":             int(self.settled),
            "episode_duration_s":  round(self.episode_duration_s,   4),
            "max_lateral_err_m":   round(self.max_lateral_error_m,  4),
        }


def compute_kpis(
    times:         Sequence[float],
    lateral_errors: Sequence[float],
    settling_tol:  float = 0.05,
    settling_window_s: float = 1.0,
    steady_frac:   float = 0.2,
    gains_name:    str   = "",
) -> KPIResult:
    """
    Compute all three KPIs from a lateral error time series.

    Parameters
    ----------
    times : array-like, shape (N,)
        Simulation timestamps (s).
    lateral_errors : array-like, shape (N,)
        Signed lateral error e_lat at each timestep (m).
    settling_tol : float
        Half-width of the settling band (m).  Default 0.05 m.
    settling_window_s : float
        Duration the error must stay inside the band to count as settled (s).
    steady_frac : float
        Fraction of episode tail used for steady-state error.  Default 20%.
    gains_name : str
        Label forwarded into the KPIResult for logging.

    Returns
    -------
    KPIResult
    """
    t  = np.asarray(times,          dtype=float)
    el = np.asarray(lateral_errors, dtype=float)
    N  = len(t)

    if N < 2:
        return KPIResult(0.0, 0.0, 0.0, False, 0.0, 0.0, gains_name)

    dt_mean          = float(np.mean(np.diff(t)))
    episode_duration = float(t[-1] - t[0])
    max_lat_err      = float(np.max(np.abs(el)))

    # ── 1. Overshoot ──────────────────────────────────────────────────────
    # Initial error sign determines which direction is "overshoot"
    e0_sign = np.sign(el[0]) if el[0] != 0.0 else 1.0

    # Find first zero-crossing index
    cross_idx = N  # default: no crossing
    for i in range(1, N):
        if el[i] * e0_sign <= 0.0:
            cross_idx = i
            break

    if cross_idx < N:
        # Overshoot = max excursion in the opposite direction after crossing
        overshoot = float(np.max(-e0_sign * el[cross_idx:]))
        overshoot = max(0.0, overshoot)
    else:
        overshoot = 0.0

    # ── 2. Settling Time ──────────────────────────────────────────────────
    window_steps = max(1, int(np.round(settling_window_s / dt_mean)))
    abs_el       = np.abs(el)
    settled      = False
    settling_time = episode_duration  # pessimistic default

    for i in range(N - window_steps):
        window = abs_el[i: i + window_steps]
        if np.all(window <= settling_tol):
            settling_time = float(t[i] - t[0])
            settled = True
            break

    # ── 3. Steady-State Error ─────────────────────────────────────────────
    tail_start    = max(0, int(N * (1.0 - steady_frac)))
    steady_state  = float(np.mean(abs_el[tail_start:]))

    return KPIResult(
        overshoot_m          = overshoot,
        settling_time_s      = settling_time,
        steady_state_error_m = steady_state,
        settled              = settled,
        episode_duration_s   = episode_duration,
        max_lateral_error_m  = max_lat_err,
        gains_name           = gains_name,
    )


def extract_lateral_series(trajectory: list[dict], path) -> tuple[list, list]:
    """
    Helper: extract (times, lateral_errors) from a robot history list
    and a path object.

    Parameters
    ----------
    trajectory : list[dict]
        Robot history from DifferentialDriveRobot.history.
    path : BasePath
        The reference path used in the episode.

    Returns
    -------
    (times, lateral_errors) — plain Python lists, ready for compute_kpis().
    """
    times  = []
    errors = []
    for rec in trajectory:
        nr = path.get_nearest(rec["x"], rec["y"])
        times.append(rec["t"])
        errors.append(nr.lateral_error)
    return times, errors
