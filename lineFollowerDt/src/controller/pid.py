"""
pid.py — PID Controller for Line-Following Robot
=================================================
Implements a dual-channel PID controller that minimizes:

  Channel A — Lateral error   e_lat  (m)   → contributes to omega
  Channel B — Heading error   e_head (rad) → contributes to omega

The two channels are combined to produce the angular velocity command:

  omega = -(Kp_lat * e_lat + Ki_lat * I_lat + Kd_lat * D_lat)
          -(Kp_head* e_head+ Ki_head* I_head+ Kd_head* D_head)

Linear velocity uses optional speed tapering — slows the robot when
lateral error is large, allowing the controller more time to recover:

  v = v_ref * max(v_min_frac, 1 - taper_alpha * |e_lat|)

Anti-windup strategy: integrator clamping
  If |I_channel| > windup_limit, the integrator is frozen for that
  channel on the current step.  This is the simplest and most robust
  anti-windup approach for a teaching context.

Gain sets (used by E1 — PID Gain Sweep experiment):
  GAIN_SETS is a list of named GainConfig objects covering a range
  from under-damped (aggressive) to over-damped (sluggish), plus
  a tuned baseline that should perform well on the straight path.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional


# ---------------------------------------------------------------------------
# Gain configuration
# ---------------------------------------------------------------------------

@dataclass
class GainConfig:
    """
    Complete gain parameterisation for one experiment run.

    Attributes
    ----------
    name : str
        Human-readable label used in plots and CSV logs.
    Kp_lat, Ki_lat, Kd_lat : float
        PID gains for the lateral error channel.
    Kp_head, Ki_head, Kd_head : float
        PID gains for the heading error channel.
    v_ref : float
        Nominal forward speed (m/s).
    taper_alpha : float
        Speed-taper coefficient.  v = v_ref*(1 - alpha*|e_lat|).
        Set to 0.0 to disable tapering.
    v_min_frac : float
        Minimum speed as a fraction of v_ref (prevents stopping).
    windup_limit : float
        Integrator clamp magnitude (same for both channels).
    omega_max : float
        Saturation limit on angular velocity output (rad/s).
    """
    name:         str   = "baseline"
    Kp_lat:       float = 1.2
    Ki_lat:       float = 0.05
    Kd_lat:       float = 0.3
    Kp_head:      float = 1.8
    Ki_head:      float = 0.02
    Kd_head:      float = 0.4
    v_ref:        float = 1.0
    taper_alpha:  float = 0.3
    v_min_frac:   float = 0.25
    windup_limit: float = 2.0
    omega_max:    float = np.pi


# ---------------------------------------------------------------------------
# Predefined gain sets for E1 — PID Gain Sweep
# ---------------------------------------------------------------------------

GAIN_SETS: list[GainConfig] = [
    GainConfig(
        name        = "G1_sluggish",
        Kp_lat      = 0.4,  Ki_lat  = 0.01, Kd_lat  = 0.05,
        Kp_head     = 0.6,  Ki_head = 0.01, Kd_head = 0.1,
        v_ref       = 0.8,
        taper_alpha = 0.1,
    ),
    GainConfig(
        name        = "G2_underdamped",
        Kp_lat      = 2.0,  Ki_lat  = 0.1,  Kd_lat  = 0.1,
        Kp_head     = 2.5,  Ki_head = 0.05, Kd_head = 0.1,
        v_ref       = 1.0,
        taper_alpha = 0.2,
    ),
    GainConfig(
        name        = "G3_baseline",   # expected best on straight path
        Kp_lat      = 1.2,  Ki_lat  = 0.05, Kd_lat  = 0.3,
        Kp_head     = 1.8,  Ki_head = 0.02, Kd_head = 0.4,
        v_ref       = 1.0,
        taper_alpha = 0.3,
    ),
    GainConfig(
        name        = "G4_aggressive",
        Kp_lat      = 3.5,  Ki_lat  = 0.2,  Kd_lat  = 0.5,
        Kp_head     = 4.0,  Ki_head = 0.1,  Kd_head = 0.6,
        v_ref       = 1.2,
        taper_alpha = 0.4,
    ),
    GainConfig(
        name        = "G5_no_integral",  # PD-only baseline (used in E4)
        Kp_lat      = 1.2,  Ki_lat  = 0.0,  Kd_lat  = 0.3,
        Kp_head     = 1.8,  Ki_head = 0.0,  Kd_head = 0.4,
        v_ref       = 1.0,
        taper_alpha = 0.3,
    ),
]

# E4 convenience references
GAINS_PID = GAIN_SETS[2]   # G3_baseline  — full PID
GAINS_PD  = GAIN_SETS[4]   # G5_no_integral — PD only


# ---------------------------------------------------------------------------
# PID controller state
# ---------------------------------------------------------------------------

@dataclass
class _ChannelState:
    """Internal integrator + derivative state for one PID channel."""
    integral:  float = 0.0
    prev_error: float = 0.0
    initialized: bool = False


class PIDController:
    """
    Dual-channel PID controller for line-following.

    Usage
    -----
    pid = PIDController(gains, dt)
    v, omega = pid.step(e_lat, e_head)

    Parameters
    ----------
    gains : GainConfig
        Gain set to use.
    dt : float
        Control timestep (s).  Must match the controller client rate.
    """

    def __init__(self, gains: GainConfig, dt: float = 0.1):
        self.gains = gains
        self.dt    = dt
        self._lat  = _ChannelState()
        self._head = _ChannelState()

        # Per-step output log
        self.history: list[dict] = []

    # ------------------------------------------------------------------
    # Private: single-channel PID update
    # ------------------------------------------------------------------

    def _channel_update(
        self,
        state: _ChannelState,
        error: float,
        Kp: float, Ki: float, Kd: float,
        windup_limit: float,
    ) -> float:
        """
        Compute one PID channel output with anti-windup clamping.

        Discrete PID (forward Euler):
          P = Kp * e
          I += Ki * e * dt        (frozen if |I| > windup_limit)
          D = Kd * (e - e_prev) / dt

        Returns P + I + D.
        """
        P = Kp * error

        # Derivative — zero on first call to avoid initial kick
        if not state.initialized:
            D = 0.0
            state.initialized = True
        else:
            D = Kd * (error - state.prev_error) / self.dt

        # Integrator with anti-windup clamping
        new_integral = state.integral + Ki * error * self.dt
        if abs(new_integral) <= windup_limit:
            state.integral = new_integral
        # else: hold current integral (freeze)

        state.prev_error = error

        return P + state.integral + D

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def step(self, e_lat: float, e_head: float) -> tuple[float, float]:
        """
        Compute velocity commands for one control step.

        Parameters
        ----------
        e_lat : float
            Lateral error (m).
        e_head : float
            Heading error (rad).

        Returns
        -------
        v : float
            Linear velocity command (m/s).
        omega : float
            Angular velocity command (rad/s).
        """
        g = self.gains

        # Lateral channel
        u_lat = self._channel_update(
            self._lat, e_lat,
            g.Kp_lat, g.Ki_lat, g.Kd_lat,
            g.windup_limit,
        )

        # Heading channel
        u_head = self._channel_update(
            self._head, e_head,
            g.Kp_head, g.Ki_head, g.Kd_head,
            g.windup_limit,
        )

        # Combine: both channels correct omega (negative = corrective)
        omega = float(np.clip(-(u_lat + u_head), -g.omega_max, g.omega_max))

        # Speed tapering: slow down when far from path
        taper = max(g.v_min_frac, 1.0 - g.taper_alpha * abs(e_lat))
        v     = float(g.v_ref * taper)

        self.history.append({
            "e_lat":   e_lat,
            "e_head":  e_head,
            "u_lat":   u_lat,
            "u_head":  u_head,
            "v":       v,
            "omega":   omega,
            "I_lat":   self._lat.integral,
            "I_head":  self._head.integral,
        })

        return v, omega

    def reset(self, gains: Optional[GainConfig] = None) -> None:
        """
        Reset integrators and history.
        Optionally swap in a new GainConfig (used between gain-sweep runs).
        """
        if gains is not None:
            self.gains = gains
        self._lat  = _ChannelState()
        self._head = _ChannelState()
        self.history.clear()
