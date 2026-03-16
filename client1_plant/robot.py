"""
robot.py — Differential-Drive Robot Kinematics (Plant Model)
=============================================================
Implements the unicycle kinematic model for a differential-drive robot.

State vector:  s = [x, y, theta]
  x      — world-frame X position (m)
  y      — world-frame Y position (m)
  theta  — heading angle, measured CCW from +X axis (rad)

Control inputs:
  v      — linear velocity  (m/s)
  omega  — angular velocity (rad/s)

Continuous-time kinematics (used in report derivation):
  dx/dt     = v * cos(theta)
  dy/dt     = v * sin(theta)
  dtheta/dt = omega

Integration is performed with a fixed-step 4th-order Runge-Kutta (RK4)
scheme, matching the VSI simulation step.

Sensor noise is modelled as zero-mean Gaussian additive noise on the
published pose, representing IMU/encoder uncertainty.
"""

import numpy as np


class DifferentialDriveRobot:
    """
    Simulates a differential-drive (unicycle) robot.

    Parameters
    ----------
    x0 : float
        Initial X position (m).
    y0 : float
        Initial Y position (m).
    theta0 : float
        Initial heading (rad).
    dt : float
        Simulation timestep (s). Default 0.01 s → 100 Hz plant rate.
    noise_std_pos : float
        Std-dev of Gaussian position noise added to published pose (m).
    noise_std_theta : float
        Std-dev of Gaussian heading noise added to published pose (rad).
    v_max : float
        Saturation limit for linear velocity (m/s).
    omega_max : float
        Saturation limit for angular velocity (rad/s).
    """

    def __init__(
        self,
        x0: float = 0.0,
        y0: float = 0.0,
        theta0: float = 0.0,
        dt: float = 0.01,
        noise_std_pos: float = 0.0,
        noise_std_theta: float = 0.0,
        v_max: float = 2.0,
        omega_max: float = np.pi,
    ):
        # True (ground-truth) state — never exposed directly to controller
        self._x = float(x0)
        self._y = float(y0)
        self._theta = float(theta0)

        self.dt = dt
        self.noise_std_pos = noise_std_pos
        self.noise_std_theta = noise_std_theta
        self.v_max = v_max
        self.omega_max = omega_max

        # Simulation clock
        self.t = 0.0

        # History for logging / KPI computation
        self.history: list[dict] = []

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _derivatives(self, state: np.ndarray, v: float, omega: float) -> np.ndarray:
        """
        Compute continuous-time derivatives for RK4.

        f(s, u) = [v*cos(theta), v*sin(theta), omega]
        """
        _, _, theta = state
        dxdt = v * np.cos(theta)
        dydt = v * np.sin(theta)
        dthetadt = omega
        return np.array([dxdt, dydt, dthetadt])

    def _rk4_step(self, v: float, omega: float) -> None:
        """
        Advance the true state by one timestep using RK4.

        Standard RK4:
          k1 = f(s,          u)
          k2 = f(s + dt/2*k1, u)
          k3 = f(s + dt/2*k2, u)
          k4 = f(s + dt*k3,   u)
          s_next = s + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        """
        s = np.array([self._x, self._y, self._theta])
        h = self.dt

        k1 = self._derivatives(s,             v, omega)
        k2 = self._derivatives(s + h/2 * k1,  v, omega)
        k3 = self._derivatives(s + h/2 * k2,  v, omega)
        k4 = self._derivatives(s + h   * k3,  v, omega)

        s_next = s + (h / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        self._x, self._y, self._theta = s_next
        # Normalise heading to (-pi, pi]
        self._theta = self._wrap_angle(self._theta)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to the interval (-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def step(self, v: float, omega: float) -> dict:
        """
        Advance the simulation by one timestep.

        Parameters
        ----------
        v : float
            Commanded linear velocity (m/s) — will be saturated.
        omega : float
            Commanded angular velocity (rad/s) — will be saturated.

        Returns
        -------
        dict
            Noisy pose published over VSI:
            {'x', 'y', 'theta', 't', 'v_true', 'omega_true'}
        """
        # Saturate inputs (actuator limits)
        v     = float(np.clip(v,     -self.v_max,     self.v_max))
        omega = float(np.clip(omega, -self.omega_max, self.omega_max))

        # Integrate true state
        self._rk4_step(v, omega)
        self.t += self.dt

        # Add sensor noise to produce the "measured" pose
        noisy_x     = self._x     + np.random.normal(0.0, self.noise_std_pos)
        noisy_y     = self._y     + np.random.normal(0.0, self.noise_std_pos)
        noisy_theta = self._theta + np.random.normal(0.0, self.noise_std_theta)
        noisy_theta = self._wrap_angle(noisy_theta)

        record = {
            "t":          self.t,
            "x":          noisy_x,
            "y":          noisy_y,
            "theta":      noisy_theta,
            "x_true":     self._x,
            "y_true":     self._y,
            "theta_true": self._theta,
            "v":          v,
            "omega":      omega,
        }
        self.history.append(record)
        return record

    def reset(self, x0: float = 0.0, y0: float = 0.0, theta0: float = 0.0) -> None:
        """Reset robot state and history (used between experiment runs)."""
        self._x     = float(x0)
        self._y     = float(y0)
        self._theta = float(theta0)
        self.t      = 0.0
        self.history.clear()

    def apply_disturbance(self, lateral_magnitude: float) -> None:
        """
        Apply an instantaneous lateral impulse to the robot's true position.
        Positive magnitude pushes the robot to the LEFT of its current heading.

        Used by E3 (noise + disturbance rejection experiment) to simulate
        a bump or slip event mid-episode.

        Parameters
        ----------
        lateral_magnitude : float
            Displacement in metres, perpendicular to current heading.
        """
        h = self._theta
        self._x += lateral_magnitude * (-np.sin(h))
        self._y += lateral_magnitude * ( np.cos(h))

    @property
    def true_pose(self) -> tuple[float, float, float]:
        """Return ground-truth pose (x, y, theta) — for KPI computation only."""
        return self._x, self._y, self._theta

    @property
    def state_dict(self) -> dict:
        """Return current true state as a dict."""
        return {"x": self._x, "y": self._y, "theta": self._theta, "t": self.t}
