"""
errors.py — Control Error Computation for Line-Following
=========================================================
Computes the two error signals that feed the PID controller:

  e_lat  — signed lateral error (m)
           Positive → robot is LEFT of path (must steer right)
           Negative → robot is RIGHT of path (must steer left)

  e_head — heading error (rad)
           e_head = theta_robot − heading_ref
           Positive → robot points LEFT of path tangent
           Negative → robot points RIGHT of path tangent

Control law (implemented in pid.py):
  omega = −Kp_lat * e_lat − Kp_head * e_head
        − Ki_lat * ∫e_lat dt − Ki_head * ∫e_head dt
        − Kd_lat * ė_lat    − Kd_head * ė_head

  v     = v_ref * (1 − alpha * |e_lat|)   [speed tapering — optional]

Both errors are computed from the NearestResult returned by path.get_nearest().
"""

import numpy as np
from dataclasses import dataclass
from path import NearestResult


@dataclass
class ControlErrors:
    """All error signals for one control step."""
    e_lat:       float   # Lateral error (m)
    e_head:      float   # Heading error (rad)
    heading_ref: float   # Reference heading at foot point (rad)
    foot_x:      float   # Foot point X (m)
    foot_y:      float   # Foot point Y (m)
    progress:    float   # Path completion fraction [0, 1]
    arc_length:  float   # Arc length to foot (m)


def compute_errors(
    robot_x: float,
    robot_y: float,
    robot_theta: float,
    nearest: NearestResult,
) -> ControlErrors:
    """
    Compute lateral and heading errors given the robot pose and the
    nearest-point result from path.get_nearest().

    Parameters
    ----------
    robot_x, robot_y : float
        Current robot position (m) — from noisy pose published by Client 1.
    robot_theta : float
        Current robot heading (rad) — from noisy pose published by Client 1.
    nearest : NearestResult
        Output of path.get_nearest(robot_x, robot_y).

    Returns
    -------
    ControlErrors
        Dataclass with all error signals ready for PID input.

    Notes
    -----
    Heading error is wrapped to (−π, π] to avoid 2π discontinuities
    that would cause derivative kick in the PID.
    """
    e_lat  = nearest.lateral_error

    # Wrap heading error to (−pi, pi]
    raw    = robot_theta - nearest.heading_ref
    e_head = (raw + np.pi) % (2 * np.pi) - np.pi

    return ControlErrors(
        e_lat       = e_lat,
        e_head      = e_head,
        heading_ref = nearest.heading_ref,
        foot_x      = nearest.foot_x,
        foot_y      = nearest.foot_y,
        progress    = nearest.progress,
        arc_length  = nearest.arc_length,
    )
