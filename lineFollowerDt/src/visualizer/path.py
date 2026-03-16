"""
path.py — Path Definitions for Line-Following Experiments
==========================================================
Provides two path types required by the capstone spec:

  1. StraightPath   — a finite straight line segment between two points.
  2. CurvedPath     — a piecewise path built from circular arc segments
                      (approximated as dense polylines), supporting
                      arbitrary curvature sequences.

Both classes expose a common interface:
  - path.points          → (N, 2) array of (x, y) waypoints
  - path.get_nearest(x, y) → (index, foot_point, signed_lateral_error,
                               heading_reference, arc_length_so_far)
  - path.total_length    → total arc length (m)
  - path.heading_at(idx) → reference heading (rad) at waypoint index

Design note
-----------
The controller (Client 2) calls get_nearest() every control step to
obtain:
  e_lat  — signed lateral error (+ = robot left of path)
  e_head — heading error = theta_robot − heading_ref

These two signals feed directly into the PID loops.
"""

import numpy as np
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _build_headings(points: np.ndarray) -> np.ndarray:
    """
    Compute forward-difference heading at each waypoint.

    For waypoint i:  heading_i = atan2(y_{i+1} - y_i, x_{i+1} - x_i)
    Last point reuses the second-to-last heading.
    """
    headings = np.zeros(len(points))
    for i in range(len(points) - 1):
        dx = points[i+1, 0] - points[i, 0]
        dy = points[i+1, 1] - points[i, 1]
        headings[i] = np.arctan2(dy, dx)
    headings[-1] = headings[-2]
    return headings


def _arc_lengths(points: np.ndarray) -> np.ndarray:
    """
    Cumulative arc-length array.  arc_lengths[0] = 0.
    """
    diffs = np.diff(points, axis=0)
    seg_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
    return np.concatenate([[0.0], np.cumsum(seg_lengths)])


def _signed_lateral_error(
    px: float, py: float,
    ax: float, ay: float,
    heading: float,
) -> float:
    """
    Signed lateral distance from point (px, py) to path point (ax, ay)
    whose forward direction is `heading`.

    Sign convention (right-hand rule):
      + → robot is to the LEFT  of the path tangent
      − → robot is to the RIGHT of the path tangent

    Derivation:
      Let t = [cos(h), sin(h)]  be the unit tangent vector.
      Let n = [-sin(h), cos(h)] be the unit normal (pointing left).
      Let d = [px - ax, py - ay].
      e_lat = dot(d, n) = (px-ax)*(-sin(h)) + (py-ay)*cos(h)
                        = -(px-ax)*sin(h) + (py-ay)*cos(h)
    """
    dx = px - ax
    dy = py - ay
    return -dx * np.sin(heading) + dy * np.cos(heading)


# ---------------------------------------------------------------------------
# Nearest-point query result
# ---------------------------------------------------------------------------

@dataclass
class NearestResult:
    index:         int    # Index of closest waypoint
    foot_x:        float  # X of foot point on path
    foot_y:        float  # Y of foot point on path
    lateral_error: float  # Signed lateral error e_lat (m)
    heading_ref:   float  # Reference heading at foot (rad)
    arc_length:    float  # Cumulative arc length to foot (m)
    progress:      float  # Fraction of path completed [0, 1]


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------

class BasePath:
    """Shared interface for all path types."""

    def __init__(self, points: np.ndarray):
        assert points.ndim == 2 and points.shape[1] == 2, \
            "points must be shape (N, 2)"
        self.points   = points                        # (N, 2)
        self.headings = _build_headings(points)       # (N,)
        self.arc_lens = _arc_lengths(points)          # (N,)

    @property
    def total_length(self) -> float:
        return float(self.arc_lens[-1])

    def heading_at(self, idx: int) -> float:
        return float(self.headings[idx])

    def get_nearest(self, robot_x: float, robot_y: float) -> NearestResult:
        """
        Find the closest waypoint to (robot_x, robot_y) and compute
        all control-relevant quantities.
        """
        rp = np.array([robot_x, robot_y])
        dists = np.hypot(self.points[:, 0] - rp[0],
                         self.points[:, 1] - rp[1])
        idx = int(np.argmin(dists))

        foot_x = self.points[idx, 0]
        foot_y = self.points[idx, 1]
        h_ref  = self.headings[idx]

        e_lat = _signed_lateral_error(robot_x, robot_y, foot_x, foot_y, h_ref)
        arc   = float(self.arc_lens[idx])
        prog  = arc / self.total_length if self.total_length > 0 else 0.0

        return NearestResult(
            index=idx,
            foot_x=foot_x,
            foot_y=foot_y,
            lateral_error=e_lat,
            heading_ref=h_ref,
            arc_length=arc,
            progress=prog,
        )

    def is_finished(self, robot_x: float, robot_y: float,
                    threshold: float = 0.3) -> bool:
        """
        Returns True when the robot is within `threshold` metres of the
        last waypoint — used to terminate a simulation episode.
        """
        end = self.points[-1]
        return float(np.hypot(robot_x - end[0], robot_y - end[1])) < threshold


# ---------------------------------------------------------------------------
# 1. Straight Path
# ---------------------------------------------------------------------------

class StraightPath(BasePath):
    """
    A straight-line path from `start` to `end`.

    Parameters
    ----------
    start : array-like, shape (2,)
        Starting point (x, y) in metres.
    end : array-like, shape (2,)
        End point (x, y) in metres.
    resolution : float
        Spacing between waypoints (m). Smaller = more accurate nearest query.
    """

    def __init__(
        self,
        start=(0.0, 0.0),
        end=(10.0, 0.0),
        resolution: float = 0.05,
    ):
        start = np.asarray(start, dtype=float)
        end   = np.asarray(end,   dtype=float)
        length = float(np.hypot(*(end - start)))
        n = max(2, int(np.ceil(length / resolution)) + 1)
        xs = np.linspace(start[0], end[0], n)
        ys = np.linspace(start[1], end[1], n)
        points = np.column_stack([xs, ys])
        super().__init__(points)

        self.start = start
        self.end   = end


# ---------------------------------------------------------------------------
# 2. Curved Path — piecewise circular arcs
# ---------------------------------------------------------------------------

class CurvedPath(BasePath):
    """
    A piecewise path made of circular arc segments.

    Each segment is defined by:
      - centre     : (cx, cy) — centre of curvature
      - radius     : r (m) — radius of the arc
      - start_angle: angle (rad) to the arc start point from centre
      - sweep      : total angular sweep (rad, +CCW / -CW)
      - resolution : arc-length spacing between sampled points (m)

    Usage example (S-bend):
    ------------------------
    segments = [
        # Segment 0 — left arc, centre above start
        {"centre": (0, 5), "radius": 5, "start_angle": -np.pi/2, "sweep":  np.pi/2},
        # Segment 1 — right arc, continue C-shape
        {"centre": (10, 5), "radius": 5, "start_angle": np.pi,   "sweep": -np.pi/2},
    ]
    path = CurvedPath(segments)
    """

    def __init__(self, segments: list[dict], resolution: float = 0.05):
        all_points = []

        for seg in segments:
            cx, cy      = seg["centre"]
            r           = float(seg["radius"])
            a0          = float(seg["start_angle"])
            sweep       = float(seg["sweep"])
            n           = max(2, int(np.ceil(abs(sweep) * r / resolution)) + 1)
            angles      = np.linspace(a0, a0 + sweep, n)
            xs          = cx + r * np.cos(angles)
            ys          = cy + r * np.sin(angles)
            seg_pts     = np.column_stack([xs, ys])

            # Avoid duplicate junction points between segments
            if all_points:
                seg_pts = seg_pts[1:]
            all_points.append(seg_pts)

        points = np.vstack(all_points)
        super().__init__(points)


# ---------------------------------------------------------------------------
# Factory — convenience constructors used by experiments
# ---------------------------------------------------------------------------

def make_straight(length: float = 15.0, angle_deg: float = 0.0) -> StraightPath:
    """
    Create a straight path of given length starting at origin,
    oriented at `angle_deg` degrees from the +X axis.
    """
    a = np.deg2rad(angle_deg)
    end = (length * np.cos(a), length * np.sin(a))
    return StraightPath(start=(0.0, 0.0), end=end)


def make_s_curve(straight_lead: float = 3.0) -> CurvedPath:
    """
    Create a symmetric S-bend path:
      - Optional straight lead-in (approximated as tight-radius arc is omitted;
        caller should prepend a StraightPath if needed).
      - Left arc: centre (straight_lead + 5, 0), radius 5, sweep +pi/2
      - Right arc: centre (straight_lead + 5, 10), radius 5, sweep -pi/2

    Resulting path goes from (straight_lead, -5) → curves right → ends heading +X.
    """
    cx1 = straight_lead + 5.0
    segments = [
        {
            "centre":      (cx1, 0.0),
            "radius":      5.0,
            "start_angle": -np.pi / 2,   # start below centre
            "sweep":        np.pi / 2,   # CCW → end pointing right
        },
        {
            "centre":      (cx1 + 5.0, 5.0),
            "radius":      5.0,
            "start_angle":  np.pi,       # start left of centre
            "sweep":       -np.pi / 2,   # CW → end pointing right
        },
    ]
    return CurvedPath(segments)


def make_random_spawn(path: BasePath,
                      lateral_range: float = 1.5,
                      heading_noise_deg: float = 20.0,
                      rng: np.random.Generator | None = None
                      ) -> tuple[float, float, float]:
    """
    Generate a random starting pose near the beginning of `path`.

    Returns (x0, y0, theta0) suitable for robot.reset().

    Parameters
    ----------
    path : BasePath
        The path to spawn near.
    lateral_range : float
        Maximum lateral offset from path start (m).
    heading_noise_deg : float
        Max heading deviation from path heading at start (deg).
    rng : np.random.Generator, optional
        For reproducibility. Defaults to numpy default rng.
    """
    if rng is None:
        rng = np.random.default_rng()

    # Lateral offset — perpendicular to path start heading
    h0    = path.headings[0]
    e_lat = rng.uniform(-lateral_range, lateral_range)

    # Perpendicular direction (left-normal)
    x0 = path.points[0, 0] + e_lat * (-np.sin(h0))
    y0 = path.points[0, 1] + e_lat * ( np.cos(h0))

    # Heading noise
    dtheta = np.deg2rad(rng.uniform(-heading_noise_deg, heading_noise_deg))
    theta0 = h0 + dtheta

    return float(x0), float(y0), float(theta0)
