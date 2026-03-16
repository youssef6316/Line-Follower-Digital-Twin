"""
vsi_bridge.py — VSI Abstraction Layer
======================================
This module is the ONLY file that changes between standalone mode and
real VSI deployment.  All three clients import from here instead of
calling VSI APIs directly.

Architecture
------------
Standalone mode (NOW):
  SharedBus acts as an in-process message bus.  All three clients run
  in the same Python process (run_standalone.py) and share one SharedBus
  instance.  Signal reads/writes are just dict lookups — zero latency.

VSI mode (LATER — when Innexis VSI is installed):
  Replace SharedBus with real VSI python2DtCan gateway calls.
  The three clients become separate processes launched by VsiSim.
  The only change required in client code is:
    bridge = VSIBridge(mode="vsi")   instead of   mode="standalone"

Signal schema (mirrors vsiBuildCommands):
  plant → controller, visualizer:
    "x"          : float  — noisy position X (m)
    "y"          : float  — noisy position Y (m)
    "theta"      : float  — noisy heading (rad)
    "x_true"     : float  — ground-truth X (m)    [visualizer only]
    "y_true"     : float  — ground-truth Y (m)    [visualizer only]
    "theta_true" : float  — ground-truth heading  [visualizer only]
    "t"          : float  — simulation time (s)
    "progress"   : float  — path completion [0,1]

  controller → plant, visualizer:
    "v"          : float  — linear velocity command (m/s)
    "omega"      : float  — angular velocity command (rad/s)
    "e_lat"      : float  — lateral error (m)
    "e_head"     : float  — heading error (rad)

VSI CAN frame mapping (from vsiBuildCommands):
  CAN ID 10 — pose_x       (plant → all)
  CAN ID 11 — pose_y       (plant → all)
  CAN ID 12 — pose_theta   (plant → all)
  CAN ID 13 — v_cmd        (controller → all)
  CAN ID 14 — omega_cmd    (controller → all)
  CAN ID 15 — e_lat        (controller → visualizer)
  CAN ID 16 — e_head       (controller → visualizer)
  CAN ID 17 — sim_time     (plant → all)
  CAN ID 18 — progress     (plant → all)
"""

import threading
from typing import Any


# ---------------------------------------------------------------------------
# Shared in-process bus (standalone mode)
# ---------------------------------------------------------------------------

class SharedBus:
    """
    Thread-safe in-process signal bus used in standalone mode.

    All three client threads read and write to a single SharedBus instance.
    This perfectly mimics the publish/subscribe semantics of the VSI
    CAN backplane without requiring any network or IPC infrastructure.
    """

    def __init__(self):
        self._signals: dict[str, Any] = {}
        self._lock    = threading.Lock()
        self._events: dict[str, threading.Event] = {}

    def publish(self, signal: str, value: Any) -> None:
        """Write a signal value (non-blocking)."""
        with self._lock:
            self._signals[signal] = value
            if signal in self._events:
                self._events[signal].set()

    def read(self, signal: str, default: Any = 0.0) -> Any:
        """Read the latest value of a signal (non-blocking)."""
        with self._lock:
            return self._signals.get(signal, default)

    def publish_batch(self, signals: dict[str, Any]) -> None:
        """Atomically publish multiple signals."""
        with self._lock:
            self._signals.update(signals)

    def read_batch(self, keys: list[str], default: Any = 0.0) -> dict:
        """Atomically read multiple signals."""
        with self._lock:
            return {k: self._signals.get(k, default) for k in keys}


# ---------------------------------------------------------------------------
# VSI Bridge — wraps either SharedBus or real VSI API
# ---------------------------------------------------------------------------

class VSIBridge:
    """
    Unified interface used by all three clients.

    Parameters
    ----------
    mode : str
        "standalone" — use SharedBus (default, works now)
        "vsi"        — use real Innexis VSI python2DtCan gateway
    bus : SharedBus, optional
        Shared bus instance (standalone mode only).
        All three clients must share the SAME instance.
    client_name : str
        Name of this client ("plant", "controller", "visualizer").
        Used for logging only.
    """

    PLANT_SIGNALS      = ["x", "y", "theta", "x_true", "y_true",
                           "theta_true", "t", "progress"]
    CONTROLLER_SIGNALS = ["v", "omega", "e_lat", "e_head"]
    ALL_SIGNALS        = PLANT_SIGNALS + CONTROLLER_SIGNALS

    def __init__(
        self,
        mode:        str         = "standalone",
        bus:         SharedBus | None = None,
        client_name: str         = "unknown",
    ):
        self.mode        = mode
        self.client_name = client_name

        if mode == "standalone":
            assert bus is not None, \
                "SharedBus instance required in standalone mode."
            self._bus = bus
            self._vsi = None
        elif mode == "vsi":
            # ── VSI mode: import the real VSI Python API ──────────────────
            # When running inside the VSI container, vsiCommonPythonApi is
            # available on sys.path.  Uncomment when deploying:
            #
            # import vsiCommonPythonApi as vsi_api
            # self._vsi = vsi_api
            #
            # For now, raise a clear error so the swap-point is obvious:
            raise NotImplementedError(
                "VSI mode not yet active.  Install Innexis VSI and replace "
                "this block with:  import vsiCommonPythonApi as vsi_api"
            )
        else:
            raise ValueError(f"Unknown mode '{mode}'. Use 'standalone' or 'vsi'.")

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def publish(self, signal: str, value: float) -> None:
        """
        Publish one signal.

        Standalone: writes to SharedBus.
        VSI:        calls self.mySignals.<signal> = value
                    (vsiCommonPythonApi handles CAN frame packing)
        """
        if self.mode == "standalone":
            self._bus.publish(signal, value)
        else:
            # VSI equivalent:
            # setattr(self._vsi_signals, signal, value)
            pass

    def publish_batch(self, signals: dict) -> None:
        """Atomically publish multiple signals."""
        if self.mode == "standalone":
            self._bus.publish_batch(signals)
        else:
            for k, v in signals.items():
                self.publish(k, v)

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def read(self, signal: str, default: float = 0.0) -> float:
        """
        Read one signal.

        Standalone: reads from SharedBus.
        VSI:        reads self.mySignals.<signal>
        """
        if self.mode == "standalone":
            return self._bus.read(signal, default)
        else:
            # VSI equivalent:
            # return getattr(self._vsi_signals, signal)
            return default

    def read_plant_state(self) -> dict:
        """Convenience: read all plant-published signals at once."""
        return self._bus.read_batch(self.PLANT_SIGNALS)

    def read_controller_output(self) -> dict:
        """Convenience: read all controller-published signals at once."""
        return self._bus.read_batch(self.CONTROLLER_SIGNALS)

    # ------------------------------------------------------------------
    # Simulation time (VSI provides this via getSimulationTimeInNs)
    # ------------------------------------------------------------------

    def sim_time_s(self) -> float:
        """
        Return current simulation time in seconds.

        Standalone: reads "t" from bus.
        VSI:        vsiCommonPythonApi.getSimulationTimeInNs() * 1e-9
        """
        if self.mode == "standalone":
            return self._bus.read("t", 0.0)
        else:
            # return self._vsi.getSimulationTimeInNs() * 1e-9
            return 0.0
