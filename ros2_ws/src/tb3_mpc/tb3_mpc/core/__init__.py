"""ROS-free core: model, references, controllers, NMPC, perception, simulation, metrics."""
from __future__ import annotations

from .controllers import Kanayama, KanayamaGains, Samson, SamsonGains, SlidingMode, SMCGains
from .nmpc import NMPC, NMPCConfig
from .safety_filter import CBFSafetyFilter, FilteredController

CONTROLLER_NAMES = ("nmpc", "kanayama", "samson", "smc")


def make_controller(name: str, params: dict | None = None, dt: float = 0.1,
                    safety_filter: bool = False, max_obstacles: int = 8):
    """Factory used by the ROS node and the benchmark scripts.

    ``params`` holds the gains / NMPC config fields for the chosen controller.
    ``safety_filter`` wraps a *baseline* in the CBF-QP filter (NMPC handles
    obstacles inside its own optimisation and ignores this flag).
    """
    params = dict(params or {})
    if name == "nmpc":
        params.setdefault("dt", dt)
        params.setdefault("max_obstacles", max_obstacles)
        return NMPC(NMPCConfig(**params))
    if name == "kanayama":
        ctrl = Kanayama(dt, KanayamaGains(**params))
    elif name == "samson":
        ctrl = Samson(dt, SamsonGains(**params))
    elif name == "smc":
        ctrl = SlidingMode(dt, SMCGains(**params))
    else:
        raise ValueError(f"unknown controller {name!r}; choose from {CONTROLLER_NAMES}")
    if safety_filter:
        ctrl = FilteredController(ctrl, CBFSafetyFilter(max_obstacles=max_obstacles))
    return ctrl
