"""The plant side of a haptic-teleop run on the real FR3.

The haptic loop runs in its own process (``haptic_teleop.HapticProcess``), so nothing in this
process — ROS callbacks, `Robot`, logging — can delay it. This module is what stays here:
[PlantLoop][] runs at the plant rate on the main thread and

- streams the targets to ``osc_controller``: the leader state for the coupling methods, the
  proxy state and a feedforward force for the proxy methods, both read from the haptic loop's
  ``HapticStateMailbox``;
- for ``proxy-rim``, refreshes the dynamics model and publishes it in a ``ModelMailbox``;
- freezes the robot and ends the run when the haptic loop's watchdog trips (plant samples went
  stale), read from its ``StatusMailbox``.

The plant samples go the other way without passing through here: `FR3Plant`'s callbacks
publish them in a ``PlantMailbox``.

Interface points. The coupling methods couple the leader to the controller's end-effector
(``ee_state``), so the leader's origin is that point. The RIM proxy lives at the interface
point of the dynamics model (`RobotModelAdapter`, e.g. a tool tip), so for RIM the leader's
origin is shifted onto it and the proxy target is shifted back by ``tool_correction``
(end-effector minus model interface point, in interface coordinates) before it is sent.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from haptic_teleop import HapticStateMailbox, ModelMailbox, StatusMailbox, clamp_force

__all__ = ["PROXY_METHODS", "PlantLoop", "PlantLoopConfig"]

PROXY_METHODS = ("proxy-rim", "proxy-fixed-mass")


@dataclass(kw_only=True)
class PlantLoopConfig:
    """Plant-side parameters.

    Args:
        method: rendering method name.
        plant_hz: plant loop rate: target streaming and model updates [Hz].
        feedforward_cap: clamp on the feedforward force sent to the robot (proxy methods) [N].
        tool_correction: ``(dim,)`` end-effector minus model interface point [m]; proxy-rim only.
    """

    method: str
    plant_hz: float = 50.0
    feedforward_cap: float = 15.0
    tool_correction: np.ndarray | None = None

    @property
    def proxy(self) -> bool:
        """True for the proxy methods (RIM, fixed-mass), which command the robot themselves."""
        return self.method in PROXY_METHODS


@dataclass
class _Shared:
    stop: bool = False
    stop_reason: str = ""
    model_updates: int = 0
    commands: list[tuple[float, np.ndarray, np.ndarray, np.ndarray]] = field(default_factory=list)


class PlantLoop:
    """One plant tick per call (``step(tick, t_s)``, from `run_loop`). See the module docstring.

    Args:
        cfg: plant-side parameters.
        plant: an [FR3Plant][fr3_haptic.plant.FR3Plant] (or anything with ``aim``, ``command``,
            ``freeze``).
        haptic_state: the haptic loop's leader / proxy state.
        status: the haptic loop's status (watchdog).
        dim: interface dimension.
        system: a ``pyrim.SystemInterface`` (proxy-rim only), updated every tick.
        model_out: where the model goes (proxy-rim only).
    """

    def __init__(
        self,
        cfg: PlantLoopConfig,
        *,
        plant,
        haptic_state: HapticStateMailbox,
        status: StatusMailbox,
        dim: int,
        system=None,
        model_out: ModelMailbox | None = None,
    ) -> None:
        if cfg.method == "proxy-rim" and (system is None or model_out is None):
            raise ValueError("proxy-rim needs a system (FR3System) and a model mailbox")
        self.cfg = cfg
        self.plant = plant
        self.haptic_state = haptic_state
        self.status = status
        self.system = system
        self.model_out = model_out
        self._tool_correction = np.zeros(dim) if cfg.tool_correction is None else np.asarray(cfg.tool_correction, dtype=float)
        self.shared = _Shared()

    @property
    def stopped(self) -> bool:
        """True once the plant side asked the run to end."""
        return self.shared.stop

    def request_stop(self, reason: str) -> None:
        if not self.shared.stop:
            self.shared.stop_reason = reason
        self.shared.stop = True

    def step(self, tick: int, t_s: float) -> None:
        """One plant tick: watchdog check, model update, then targets."""
        s = self.shared
        status = self.status.read()
        if status is not None and status["watchdog_tripped"]:
            self.plant.freeze()
            self.request_stop("plant samples stale (haptic watchdog)")
            return

        if self.system is not None:
            self.system.update()
            if self.system.ready:
                s.model_updates += 1
                self.model_out.write(self.system.model)

        state = self.haptic_state.read()
        if state is None:
            return
        if self.cfg.proxy:
            x, v = state["proxy_x"], state["proxy_v"]
            if np.isnan(x).any():
                return  # proxy not running yet: the controller holds its pose
            target = x + self._tool_correction
            f_ff = clamp_force(-state["force"], self.cfg.feedforward_cap)
            self.plant.command(target, v, f_ff)
            s.commands.append((t_s, target.copy(), v.copy(), f_ff))
        else:
            x_l, v_l = state["x_l"], state["v_l"]
            self.plant.aim(x_l, v_l)
            s.commands.append((t_s, x_l.copy(), v_l.copy(), np.zeros_like(x_l)))

    def shutdown_robot(self) -> None:
        """Freeze the robot at the end of the run (idempotent)."""
        self.plant.freeze()
