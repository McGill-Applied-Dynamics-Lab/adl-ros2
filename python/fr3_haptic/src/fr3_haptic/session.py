"""This process's side of a haptic-teleop run on the real FR3, split by direction.

The haptic loop runs in its own process (``haptic_teleop.HapticProcess``). What stays here is
the robot side, one loop per direction, each at its own rate:

**Robot → haptic (the robot's state).**

- Plant samples (``x_i``, ``v_i``, ``λ``): `FR3Plant`'s callbacks receive every 1 kHz controller
  tick and publish one per ``1 / --model-update-hz`` into a ``PlantMailbox``. No loop here.
- [RimModelLoop][]: for ``proxy-rim``, recomputes the dynamics model at ``--rim-update-hz``
  (at most the model update rate) and publishes it in a ``ModelMailbox``.

**Haptic → robot (the targets).**

- [CommandLoop][]: at ``--command-hz``, reads the haptic loop's ``HapticStateMailbox`` and
  streams the targets to ``osc_controller``: the leader state for the coupling methods, the
  proxy state and a feedforward force for the proxy methods. It also freezes the robot and ends
  the run when the haptic watchdog trips (plant samples went stale), read from its
  ``StatusMailbox``.

A delay on either link belongs at its entry: before a sample is published (robot → haptic) and
before a target is sent (haptic → robot).

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

__all__ = ["PROXY_METHODS", "CommandLoop", "CommandLoopConfig", "RimModelLoop"]

PROXY_METHODS = ("proxy-rim", "proxy-fixed-mass")


@dataclass(kw_only=True)
class CommandLoopConfig:
    """Haptic → robot parameters.

    Args:
        method: rendering method name.
        command_hz: target streaming rate [Hz].
        feedforward_cap: clamp on the feedforward force sent to the robot (proxy methods) [N].
        tool_correction: ``(dim,)`` end-effector minus model interface point [m]; proxy-rim only.
    """

    method: str
    command_hz: float = 50.0
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
    commands: list[tuple[float, np.ndarray, np.ndarray, np.ndarray]] = field(default_factory=list)


class CommandLoop:
    """One command tick per call (``step(tick, t_s)``, from `run_loop`). See the module docstring.

    Args:
        cfg: haptic → robot parameters.
        plant: an [FR3Plant][fr3_haptic.plant.FR3Plant] (or anything with ``aim``, ``command``,
            ``freeze``).
        haptic_state: the haptic loop's leader / proxy state.
        status: the haptic loop's status (watchdog).
        dim: interface dimension.
    """

    def __init__(
        self,
        cfg: CommandLoopConfig,
        *,
        plant,
        haptic_state: HapticStateMailbox,
        status: StatusMailbox,
        dim: int,
    ) -> None:
        self.cfg = cfg
        self.plant = plant
        self.haptic_state = haptic_state
        self.status = status
        self._tool_correction = np.zeros(dim) if cfg.tool_correction is None else np.asarray(cfg.tool_correction, dtype=float)
        self.shared = _Shared()

    @property
    def stopped(self) -> bool:
        """True once this side asked the run to end."""
        return self.shared.stop

    def request_stop(self, reason: str) -> None:
        if not self.shared.stop:
            self.shared.stop_reason = reason
        self.shared.stop = True

    def step(self, tick: int, t_s: float) -> None:
        """One command tick: watchdog check, then targets."""
        s = self.shared
        status = self.status.read()
        if status is not None and status["watchdog_tripped"]:
            self.plant.freeze()
            self.request_stop("plant samples stale (haptic watchdog)")
            return

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


class RimModelLoop:
    """One RIM model update per call (``step(tick, t_s)``, from `run_loop`, at ``--rim-update-hz``).

    Args:
        system: a ``pyrim.SystemInterface`` with a ``ready`` flag (`FR3System`).
        model_out: where the model goes, for the haptic process.
    """

    def __init__(self, system, model_out: ModelMailbox) -> None:
        self.system = system
        self.model_out = model_out
        self.updates = 0

    def step(self, tick: int, t_s: float) -> None:
        self.system.update()
        if self.system.ready:
            self.model_out.write(self.system.model)
            self.updates += 1
