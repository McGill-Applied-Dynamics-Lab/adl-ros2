"""Single-script orchestrator for multi-rate RIM teleoperation."""

from __future__ import annotations

import threading
import time

import numpy as np
from arm_client.robot import Pose, Robot
from arm_client.teleop.inverse3_teleop import Inverse3Teleop

from .adapters import ModelEstimatorAdapter
from .config import RIMTeleopConfig
from .delay_rim import DelayRIM
from .monitoring import LoopRateMonitor
from .rim_compute import RIMCalculator
from .rim_types import DynModel


_AXIS_TO_INDEX = {"x": 0, "y": 1, "z": 2}


class RIMTeleopOrchestrator:
    """Coordinates adapters and multi-rate loops in one Python process."""

    def __init__(self, config: RIMTeleopConfig) -> None:
        self.cfg = config
        self._axis = _AXIS_TO_INDEX[config.interface.axis]

        self._robot = Robot(namespace=config.robot.namespace)
        self._home_pose: Pose | None = None

        self.model_adapter = ModelEstimatorAdapter(
            node=self._robot.node,
            robot=self._robot,
            model_cfg=config.model,
            interface_cfg=config.interface,
            update_rate_hz=config.rates.model_rate_hz,
        )

        self.haply = Inverse3Teleop(
            initial_robot_position=np.zeros(3),
            config=config.inverse3,
            center_on_start=False,
        )

        self.rim_calc = RIMCalculator()
        self.delay_rim = DelayRIM(
            interface_dim=1,
            dt=1.0 / config.rates.rim_rate_hz,
            stiffness=config.interface.stiffness,
            damping=config.interface.damping,
            contact_surface=config.interface.contact_surface,
        )

        self._running = False
        self._deadman_active = not self.cfg.safety.deadman_required
        self._threads: list[threading.Thread] = []
        self._loop_monitors = {
            "haptic": LoopRateMonitor(config.rates.haptic_rate_hz),
            "rim": LoopRateMonitor(config.rates.rim_rate_hz),
            "control": LoopRateMonitor(config.rates.control_rate_hz),
        }

    def start(self) -> None:
        # Initialize the robot
        self._robot.wait_until_ready()
        self._robot.controller_switcher_client.switch_controller(self.cfg.robot.controller_name)
        self._home_pose = self._robot.end_effector_pose.copy()

        self.haply.start()
        self.model_adapter.start()
        self._running = True
        self._threads = [
            threading.Thread(target=self._haptic_loop, daemon=True),
            threading.Thread(target=self._rim_loop, daemon=True),
            # threading.Thread(target=self._control_loop, daemon=True),
            threading.Thread(target=self._log_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._running = False
        for t in self._threads:
            if t.is_alive():
                t.join(timeout=2.0)
        if self.haply.is_connected():
            self.haply.set_interface_force(np.zeros(1, dtype=float))
        self.haply.stop()
        self.model_adapter.stop()
        self._robot.shutdown()

    @property
    def _home_orientation(self):
        if self._home_pose is None:
            return self._robot.end_effector_pose.orientation
        return self._home_pose.orientation

    def _send_axis_target(self, axis: int, target_axis_value: float) -> None:
        current_pose = self._robot.end_effector_pose
        target_position = current_pose.position.copy()
        target_position[axis] = target_axis_value

        traj_msg = self._robot.build_online_planning_step_trajectory(
            target_position=target_position,
            target_orientation=self._home_orientation,
            trajectory_length=self.cfg.robot.trajectory_length,
            dt=self.cfg.robot.trajectory_dt,
        )
        self._robot.send_joint_trajectory(traj_msg)

    def set_deadman(self, active: bool) -> None:
        """Set deadman state. When inactive, command and force outputs are gated off."""
        self._deadman_active = active

    def _is_robot_state_fresh(self, now: float) -> bool:
        stamps = self._robot.get_state_update_times()
        timeout = self.cfg.safety.stale_state_timeout_s

        joint_stamp = stamps["joint"]
        pose_stamp = stamps["pose"]
        if joint_stamp is None or pose_stamp is None:
            return False
        return (now - joint_stamp) < timeout and (now - pose_stamp) < timeout

    def _is_model_fresh(self, model: DynModel | None, now: float) -> bool:
        return model is not None and (now - model.stamp_s) < self.cfg.safety.stale_model_timeout_s

    def _safety_allows_output(self, now: float) -> bool:
        if self.cfg.safety.deadman_required and not self._deadman_active:
            return False
        if not self._is_robot_state_fresh(now):
            return False
        model = self.model_adapter.latest()
        if not self._is_model_fresh(model, now):
            return False
        return True

    def _haptic_loop(self) -> None:
        dt = 1.0 / self.cfg.rates.haptic_rate_hz
        next_tick = time.perf_counter()
        while self._running:
            leader_pos, leader_vel = self.haply.get_interface_state()
            self.delay_rim.add_leader_state(leader_pos, leader_vel)
            now = time.time()
            force = (
                self.delay_rim.get_interface_force() if self._safety_allows_output(now) else np.zeros(1, dtype=float)
            )
            self.haply.set_interface_force(force)
            self._loop_monitors["haptic"].tick()
            next_tick += dt
            time.sleep(max(0.0, next_tick - time.perf_counter()))

    def _rim_loop(self) -> None:
        dt = 1.0 / self.cfg.rates.rim_rate_hz
        next_tick = time.perf_counter()
        while self._running:
            self.delay_rim.step()
            self._loop_monitors["rim"].tick()
            next_tick += dt
            time.sleep(max(0.0, next_tick - time.perf_counter()))

    def _control_loop(self) -> None:
        dt = 1.0 / self.cfg.rates.control_rate_hz
        next_tick = time.perf_counter()
        while self._running:
            model = self.model_adapter.latest()
            now = time.time()
            if self._is_model_fresh(model, now):
                rim = self.rim_calc.compute(model)
                self.delay_rim.update_rim(rim)
                rim_x, _ = self.delay_rim.get_rim_state()
                if rim_x is not None and self._safety_allows_output(now):
                    self._send_axis_target(axis=self._axis, target_axis_value=float(rim_x[0]))
            self._loop_monitors["control"].tick()
            next_tick += dt
            time.sleep(max(0.0, next_tick - time.perf_counter()))

    def _log_loop(self) -> None:
        log_period = 5.0  # sec
        while self._running:
            summaries = []
            for name, monitor in self._loop_monitors.items():
                snap = monitor.snapshot()
                summaries.append(
                    f"{name}: {snap.measured_hz:.1f}Hz target={snap.target_hz:.1f}Hz p95={snap.p95_dt_ms:.2f}ms"
                )
            self._robot.node.get_logger().info(" | ".join(summaries), throttle_duration_sec=log_period)
