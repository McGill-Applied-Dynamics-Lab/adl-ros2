"""Local Pinocchio model estimator running outside ROS callbacks."""

from __future__ import annotations

import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pinocchio as pin
from rclpy.node import Node
from arm_client.robot import Robot

from ..config import InterfaceConfig, ModelConfig
from ..filters import LowPassFilter
from ..rim_types import DynModel


_AXIS_TO_INDEX = {"x": 0, "y": 1, "z": 2}


class ModelEstimatorAdapter:
    """Compute dynamic model terms periodically from latest robot state."""

    def __init__(
        self,
        node: Node,
        robot: Robot,
        model_cfg: ModelConfig,
        interface_cfg: InterfaceConfig,
        update_rate_hz: float,
    ) -> None:
        self._node = node
        self._robot = robot
        self._update_rate_hz = update_rate_hz
        self._dt = 1.0 / update_rate_hz
        self._frame_name = model_cfg.ee_frame_name

        urdf_root = get_package_share_directory(model_cfg.urdf_package)
        urdf_path = os.path.join(urdf_root, model_cfg.urdf_relative_path)
        self._model = pin.buildModelFromUrdf(urdf_path)
        self._data = self._model.createData()
        self._ee_frame_id = self._model.getFrameId(self._frame_name)

        axis_idx = _AXIS_TO_INDEX[interface_cfg.axis]
        self._di = np.zeros((1, 6))
        self._di[0, axis_idx] = 1.0

        self._f_q = LowPassFilter(model_cfg.filter_alpha_q)
        self._f_dq = LowPassFilter(model_cfg.filter_alpha_q_dot)
        self._f_tau = LowPassFilter(model_cfg.filter_alpha_tau)

        self._latest: DynModel | None = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _run_loop(self) -> None:
        next_tick = time.perf_counter()
        while self._running:
            self._compute_once()
            next_tick += self._dt
            time.sleep(max(0.0, next_tick - time.perf_counter()))

    def _compute_once(self) -> None:
        try:
            q = self._f_q.update(self._robot.q)
            dq = self._f_dq.update(self._robot.dq)
            tau = self._f_tau.update(self._robot.tau)
        except RuntimeError:
            return

        pin.computeAllTerms(self._model, self._data, q, dq)
        pin.updateFramePlacements(self._model, self._data)

        j_ee = pin.computeFrameJacobian(self._model, self._data, q, self._ee_frame_id, pin.LOCAL_WORLD_ALIGNED)
        j_dot_ee = pin.frameJacobianTimeVariation(
            self._model, self._data, q, dq, self._ee_frame_id, pin.LOCAL_WORLD_ALIGNED
        )

        ai = self._di @ j_ee
        ai_dot = self._di @ j_dot_ee
        b_i = (ai_dot @ dq).reshape(1)

        x_ee = self._data.oMf[self._ee_frame_id].translation
        v_ee = j_ee[:3, :] @ dq

        axis = int(np.argmax(self._di[0, :3]))
        x_i = np.array([x_ee[axis]], dtype=float)
        v_i = np.array([v_ee[axis]], dtype=float)

        n = self._model.nv
        dyn = DynModel(
            n=n,
            m=1,
            q=q.copy(),
            q_dot=dq.copy(),
            x_i=x_i,
            v_i=v_i,
            M=self._data.M.copy(),
            c=(self._data.nle - self._data.g).copy(),
            J_i=ai.copy(),
            b_i=b_i.copy(),
            tau_ext=tau.copy(),
            stamp_s=time.time(),
        )

        with self._lock:
            self._latest = dyn

    def latest(self) -> DynModel | None:
        with self._lock:
            return self._latest
