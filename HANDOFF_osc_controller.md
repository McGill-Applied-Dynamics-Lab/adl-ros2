# Handoff: osc_controller testing and arm_client improvements

From the `fr3-improvements` / `adl-python integration` sessions (2026-10-06). That session stays
on haptic teleoperation (`fr3_haptic`); this page is for the session working on the controller
and the client. Not committed anywhere — delete it once absorbed.

## Where things are

| What | Where | State |
|---|---|---|
| `osc_controller` stage 1 | franka-pc `~/workspaces/franka_ws/src/franka-server`, branch `feat/osc-controller`, commit `74ceede` | built, **not run on the robot**, **not pushed** |
| `/fr3/osc/ee_state` (Odometry, same tick/stamp as `task_wrench`) | same branch, commit `c63df93` | built, not pushed. Used by `fr3_haptic`; keep it |
| `Robot.set_target_streaming()` / `Robot.publish_target()` | adl-ros2 `feat/fr3-haptic` (`92835de`) | pushed |
| `ParametersClient.wait_until_ready` fix (`and` → `or`) | adl-ros2 `feat/fr3-haptic` (`dabd6a8`) | pushed |
| Zero initial target twist | adl-ros2 `d0e20ba` (on `feat-activation`, merged into `feat/fr3-haptic` as `ea90bc2`) | committed |
| **Uncommitted controller-testing edits** | worktree `adl-ros2.worktrees/fr3-haptic`: `robot.py` (adds `osc_controller_parameters_client`), `configs/controllers/osc/default.yaml` (k_pos 400 → 100), `examples/01_switch_controller.py`, `examples/03_figure_eight.py`, `parameters_client.py` (formatting only), `pixi.lock` | in progress |

franka-pc is one shared checkout with no worktree: check `git status` there before building.
Build: `cd ~/workspaces/franka_ws && source install/setup.sh && colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release' '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON' --packages-select fr3_controllers franka_server`.

**Bug in the uncommitted edits:** `examples/03_figure_eight.py` switches to `osc_controller` but
loads its parameters with `robot.osc_pd_controller_parameters_client`. It should use
`robot.osc_controller_parameters_client` (added in the same uncommitted `robot.py` edit).

## Priority 1 — `Robot`'s executor drops most high-rate messages (measured)

rclpy's `MultiThreadedExecutor`, which `Robot._spin_node` uses (`robot.py:282-285`),
delivered **~14 Hz of a 1 kHz best-effort topic** and **~164 Hz of a reliable one**. A
`SingleThreadedExecutor` in the same setup received the full **1000 Hz**. Measured on the client
PC, local DDS, isolated domain (`ROS_DOMAIN_ID=77`, `ROS_LOCALHOST_ONLY=1`), 1 kHz publisher of
`Odometry` + `WrenchStamped`.

Consequence: `Robot`'s own subscriptions (`joint_states`, `current_pose`, external wrench,
`FrankaRobotState`) are probably far below their publish rates. `robot.q`, `end_effector_pose`,
the wrench, state-freshness checks, `wait_until_ready`, any logging through `Robot`, and the
Pinocchio model used for RIM all see stale data. Tracking measurements made through `Robot`
are suspect until this is fixed.

`fr3_haptic.plant.FR3Plant` works around it with its own node on a `SingleThreadedExecutor`
thread (1000 Hz paired, gaps p99 1.1 ms). For `Robot`: switch `_spin_node` to a
`SingleThreadedExecutor`, or move the high-rate subscriptions to their own node and executor.
Re-measure with a counter per callback (first check: `ros2 topic hz` vs. callback count).

Minimal reproduction (two terminals, same env):

```python
# pub.py — 1 kHz publisher
import rclpy; from rclpy.node import Node; from nav_msgs.msg import Odometry
rclpy.init(); n = Node("pub"); p = n.create_publisher(Odometry, "/bench", 1)
n.create_timer(0.001, lambda: p.publish(Odometry())); rclpy.spin(n)
```

```python
# sub.py — count deliveries; swap the executor class to compare
import threading, time, rclpy; from rclpy.qos import qos_profile_sensor_data; from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
rclpy.init(); n = rclpy.create_node("sub"); c = [0]
n.create_subscription(Odometry, "/bench", lambda m: c.__setitem__(0, c[0] + 1), qos_profile_sensor_data)
ex = MultiThreadedExecutor(num_threads=4); ex.add_node(n); threading.Thread(target=ex.spin, daemon=True).start()
time.sleep(1); c0 = c[0]; time.sleep(3); print((c[0] - c0) / 3, "Hz")
```

## Priority 2 — test `osc_controller` stage 1 on the robot

Not yet run on hardware. Suggested sequence (hand on the stop):

1. Restart franka-server so the new plugin and `ee_state` are loaded; `ros2 topic hz /fr3/osc/ee_state` ≈ 1 kHz.
2. Home (JTC), switch to `osc_controller` with no targets streaming: holds pose, no jump; push lightly → springs back, elbow returns.
3. Load `configs/controllers/osc/default.yaml`; publish one target 2–3 cm away: smooth, no overshoot.
4. Figure eight (`examples/03_figure_eight.py`, fix above) on `osc_controller` vs `osc_pd_controller`; compare `/fr3/osc/task_error`.
5. If it vibrates: lower `gains.k_rot_*` first, then `control.partial_inertia_decoupling: true`.

Gain units: with `control.inertia_decoupling: true` (default) gains are in 1/s² and 1/s; with
`false`, N/m and N·s/m. The osc_pd YAMLs do not carry over.

## Priority 3 — stale targets on controller switch

`Robot` republishes `target_pose` (100 Hz), `target_joint`, `target_wrench`, `target_twist`
(`publish_frequency`) from timers (`robot.py:254-275`) whatever controller is active, and
`follow_joint_trajectory` / `send_joint_trajectory` / `execute_sequence` never update
`_target_pose`. Switching back to a Cartesian controller after a joint-trajectory move can snap
the arm to an old pose. `osc_controller` drops targets received while inactive, but the next
republish still delivers the stale one.

Fix: re-seed targets from the measured state on every controller switch and after every
trajectory (and/or stop republishing to inactive controllers). The streaming mode on
`feat/fr3-haptic` is the haptic-specific workaround: enabled before the switch.

## What `fr3_haptic` needs from `osc_controller` (keep stable)

- Per-axis gains settable at runtime; `fr3_teleop` sets `control.inertia_decoupling: false`,
  `gains.k_pos_<axis>` = coupling `kv`, `gains.d_pos_<axis>` = `dv` (N/m), `feedforward.twist: true`
  (damping is `D(v_target − v)`), `feedforward.wrench` per method. Base preset:
  `python/fr3_haptic/configs/osc_teleop.yaml` on `feat/fr3-haptic`.
- `ee_state` and `task_wrench` published every tick with the **same stamp** (the client pairs them by stamp).
- Topics: `/fr3/target_pose`, `/fr3/target_twist`, `/fr3/target_wrench`, `/fr3/osc/ee_state`, `/fr3/osc/task_wrench`.
- Things that make the coupling spring nonlinear, so keep them configurable: `limits.max_position_error`
  (default 5 cm caps the pull of far targets; teleop sets 0.3 m), `limits.force_limit`, `limits.delta_tau_max` (1 Nm/tick).
- `feedforward.timeout_ticks: 100` (100 ms) zeroes twist/accel/wrench: a target stream slower than
  10 Hz loses its feedforward between messages.

## osc_controller — other open items

- **End-effector frame:** the controller tracks libfranka `kEndEffector`; the client uses
  `fr3_hand_tcp` (IK, `current_pose`) and a Pinocchio tool-tip frame (RIM). No single source of truth.
- **Stage 2:** force/motion selection axes and frames; wrench feedback from `O_F_ext_hat_K`
  (low-pass + clamped integral).
- `franka_server/config/controllers.yaml:264`: `osc_pd_controller` gains are under `virtual_coupling:`
  but the parameters are `gains.*`, so those server-side values are silently ignored.
- `on_configure` sets the collision behaviour with a blocking service call.

## arm_client — remaining items from the review

API gaps:
- `move_joints(q, speed/duration)` blocking with velocity limits; `move_relative(dpos, drot, frame="base"|"tool")`;
  `wait_until_reached(tol, timeout)`; `Robot.switch_controller()` that also re-seeds targets; a gains helper
  mapping controller name → `ParametersClient`.
- Smooth time-scaled `move_to` (min-jerk) with angular speed; `config.ik_max_joint_velocity/acceleration` are unused.

Robustness:
- Every service/action future is waited on with a hot busy loop and no timeout
  (`controller_switcher.py`, `joint_trajectory_controller_client.py`); goal acceptance is never checked.
- IK planner reloads the URDF and re-JITs on every call (`planning/ik_pyroki.py`).

Bugs:
- `home()` duration truncated: `Duration(seconds=int(time_to_goal))` (`joint_trajectory_controller_client.py:95`).
- `set_target_joint(list)` crashes in the publish timer (no `np.asarray`, `robot.py:559`).
- Rotation-only `move_to` (`robot.py:1307`): zero distance → zero duration → jump (Cartesian) or error (joint).
- `_JOINT_CONTROLLER_KEYWORDS` includes `joint_space_controller` (`robot.py:110`), so `move_to` switches away from it.
- `execute_trajectory` was renamed `execute_cartesian_traj` (`robot.py:1369`); still called in
  `examples/07_follow_trajectory.py:116` and ~10 experiment scripts (`grep -rn "\.execute_trajectory(" examples experiments src`).
- External wrench sign flip races with readers (`_callback_current_wrench`, `robot.py:657`).
- `Pose.__add__`/`__sub__` are not inverses in the `(a - b) + b` order.

## Suggested order

1. Fix the `Robot` executor (everything that reads state depends on it).
2. Stage 1 hardware test of `osc_controller`.
3. Stale-target re-seeding on switch.
4. Convenience API and the bug list.

Coordinate on `robot.py`: `feat/fr3-haptic` already changes it (streaming mode). Merge that
branch first, or keep edits to `Robot` small and rebase.
