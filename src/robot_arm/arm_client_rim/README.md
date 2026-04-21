# arm_client_rim

RIM-based bilateral teleoperation package for FR3 using `arm_client`.

## What this package provides

- Single-script orchestration path (`rim_teleop`) instead of multi-node launch choreography.
- Clear separation between computation and ROS interfaces:
  - Compute: reduced model types, `RIMCalculator`, `DelayRIM` integrator.
  - ROS/device adapters: local Pinocchio model estimation from `arm_client` state and Haply I/O.
  - Orchestration layer: robot startup/controller switching and command streaming.
- Multi-rate runtime loops with built-in loop-rate/jitter monitoring logs.

## Run

```bash
pixi run -e humble ros2 run arm_client_rim rim_teleop --config src/robot_arm/arm_client_rim/config/rim_teleop_default.yaml
```

Optional CLI overrides still work and take precedence over the YAML file.

```bash
pixi run -e humble ros2 run arm_client_rim rim_teleop \
  --config src/robot_arm/arm_client_rim/config/rim_teleop_default.yaml \
  --axis z --control-rate 60
```

## Notes

- This implementation is additive and does not modify legacy `franka_rim` launch workflows.
- Initial scope is 1-axis RIM teleoperation with Haply force feedback.
- Local model estimation uses Pinocchio from `arm_client.Robot` state streams and runs outside ROS callbacks.
- Inverse3 reduced-axis interface logic/config is centralized in `arm_client/teleop/inverse3_teleop.py`.
