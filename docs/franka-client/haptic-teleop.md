# Haptic Teleoperation

`fr3_haptic` drives the real FR3 from a Haply Inverse3 using the rendering methods of
`haptic_teleop` (ZOH, linear, TDPA, RIM, fixed-mass) from
[adl-python](https://github.com/Applied-Dynamics-Lab/adl-python). The code is in
`python/fr3_haptic/`; the roadmap is in `python/fr3_haptic/PLAN.md`.

## Architecture

```mermaid
flowchart LR
    I3(["Inverse3"])

    subgraph CLIENT["Client PC: fr3_teleop (Python)"]
        H["Haptic loop<br/>1 kHz"]
        P["Plant loop<br/>50 to 1000 Hz"]
        S["Robot state<br/>1 kHz"]
    end

    subgraph SERVER["franka-pc (C++)"]
        OSC["osc_controller<br/>1 kHz"]
    end

    FR3(["FR3 arm"])

    I3 <-->|"position / force"| H
    H -->|"leader position"| P
    P -->|"targets"| OSC
    OSC <-->|"torques / state"| FR3
    OSC -->|"robot position<br/>+ coupling force"| S
    S -->|"latest sample"| H
```

| Loop | Rate | Runs on | Job |
|---|---|---|---|
| Haptic loop | 1 kHz | client PC | Reads the handle, runs the rendering method, writes the force back. Never goes through ROS. |
| Plant loop | 50 to 1000 Hz (`--plant-hz`) | client PC | Streams the targets to the robot. |
| Robot state | 1 kHz | client PC | Receives the robot's state and hands the latest sample to the haptic loop. |
| `osc_controller` | 1 kHz | franka-pc | Moves the robot toward the target through a spring-damper. |

The three client loops are threads of one Python process. The two client-server links are
ROS 2 topics over the network.

## What the robot is sent

The method family decides what the plant loop streams:

| Methods | Target sent to `osc_controller` | Who runs the coupling spring |
|---|---|---|
| `zoh`, `linear`, `tdpa-zoh`, `tdpa-linear` | the handle's position and velocity | `osc_controller`, with its interface-axis gains set to the coupling `kv`, `dv` |
| `proxy-rim`, `proxy-fixed-mass` | the proxy's position and velocity, plus a feedforward force | the proxy, simulated in the haptic loop at 1 kHz |

## Running

```bash
# Forces off: the arm follows the handle along z
pixi run -e humble fr3_teleop --conf python/fr3_haptic/configs/teleop.yaml

# With force feedback, another method, saving the run
pixi run -e humble fr3_teleop --conf python/fr3_haptic/configs/teleop.yaml --method linear --force --save
```

Needs franka-server running with `osc_controller`, the colcon overlay sourced, and Haply's Inlet
service. Every flag can be set in the YAML; `--force`, `--home` and `--save` are CLI-only.

## Logged data

With `--save`, each run writes one MCAP file (`samples.mcap`) and `metadata.json` (all flags,
controller gains, hold pose, git commit) under `--output-dir`. Streams:

| Stream | Rate | Content |
|---|---|---|
| `haptic` | 1 kHz | handle position/velocity on the interface axis, rendered force, plant age, energy, guard and watchdog state |
| `plant` | `--sample-hz` | robot interface position/velocity, coupling force, EE position, task force |
| `command` | `--plant-hz` | targets sent to the robot (position, velocity, feedforward force) |
| `robot_state` | `--robot-log-hz` (50) | every numeric field of `FrankaRobotState`: measured/desired joint state, motor state, external torque and wrench estimates, EE poses, inertias, errors, ... |
| `joint_torques_cmd`, `task_error`, `task_wrench`, `ee_state` | `--robot-log-hz` | `osc_controller` outputs |

Robot-side streams are recorded at a reduced rate on purpose: decoding one `FrankaRobotState`
in Python takes ~0.4 ms of the interpreter lock the 1 kHz haptic loop needs, so only the kept
messages are decoded. Timestamps of `plant` and the robot-side streams are local receive times
since the start of the run; the controller's own stamp is the `t_s` column.

!!! warning "Safety"
    - Forces are off unless `--force`, and fade in over the first second.
    - If the robot's state stops arriving for more than `plant_max_age_ms`, the handle force fades
      out, the robot holds its position, and the run ends.
    - The first target is the robot's current position, read from `osc_controller` itself.
