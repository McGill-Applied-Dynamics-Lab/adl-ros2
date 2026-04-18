# adl-ros2
Franka Client and ROS2 packages to perform experiments with a Franka Research 3. 

[![CI](https://github.com/McGill-Applied-Dynamics-Lab/adl-ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/McGill-Applied-Dynamics-Lab/adl-ros2/actions/workflows/ci.yml)
[![Documentation](https://github.com/McGill-Applied-Dynamics-Lab/adl-ros2/actions/workflows/docs.yml/badge.svg)](https://github.com/McGill-Applied-Dynamics-Lab/adl-ros2/actions/workflows/docs.yml)

Go to [ADL ROS2 Documentation](https://mcgill-applied-dynamics-lab.github.io/adl-ros2/).

---

## Adjustments for `acoustic_sensing` — notes for merge (Charles Sirois)

The following changes were made on the `updated_sim2real` branch to fix the colcon build inside the pixi environment and tidy up the `acoustic_sensing` package. Please review before merging.

### 1. `pixi.toml` — build task now runs through bash

**What changed:** the `build` task was rewritten from a command list to a `bash -c` string:

```toml
# Before
build = { cmd = ["colcon", "build", "--build-base", "build_$ROS_DISTRO", ...] }

# After
build = { cmd = """
bash -c 'colcon build \
  --build-base build_$ROS_DISTRO \
  --install-base install_$ROS_DISTRO \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug'
""" }
```

**Why:** `pixi run build` spawns colcon directly, so `BASH_ENV` (which points to
`install_humble/setup.bash`) is never sourced before colcon runs. Without it,
`AMENT_PREFIX_PATH` is not fully extended with the local workspace paths at the
moment colcon generates the cmake environment, causing every package that uses
`rosidl_typesupport_c` (i.e. `acoustic_sensing`, `arm_interfaces`, `franka_msgs`)
to fail with `No 'rosidl_typesupport_c' found`. Wrapping in `bash -c` triggers
`BASH_ENV` sourcing before colcon starts, so the workspace is visible to cmake.

### 2. `pixi.toml` — new dependency: `ros-humble-ament-cmake-clang-format`

Added to `[feature.humble.dependencies]`:

```toml
ros-humble-ament-cmake-clang-format = "*"
```

**Why:** `franka_gripper` and `franka_hardware` call `find_package(ament_cmake_clang_format)`
in their `CMakeLists.txt`. This package was not previously listed, causing those
two packages to fail in a clean build. It is available on the robostack-humble channel.

### 3. `pixi.toml` — `ROS_DOMAIN_ID` set in activation env

Added to `[activation.env]`:

```toml
ROS_DOMAIN_ID = "1"
```

**Why:** previously this had to be exported manually in every shell session. It is now
set automatically whenever the pixi environment activates.

### 4. `src/acoustic_sensing/CMakeLists.txt` — corrected install script paths

The two plotting scripts were moved from `scripts/` to `scripts/testing/` at some
point, but `CMakeLists.txt` was not updated. This caused the install step to fail
with a missing-file error. Fixed:

```cmake
# Before
scripts/acoustic_plot_receiver.py
scripts/acoustic_plot_2receiver.py

# After
scripts/testing/acoustic_plot_receiver.py
scripts/testing/acoustic_plot_2receiver.py
```

### 5. `rolling_contact_experiment_fr3pose.py` and `rolling_contact_experiment.py` — deferred trial directory creation

`_next_trial_dir()` no longer creates the directory immediately. The `mkdir` was
moved into `_save_trial_result()`, so the `trial_XX/` folder is only created on
disk once a complete roll finishes and the first `.pkl` is written. This prevents
empty trial directories from accumulating when a run is aborted before any data is
saved.

---

## Running the Acoustic-Sensing Experiments (for Wilfred)

This section is a quickstart for the two current experiment families under
`src/acoustic_sensing/scripts/`:

- `rolling_testing/` — slide a finray sensor along a cylinder and capture RF.
- `orientation_testing/` — press the sensor onto a bar held at various angles.

Each subfolder has its own detailed `README.md` — the notes below are the
minimum needed to get from a fresh clone to a running experiment.

### File map

`rolling_testing/pose/` — **position-controlled rolling** (Cartesian pose streaming)
| File | Role |
|------|------|
| `rolling_contact_experiment_fr3pose.py` | **Primary** headless runner using `fr3_pose_controller` (smooth quintic-interp motion) |
| `live_rolling_view.py` | GUI runner — same experiment with a live pyqtgraph viewer (RF, phase, EEF metrics) |
| `rolling_contact_experiment.py` | Legacy joint-trajectory version (kept for reference) |
| `rolling_config.py` | All tunables: speeds, compress depth, roller geometry, serial port, namespace |
| `read_rolling_results.py` | Replay a saved `.pkl` trial |

`rolling_testing/force/` — **force-controlled rolling** (adaptive PID / spline feedforward)
| File | Role |
|------|------|
| `rolling_contact_experiment_force.py` | Main runner — adaptive-PID force-controlled slide |
| `rolling_contact_experiment_spline.py` | Spline-feedforward slide (uses pre-learned Z(Y) profile) |
| `rolling_force_config.py` | All tunables for both variants |
| `build_depth_trajectory.py` | Build a Z(Y) spline from past force-control runs |
| `extract_depth_profile.py` | Extract the Z-depth profile from a result for spline fitting |
| `read_force_results.py` | Replay/visualise saved `.pkl` results |

`rolling_testing/rolling_testing_teensy_firmware/` — Teensy side
| File | Role |
|------|------|
| `rolling_testing_teensy_firmware.ino` | MCU firmware for RF streaming (responds to `0x43`/`0x45`) |
| `live_plot_send_67_stop_69.py` | Standalone host Qt live viewer for Teensy RF |

`orientation_testing/` — **press-hold-lift at angles**
| File | Role |
|------|------|
| `orientation_experiment.py` | Headless runner (terminal prompts) |
| `live_orientation_view.py` | GUI runner with live bar-angle diagram + RF + EEF |
| `orientation_config.py` | All tunables (angles, N_REPEATS, force/torque hold modes) |
| `read_orientation_results.py` | Replay saved `.pkl` results |

### Pick the right script for what you want to do

| Goal | Entry-point script | Config file |
|------|--------------------|-------------|
| Roll the sensor at constant speed, position-controlled, **no GUI** | `rolling_testing/pose/rolling_contact_experiment_fr3pose.py` | `rolling_testing/pose/rolling_config.py` |
| Same as above, **with live pyqtgraph GUI** (RF, phase, EEF metrics) | `rolling_testing/pose/live_rolling_view.py` | `rolling_testing/pose/rolling_config.py` |
| Roll with constant **contact force** (adaptive-PID) | `rolling_testing/force/rolling_contact_experiment_force.py` | `rolling_testing/force/rolling_force_config.py` |
| Roll along a pre-learned **Z(Y) depth spline** (spline feedforward) | `rolling_testing/force/rolling_contact_experiment_spline.py` | `rolling_testing/force/rolling_force_config.py` |
| Press-hold-lift at a fixed bar angle, **no GUI** | `orientation_testing/orientation_experiment.py` | `orientation_testing/orientation_config.py` |
| Same as above, **with live GUI** (bar-angle diagram + RF) | `orientation_testing/live_orientation_view.py` | `orientation_testing/orientation_config.py` |
| Replay a saved rolling `.pkl` (position-control) | `rolling_testing/pose/read_rolling_results.py` | — (CLI arg: path to `.pkl`) |
| Replay a saved rolling `.pkl` (force-control) | `rolling_testing/force/read_force_results.py` | — |
| Replay a saved orientation `.pkl` | `orientation_testing/read_orientation_results.py` | — |
| Build a Z(Y) spline from past force-control runs | `rolling_testing/force/build_depth_trajectory.py` | — |

### What each runner does (plain-language flow)

#### `rolling_contact_experiment_fr3pose.py` / `live_rolling_view.py`
Position-controlled rolling using the `fr3_pose_controller` (Cartesian pose streaming).
Per trial, at each speed in `ROLL_SPEEDS_M_S`:

1. Move above the cylinder to `approach_pose` (= `CONTACT_POSITION_M` + `APPROACH_HEIGHT_M` in +Z).
2. Descend to `CONTACT_POSITION_M`.
3. Start the Teensy RF stream (`0x43`), wait for the first RF frame.
4. Compress by `COMPRESS_DEPTH_M` in −Z.
5. Slide by `FINRAY_LENGTH_M * ROLL_LENGTH_PERCENT/100` in **+Y** at the trial speed.
6. Decompress back to surface in +Z.
7. Stop RF (`0x45`), save `.pkl` + PNG plots under `results/trial_NN/`.
8. Return to approach pose, wait for Enter / GUI "Continue" to start the next trial.

The GUI variant (`live_rolling_view.py`) is identical but with a pyqtgraph window; use `--dry-run` to exercise the motion without the Teensy.

#### `rolling_contact_experiment_force.py`
Adaptive-PID force control. Compress/decompress are still position-controlled, but the slide maintains a target Fz band (`FORCE_LOWER_N` ≤ Fz ≤ `FORCE_UPPER_N`) by nudging Z at `FORCE_CTRL_HZ` while Y advances at `SLIDE_SPEED_M_S`. The P gain is estimated online from the local fin-ray stiffness; Ki/Kd are fixed. The slide terminates on Y distance, time (`SLIDE_TIMEOUT_S`), or contact loss (`CONTACT_FORCE_THRESHOLD_N`).

#### `rolling_contact_experiment_spline.py`
Same motion plan, but Z during the slide is driven by a pre-learned `Z(Y)` spline (built by `build_depth_trajectory.py` from previous force-control runs). Deterministic, no feedback — good when you want repeatable Z profiles.

#### `orientation_experiment.py` / `live_orientation_view.py`
For each bar angle you enter at the terminal, runs `N_REPEATS` press-hold-lift cycles:

1. Move to `approach_pose` above the bar.
2. Descend to `CONTACT_POSITION_M`.
3. Compress by `COMPRESS_DEPTH_M`.
4. Hold for `HOLD_AT_CONTACT_SEC`. During the hold:
   - If `HOLD_FORCE_CONTROL` is `True`, a P-controller keeps Fz at the value snapshotted after settling.
   - If `HOLD_TORQUE_ADAPT` is `True`, Ry and Rz drift to null out wrist torques (auto-find the surface tilt). Rx (the 180° flip) is always locked.
5. Lift back to contact, then to approach.
6. Save `.pkl` under `results/trial_NN/`; loop to the next repeat.

### How to change parameters

All tuning happens in **the config file next to the runner**. You do **not** edit the runner itself for normal parameter changes. The parameter names below are listed in the order you're most likely to touch them.

#### Before every session (the ones you almost always change)

In `rolling_config.py` / `rolling_force_config.py` / `orientation_config.py`:

| Parameter | Meaning | Typical edit |
|-----------|---------|--------------|
| `CONTACT_POSITION_M` | World XYZ (m) where the sensor just touches the target. Jog the robot, read the EE position, paste here. | **Always** re-measure before a new session — the target moves between setups. |
| `SERIAL_PORT` | Teensy USB path. | `/dev/ttyACM0` or `/dev/ttyACM1` — check `ls /dev/ttyACM*`. |
| `DRY_RUN` | Skip serial / RF while still moving the robot. | Set `True` when debugging motion only. |
| `BASE_ORI_EULER_DEG` | Tool orientation at contact (xyz-Euler, degrees). | `[180,0,0]` for flat; small roll values (`[180,-2,0]`) align with a tilted surface. |
| `ROBOT_NAMESPACE` | ROS namespace the bringup publishes under. | Leave as `"fr3"` unless you changed the launch file. |

#### Rolling (position control) — `rolling_testing/pose/rolling_config.py`

| Parameter | What it changes |
|-----------|-----------------|
| `ROLL_SPEEDS_M_S` | List of slide speeds to sweep — one trial per entry. |
| `COMPRESS_DEPTH_M` | How hard to press the sensor before sliding (2–10 mm typical). |
| `FINRAY_LENGTH_M`, `ROLL_LENGTH_PERCENT` | Slide distance = their product. |
| `APPROACH_HEIGHT_M` | Clearance above the cylinder between trials. |
| `APPROACH_SPEED_M_S` / `RETURN_SPEED_M_S` | Speeds for descent/ascent and inter-trial repositioning. |
| `ROLLER_DIAMETER_M` | Used only for logging / post-processing; does not affect motion. |
| `SETTLE_SEC` | Dwell time between segments. |

#### Rolling (force control) — `rolling_testing/force/rolling_force_config.py`

| Parameter | What it changes |
|-----------|-----------------|
| `FORCE_UPPER_N`, `FORCE_LOWER_N` | Target Fz dead-band. Both negative (robot presses into surface). Narrow the band (< 1 N) for tight control, widen it to prevent oscillation. |
| `SLIDE_SPEED_M_S` | Constant Y advance rate during force control. |
| `KP_INITIAL`, `KP_MIN`, `KP_MAX` | Adaptive-P bounds (Kp = 1/k_est, clamped). Raise `KP_MAX` if the controller under-corrects on the compliant tip; lower `KP_MIN` if it over-corrects near the stiff base. |
| `KI_FORCE`, `KD_FORCE`, `KI_WINDUP_CLAMP_M` | Integral/derivative gains. Set both to `0` for pure adaptive-P. |
| `FORCE_ACQUIRE_*` | Pre-slide phase that presses slowly until `fz ≤ FORCE_UPPER_N`. Increase `FORCE_ACQUIRE_MAX_DEPTH_M` if the compress depth alone doesn't reach the target band. |
| `MAX_Z_CORRECTION_M`, `MAX_VZ_SPEED_M_S` | Safety clamps on Z-adjustment travel and speed. |
| `CONTACT_FORCE_THRESHOLD_N` | Abort slide if contact is lost (Fz > this). |
| `SLIDE_TIMEOUT_S` | Hard timeout. |
| `FORCE_CTRL_HZ` | Control-loop rate; 50 Hz is the default. |

#### Orientation — `orientation_testing/orientation_config.py`

| Parameter | What it changes |
|-----------|-----------------|
| `COMPRESS_DEPTH_M` | How deep to press each cycle. |
| `HOLD_AT_CONTACT_SEC` | RF capture duration per repeat. |
| `N_REPEATS` | Press-hold-lift cycles per bar angle. |
| `HOLD_FORCE_CONTROL` | `True` → P-control Fz during hold (keeps force from lifting the robot). |
| `HOLD_FORCE_KP`, `HOLD_FORCE_DEADBAND_N`, `HOLD_FORCE_MAX_Z_CORR_M` | Gains and safety clamp for the hold-force controller. |
| `HOLD_TORQUE_ADAPT` | `True` → let Ry, Rz drift to null wrist torques (auto-aligns to tilt). Rx stays at 180°. |
| `HOLD_TORQUE_KP_DEG_PER_NM`, `HOLD_TORQUE_DEADBAND_NM`, `HOLD_TORQUE_MAX_CORR_DEG` | Torque-compliance gains and clamps. Flip the sign of Kp if it tilts the wrong way. |
| `WAVEGUIDE_LATERAL_POSITIONS_MM`, `WAVEGUIDE_SAMPLE_PITCH_MM` | Geometric calibration — only touch these if the sensor geometry changes. |

### Full CLI workflow — fresh clone to running a trial

```bash
# 0. Prereqs: pixi installed, Franka robot reachable, Teensy on the expected /dev/tty port
cd /home/macrobotics/Documents/waveguide_gripper_project/shiyao_sensing/adl-ros2

# 1. Build the workspace (first run downloads ROS packages; ~5–15 min)
pixi run -e humble build
#    Writes to:  build_humble/   install_humble/
#    Uses bash -c internally so BASH_ENV sources install_humble/setup.bash
#    — required for rosidl_typesupport_c to be found.

# 2. Generate the VS Code env file so F5 debugging works
pixi run -e humble gen-vscode-env
#    Writes:  .vscode/.env.humble   (consumed by the
#             "Python Debugger: Current File (ROS2)" launch config)

# 3. Enter the pixi shell (this sources install_humble/setup.sh automatically
#    via [activation.scripts] in pixi.toml; also exports ROS_DOMAIN_ID=01)
pixi shell -e humble

# 4. (Optional) Verify activation worked
echo $ROS_DOMAIN_ID                            # should print 01
echo $AMENT_PREFIX_PATH | tr ':' '\n' | head   # should list install_humble/<pkg>/...
python -c "import arm_client; print(arm_client.CONFIG_DIR)"   # should print a path, no error

# 5. In a SEPARATE pixi shell, bring up the robot
pixi shell -e humble
ros2 launch franka_bringup franka.launch.py   # or whichever bringup matches your setup
#   Check topics: ros2 topic list

# 6. Back in the first shell, run an experiment
# --- Rolling, position control, headless ---
cd src/acoustic_sensing/scripts/rolling_testing/pose
python rolling_contact_experiment_fr3pose.py

# --- Rolling, position control, live GUI ---
python live_rolling_view.py                   # add --dry-run to skip the Teensy

# --- Rolling, force control ---
cd ../force
python rolling_contact_experiment_force.py    # or rolling_contact_experiment_spline.py

# --- Orientation, headless ---
cd ../../orientation_testing
python orientation_experiment.py

# --- Orientation, live GUI ---
python live_orientation_view.py
```

### Running from VS Code

1. Open the repo in VS Code (the `renan-r-santos.pixi-code` extension will pick up
   the pixi env automatically).
2. Make sure steps 1 and 2 above have been run at least once.
3. Open the Run and Debug panel (Ctrl+Shift+D).
4. Select **"Python Debugger: Current File (ROS2)"** from the dropdown — this
   is the only config that loads `.vscode/.env.humble`.
5. Open the script you want to run and press F5.

> ⚠️  The plain "▶ Run Python File" button (top-right of the editor) does **not**
> load the env file and will fail with `ImportError: cannot import name 'CONFIG_DIR'
> from 'arm_client' (unknown location)`.  Use the debug launch config instead.

### Troubleshooting: `ImportError: cannot import name 'CONFIG_DIR' from 'arm_client' (unknown location)`

Full symptom:

```
(adl-ros2:humble) $ /home/.../adl-ros2/.pixi/envs/humble/bin/python \
                   /home/.../rolling_contact_experiment_fr3pose.py
Traceback (most recent call last):
  File "rolling_contact_experiment_fr3pose.py", line 38, in <module>
    from arm_client import CONFIG_DIR
ImportError: cannot import name 'CONFIG_DIR' from 'arm_client' (unknown location)
```

**Cause:** You invoked the interpreter by its full path
(`/.pixi/envs/humble/bin/python ...`). Pixi's activation **does not run** in
that case, so `install_humble/setup.sh` is never sourced, `PYTHONPATH` does
not include `install_humble/arm_client/lib/python3.10/site-packages`, and
Python resolves `arm_client` to a stray namespace package with no
`CONFIG_DIR`. The same happens if `install_humble/` is missing because the
workspace hasn't been built.

**Fix — pick one of the three, they're equivalent:**

```bash
# Option A: drop into the pixi shell and just call `python`
pixi shell -e humble
python src/acoustic_sensing/scripts/rolling_testing/pose/rolling_contact_experiment_fr3pose.py

# Option B: let pixi wrap the call — no shell needed
pixi run -e humble python \
    src/acoustic_sensing/scripts/rolling_testing/pose/rolling_contact_experiment_fr3pose.py

# Option C: already inside a pixi shell but the error still shows?
# It means install_humble/setup.sh didn't exist when you entered the shell.
# Rebuild, then re-enter:
exit
pixi run -e humble build
pixi shell -e humble
python src/acoustic_sensing/scripts/rolling_testing/pose/rolling_contact_experiment_fr3pose.py
```

**Sanity check** before running the experiment:

```bash
echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep install_humble   # must return at least one line
python -c "import arm_client; print(arm_client.__file__)"      # must print a real path, not None
```

If both succeed, the `CONFIG_DIR` import will succeed too.

### Common gotchas

- **Don't invoke `.pixi/envs/humble/bin/python` directly.** It bypasses pixi
  activation, so `install_humble/setup.sh` never runs and
  `AMENT_PREFIX_PATH` / `PYTHONPATH` are not set — the `arm_client (unknown
  location)` error is the symptom.
- **Don't run bare `colcon build` inside `pixi shell -e humble`.** It writes
  to `build/`/`install/` (wrong dirs), and because `BASH_ENV` only fires for
  non-interactive bash, `rosidl_typesupport_c` is not discoverable → message
  packages fail to build. Always use `pixi run -e humble build`.
- **Teensy port:** change `SERIAL_PORT` in `rolling_config.py` /
  `rolling_force_config.py` / `orientation_config.py` to match your system
  (`/dev/ttyACM0` or `/dev/ttyUSB0`).
- **Results:** every experiment writes `trial_NN/` folders under a `results/`
  directory next to the script. Trial dirs are only created after the first
  `.pkl` is saved, so aborted runs don't leave empty folders.