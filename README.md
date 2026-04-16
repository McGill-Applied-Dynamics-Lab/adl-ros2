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