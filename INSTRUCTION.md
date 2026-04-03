# INSTRUCTION

This file collects the setup and runtime notes that were previously added to the root `README.md`, plus the full command flow for creating, building, and running the Pixi-based ROS 2 environment in this repository.

## What Was Updated For `franka_msgs`

`franka_msgs` is not available from the Pixi package registry used in this repo, so it must be provided from source.

What was done in this workspace:

1. Confirmed the runtime import failure came from code that imports `franka_msgs`, especially:
   - `src/arm_client/arm_client/gripper/franka_hand.py`
   - several `robot_tasks` and `franka_rim` nodes/tests
2. Confirmed that adding `ros-humble-franka-msgs` to `pixi.toml` does not work because that package is not published in the active Pixi/conda channels.
3. Added the upstream `franka_ros2` repository under `src/franka_ros2/`.
4. Verified that `src/franka_ros2/franka_msgs/` exists and is a normal ROS 2 package.
5. Updated package metadata so local Python/ROS packages that import `franka_msgs` declare it:
   - `src/arm_client/package.xml`
   - `src/robot_arm/franka_rim/package.xml`
   - `src/robot_arm/robot_arm_testing/package.xml`

## Pixi Environment

Install the Pixi environment for ROS 2 Humble:

```bash
pixi install -e humble
```

Enter the environment:

```bash
pixi shell -e humble
```

If you prefer not to use `pixi shell`, you can source the environment directly:

```bash
source /home/sni22/Documents/sim2real_adlros/.pixi/envs/humble/setup.sh
```

## Clean ROS Build Environment

Before building ROS packages in this workspace, clear stale ROS paths:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /home/sni22/Documents/sim2real_adlros/.pixi/envs/humble/setup.sh
```

## Build The Workspace

Build the whole workspace:

```bash
colcon build --base-paths src \
  --build-base build_humble \
  --install-base install_humble \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

Or use the Pixi task defined in `pixi.toml`:

```bash
pixi run -e humble build
```

After a successful build, source the workspace overlay:

```bash
source /home/sni22/Documents/sim2real_adlros/install_humble/setup.bash
```

## Build `franka_msgs` Specifically

To build only `franka_msgs` and the immediate client package that uses it:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /home/sni22/Documents/sim2real_adlros/.pixi/envs/humble/setup.sh
colcon build --base-paths src --packages-select franka_msgs arm_client \
  --build-base build_humble --install-base install_humble \
  --symlink-install --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug
source /home/sni22/Documents/sim2real_adlros/install_humble/setup.bash
```

## Verify `franka_msgs`

Check that the package exists in the source tree:

```bash
colcon list --base-paths src | grep franka_msgs
```

Check that Python can import the generated messages/actions after build:

```bash
python -c "from franka_msgs.action import Grasp, Homing, Move; print('franka_msgs ok')"
```

## Run Examples

General pattern:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /home/sni22/Documents/sim2real_adlros/.pixi/envs/humble/setup.sh
source /home/sni22/Documents/sim2real_adlros/install_humble/setup.bash
python /home/sni22/Documents/sim2real_adlros/examples/11_open_gripper.py
```

Example: open the gripper:

```bash
python /home/sni22/Documents/sim2real_adlros/examples/11_open_gripper.py
```

Example: rotate the robot with OSC PD:

```bash
python /home/sni22/Documents/sim2real_adlros/examples/01_rotate_head_90.py
```

## Notes

- If `colcon` says `ignoring unknown package 'franka_msgs'`, the source package is not present under `src/`.
- `arm_client` can build even when `franka_msgs` is missing, but runtime imports for the gripper client will still fail until `franka_msgs` is built and sourced.
- `franka_msgs` is currently handled as a source package in this workspace, not as a direct Pixi dependency.
