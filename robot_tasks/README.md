# robot_tasks
To execute tasks on the robot arm.

## To run
1. Start robot
2. Launch interface
3. Launch task

### Examples

## Docker Setup
To get cuda working: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/sample-workload.html
And the docker-compose options.

```
python3 -m pip install torch torchvision torchaudio
```

*Note*
To view container building log `docker logs franka_ros2`

## RL Agent
Node to load an skrl agent and send its actions.
It works based on actions -> an action call is required to start its control loop.

For the PegInHole agent:
```
ros2 action send_goal /insert_action /arm_interfaces/action/PegInHole "{}"
```

To run testing:
```

```

## TODO
- [ ] Fix torch installation in the Dockerfile

- [x] Load my agent
- [x] Subscribe to topics needed
- [x] Subscribe to services
- [ ] Trigger services
- [x] Process observations and send actions
- [ ] Integrate in `fr3_insert.py`

*see this chat https://gemini.google.com/app/e19525bb077db06c*