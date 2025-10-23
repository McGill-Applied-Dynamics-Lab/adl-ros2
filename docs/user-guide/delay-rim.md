# DelayRIM

Instruction on how to use the DelayRIM package

## Launching
1. Start Foxglove
```bash
foxglove # or without the alias: ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Real Robot
1. Start the server on the Franka PC
```
cd ...
docker compose up launch-franka
```

2. Start the I3 node
```
ros2 run inverse3_ros2 inverse3_node
# OR w/o force feedback
ros2 run inverse3_ros2 inverse3_node --ros-args --param disable_forces:=true
```

3. Launch delay rim
```bash
ros2 launch franka_rim franka_rim.launch.py config_file:=fr3_sim_config.yaml fake_i3:=false save_data:=false
```

### Sim
*To replay bag data*

1. Launch the DelayRIM nodes
```bash
ros2 launch franka_rim franka_rim.launch.py config_file:=fr3_sim_config.yaml fake_i3:=false save_data:=false
```

2. Replay the data
```
python src/robot_arm/franka_rim/scripts/replay_analysis.py <file_id>
```


## Nodes
TODO: THIS SHOULD COME FROM THE FILES
### `franka_model_node`
To compute the model of the franka

Input: state of the robot

Output: /fr3_model [FrankaModel]

### `franka_rim_node`
File: 

To compute the RIM of the Model message

Input: /fr3_model
Out: /fr3_rim

### `delay_rim_node`
Compute the target_pose and interaction forces from the history of the I3 states.

Main Loops
- `_update`: Runs at `update_freq` param (default 1 KhZ). 
    - Calls:
        - `_pub_rim_interface_force`: Publish the last computed interface forces between estimated rim and I3
        - `_pub_rim_state`: Publish estimated RIM state 
- 

In: I3 state, rim
Out: target_pose, i3_wrench

