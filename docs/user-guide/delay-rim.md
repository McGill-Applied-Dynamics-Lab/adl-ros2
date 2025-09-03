# DelayRIM

Instruction on how to use the DelayRIM package

## Launching
1. Start Foxglove
```bash
foxglove # or without the alias: ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Real Robot
1. Start the server on the Franka PC

2. Check everything is ok (TODO)
```bash
python examples/fr3_go_home.py
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