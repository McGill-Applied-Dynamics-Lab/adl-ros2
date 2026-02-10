# Experiments
How to save data and process experiments.

By *experiment*, we mean tests that aim at comparing different responses when the input is the same. 

## For a new Experiment

1. Run experiment with bag data recording
Example of launch file configuration
```python
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_filename = f"franka_rim_data_{timestamp}"
    bag_filepath = Path("data") / bag_filename  # Store in data directory

    # Define topics to record (add/remove as needed)
    topics_to_record = [
        "/fr3/current_pose",
        "/fr3_rim",
        ...
    ]
    bag_record_node = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            *topics_to_record,
            "--output",
            bag_filepath.as_posix(),
            "--storage",
            "sqlite3",
            "--max-bag-size",
            "0",  # No size limit
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("save_data")),
    )
```

2. Define your experiment configuration in `config/experiment_configs`
```yaml
my_experiment:
  experiments:  # List of bag files recorded for this experiment
  - franka_rim_data_20250911_145004
  output_filename: my_experiment_processed.pkl
  topics:  # For each value here, will extract the topic to a dataframe
    fr3_joint_states: /fr3/joint_states
    fr3_pose: /fr3/current_pose
    haptic_pose: /haptic_pose
    rim_force: /rim/interface_force
    rim_pose: /rim/pose
```

3. Run the processing script
This will extract a dataframe for all the experiments you specified
```bash
python scripts/bag_data_processing.py --config-file config/experiment_configs.yaml --config int_force_free_space
``` 

4. Plot and analyze your data
See the `scripts/rim_data_analysis` for an example on how to load the dataframes and plot them.

## Experiment python package
See issue #28

