# Robot Controller Validation Experiments

This directory contains scripts for validating robot controller performance through systematic trajectory tracking experiments.

## Quick Start

Simply run the script with your desired parameters set at the top of the file:

```bash
python3 sin_traj.py
```

## Configuration

Edit the parameters at the top of `sin_traj.py` to customize your experiment:

```python
# ========================================
# EXPERIMENT PARAMETERS - MODIFY HERE
# ========================================

# Trajectory Parameters
START_POSITION = [0.4, 0.0, 0.4]  # [x, y, z] in meters
FREQUENCY = 50.0                   # Execution frequency in Hz
SIN_FREQ = 0.2                    # Sinusoidal frequency in Hz
AMPLITUDE = 0.1                   # Trajectory amplitude in meters
DURATION = 5.0                    # Experiment duration in seconds

# Robot Configuration
ROBOT_NAMESPACE = "fr3"           # Robot namespace
CONTROLLER = "haptic_controller"  # "haptic_controller" or "cartesian_impedance_controller"
CONTROLLER_CONFIG = None          # Path to custom config file (None for default)
CONNECTION_TIMEOUT = 2.0          # Robot connection timeout in seconds
HOME_ROBOT = True                 # Whether to home robot before experiment

# Experiment Settings
EXPERIMENT_NAME = None            # Custom name (None for auto-generated)
```

## Example Configurations

### 1. Default Test
```python
START_POSITION = [0.4, 0.0, 0.4]
FREQUENCY = 50.0
SIN_FREQ = 0.2
AMPLITUDE = 0.1
DURATION = 5.0
CONTROLLER = "haptic_controller"
```

### 2. High Precision Test
```python
START_POSITION = [0.4, 0.0, 0.4]
FREQUENCY = 100.0
SIN_FREQ = 0.5
AMPLITUDE = 0.02
DURATION = 8.0
CONTROLLER = "haptic_controller"
EXPERIMENT_NAME = "precision_test"
```

### 3. Large Motion Test
```python
START_POSITION = [0.4, 0.0, 0.4]
FREQUENCY = 30.0
SIN_FREQ = 0.1
AMPLITUDE = 0.15
DURATION = 15.0
CONTROLLER = "haptic_controller"
EXPERIMENT_NAME = "workspace_test"
```

### 4. Controller Comparison
```python
# Test 1: Haptic Controller
CONTROLLER = "haptic_controller"
EXPERIMENT_NAME = "haptic_test"

# Test 2: Cartesian Impedance Controller  
CONTROLLER = "cartesian_impedance_controller"
EXPERIMENT_NAME = "cartesian_test"
```

### 5. Stress Testing
```python
FREQUENCY = 200.0
SIN_FREQ = 1.0
AMPLITUDE = 0.08
DURATION = 2.0
EXPERIMENT_NAME = "stress_test"
```

## Output Structure

Each experiment creates a results directory: `experiments/benchmarks/results/<experiment_name>/`

```
results/
└── sin_traj_20250922_143022/
    ├── experiment_data.parquet      # Raw trajectory data
    ├── results.yaml                 # Metrics and metadata
    ├── position_tracking.png        # 4x1 position tracking plots
    ├── joint_torques.png           # 7x1 joint torque plots  
    ├── joint_positions.png         # 7x1 joint position plots
    └── joint_velocities.png        # 7x1 joint velocity plots
```

## Key Metrics Tracked

### Position Tracking Errors
- Mean, Max, RMS errors for X, Y, Z axes (in mm)
- 3D combined tracking error metrics

### Execution Performance  
- Mean execution frequency and consistency
- Timing accuracy and jitter analysis

### Controller Data
- Joint positions, velocities, torques (7 DOF)
- End-effector poses and orientations
- Controller parameter snapshots

## Tips for Effective Testing

1. **Start with defaults**: Use default parameters first to establish baseline performance
2. **Systematic variation**: Change one parameter at a time to isolate effects
3. **Multiple runs**: Run each configuration multiple times for statistical reliability
4. **Compare controllers**: Test the same trajectory with different controllers
5. **Document findings**: Use descriptive experiment names and keep notes

## Troubleshooting

### Common Issues
- **Dependencies missing**: Run `pip install pandas pyyaml pyarrow` if needed
- **Robot connection timeout**: Increase `CONNECTION_TIMEOUT` value or check robot status
- **Controller switch failed**: Ensure robot is homed and controllers are loaded properly

### Getting Help
The script will show the current configuration before running. Review the parameters at the top of `sin_traj.py` to ensure they match your intended experiment.