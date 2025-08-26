# Robot Tasks

Task execution framework with reinforcement learning capabilities for robot automation.

## Overview

The Robot Tasks package provides a comprehensive framework for defining, executing, and learning robotic tasks. It integrates reinforcement learning (RL) capabilities with traditional task execution, enabling robots to learn and improve their performance over time.

## Key Features

- **Task Framework**: Modular task definition and execution system
- **RL Integration**: Support for various reinforcement learning algorithms
- **Training Pipeline**: Complete training and evaluation infrastructure
- **Agent Management**: Trained agent deployment and monitoring
- **Mock Environment**: Testing and development without hardware

## Package Structure

```
robot_tasks/
├── robot_tasks/
│   ├── __init__.py
│   ├── RLAgent.py              # RL agent implementations
│   ├── actions_*.csv           # Training data and action logs
│   ├── mock/                   # Mock environment for testing
│   ├── obs_test/              # Observation testing utilities
│   └── rl_agent/              # RL algorithm implementations
├── agents/                     # Pre-trained agent models
│   ├── insert/                # Insertion task agents
│   └── lift/                  # Lifting task agents
├── scripts/                   # Executable scripts
│   ├── fr3_insert.py          # Insertion task script
│   └── ...                    # Other task scripts
├── rviz/                      # RViz visualization configs
└── test/                      # Unit tests
```

## Supported Tasks

### Lift Task
Pick and place operations with objects.

- **Objective**: Lift objects from surface to target height
- **Observations**: Object pose, gripper state, joint positions
- **Actions**: Joint velocities, gripper commands
- **Rewards**: Task completion, efficiency, safety

### Insert Task  
Precise insertion operations requiring fine motor control.

- **Objective**: Insert objects into tight-fitting receptacles
- **Observations**: Force/torque feedback, visual feedback, pose estimation
- **Actions**: Compliant motion commands
- **Rewards**: Successful insertion, minimal force, alignment

## RL Framework

### Supported Algorithms

- **PPO (Proximal Policy Optimization)**: Stable policy gradient method
- **SAC (Soft Actor-Critic)**: Sample-efficient off-policy algorithm
- **Custom Algorithms**: Framework for implementing new methods

### Training Environment

The package provides a standardized environment interface compatible with popular RL libraries:

```python
from robot_tasks.environments import LiftEnvironment

# Create training environment
env = LiftEnvironment(
    robot_interface=robot_arm_interface,
    task_config={
        'objects': ['cube', 'cylinder'],
        'target_height': 0.2,
        'success_threshold': 0.05
    }
)

# Train agent
from robot_tasks.rl_agent import PPOAgent
agent = PPOAgent(env)
agent.train(total_timesteps=100000)
```

## Quick Start

### Running Pre-trained Agents

```bash
# Launch task environment
ros2 launch robot_tasks lift_task.launch.py

# Run trained lifting agent
ros2 run robot_tasks run_trained_agent.py --task=lift --model=agents/lift/model_v2.pth

# Monitor performance
ros2 topic echo /task_manager/performance_metrics
```

### Training New Agents

```bash
# Start training environment
ros2 launch robot_tasks training_env.launch.py task:=lift

# Begin training (in separate terminal)
python3 src/robot_tasks/scripts/train_lift_agent.py --config=config/lift_training.yaml
```

### Mock Environment Testing

For development without hardware:

```bash
# Launch mock environment
ros2 launch robot_tasks mock_env.launch.py

# Test task execution
python3 src/robot_tasks/robot_tasks/mock/test_task.py
```

## Configuration

### Task Configuration

Tasks are configured via YAML files in the `config/` directory:

```yaml
# lift_task.yaml
task:
  name: "lift_object"
  type: "manipulation"
  
environment:
  objects:
    - type: "cube"
      size: [0.05, 0.05, 0.05]
      mass: 0.1
    - type: "cylinder"
      radius: 0.03
      height: 0.08
      mass: 0.15
      
  workspace:
    x_range: [0.3, 0.7]
    y_range: [-0.3, 0.3]
    z_range: [0.0, 0.5]
    
training:
  algorithm: "PPO"
  total_timesteps: 1000000
  learning_rate: 3e-4
  batch_size: 64
  
rewards:
  success: 100.0
  collision: -50.0
  timeout: -10.0
  distance_to_target: -1.0
```

### RL Algorithm Configuration

```yaml
# ppo_config.yaml
algorithm: "PPO"
hyperparameters:
  learning_rate: 3e-4
  n_steps: 2048
  batch_size: 64
  n_epochs: 10
  gamma: 0.99
  gae_lambda: 0.95
  clip_range: 0.2
  ent_coef: 0.01
  vf_coef: 0.5
```

## API Reference

### Core Classes

#### TaskExecutor
Main class for task execution and management.

```python
from robot_tasks.task_executor import TaskExecutor

executor = TaskExecutor()
success = executor.execute_task(task_name, task_config)
```

#### RLAgent
Base class for reinforcement learning agents.

```python
from robot_tasks.RLAgent import RLAgent

agent = RLAgent.load("path/to/model.pth")
action = agent.predict(observation)
```

### Environment Interface

All task environments implement a common interface:

```python
class TaskEnvironment:
    def reset(self) -> np.ndarray:
        """Reset environment and return initial observation."""
        pass
        
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:
        """Execute action and return (obs, reward, done, info)."""
        pass
        
    def render(self, mode: str = 'human') -> None:
        """Render the environment."""
        pass
```

## Data Collection

### Training Data Format

Training actions and observations are logged in CSV format:

```csv
timestamp,joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,joint_7,gripper,reward
1620000000.123,0.1,-0.5,0.8,-1.2,0.3,1.5,0.7,0.02,5.5
```

### Performance Metrics

```python
# Performance tracking
metrics = {
    'success_rate': 0.85,
    'average_reward': 45.3,
    'episode_length': 150.2,
    'collision_rate': 0.02
}
```

## Integration with Robot Arm

The package integrates seamlessly with the robot arm system:

```python
from robot_arm_interface import RobotArmInterface
from robot_tasks.environments import LiftEnvironment

# Connect to robot
robot = RobotArmInterface()
await robot.connect()

# Create task environment with real robot
env = LiftEnvironment(robot_interface=robot, use_real_robot=True)

# Execute trained policy
agent = RLAgent.load("trained_model.pth")
obs = env.reset()
done = False

while not done:
    action = agent.predict(obs)
    obs, reward, done, info = env.step(action)
```

## Monitoring and Visualization

### RViz Integration

Task execution can be visualized in RViz:

```bash
# Launch with visualization
ros2 launch robot_tasks lift_task.launch.py use_rviz:=true

# Load task-specific RViz config
rviz2 -d src/robot_tasks/rviz/lift_env.rviz
```

### Performance Dashboard

Monitor training progress and agent performance:

```bash
# Launch monitoring dashboard
ros2 run robot_tasks performance_monitor.py --port=8080

# View at http://localhost:8080
```

## Extending the Framework

### Adding New Tasks

1. **Define Task Environment**:
```python
from robot_tasks.base_environment import BaseTaskEnvironment

class MyTaskEnvironment(BaseTaskEnvironment):
    def __init__(self, config):
        super().__init__(config)
        # Task-specific initialization
        
    def _compute_reward(self, obs, action, next_obs):
        # Implement reward function
        pass
```

2. **Create Configuration**:
```yaml
# my_task_config.yaml
task:
  name: "my_task"
  observation_space: "continuous"
  action_space: "continuous"
```

3. **Register Task**:
```python
from robot_tasks.task_registry import register_task
register_task("my_task", MyTaskEnvironment)
```

## Testing

### Unit Tests

```bash
# Run all tests
colcon test --packages-select robot_tasks

# Run specific test
python3 -m pytest test/test_rl_agent.py -v
```

### Integration Tests

```bash
# Test with mock environment
python3 test/integration/test_mock_environment.py

# Test with simulation
ros2 launch robot_tasks test_integration.launch.py
```

## Performance Optimization

### Training Acceleration

- **GPU Support**: Automatic GPU detection and usage
- **Vectorized Environments**: Parallel environment execution
- **Experience Replay**: Efficient data reuse

### Deployment Optimization

- **Model Compression**: Reduce model size for deployment
- **Inference Optimization**: Fast action computation
- **Memory Management**: Efficient resource usage

## Troubleshooting

### Common Training Issues

**Training not converging**
- Check reward function design
- Adjust hyperparameters
- Verify environment reset logic

**High collision rates**
- Increase safety margins
- Improve observation space
- Add collision penalties

**Slow training**
- Use GPU if available
- Increase batch size
- Optimize environment code

## Contributing

When contributing to the robot tasks framework:

- Follow the task environment interface
- Include comprehensive tests
- Document reward function design
- Provide example configurations

See the [contributing guide](../developer-guide/contributing.md) for detailed guidelines.

## License

Licensed under the same terms as the main ADG ROS2 project.
