# Signal Filtering in Franka Model Node

The `franka_model_node` includes configurable low-pass filtering for noisy joint measurements.

## Filtered Signals

The following signals can be filtered:
- `q` - Joint positions
- `q_dot` - Joint velocities  
- `q_ddot` - Joint accelerations
- `tau` - Measured joint torques
- `f_ext_robot` - External force estimates from robot
- `x_ee` - End-effector position
- `v_ee` - End-effector velocity

## Filter Parameters

The filter uses a simple first-order low-pass filter:

```
y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
```

Where:
- `alpha` = filter coefficient (0 < alpha ≤ 1)
- `x[k]` = new measurement
- `y[k]` = filtered output
- `y[k-1]` = previous filtered value

### Alpha Coefficient

The `alpha` parameter controls the filtering strength:

- **`alpha = 1.0`**: No filtering (raw measurements)
- **`alpha = 0.5`**: Moderate filtering (50% new, 50% old)
- **`alpha = 0.1`**: Heavy filtering (10% new, 90% old)

**Rule of thumb**: 
- Higher derivatives (velocities, accelerations) need more filtering → lower alpha
- Positions can use less filtering → higher alpha

### Time Constant Relationship

For a given update period `dt` and desired filter time constant `tau`:

```
alpha = dt / (dt + tau)
```

Example: At 1000 Hz (`dt = 0.001s`), `alpha = 0.1` gives `tau ≈ 9ms`

## Configuration

Configure filters in your YAML config file:

```yaml
franka_model_node:
  ros__parameters:
    filter:
      enabled: true
      alpha_q: 0.3        # Joint positions
      alpha_q_dot: 0.2    # Joint velocities
      alpha_q_ddot: 0.1   # Joint accelerations (most filtering)
      alpha_tau: 0.2      # Joint torques
      alpha_f_ext: 0.15   # External forces
      alpha_x_ee: 0.3     # End-effector position
      alpha_v_ee: 0.2     # End-effector velocity
```

### Runtime Parameter Updates

You can modify filter parameters at runtime without restarting the node:

```bash
# Change a single alpha parameter
ros2 param set /franka_model_node filter.alpha_q 0.5

# Enable/disable filtering
ros2 param set /franka_model_node filter.enabled false

# View current filter parameters
ros2 param list /franka_model_node | grep filter

# Get a specific parameter value
ros2 param get /franka_model_node filter.alpha_q_dot
```

**Note**: Parameter changes take effect immediately. The filter state is preserved when you change alpha values, so there's no discontinuity in the filtered output.

## Tuning Tips

1. **Start conservative**: Use lower alpha values (0.1-0.2) for safety
2. **Increase gradually**: If response is too sluggish, increase alpha
3. **Monitor performance**: Check that filtered signals don't lag behind actual motion
4. **Different rates**: Signals with higher noise need lower alpha values
5. **Disable for debugging**: Set `enabled: false` to see raw measurements

## Implementation Details

- Filtering is applied in `_robot_state_callback()` as measurements arrive
- Filters initialize on first measurement (no manual initialization needed)
- All downstream model computations use filtered values
- Commanded signals (`tau_d`, `tau_ext`) are NOT filtered (they're control inputs)

## When to Disable Filtering

Consider disabling filters when:
- Debugging measurement issues
- Comparing with ground truth data
- Your robot already has good sensor filtering
- You need maximum responsiveness (e.g., impact detection)
