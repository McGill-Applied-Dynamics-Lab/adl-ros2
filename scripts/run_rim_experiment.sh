#!/bin/bash

# RIM Experiment Runner and Analyzer
# Usage: ./run_rim_experiment.sh [experiment_name] [options]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
DATA_DIR="$WORKSPACE_ROOT/data"
ANALYSIS_DIR="$DATA_DIR/analysis"

# Default values
EXPERIMENT_NAME="${1:-rim_experiment_$(date +%Y%m%d_%H%M%S)}"
SAVE_DATA="${2:-true}"
FAKE_I3="${3:-true}"
DURATION="${4:-60}"  # Default 60 seconds

echo "=== RIM Experiment Runner ==="
echo "Experiment name: $EXPERIMENT_NAME"
echo "Save data: $SAVE_DATA"
echo "Fake I3: $FAKE_I3"
echo "Duration: $DURATION seconds"
echo "Data directory: $DATA_DIR"
echo

# Create data and analysis directories
mkdir -p "$DATA_DIR"
mkdir -p "$ANALYSIS_DIR"

# Function to cleanup on exit
cleanup() {
    echo "Cleaning up..."
    # Kill launch file if it's still running
    pkill -f "franka_rim.launch.py" || true
    echo "Cleanup complete."
}

# Set trap for cleanup on script exit
trap cleanup EXIT

# Source ROS2 environment
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/../../install/setup.bash"

# Launch the experiment
echo "Starting RIM experiment..."
ros2 launch franka_rim franka_rim.launch.py \
    save_data:=$SAVE_DATA \
    fake_i3:=$FAKE_I3 &

LAUNCH_PID=$!
echo "Launch file started with PID: $LAUNCH_PID"

# Wait for specified duration
echo "Running experiment for $DURATION seconds..."
sleep "$DURATION"

# Stop the launch file
echo "Stopping experiment..."
kill $LAUNCH_PID
wait $LAUNCH_PID 2>/dev/null || true

# Find the most recent bag file
if [ "$SAVE_DATA" = "true" ]; then
    echo "Looking for recorded data..."
    LATEST_BAG=$(find "$DATA_DIR" -name "franka_rim_data_*" -type d | sort -r | head -n 1)
    
    if [ -n "$LATEST_BAG" ]; then
        echo "Found bag file: $LATEST_BAG"
        
        # Run analysis
        echo "Running data analysis..."
        cd "$SCRIPT_DIR"
        python3 analyze_rim_data.py "$LATEST_BAG" \
            --output-dir "$ANALYSIS_DIR/${EXPERIMENT_NAME}" \
            --topics /rim/interface_force /rim/interface_position /joint_states
        
        echo "Analysis complete. Results saved to: $ANALYSIS_DIR/${EXPERIMENT_NAME}"
        echo "Generated files:"
        ls -la "$ANALYSIS_DIR/${EXPERIMENT_NAME}/"
    else
        echo "Warning: No bag file found for analysis."
    fi
else
    echo "Data recording was disabled. No analysis performed."
fi

echo "Experiment complete!"
