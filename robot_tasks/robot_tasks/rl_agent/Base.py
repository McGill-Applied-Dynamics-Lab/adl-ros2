import rclpy
import rclpy.action
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# RL imports
import pinocchio as pin
import numpy as np
import torch
import gymnasium as gym
from pathlib import Path
import yaml
import copy
import abc


# ROS2 Communications
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist  # Example message types
from franka_msgs.msg import FrankaRobotState  # TODO: Update when processed in the interface

from robot_arm_interface.fr3_interface import GoalSource, ControlMode

from robot_arm_interface.utils import motion2rostwist, rostwist2motion, rospose2se3, se32rospose

# Action and services
from arm_interfaces.srv import SetControlMode, GetControlMode, SetGoalSource, GetGoalSource

from action_msgs.msg import GoalInfo, GoalStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup  # Recommended for Action Server

import time
import threading  # For potential locks if needed with MTE
import asyncio

from robot_tasks.rl_agent.Agents import load_ppo_agent, load_robomimic_agent, RobomimicAgentWrapper


class RlAgentNode(Node, abc.ABC):
    """
    Abstract base class for augmented RL agents in ROS2.

    This class provides the core structure for implementing RL agents
    that can be used in ROS2 environments. Specific tasks should be implemented
    by creating subclasses that override the abstract methods.
    """

    def __init__(
        self,
        node_name,
        action_type,
        action_server_name,
        agent_lib,
        default_models_dir,
        default_agent_dir,
        observation_dim,
        action_dim,
        default_ctrl_frequency=50.0,
    ):
        """
        Initialize the RL agent node.

        Args:
            node_name: Name of the ROS2 node
            action_type: Type of the ROS2 action for the action server
            action_server_name: Name of the action server
            agent_lib: Library of the agent (e.g., skrl, ...)
            default_models_dir: Directory to load models from
            default_agent_dir: Directory to load the default agent from
            agent_dir: Directory to load agent from
            observation_dim: Dimension of the observation space
            action_dim: Dimension of the action space
        """
        super().__init__(node_name)
        self.get_logger().info(f"{node_name} Deployment Node started.")

        # Action Server needs a lock for safe state transitions if using MTE
        self._lock = threading.Lock()
        # Goal handle storage - protected by the lock
        self._goal_handle = None

        #! --- Core Parameters ---
        self._action_type = action_type
        self._action_server_name = action_server_name
        self._agent_lib = agent_lib
        self._observation_dim = observation_dim
        self._action_dim = action_dim

        # Add task success/failure state tracking to the parent class
        self._is_task_success = False
        self._is_task_failure = False
        self._task_finished = False

        #! --- Parameters ---
        self.declare_parameter(
            "models_dir",
            str(default_models_dir),
            # descriptor="Directory to load models from (absolute path)"
        )
        self.declare_parameter(
            "agent_dir",
            str(default_agent_dir),
            # descriptor="Directory to load the default agent from (relative path to `models_dir`)",
        )
        self.declare_parameter("control_frequency", default_ctrl_frequency)  # Hz - Rate of the control loop
        self.declare_parameter("action_scale", 0.05)  # Optional scaling for actions

        self._models_dir: Path = Path(self.get_parameter("models_dir").get_parameter_value().string_value)
        self._agent_dir: Path = self._models_dir / Path(
            self.get_parameter("agent_dir").get_parameter_value().string_value
        )

        control_frequency = self.get_parameter("control_frequency").get_parameter_value().double_value
        self.action_scale = self.get_parameter("action_scale").get_parameter_value().double_value / 10

        #! --- Define Spaces ---
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(self._observation_dim,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(self._action_dim,), dtype=np.float32
        )  # Assuming normalized actions

        #! --- Load Agent ---
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        self._load_agent()

        #! --- ROS 2 Communication ---
        self.callback_group = ReentrantCallbackGroup()

        # Initialize ROS components
        self._init_subscribers()
        self._init_publishers()
        self._init_services()
        self._init_action_server()  # Added Action Server init

        #! --- Control Loop State ---
        self.latest_observation = None
        self._last_action = np.zeros(self._action_dim, dtype=np.float32)
        self._task_finished = False
        self._control_loop_active = False

        self._ctrl_mode = ControlMode.CART_VEL
        self._goal_src = GoalSource.TOPIC

        # --- Control Loop Timer ---
        self.timer_period = 1.0 / control_frequency
        self.control_timer = self.create_timer(
            self.timer_period, self.control_loop_callback, callback_group=self.callback_group
        )

        self.get_logger().info(f"Node initialized. Control loop frequency: {control_frequency} Hz.")

    def _load_agent(self):
        """Load the RL agent from the paths."""
        if self._agent_lib == "skrl":
            self.get_logger().info("Loading skrl agent")
            try:
                agent_name = "agent.pt"
                agent_abs_path = self._agent_dir / agent_name

                self.agent = load_ppo_agent(
                    agent_name=agent_name,
                    agent_dir=self._agent_dir,
                    observation_space=self.observation_space,
                    action_space=self.action_space,
                    device=self.device,
                )

                self.agent.load(agent_abs_path)
                self.agent.set_running_mode("eval")
                self.get_logger().info(f"Agent loaded successfully from {self._agent_dir}")

            except Exception as e:
                self.get_logger().error(f"Failed to load skrl agent from {self._agent_dir}: {e}")
                # Handle error appropriately - maybe shutdown node
                rclpy.shutdown()
                return

        elif self._agent_lib == "robomimic":
            self.get_logger().info("Loading robomimic agent")
            agent_name = "agent.pth"
            agent_abs_path = self._agent_dir / agent_name

            try:
                # Load the agent from the specified checkpoint path
                self.agent: RobomimicAgentWrapper = load_robomimic_agent(
                    checkpoint_path=agent_abs_path, device=self.device
                )
                # The agent is already in eval mode, but let's set it explicitly for consistency
                # self.agent.set_running_mode("eval")
                self.get_logger().info(f"Robomimic agent loaded successfully from {agent_abs_path}")

            except Exception as e:
                self.get_logger().error(f"Failed to load robomimic agent from {agent_abs_path}: {e}")
                # Handle error appropriately - maybe shutdown node
                rclpy.shutdown()
                return
        else:
            self.get_logger().error(f"Unsupported agent library: {self._agent_lib}")
            self.get_logger().error("Supported options are: 'skrl', 'robomimic'")
            rclpy.shutdown()
            return

    @abc.abstractmethod
    def _init_subscribers(self):
        """
        Initialize subscribers for robot and env states.
        Must be implemented by child classes.
        """
        pass

    @abc.abstractmethod
    def _init_publishers(self):
        """
        Initialize publishers for robot commands.
        Must be implemented by child classes.
        """
        pass

    def _init_services(self):
        """
        Initialize service clients and servers
        """
        # **** Service client ****
        self._fr3_int_set_goal_src = self.create_client(
            SetGoalSource, "/fr3_interface/set_goal_source", callback_group=self.callback_group
        )
        self._fr3_int_set_ctrl_mode = self.create_client(
            SetControlMode, "/fr3_interface/set_control_mode", callback_group=self.callback_group
        )

        # Check if the control mode service is available (non-blocking)
        while not self._fr3_int_set_goal_src.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service '/fr3_interface/set_goal_source' not available, waiting...")

        while not self._fr3_int_set_ctrl_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service '/fr3_interface/set_control_mode' not available, waiting...")

        self.get_logger().info("Connected to services.")

    def _init_action_server(self):
        """
        Initialize the Action Server.
        """
        self._action_server = ActionServer(
            self,
            self._action_type,
            self._action_server_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,  # Use reentrant group
        )
        self.get_logger().info(f"Action server '{self._action_server_name}' started.")

    #! --- MARK: Action Server Callbacks ---
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server only allows one goal at a time
        with self._lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().warn("Action already in progress. Rejecting new goal.")
                return GoalResponse.REJECT

        self.get_logger().info("Received goal request. Accepting.")

        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        """Start execution of a newly accepted goal."""
        with self._lock:
            # Abort previous goal if it somehow wasn't cleaned up yet
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().warn("Unexpected: Aborting previous active goal.")
                self._goal_handle.abort()

            self._goal_handle = goal_handle

        # Start the execution thread/logic
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request.")
        # Always accept cancellations
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """Executes the action goal."""
        self.get_logger().info("Executing goal...")

        # --- 1. Set Control Mode and Goal Source ---
        self._task_finished = False
        self._is_task_success = False
        self._is_task_failure = False
        self._control_loop_active = False

        mode_set_ok = await self._set_robot_mode_async(control_mode=self._ctrl_mode, goal_source=self._goal_src)

        if not mode_set_ok:
            self.get_logger().error("Failed to set robot modes for RL Agent. Aborting goal.")
            result = self._create_result(success=False, message="Failed to set robot control mode or goal source.")
            goal_handle.abort()
            self._cleanup_goal(goal_handle=None, lock_handle=True)
            return result

        # Custom initialization for the task
        self.init_task_execution()

        # --- 2. Start Control Loop (Implicitly via `_control_loop_active`) ---
        self.get_logger().info("Robot modes set. RL control loop is now active for this goal.")

        self._last_action = np.zeros(self._action_dim, dtype=np.float32)
        self._control_loop_active = True

        # --- 3. Monitor for Completion or Cancellation ---
        result = self._create_result()
        while rclpy.ok():
            with self._lock:
                if not goal_handle.is_active:
                    self.get_logger().info("Goal is no longer active (completed or aborted externally).")
                    self._control_loop_active = False

                    # Result should have been set by succeed/abort call
                    # Ensure cleanup happens if it wasn't already
                    await self._deactivate_and_reset_mode_async()

                    self._cleanup_goal(goal_handle, lock_handle=False)  # Pass handle for safety
                    self.get_logger().info("Action completed")
                    return result

                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Goal cancel requested.")
                    self._control_loop_active = False
                    await self._deactivate_and_reset_mode_async()

                    result = self._create_result(success=False, message="Task cancelled.")
                    try:
                        goal_handle.canceled()
                    except Exception as e:
                        self.get_logger().warn(f"Error setting goal to canceled: {e}")
                    self._cleanup_goal(goal_handle, lock_handle=False)

                    self.get_logger().info("Action completed")
                    return result

                if self._task_finished:
                    self.get_logger().info("Task finished.")
                    self._control_loop_active = False

                    # Get task result
                    success, message = self.get_task_result()

                    result = self._create_result(success=success, message=message)

                    try:
                        if success:
                            goal_handle.succeed()
                        else:
                            goal_handle.abort()
                    except Exception as e:
                        self.get_logger().warn(f"Error setting goal state: {e}")

                    await self._deactivate_and_reset_mode_async()

                    self._cleanup_goal(goal_handle, lock_handle=False)

                    self.get_logger().info("Action completed")
                    return result

            # Allow other callbacks to run
            time.sleep(0.1)

        # If rclpy is shut down
        self.get_logger().warn("RCLPY shutdown during goal execution.")
        self._control_loop_active = False
        try:
            await self._deactivate_and_reset_mode_async()
        except Exception as e:
            self.get_logger().warn(f"Error during shutdown cleanup: {e}")

        result = self._create_result(success=False, message="System shutdown during execution.")
        try:
            # Try to set the goal state to aborted if possible
            if goal_handle.is_active:
                goal_handle.abort()
        except Exception:
            # We're shutting down, so errors are expected here
            pass

        self._cleanup_goal(goal_handle, lock_handle=True)
        return result

    @abc.abstractmethod
    def _create_result(self, success=False, message=""):
        """
        Create a result message for the action server.
        Must be implemented by child classes.
        """
        pass

    def _cleanup_goal(self, goal_handle=None, lock_handle=False):
        """Reset goal handle, ensuring lock safety."""
        self.get_logger().info("Cleaning up goal handle...")

        def reset_goal_handle():
            # Only reset if the provided handle matches the current one, or if forcing
            if goal_handle is None or self._goal_handle == goal_handle:
                self._goal_handle = None
                self._task_finished = True
                self.get_logger().debug("Cleaned up goal handle.")
            else:
                self.get_logger().warn("Cleanup requested for a non-current/mismatched goal handle.")

        try:
            if lock_handle:
                with self._lock:
                    reset_goal_handle()
            else:
                reset_goal_handle()

        except Exception as e:
            # Make sure we reset even if there's an error
            self.get_logger().warn(f"Error during goal cleanup: {e}")
            self._goal_handle = None
            self._task_finished = True

    #! --- MARK: RL Methods ---
    @torch.inference_mode()
    def _get_action(self) -> torch.Tensor:
        """
        Get the action from the agent based on the latest observation.
        """
        # timestep and timesteps are not used in the PPO agent (returns: action, log_prob, outputs)
        try:
            action, _, _ = self.agent.act(states=self.latest_observation, timestep=0, timesteps=0)

        except Exception as e:
            self.get_logger().error(f"Error while getting action from agent: {e}", throttle_duration_sec=2)
            return torch.zeros(self._action_dim, dtype=torch.float32)

        return action[0]

    @abc.abstractmethod
    def _get_observation(self) -> torch.Tensor:
        """
        Get the latest observation from the robot's sensors.
        Must be implemented by child classes.

        Returns:
            torch.Tensor: The observation tensor
        """
        pass

    @abc.abstractmethod
    def init_task_execution(self):
        """
        Initialize task-specific parameters before executing the task.
        Called before starting the control loop.
        """
        pass

    @abc.abstractmethod
    def check_task_status(self):
        """
        Check if the task is finished (success or failure).

        Returns:
            Tuple[bool, bool]: (is_success, is_failure)
        """
        pass

    def get_task_result(self):
        """
        Get the final result of the task execution.
        Default implementation based on task success/failure flags.
        Child classes can override this for custom messages.

        Returns:
            Tuple[bool, str]: (success, message)
        """
        if self._is_task_success:
            return True, "Task completed successfully."

        elif self._is_task_failure:
            return False, "Task failed."

        return False, "Task incomplete."

    @abc.abstractmethod
    def process_action(self, action):
        """
        Process the raw action from the agent.

        Args:
            action: The raw action from the agent

        Returns:
            The processed action to be sent to the robot
        """
        pass

    @abc.abstractmethod
    def send_robot_command(self):
        """
        Send the processed action to the robot.
        """
        pass

    #! --- MARK: Control Loop ---
    def control_loop_callback(self):
        """
        Main control loop that runs at the specified frequency.
        """
        #! Check if there is an active goal
        if not self._control_loop_active:
            self.get_logger().debug("Control loop inactive, control loop idle.", throttle_duration_sec=10)
            return

        with self._lock:
            if self._goal_handle is None or not self._goal_handle.is_active:
                self.get_logger().debug("No active goal handle, control loop idle.", throttle_duration_sec=10)
                return  # Do nothing if no goal is active

        if self._task_finished:
            return

        self.get_logger().debug("Control loop running...", throttle_duration_sec=2.0)
        # -- Get observations
        self.latest_observation = self._get_observation()
        if self.latest_observation is None:
            self.get_logger().warn("Control loop active, but observation is not ready.")
            return

        # -- Check if success/failure
        is_success, is_failure = self.check_task_status()

        # Update task state based on status check
        if is_success:
            self._is_task_success = True
            self._is_task_failure = False
            self._task_finished = True
            self.get_logger().info("Task completed successfully.")
            return

        if is_failure:
            self._is_task_success = False
            self._is_task_failure = True
            self._task_finished = True
            self.get_logger().info("Task failed.")
            return

        # -- Get Action from Agent
        action_tensor = self._get_action()
        action = action_tensor.cpu().numpy()  # Convert to numpy array for processing
        self._last_action = action  # Store last action for observation

        # -- Process and send action
        self.process_action(action)
        self.send_robot_command()

    #! --- MARK: Mode Setting ---
    async def _set_robot_mode_async(self, control_mode: ControlMode, goal_source: GoalSource):
        """Helper to asynchronously set control mode and goal source."""
        if not self._fr3_int_set_ctrl_mode.service_is_ready() or not self._fr3_int_set_goal_src.service_is_ready():
            self.get_logger().error("Control mode or goal source service not ready.")
            return False

        self.get_logger().info(f"Setting control mode and goal source to '{control_mode}' and '{goal_source}'.")
        ctrl_mode_req = SetControlMode.Request(control_mode=control_mode.value)
        goal_src_req = SetGoalSource.Request(goal_source=goal_source.value)

        try:
            # Call services concurrently
            ctrl_future = self._fr3_int_set_ctrl_mode.call_async(ctrl_mode_req)
            goal_future = self._fr3_int_set_goal_src.call_async(goal_src_req)

            # Wait for both futures to complete
            ctrl_response = await ctrl_future
            goal_response = await goal_future

            if ctrl_response.success and goal_response.success:
                self.get_logger().info(
                    f"Successfully set control mode to {control_mode} and goal source to {goal_source}."
                )
                return True
            else:
                self.get_logger().error(
                    f"Failed to set modes: ControlMode success={ctrl_response.success}, GoalSource success={goal_response.success}"
                )
                return False

        except Exception as e:
            self.get_logger().error(f"Exception while setting robot modes: {e}")
            return False

    async def _deactivate_and_reset_mode_async(self):
        """Asynchronously sends zero command and requests default modes."""
        self.get_logger().info("Deactivating agent and resetting control mode...")

        # Request default modes asynchronously
        try:
            # Only attempt service calls if ROS is still active
            if (
                rclpy.ok()
                and self._fr3_int_set_ctrl_mode.service_is_ready()
                and self._fr3_int_set_goal_src.service_is_ready()
            ):
                ctrl_mode_req = SetControlMode.Request(control_mode=ControlMode.PAUSE.value)
                # Actually await the call so we know it completes
                await self._fr3_int_set_ctrl_mode.call_async(ctrl_mode_req)
                self.get_logger().info("Successfully set control mode to `PAUSE`")
            else:
                self.get_logger().warn("Cannot reset modes: ROS context invalid or services not ready.")
        except Exception as e:
            # This likely means ROS is shutting down, we can only log and move on
            self.get_logger().warn(f"Error while deactivating agent: {e}. This is expected during shutdown.")

    # Helper method for converting between int and enum
    def _get_enum_from_value(self, enum_class, value):
        """
        Convert an integer value back to its enum constant.

        Args:
            enum_class: The enum class (e.g., ControlMode, GoalSource)
            value: The integer value

        Returns:
            The corresponding enum member or None if not found
        """
        try:
            # Method 1: Direct conversion if it's a proper Python Enum
            return enum_class(value)
        except (ValueError, TypeError):
            # Method 2: Manual lookup through members
            for member in enum_class:
                if member.value == value:
                    return member
            return None
