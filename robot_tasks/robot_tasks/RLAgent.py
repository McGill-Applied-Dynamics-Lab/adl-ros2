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

# SKRL
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG

# from skrl.multi_agents.torch.ippo import IPPO, IPPO_DEFAULT_CONFIG
# from skrl.multi_agents.torch.mappo import MAPPO, MAPPO_DEFAULT_CONFIG
from skrl.resources.preprocessors.torch import RunningStandardScaler  # noqa
from skrl.resources.schedulers.torch import KLAdaptiveLR  # noqa
from skrl.utils.model_instantiators.torch import deterministic_model, gaussian_model, shared_model
from skrl.memories.torch import RandomMemory

# ROS2 Communications
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist  # Example message types
from franka_msgs.msg import FrankaRobotState  # TODO: Update when processed in the interface

from robot_arm_interface.fr3_interface import GoalSource, ControlMode

from robot_arm_interface.utils import motion2rostwist, rostwist2motion, rospose2se3, se32rospose

# Action and services
from arm_interfaces.action import PegInHole
from arm_interfaces.srv import SetControlMode, GetControlMode, SetGoalSource, GetGoalSource
from std_srvs.srv import Trigger

from action_msgs.msg import GoalInfo, GoalStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup  # Recommended for Action Server
from rclpy.executors import MultiThreadedExecutor

import time
import threading  # For potential locks if needed with MTE
import asyncio


def _process_cfg(cfg: dict) -> dict:
    """Convert simple types to skrl classes/components.

    Args:
        cfg: A configuration dictionary

    Returns:
        Updated dictionary
    """

    _direct_eval = [
        "learning_rate_scheduler",
        "shared_state_preprocessor",
        "state_preprocessor",
        "value_preprocessor",
    ]

    def reward_shaper_function(scale):
        def reward_shaper(rewards, *args, **kwargs):
            return rewards * scale

        return reward_shaper

    def update_dict(d):
        for key, value in list(d.items()):
            if isinstance(value, dict):
                update_dict(value)
            else:
                if key in _direct_eval:
                    if type(d[key]) is str:
                        d[key] = eval(value)
                elif key.endswith("_kwargs"):
                    d[key] = value if value is not None else {}
                elif key in ["rewards_shaper_scale"]:
                    d["rewards_shaper"] = reward_shaper_function(value)
        return d

    return update_dict(copy.deepcopy(cfg))


def load_ppo_agent(agent_name, agent_dir, observation_space, action_space, device="cpu"):
    """Load an SKRL PPO agent from a given directory.
    Args:
        agent_name (str): Name of the agent.
        agent_dir (Path): Path to the agent directory.
        observation_space (gym.Space): Observation space.
        action_space (gym.Space): Action space.
        device (str): Device to load the model on. Default is "cpu".
    """
    agent_path = agent_dir / agent_name
    print(f"Loading agent from {agent_path}")

    # Load cfg
    cfg_path = agent_dir / "params" / "agent.yaml"
    print(f"[INFO]: Parsing configuration from: {cfg_path}")
    with open(cfg_path, encoding="utf-8") as f:
        cfg = yaml.full_load(f)

    #! Create models
    models = {}

    models["policy"] = shared_model(
        observation_space=observation_space,
        action_space=action_space,
        device=device,
        structure=[cfg["models"]["policy"]["class"], cfg["models"]["value"]["class"]],
        roles=["policy", "value"],
        parameters=[
            _process_cfg(cfg["models"]["policy"]),
            _process_cfg(cfg["models"]["value"]),
        ],
    )
    models["value"] = models["policy"]

    #! Memory
    memory_class_mapping = {
        "RandomMemory": RandomMemory,
    }
    memory_class = memory_class_mapping[cfg["memory"]["class"]]
    del cfg["memory"]["class"]

    if cfg["memory"]["memory_size"] < 0:
        cfg["memory"]["memory_size"] = cfg["agent"]["rollouts"]

    memory = memory_class(num_envs=1, device=device, **_process_cfg(cfg["memory"]))

    #! Agent
    if cfg["agent"]["class"] != "PPO":
        raise NotImplementedError(f"Agent class '{cfg['agent']['class']}' not implemented")

    # Agent cfg
    agent_cfg = PPO_DEFAULT_CONFIG.copy()
    agent_cfg.update(_process_cfg(cfg["agent"]))
    agent_cfg["state_preprocessor_kwargs"].update({"size": observation_space, "device": device})
    agent_cfg["value_preprocessor_kwargs"].update({"size": 1, "device": device})

    agent = PPO(
        models=models,
        memory=memory,
        cfg=agent_cfg,
        observation_space=observation_space,
        action_space=action_space,
        device=device,
    )

    return agent


class AugRlAgent(Node, abc.ABC):
    """
    Abstract base class for augmented RL agents in ROS2.

    This class provides the core structure for implementing RL agents
    that can be used in ROS2 environments. Specific tasks should be implemented
    by creating subclasses that override the abstract methods.
    """

    def __init__(self, node_name, action_type, action_server_name, agent_dir, observation_dim, action_dim):
        """
        Initialize the RL agent node.

        Args:
            node_name: Name of the ROS2 node
            action_type: Type of the ROS2 action for the action server
            action_server_name: Name of the action server
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
        self._agent_dir = Path(agent_dir) if isinstance(agent_dir, str) else agent_dir
        self._observation_dim = observation_dim
        self._action_dim = action_dim

        # Add task success/failure state tracking to the parent class
        self._is_task_success = False
        self._is_task_failure = False
        self._task_finished = False

        #! --- Parameters ---
        self.declare_parameter("agent_checkpoint_path", str(self._agent_dir / "agent.pt"))
        self.declare_parameter("control_frequency", 50.0)  # Hz - Rate of the control loop
        self.declare_parameter("action_scale", 0.05)  # Optional scaling for actions

        # Get parameters
        self._agent_checkpoint_path: Path = Path(
            self.get_parameter("agent_checkpoint_path").get_parameter_value().string_value
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

        # --- Control Loop Timer ---
        self.timer_period = 1.0 / control_frequency
        self.control_timer = self.create_timer(
            self.timer_period, self.control_loop_callback, callback_group=self.callback_group
        )

        self.get_logger().info(f"Node initialized. Control loop frequency: {control_frequency} Hz.")

    def _load_agent(self):
        """Load the RL agent from the paths."""
        try:
            agent_name = self._agent_checkpoint_path.name  # Name of the agent file ex: `agent.pt`

            self.agent = load_ppo_agent(
                agent_name=agent_name,
                agent_dir=self._agent_dir,
                observation_space=self.observation_space,
                action_space=self.action_space,
                device=self.device,
            )

            self.agent.load(self._agent_checkpoint_path)
            self.agent.set_running_mode("eval")
            self.get_logger().info(f"Agent loaded successfully from {self._agent_dir}")

        except Exception as e:
            self.get_logger().error(f"Failed to load agent from {self._agent_dir}: {e}")
            # Handle error appropriately - maybe shutdown node
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

        mode_set_ok = await self._set_robot_mode_async(control_mode=ControlMode.CART_VEL, goal_source=GoalSource.TOPIC)

        if not mode_set_ok:
            self.get_logger().error("Failed to set robot modes for RL Agent. Aborting goal.")
            result = self._create_result(success=False, message="Failed to set robot control mode or goal source.")
            goal_handle.abort()
            self._cleanup_goal()
            return result

        # --- 2. Start Control Loop (Implicitly via goal_handle active state) ---
        self.get_logger().info("Robot modes set. RL control loop is now active for this goal.")
        # Reset last action
        self._last_action = np.zeros(self._action_dim, dtype=np.float32)

        # Custom initialization for the task
        self.init_task_execution()

        # --- 3. Monitor for Completion or Cancellation ---
        result = self._create_result()
        while rclpy.ok():
            with self._lock:
                if not goal_handle.is_active:
                    self.get_logger().info("Goal is no longer active (completed or aborted externally).")

                    # Result should have been set by succeed/abort call
                    # Ensure cleanup happens if it wasn't already
                    self._deactivate_and_reset_mode_async()

                    self._cleanup_goal(goal_handle)  # Pass handle for safety
                    self.get_logger().info("Action completed")
                    return result

                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Goal cancel requested.")
                    self._deactivate_and_reset_mode_async()

                    result = self._create_result(success=False, message="Task cancelled.")
                    goal_handle.canceled()
                    self._cleanup_goal(goal_handle)

                    self.get_logger().info("Action completed")
                    return result

                if self._task_finished:
                    self.get_logger().info("Task finished.")

                    # Get task result
                    success, message = self.get_task_result()

                    result = self._create_result(success=success, message=message)

                    if success:
                        goal_handle.succeed()

                    else:
                        goal_handle.abort()

                    self._deactivate_and_reset_mode_async()
                    self._cleanup_goal(goal_handle)
                    self.get_logger().info("Action completed")
                    return result

            # Allow other callbacks to run
            time.sleep(0.1)

        # If rclpy is shut down
        self.get_logger().warn("RCLPY shutdown during goal execution.")
        await self._deactivate_and_reset_mode_async()

        result = self._create_result(success=False, message="System shutdown during execution.")
        self._cleanup_goal(goal_handle)
        return result

    @abc.abstractmethod
    def _create_result(self, success=False, message=""):
        """
        Create a result message for the action server.
        Must be implemented by child classes.
        """
        pass

    def _cleanup_goal(self, goal_handle=None):
        """Reset goal handle, ensuring lock safety."""
        self.get_logger().info("Cleaning up goal handle...")

        # Only reset if the provided handle matches the current one, or if forcing
        if goal_handle is None or self._goal_handle == goal_handle:
            self._goal_handle = None
            self.get_logger().debug("Cleaned up goal handle.")
        else:
            self.get_logger().warn("Cleanup requested for a non-current/mismatched goal handle.")

    #! --- MARK: RL Methods ---
    @torch.inference_mode()
    def _get_action(self) -> torch.Tensor:
        """
        Get the action from the agent based on the latest observation.
        """
        # timestep and timesteps are not used in the PPO agent (returns: action, log_prob, outputs)
        action, _, _ = self.agent.act(states=self.latest_observation, timestep=0, timesteps=0)

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
        # Check if there is an active goal
        with self._lock:
            if self._goal_handle is None or not self._goal_handle.is_active:
                self.get_logger().debug("No active goal, control loop idle.", throttle_duration_sec=10)
                return  # Do nothing if no goal is active

        if self._task_finished:
            return

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

    def _deactivate_and_reset_mode_async(self):
        """Asynchronously sends zero command and requests default modes."""
        self.get_logger().info("Deactivating agent and resetting control mode...")

        # Request default modes asynchronously (fire and forget)
        if self._fr3_int_set_ctrl_mode.service_is_ready() and self._fr3_int_set_goal_src.service_is_ready():
            ctrl_mode_req = SetControlMode.Request(control_mode=ControlMode.PAUSE.value)
            # Fire and forget these calls
            self._fr3_int_set_ctrl_mode.call_async(ctrl_mode_req)
            self.get_logger().info("Requested to set control mode to `PAUSE`")
        else:
            self.get_logger().warn("Cannot reset modes: control mode or goal source service not ready.")

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


class Insert_AugRlAgentNode(AugRlAgent):
    """
    Implementation of the AugRlAgent for insertion tasks.
    """

    def __init__(self):
        # Constants
        self.SOCKET_POSE = np.array([0.6, 0.0, 0.0])
        self.Z_STOP = 0.015  # 1cm

        # pkg_dir = Path(__file__).parent.parent
        pkg_dir = Path(get_package_share_directory("robot_tasks"))
        agent_dir = pkg_dir / "agents" / "insert"

        # Set up and load environment config
        env_cfg_path = agent_dir / "params" / "env.yaml"
        print(f"[INFO]: Parsing configuration from: {env_cfg_path}")
        with open(env_cfg_path, encoding="utf-8") as f:
            self.env_cfg = yaml.full_load(f)

        # Initialize base class
        super().__init__(
            node_name="rl_agent_node",
            action_type=PegInHole,
            action_server_name="/insert_action",
            agent_dir=agent_dir,
            observation_dim=24,
            action_dim=3,
        )

        # Initialize state storage specific to insertion task
        self.default_joint_poses = [
            0.0,
            0.53249995,
            0.0,
            -2.02528006,
            0.0,
            2.55778002,
            0.78539816,
        ]  # Start position of the joints [rad]

        self._joint_state = None

        # Poses and twists
        self.X_GP = pin.SE3(
            pin.rpy.rpyToMatrix(np.array([0, 0, 0])), np.array([0, 0, 0.04])
        )  # Gripper to peg transform
        self.V_G = None  # Gripper velocity
        self.V_G_des = None  # Desired gripper velocity

        self.X_S = pin.SE3(pin.rpy.rpyToMatrix(np.array([0, 0, 0])), self.SOCKET_POSE)  # Socket position
        self.X_P = None  # Peg position
        self.X_G = None  # Gripper pose

    def _init_subscribers(self):
        """
        Initialize subscribers for robot and env states.
        """
        self.get_logger().info("Initializing subscribers...")

        # --- robot ---
        robot_topic = "/franka_robot_state_broadcaster/robot_state"
        self._robot_sub = self.create_subscription(FrankaRobotState, robot_topic, self._robot_state_callback, 10)

    def _init_publishers(self):
        """
        Initialize publishers for robot commands.
        """
        self.get_logger().info("Initializing publishers...")

        robot_cmd_topic = "/robot_arm/gripper_vel_command"
        self._robot_cmd_pub = self.create_publisher(
            TwistStamped,
            robot_cmd_topic,
            10,
        )

    def _create_result(self, success=False, message=""):
        """Create a PegInHole result message."""
        result = PegInHole.Result()
        result.success = success
        result.message = message
        return result

    def _robot_state_callback(self, robot_state_msg: FrankaRobotState):
        """
        Callback for the '/franka_robot_state_broadcaster/robot_state' topic.
        """
        self._joint_state = robot_state_msg.measured_joint_state
        self.X_G = rospose2se3(robot_state_msg.o_t_ee.pose)
        self.V_G = rostwist2motion(robot_state_msg.o_dp_ee_d.twist)

    def _get_observation(self) -> torch.Tensor:
        """
        Get the latest observation from the robot's sensors.
        """
        # Return if one observation component is not available
        if self._joint_state is None or self.X_GP is None or self.X_S is None:
            self.get_logger().warn("Missing some observations...", throttle_duration_sec=5)
            return None

        #! Process observations
        joint_pos_rel = np.array(self._joint_state.position) - self.default_joint_poses
        joint_pos_rel = np.concatenate([joint_pos_rel, np.array([0.0, 0.0])])  # Add fingers

        joint_vel_rel = np.array(self._joint_state.velocity)
        joint_vel_rel = np.concatenate([joint_vel_rel, np.array([0.0, 0.0])])  # Add fingers

        self.X_P = self.X_G * self.X_GP
        self.peg_error = self.X_P.translation - self.X_S.translation
        peg_pos_rel = self.peg_error

        last_action = self._last_action

        obs_list = [joint_pos_rel, joint_vel_rel, peg_pos_rel, last_action]

        try:
            observation = np.concatenate(obs_list).astype(np.float32)
            if observation.shape != (self._observation_dim,):
                self.get_logger().error(
                    f"Observation shape mismatch: expected ({self._observation_dim},), got {observation.shape}"
                )
                return None
        except ValueError as e:
            self.get_logger().error(f"Error concatenating observation: {e}")
            return None

        obs_tensor = torch.tensor(observation, dtype=torch.float32).to(self.device)
        return obs_tensor

    def init_task_execution(self):
        """Initialize task-specific parameters."""
        # Set motion parameters
        self.des_ee_speed = self.env_cfg["actions"]["arm_action"]["des_ee_speed"] / 10
        self.V_WG_ctrl = pin.Motion(np.array([0, 0, self.des_ee_speed]), np.zeros(3))  # Gripper velocity in world frame

    def check_task_status(self):
        """
        Check if insertion is successful or failed.

        Returns:
            Tuple[bool, bool]: (is_success, is_failure)
        """
        success = False
        failure = False
        z_thres = self.Z_STOP  # 1cm
        xy_thres = 0.03  # 3cm

        # Peg z reached the bottom
        if self.X_P.translation[2] < z_thres:
            # Check if x in range
            xy_err = np.linalg.norm(self.peg_error[:2])
            if xy_err < xy_thres:
                success = True
                failure = False
            else:
                success = False
                failure = True

        return success, failure

    def get_task_result(self):
        """Get the result of the insertion task with custom messages."""
        if self._is_task_success:
            return True, "Insertion succeeded."
        elif self._is_task_failure:
            return False, "Insertion failed."
        return False, "Insertion incomplete."

    def process_action(self, action):
        """Process the raw action from the agent."""
        # Scale the action
        processed_action = action * self.action_scale

        # Create motion from action
        self.V_G_agt = pin.Motion(
            np.array([processed_action[0], 0.0, processed_action[1]]),
            np.array([0.0, processed_action[2], 0.0]),
        )

        # Combine with control action
        self.V_G_des = self.V_WG_ctrl + self.V_G_agt

    def send_robot_command(self):
        """Send the command to the robot."""
        msg = TwistStamped()
        gripper_twist = motion2rostwist(self.V_G_des)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist = gripper_twist

        self._robot_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Insert_AugRlAgentNode()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()

        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt detected.")

        finally:
            node.get_logger().info("Shutting down executor...")
            executor.shutdown()
            node.destroy_node()

    except Exception as e:
        # Log any exceptions raised during node initialization
        print(f"Error during node initialization or shutdown: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
            print("RCLPY shutdown.")


if __name__ == "__main__":
    main()
