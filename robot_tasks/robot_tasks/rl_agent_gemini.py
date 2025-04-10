import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# RL imports
import numpy as np
import torch
import gymnasium as gym
from pathlib import Path
import yaml
import copy
import pinocchio as pin

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
from arm_interfaces.srv import SetControlMode, GetControlMode, SetGoalSource, GetGoalSource

# **** Action Server Imports ****
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup  # Recommended for Action Server
from rclpy.executors import MultiThreadedExecutor

# Assume you have defined PegInHole.action in custom_interfaces/action/
# Replace with your actual import
from custom_interfaces.action import PegInHole
import threading  # For potential locks if needed with MTE


SOCKET_POSE = np.array([0.6, 0.0, 0.0])


def _process_cfg(cfg: dict) -> dict:
    """Convert simple types to skrl classes/components

    :param cfg: A configuration dictionary

    :return: Updated dictionary
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


# Renamed class to RLAgent
class RLAgent(Node):
    def __init__(self):
        # Renamed node
        super().__init__("rl_agent")
        self.get_logger().info("RL Agent Deployment Node started.")

        # Action Server needs a lock for safe state transitions if using MTE
        self._lock = threading.Lock()
        # Goal handle storage - protected by the lock
        self._goal_handle = None

        #! --- Parameters ---
        self.declare_parameter(
            "agent_checkpoint_path", "/path/to/your/agent.pt"
        )  # IMPORTANT: Update this! # [cite: 12]
        self.declare_parameter("control_frequency", 50.0)  # Hz - Rate of the control loop # [cite: 13]
        self.declare_parameter("action_scale", 0.05)  # Optional scaling for actions # [cite: 13]
        self.declare_parameter("action_server_name", "/peg_in_hole")  # Name for the action server
        self.declare_parameter("set_control_mode_service", "/fr3_interface/set_control_mode")  # [cite: 31, 32]
        self.declare_parameter("set_goal_source_service", "/fr3_interface/set_goal_source")  # [cite: 31, 32]
        self.declare_parameter(
            "agent_control_mode", ControlMode.CART_VEL.value
        )  # Mode value for activation # [cite: 56]
        self.declare_parameter(
            "default_control_mode", ControlMode.JOINT_POS.value
        )  # Mode value for deactivation (Example: JOINT_POS)
        self.declare_parameter("agent_goal_source", GoalSource.TOPIC.value)  # Goal source for activation # [cite: 57]
        self.declare_parameter(
            "default_goal_source", GoalSource.JOINT_POS_CMD.value
        )  # Goal source for deactivation (Example: JOINT_POS_CMD)

        agent_path_param = self.get_parameter("agent_checkpoint_path").get_parameter_value().string_value
        control_frequency = self.get_parameter("control_frequency").get_parameter_value().double_value  # [cite: 13]
        self.action_scale = (
            self.get_parameter("action_scale").get_parameter_value().double_value / 10
        )  # [cite: 13] # NOTE: Scaling factor adjusted?
        action_server_name = self.get_parameter("action_server_name").get_parameter_value().string_value
        set_control_mode_service_name = (
            self.get_parameter("set_control_mode_service").get_parameter_value().string_value
        )  # [cite: 31, 32]
        set_goal_source_service_name = (
            self.get_parameter("set_goal_source_service").get_parameter_value().string_value
        )  # [cite: 31, 32]
        self.agent_control_mode = (
            self.get_parameter("agent_control_mode").get_parameter_value().integer_value
        )  # [cite: 56]
        self.default_control_mode = self.get_parameter("default_control_mode").get_parameter_value().integer_value
        self.agent_goal_source = (
            self.get_parameter("agent_goal_source").get_parameter_value().integer_value
        )  # [cite: 57]
        self.default_goal_source = self.get_parameter("default_goal_source").get_parameter_value().integer_value

        #! --- Configs ---
        # Assuming this logic remains the same # [cite: 14, 15]
        pkg_dir = Path(__file__).parent.parent
        agent_dir = pkg_dir / "agents" / "insert"
        agent_cfg_path = agent_dir / "params" / "agent.yaml"
        with open(agent_cfg_path, encoding="utf-8") as f:
            agent_cfg = yaml.full_load(f)
        env_cfg_path = agent_dir / "params" / "env.yaml"
        with open(env_cfg_path, encoding="utf-8") as f:
            env_cfg = yaml.full_load(f)

        #! --- Define Observation and Action Spaces ---
        self._observation_dim = 24  # [cite: 16]
        self._action_dim = 3  # [cite: 16]
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(self._observation_dim,), dtype=np.float32
        )  # [cite: 16]
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(self._action_dim,), dtype=np.float32
        )  # [cite: 16, 17]

        #! --- Load Agent ---
        self.device = "cuda" if torch.cuda.is_available() else "cpu"  # [cite: 18]
        self.get_logger().info(f"Using device: {self.device}")  # [cite: 18]
        agent_name = "agent.pt"  # [cite: 19]
        agent_path = agent_dir / agent_name  # [cite: 19]
        self.agent: PPO = load_ppo_agent(  # [cite: 19]
            agent_name=agent_name,
            agent_dir=agent_dir,
            observation_space=self.observation_space,
            action_space=self.action_space,
            device=self.device,
        )
        try:
            self.agent.load(agent_path)  # [cite: 19]
            self.agent.set_running_mode("eval")  # [cite: 20]
            self.get_logger().info(f"Agent loaded successfully from {agent_path}")  # [cite: 20]
        except Exception as e:
            self.get_logger().error(f"Failed to load agent from {agent_path}: {e}")  # [cite: 20]
            rclpy.shutdown()
            return

        #! --- ROS 2 Communication ---
        # Using ReentrantCallbackGroup for Action Server + Service Clients + Timer/Subs
        # This allows callbacks (like service done callbacks) to run even if the action execute callback is active
        self.callback_group = ReentrantCallbackGroup()

        self._init_subscribers()
        self._init_publishers()
        self._init_services_clients()  # Renamed for clarity
        self._init_action_server()  # Added Action Server init

        #! --- State Storage ---
        self.default_joint_poses = np.array(
            [  # [cite: 22]
                0.0,
                0.53249995,
                0.0,
                -2.02528006,
                0.0,
                2.55778002,
                0.78539816,
            ]
        )

        self.latest_observation = None

        self._joint_state: JointState = None  # [cite: 23]
        self.X_GP: pin.SE3 = pin.SE3(  # [cite: 23]
            pin.rpy.rpyToMatrix(np.array([0, 0, 0])), np.array([0, 0, 0.04])
        )
        self.V_G: pin.Motion = None  # [cite: 24]
        self.V_G_des: pin.Motion = pin.Motion.Zero()  # Initialize to zero # [cite: 24]

        self.X_S: pin.SE3 = pin.SE3(  # [cite: 24]
            pin.rpy.rpyToMatrix(np.array([0, 0, 0])), SOCKET_POSE
        )
        self.X_P: pin.SE3 = None  # [cite: 24]
        self.X_G: pin.SE3 = None  # [cite: 25]
        self.peg_error = np.zeros(3)  # Initialize # [cite: 36]

        self._last_action: np.array = np.zeros(self._action_dim, dtype=np.float32)  # [cite: 25]

        # ** RPL Params **
        self.des_ee_speed = env_cfg["actions"]["arm_action"]["des_ee_speed"] / 10  # [cite: 25]
        self.V_WG_ctrl: pin.Motion = pin.Motion(np.zeros(6))  # Initialize # [cite: 25]

        # **** Control Loop State ****
        # is_active is now implicitly managed by whether a goal is active
        # We don't need a separate self.is_active flag

        # --- Control Loop Timer ---
        self.timer_period = 1.0 / control_frequency  # [cite: 26]
        # Assign timer to the reentrant callback group
        self.control_timer = self.create_timer(
            self.timer_period, self.control_loop_callback, callback_group=self.callback_group
        )  # [cite: 26]

        self.get_logger().info(f"Node initialized. Action server '{action_server_name}' running.")  # [cite: 26, 27]

    def _init_subscribers(self):
        """
        Initialize subscribers for robot and env states.
        """
        self.get_logger().info("Initializing subscribers...")  # [cite: 28]
        robot_topic = "/franka_robot_state_broadcaster/robot_state"  # [cite: 28]
        # Assign subscriber to the reentrant callback group
        self._robot_sub = self.create_subscription(
            FrankaRobotState, robot_topic, self._robot_state_callback, 10, callback_group=self.callback_group
        )  # [cite: 28]
        # Add other subscribers if needed...

    def _init_publishers(self):
        """
        Initialize publishers for robot commands.
        """
        self.get_logger().info("Initializing publishers...")  # [cite: 30]
        robot_cmd_topic = "/robot_arm/gripper_vel_command"  # [cite: 30]
        self._robot_cmd_pub = self.create_publisher(TwistStamped, robot_cmd_topic, 10)  # [cite: 30]

    def _init_services_clients(self):  # Renamed from _init_services
        """
        Init services clients
        """
        self.get_logger().info("Initializing service clients...")  # [cite: 31]
        set_control_mode_service_name = (
            self.get_parameter("set_control_mode_service").get_parameter_value().string_value
        )
        set_goal_source_service_name = self.get_parameter("set_goal_source_service").get_parameter_value().string_value

        # Assign clients to the reentrant callback group
        self._fr3_int_set_ctrl_mode = self.create_client(
            SetControlMode, set_control_mode_service_name, callback_group=self.callback_group
        )  # [cite: 31]
        self._fr3_int_set_goal_src = self.create_client(
            SetGoalSource, set_goal_source_service_name, callback_group=self.callback_group
        )  # [cite: 31]

        # Wait for services in a non-blocking way (or handle connection later)
        if not self._fr3_int_set_goal_src.wait_for_service(timeout_sec=2.0):  # [cite: 32]
            self.get_logger().warn(f"Service '{set_goal_source_service_name}' not available.")
        if not self._fr3_int_set_ctrl_mode.wait_for_service(timeout_sec=2.0):  # [cite: 32]
            self.get_logger().warn(f"Service '{set_control_mode_service_name}' not available.")
        self.get_logger().info("Service clients initialized.")  # [cite: 32]

    def _init_action_server(self):
        """
        Initialize the Action Server.
        """
        action_server_name = self.get_parameter("action_server_name").get_parameter_value().string_value
        self._action_server = ActionServer(
            self,
            PegInHole,  # Replace with your actual action type
            action_server_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,  # Use reentrant group
        )
        self.get_logger().info(f"Action server '{action_server_name}' started.")

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

    def handle_accepted_callback(self, goal_handle):
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

    async def execute_callback(self, goal_handle):
        """Executes the action goal."""
        self.get_logger().info("Executing goal...")

        # --- 1. Set Control Mode and Goal Source ---
        mode_set_ok = await self._set_robot_mode_async(self.agent_control_mode, self.agent_goal_source)

        if not mode_set_ok:
            self.get_logger().error("Failed to set robot modes for RL Agent. Aborting goal.")
            result = PegInHole.Result(success=False, message="Failed to set robot control mode or goal source.")
            goal_handle.abort(result)
            self._cleanup_goal()
            return result

        # --- 2. Start Control Loop (Implicitly via goal_handle active state) ---
        self.get_logger().info("Robot modes set. RL control loop is now active for this goal.")
        # Reset last action maybe?
        self._last_action = np.zeros(self._action_dim, dtype=np.float32)

        # --- 3. Monitor for Completion or Cancellation ---
        result = PegInHole.Result()
        while rclpy.ok():
            with self._lock:
                if not goal_handle.is_active:
                    self.get_logger().info("Goal is no longer active (completed or aborted externally).")
                    # Result should have been set by succeed/abort call
                    # Ensure cleanup happens if it wasn't already
                    await self._deactivate_and_reset_mode()
                    self._cleanup_goal(goal_handle)  # Pass handle for safety
                    # Return the result that was hopefully set
                    # Need a way to get the result set by succeed/abort?
                    # Let's return a default failure if we reach here unexpectedly
                    return PegInHole.Result(success=False, message="Goal became inactive unexpectedly.")

                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Goal cancel requested.")
                    await self._deactivate_and_reset_mode()
                    result.success = False
                    result.message = "Task cancelled."
                    goal_handle.canceled(result)
                    self._cleanup_goal(goal_handle)
                    return result

            # Check success/failure conditions (done within control_loop_callback now)
            # Publish feedback (optional - can be done in control_loop_callback)
            # feedback_msg = PegInHole.Feedback()
            # feedback_msg.distance_to_target = np.linalg.norm(self.peg_error)
            # goal_handle.publish_feedback(feedback_msg)

            # Allow other callbacks to run
            await asyncio.sleep(0.1)  # Use asyncio sleep

        # If rclpy is shut down
        self.get_logger().warn("RCLPY shutdown during goal execution.")
        await self._deactivate_and_reset_mode()
        result.success = False
        result.message = "System shutdown during execution."
        # Cannot set goal status if node is shutting down
        # goal_handle.abort(result) # Might fail
        self._cleanup_goal(goal_handle)
        return result

    def _cleanup_goal(self, goal_handle=None):
        """Reset goal handle, ensuring lock safety."""
        with self._lock:
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
        if self.latest_observation is None:
            # Handle case where observation isn't ready yet
            return torch.zeros(self._action_dim, device=self.device)  # Return zero action

        # Ensure observation is on the correct device
        obs_tensor = self.latest_observation.to(self.device)
        # timestep and timesteps are not used in the PPO agent (returns: action, log_prob, outputs) # [cite: 33]
        action, _, _ = self.agent.act(states=obs_tensor, timestep=0, timesteps=0)  # [cite: 33]
        return action[0]  # Assuming non-batched

    def _get_observation(self) -> torch.Tensor:
        """
        Get the latest observation from the robot's sensors.
        """
        # Check prerequisites # [cite: 35]
        if self._joint_state is None or self.X_G is None or self.X_S is None:
            # self.get_logger().warn("Observation prerequisites not met.", throttle_duration_sec=5)
            return None  # Return None if not ready

        #! Process observations # [cite: 36]
        joint_pos_rel = np.array(self._joint_state.position) - self.default_joint_poses  # [cite: 36]
        # Fingers might not be part of default_joint_poses if length mismatches
        if len(joint_pos_rel) == 7 and len(self.default_joint_poses) == 7:  # Assuming 7 joints
            joint_pos_rel = np.concatenate([joint_pos_rel, np.array([0.0, 0.0])])  # Add dummy fingers # [cite: 36]
        else:
            # Handle potential mismatch or already correct length
            if len(joint_pos_rel) != 9:  # Assuming obs requires 9
                self.get_logger().error(f"Joint position length mismatch: expected 9, got {len(joint_pos_rel)}")
                return None

        joint_vel_rel = np.array(self._joint_state.velocity)  # [cite: 36]
        if len(joint_vel_rel) == 7:
            joint_vel_rel = np.concatenate([joint_vel_rel, np.array([0.0, 0.0])])  # Add dummy fingers # [cite: 36]
        elif len(joint_vel_rel) != 9:
            self.get_logger().error(f"Joint velocity length mismatch: expected 9, got {len(joint_vel_rel)}")
            return None

        # Update peg pose relative to gripper # [cite: 36]
        self.X_P = self.X_G * self.X_GP  # [cite: 36]
        self.peg_error = self.X_P.translation - self.X_S.translation  # [cite: 36]
        peg_pos_rel = self.peg_error  # [cite: 36]
        # print(f"Peg pose: {self.X_P.translation[2]}\t\t{peg_pos_rel[2]}") # [cite: 37] # DEBUG

        last_action = self._last_action  # [cite: 37]

        obs_list = [joint_pos_rel, joint_vel_rel, peg_pos_rel, last_action]  # [cite: 37]

        try:
            observation = np.concatenate(obs_list).astype(np.float32)  # [cite: 37]
            if observation.shape != (self._observation_dim,):  # [cite: 37]
                self.get_logger().error(
                    f"Observation shape mismatch: expected ({self._observation_dim},), got {observation.shape}"
                )
                return None
        except ValueError as e:
            self.get_logger().error(f"Error concatenating observation: {e}")
            return None

        obs_tensor = torch.tensor(observation, dtype=torch.float32)  # Don't move to device here, do it in _get_action

        return obs_tensor

    # --- Sensor Callback ---
    def _robot_state_callback(self, robot_state_msg: FrankaRobotState):  # Corrected type hint # [cite: 38]
        """
        Callback for the '/franka_robot_state_broadcaster/robot_state' topic.
        """
        # NOTE: Accessing fields based on FrankaRobotState definition
        # Ensure these fields exist in your FrankaRobotState message definition
        # If using a different message, adjust accordingly.
        # Example field access (adjust based on actual message definition):
        if hasattr(robot_state_msg, "measured_joint_state"):
            self._joint_state = robot_state_msg.measured_joint_state  # [cite: 40]
        else:
            # Maybe joint state is directly in root message?
            # self._joint_state = robot_state_msg # Example if JointState fields are at root
            self.get_logger().warn_once("Cannot find 'measured_joint_state' in FrankaRobotState msg.")
            # Use default if needed for observation size?
            # self._joint_state = JointState(position=np.zeros(7), velocity=np.zeros(7))

        if hasattr(robot_state_msg, "o_t_ee"):  # Check if field exists
            # Assuming o_t_ee is a Pose type directly
            self.X_G = rospose2se3(robot_state_msg.o_t_ee)  # [cite: 40]
        elif hasattr(robot_state_msg, "pose"):  # Maybe pose is at root?
            self.X_G = rospose2se3(robot_state_msg.pose)
        else:
            self.get_logger().warn_once("Cannot find pose ('o_t_ee' or 'pose') in FrankaRobotState msg.")

        if hasattr(robot_state_msg, "o_dp_ee_d"):  # Check if field exists
            # Assuming o_dp_ee_d is a Twist type directly
            self.V_G = rostwist2motion(robot_state_msg.o_dp_ee_d)  # [cite: 40]
        elif hasattr(robot_state_msg, "twist"):  # Maybe twist is at root?
            self.V_G = rostwist2motion(robot_state_msg.twist)
        else:
            self.get_logger().warn_once("Cannot find twist ('o_dp_ee_d' or 'twist') in FrankaRobotState msg.")
            self.V_G = pin.Motion.Zero()  # Set to zero if not found

        # --- Update Observation after processing state ---
        # This ensures self.latest_observation is updated whenever new state comes in
        self.latest_observation = self._get_observation()

    #! --- MARK: Control Loop ---
    def control_loop_callback(self):
        # Check if there is an active goal
        with self._lock:
            if self._goal_handle is None or not self._goal_handle.is_active:
                # self.get_logger().debug("No active goal, control loop idle.", throttle_duration_sec=10)
                return  # Do nothing if no goal is active

            # Get a reference to the current goal handle inside the lock
            goal_handle = self._goal_handle

        # --- Get Observation ---
        # Observation is updated in the _robot_state_callback now
        if self.latest_observation is None:
            self.get_logger().warn("Control loop active, but observation is not ready.", throttle_duration_sec=5)
            return

        # --- Check if success/failure ---
        success, failure = self.check_insertion_status()  # [cite: 41]
        task_finished = False
        result = PegInHole.Result()

        if success:  # [cite: 41]
            self.get_logger().info("Insertion success!")  # [cite: 41]
            result.success = True
            result.message = "Insertion successful."
            goal_handle.succeed(result)
            task_finished = True

        elif failure:  # [cite: 42]
            self.get_logger().info("Insertion failed!")  # [cite: 42]
            result.success = False
            result.message = "Insertion failed (e.g., missed hole)."
            goal_handle.abort(result)  # Use abort for failure condition
            task_finished = True

        if task_finished:
            # Deactivate and cleanup outside the lock if possible, or call async helper
            self._deactivate_and_reset_mode_async()  # Fire and forget deactivation
            self._cleanup_goal(goal_handle)
            return  # Stop further processing in this loop iteration

        # --- Get Action from Agent ---
        action_tensor: torch.tensor = self._get_action()  # [cite: 42]
        action = action_tensor.cpu().numpy()  # Convert to numpy array # [cite: 42]
        self._last_action = action  # Store last action for observation

        # --- Postprocess Action ---
        processed_action = action * self.action_scale  # [cite: 43]
        # processed_action = self.clip_action(processed_action) # [cite: 43]

        # --- Combine w/ ctrl action (or interpret action directly) ---
        # Assuming action represents velocity commands directly
        # Modify this based on how your agent's actions map to robot commands
        self.V_G_agt = pin.Motion(  # [cite: 43]
            np.array(
                [processed_action[0], 0.0, processed_action[1]]
            ),  # Mapping action[0]->vx, action[1]->vz ? # [cite: 44]
            np.array([0.0, processed_action[2], 0.0]),  # Mapping action[2]->wy ? # [cite: 44]
        )

        # Check if applying a base velocity is still desired
        # self.V_WG_ctrl: pin.Motion = pin.Motion(np.array([0, 0, self.des_ee_speed]), np.zeros(3)) # [cite: 44]
        # self.V_G_des = self.V_WG_ctrl + self.V_G_agt # [cite: 44]
        self.V_G_des = self.V_G_agt  # Use agent action directly

        # --- Send Action to Robot ---
        self.send_robot_command()  # [cite: 45]

    def clip_action(self, action):
        # Implement safety clipping...
        # self.get_logger().warn("Action clipping not implemented yet!", throttle_duration_sec=10) # [cite: 46]
        return action  # [cite: 46]

    def send_robot_command(self):
        """
        Convert the desired gripper twist to ROS 2 message format and publish it.
        """
        msg: TwistStamped = TwistStamped()  # [cite: 47]
        gripper_twist: Twist = motion2rostwist(self.V_G_des)  # [cite: 47]
        msg.header.stamp = self.get_clock().now().to_msg()  # [cite: 47]
        msg.twist = gripper_twist  # [cite: 47]

        self._robot_cmd_pub.publish(msg)  # [cite: 47]

    def check_insertion_status(self):
        """
        Check if insertion is successful or failed based on peg pose relative to socket.
        """
        if self.X_P is None:  # Ensure peg pose is calculated
            return False, False

        success = False  # [cite: 48]
        failure = False  # [cite: 48]
        z_thres = 0.01  # 1cm threshold below socket origin # [cite: 48]
        xy_thres = 0.03  # 3cm radial threshold # [cite: 48]

        # Check Z position relative to socket Z (assuming socket origin is target)
        # self.peg_error[2] = self.X_P.translation[2] - self.X_S.translation[2]
        if self.X_P.translation[2] < (
            self.X_S.translation[2] + z_thres
        ):  # Check if below threshold relative to socket Z # [cite: 48]
            # Check XY error relative to socket XY
            xy_err = np.linalg.norm(self.peg_error[:2])  # [cite: 48, 49]
            if xy_err < xy_thres:  # [cite: 49]
                success = True  # [cite: 49]
            else:
                failure = True  # [cite: 49, 50] # Failed because it's deep enough but misaligned

        # Optional: Add failure condition if forces are too high?

        return success, failure  # [cite: 50]

    # --- Mode Setting and Deactivation ---

    async def _set_robot_mode_async(self, control_mode_val, goal_source_val):
        """Helper to asynchronously set control mode and goal source."""
        if not self._fr3_int_set_ctrl_mode.service_is_ready() or not self._fr3_int_set_goal_src.service_is_ready():
            self.get_logger().error("Control mode or goal source service not ready.")
            return False

        ctrl_mode_req = SetControlMode.Request(control_mode=control_mode_val)  # [cite: 56]
        goal_src_req = SetGoalSource.Request(goal_source=goal_source_val)  # [cite: 57]

        try:
            # Call services concurrently
            ctrl_future = self._fr3_int_set_ctrl_mode.call_async(ctrl_mode_req)
            goal_future = self._fr3_int_set_goal_src.call_async(goal_src_req)

            # Wait for both futures to complete
            # Use asyncio.gather or similar if running in an async context,
            # otherwise, need rclpy mechanisms if called from sync context
            # For simplicity here, assuming called from async execute_callback
            ctrl_response = await ctrl_future
            goal_response = await goal_future

            if ctrl_response.success and goal_response.success:
                self.get_logger().info(
                    f"Successfully set control mode to {control_mode_val} and goal source to {goal_source_val}."
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
        # Send zero command
        self.V_G_des = pin.Motion.Zero()  # [cite: 53]
        self.send_robot_command()  # [cite: 53]

        # Request default modes asynchronously (fire and forget)
        if self._fr3_int_set_ctrl_mode.service_is_ready() and self._fr3_int_set_goal_src.service_is_ready():
            ctrl_mode_req = SetControlMode.Request(control_mode=self.default_control_mode)
            goal_src_req = SetGoalSource.Request(goal_source=self.default_goal_source)
            # Fire and forget these calls
            self._fr3_int_set_ctrl_mode.call_async(ctrl_mode_req)
            self._fr3_int_set_goal_src.call_async(goal_src_req)
            self.get_logger().info("Requested reset to default control mode and goal source.")
        else:
            self.get_logger().warn("Cannot reset modes: control mode or goal source service not ready.")

    # Remove the old is_active property and its setter [cite: 50, 51, 52, 53, 54, 55, 56, 57, 58]
    # Remove old _start_control_callback and _stop_control_callback [cite: 58, 59, 60, 61, 62, 63]
    # Remove old _set_ctrl_mode_done_callback and _set_goal_src_done_callback [cite: 63, 64, 65, 66, 67, 68, 69, 70, 71]


# Need asyncio for await in execute_callback
import asyncio


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RLAgent()  # Renamed class
        # Use MultiThreadedExecutor for Action Server responsiveness
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt detected.")
        finally:
            node.get_logger().info("Shutting down executor...")
            executor.shutdown()
            # Deactivate and reset mode on shutdown (best effort)
            node._deactivate_and_reset_mode_async()
            # Allow some time for async calls? Might not work reliably here.
            # time.sleep(0.5)
            node.destroy_node()
    except Exception as e:
        # Log any exceptions raised during node initialization
        print(f"Error during node initialization or shutdown: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
            print("RCLPY shutdown.")


if __name__ == "__main__":
    main()  # [cite: 73]
