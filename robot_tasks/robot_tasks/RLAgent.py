import rclpy
import rclpy.action
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

SOCKET_POSE = np.array([0.6, 0.0, 0.0])
Z_STOP = 0.015  # 1cm


def _process_cfg(cfg: dict) -> dict:
    """
    Convert simple types to skrl classes/components
    cfg: A configuration dictionary
    return: Updated dictionary
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


class RlAgentNode(Node):
    def __init__(self):
        super().__init__("rl_agent_node")
        self.get_logger().info("RL Agent Deployment Node started.")

        # Action Server needs a lock for safe state transitions if using MTE
        self._lock = threading.Lock()
        # Goal handle storage - protected by the lock
        self._goal_handle: ServerGoalHandle = None

        #! --- Parameters ---
        self.declare_parameter("agent_checkpoint_path", "/path/to/your/agent.pt")  # IMPORTANT: Update this!
        self.declare_parameter("control_frequency", 50.0)  # Hz - Rate of the control loop
        self.declare_parameter("action_scale", 0.05)  # Optional scaling for actions

        agent_path = self.get_parameter("agent_checkpoint_path").get_parameter_value().string_value
        control_frequency = self.get_parameter("control_frequency").get_parameter_value().double_value
        self.action_scale = self.get_parameter("action_scale").get_parameter_value().double_value / 10

        #! --- Configs ---
        pkg_dir = Path(__file__).parent.parent
        agent_dir = pkg_dir / "agents" / "insert"

        # Load cfg
        agent_cfg_path = agent_dir / "params" / "agent.yaml"
        print(f"[INFO]: Parsing configuration from: {agent_cfg_path}")
        with open(agent_cfg_path, encoding="utf-8") as f:
            agent_cfg = yaml.full_load(f)

        env_cfg_path = agent_dir / "params" / "env.yaml"
        print(f"[INFO]: Parsing configuration from: {env_cfg_path}")
        with open(env_cfg_path, encoding="utf-8") as f:
            env_cfg = yaml.full_load(f)

        #! --- Define Observation and Action Spaces ---
        # These MUST match the spaces used during training in Isaac Sim
        self._observation_dim = 24
        self._action_dim = 3

        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(self._observation_dim,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(self._action_dim,), dtype=np.float32
        )  # Assuming normalized actions

        #! --- Load Agent ---
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        agent_name = "agent.pt"
        agent_path = agent_dir / agent_name
        # package_share_directory = Path(get_package_share_directory("robot_tasks"))  # TODO: Update this!

        self.agent: PPO = load_ppo_agent(
            agent_name=agent_name,
            agent_dir=agent_dir,
            observation_space=self.observation_space,
            action_space=self.action_space,
            device=self.device,
        )

        try:
            self.agent.load(agent_path)
            self.agent.set_running_mode("eval")
            self.get_logger().info(f"Agent loaded successfully from {agent_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to load agent from {agent_path}: {e}")
            # Handle error appropriately - maybe shutdown node
            rclpy.shutdown()
            return

        #! --- ROS 2 Communication ---
        self.callback_group = ReentrantCallbackGroup()

        self._action_server_name = "/insert_action"

        self._init_subscribers()
        self._init_publishers()
        self._init_services()
        self._init_action_server()  # Added Action Server init

        #! --- State Storage ---
        self.default_joint_poses = [
            0.0,
            0.53249995,
            0.0,
            -2.02528006,
            0.0,
            2.55778002,
            0.78539816,
        ]  # Start position of the joints [rad]

        self.latest_observation = None

        self._joint_state: JointState = None
        # self._current_joint_velocities: np.array = None

        # Poses and twistpin.Motion
        self.X_GP: pin.SE3 = pin.SE3(
            pin.rpy.rpyToMatrix(np.array([0, 0, 0])), np.array([0, 0, 0.04])
        )  # Gripper to peg transform
        self.V_G: pin.Motion = None  # Gripper velocity
        self.V_G_des: pin.Motion = None  # Desired gripper velocity

        self.X_S: pin.SE3 = pin.SE3(
            pin.rpy.rpyToMatrix(np.array([0, 0, 0])), SOCKET_POSE
        )  # Socket position # TODO: Update from topic (camera)
        self.X_P: pin.SE3 = None  # Peg position #TODO: Read from a topic
        self.X_G: pin.SE3 = None  # Gripper pose

        self._last_action: np.array = np.zeros(self._action_dim, dtype=np.float32)

        self._is_insertion_success = False
        self._is_insertion_failed = False
        self._task_finished = False

        # ** RPL Params **
        self.des_ee_speed = env_cfg["actions"]["arm_action"]["des_ee_speed"] / 10
        # self.des_ee_speed = 0.0
        self.V_WG_ctrl: pin.Motion = pin.Motion(
            np.array([0, 0, self.des_ee_speed]), np.zeros(3)
        )  # Gripper velocity in world frame

        # **** Control Loop State ****
        # self._is_active = False  # Flag to control if the agent loop runs
        # self.is_active = True  # Initially inactive

        # --- Control Loop Timer ---
        self.timer_period = 1.0 / control_frequency
        self.control_timer = self.create_timer(
            self.timer_period, self.control_loop_callback, callback_group=self.callback_group
        )

        self.get_logger().info(f"Node initialized. Control loop frequency: {control_frequency} Hz.")

    def _init_subscribers(self):
        """
        Initialize subscribers for robot and env states.
        """
        self.get_logger().info("Initializing subscribers...")

        # --- robot ---
        # TODO: Update when processed in the interface
        robot_topic = "/franka_robot_state_broadcaster/robot_state"
        self._robot_sub = self.create_subscription(FrankaRobotState, robot_topic, self._robot_state_callback, 10)

        # # TODO
        # # --- env ---
        # socket_topic = '/env/socket_pose'
        # self._socket_sub = self.create_subscription(
        #     PoseStamped, socket_topic, self._socket_state_callback, 10
        # )

    def _init_publishers(self):
        """
        Initialize publishers for robot commands.
        """
        self.get_logger().info("Initializing publishers...")

        robot_cmd_topic = "/robot_arm/gripper_vel_command"
        self._robot_cmd_pub = self.create_publisher(
            TwistStamped,
            robot_cmd_topic,  # Topic for controller commands
            10,
        )

    def _init_services(self):
        """
        Init services clients and servers
        """

        # # **** Service Servers for Start/Stop ****
        # self._start_service = self.create_service(Trigger, "start_rl_agent", self._start_control_callback)
        # self._stop_service = self.create_service(Trigger, "stop_rl_agent", self._stop_control_callback)
        # self.get_logger().info("Offering services: 'start_rl_agent' and 'stop_rl_agent'")

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
            PegInHole,  # Replace with your actual action type
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

        mode_set_ok = await self._set_robot_mode_async(control_mode=ControlMode.CART_VEL, goal_source=GoalSource.TOPIC)

        if not mode_set_ok:
            self.get_logger().error("Failed to set robot modes for RL Agent. Aborting goal.")
            result = PegInHole.Result(success=False, message="Failed to set robot control mode or goal source.")
            goal_handle.abort()
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
                    self._deactivate_and_reset_mode_async()

                    self._cleanup_goal(goal_handle)  # Pass handle for safety
                    self.get_logger().info("Action completed")
                    return result

                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Goal cancel requested.")
                    self._deactivate_and_reset_mode_async()  # gemini had `await _deactivate_and_reset_mode` but not the method...

                    result.success = False
                    result.message = "Task cancelled."
                    goal_handle.canceled()
                    self._cleanup_goal(goal_handle)

                    self.get_logger().info("Action completed")
                    return result

                if self._task_finished:
                    self.get_logger().info("Task finished.")

                    # Insertion succeded
                    if self._is_insertion_success:
                        result.success = True
                        result.message = "Goal succeeded."
                        goal_handle.succeed()

                    # Insertion failed
                    elif self._is_insertion_failed:
                        result.success = False
                        result.message = "Insertion failed."
                        goal_handle.abort()

                    self._deactivate_and_reset_mode_async()

                    self._cleanup_goal(goal_handle)
                    self.get_logger().info("Action completed")
                    return result

            # Check success/failure conditions (done within control_loop_callback now)
            # Publish feedback (optional - can be done in control_loop_callback)
            # feedback_msg = PegInHole.Feedback()
            # feedback_msg.distance_to_target = np.linalg.norm(self.peg_error)
            # goal_handle.publish_feedback(feedback_msg)

            # Allow other callbacks to run
            # await asyncio.sleep(0.1)  # Use asyncio sleep
            time.sleep(0.1)

        # If rclpy is shut down
        self.get_logger().warn("RCLPY shutdown during goal execution.")
        await self._deactivate_and_reset_mode_async()

        result.success = False
        result.message = "System shutdown during execution."

        # Cannot set goal status if node is shutting down
        # goal_handle.abort(result) # Might fail
        self._cleanup_goal(goal_handle)
        return result

    def _cleanup_goal(self, goal_handle=None):
        """Reset goal handle, ensuring lock safety."""
        self.get_logger().info("Cleaning up goal handle...")

        #! Must be called from within a lock to ensure thread safety
        # with self._lock:
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

    def _get_observation(self) -> torch.Tensor:
        """
        Get the latest observation from the robot's sensors.
            - joint_pos_rel
            - joint_vel_rel
            - peg_pos_rel
            - last_action
        """
        # Return if one observation component is not available
        if self._joint_state is None or self.X_GP is None or self.X_S is None:
            self.get_logger().warn("Missing some observations...", throttle_duration_sec=5)
            return None

        #! Process observations
        joint_pos_rel = np.array(self._joint_state.position) - self.default_joint_poses
        joint_pos_rel = np.concatenate([joint_pos_rel, np.array([0.0, 0.0])])  # Add fingers #TODO: From real msg

        joint_vel_rel = np.array(self._joint_state.velocity)
        joint_vel_rel = np.concatenate([joint_vel_rel, np.array([0.0, 0.0])])  # Add fingers #TODO: From real msg

        self.X_P = self.X_G * self.X_GP
        self.peg_error = self.X_P.translation - self.X_S.translation
        peg_pos_rel = self.peg_error
        # print(f"Peg pose: {self.X_P.translation[2]}\t\t{peg_pos_rel[2]}")

        last_action = self._last_action

        obs_list = [joint_pos_rel, joint_vel_rel, peg_pos_rel, last_action]

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

        obs_tensor = torch.tensor(observation, dtype=torch.float32).to(self.device)

        return obs_tensor

    # --- Callback Functions for Subscriptions ---

    def _robot_state_callback(self, robot_state_msg: FrankaRobotState):
        """
        Callback for the '/franka_robot_state_broadcaster/robot_state' topic.

        Parameters
        ----------
        robot_state_msg : franka_msgs/msg/FrankaRobotState
            Message received from the topic.
        """
        self._joint_state: JointState = robot_state_msg.measured_joint_state
        # self._current_joint_velocities: JointState = robot_state_msg.desired_joint_state

        self.X_G = rospose2se3(robot_state_msg.o_t_ee.pose)
        self.V_G = rostwist2motion(robot_state_msg.o_dp_ee_d.twist)

    #! --- MARK: Control Loop ---

    def control_loop_callback(self):
        # Check if there is an active goal
        with self._lock:
            if self._goal_handle is None or not self._goal_handle.is_active:
                self.get_logger().debug("No active goal, control loop idle.", throttle_duration_sec=10)
                return  # Do nothing if no goal is active

            # Get a reference to the current goal handle inside the lock
            goal_handle = self._goal_handle

        if self._task_finished:
            # self.get_logger().info("Task finished, exiting control loop.")
            return

        # -- Get observations
        self.latest_observation: torch.tensor = self._get_observation()
        if self.latest_observation is None:
            self.get_logger().warn("Control loop active, but observation is not ready.")  # , throttle_duration_sec=5)
            return

        # -- Check if success/failure
        self._is_insertion_success, self._is_insertion_failed = self.check_insertion_status()

        if self._is_insertion_success or self._is_insertion_failed:
            self._task_finished = True

            if self._is_insertion_success:
                self.get_logger().info("Insertion success!")
                # goal_handle.succeed()
                self._task_finished = True

            elif self._is_insertion_failed:
                self.get_logger().info("Insertion failed!")
                # goal_handle.abort()  # Use abort for failure condition
                self._task_finished = True

            return

        # if self._task_finished:
        #     # Deactivate and cleanup outside the lock if possible, or call async helper
        #     # self._deactivate_and_reset_mode_async()  # Fire and forget deactivation
        #     # self._cleanup_goal(goal_handle)
        #     return  # Stop further processing in this loop iteration

        # -- Get Action from Agent
        action_tensor: torch.tensor = self._get_action()
        action = action_tensor.cpu().numpy()  # Convert to numpy array for processing
        self._last_action = action  # Store last action for observation

        # -- Postprocess Action
        processed_action = action * self.action_scale
        # processed_action = self.clip_action(processed_action)  # Implement clipping function

        # -- Combine w/ ctrl action
        self.V_G_agt = pin.Motion(
            np.array([processed_action[0], 0.0, processed_action[1]]),
            np.array([0.0, processed_action[2], 0.0]),
        )

        self.V_WG_ctrl: pin.Motion = pin.Motion(
            np.array([0, 0, self.des_ee_speed]), np.zeros(3)
        )  # Gripper velocity in world frame

        self.V_G_des = self.V_WG_ctrl
        # self.V_G_des = self.V_WG_ctrl + self.V_G_agt

        # 4. Send Action to Robot
        self.send_robot_command()

    def clip_action(self, action):
        # Implement safety clipping based on your robot's real limits
        # Example: velocity limits
        # max_vel = [1.0, 1.0, 1.0, 1.5, 1.5, 1.5] # rad/s per joint
        # min_vel = [-1.0, -1.0, -1.0, -1.5, -1.5, -1.5]
        # clipped_action = np.clip(action, min_vel, max_vel)
        # return clipped_action
        self.get_logger().warn("Action clipping not implemented yet!", throttle_duration_sec=10)
        return action  # Placeholder

    def send_robot_command(self):
        """
        Convert the desired gripper twist to ROS 2 message format and publish it.
        """
        msg: TwistStamped = TwistStamped()
        gripper_twist: Twist = motion2rostwist(self.V_G_des)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist = gripper_twist

        self._robot_cmd_pub.publish(msg)
        # self.get_logger().info(f"Published command: {gripper_twist}")

    def check_insertion_status(self):
        """
        Check if insertion is successful or failed.
        """
        success = False
        failure = False
        z_thres = Z_STOP  # 1cm
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

        # TODO: Add failure condition if forces are too high

        # TODO: Add timeout condition

        return success, failure

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
            # Use asyncio.gather or similar if running in an async context,
            # otherwise, need rclpy mechanisms if called from sync context
            # For simplicity here, assuming called from async execute_callback
            ctrl_response: SetControlMode.Response = await ctrl_future
            goal_response: SetGoalSource.Response = await goal_future

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

        # Send zero command
        self.V_G_des = pin.Motion.Zero()  # [cite: 53]
        self.send_robot_command()  # [cite: 53]

        # Request default modes asynchronously (fire and forget)
        if self._fr3_int_set_ctrl_mode.service_is_ready() and self._fr3_int_set_goal_src.service_is_ready():
            ctrl_mode_req = SetControlMode.Request(control_mode=ControlMode.PAUSE.value)
            # goal_src_req = SetGoalSource.Request(goal_source=self.default_goal_source)
            # Fire and forget these calls
            self._fr3_int_set_ctrl_mode.call_async(ctrl_mode_req)
            # self._fr3_int_set_goal_src.call_async(goal_src_req)
            self.get_logger().info("Requested to set control mode to `PAUSE`")

        else:
            self.get_logger().warn("Cannot reset modes: control mode or goal source service not ready.")

    # Add this helper method for converting between int and enum
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

    # Example usage:
    def handle_current_mode(self, mode_int):
        """Example of how to convert an integer back to enum"""
        mode = self._get_enum_from_value(ControlMode, mode_int)
        if mode is not None:
            self.get_logger().info(f"Current control mode: {mode.name}")
        else:
            self.get_logger().warn(f"Unknown control mode value: {mode_int}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RlAgentNode()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()

        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt detected.")

        finally:
            node.get_logger().info("Shutting down executor...")
            executor.shutdown()

            # # Deactivate and reset mode on shutdown (best effort)
            # node._deactivate_and_reset_mode_async()

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
    main()
