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

# SKRL
from skrl.agents.torch import Agent
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
from skrl.multi_agents.torch.ippo import IPPO, IPPO_DEFAULT_CONFIG
from skrl.multi_agents.torch.mappo import MAPPO, MAPPO_DEFAULT_CONFIG
from skrl.resources.preprocessors.torch import RunningStandardScaler  # noqa
from skrl.resources.schedulers.torch import KLAdaptiveLR  # noqa
from skrl.utils.model_instantiators.torch import deterministic_model, gaussian_model, shared_model
from skrl.memories.torch import RandomMemory

# Import your skrl agent definition
# from your_skrl_project.agents import YourAgentClass

# Message types
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped  # Example message types
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # Example for ros2_control

# Action and services
from std_srvs.srv import Trigger


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


# class SkrlAgent:
#     def __init__(self, agent_name, agent_dir, observation_space, action_space, device="cpu"):
#         print("Initializing Placeholder Agent")
#         self.agent_name = agent_name
#         self.agent_dir = agent_dir

#         self.agent = None

#         self.observation_space: gym.spaces.Box = observation_space
#         self.action_space: gym.spaces.Box = action_space
#         self.device = device  # cuda or cpu

#     def eval(self):
#         print("Setting agent to evaluation mode")
#         self.policy_net.eval()

#     def act(self, observation, deterministic=True):
#         print("Generating action")

#         # --- In reality, perform inference ---
#         obs_tensor = torch.tensor(observation, dtype=torch.float32, device=self.device).unsqueeze(0)
#         with torch.no_grad():
#             action_tensor = self.policy_net(obs_tensor)  # Or appropriate method
#         action = action_tensor.cpu().numpy().squeeze(0)

#         # --- Placeholder action ---
#         # Ensure the action shape matches your action_space definition
#         # action = np.zeros(self.action_space.shape)

#         return action, None, {}  # skrl act usually returns (action, logprob, other_outputs)


class RlAgentNode(Node):
    def __init__(self):
        super().__init__("rl_agent_node")
        self.get_logger().info("RL Agent Deployment Node started.")

        #! --- Parameters ---
        self.declare_parameter("agent_checkpoint_path", "/path/to/your/agent.pt")  # IMPORTANT: Update this!
        self.declare_parameter("control_frequency", 50.0)  # Hz - Rate of the control loop
        self.declare_parameter("action_scale", 1.0)  # Optional scaling for actions
        # Add more parameters as needed (e.g., topic names, robot limits)

        agent_path = self.get_parameter("agent_checkpoint_path").get_parameter_value().string_value
        control_frequency = self.get_parameter("control_frequency").get_parameter_value().double_value
        self.action_scale = self.get_parameter("action_scale").get_parameter_value().double_value

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
        # package_share_directory = Path(get_package_share_directory("robot_tasks"))  # TODO: Update this!
        pkg_dir = Path(__file__).parent.parent
        agent_dir = pkg_dir / "agents" / "insert"
        agent_path = agent_dir / agent_name

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

        # --- ROS 2 Communication ---
        # ** Subscriptions (Adapt to your robot's topics) **
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",  # Topic for robot joint states
            self.joint_state_callback,
            10,
        )

        # ** Publishers / Clients (Adapt to how you control your robot) **
        # Example: Using ros2_control JointTrajectoryController
        self.joint_command_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",  # Topic for controller commands
            10,
        )

        # --- State Storage ---
        self.latest_observation = None
        self.joint_names = []  # Store joint names if needed for publishing commands
        # Initialize buffers for different parts of the observation
        self.current_joint_positions = None
        self.current_joint_velocities = None
        # self.current_ft_wrench = None
        # self.current_tcp_pose = None

        # **** Control Loop State ****
        self.is_active = False  # Flag to control if the agent loop runs

        # **** Service Servers for Start/Stop ****
        self._start_service = self.create_service(Trigger, "start_rl_agent", self._start_control_callback)
        self._stop_service = self.create_service(Trigger, "stop_rl_agent", self._stop_control_callback)
        self.get_logger().info("Offering services: 'start_rl_agent' and 'stop_rl_agent'")

        # --- Control Loop Timer ---
        self.timer_period = 1.0 / control_frequency
        self.control_timer = self.create_timer(self.timer_period, self.control_loop_callback)

        self.get_logger().info(f"Node initialized. Control loop frequency: {control_frequency} Hz.")

    # !--- RL Methods ---
    @torch.inference_mode()
    def _get_action(self) -> torch.Tensor:
        """
        Get the action from the agent based on the latest observation.
        """
        # timestep and timesteps are not used in the PPO agent
        action = self.agent.act(observation=self.latest_observation, timestep=0, timesteps=0)[0]

        return action

    def _get_observation(self) -> torch.Tensor:
        """
        Get the latest observation from the robot's sensors.
        """
        # This should be implemented to return the current observation
        # For now, we return a dummy observation
        return np.zeros(self._observation_dim, dtype=np.float32)

    # --- Callback Functions for Subscriptions ---
    def joint_state_callback(self, msg: JointState):
        # Store the latest joint states
        # IMPORTANT: Ensure the order matches what your agent expects!
        if not self.joint_names:  # Store joint names on first message if needed
            self.joint_names = msg.name
        # You might need to reorder msg.position/velocity based on self.joint_names
        # if the order isn't guaranteed
        self.current_joint_positions = np.array(msg.position, dtype=np.float32)
        self.current_joint_velocities = np.array(msg.velocity, dtype=np.float32)
        self._update_observation()

    # --- Observation Assembly ---
    def _update_observation(self):
        # Combine all received sensor data into the single observation vector
        # that the agent expects. Handle cases where some data hasn't arrived yet.
        if (
            self.current_joint_positions is not None and self.current_joint_velocities is not None
        ):  # Add checks for other sensors
            # **CRITICAL**: Ensure the order and format matches the training observation space EXACTLY!
            # This might involve normalization if you normalized during training.
            try:
                obs_list = [
                    self.current_joint_positions,
                    self.current_joint_velocities,
                    # self.current_ft_wrench,
                    # self.current_tcp_pose,
                    # ... other components
                ]
                # Check if all components are available
                if all(comp is not None for comp in obs_list):
                    self.latest_observation = np.concatenate(obs_list).astype(np.float32)
                    # Optional: Normalize observation if required by the agent
                    # self.latest_observation = self.normalize_observation(self.latest_observation)
                # else:
                #    self.get_logger().warn("Waiting for all observation components...", throttle_duration_sec=5)

            except ValueError as e:
                self.get_logger().error(f"Error concatenating observation components: {e}. Check shapes.")
                self.latest_observation = None  # Invalidate observation

    # --- Control Loop ---
    def control_loop_callback(self):
        # 1. Get observations
        if self.latest_observation is None:
            # self.get_logger().warn("No valid observation received yet.", throttle_duration_sec=5)
            return

        # 2. Get Action from Agent
        action: torch.tensor = self._get_action()

        # 3. Postprocess Action
        # Scale actions if necessary (e.g., if agent outputs [-1, 1] but robot needs rad/s)
        # Clip actions to robot limits (SAFETY!)
        processed_action = action * self.action_scale
        processed_action = self.clip_action(processed_action)  # Implement clipping function

        # 4. Send Action to Robot
        self.send_robot_command(processed_action)

    def clip_action(self, action):
        # Implement safety clipping based on your robot's real limits
        # Example: velocity limits
        # max_vel = [1.0, 1.0, 1.0, 1.5, 1.5, 1.5] # rad/s per joint
        # min_vel = [-1.0, -1.0, -1.0, -1.5, -1.5, -1.5]
        # clipped_action = np.clip(action, min_vel, max_vel)
        # return clipped_action
        self.get_logger().warn("Action clipping not implemented yet!", throttle_duration_sec=10)
        return action  # Placeholder

    def send_robot_command(self, action: torch.tensor):
        # Convert the agent's action into the correct ROS 2 message/service format

        # --- Example for ros2_control JointTrajectoryController (sending velocity commands) ---
        # This is a simplified example assuming 'action' contains target velocities.
        # A more robust implementation might send positions based on integrating velocities
        # or use a dedicated velocity controller interface if available.
        if self.joint_names:
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.velocities = [float(v) for v in action]  # Agent action -> velocity command
            # Set positions based on integrating velocity over timer_period, or use current pos for safety
            # For simplicity, often just setting velocities is enough if the controller handles it.
            # point.positions = [...] # Optional: Calculate target positions
            point.time_from_start = rclpy.duration.Duration(
                seconds=self.timer_period * 1.5
            ).to_msg()  # Time to reach target

            traj_msg.points.append(point)

            self.joint_command_pub.publish(traj_msg)
        else:
            self.get_logger().warn("Joint names not received yet, cannot send command.", throttle_duration_sec=5)

        # --- Example for direct velocity controller ---
        # vel_msg = Float64MultiArray()
        # vel_msg.data = [float(v) for v in action]
        # self.joint_velocity_pub.publish(vel_msg)

        # --- Example for service call (e.g., trigger gripper) ---
        # request = Trigger.Request()
        # future = self.gripper_client.call_async(request)
        # Add handling for service response if needed

    # --- Service Callbacks ---
    def _start_control_callback(self, request: Trigger.Request, response: Trigger.Response):
        if self.is_active:
            response.success = False
            response.message = "Control loop is already active."
            self.get_logger().warn("Start control called when already active.")
        else:
            self.is_active = True
            # Optional: Reset any internal agent state if necessary upon starting
            # e.g., if your agent uses RNNs or has internal memory
            response.success = True
            response.message = "Control loop started."
            self.get_logger().info("Control loop activated via service call.")
        return response

    def _stop_control_callback(self, request: Trigger.Request, response: Trigger.Response):
        if not self.is_active:
            response.success = False
            response.message = "Control loop is already inactive."
            self.get_logger().warn("Stop control called when already inactive.")
        else:
            self.is_active = False
            # --- Safety: Send a zero command immediately upon stopping ---
            self.get_logger().info("Sending zero command due to stop request.")
            zero_action = np.zeros(self._action_dim, dtype=np.float32)
            self.send_robot_command(zero_action)  # Send command to stop motion
            # -------------------------------------------------------------
            response.success = True
            response.message = "Control loop stopped."
            self.get_logger().info("Control loop deactivated via service call.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RlAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected, shutting down.")
    finally:
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
