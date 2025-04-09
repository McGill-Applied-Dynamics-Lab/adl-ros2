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

from robot_arm_interface.utils import motion2rostwist, rostwist2motion, rospose2se3, se32rospose

# Action and services
from std_srvs.srv import Trigger

SOCKET_POSE = np.array([0.5, 0.0, 0.10])


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


class RlAgentNode(Node):
    def __init__(self):
        super().__init__("rl_agent_node")
        self.get_logger().info("RL Agent Deployment Node started.")

        #! --- Parameters ---
        self.declare_parameter("agent_checkpoint_path", "/path/to/your/agent.pt")  # IMPORTANT: Update this!
        self.declare_parameter("control_frequency", 50.0)  # Hz - Rate of the control loop
        self.declare_parameter("action_scale", 0.05)  # Optional scaling for actions
        # Add more parameters as needed (e.g., topic names, robot limits)

        agent_path = self.get_parameter("agent_checkpoint_path").get_parameter_value().string_value
        control_frequency = self.get_parameter("control_frequency").get_parameter_value().double_value
        self.action_scale = self.get_parameter("action_scale").get_parameter_value().double_value

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
        self._init_subscribers()
        self._init_publishers()

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
            pin.rpy.rpyToMatrix(np.array([0, 0, 0])), np.array([0, 0, 0.107])
        )  # Gripper to peg transform
        self.V_G: pin.Motion = None  # Gripper velocity
        self.V_G_des: pin.Motion = None  # Desired gripper velocity

        self.X_S: pin.SE3 = pin.SE3(
            pin.rpy.rpyToMatrix(np.array([0, 0, 0])), SOCKET_POSE
        )  # Socket position # TODO: Update from topic (camera)
        self.X_P: pin.SE3 = None  # Peg position

        self._last_action: np.array = np.zeros(self._action_dim, dtype=np.float32)

        # ** RPL Params **
        des_ee_speed = env_cfg["actions"]["arm_action"]["des_ee_speed"]
        self.V_WG_ctrl: pin.Motion = pin.Motion(
            np.array([0, 0, des_ee_speed]), np.zeros(3)
        )  # Gripper velocity in world frame

        # **** Control Loop State ****
        self.is_active = True  # Flag to control if the agent loop runs

        # **** Service Servers for Start/Stop ****
        self._start_service = self.create_service(Trigger, "start_rl_agent", self._start_control_callback)
        self._stop_service = self.create_service(Trigger, "stop_rl_agent", self._stop_control_callback)
        self.get_logger().info("Offering services: 'start_rl_agent' and 'stop_rl_agent'")

        # --- Control Loop Timer ---
        self.timer_period = 1.0 / control_frequency
        self.control_timer = self.create_timer(self.timer_period, self.control_loop_callback)

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

        robot_cmd_topic = "/robot_cmd"
        self._robot_cmd_pub = self.create_publisher(
            TwistStamped,
            robot_cmd_topic,  # Topic for controller commands
            10,
        )

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
            self.get_logger().warn("Waiting for joint states...", throttle_duration_sec=5)
            return np.zeros(self._observation_dim, dtype=np.float32)

        #! Process observations
        joint_pos_rel = np.array(self._joint_state.position) - self.default_joint_poses
        joint_pos_rel = np.concatenate([joint_pos_rel, np.array([0.0, 0.0])])  # Add fingers #TODO: From real msg

        joint_vel_rel = np.array(self._joint_state.velocity)
        joint_vel_rel = np.concatenate([joint_vel_rel, np.array([0.0, 0.0])])  # Add fingers #TODO: From real msg

        self.X_P = self.X_G * self.X_GP
        peg_pos_rel = self.X_P.translation - self.X_S.translation

        last_action = self._last_action

        obs_list = [joint_pos_rel, joint_vel_rel, peg_pos_rel, last_action]

        observation = np.concatenate(obs_list).astype(np.float32)
        assert observation.shape == (self._observation_dim,), f"Observation shape mismatch: {observation.shape}"

        obs_tensor = torch.tensor(observation, dtype=torch.float32).to(self.device)

        return obs_tensor

    # --- Callback Functions for Subscriptions ---
    def _robot_state_callback(self, robot_state_msg: PoseStamped):
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
        if not self.is_active:
            return

        # 1. Get observations
        self.latest_observation: torch.tensor = self._get_observation()

        # 2. Get Action from Agent
        action_tensor: torch.tensor = self._get_action()
        action = action_tensor.cpu().numpy()  # Convert to numpy array for processing

        # 3. Postprocess Action
        # Scale actions if necessary (e.g., if agent outputs [-1, 1] but robot needs rad/s)
        # Clip actions to robot limits (SAFETY!)
        processed_action = action * self.action_scale
        # processed_action = self.clip_action(processed_action)  # Implement clipping function

        # 4. Combine w/ ctrl action
        self.V_G_agt = pin.Motion(
            np.array([processed_action[0], 0.0, processed_action[1]]),
            np.array([0.0, processed_action[2], 0.0]),
        )

        self.V_G_des = self.V_WG_ctrl + self.V_G_agt

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

    # --- Service Callbacks ---
    def _start_control_callback(self, request: Trigger.Request, response: Trigger.Response):
        """
        Service callback to start the control loop.
        """
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
        """
        Service callback to stop the control loop.
        """
        if not self.is_active:
            response.success = False
            response.message = "Control loop is already inactive."
            self.get_logger().warn("Stop control called when already inactive.")

        else:
            self.is_active = False

            # --- Safety: Send a zero command immediately upon stopping ---
            self.get_logger().info("Sending zero command due to stop request.")
            self.V_G_des = pin.Motion()
            self.send_robot_command()  # Send command to stop motion

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
