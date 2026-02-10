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

# Robomimic imports
import robomimic
import robomimic.utils.file_utils as FileUtils
import robomimic.utils.torch_utils as TorchUtils
import robomimic.utils.obs_utils as ObsUtils
from robomimic.algo import RolloutPolicy


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


def load_ppo_agent(agent_name, agent_dir, observation_space, action_space, device="cpu") -> PPO:
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


class RobomimicAgentWrapper:
    """
    Wrapper class for Robomimic agents to make them compatible with the RlAgentNode interface.
    This allows the RlAgentNode to use either SKRL or Robomimic agents with the same API.
    """

    def __init__(self, policy, device="cpu"):
        """
        Initialize the Robomimic agent wrapper.

        Args:
            policy (RolloutPolicy): Robomimic policy loaded from a checkpoint
            device (str): Device to load the model on. Default is "cpu".
        """
        self.policy: RolloutPolicy = policy
        self.device = device
        self.episode_started = False

    def act(self, states, timestep=0, timesteps=0):
        """
        Get an action from the policy based on the current state.

        Args:
            states (torch.Tensor): The current state/observation
            timestep: Unused, kept for compatibility with SKRL
            timesteps: Unused, kept for compatibility with SKRL

        Returns:
            Tuple[torch.Tensor, None, None]: (action, None, None)
        """
        # Make sure the policy is ready for the episode
        if not self.episode_started:
            self.policy.start_episode()
            self.episode_started = True

        # Convert the state to a dictionary if needed by the policy
        # Robomimic policies typically expect observations as dictionaries
        if isinstance(states, torch.Tensor):
            # Convert to numpy for processing
            states_np = states.cpu().numpy()

            # If this is a batch of states with batch dim of 1, remove the batch dimension
            if states_np.ndim > 1 and states_np.shape[0] == 1:
                states_np = states_np[0]

            # Create a simple observation dict with "obs" key
            obs_dict = {"obs": states_np}
        else:
            # Assume it's already in the format the policy expects
            obs_dict = states

        # Get action from policy (returns numpy array)
        action_np = self.policy(ob=obs_dict)

        # Convert to torch tensor and ensure it has batch dimension
        action = torch.tensor(action_np, device=self.device).unsqueeze(0)

        # Return format compatible with RlAgentNode's expectations
        # SKRL's act() returns (action, log_prob, outputs)
        return action, None, None

    def load(self, path):
        """
        This is a no-op since the policy is already loaded in the constructor.
        Kept for compatibility with SKRL.

        Args:
            path (str or Path): Path to the checkpoint
        """
        # The policy is already loaded in the constructor
        pass

    # def set_running_mode(self, mode):
    #     """
    #     Set the running mode of the agent.

    #     Args:
    #         mode (str): Running mode, e.g. "eval"
    #     """
    #     # Robomimic policies are already in eval mode after loading
    #     # but we'll make sure
    #     if mode == "eval":
    #         self.policy.model.eval()

    #     elif mode == "train":
    #         self.policy.model.train()

    def reset(self):
        """Reset the agent state for a new episode."""
        self.episode_started = False


def load_robomimic_agent(checkpoint_path, device=None):
    """
    Load a Robomimic agent from a given checkpoint path.

    Args:
        checkpoint_path (str or Path): Path to the saved checkpoint pth file
        device (str, optional): Device to load the model on. If None, will try to use CUDA.

    Returns:
        RobomimicAgentWrapper: A wrapped Robomimic agent compatible with RlAgentNode
    """
    # Convert checkpoint path to string if it's a Path object
    if isinstance(checkpoint_path, Path):
        checkpoint_path = str(checkpoint_path)

    # Set device
    if device is None:
        device = TorchUtils.get_torch_device(try_to_use_cuda=True)

    # Restore policy from checkpoint
    try:
        policy, ckpt_dict = FileUtils.policy_from_checkpoint(ckpt_path=checkpoint_path, device=device, verbose=True)

        # Make sure the policy is a RolloutPolicy
        if not isinstance(policy, RolloutPolicy):
            raise TypeError(f"Expected RolloutPolicy, got {type(policy)}")

        # Wrap the policy for compatibility with RlAgentNode
        wrapped_agent = RobomimicAgentWrapper(policy=policy, device=device)

        print(f"Successfully loaded Robomimic agent from {checkpoint_path}")
        return wrapped_agent

    except Exception as e:
        print(f"Failed to load Robomimic agent from {checkpoint_path}: {e}")
        raise e
