<p align="center">
    <img src="media/MADL_logo.svg" alt="ADL Logo"/>
</p>

# ADL ROS2 Documentation

Welcome to the **Applied Dynamics Lab ROS2** documentation! 
This guide covers all the ROS2 packages developed by the McGill Applied Dynamics Lab for robotic systems, 
particularly focusing on controlling the Franka Research 3.

## Franka Controller
Main packages to control and develop with the Franka. 
The controller is dividied in two:

1. **[Franka Server](franka-server/index.md)**: Low level interface with the arm. Runs on the mini-pc
2. **[Franka Client](franka-client/index.md)**: Higher level interface. Runs on the user's workstation. 

<figure markdown="span">
  ![test](media/figures/network_architecture.png){ width="500" }
  <figcaption>Network Setup</figcaption>
</figure>

### Quick Start

1. **Getting Started**: Check out the [getting started guide](user-guide/getting-started.md)
2. **Installation**: For steps on how to setup the robots and PCs, follow the [installation guide](user-guide/installation.md)
3. **Examples**: Explore the [tutorials](tutorials/index.md) (To Come...)

### User Guides
Documentation and user guides of each packages.

- **[Installation](user-guide/installation.md)**
- **[Getting Started](user-guide/getting-started.md)**
- **[Franka Server](user-guide/franka-server.md)**
- **[Experiments](user-guide/experiments.md)**
- **[DelayRIM](user-guide/delay-rim.md)**

## Other packages
...



<!-- ## Key Features
- 🦾 **Robot Arm Control**: Complete control stack for Franka FR3 robot
- 🤖 **RL Integration**: Reinforcement learning task execution
- 🎮 **Teleoperation**: Teleoperation of the arm with the Inverse3 and Joystick -->

## System Requirements

It's recommended to use the Docker container for installing the packages. 


If you prefer to install locally:

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+

<!-- 
## Contributing

We welcome contributions! Please see our [contributing guide](developer-guide/contributing.md) for details on:

- Code style and standards
- Pull request process
- Issue reporting
- Development workflow 
-->

## Support

- 📖 Documentation
- 🐛 Issues: [GitHub Issues](https://github.com/McGill-Applied-Dynamics-Lab/ADL-ros2/issues)
<!-- - 💬 Discussions: [GitHub Discussions](https://github.com/McGill-Applied-Dynamics-Lab/ADL-ros2/discussions) -->

---

*Made by the McGill Applied Dynamics Lab*
