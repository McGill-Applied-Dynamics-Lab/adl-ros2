# Getting Started

This guide will help you get up and running with the project and basic functionalities of the Franka Arm. It assumes
the server is already installed on the mini PC. If it's not the case follow the 
[server installation instructions](franka-server/installation.md).

!!! tip "Glossary"

    - **Franka Control** / **Controller**: Black Box on the table
    - **Franka PC** / **Desktop**: Franka PC on the ground
    - **Franka Desk**: Web app to control the robot
    - **FR3-server**: Set of ros2 nodes to publish the robot states and send position commands. Running on the *Franka PC*.

!!! info "General Info"
    
    Model: Franka Research 3
    - - -
    **IPs**

    - Controller IP: 10.69.54.223
    - Franka Desktop: 10.69.54.222
    - - -
    **ROS2**

    - ROS2 Version: humble
    - Domain: 01

!!! note "Useful Resources"

    - [FR3 Documents](https://franka.de/documents)
    	- [Datasheet](https://download.franka.de/documents/Datasheet%20Franka%20Research%203_R02212_1.2_EN.pdf)
    	- [Product manual](https://download.franka.de/documents/Product%20Manual%20Franka%20Research%203_R02210_1.1_EN.pdf)
    	- [Franka hand ](https://download.franka.de/documents/Product%20Manual%20Franka%20Hand_R50010_1.1_EN.pdf)
    - [FCI Documentation](https://frankarobotics.github.io/docs/index.html)
    - [FCI API Doc (c++)](https://frankarobotics.github.io/libfranka/0.15.0/)
    - [FR3 DH Parameters](https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters)
    - [FR3 Limits](https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3)

## 1. Connect to the Franka PC (server)

1. Connect your host to the switch via the Ethernet cable

2. Set your network configuration

      1. **IP**: 192.168.1.3, **Net Mask**: 255.255.255.0

      2. Test the connection
        ```bash
        ping 192.168.1.2 # Ping the Franka PC
        ```
    <figure markdown="span">
    ![network_image](media/figures/network_architecture.png){ width="500" }
    <figcaption>Network Setup</figcaption>
    </figure>

3. Configure the ssh connection to the *Franka PC*
    <!-- TODO: Update username -->

    1. Test that ssh works
    ```bash
    ssh csirois@192.168.1.2
    # pwd: FrankaPC2024
    ```

    2. Add ssh to your configs

        - In `~/.ssh/config` add the following configuration
  
            ```bash
            Host franka-pc
                HostName 192.168.1.2
                User csirois
                IdentityFile ~/.ssh/id_ed25519  # (1)!
                ForwardX11 yes
                ForwardX11Trusted yes
                LocalForward 8443 10.69.54.223:443 # (2)!
            ```

            1. To log without the password. See comment below. 
            2. To forward desk to local port

        - You can now connect to the Franka PC with:
        ```bash
        ssh franka-pc
        # pwd: FrankaPC2024
        ``` 

        <!-- TODO: #49 [docs] Instructions to install zsh fonts when ssh -->

    ??? tip "SSH without the password"
        TODO...
        <!-- # TODO: #48 Add ssh withouth password instructions -->


## 2. Start the Robot
<!-- TODO: Add Desk images -->

1. Power on the *Franka Controller*

2. Open *Franka Desk*
	- You have two options: on the **Franka PC** or via **SSH** (recommended)

    - **On the *Franka PC***
        - Open a session using the monitor connected to the PC
            - User: csirois, pwd: FrankaPC2024
        - In a browser, go to *10.69.54.223*
  
    - **Using SSH on your laptop**
        - SSH to the Franka PC
        ```
        ssh franka-pc
        ```
        - In your browser, open desk [https://localhost:8443/desk/](https://localhost:8443/desk/)
  <!-- TODO: #47 [docs] Add securit risk screenshots -->

3. Unlock the joints
<!-- TODO: #46 Add picture -->

4. Activate FCI

admin
frankaadmin

## 3. Test the robot
See [Franka Server User Guide](./franka-server.md) for a comprehensive guides on the **Franka Server** 
functionalities.

1. TODO: Add test script

```bash
# On the server
fr3-launch
```

!!! tip "Save the motors"

    Make sure to lock the joints when you don't plan to use the robot for a few minutes. It prevents the motors 
    from overheating.

## 4. Setup the client
The ADL ROS2 packages provide various way for controlling the FR3. The easiest is to use the `arm_client`, a python wrapper
around the ROS2 server.

Follow the steps to install the project:

1. On your host, clone the project
    ```bash
    git clone https://github.com/McGill-Applied-Dynamics-Lab/adl-ros2.git
    ```

2. Install the environment
    ```bash
    cd adl-ros2
    pixi install # (1)!
    pixi run setup-colcon 
    ```
    1. See the note below for pixi installation


3. Build the ROS2 packages
    ```bash
    pixi shell -e humble # (1)!
    pixi run -e humble build     
    ```
    1. To start a shell in the pixi environment


    !!! info "ROS2 Workspace"
    
        This build the ros2 packages to `install_humble` and `build_humble`

4. Verify the installation
    ```bash
    pixi run -e humble test-install
    ```

!!! note "Pixi"

    [Pixi](https://pixi.prefix.dev/latest/) is a fast package management tool. 
    To install, follow the [installation instructions](https://pixi.prefix.dev/latest/installation/).


## 5. Control the robot!
1. Start the server
    ```bash
    # On the Franka PC
    fr3-launch
    ```

2. Run an example
    ```bash
    # On your workstation
    pixi run -e humble python examples/00_home.py
    ```

If you wish to to run scripts from Visual Studio Code, look at [VS Code Setup Instructions](). 


Explore the other [`examples`](https://github.com/McGill-Applied-Dynamics-Lab/adl-ros2/tree/main/examples) 
to discover various ways of controlling the robots.


## Next Steps

- Explore the [API Reference](..) for detailed documentation
- Review the [Developer Guide](../developer-guide/contributing.md) if you want to contribute
