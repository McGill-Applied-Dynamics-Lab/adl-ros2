# Installation
Detailed installation instructions for setting up the Franka arm and the ADL ROS2 packages.
*Step 1 and 2 shouldn't really be needed now that everything is setup...*

This just on how to get everything ready, see [getting started](./getting-started.md) for instructions on how to use the
different parts of the project.


## 1. Franka Research 3
Installation of the Franka Research 3

## 2. Franka PC
> INFO: **System Requirements**
> **Operating System**
>
> - Real-time Ubuntu 22.04 LTS (Recommended)
>
> - - -
> **Software Dependencies**
> 
> - Docker
>


Steps to setup the Franka PC.

- Ubuntu 22.04
- Docker
- IP
- Ping FR3
- Franka Server
- Test libfranka
- Test server

## 3. ADL ROS 2
> INFO: **System Requirements**
> **Operating System**
>
> - Ubuntu 22.04 LTS (Recommended)
>
> - - -
> **Software Dependencies**
> 
> - Docker
>

How to setup the client packages to control the robot arm via ROS2, Python, or Matlab. 

1. Install Docker
    - [Instructions](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
    - Make sure to also perform the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) 


2. Clone the repo
```
git clone git@github.com:McGill-Applied-Dynamics-Lab/adl-ros2.git
```
Because the repo is private (for now...), you will probably have to 
[setup ssh with github](https://docs.github.com/en/authentication/connecting-to-github-with-ssh). 

3. Open the folder in VsCode 

4. Install the [Vs Code Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) (it's part of the [remote development extension pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)).

5. Build and open in the devcontainer
    - `>Dev Containers: Rebuild and Reopen in Container` (ctrl+shift+P to open the command palette)
    - *This takes a while as it clones the dependencies and builds the packages*

6. Test installation
    ```
    python scripts/test_install.py
    ```

## Next Steps

After successful installation:

1. Follow the [Getting Started Guide](./getting-started.md)
2. Explore the [Package Documentation](../packages/index.md)
