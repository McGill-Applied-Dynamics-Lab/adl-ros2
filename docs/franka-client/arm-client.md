# Franka Client
The client is, running on the user's PC, is a Python wrapper around the ROS2 interface.

It's inspired by [crisp_py](https://github.com/utiasDSL/crisp_py?tab=readme-ov-file).

For installation, refer to the [getting started guide](../getting-started.md#4-setup-the-client)


## Debugging in VsCode
<!-- TODO: Move to dev guides -->
### Python Files
1. After building, update the env variables
```
pixi run -e humble gen-vscode-env
```

2. In the launch config
```
{
    ...
    "python": "${workspaceFolder}/.pixi/envs/humble/bin/python", // Or specify via interpreter
    "envFile": "${workspaceFolder}/.vscode/.env.humble", 
    ...
},
```
