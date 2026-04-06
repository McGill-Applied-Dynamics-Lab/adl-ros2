# Franka Client
The client is running on the user's PC. It provides a python wrapper around ROS2.

## Installation

1. Install pixi
https://pixi.prefix.dev/latest/installation/

2. 
```
pixi run setup-colcon
```

3. 
```
pixi shell -e humble
```

4. To build
```
pixi run -e humble build     
```

5. To run a file
```

```

## Debugging in VsCode
Need some fiddling to pass the correct variables to the debugger. 

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
