# Franka Server Controllers
How to modify and add new controllers.

The main config file: `franka_server/config/controllers.yaml`
Used to specify the ROS2 controllers.

## Available Controllers
To list all available controllers (with the server running):
```
ros2 control list_controllers -c /fr3/controller_manager
```


[**CRISP Controllers**](https://utiasdsl.github.io/crisp_controllers/)

- 


## Adding a new Controller

x. Add the controller to the config file

x. Add the controller name to the launch file so it's loaded
In `franka_server/launch/franka.launch.py:206`
```
  controllers_list = [
        # --- Broadcasters
        ...
        ["new_controller_name", "--inactive"],
    ]
```

x. When the server is running, you can check the 