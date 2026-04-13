# Installation
Page on how to install the libraries setup the network to run the franka server on the mini desktop.

!!! tip "Glossary"
  
    - **Franka Control** / **Controller**: Black Box on the table
    - **Franka PC** / **Desktop**: Franka PC on the ground
    - **Franka Desk**: Web app to control the robot
    - **FR3-server**: Set of ros2 nodes to publish the robot states and send position commands. Running on the *Franka PC*.

## Network Setup
<figure markdown="span">
  ![test](../media/figures/network_architecture.png){ width="500" }
  <figcaption>Network Setup</figcaption>
</figure>

TODO: Steps to setup network, install FCI and the server. 


### Testing `libfranka`
Libfranka is the low level c++ api to send commands to the robot. 
Test that you are able to connect to the robot:
```
cd ~/git/libfranka/build
./examples/echo_robot_state $FR3_IP
```

To check the realtime is working:
```
# This moves the robot home
./examples/communication_test $FR3_IP
```

Other examples are available. You can take a look with 
```
ls examples
```