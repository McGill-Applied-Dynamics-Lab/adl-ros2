from typing import Any, List, Literal, Tuple
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# from isaac_ros2_messages.srv import GetPrimAttribute, SetPrimAttribute

# Interfaces
import rclpy.time
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    SwitchController,
)

from controller_manager_msgs.msg import ControllerState


from enum import Enum


class ControllerStateEnum(Enum):
    UNCONFIGURED = "unconfigured"
    INACTIVE = "inactive"
    ACTIVE = "active"
    FINALIZED = "finalized"


class FeedbackController:
    def __init__(self, name: str, state: ControllerStateEnum, type: str):
        self.name = name
        self.state = state
        self.type = type

    @classmethod
    def from_controller_state_msg(cls, controller_state: ControllerState):
        ctrl_name = controller_state.name
        ctrl_state = ControllerStateEnum(controller_state.state)
        type = controller_state.type

        return cls(ctrl_name, ctrl_state, type)


class FeedbackControllerManager:
    def __init__(self, node: Node, cb_group: MutuallyExclusiveCallbackGroup):
        self._node = node
        self._node.get_logger().info("Initializing Feedback Controller Manager...")

        self._always_active_controllers = ["franka_robot_state_broadcaster", "joint_state_broadcaster"]

        self._controllers: dict | None = None
        self._activated_controllers = []
        self._cb_group = cb_group

        #! Services
        self._list_controllers_srv = None
        self._switch_controller_srv = None
        self._init_services()

        # await self._get_cm_controllers()

    def _init_services(self):
        self._service_list = []

        # Service - List controllers
        self._list_controllers_srv = self._node.create_client(
            ListControllers, "/controller_manager/list_controllers", callback_group=self._cb_group
        )
        self._service_list.append(self._list_controllers_srv)

        # Service - Switch controller
        self._switch_controller_srv = self._node.create_client(
            SwitchController, "/controller_manager/switch_controller", callback_group=self._cb_group
        )
        self._service_list.append(self._switch_controller_srv)

        # TODO: FIX FOR ISAACSIM
        # # IsaacSim services if hw_type is isaac
        # if self._node.hw_type == "isaac":
        #     self._get_prim_attribute_srv = self._node.create_client(
        #         GetPrimAttribute, "/get_prim_attribute", callback_group=self._cb_group
        #     )
        #     self._service_list.append(self._get_prim_attribute_srv)

        #     self._set_prim_attribute_srv = self._node.create_client(
        #         SetPrimAttribute, "/set_prim_attribute", callback_group=self._cb_group
        #     )

        # Wait for services
        self._node.get_logger().info("Waiting cm services...")
        for srv in self._service_list:
            while not srv.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().info(f"service {srv.srv_name} not available, waiting again...")

        self._node.get_logger().info("CM services are up!")

    async def _get_cm_controllers(self):
        """
        Get a list of controllers from the controller manager
        """
        self._node.get_logger().info("Getting list of controllers from cm...")

        self._controllers = {}

        req = ListControllers.Request()
        future = self._list_controllers_srv.call_async(req)
        # rclpy.spin_until_future_complete(self._node, future)

        result = await future

        if result is not None:
            self._activated_controllers = []
            controllers = result.controller

            for controller_msg in controllers:
                controller = FeedbackController.from_controller_state_msg(controller_msg)

                self._controllers[controller.name] = controller

                if (
                    controller.state == ControllerStateEnum.ACTIVE
                    and controller.name not in self._always_active_controllers
                ):
                    self._activated_controllers.append(controller.name)

        else:
            self._node.get_logger().error("Service call failed!")

    async def switch_controller(self, controller_name: str):
        """
        To switch to a controller
        """
        self._node.get_logger().info(f"Switching to controller: {controller_name}")

        await self._get_cm_controllers()

        if controller_name not in self._controllers:
            self._node.get_logger().error(f"Controller: {controller_name} is not available!")

        if self._controllers[controller_name].state == ControllerStateEnum.ACTIVE:
            if len(self._activated_controllers) == 1:
                self._node.get_logger().info(f"Controller: {controller_name} is already active!")
                return

        #! Switch controller
        if self._node.hw_type == "isaac":
            self._node.get_logger().info("Setting Isaac Sim drive gains...")
            if controller_name == "joint_trajectory_controller":
                await self._set_isaac_drive_gains("position")
            elif controller_name == "my_vel_controller":
                await self._set_isaac_drive_gains("velocity")

        req = SwitchController.Request()

        # Set message
        req.activate_controllers = [controller_name]

        if self._activated_controllers is not None:
            req.deactivate_controllers = self._activated_controllers

        req.strictness = 1  # BEST_EFFORT=1, STRICT=2
        # req.activate_asap
        req.timeout = Duration(seconds=1).to_msg()

        # Call service
        future = self._switch_controller_srv.call_async(req)
        result = await future

        if result is not None:
            switch_ok = result.ok

            if switch_ok:
                self._node.get_logger().info(f"Switched to controller: {controller_name}")
                self._activated_controller = controller_name

            else:
                self._node.get_logger().error(f"Failed to switch to controller: {controller_name}")

        else:
            self._node.get_logger().error("Service call failed!")

    async def _set_isaac_drive_gains(self, ctrl_type: Literal["position", "velocity"]):
        self._node.get_logger().info(f"Setting Isaac Sim drive gains for {ctrl_type}")

        base_prim_path = "/World/franka_alt_fingers"
        damping_attr = "drive:angular:physics:damping"
        stiffness_attr = "drive:angular:physics:stiffness"

        GAINS = {
            # '<prim>': ([position_damping, position_stiffness], [velocity_damping, velocity_stiffness])
            "/panda_link0/panda_joint1": ([52, 1050], [500, 0]),
            "/panda_link1/panda_joint2": ([52, 1050], [500, 0]),
            "/panda_link2/panda_joint3": ([52, 1050], [500, 0]),
            "/panda_link3/panda_joint4": ([52, 436], [500, 0]),
            "/panda_link4/panda_joint5": ([52, 436], [500, 0]),
            "/panda_link5/panda_joint6": ([52, 261], [500, 0]),
            "/panda_link6/panda_joint7": ([52, 87], [500, 0]),
        }

        for each_joint, gains in GAINS.items():
            srv_req = SetPrimAttribute.Request()
            srv_req.path = base_prim_path + each_joint

            # Damping
            srv_req.attribute = damping_attr
            val = gains[0][0] if ctrl_type == "position" else gains[1][0]
            srv_req.value = str(val)

            future = self._set_prim_attribute_srv.call_async(srv_req)
            result = await future

            if result is not None:
                if result.success:
                    self._node.get_logger().info(f"Set {ctrl_type} damping for {each_joint} to {val}")

                else:
                    self._node.get_logger().error(f"Failed to set {ctrl_type} damping for {each_joint}")
                    return

            else:
                self._node.get_logger().error("Service call failed!")
                return

            # Stiffness
            srv_req.attribute = stiffness_attr
            val = gains[0][1] if ctrl_type == "position" else gains[1][1]
            srv_req.value = str(val)

            future = self._set_prim_attribute_srv.call_async(srv_req)
            result = await future

            if result is not None:
                if result.success:
                    self._node.get_logger().info(f"Set {ctrl_type} stiffness for {each_joint} to {val}")

                else:
                    self._node.get_logger().error(f"Failed to set {ctrl_type} stiffness for {each_joint}")
                    return

            else:
                self._node.get_logger().error("Service call failed!")
                return
