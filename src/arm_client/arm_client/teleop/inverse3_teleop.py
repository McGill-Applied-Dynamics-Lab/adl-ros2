import time
import numpy as np
import asyncio
import websockets
import orjson
import numpy as np
from typing import Tuple, Dict, Any, Optional
import threading
import time
from dataclasses import dataclass, field


from arm_client.teleop.base_teleop import BaseTeleop


@dataclass
class Inverse3Config:
    """
    Inverse3 Configuration.

    Configuration for the Inverse 3 haptic device.
    """

    uri: str = "ws://localhost:10001"
    translation_scale: float = 5.0
    workspace_center: list = field(default_factory=lambda: [0.0, -0.17, 0.16])
    enable_force_feedback: bool = True
    stiffness: float = 150.0
    force_cap: float = 2.0
    orientation_default: list = field(default_factory=lambda: [0.0, 1.0, 0.0, 0.0])  # Defaults downward (w, x, y, z)


def close_to_point(point_position, device_pos, thres):
    """Helper function checks if the device is close to a point."""
    distance = np.linalg.norm(device_pos - point_position)
    return distance < thres


class Inverse3Websocket:
    """WebSocket-based interface for Inverse3 haptic device."""

    def __init__(self, uri: str = "ws://localhost:10001"):
        """
        Initialize the Inverse3 websocket interface.

        Args:
            uri: WebSocket URI for the Inverse3 service
        """
        self.uri = uri
        self.device_id: Optional[str] = None
        self.handedness: Optional[str] = None

        # Current state
        self._position = np.zeros(3)
        self._velocity = np.zeros(3)
        self._force_command = np.zeros(3)

        # Connection state
        self._connected = False
        self._running = False
        self._websocket = None

        # Threading
        self._loop = None
        self._thread = None
        self._state_lock = threading.Lock()

    def start(self) -> bool:
        """
        Start the websocket connection in a separate thread.

        Returns:
            True if connection started successfully, False otherwise
        """
        if self._running:
            return True

        self._running = True
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()

        # Wait for connection to establish
        timeout = 5.0  # seconds
        start_time = time.time()
        while not self._connected and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        return self._connected

    def stop(self):
        """Stop the websocket connection."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _run_async_loop(self):
        """Run the asyncio event loop in a separate thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._websocket_loop())
        except Exception as e:
            print(f"WebSocket loop error: {e}")
        finally:
            self._loop.close()

    async def _websocket_loop(self):
        """Main websocket communication loop."""
        first_message = True

        try:
            async with websockets.connect(self.uri) as ws:
                self._websocket = ws
                self._connected = True
                print(f"Connected to Inverse3 at {self.uri}")

                while self._running:
                    try:
                        # Receive data from the device
                        response = await asyncio.wait_for(ws.recv(), timeout=0.1)
                        data = orjson.loads(response)

                        # Get devices list from the data
                        inverse3_devices = data.get("inverse3", [])

                        if not inverse3_devices:
                            if first_message:
                                print("No Inverse3 device found.")
                                break
                            continue

                        # Get the first device from the list
                        inverse3_data = inverse3_devices[0]

                        # Handle the first message to get device IDs
                        if first_message:
                            first_message = False
                            self.device_id = inverse3_data.get("device_id")
                            self.handedness = inverse3_data.get("config", {}).get("handedness")
                            print(f"Inverse3 device ID: {self.device_id}, Handedness: {self.handedness}")

                        # Extract position and velocity from device state
                        state = inverse3_data.get("state", {})
                        position_dict = state.get("cursor_position", {})
                        velocity_dict = state.get("cursor_velocity", {})

                        # Update internal state
                        with self._state_lock:
                            self._position = np.array(
                                [position_dict.get("x", 0.0), position_dict.get("y", 0.0), position_dict.get("z", 0.0)]
                            )
                            self._velocity = np.array(
                                [velocity_dict.get("x", 0.0), velocity_dict.get("y", 0.0), velocity_dict.get("z", 0.0)]
                            )

                        # Send force command
                        await self._send_force_command(ws)

                    except asyncio.TimeoutError:
                        # Continue if no message received within timeout
                        await self._send_force_command(ws)
                        continue
                    except Exception as e:
                        print(f"Error in websocket loop: {e}")
                        break

        except Exception as e:
            print(f"WebSocket connection error: {e}")
        finally:
            self._connected = False
            self._websocket = None

    async def _send_force_command(self, ws):
        """Send force command to the device."""
        if self.device_id is None:
            return

        with self._state_lock:
            force = {
                "x": float(self._force_command[0]),
                "y": float(self._force_command[1]),
                "z": float(self._force_command[2]),
            }

        request_msg = {"inverse3": [{"device_id": self.device_id, "commands": {"set_cursor_force": {"values": force}}}]}

        await ws.send(orjson.dumps(request_msg))

    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get current position and velocity of the end effector.

        Returns:
            Tuple of (position, velocity) as numpy arrays
        """
        with self._state_lock:
            return self._position.copy(), self._velocity.copy()

    def apply_force(self, force: np.ndarray):
        """
        Set the force to be applied to the end effector.

        Args:
            force: 3D force vector as numpy array
        """
        if len(force) != 3:
            raise ValueError("Force must be a 3D vector")

        with self._state_lock:
            self._force_command = np.array(force, dtype=float)

    def end_effector_force(self, force_list: list) -> Tuple[list, list]:
        """
        Legacy compatibility method matching HaplyHardwareAPI interface.

        Args:
            force_list: List of 3 force values

        Returns:
            Tuple of (position_list, velocity_list)
        """
        # Apply force
        self.apply_force(np.array(force_list))

        # Get current state
        position, velocity = self.get_state()

        return position.tolist(), velocity.tolist()

    def device_wakeup_dict(self) -> Dict[str, Any]:
        """
        Legacy compatibility method for device initialization.

        Returns:
            Dictionary with device information
        """
        return {"device_id": self.device_id, "handedness": self.handedness, "connected": self._connected}

    def is_connected(self) -> bool:
        """Check if the device is connected."""
        return self._connected


class Inverse3Teleop(BaseTeleop):
    def __init__(
        self,
        initial_robot_position: np.ndarray,
        config: Inverse3Config = Inverse3Config(),
    ):
        """
        Inverse3 Teleoperation device mapping relative cursor movements to the robot space.

        Args:
            initial_robot_position: The starting [x, y, z] of the robot end-effector.
            config: Inverse3 settings.
        """
        print(f"Initializing Inverse3 device...")

        self.config = config
        self.translation_scale = self.config.translation_scale
        self.workspace_center = np.array(self.config.workspace_center, dtype=float)
        self.enable_force_feedback = self.config.enable_force_feedback
        self.stiffness = self.config.stiffness
        self.force_cap = self.config.force_cap

        self._target_orientation = np.array(self.config.orientation_default, dtype=float)
        self._robot_initial_pos = np.array(initial_robot_position, dtype=float)

        self.i3 = Inverse3Websocket(uri=self.config.uri)
        self._i3_initial_pos = None

        # Print config
        print(f"I3 Config:")
        print(f"\t- Translation Scale: {self.translation_scale}")
        print(f"\t- Workspace Center: {self.workspace_center}")
        print(f"\t- Enable Force Feedback: {self.enable_force_feedback}")
        print(f"\t- Stiffness: {self.stiffness}")
        print(f"\t- Force Cap: {self.force_cap}")

    def start(self) -> None:
        """Start the Inverse3 connection and perform clutching sequence."""
        print("\tConnecting to Inverse3 device via WebSocket...")
        if not self.i3.start():
            raise RuntimeError("Failed to connect to Inverse3!")
        print("\tConnected to Inverse3 device!")

        print("\tMoving device to the center of the workspace region...")
        dt = 0.01
        i3_at_center = False
        while not i3_at_center and self.i3.is_connected():
            pos, _ = self.i3.get_state()
            if close_to_point(self.workspace_center, pos, 0.015):
                i3_at_center = True
                self.i3.apply_force(np.zeros(3))
            else:
                # Apply proportional force to pull cursor to center
                direction = self.workspace_center - pos
                distance = np.linalg.norm(direction)
                dir_norm = direction / distance if distance > 0 else np.zeros(3)

                # Gentle pull towards center
                force = dir_norm * (distance * 50.0)

                # Cap force
                force_norm = np.linalg.norm(force)
                if force_norm > 1.5:  # Safe centering cap
                    force = (force / force_norm) * 1.5

                self.i3.apply_force(force)
            time.sleep(dt)

        print("\tInverse3 initialized! Ready for teleoperation.")
        self._i3_initial_pos, _ = self.i3.get_state()

    def stop(self) -> None:
        """Stop device tracking and remove all forces."""
        self.i3.apply_force(np.zeros(3))
        self.i3.stop()

    @property
    def target_position(self) -> np.ndarray:
        """Returns the relative target workspace position mapped to the robot."""
        if not self.i3.is_connected() or self._i3_initial_pos is None:
            return self._robot_initial_pos

        pos, _ = self.i3.get_state()
        delta_i3 = pos - self._i3_initial_pos

        # Note: Depending on Inverse3 physical setup versus the Franka base frame,
        # cross-axis configurations might be needed here (e.g. mapping X to Y).
        # We start with a direct uniform map in all 3 axes.
        delta_franka = delta_i3 * self.translation_scale
        return self._robot_initial_pos + delta_franka

    @property
    def target_orientation(self) -> np.ndarray:
        """Returns the fixed target orientation in (w, x, y, z)."""
        return self._target_orientation

    def provide_feedback(self, actual_robot_pos: np.ndarray) -> None:
        """
        Calculates and applies the virtual coupling force acting as an impedance spring
        based on the physical robot positional lag versus the target.
        """
        if not self.i3.is_connected() or not self.enable_force_feedback:
            self.i3.apply_force(np.zeros(3))
            return

        current_i3_pos, _ = self.i3.get_state()

        # Map actual robot position back down into the Inverse3 scale.
        # This tells us where the Inverse3 'should' feel the physical boundary.
        actual_franka_delta = actual_robot_pos - self._robot_initial_pos
        target_i3_pos = self._i3_initial_pos + (actual_franka_delta / self.translation_scale)

        # Virtual spring pulling I3 physically towards the real target limit
        error_i3 = target_i3_pos - current_i3_pos
        force = error_i3 * self.stiffness

        # Basic force capping for safety
        force_norm = np.linalg.norm(force)
        if force_norm > self.force_cap:
            force = (force / force_norm) * self.force_cap

        self.i3.apply_force(force)
