import asyncio
import websockets
import orjson
import numpy as np
from typing import Tuple, Dict, Any, Optional
import threading
import time


class Inverse3:
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
