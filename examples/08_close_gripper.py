"""Open then close the Franka gripper (end effector)."""

import time

from arm_client.gripper.franka_hand import Gripper

CLOSE_DETECTION_TOLERANCE_M = 0.0005


def main():
    gripper = Gripper(namespace="fr3")
    try:
        print("Waiting for gripper to be ready...")
        gripper.wait_until_ready(timeout=5.0)

        print(f"Current width: {gripper.value:.4f} m")
        print("Opening gripper...")
        opened = gripper.open(block=True)
        if opened:
            time.sleep(0.5)
            print(f"Opened successfully. Width: {gripper.value:.4f} m")
        else:
            print(f"Open action failed. Current width: {gripper.value:.4f} m")
            return

        width_before_close = gripper.value
        print("Closing gripper fully...")
        closed = gripper.close(block=True)
        if closed:
            time.sleep(0.5)
            final_width = gripper.value
            closing_distance = max(0.0, width_before_close - final_width)
            print(f"Closed successfully. Width: {final_width:.4f} m")
            print(f"Closing distance: {closing_distance:.4f} m")
            is_closed_within_tolerance = final_width <= CLOSE_DETECTION_TOLERANCE_M
            print(
                f"Close detection (<= {CLOSE_DETECTION_TOLERANCE_M:.4f} m): "
                f"{'PASS' if is_closed_within_tolerance else 'FAIL'}"
            )
        else:
            print(f"Close action failed. Current width: {gripper.value:.4f} m")
    finally:
        gripper.shutdown()


if __name__ == "__main__":
    main()
