#!/usr/bin/env python3
"""
Simple test script for Inverse3 WebSocket interface.

This script provides a basic test of the Inverse3 class without visualization dependencies.
It connects to the device and prints position/velocity data to the console.

Usage:
    python simple_inverse3_test.py [--uri ws://localhost:10001] [--duration 30]
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path

from inverse3_ros2.websocket_inverse3 import Inverse3


def test_basic_connection(uri: str):
    """Test basic connection and device information."""
    print("Testing basic connection...")

    inverse3 = Inverse3(uri)

    if not inverse3.start():
        print("❌ Failed to connect to device")
        return False

    print("✅ Connected to device")

    # Get device info
    device_info = inverse3.device_wakeup_dict()
    print(f"Device ID: {device_info.get('device_id')}")
    print(f"Handedness: {device_info.get('handedness')}")
    print(f"Connected: {device_info.get('connected')}")

    inverse3.stop()
    return True


def test_state_reading(uri: str, duration: float = 5.0):
    """Test reading position and velocity data."""
    print(f"\nTesting state reading for {duration} seconds...")

    inverse3 = Inverse3(uri)

    if not inverse3.start():
        print("❌ Failed to connect to device")
        return False

    print("✅ Connected, reading state data...")
    print("Move the haptic device to see position changes...")
    print()

    start_time = time.time()
    positions = []
    velocities = []

    try:
        while (time.time() - start_time) < duration:
            position, velocity = inverse3.get_state()
            positions.append(position)
            velocities.append(velocity)

            # Print current state
            print(
                f"Position: [{position[0]:+6.4f}, {position[1]:+6.4f}, {position[2]:+6.4f}] m  "
                f"Velocity: [{velocity[0]:+6.4f}, {velocity[1]:+6.4f}, {velocity[2]:+6.4f}] m/s  "
                f"Speed: {np.linalg.norm(velocity):6.4f} m/s",
                end="\r",
            )

            time.sleep(0.1)  # 10 Hz update rate

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        inverse3.stop()

    # Calculate statistics
    if positions:
        positions_array = np.array(positions)
        velocities_array = np.array(velocities)

        print(f"\n\nStatistics over {len(positions)} samples:")
        print(
            f"Position range: X[{np.min(positions_array[:, 0]):.4f}, {np.max(positions_array[:, 0]):.4f}] "
            f"Y[{np.min(positions_array[:, 1]):.4f}, {np.max(positions_array[:, 1]):.4f}] "
            f"Z[{np.min(positions_array[:, 2]):.4f}, {np.max(positions_array[:, 2]):.4f}] m"
        )
        print(f"Max speed: {np.max(np.linalg.norm(velocities_array, axis=1)):.4f} m/s")
        print(f"Average speed: {np.mean(np.linalg.norm(velocities_array, axis=1)):.4f} m/s")

    return True


def test_force_feedback(uri: str, duration: float = 5.0):
    """Test force feedback functionality."""
    print(f"\nTesting force feedback for {duration} seconds...")

    inverse3 = Inverse3(uri)

    if not inverse3.start():
        print("❌ Failed to connect to device")
        return False

    print("✅ Connected, applying sinusoidal force in X direction...")
    print("You should feel a gentle force pulling/pushing in the X direction")
    print()

    start_time = time.time()
    force_amplitude = 1  # N
    frequency = 0.5  # Hz

    try:
        while (time.time() - start_time) < duration:
            current_time = time.time() - start_time

            # Apply sinusoidal force in X direction
            force_y = force_amplitude * np.sin(2 * np.pi * frequency * current_time)
            force = np.array([0.0, force_y, 0.0])
            inverse3.apply_force(force)

            # Get current state
            position, velocity = inverse3.get_state()

            print(
                f"Time: {current_time:5.2f}s  Force: [{force[0]:+6.3f}, {force[1]:+6.3f}, {force[2]:+6.3f}] N  "
                f"Position: [{position[0]:+6.4f}, {position[1]:+6.4f}, {position[2]:+6.4f}] m",
                end="\r",
            )

            time.sleep(0.02)  # 50 Hz update rate

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        # Reset forces to zero
        inverse3.apply_force(np.zeros(3))
        time.sleep(0.1)
        inverse3.stop()

    print("\n✅ Force feedback test completed")
    return True


def test_legacy_interface(uri: str):
    """Test the legacy HaplyHardwareAPI-compatible interface."""
    print(f"\nTesting legacy interface compatibility...")

    inverse3 = Inverse3(uri)

    if not inverse3.start():
        print("❌ Failed to connect to device")
        return False

    print("✅ Connected, testing legacy methods...")

    # Test device_wakeup_dict
    wakeup_dict = inverse3.device_wakeup_dict()
    print(f"device_wakeup_dict(): {wakeup_dict}")

    # Test end_effector_force method
    test_force = [0.1, 0.0, 0.0]
    position_list, velocity_list = inverse3.end_effector_force(test_force)
    print(f"end_effector_force({test_force}) -> position: {position_list}, velocity: {velocity_list}")

    # Reset force and test again
    zero_force = [0.0, 0.0, 0.0]
    position_list, velocity_list = inverse3.end_effector_force(zero_force)
    print(f"end_effector_force({zero_force}) -> position: {position_list}, velocity: {velocity_list}")

    inverse3.stop()
    print("✅ Legacy interface test completed")
    return True


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Simple test for Inverse3 WebSocket interface")
    parser.add_argument("--uri", default="ws://localhost:10001", help="WebSocket URI (default: ws://localhost:10001)")
    parser.add_argument("--duration", type=float, default=5.0, help="Test duration in seconds (default: 5.0)")
    parser.add_argument(
        "--test",
        choices=["connection", "state", "force", "legacy", "all"],
        default="all",
        help="Test to run (default: all)",
    )

    args = parser.parse_args()

    print("Inverse3 WebSocket Simple Test")
    print("=" * 30)
    print(f"URI: {args.uri}")
    print(f"Duration: {args.duration}s")
    print(f"Test: {args.test}")
    print()

    tests_to_run = []
    if args.test == "all":
        tests_to_run = ["connection", "state", "force", "legacy"]
    else:
        tests_to_run = [args.test]

    results = {}

    for test_name in tests_to_run:
        try:
            if test_name == "connection":
                results[test_name] = test_basic_connection(args.uri)
            elif test_name == "state":
                results[test_name] = test_state_reading(args.uri, args.duration)
            elif test_name == "force":
                results[test_name] = test_force_feedback(args.uri, args.duration)
            elif test_name == "legacy":
                results[test_name] = test_legacy_interface(args.uri)
        except Exception as e:
            print(f"❌ Test '{test_name}' failed with error: {e}")
            results[test_name] = False

    # Summary
    print(f"\n{'=' * 30}")
    print("Test Results Summary:")
    for test_name, success in results.items():
        status = "✅ PASSED" if success else "❌ FAILED"
        print(f"  {test_name}: {status}")

    all_passed = all(results.values())
    print(f"\nOverall: {'✅ ALL TESTS PASSED' if all_passed else '❌ SOME TESTS FAILED'}")

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
