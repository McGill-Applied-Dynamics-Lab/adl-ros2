#!/usr/bin/env python3
"""
Simple test script to verify ROS2 installation is working.

Usage: python3 scripts/test_install.py
"""

import os
import subprocess
import sys


def run_command(cmd):
    """Run a command and return True if successful."""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        return result.returncode == 0
    except Exception:
        return False


def test_ros2_installation():
    """Test basic ROS2 installation."""
    print("Testing ROS2 installation...")

    tests = [
        ("ROS_DISTRO environment variable", lambda: os.environ.get("ROS_DISTRO") == "humble"),
        ("ros2 command available", lambda: run_command("which ros2")),
        ("ROS2 nodes accessible", lambda: run_command("ros2 node list")),
        ("colcon build tool available", lambda: run_command("which colcon")),
        ("Python rclpy import", lambda: __import__("rclpy") is not None),
        ("Workspace sourced", lambda: os.path.exists("/home/ros/ros2_ws/install/setup.bash")),
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        try:
            result = test_func()
            status = "✓ PASS" if result else "✗ FAIL"
            print(f"  {status} {test_name}")
            if result:
                passed += 1
        except Exception as e:
            print(f"  ✗ FAIL {test_name} - {e}")

    print(f"\nResults: {passed}/{total} tests passed")

    if passed == total:
        print("🎉 ROS2 installation looks good!")
        return True
    else:
        print("⚠️  Some issues found. Check the failed tests above.")
        return False


if __name__ == "__main__":
    success = test_ros2_installation()
    sys.exit(0 if success else 1)
