#!/usr/bin/env python3
"""
Verify that the pixi environment and ROS2 installation are working correctly.

Usage: pixi run python scripts/test_install.py
"""

import importlib
import os
import subprocess
import sys


def run_command(cmd):
    """Run a command and return (success, output)."""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        return result.returncode == 0, result.stdout.strip()
    except Exception as e:
        return False, str(e)


def test_environment():
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "unknown")

    if env_name == "default":
        print("⚠️  You are running in the default pixi environment.")
        print("    Please run with a specific environment, e.g.:")
        print("    pixi run -e humble test-install")
        return False

    print(f"Environment: {env_name}\n")

    tests = []

    # --- Pixi environment
    tests.append(("Running inside pixi environment", lambda: "PIXI_ENVIRONMENT_NAME" in os.environ))
    tests.append(("Correct pixi environment (humble)", lambda: os.environ.get("PIXI_ENVIRONMENT_NAME") == "humble"))

    # --- ROS2
    tests.append(("ROS_DISTRO is humble", lambda: os.environ.get("ROS_DISTRO") == "humble"))
    tests.append(("ros2 CLI available", lambda: run_command("which ros2")[0]))
    tests.append(("colcon available", lambda: run_command("which colcon")[0]))

    # --- Python packages
    for pkg in ["rclpy", "std_msgs", "geometry_msgs", "sensor_msgs"]:
        tests.append((f"Python import: {pkg}", lambda p=pkg: importlib.import_module(p) is not None))

    # --- Workspace build
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    install_base = os.path.join(os.getcwd(), f"install_{ros_distro}")
    tests.append((f"Workspace built (install_{ros_distro}/ exists)", lambda: os.path.isdir(install_base)))
    tests.append(("Workspace sourced (AMENT_PREFIX_PATH set)", lambda: bool(os.environ.get("AMENT_PREFIX_PATH"))))
    tests.append(("arm_client importable (arm_client)", lambda: importlib.import_module("arm_client") is not None))

    # --- Run
    print("Verifying pixi + ROS2 environment...\n")
    passed = 0
    failed_names = []

    for name, fn in tests:
        try:
            ok = fn()
        except Exception as e:
            ok = False
            name = f"{name} ({e})"
        print(f"  {'✓' if ok else '✗'} {name}")
        if ok:
            passed += 1
        else:
            failed_names.append(name)

    total = len(tests)
    print(f"\nResults: {passed}/{total} passed")

    if passed == total:
        print("🎉 Environment looks good — ready to run and debug!")
    else:
        print("⚠️  Some checks failed:")
        for name in failed_names:
            print(f"   • {name}")
        print("\nTip: make sure you ran `pixi run -e humble build` and `pixi run gen-vscode-env` first.")

    return passed == total


if __name__ == "__main__":
    sys.exit(0 if test_environment() else 1)
