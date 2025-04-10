from setuptools import find_packages, setup

import os
from glob import glob

package_name = "robot_tasks"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include launch files if any
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
        # Include config files if they need to be installed
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
        # IMPORTANT: Install dummy agent/config files needed for the test
        (os.path.join("share", package_name, "agents/insert"), glob(os.path.join("agents/insert", "*.pt"))),
        (
            os.path.join("share", package_name, "agents/insert/params"),
            glob(os.path.join("agents/insert/params", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="charles.sirois@mail.mcgill.ca",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Make sure your RLAgent node executable is listed
            "rl_agent = robot_tasks.RLAgent:main",
            "mock_control_mode_service = robot_tasks.mock_control_mode_service:main",
            "mock_goal_source_service = robot_tasks.mock_goal_source_service:main",
        ],
    },
)
