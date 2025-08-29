from setuptools import find_packages, setup

import os
from glob import glob

package_name = "robot_tasks"

agent_files = []
for root, dirs, files in os.walk("agents"):
    for file in files:
        file_path = os.path.join(root, file)
        dest_path = os.path.join("share", package_name, root)
        agent_files.append((dest_path, [file_path]))

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
        # (os.path.join("share", package_name, "agents/insert"), glob(os.path.join("agents/insert", "*.pt"))),
        # (
        #     os.path.join("share", package_name, "agents/insert/params"),
        #     glob(os.path.join("agents/insert/params", "*.yaml")),
        # ),
        # Include all files in the 'agents' folder
        # (os.path.join("share", package_name, "agents"), glob(os.path.join("agents", "**"), recursive=True)),
    ]
    + agent_files,
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
            "insert_rl_agent = robot_tasks.rl_agent.Insert:main",
            "lift_rl_agent = robot_tasks.rl_agent.Lift:main",
            "mock_control_mode_service = robot_tasks.mock.mock_control_mode_service:main",
            "mock_goal_source_service = robot_tasks.mock.mock_goal_source_service:main",
        ],
    },
)
