from glob import glob
import os

from setuptools import find_packages, setup

package_name = "arm_client_rim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *[
            (os.path.join("share", package_name, os.path.dirname(f)), [f])
            for f in glob("config/**", recursive=True)
            if os.path.isfile(f)
        ],
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Charles Sirois",
    maintainer_email="charles.sirois@mail.mcgill.ca",
    description="RIM-based bilateral teleoperation integration using arm_client",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rim_teleop = arm_client_rim.main:main",
        ],
    },
)
