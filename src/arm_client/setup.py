from setuptools import find_packages, setup

package_name = 'arm_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='charles.sirois@mail.mcgill.ca',
    description='Client library and example scripts for controlling Franka FR3 using ROS2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # You can still register a clean alias if you want:
            # 'delta_move = arm_client.scripts.04_delta_move:main',
        ],
    },
    scripts=[
        'scripts/00_home.py',
        'scripts/01_switch_controller.py',
        'scripts/02_move_to.py',
        'scripts/03_figure_eight.py',
        'scripts/04_delta_move.py',   # <--- your new script
        'scripts/example.py',
        'scripts/i3_teleop_setup.py',
    ],
)
