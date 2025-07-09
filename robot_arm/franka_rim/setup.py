from setuptools import find_packages, setup
import os

package_name = "franka_rim"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    # Robot model files
    (
        "share/" + package_name + "/models",
        [f"models/{f}" for f in os.listdir("models") if os.path.isfile(os.path.join("models", f))],
    ),
    # rviz configuration files
    (
        "share/" + package_name + "/rviz",
        [f"rviz/{f}" for f in os.listdir("rviz") if os.path.isfile(os.path.join("rviz", f))],
    ),
]


def get_all_files_recursive(src_dir, dst_prefix):
    file_list = []
    for root, _, files in os.walk(src_dir):
        for f in files:
            abs_path = os.path.join(root, f)
            rel_path = os.path.relpath(abs_path, src_dir)
            file_list.append((os.path.join(dst_prefix, rel_path), abs_path))
    return file_list


def add_data_files_recursive(data_files, src_root, dst_root):
    for dirpath, _, filenames in os.walk(src_root):
        if filenames:
            rel_dir = os.path.relpath(dirpath, src_root)
            dst_dir = os.path.join(dst_root, rel_dir) if rel_dir != "." else dst_root
            files = [os.path.join(dirpath, f) for f in filenames]
            data_files.append((dst_dir, files))


# Add meshes files
meshes_dir = os.path.join("models", "meshes")
if os.path.isdir(meshes_dir):
    add_data_files_recursive(data_files, meshes_dir, f"share/{package_name}/models/meshes")

# Add launch files
launch_dir = "launch"
if os.path.isdir(launch_dir):
    add_data_files_recursive(data_files, launch_dir, f"share/{package_name}/launch")

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="charles.sirois@mail.mcgill.ca",
    description="Compute Reduced Interface Model (RIM) for Franka Research 3",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "franka_model_node = franka_rim.franka_model_node:main",
            "franka_rim_node = franka_rim.franka_rim_node:main",
            "delay_rim_node = franka_rim.delay_rim_node:main",
            "simple_mass_system_node = franka_rim.simple_mass_system_node:main",
        ],
    },
    package_data={"franka_rim": ["models/*.urdf"]},
    include_package_data=True,
)
