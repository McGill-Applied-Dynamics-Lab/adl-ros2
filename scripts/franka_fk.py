import time


import numpy as np

import pyroki as pk


from robot_descriptions.loaders.yourdfpy import load_robot_description

import yourdfpy


import viser
from viser.extras import ViserUrdf

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

import jax
import jax.numpy as jnp
import jax_dataclasses as jdc
import jaxlie
import jaxls
import numpy as onp


JOINTS = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def get_fr3_urdf() -> yourdfpy.URDF:
    """
    Resolve the FR3 URDF path from the franka_description package.
    You need to pre-generate the URDF from xacro once:

        xacro $(ros2 pkg prefix franka_description)/share/franka_description/ \\
              robots/fr3/fr3.urdf.xacro hand:=true > /tmp/fr3.urdf

    Or point this at wherever you keep your generated URDF.
    """
    share = get_package_share_directory("franka_rim")
    # If you have a pre-generated URDF:
    models_dir = Path(share) / "models"
    urdf_path = models_dir / "fr3_franka_hand.urdf"
    meshes_dir = models_dir / "meshes"

    if not urdf_path.exists():
        raise FileNotFoundError(
            f"URDF not found at {urdf_path}. "
            "Generate it first:\n"
            "  xacro franka_description/robots/fr3/fr3.urdf.xacro "
            "hand:=true > fr3.urdf"
        )

    urdf = yourdfpy.URDF.load(
        urdf_path,
        mesh_dir=meshes_dir,
    )

    return urdf


def main():
    """Main function for basic IK."""

    urdf = get_fr3_urdf()
    target_link_name = "fr3_hand_tcp"

    # Create robot.
    robot = pk.Robot.from_urdf(urdf)

    # Set up visualizer.

    server = viser.ViserServer()

    server.scene.add_grid("/ground", width=2, height=2)

    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    # Create interactive controller with initial position.

    ik_target = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=(0.61, 0.0, 0.56), wxyz=(0, 0, 1, 0)
    )

    joint_values = JOINTS

    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    while True:
        # Solve IK.

        start_time = time.time()

        # solution = solve_ik(
        #     robot=robot,
        #     target_link_name=target_link_name,
        #     target_position=np.array(ik_target.position),
        #     target_wxyz=np.array(ik_target.wxyz),
        # )
        sol = np.hstack([JOINTS, 0.04])

        # Update timing handle.

        elapsed_time = time.time() - start_time

        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

        # Update visualizer.

        urdf_vis.update_cfg(sol)


if __name__ == "__main__":
    main()
