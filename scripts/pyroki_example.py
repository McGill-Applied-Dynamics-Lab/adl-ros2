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


def solve_ik(
    robot: pk.Robot,
    target_link_name: str,
    target_wxyz: onp.ndarray,
    target_position: onp.ndarray,
) -> onp.ndarray:
    """
    Solves the basic IK problem for a robot.

    Args:
        robot: PyRoKi Robot.
        target_link_name: String name of the link to be controlled.
        target_wxyz: onp.ndarray. Target orientation.
        target_position: onp.ndarray. Target position.

    Returns:
        cfg: onp.ndarray. Shape: (robot.joint.actuated_count,).
    """
    assert target_position.shape == (3,) and target_wxyz.shape == (4,)
    target_link_index = robot.links.names.index(target_link_name)
    cfg = _solve_ik_jax(
        robot,
        jnp.array(target_link_index),
        jnp.array(target_wxyz),
        jnp.array(target_position),
    )
    assert cfg.shape == (robot.joints.num_actuated_joints,)
    return onp.array(cfg)


@jdc.jit
def _solve_ik_jax(
    robot: pk.Robot,
    target_link_index: jax.Array,
    target_wxyz: jax.Array,
    target_position: jax.Array,
) -> jax.Array:
    joint_var = robot.joint_var_cls(0)
    variables = [joint_var]
    costs = [
        pk.costs.pose_cost_analytic_jac(
            robot,
            joint_var,
            jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(target_wxyz), target_position),
            target_link_index,
            pos_weight=50.0,
            ori_weight=10.0,
        ),
        pk.costs.limit_constraint(
            robot,
            joint_var,
        ),
    ]
    sol = (
        jaxls.LeastSquaresProblem(costs=costs, variables=variables)
        .analyze()
        .solve(
            verbose=False,
            linear_solver="dense_cholesky",
            trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
        )
    )
    return sol[joint_var]


def main():
    """Main function for basic IK."""

    urdf = load_robot_description("panda_description")
    # urdf = get_fr3_urdf()

    target_link_name = "panda_hand"
    # target_link_name = "fr3_hand_tcp"

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

    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    while True:
        # Solve IK.

        start_time = time.time()

        solution = solve_ik(
            robot=robot,
            target_link_name=target_link_name,
            target_position=np.array(ik_target.position),
            target_wxyz=np.array(ik_target.wxyz),
        )

        # Update timing handle.

        elapsed_time = time.time() - start_time

        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

        # Update visualizer.

        urdf_vis.update_cfg(solution)


if __name__ == "__main__":
    main()
