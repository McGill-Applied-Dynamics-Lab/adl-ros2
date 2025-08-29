import numpy as np
from scipy.spatial.transform import Rotation
import random

# Mass matrix for a cylinder in 3d
def mass_matrix_rod(mass_density, rod_length, rod_radius, cur_R):
    rod_mass = mass_density * np.pi * rod_radius**2 * rod_length
    # "Products" of inertia around com
    Ixx = (1. / 12.) * rod_mass * (3 * rod_radius**2 + rod_length**2)
    Iyy = (1. / 2.) * rod_mass * rod_radius**2
    Izz = (1. / 12.) * rod_mass * (3 * rod_radius**2 + rod_length**2)
    I = np.array([
        [Ixx, 0., 0.],
        [0., Iyy, 0.],
        [0., 0., Izz]
    ])
    cur_I = Rotation.as_matrix(cur_R) @ I @ np.transpose(Rotation.as_matrix(cur_R))
    cur_mass_matrix = np.block([
        [rod_mass * np.identity(3), np.zeros((3, 3))],
        [np.zeros((3, 3)), cur_I]
    ])

    return cur_mass_matrix

def mass_matrix_sphere(mass_density, sphere_radius, cur_R):
    sphere_mass = mass_density * np.pi * sphere_radius**2
    Ixx = (2. / 5.) * sphere_mass * sphere_radius**2
    I = np.array([
        [Ixx, 0., 0.],
        [0., Ixx, 0.],
        [0., 0., Ixx]
    ])
    cur_I = Rotation.as_matrix(cur_R) @ I @ np.transpose(Rotation.as_matrix(cur_R))
    cur_mass_matrix = np.block([
        [sphere_mass * np.identity(3), np.zeros((3,3))],
        [np.zeros((3,3)), cur_I]
    ])

    return cur_mass_matrix

def system_mass_matrix(params):

    mass = np.block([
        [params.mass_matrix_drone, np.zeros((6,18))],
        [np.zeros((6,6)), params.mass_matrix_rod1, np.zeros((6,12))],
        [np.zeros((6,12)), params.mass_matrix_rod2, np.zeros((6,6))],
        [np.zeros((6,18)), params.mass_matrix_load]
    ])

    return mass

# Rotation matrix about the axis set by the angular velocity
def new_rotation_matrix(cur_R, angular_velocity, h):
    if np.linalg.norm(angular_velocity) != 0:
        angular_velocity.shape = (3, 1)
        theta = h * angular_velocity
        vec = (1.0 / np.linalg.norm(theta)) * np.transpose(theta) * np.sin(np.linalg.norm(theta) / 2.)
        scalar = np.cos(np.linalg.norm(theta) / 2.)
        quaternion = [vec[0, 0], vec[0, 1], vec[0, 2], scalar]
        rotation_matrix_from_quaternion = Rotation.from_quat(quaternion)
        rotation_matrix = rotation_matrix_from_quaternion * cur_R
    else:
        rotation_matrix = cur_R

    return rotation_matrix

def noninertial_terms_rod(mass_density, rod_radius, rod_length, angular_velocity):
    rod_mass = mass_density * np.pi * rod_radius**2 * rod_length
    # "Products" of inertia around com
    Ixx = (1. / 12.) * rod_mass * (3 * rod_radius**2 + rod_length**2)
    Iyy = (1. / 2.) * rod_mass * rod_radius**2
    Izz = (1. / 12.) * rod_mass * (3 * rod_radius**2 + rod_length**2)
    I = np.array([
        [Ixx, 0., 0.],
        [0., Iyy, 0.],
        [0., 0., Izz]
    ])

    noninertial_forces = cross_prod(angular_velocity, I @ angular_velocity)
    noninertial_forces.shape = (3, 1)

    return noninertial_forces

def noninertial_terms_sphere(mass_density, sphere_radius, angular_velocity):
    sphere_mass = mass_density * np.pi * sphere_radius**2
    Ixx = (2. / 5.) * sphere_mass * sphere_radius**2
    I = np.array([
        [Ixx, 0., 0.],
        [0., Ixx, 0.],
        [0., 0., Ixx]
    ])

    noninertial_forces = cross_prod(angular_velocity, I @ angular_velocity)
    noninertial_forces.shape = (3, 1)

    return noninertial_forces

def skew_symmetric_matrix_from_vector(v):
    if np.linalg.norm(v) != 0:
        v.shape = (3, 1)
        skew_matrix = np.array([
            [0., -v[2,0], v[1,0]],
            [v[2,0], 0., -v[0,0]],
            [-v[1,0], v[0,0], 0.]])
    else:
        skew_matrix = np.zeros((3, 3))

    return skew_matrix

def cross_prod(a, b):
    result = np.array([a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]])

    return result

def new_radial_vectors(params):
    # drone com -- rod1 -- rod2 -- load
    radial_drone = np.array([0., 0., params.drone_radius])
    radial_drone.shape = (3,1)
    new_radial_drone = Rotation.as_matrix(params.rotation_matrix_drone) @ radial_drone

    radial_rod1 = np.array([0., 0., (1./ 2.)*params.rod_length])
    radial_rod1.shape = (3, 1)
    new_radial_rod1 = Rotation.as_matrix(params.rotation_matrix_rod1) @ radial_rod1

    radial_rod2 = np.array([0., 0., (1./2.)*params.rod_length])
    radial_rod2.shape = (3, 1)
    new_radial_rod2 = Rotation.as_matrix(params.rotation_matrix_rod2) @ radial_rod2

    radial_load = np.array([0., 0., params.load_radius])
    radial_load.shape = (3, 1)
    new_radial_load = Rotation.as_matrix(params.rotation_matrix_load) @ radial_load

    return new_radial_drone, new_radial_rod1, new_radial_rod2, new_radial_load

def get_Jac(params):
    # only linear dof remain (constraint is pos. of ee - pos. of drone com = 0)
    new_radial_drone, new_radial_rod1, new_radial_rod2, new_radial_load = new_radial_vectors(params)
    new_radial_drone.shape = (3, 1)
    new_radial_rod1.shape = (3, 1)
    new_radial_rod2.shape = (3, 1)
    new_radial_load.shape = (3, 1)

    Jac_interface = np.block([
        [np.identity(3), np.zeros((3,21))]
    ])
    Jac_interface_dot = np.block([
        [np.zeros((3,24))]
    ])

    Jac_rod1_drone = np.block([
        [np.identity(3), skew_symmetric_matrix_from_vector(new_radial_drone), -np.identity(3),
         skew_symmetric_matrix_from_vector(new_radial_rod1), np.zeros((3,12))]
    ])
    cross_product_vector_drone = cross_prod(params.drone_com_velocity[3:6], new_radial_drone)
    cross_product_vector_rod1 = cross_prod(params.rod1_com_velocity[3:6], new_radial_rod1)
    Jac_rod1_drone_dot = np.block([
        [np.zeros((3,3)), skew_symmetric_matrix_from_vector(cross_product_vector_drone), np.zeros((3,3)),
         skew_symmetric_matrix_from_vector(cross_product_vector_rod1), np.zeros((3,12))]
    ])

    Jac_rod2_rod1 = np.block([
        [np.zeros((3,6)), np.identity(3), skew_symmetric_matrix_from_vector(new_radial_rod1), -np.identity(3),
         skew_symmetric_matrix_from_vector(new_radial_rod2), np.zeros((3, 6))]
    ])
    cross_product_vector_rod1 = cross_prod(params.rod1_com_velocity[3:6], new_radial_rod1)
    cross_product_vector_rod2 = cross_prod(params.rod2_com_velocity[3:6], new_radial_rod2)
    Jac_rod2_rod1_dot = np.block([
        [np.zeros((3, 6)),  np.zeros((3,3)), skew_symmetric_matrix_from_vector(cross_product_vector_rod1), np.zeros((3, 3)),
         skew_symmetric_matrix_from_vector(cross_product_vector_rod2), np.zeros((3, 6))]
    ])

    Jac_load_rod2 = np.block([
        [np.zeros((3,12)), np.identity(3), skew_symmetric_matrix_from_vector(new_radial_rod2), -np.identity(3),
         skew_symmetric_matrix_from_vector(new_radial_load)]
    ])
    cross_product_vector_rod2 = cross_prod(params.rod2_com_velocity[3:6], new_radial_rod2)
    cross_product_vector_load = cross_prod(params.load_com_velocity[3:6], new_radial_load)
    Jac_load_rod2_dot = np.block([
        [np.zeros((3, 12)), np.zeros((3,3)), skew_symmetric_matrix_from_vector(cross_product_vector_rod2), np.zeros((3, 3)),
         skew_symmetric_matrix_from_vector(cross_product_vector_load)]
    ])

    Jac = np.block([[Jac_interface], [Jac_rod1_drone], [Jac_rod2_rod1], [Jac_load_rod2]])
    Jac_dot = np.block([[Jac_interface_dot], [Jac_rod1_drone_dot], [Jac_rod2_rod1_dot], [Jac_load_rod2_dot]])

    return Jac, Jac_dot

def get_effective_params(params, h, generalized_force_g_drone, generalized_force_g_rod1, generalized_force_g_rod2, generalized_force_g_load, Jac, Jac_dot):

    Jac_interface = Jac[0:3,0:24]
    Jac_interface_dot = Jac_dot[0:3,0:24]
    Jac_constraint = Jac[3:21,0:24]
    Jac_constraint_dot = Jac_dot[3:21,0:24]

    # Computation of effective mass and effective force (from A. Peiret thesis)
    M = system_mass_matrix(params)

    Minv = np.linalg.inv(M)
    Pc = Minv @ np.transpose(Jac_constraint) @ np.linalg.inv(Jac_constraint @ Minv @ np.transpose(Jac_constraint)) @ Jac_constraint
    effective_mass = np.linalg.inv(Jac_interface @ (np.identity(24) - Pc) @ Minv @ np.transpose(Jac_interface))

    system_generalized_velocity = np.concatenate((params.drone_com_velocity, params.rod1_com_velocity, params.rod2_com_velocity, params.load_com_velocity), axis=0)
    prev_system_generalized_velocity = np.concatenate((params.prev_drone_com_velocity, params.prev_rod1_com_velocity, params.prev_rod2_com_velocity, params.prev_load_com_velocity), axis=0)
    system_generalized_acceleration = (1 / h) * (system_generalized_velocity - prev_system_generalized_velocity)

    applied_force = np.concatenate((generalized_force_g_drone, generalized_force_g_rod1, generalized_force_g_rod2, generalized_force_g_load))

    noninertial_forces_drone = noninertial_terms_sphere(params.mass_drone, params.drone_radius,
                                                        system_generalized_velocity[3:6])
    noninertial_forces_drone.shape = (3, 1)
    noninertial_forces_rod1 = noninertial_terms_rod(params.mass_density, params.rod_radius,
                                                    params.rod_length, system_generalized_velocity[9:12])
    noninertial_forces_rod1.shape = (3, 1)
    noninertial_forces_rod2 = noninertial_terms_rod(params.mass_density, params.rod_radius,
                                                    params.rod_length, system_generalized_velocity[15:18])
    noninertial_forces_rod2.shape = (3, 1)
    noninertial_forces_load = noninertial_terms_sphere(params.mass_load, params.load_radius,
                                                       system_generalized_velocity[21:24])
    noninertial_forces_load.shape = (3, 1)
    noninertial_forces = np.concatenate((np.zeros((3, 1)), noninertial_forces_drone, np.zeros((3, 1)),
                                         noninertial_forces_rod1, np.zeros((3, 1)), noninertial_forces_rod2,
                                         np.zeros((3, 1)), noninertial_forces_load), axis=0)
    noninertial_forces.shape = (24, 1)

    effective_force = effective_mass @ (Jac_interface @ (np.identity(24) - Pc) @ Minv @ (applied_force - noninertial_forces) + Jac_interface_dot @ system_generalized_velocity + Jac_interface @ Pc @ system_generalized_acceleration)
    interface_velocity = Jac_interface @ system_generalized_velocity

    return effective_mass, effective_force, interface_velocity

# Semi-implicit integrator that integrates the system
def time_step_system(params, inverse3_position, h, generalized_force_g_drone, generalized_force_g_rod1, generalized_force_g_rod2, generalized_force_g_load, Jac):

    system_compliance = 1e-4
    interface_stiffness = params.interface_stiffness
    interface_damping = params.interface_damping
    interface_coeff = interface_stiffness/(interface_damping + h*interface_stiffness)
    regularization_coeff = 1/(h*(interface_damping + h*interface_stiffness))

    new_radial_drone, new_radial_rod1, new_radial_rod2, new_radial_load = new_radial_vectors(params)
    new_radial_drone.shape = (3, 1)
    new_radial_rod1.shape = (3, 1)
    new_radial_rod2.shape = (3, 1)
    new_radial_load.shape = (3, 1)

    system_generalized_velocity = np.concatenate(
        (params.drone_com_velocity, params.rod1_com_velocity, params.rod2_com_velocity, params.load_com_velocity),
        axis=0)

    noninertial_forces_drone = np.concatenate((np.zeros((3,1)), noninertial_terms_sphere(params.mass_drone, params.drone_radius,
                                                        system_generalized_velocity[3:6])), axis=0)
    noninertial_forces_drone.shape = (6, 1)
    noninertial_forces_rod1 = np.concatenate((np.zeros((3,1)), noninertial_terms_rod(params.mass_density, params.rod_radius,
                                                    params.rod_length, system_generalized_velocity[9:12])), axis=0)
    noninertial_forces_rod1.shape = (6, 1)
    noninertial_forces_rod2 = np.concatenate((np.zeros((3,1)), noninertial_terms_rod(params.mass_density, params.rod_radius,
                                                    params.rod_length, system_generalized_velocity[15:18])), axis=0)
    noninertial_forces_rod2.shape = (6, 1)
    noninertial_forces_load = np.concatenate((np.zeros((3,1)), noninertial_terms_sphere(params.mass_load, params.load_radius,
                                                       system_generalized_velocity[21:24])), axis=0)
    noninertial_forces_load.shape = (6, 1)

    W = np.zeros((36, 36))
    W[0:24, 0:24] = system_mass_matrix(params)
    W[0:24, 24:36] = -np.transpose(Jac)
    W[24:36, 0:24] = Jac
    W[24:36,24:36] = (system_compliance / (h * h)) * np.identity(12)
    W[24:27,24:27] = (regularization_coeff)*np.identity(3)

    phi_interface = params.drone_com - inverse3_position
    phi_rod1_drone = (params.drone_com - new_radial_drone) - (params.rod1_com + new_radial_rod1)
    phi_rod2_rod1 = (params.rod1_com - new_radial_rod1) - (params.rod2_com + new_radial_rod2)
    phi_load_rod2 = (params.rod2_com - new_radial_rod2) - (params.load_com + new_radial_load)

    b = np.zeros((36, 1))
    b[0:6, 0:1] = params.mass_matrix_drone @ params.drone_com_velocity + h * (generalized_force_g_drone - noninertial_forces_drone)
    b[6:12, 0:1] = params.mass_matrix_rod1 @ params.rod1_com_velocity + h * (generalized_force_g_rod1 - noninertial_forces_rod1)
    b[12:18, 0:1] = params.mass_matrix_rod2 @ params.rod2_com_velocity + h * (generalized_force_g_rod2 - noninertial_forces_rod2)
    b[18:24, 0:1] = params.mass_matrix_load @ params.load_com_velocity + h * (generalized_force_g_load - noninertial_forces_load)
    b[24:27, 0:1] = -interface_coeff * phi_interface
    b[27:30, 0:1] = (-1. / h) * phi_rod1_drone
    b[30:33, 0:1] = (-1. / h) * phi_rod2_rod1
    b[33:36, 0:1] = (-1. / h) * phi_load_rod2

    x = np.linalg.solve(W, b)

    params.prev_drone_com_velocity = params.drone_com_velocity
    params.prev_drone_com_velocity.shape = (6,1)
    params.prev_rod1_com_velocity = params.rod1_com_velocity
    params.prev_rod1_com_velocity.shape = (6,1)
    params.prev_rod2_com_velocity = params.rod2_com_velocity
    params.prev_rod2_com_velocity.shape = (6,1)
    params.prev_load_com_velocity = params.load_com_velocity
    params.prev_load_com_velocity.shape = (6,1)

    params.drone_com_velocity = x[0:6]
    params.drone_com_velocity.shape = (6, 1)
    params.rod1_com_velocity = x[6:12]
    params.rod1_com_velocity.shape = (6, 1)
    params.rod2_com_velocity = x[12:18]
    params.rod2_com_velocity.shape = (6, 1)
    params.load_com_velocity = x[18:24]
    params.load_com_velocity.shape = (6, 1)

    params.drone_com = params.drone_com + h * params.drone_com_velocity[0:3, 0:1]
    params.rod1_com = params.rod1_com + h * params.rod1_com_velocity[0:3, 0:1]
    params.rod2_com = params.rod2_com + h * params.rod2_com_velocity[0:3, 0:1]
    params.load_com = params.load_com + h * params.load_com_velocity[0:3, 0:1]

    params.rotation_matrix_drone = new_rotation_matrix(params.rotation_matrix_drone, params.drone_com_velocity[3:6], h)
    params.rotation_matrix_rod1 = new_rotation_matrix(params.rotation_matrix_rod1, params.rod1_com_velocity[3:6], h)
    params.rotation_matrix_rod2 = new_rotation_matrix(params.rotation_matrix_rod2, params.rod2_com_velocity[3:6], h)
    params.rotation_matrix_load = new_rotation_matrix(params.rotation_matrix_load, params.load_com_velocity[3:6], h)

def display_setup(drone_radius, rod_length, load_radius):

    drone_com = np.array([0., 0., 0.])
    drone_com.shape = (3, 1)

    rod1_com = np.array([0., 0., -drone_radius-rod_length/2.])
    rod1_com.shape = (3, 1)

    rod2_com = np.array([0., 0., -drone_radius-3.*rod_length/2.])
    rod2_com.shape = (3, 1)

    load_com = np.array([0., 0., -drone_radius-2.*rod_length-load_radius])
    load_com.shape = (3, 1)

    return drone_com, rod1_com, rod2_com, load_com

def sample_delay_distribution(max_delay,min_delay):
    return random.randint(min_delay, max_delay)
