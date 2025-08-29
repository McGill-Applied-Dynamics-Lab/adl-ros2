import numpy as np
import vpython as vp
import math_helper_functions
from scipy.spatial.transform import Rotation
import copy

class Inverse3State:
    def __init__(self, inverse3_position = [], inverse3_velocity = []):
        self.inverse3_position = inverse3_position
        self.inverse3_velocity = inverse3_velocity

class EffectiveParams:
    def __init__(self, interface_stiffness=0, interface_damping=0, effective_mass=0, effective_force=[],
                 interface_velocity=[], drone_com = [], drone_com_velocity = []):
        self.interface_stiffness = interface_stiffness
        self.interface_damping = interface_damping

        self.effective_mass = effective_mass
        self.effective_force = effective_force

        self.interface_velocity = interface_velocity

        self.drone_com = drone_com
        self.drone_com_velocity = drone_com_velocity

        self.time_stamp = -1
        self.time_popped = 1

class VirtualState:
    def __init__(self, rod1_com=[], rod2_com=[],drone_com=[], load_com=[],
                 rod1_com_velocity=[], rod2_com_velocity=[], drone_com_velocity=[],load_com_velocity=[],
                 drone_axis=[], rod1_axis=[], rod2_axis=[], load_axis=[]):
        self.rod1_com = rod1_com
        self.rod2_com = rod2_com
        self.drone_com = drone_com
        self.load_com = load_com
        self.rod1_com_velocity = rod1_com_velocity
        self.rod2_com_velocity = rod2_com_velocity
        self.drone_com_velocity = drone_com_velocity
        self.load_com_velocity = load_com_velocity
        self.drone_axis = drone_axis
        self.rod1_axis = rod1_axis
        self.rod2_axis = rod2_axis
        self.load_axis = load_axis

class VirtualEnvironment:
    def __init__(self, hv=0, time_step=0, mass_drone=0, drone_radius=0, mass_density=0, rod_length=0, rod_radius=0, mass_load=0, load_radius=0,
                 rotation_matrix_drone=[], rotation_matrix_rod1=[], rotation_matrix_rod2=[], rotation_matrix_load=[],
                 mass_matrix_drone=[], mass_matrix_rod1=[], mass_matrix_rod2=[], mass_matrix_load=[], mass_matrix = [], inverse_mass_matrix=[],
                 drone_com=[], drone_com_velocity=[], prev_drone_com_velocity=[], drone_com_acceleration=[],
                 rod1_com=[], rod1_com_velocity=[], prev_rod1_com_velocity=[], rod1_com_acceleration=[],
                 rod2_com=[], rod2_com_velocity=[], prev_rod2_com_velocity=[], rod2_com_acceleration=[],
                 load_com=[], load_com_velocity=[], prev_load_com_velocity=[], load_com_acceleration=[],
                 system_generalized_velocity=[], prev_system_generalized_velocity=[], system_generalized_acceleration=[], time_stamp=-1,
                 time_popped=1):
        self.hv = hv
        self.time_step = time_step

        self.time_stamp = time_stamp
        self.time_popped = time_popped

        self.mass_drone = mass_drone
        self.drone_radius = drone_radius
        self.mass_density = mass_density
        self.rod_length = rod_length
        self.rod_radius = rod_radius
        self.mass_load = mass_load
        self.load_radius = load_radius

        self.mass_matrix_drone = mass_matrix_drone
        self.mass_matrix_rod1 = mass_matrix_rod1
        self.mass_matrix_rod2 = mass_matrix_rod2
        self.mass_matrix_load = mass_matrix_load
        self.mass_matrix = mass_matrix
        self.inverse_mass_matrix = inverse_mass_matrix

        self.rotation_matrix_drone = rotation_matrix_drone
        self.rotation_matrix_rod1 = rotation_matrix_rod1
        self.rotation_matrix_rod2 = rotation_matrix_rod2
        self.rotation_matrix_load = rotation_matrix_load

        self.drone_com = drone_com
        self.drone_com_velocity = drone_com_velocity
        self.prev_drone_com_velocity = prev_drone_com_velocity
        self.drone_com_acceleration = drone_com_acceleration
        self.rod1_com = rod1_com
        self.rod1_com_velocity = rod1_com_velocity
        self.prev_rod1_com_velocity = prev_rod1_com_velocity
        self.rod1_com_acceleration = rod1_com_acceleration
        self.rod2_com = rod2_com
        self.rod2_com_velocity = rod2_com_velocity
        self.prev_rod2_com_velocity = prev_rod2_com_velocity
        self.rod2_com_acceleration = rod2_com_acceleration
        self.load_com = load_com
        self.load_com_velocity = load_com_velocity
        self.prev_load_com_velocity = prev_load_com_velocity
        self.load_com_acceleration = load_com_acceleration

        self.system_generalized_velocity = system_generalized_velocity
        self.prev_system_generalized_velocity = prev_system_generalized_velocity
        self.system_generalized_acceleration = system_generalized_acceleration

        self.initialize_virtual_params()

    def initialize_virtual_params(self):
        #self.hv = 0.001

        #self.mass_drone = 2.5
        #self.drone_radius = 0.05
        #self.mass_density = 10000
        #self.rod_length = 0.1
        #self.rod_radius = 0.01
        #self.mass_load = 1
        #self.load_radius = 0.03

        self.rotation_matrix_drone = Rotation.from_matrix(np.identity(3))
        self.rotation_matrix_rod1 = Rotation.from_matrix(np.identity(3))
        self.rotation_matrix_rod2 = Rotation.from_matrix(np.identity(3))
        self.rotation_matrix_load = Rotation.from_matrix(np.identity(3))

        self.mass_matrix_drone = self.mass_matrix_sphere(self.mass_drone, self.drone_radius,
                                                      self.rotation_matrix_drone)
        self.mass_matrix_rod1 = self.mass_matrix_rod(self.mass_density, self.rod_length, self.rod_radius,
                                                  self.rotation_matrix_rod1)
        self.mass_matrix_rod2 = self.mass_matrix_rod(self.mass_density, self.rod_length, self.rod_radius,
                                                  self.rotation_matrix_rod2)
        self.mass_matrix_load = self.mass_matrix_sphere(self.mass_load, self.load_radius, self.rotation_matrix_load)
        self.mass_matrix = self.system_mass_matrix()
        self.inverse_mass_matrix = np.linalg.inv(self.mass_matrix)

        self.drone_com = np.array([0., 0., 0.])
        self.drone_com.shape = (3, 1)
        self.drone_com_velocity = np.zeros((6, 1))
        self.drone_com_velocity.shape = (6, 1)
        self.prev_drone_com_velocity = np.zeros((6, 1))
        self.prev_drone_com_velocity.shape = (6, 1)
        self.drone_com_acceleration = np.zeros((6, 1))
        self.drone_com_acceleration.shape = (6, 1)
        self.rod1_com = np.array([0., 0., -self.drone_radius - self.rod_length / 2.])
        self.rod1_com.shape = (3, 1)
        self.rod1_com_velocity = np.zeros((6, 1))
        self.rod1_com_velocity.shape = (6, 1)
        self.prev_rod1_com_velocity = np.zeros((6, 1))
        self.prev_rod1_com_velocity.shape = (6, 1)
        self.rod1_com_acceleration = np.zeros((6, 1))
        self.rod1_com_acceleration.shape = (6, 1)
        self.rod2_com = np.array([0., 0., -self.drone_radius - 3. * self.rod_length / 2.])
        self.rod2_com.shape = (3, 1)
        self.rod2_com_velocity = np.zeros((6, 1))
        self.rod2_com_velocity.shape = (6, 1)
        self.prev_rod2_com_velocity = np.zeros((6, 1))
        self.prev_rod2_com_velocity.shape = (6, 1)
        self.rod2_com_acceleration = np.zeros((6, 1))
        self.rod2_com_acceleration.shape = (6, 1)
        self.load_com = np.array([0., 0., -self.drone_radius - 2. * self.rod_length - self.load_radius])
        self.load_com.shape = (3, 1)
        self.load_com_velocity = np.zeros((6, 1))
        self.load_com_velocity.shape = (6, 1)
        self.load_rod1_com_velocity = np.zeros((6, 1))
        self.load_rod1_com_velocity.shape = (6, 1)
        self.prev_load_com_velocity = np.zeros((6, 1))
        self.prev_load_com_velocity.shape = (6, 1)
        self.load_com_acceleration = np.zeros((6, 1))
        self.load_com_acceleration.shape = (6, 1)

        self.system_generalized_velocity = np.concatenate((self.drone_com_velocity, self.rod1_com_velocity,
                    self.rod2_com_velocity, self.load_com_velocity), axis=0)
        self.prev_system_generalized_velocity = np.concatenate((self.prev_drone_com_velocity, self.prev_rod1_com_velocity,
                    self.prev_rod2_com_velocity, self.prev_load_com_velocity), axis=0)
        self.system_generalized_acceleration = (1 / self.hv) * (self.system_generalized_velocity - self.prev_system_generalized_velocity)

        new_radial_drone, new_radial_rod1, new_radial_rod2, new_radial_load = self.new_radial_vectors()
        Jac, Jac_dot = self.get_Jac(new_radial_drone, new_radial_rod1, new_radial_rod2, new_radial_load)
        applied_force_terms = self.applied_forces()
        noninertial_force_terms = self.noninertial_forces()
        self.effective_params = self.calculate_effective_params(applied_force_terms, noninertial_force_terms, Jac,
                                                          Jac_dot)

    def mass_matrix_rod(self, mass_density, rod_length, rod_radius, cur_R):
        rod_mass = mass_density * np.pi * rod_radius ** 2 * rod_length
        # "Products" of inertia around com
        Ixx = (1. / 12.) * rod_mass * (3 * rod_radius ** 2 + rod_length ** 2)
        Iyy = (1. / 2.) * rod_mass * rod_radius ** 2
        Izz = (1. / 12.) * rod_mass * (3 * rod_radius ** 2 + rod_length ** 2)
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

    def mass_matrix_sphere(self, mass, sphere_radius, cur_R):
        sphere_mass = mass
        Ixx = (2. / 5.) * sphere_mass * sphere_radius ** 2
        I = np.array([
            [Ixx, 0., 0.],
            [0., Ixx, 0.],
            [0., 0., Ixx]
        ])
        cur_I = Rotation.as_matrix(cur_R) @ I @ np.transpose(Rotation.as_matrix(cur_R))
        cur_mass_matrix = np.block([
            [sphere_mass * np.identity(3), np.zeros((3, 3))],
            [np.zeros((3, 3)), cur_I]
        ])
        return cur_mass_matrix

    def system_mass_matrix(self):
        mass = np.block([
            [self.mass_matrix_drone, np.zeros((6, 18))],
            [np.zeros((6, 6)), self.mass_matrix_rod1, np.zeros((6, 12))],
            [np.zeros((6, 12)), self.mass_matrix_rod2, np.zeros((6, 6))],
            [np.zeros((6, 18)), self.mass_matrix_load]
        ])
        return mass

    # Rotation matrix about the axis set by the angular velocity
    def new_rotation_matrix(self, cur_R, angular_velocity, h):
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

    def applied_forces(self):
        g = -9.81

        mg_drone = np.array([0.0, 0.0, self.mass_drone * g])
        mg_drone.shape = (3, 1)
        mg_rod = np.array([0.0, 0.0, self.mass_density * np.pi * self.rod_radius ** 2 * self.rod_length * g])
        mg_rod.shape = (3, 1)
        mg_load = np.array([0.0, 0.0, self.mass_load * g])
        mg_load.shape = (3, 1)

        generalized_force_g_drone = np.concatenate((mg_drone, np.zeros((3, 1))), axis=None)
        generalized_force_g_drone.shape = (6, 1)
        generalized_force_g_rod1 = np.concatenate((mg_rod, np.zeros((3, 1))), axis=None)
        generalized_force_g_rod1.shape = (6, 1)
        generalized_force_g_rod2 = np.concatenate((mg_rod, np.zeros((3, 1))), axis=None)
        generalized_force_g_rod2.shape = (6, 1)
        generalized_force_g_load = np.concatenate((mg_load, np.zeros((3, 1))), axis=None)
        generalized_force_g_load.shape = (6, 1)

        applied_forces = np.concatenate(
            (generalized_force_g_drone, generalized_force_g_rod1, generalized_force_g_rod2, generalized_force_g_load))
        applied_forces.shape = (24,1)

        return applied_forces

    def noninertial_terms_rod(self, mass_density, rod_radius, rod_length, angular_velocity):
        rod_mass = mass_density * np.pi * rod_radius ** 2 * rod_length
        # "Products" of inertia around com
        Ixx = (1. / 12.) * rod_mass * (3 * rod_radius ** 2 + rod_length ** 2)
        Iyy = (1. / 2.) * rod_mass * rod_radius ** 2
        Izz = (1. / 12.) * rod_mass * (3 * rod_radius ** 2 + rod_length ** 2)
        I = np.array([
            [Ixx, 0., 0.],
            [0., Iyy, 0.],
            [0., 0., Izz]
        ])

        noninertial_forces = math_helper_functions.cross_prod(angular_velocity, I @ angular_velocity)
        noninertial_forces.shape = (3, 1)

        return noninertial_forces

    def noninertial_terms_sphere(self, mass_density, sphere_radius, angular_velocity):
        sphere_mass = mass_density * np.pi * sphere_radius ** 2
        Ixx = (2. / 5.) * sphere_mass * sphere_radius ** 2
        I = np.array([
            [Ixx, 0., 0.],
            [0., Ixx, 0.],
            [0., 0., Ixx]
        ])

        noninertial_forces = math_helper_functions.cross_prod(angular_velocity, I @ angular_velocity)
        noninertial_forces.shape = (3, 1)

        return noninertial_forces

    def noninertial_forces(self):
        noninertial_forces_drone = self.noninertial_terms_sphere(self.mass_drone, self.drone_radius,
                                                            self.system_generalized_velocity[3:6])
        noninertial_forces_drone.shape = (3, 1)
        noninertial_forces_rod1 = self.noninertial_terms_rod(self.mass_density, self.rod_radius,
                                                        self.rod_length, self.system_generalized_velocity[9:12])
        noninertial_forces_rod1.shape = (3, 1)
        noninertial_forces_rod2 = self.noninertial_terms_rod(self.mass_density, self.rod_radius,
                                                        self.rod_length, self.system_generalized_velocity[15:18])
        noninertial_forces_rod2.shape = (3, 1)
        noninertial_forces_load = self.noninertial_terms_sphere(self.mass_load, self.load_radius,
                                                           self.system_generalized_velocity[21:24])
        noninertial_forces_load.shape = (3, 1)

        noninertial_forces = np.concatenate((np.zeros((3, 1)), noninertial_forces_drone, np.zeros((3, 1)),
                                             noninertial_forces_rod1, np.zeros((3, 1)), noninertial_forces_rod2,
                                             np.zeros((3, 1)), noninertial_forces_load), axis=0)
        noninertial_forces.shape = (24, 1)

        return noninertial_forces

    def new_radial_vectors(self):
        # drone com -- rod1 -- rod2 -- load
        radial_drone = np.array([0., 0., self.drone_radius])
        radial_drone.shape = (3, 1)
        new_radial_drone = Rotation.as_matrix(self.rotation_matrix_drone) @ radial_drone
        new_radial_drone.shape = (3, 1)

        radial_rod1 = np.array([0., 0., (1. / 2.) * self.rod_length])
        radial_rod1.shape = (3, 1)
        new_radial_rod1 = Rotation.as_matrix(self.rotation_matrix_rod1) @ radial_rod1
        new_radial_rod1.shape = (3, 1)

        radial_rod2 = np.array([0., 0., (1. / 2.) * self.rod_length])
        radial_rod2.shape = (3, 1)
        new_radial_rod2 = Rotation.as_matrix(self.rotation_matrix_rod2) @ radial_rod2
        new_radial_rod2.shape = (3, 1)

        radial_load = np.array([0., 0., self.load_radius])
        radial_load.shape = (3, 1)
        new_radial_load = Rotation.as_matrix(self.rotation_matrix_load) @ radial_load
        new_radial_load.shape = (3, 1)

        return new_radial_drone, new_radial_rod1, new_radial_rod2, new_radial_load

    def get_Jac(self, new_radial_drone, new_radial_rod1, new_radial_rod2, new_radial_load):
        # only linear dof remain (constraint is pos. of ee - pos. of drone com = 0)
        Jac_interface = np.block([
            [np.identity(3), np.zeros((3, 21))]
        ])
        Jac_interface_dot = np.block([
            [np.zeros((3, 24))]
        ])

        Jac_rod1_drone = np.block([
            [np.identity(3),math_helper_functions.skew_symmetric_matrix_from_vector(new_radial_drone), -np.identity(3),
             math_helper_functions.skew_symmetric_matrix_from_vector(new_radial_rod1), np.zeros((3, 12))]
        ])
        cross_product_vector_drone = math_helper_functions.cross_prod(self.drone_com_velocity[3:6], new_radial_drone)
        cross_product_vector_rod1 = math_helper_functions.cross_prod(self.rod1_com_velocity[3:6], new_radial_rod1)
        Jac_rod1_drone_dot = np.block([
            [np.zeros((3, 3)), math_helper_functions.skew_symmetric_matrix_from_vector(cross_product_vector_drone), np.zeros((3, 3)),
             math_helper_functions.skew_symmetric_matrix_from_vector(cross_product_vector_rod1), np.zeros((3, 12))]
        ])

        Jac_rod2_rod1 = np.block([
            [np.zeros((3, 6)), np.identity(3), math_helper_functions.skew_symmetric_matrix_from_vector(new_radial_rod1), -np.identity(3),
             math_helper_functions.skew_symmetric_matrix_from_vector(new_radial_rod2), np.zeros((3, 6))]
        ])
        cross_product_vector_rod1 = math_helper_functions.cross_prod(self.rod1_com_velocity[3:6], new_radial_rod1)
        cross_product_vector_rod2 = math_helper_functions.cross_prod(self.rod2_com_velocity[3:6], new_radial_rod2)
        Jac_rod2_rod1_dot = np.block([
            [np.zeros((3, 6)), np.zeros((3, 3)), math_helper_functions.skew_symmetric_matrix_from_vector(cross_product_vector_rod1),
             np.zeros((3, 3)),
             math_helper_functions.skew_symmetric_matrix_from_vector(cross_product_vector_rod2), np.zeros((3, 6))]
        ])

        Jac_load_rod2 = np.block([
            [np.zeros((3, 12)), np.identity(3), math_helper_functions.skew_symmetric_matrix_from_vector(new_radial_rod2), -np.identity(3),
             math_helper_functions.skew_symmetric_matrix_from_vector(new_radial_load)]
        ])
        cross_product_vector_rod2 = math_helper_functions.cross_prod(self.rod2_com_velocity[3:6], new_radial_rod2)
        cross_product_vector_load = math_helper_functions.cross_prod(self.load_com_velocity[3:6], new_radial_load)
        Jac_load_rod2_dot = np.block([
            [np.zeros((3, 12)), np.zeros((3, 3)), math_helper_functions.skew_symmetric_matrix_from_vector(cross_product_vector_rod2),
             np.zeros((3, 3)),
             math_helper_functions.skew_symmetric_matrix_from_vector(cross_product_vector_load)]
        ])

        Jac = np.block([[Jac_interface], [Jac_rod1_drone], [Jac_rod2_rod1], [Jac_load_rod2]])
        Jac.shape = (12, 24)
        Jac_dot = np.block([[Jac_interface_dot], [Jac_rod1_drone_dot], [Jac_rod2_rod1_dot], [Jac_load_rod2_dot]])
        Jac_dot.shape = (12, 24)

        return Jac, Jac_dot

    def calculate_effective_params(self, applied_force_terms, noninertial_force_terms, Jac, Jac_dot):

        effective_params = EffectiveParams()
        # Also in local_main
        effective_params.interface_stiffness = 3000
        effective_params.interface_damping = 2

        Jac_interface = Jac[0:3, 0:24]
        Jac_interface_dot = Jac_dot[0:3, 0:24]
        Jac_constraint = Jac[3:21, 0:24]
        # Jac_constraint_dot = Jac_dot[3:21, 0:24]

        Pc = self.inverse_mass_matrix @ np.transpose(Jac_constraint) @ np.linalg.inv(
            Jac_constraint @ self.inverse_mass_matrix @ np.transpose(Jac_constraint)) @ Jac_constraint

        effective_params.effective_mass = np.linalg.inv(Jac_interface @ (np.identity(24) - Pc) @ self.inverse_mass_matrix @ np.transpose(Jac_interface))

        effective_params.effective_force = effective_params.effective_mass @ (Jac_interface @ (np.identity(24) - Pc) @ self.inverse_mass_matrix @ (
                    applied_force_terms - noninertial_force_terms) + Jac_interface_dot @ self.system_generalized_velocity + Jac_interface @ Pc @ self.system_generalized_acceleration)
        effective_params.interface_velocity = Jac_interface @ self.system_generalized_velocity

        effective_params.drone_com = self.drone_com
        effective_params.drone_com_velocity = self.drone_com_velocity

        return effective_params

    def get_effective_params(self):
        return copy.deepcopy(self.effective_params)

    def time_step_system(self, inverse3_position, inverse3_velocity, effective_params, new_radial_drone, new_radial_rod1, new_radial_rod2,
                        new_radial_load, applied_force_terms, noninertial_force_terms, Jac):

        system_compliance = 1e-4
        interface_coeff = effective_params.interface_stiffness / (effective_params.interface_damping + self.hv * effective_params.interface_stiffness)
        regularization_coeff = 1 / (self.hv * (effective_params.interface_damping + self.hv * effective_params.interface_stiffness))

        W = np.zeros((36, 36))
        W[0:24, 0:24] = self.mass_matrix
        W[0:24, 24:36] = -np.transpose(Jac)
        W[24:36, 0:24] = Jac
        W[24:36, 24:36] = 0.01 * (regularization_coeff) * np.identity(12)
        W[24:27, 24:27] = (regularization_coeff) * np.identity(3)

        phi_interface = self.drone_com - inverse3_position
        phi_rod1_drone = (self.drone_com - new_radial_drone) - (self.rod1_com + new_radial_rod1)
        phi_rod2_rod1 = (self.rod1_com - new_radial_rod1) - (self.rod2_com + new_radial_rod2)
        phi_load_rod2 = (self.rod2_com - new_radial_rod2) - (self.load_com + new_radial_load)

        b = np.zeros((36, 1))
        b[0:6, 0:1] = self.mass_matrix_drone @ self.drone_com_velocity + self.hv * (
                    applied_force_terms[0:6] - noninertial_force_terms[0:6])
        b[6:12, 0:1] = self.mass_matrix_rod1 @ self.rod1_com_velocity + self.hv * (
                    applied_force_terms[6:12] - noninertial_force_terms[6:12])
        b[12:18, 0:1] = self.mass_matrix_rod2 @ self.rod2_com_velocity + self.hv * (
                    applied_force_terms[12:18] - noninertial_force_terms[12:18])
        b[18:24, 0:1] = self.mass_matrix_load @ self.load_com_velocity + self.hv * (
                    applied_force_terms[18:24] - noninertial_force_terms[18:24])
        b[24:27, 0:1] = -interface_coeff * phi_interface
        b[27:30, 0:1] = (-1. / self.hv) * phi_rod1_drone
        b[30:33, 0:1] = (-1. / self.hv) * phi_rod2_rod1
        b[33:36, 0:1] = (-1. / self.hv) * phi_load_rod2

        x = np.linalg.solve(W, b)

        self.prev_drone_com_velocity = self.drone_com_velocity
        self.prev_drone_com_velocity.shape = (6, 1)
        self.prev_rod1_com_velocity = self.rod1_com_velocity
        self.prev_rod1_com_velocity.shape = (6, 1)
        self.prev_rod2_com_velocity = self.rod2_com_velocity
        self.prev_rod2_com_velocity.shape = (6, 1)
        self.prev_load_com_velocity = self.load_com_velocity
        self.prev_load_com_velocity.shape = (6, 1)

        self.drone_com_velocity = x[0:6]
        self.drone_com_velocity.shape = (6, 1)
        self.rod1_com_velocity = x[6:12]
        self.rod1_com_velocity.shape = (6, 1)
        self.rod2_com_velocity = x[12:18]
        self.rod2_com_velocity.shape = (6, 1)
        self.load_com_velocity = x[18:24]
        self.load_com_velocity.shape = (6, 1)

        self.drone_com = self.drone_com + self.hv * self.drone_com_velocity[0:3, 0:1]
        self.rod1_com = self.rod1_com + self.hv * self.rod1_com_velocity[0:3, 0:1]
        self.rod2_com = self.rod2_com + self.hv * self.rod2_com_velocity[0:3, 0:1]
        self.load_com = self.load_com + self.hv * self.load_com_velocity[0:3, 0:1]

        self.rotation_matrix_drone = self.new_rotation_matrix(self.rotation_matrix_drone, self.drone_com_velocity[3:6], self.hv)
        self.rotation_matrix_rod1 = self.new_rotation_matrix(self.rotation_matrix_rod1, self.rod1_com_velocity[3:6], self.hv)
        self.rotation_matrix_rod2 = self.new_rotation_matrix(self.rotation_matrix_rod2, self.rod2_com_velocity[3:6], self.hv)
        self.rotation_matrix_load = self.new_rotation_matrix(self.rotation_matrix_load, self.load_com_velocity[3:6], self.hv)

    def get_virtual_environment_state(self):
        #This function is used to return the data needed for the graphics
        virtual_state = VirtualState()
        # rod1_com, rod2_com, drone_com, load_com, drone_axis, rod1_axis, rod2_axis, load_axis
        radial_drone, radial_rod1, radial_rod2, radial_load = self.new_radial_vectors()

        virtual_state.rod1_com = self.rod1_com
        virtual_state.rod2_com = self.rod2_com
        virtual_state.drone_com = self.drone_com
        virtual_state.load_com = self.load_com
        virtual_state.rod1_com_velocity = self.rod1_com_velocity
        virtual_state.rod2_com_velocity = self.rod2_com_velocity
        virtual_state.drone_com_velocity = self.drone_com_velocity
        virtual_state.load_com_velocity = self.load_com_velocity
        virtual_state.drone_axis = radial_drone
        virtual_state.rod1_axis = radial_rod1
        virtual_state.rod2_axis = radial_rod2
        virtual_state.load_axis = radial_load

        return virtual_state

    def virtual_main(self, inverse3_state):

        inverse3_position = inverse3_state.inverse3_position
        inverse3_velocity = inverse3_state.inverse3_velocity

        # Get new radial vectors depending on
        new_radial_drone, new_radial_rod1, new_radial_rod2, new_radial_load = self.new_radial_vectors()

        # Get Jacobians for the internal constraints
        Jac, Jac_dot = self.get_Jac(new_radial_drone, new_radial_rod1, new_radial_rod2, new_radial_load)

        # Get forces
        applied_force_terms = self.applied_forces()
        noninertial_force_terms = self.noninertial_forces()

        #We should step here before we compute the effective parameters.
        # The inverse 3 position should be passed in as a variable when this function is called.
        self.time_step_system(inverse3_position, inverse3_velocity, self.effective_params, new_radial_drone, new_radial_rod1,
                        new_radial_rod2, new_radial_load, applied_force_terms, noninertial_force_terms, Jac)

        #Partway done here, it's the right idea though.

        # Now we just save the effective params, and they'll stay in some local variable until the overall
        # loop asks for them.
        self.effective_params = self.calculate_effective_params(applied_force_terms, noninertial_force_terms, Jac, Jac_dot)





