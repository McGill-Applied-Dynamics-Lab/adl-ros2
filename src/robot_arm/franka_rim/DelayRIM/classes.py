class DoublePendulum:
    def __init__(self, time_step=0, mass_drone=0, drone_radius=0, mass_density=0, rod_length=0, rod_radius=0, mass_load=0, load_radius=0,
                 rotation_matrix_drone=[], rotation_matrix_rod1=[], rotation_matrix_rod2=[], rotation_matrix_load=[],
                 mass_matrix_drone=[], mass_matrix_rod1=[], mass_matrix_rod2=[], mass_matrix_load=[],
                 drone_com=[], drone_com_velocity=[], prev_drone_com_velocity=[], drone_com_acceleration=[],
                 rod1_com=[], rod1_com_velocity=[], prev_rod1_com_velocity=[], rod1_com_acceleration=[],
                 rod2_com=[], rod2_com_velocity=[], prev_rod2_com_velocity=[], rod2_com_acceleration=[],
                 load_com=[], load_com_velocity=[], prev_load_com_velocity=[], load_com_acceleration=[],
                 interface_stiffness = 1, interface_damping = 1, time_stamp=-1, time_popped=1):

        self.time_step = time_step

        self.mass_drone = mass_drone
        self.drone_radius = drone_radius
        self.mass_density = mass_density
        self.rod_length = rod_length
        self.rod_radius = rod_radius
        self.mass_load = mass_load
        self.load_radius = load_radius

        self.rotation_matrix_drone = rotation_matrix_drone
        self.rotation_matrix_rod1 = rotation_matrix_rod1
        self.rotation_matrix_rod2 = rotation_matrix_rod2
        self.rotation_matrix_load = rotation_matrix_load

        self.mass_matrix_drone = mass_matrix_drone
        self.mass_matrix_rod1 = mass_matrix_rod1
        self.mass_matrix_rod2 = mass_matrix_rod2
        self.mass_matrix_load = mass_matrix_load

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

        self.interface_stiffness = interface_stiffness
        self.interface_damping = interface_damping

        self.time_stamp = time_stamp
        self.time_popped = time_popped

class ReducedModel:
    def __init__(self, stiffness=1e15, damping=1e13, reduced_model_position=[], reduced_model_velocity=[],
                 phi_position=[], phi_velocity = [], effective_mass=[], interface_force=[]):

        self.stiffness = stiffness
        self.damping = damping

        self.reduced_model_position = reduced_model_position
        self.reduced_model_velocity = reduced_model_velocity

        self.phi_position = phi_position
        self.phi_velocity = phi_velocity

        self.effective_mass = effective_mass
        self.interface_force = interface_force

class ArgsSetup:
    def __init__(self, delay_RIM=False, semi_implicit_RIM_stepper=False, compliant_constraints_RIM_stepper=True, RIM_stiffness=10000, RIM_damping=10, sample_function_name='', randomSeed = 0):

        self.delay_RIM = delay_RIM
        self.semi_implicit_RIM_stepper = semi_implicit_RIM_stepper
        self.compliant_constraints_RIM_stepper = compliant_constraints_RIM_stepper
        self.RIM_stiffness = RIM_stiffness
        self.RIM_damping = RIM_damping
        self.sample_function_name = sample_function_name
        self.randomSeed = randomSeed