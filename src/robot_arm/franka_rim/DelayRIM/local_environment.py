import math_helper_functions
import numpy as np
import copy
import math
import csv
import random
from functools import partial
from virtual_environment import EffectiveParams

# The object corresponding to the reduced interface model of the environment
class ReducedModel:
    def __init__(self, stiffness=1e15, damping=1e13, reduced_model_position=np.zeros((3, 1)),
                 reduced_model_velocity=np.zeros((3, 1)),
                 phi_position=np.zeros((3, 1)), phi_velocity=np.zeros((3, 1)), interface_force=np.zeros((3, 1))):

        # Set the stiffness and damping of the virtual coupling
        self.stiffness = stiffness
        self.damping = damping

        # Set the position and velocity of the RIM object
        self.reduced_model_position = reduced_model_position
        self.reduced_model_velocity = reduced_model_velocity

        # Set position and velocity of the constraint deviation
        self.phi_position = phi_position
        self.phi_velocity = phi_velocity

        # Set the interface force
        self.interface_force = interface_force

# Set up the LocalEnvironment, which is the environment the operator interacts with
class LocalEnvironment:
    def __init__(self, hl=0.001, augmented_mass_matrix=[], inverse_augmented_mass_matrix=[], latencyModel = 'ZOH',
                 communicationModel = 'C1', constantDelay = 1, inverse3_position_queue = [], inverse3_velocity_queue = []):

        # Local time step
        self.hl = hl

        # Queues for the position and velocity of Inverse3
        self.inverse3_position_queue = inverse3_position_queue
        self.inverse3_velocity_queue = inverse3_velocity_queue

        # Needed for compliant constraints integration method
        self.augmented_mass_matrix = augmented_mass_matrix
        self.inverse_augmented_mass_matrix = inverse_augmented_mass_matrix

        # Set up the queues
        self.initialize_params_queue()

        # Set up the delay distribution
        if communicationModel == '5G':
            my_data = self.data_open_and_copy('Haply_5G.csv')
            self.delay_distribution = self.get_bootstrap_func(my_data)
        else:
            self.delay_distribution = self.get_constant_delay_distribution(constantDelay)

        # Options are: ZOH, ZOHPhi, DelayRIM
        self.latencyModel = latencyModel

        # Make ReducedModel object
        self.reduced_model = ReducedModel()

        # Length of the Inverse3 position queue
        self.packet_delay = 0

    # This function
    def do_catch_up(self, k, inverse_augmented_mass_matrix, mass_factor):
        self.reduced_model.reduced_model_position.shape = (3, 1)
        self.reduced_model.reduced_model_velocity.shape = (3, 1)

        self.reduced_model.phi_position = self.reduced_model.reduced_model_position - self.inverse3_position_queue[k]
        self.reduced_model.phi_velocity = self.reduced_model.reduced_model_velocity - self.inverse3_velocity_queue[k]

        if (k == len(self.inverse3_position_queue) - 1):
            self.reduced_model.interface_force = -self.reduced_model.stiffness * self.reduced_model.phi_position - (self.hl * self.reduced_model.stiffness + self.reduced_model.damping) * self.reduced_model.phi_velocity
            self.reduced_model.interface_force.shape = (3, 1)
        else:
            # below force terms, ddot{q_inv} - non-inertial term, effective force, interface spring force
            regular_force_terms = self.params_queue[0].effective_mass @ (self.inverse3_velocity_queue[k+1] - self.inverse3_velocity_queue[
                k]) / self.hl + self.params_queue[0].effective_force - self.reduced_model.stiffness * self.reduced_model.phi_position + (
                                          self.reduced_model.damping + self.hl * self.reduced_model.stiffness) * self.inverse3_velocity_queue[k+1]
            self.reduced_model.reduced_model_velocity = mass_factor @ self.reduced_model.reduced_model_velocity + self.hl * inverse_augmented_mass_matrix @ regular_force_terms
            self.reduced_model.reduced_model_position = self.reduced_model.reduced_model_position + self.hl * self.reduced_model.reduced_model_velocity

    def compliant_constraint_integrator(self, inverse_augmented_mass_matrix, mass_factor):
        self.reduced_model.reduced_model_position.shape = (3, 1)
        self.reduced_model.reduced_model_velocity.shape = (3, 1)
        self.reduced_model.phi_position = self.reduced_model.reduced_model_position - self.inverse3_position_queue[-1]
        self.reduced_model.phi_velocity = self.reduced_model.reduced_model_velocity - self.inverse3_velocity_queue[-1]

        regular_force_terms = self.params_queue[0].effective_mass @ (self.inverse3_velocity_queue[-1] - self.inverse3_velocity_queue[-2]) / self.hl + self.params_queue[0].effective_force - self.reduced_model.stiffness * self.reduced_model.phi_position + (
                                      self.reduced_model.damping + self.hl * self.reduced_model.stiffness) * self.inverse3_velocity_queue[-1]
        self.reduced_model.reduced_model_velocity = mass_factor @ self.reduced_model.reduced_model_velocity + self.hl * inverse_augmented_mass_matrix @ regular_force_terms
        self.reduced_model.reduced_model_position = self.reduced_model.reduced_model_position + self.hl * self.reduced_model.reduced_model_velocity
        self.reduced_model.interface_force = - self.reduced_model.stiffness * self.reduced_model.phi_position - (self.hl * self.reduced_model.stiffness + self.reduced_model.damping) * self.reduced_model.phi_velocity

    def push_state_to_queue(self, inverse3_initial_position, inverse3_initial_velocity):
        self.inverse3_position_queue.append(inverse3_initial_position)
        self.inverse3_velocity_queue.append(inverse3_initial_velocity)

    def initialize_params_queue(self):
        self.params_queue = []
        initial_params = EffectiveParams()
        self.params_queue.append(initial_params)

    def push_params_to_queue(self, cntr, params):
        if (params != 0):
            if (len(self.params_queue) > self.delay_time - 1):
                # Replace and forget
                while (len(self.params_queue) > self.delay_time - 1):
                    self.params_queue.pop(-1)
            elif (len(self.params_queue) < self.delay_time - 1):
                # Pad the end of the queue up until the new entry
                while (len(self.params_queue) < self.delay_time - 1):
                    self.params_queue.append(copy.deepcopy(self.params_queue[-1]))
                    self.params_queue[-1].time_popped = self.params_queue[-1].time_popped + 1
            self.params_queue.append(copy.deepcopy(params))
            self.params_queue[-1].time_stamp = cntr
            self.params_queue[-1].time_popped = self.delay_time + cntr
        else:
            if(len(self.params_queue) > 0):
                #pad once
                self.params_queue.append(copy.deepcopy(self.params_queue[-1]))
                self.params_queue[-1].time_popped = self.params_queue[-1].time_popped + 1

    def comm_5Gbootstrap(self, data):
        selection = random.randint(0, len(data) - 1)
        i = 0
        try:
            i = data[selection]
        except:
            stuff = 1
        try:
            x = math.ceil(i)
        except:
            stuff2 = 1
        return math.ceil(i)

    def constant_delay_distribution(self, constant):
        return constant

    def get_constant_delay_distribution(self, constant):
        return partial(self.constant_delay_distribution, constant)

    def get_bootstrap_func(self, all_data):
        return partial(self.comm_5Gbootstrap, all_data)

    def data_open_and_copy(self, data_file):
        all_data = []
        with open(data_file, 'r', encoding="utf-8-sig", newline='') as data:
            csv_reader = csv.reader(data, delimiter=',')
            for row in csv_reader:
                all_data.append(float(row[0]))
        return all_data

    def get_current_delay(self):
        return self.delay_time

    def get_delay_of_current_packet(self):
        return self.packet_delay

    def local_main(self, cntr, effective_params = 0, inverse3_current_position = [], inverse3_current_velocity = []):

        #Here's how this loop will go - we assume that:
        # The inverse is connected and working. (this is done in init_local_environment(...))
        # We have whatever information is needed from the virtual environment in the argument of this function

        self.reduced_model.stiffness = 3000
        self.reduced_model.damping = 2

        self.push_state_to_queue(inverse3_current_position, inverse3_current_velocity)

        # 1. Get delay time
        self.delay_time = self.delay_distribution()

        # 2. Add effective params to the queue
        self.push_params_to_queue(cntr, effective_params)

        if (self.params_queue[0].time_stamp > 0):
            #print(params.time_step)

            #First pop everything from the inverse 3 queue that we don't need. That means popping things from the bottom
            # until the queue has the same length as the delay + 1 (since we need one previous to do the acceleration estimate)
            while(len(self.inverse3_position_queue) > (self.params_queue[0].time_popped - self.params_queue[0].time_stamp + 1)):
                self.inverse3_position_queue.pop(0)
                self.inverse3_velocity_queue.pop(0)

            self.packet_delay = len(self.inverse3_position_queue)

            # Synchronize and set effective parameters of the reduced model
            self.reduced_model.reduced_model_position = np.array(self.params_queue[0].drone_com)
            self.reduced_model.reduced_model_position.shape = (3, 1)
            self.reduced_model.reduced_model_velocity = np.array(self.params_queue[0].drone_com_velocity[0:3])
            self.reduced_model.reduced_model_velocity.shape = (3, 1)
            self.reduced_model.phi_position = self.reduced_model.reduced_model_position - self.inverse3_position_queue[0]
            self.reduced_model.phi_velocity = self.reduced_model.reduced_model_velocity - self.inverse3_velocity_queue[0]

            # 4. Do the catch up, latencyModel = 0: 'ZOH', 1: 'ZOHPhi', 2: 'DelayRIM'
            if (self.latencyModel == 'DelayRIM'):  # iterate
                Id3 = np.identity(3)
                augmented_mass_matrix = (
                        self.params_queue[0].effective_mass + self.hl * (self.reduced_model.damping + self.hl * self.reduced_model.stiffness) * Id3)
                inverse_augmented_mass_matrix = np.linalg.inv(augmented_mass_matrix)
                mass_factor = math_helper_functions.multiply_matrix(inverse_augmented_mass_matrix, self.params_queue[0].effective_mass)
                mass_factor.shape = (3, 3)

                for k in range(0, len(self.inverse3_position_queue)):
                    self.do_catch_up(k, inverse_augmented_mass_matrix, mass_factor)
                    #self.compliant_constraint_integrator(inverse_augmented_mass_matrix, mass_factor)

            else:  # Baseline happens when constantDelay = 1 (same for any latencyModel: ZOH, ZOHPhi, and DelayRIM)
                inverse_effective_mass = np.linalg.inv(self.params_queue[0].effective_mass)
                inverse_effective_mass.shape = (3, 3)
                self.reduced_model.phi_position.shape = (3, 1)
                self.inverse3_position_queue[-1].shape = (3, 1)
                self.reduced_model.phi_velocity.shape = (3, 1)
                self.inverse3_velocity_queue[-1].shape = (3, 1)

                if self.latencyModel == 'ZOHPhi':
                    self.reduced_model.phi_position = self.reduced_model.reduced_model_position - self.inverse3_position_queue[-1]
                    self.reduced_model.phi_velocity = self.reduced_model.reduced_model_velocity - self.inverse3_velocity_queue[-1]
                else:
                    self.reduced_model.phi_position = self.reduced_model.reduced_model_position - self.inverse3_position_queue[0]
                    self.reduced_model.phi_velocity = self.reduced_model.reduced_model_velocity - self.inverse3_velocity_queue[0]

                self.reduced_model.interface_force = - self.reduced_model.stiffness * self.reduced_model.phi_position - self.reduced_model.damping * self.reduced_model.phi_velocity

            self.params_queue.pop(0)

        else:
            temp_params = self.params_queue.pop(0)
            if (len(self.params_queue) == 0):
                self.params_queue.append(temp_params)
        rendered_force = (-1) * 0.02 * self.reduced_model.interface_force #np.array([0.,0.,9.81]) #0.03 * self.reduced_model.interface_force  # generalized_force_g[0:3] #0.03
        rendered_force.shape = (3, 1)
        return rendered_force