from multiprocessing import Process, Pipe
import multiprocessing
import time
import copy
from enum import Enum
import math_helper_functions
import numpy as np

from communication_line import Packet

from virtual_environment import EffectiveParams


class CatchUpCommand(Enum):
    CATCHUP = 0
    ONESTEP = 1
    KINEMATIC = 2
    CLEAR = 3
    END = 4


class ReducedModel:
    def __init__(
        self,
        stiffness=1e15,
        damping=1e13,
        reduced_model_position=np.zeros((3, 1)),
        reduced_model_velocity=np.zeros((3, 1)),
        phi_position=np.zeros((3, 1)),
        phi_velocity=np.zeros((3, 1)),
        interface_force=np.zeros((3, 1)),
    ):
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


class RIMPacket:
    def __init__(self, packet: Packet, hl, delay, RIMType, key):
        self.key = key
        self.hl = hl
        self.delay = delay
        self.timeStamp = packet.timeStamp

        self.inverse3_position = [0, 0, 0]
        self.inverse3_velocity = [0, 0, 0]
        self.inverse3_acceleration = [0, 0, 0]

        self.params: EffectiveParams = packet.info

        # Synchronize and set effective parameters of the reduced model
        self.reduced_model = ReducedModel()
        self.reduced_model.stiffness = 3000
        self.reduced_model.damping = 1
        self.reduced_model.reduced_model_position = np.array(self.params.drone_com)
        self.reduced_model.reduced_model_position.shape = (3, 1)
        self.reduced_model.reduced_model_velocity = np.array(self.params.drone_com_velocity[0:3])
        self.reduced_model.reduced_model_velocity.shape = (3, 1)
        self.reduced_model.phi_position = self.reduced_model.reduced_model_position - self.inverse3_position
        self.reduced_model.phi_velocity = self.reduced_model.reduced_model_velocity - self.inverse3_velocity

        Id3 = np.identity(3)
        augmented_mass_matrix = (
            self.params.effective_mass + hl * (self.reduced_model.damping + hl * self.reduced_model.stiffness) * Id3
        )
        inverse_augmented_mass_matrix = np.linalg.inv(augmented_mass_matrix)
        mass_factor = math_helper_functions.multiply_matrix(inverse_augmented_mass_matrix, self.params.effective_mass)
        mass_factor.shape = (3, 3)

        self.mass_factor = mass_factor
        self.inverse_augmented_mass_matrix = inverse_augmented_mass_matrix
        self.RIMType = RIMType  # {0: 'ZOH', 1: 'ZOHPhi', 2: 'DelayRIM'}


def oneStepUpdate(mPacket: RIMPacket, inverse3_position, inverse3_velocity):
    mPacket.inverse3_position_queue.append(inverse3_position)
    mPacket.inverse3_acceleration = inverse3_velocity - mPacket.inverse3_velocity
    mPacket.inverse3_acceleration_queue.append(mPacket.inverse3_acceleration)
    mPacket.inverse3_velocity_queue.append(inverse3_velocity)


def getForce(mPacket: RIMPacket):
    mPacket.reduced_model.reduced_model_position.shape = (3, 1)
    mPacket.reduced_model.reduced_model_velocity.shape = (3, 1)

    mPacket.reduced_model.phi_position = mPacket.reduced_model.reduced_model_position - mPacket.inverse3_position
    mPacket.reduced_model.phi_velocity = mPacket.reduced_model.reduced_model_velocity - mPacket.inverse3_velocity

    mPacket.reduced_model.interface_force = (
        -mPacket.reduced_model.stiffness * mPacket.reduced_model.phi_position
        - (mPacket.hl * mPacket.reduced_model.stiffness + mPacket.reduced_model.damping)
        * mPacket.reduced_model.phi_velocity
    )
    mPacket.reduced_model.interface_force.shape = (3, 1)


def getZOHForce(mPacket: RIMPacket):
    mPacket.reduced_model.reduced_model_position.shape = (3, 1)
    mPacket.reduced_model.reduced_model_velocity.shape = (3, 1)

    mPacket.reduced_model.phi_position = mPacket.reduced_model.reduced_model_position - mPacket.inverse3_position
    mPacket.reduced_model.phi_velocity = mPacket.reduced_model.reduced_model_velocity - mPacket.inverse3_velocity
    mPacket.reduced_model.interface_force = (
        -mPacket.reduced_model.stiffness * mPacket.reduced_model.phi_position
        - mPacket.reduced_model.damping * mPacket.reduced_model.phi_velocity
    )
    mPacket.reduced_model.interface_force.shape = (3, 1)


def oneStep(mPacket: RIMPacket):
    mPacket.reduced_model.reduced_model_position.shape = (3, 1)
    mPacket.reduced_model.reduced_model_velocity.shape = (3, 1)

    mPacket.reduced_model.phi_position = mPacket.reduced_model.reduced_model_position - mPacket.inverse3_position
    mPacket.reduced_model.phi_velocity = mPacket.reduced_model.reduced_model_velocity - mPacket.inverse3_velocity

    # below force terms, ddot{q_inv} - non-inertial term, effective force, interface spring force
    regular_force_terms = (
        mPacket.params.effective_mass @ (mPacket.inverse3_acceleration) / mPacket.hl
        + mPacket.params.effective_force
        - mPacket.reduced_model.stiffness * mPacket.reduced_model.phi_position
        + (mPacket.reduced_model.damping + mPacket.hl * mPacket.reduced_model.stiffness) * mPacket.inverse3_velocity
    )
    mPacket.reduced_model.reduced_model_velocity = (
        mPacket.mass_factor @ mPacket.reduced_model.reduced_model_velocity
        + mPacket.hl * mPacket.inverse_augmented_mass_matrix @ regular_force_terms
    )
    mPacket.reduced_model.reduced_model_position = (
        mPacket.reduced_model.reduced_model_position + mPacket.hl * mPacket.reduced_model.reduced_model_velocity
    )


def catchUpProcess(connection, key):
    simulating = True
    mPacket = None
    position_queue = []
    velocity_queue = []
    while simulating:
        # Wait for an instruction,
        if not connection.poll() and mPacket is not None:
            # print(f'key: {key}, sending packet')
            # print(f'key: {key}, delay: {mPacket.delay}')
            # print(mPacket)
            connection.send(mPacket)
            # print(f'key: {key}, packet sent')

        # print(f'key: {key}, waiting for instruction')
        instruction = connection.recv()
        # print(f'key: {key}, instruction received: {instruction[0]}')

        # Instruction format:
        # [instruction_type, i3_position, i3_velocity, packet: Packet, hl, delay, RIMType]
        if instruction[0] == CatchUpCommand.CATCHUP:
            # packet: instruction[3], hl: instruction[4], delay: instruction[5], RIMType: instruction[6]
            position_queue.append(instruction[1])  # i3 position queue
            velocity_queue.append(instruction[2])  # i3 velocity queue
            mPacket = RIMPacket(instruction[3], instruction[4], instruction[5], instruction[6], key)

            while len(position_queue) > (mPacket.delay + 1):
                position_queue.pop(0)
                velocity_queue.pop(0)

            if mPacket.RIMType == "DelayRIM":
                # print(f'key: {key}, catching up')
                step = 0
                mPacket.inverse3_position = position_queue[0]
                mPacket.inverse3_velocity = velocity_queue[0]
                while step < len(position_queue) - 1:
                    step = step + 1
                    mPacket.inverse3_position = position_queue[step]
                    mPacket.inverse3_acceleration = velocity_queue[step] - mPacket.inverse3_velocity
                    mPacket.inverse3_velocity = velocity_queue[step]
                    oneStep(mPacket)

                # print(f'key: {key}, getting the force')
                getForce(mPacket)

            elif mPacket.RIMType == "ZOH":
                mPacket.inverse3_position = position_queue[0]
                mPacket.inverse3_velocity = velocity_queue[0]
                getZOHForce(mPacket)

            else:
                mPacket.inverse3_position = position_queue[-1]
                mPacket.inverse3_velocity = velocity_queue[-1]
                getZOHForce(mPacket)

        # When no new data - just step once
        elif instruction[0] == CatchUpCommand.ONESTEP:
            # new position: instruction[7], new velocity: instruction[8]
            position_queue.append(instruction[1])
            velocity_queue.append(instruction[2])

            if mPacket.RIMType == "DelayRIM":
                # print(f'key: {key}, one step')
                mPacket.inverse3_position = position_queue[-1]
                mPacket.inverse3_acceleration = velocity_queue[-1] - velocity_queue[-2]
                mPacket.inverse3_velocity = velocity_queue[-1]
                oneStep(mPacket)
                # print(f'key: {key}, getting the force')
                getForce(mPacket)
            elif mPacket.RIMType == "ZOH":
                mPacket.inverse3_position = position_queue[0]
                mPacket.inverse3_velocity = velocity_queue[0]
                getZOHForce(mPacket)
            else:
                mPacket.inverse3_position = position_queue[-1]
                mPacket.inverse3_velocity = velocity_queue[-1]
                getZOHForce(mPacket)

        elif instruction[0] == CatchUpCommand.KINEMATIC:
            position_queue.append(instruction[1])
            velocity_queue.append(instruction[2])

        elif instruction[0] == CatchUpCommand.CLEAR:
            mPacket = None
            position_queue.append(instruction[1])
            velocity_queue.append(instruction[2])

        elif instruction[0] == CatchUpCommand.END:
            simulating = False

    # print(f'key: {key}, PROCESS CLOSING')
    return


def makeCatchUpProcess(key):
    parent_conn, child_conn = Pipe()
    p = Process(
        target=catchUpProcess,
        args=(
            child_conn,
            key,
        ),
    )
    p.start()
    return (p, parent_conn)


class delayRIMModel:
    def __init__(self, *argv):
        noOfProcesses = argv[0]
        self.processKeys = [x for x in range(0, noOfProcesses)]
        self.allProcesses = []
        self.allConnections = []

        for x in range(0, noOfProcesses):
            [process, connection] = makeCatchUpProcess(x)
            self.allProcesses.append(process)
            self.allConnections.append(connection)

        self.activeProcesses = []
        self.inactiveProcesses = copy.deepcopy(self.processKeys)
        self.timeStamp = 0
        self.myState = None

    # packet instruction[1], position queue: instruction[2], velocity queue: instruction[3]
    # hl: instruction[4], delay: instruction[5], RIMType: instruction[6]
    def startCatchUp(self, packet: Packet, inverse3_position, inverse3_velocity, hl, delay, RIMType):
        if len(self.inactiveProcesses) > 0:
            process = self.inactiveProcesses.pop(0)
            # print(f'starting process: {process}, with packet: {packet}')
            self.activeProcesses.append(process)
            # print(f'process appended')
            self.allConnections[process].send(
                [CatchUpCommand.CATCHUP, inverse3_position, inverse3_velocity, packet, hl, delay, RIMType]
            )
            # print(f'process: {process} begun')

        # ? If no inactive processes are available, create a new one?
        else:
            self.processKeys.append(len(self.processKeys))
            [new_process, new_connection] = makeCatchUpProcess(self.processKeys[-1])
            self.allProcesses.append(new_process)
            self.allConnections.append(new_connection)
            self.activeProcesses.append(self.processKeys[-1])
            self.allConnections[self.processKeys[-1]].send(
                [CatchUpCommand.CATCHUP, inverse3_position, inverse3_velocity, packet, hl, delay, RIMType]
            )

    def getUpdate(self, inverse3_position, inverse3_velocity) -> RIMPacket | None:
        # go through all active processes and see if any have finished

        #! Find ready processes
        processesToFinish = []
        newFlag = False
        # print(self.inactiveProcesses)
        # print(self.activeProcesses)
        # print(self.processKeys)
        tic = time.perf_counter()

        # Send the KINEMATIC command to all inactive processes (just upate I3 )
        for k in self.inactiveProcesses:
            self.allConnections[k].send([CatchUpCommand.KINEMATIC, inverse3_position, inverse3_velocity])
        activeConnections = [self.allConnections[k] for k in self.activeProcesses]
        ready_connections = multiprocessing.connection.wait(activeConnections, 0)
        not_ready_connections = [connection for connection in activeConnections if connection not in ready_connections]

        for connection in ready_connections:
            packet = None
            while connection.poll():
                packet: RIMPacket = connection.recv()
            # print(f'retrieving key: {key}')
            processesToFinish.append(packet.key)
            if packet.timeStamp > self.timeStamp:
                newFlag = True
                returnData = packet
            connection.send([CatchUpCommand.CLEAR, inverse3_position, inverse3_velocity])

        for connection in not_ready_connections:
            # print(f'continuing: {key}')
            connection.send([CatchUpCommand.ONESTEP, inverse3_position, inverse3_velocity])

        # print(f'process poll time: {time.perf_counter() - tic}')
        # remove completed active processes from the active list and add them to the inactive list
        tic = time.perf_counter()
        for key in processesToFinish:
            self.inactiveProcesses.append(key)
        self.activeProcesses = [process for process in self.activeProcesses if process not in processesToFinish]

        # print(f'clean up processes: {time.perf_counter() - tic}')
        # choose what to return (either info or a null result)
        if newFlag == True:
            return returnData
        else:
            return None

    def endAllProcesses(self):
        # send the END command to all the processes
        for key in self.processKeys:
            self.allConnections[key].send([CatchUpCommand.END])
            self.allProcesses[key].join()
        self.allProcesses.clear()
        self.allConnections.clear()

    # This is the interface function, all models have this.
    def getForceFromData(self, packet: Packet, hl, delay, RIMType, recent_position, recent_velocity):
        # print('starting catch up')
        catchUpData: RIMPacket = self.getUpdate(recent_position, recent_velocity)

        tic0 = time.perf_counter()
        self.startCatchUp(packet, recent_position, recent_velocity, hl, delay, RIMType)
        # print(f'starting time: {time.perf_counter() - tic0}')
        # print('catch up begun and getting update')
        tic = time.perf_counter()

        # print(f'catchup time: {time.perf_counter() - tic}')
        # print('update retrieved')
        if catchUpData is None and self.myState is not None:
            catchUpData = self.myState
            catchUpData.inverse3_position = recent_position
            catchUpData.inverse3_acceleration = recent_velocity = catchUpData.inverse3_velocity
            catchUpData.inverse3_velocity = recent_velocity
            oneStep(catchUpData)
            getForce(catchUpData)
        tic = time.perf_counter()
        self.myState = catchUpData
        # print(f'copy time: {time.perf_counter() - tic}')

        # print(f'total force time: {time.perf_counter() - tic0}')
        if catchUpData is None:
            interface_force = np.zeros((3, 1))
            interface_force.shape = (3, 1)
            return interface_force
        else:
            return (-1) * 0.02 * catchUpData.reduced_model.interface_force
