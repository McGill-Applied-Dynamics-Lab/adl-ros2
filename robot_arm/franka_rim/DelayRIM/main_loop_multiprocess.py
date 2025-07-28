from multiprocessing import Process, Pipe
import time
import local_processes
import virtual_environment
import pandas as pd
import vpython as vp
from time import sleep
import numpy as np
import communication_line
import math
import csv
import datetime

class ProgramState:
    def __init__(self):
        self.isStarting = False
        self.isRunning = False
        self.isStopping = False
        self.isQuitting = False
        self.isSafe = True

class TaskState:
    def __init__(self, num_maneuvers, num_tasks):
        self.baseline = True
        self.task = 0
        self.user = 0
        self.maneuver = 0
        self.rank = [[0 for x in range(0, num_tasks)] for y in range(0, num_maneuvers)]
        self.unsavedData = False

class SimulationConfig:
    def __init__(self):
        self.latencyModel = 'ZOH'
        self.communicationModel = 'C'
        self.constantDelay = 1
        self.performTask = False


mProgramState = ProgramState()
mSimulationConfig = SimulationConfig()
multi_user_data = pd.read_csv('UserOutput/multi_user_df.csv')
num_tasks = len(multi_user_data.loc[(multi_user_data['User'] == 0) & (multi_user_data['Maneuver'] == 0)])
num_maneuvers = len(multi_user_data.loc[(multi_user_data['User'] == 0) & (multi_user_data['Task'] == 0)])
num_users = len(multi_user_data.loc[(multi_user_data['Task'] == 0) & (multi_user_data['Maneuver'] == 0)])
mTaskState = TaskState(num_maneuvers, num_tasks)
for maneuver in range(0, num_maneuvers):
    for task in range(0, num_tasks):
        task_row = multi_user_data.loc[
            (multi_user_data['User'] == mTaskState.user) & (multi_user_data['Maneuver'] == maneuver) & (
                    multi_user_data['Task'] == task)].index
        mTaskState.rank[maneuver][task] = multi_user_data.at[task_row.values[0], 'Ranking']

def communication_model_dict(num):
    dict = {0: 'C10', 1: 'C50', 2: 'C75', 3: '5G'}
    return dict[num]

def latency_model_dict(num):
    dict = {0: 'ZOH', 1: 'ZOHPhi', 2: 'DelayRIM'}
    return dict[num]

def maneuver_dict(num):
    dict = {0: 'Free Movement', 1: 'Guided Movement'}
    return dict[num]

def get_config(multi_user_data, user, maneuver, task):

    if task == -1:
        constantDelay = 1
        communicationModel = 'C1'
        latencyModel = 'ZOH'
    else:
        # Get all the tasks done by a given user
        user_tasks = multi_user_data.loc[(multi_user_data['User'] == user) & (multi_user_data['Maneuver'] == maneuver)]

        # Get the communicationModel, latencyModel, and if a constant communication model is used, the constantDelay value
        communicationModel = communication_model_dict(user_tasks.iloc[task]['CommunicationModel'])
        latencyModel = latency_model_dict(user_tasks.iloc[task]['LatencyModel'])
        constantDelay = 1
        if (communicationModel != '5G' and communicationModel != 'C1'):
            constantDelay = int(communicationModel[1:3])

    return communicationModel, latencyModel, constantDelay

def force_average(forceList):
    averageForce = [0,0,0]
    for i in range(0,len(forceList)):
        for j in range(0,3):
            averageForce[j] = averageForce[j]+forceList[i][j]

    averageForce[0] = averageForce[0] / len(forceList)
    averageForce[1] = averageForce[1] / len(forceList)
    averageForce[2] = averageForce[2] / len(forceList)
    return averageForce

def get_rank_text():
    output_text ='      '
    for maneuver in range(0, num_maneuvers):
        output_text = output_text + f' {maneuver_dict(maneuver)}  '
    output_text = output_text + '\n'
    for task in range(0, num_tasks):
        output_text = output_text + f'{task}    '
        for maneuver in range(0, num_maneuvers):
            output_text = output_text + f' {mTaskState.rank[maneuver][task]}/10    ' + ' '*len(maneuver_dict(maneuver))
        output_text = output_text + '\n'
    return output_text

def update_gui(saveButton, ranks):
    if mTaskState.unsavedData == True:
        saveButton.background = vp.color.red
    else:
        saveButton.background = vp.color.green
    ranks.text = get_rank_text()

def update_graphics(end_effector, drone, rod1, rod2, load, inverse3_position, rod1_com, rod2_com, drone_com, load_com,
                    drone_axis, rod1_axis, rod2_axis, load_axis, end_effector_label, scene1):
    drone.axis = vp.vector(-drone_axis[0], -drone_axis[1], -drone_axis[2])
    drone.pos = vp.vector(drone_com[0], drone_com[1], drone_com[2])

    rod1.axis = vp.vector(-2.*rod1_axis[0], -2.*rod1_axis[1], -2.*rod1_axis[2])
    rod1.pos = vp.vector(rod1_com[0] + rod1_axis[0], rod1_com[1] + rod1_axis[1], rod1_com[2] + rod1_axis[2])

    rod2.axis = vp.vector(-2.*rod2_axis[0], -2.*rod2_axis[1], -2.*rod2_axis[2])
    rod2.pos = vp.vector(rod2_com[0] + rod2_axis[0], rod2_com[1] + rod2_axis[1], rod2_com[2] + rod2_axis[2])

    load.axis = vp.vector(load_axis[0], load_axis[1], load_axis[2])
    load.pos = vp.vector(load_com[0], load_com[1], load_com[2])

    end_effector.pos = vp.vector(inverse3_position[0], inverse3_position[1], inverse3_position[2])

    end_effector_label.text=f'{end_effector.pos.x:.2f},{end_effector.pos.y:.2f},{end_effector.pos.z:.2f}'
    end_effector_label.pos=end_effector.pos

    if mProgramState.isSafe is False:
        scene1.background = vp.color.red

def B(b):
    if mProgramState.isRunning:
        mProgramState.isStopping = True
        b.text = '\n  Start  \n\n'
        b.background = vp.color.green
    else:
        mProgramState.isStarting = True
        b.text = '\n  Stop  \n\n'
        b.background = vp.color.red

def MBaseline(m):
    if m.checked:
        mTaskState.baseline = True
    else:
        mTaskState.baseline = False

def MUser(m):
    mTaskState.user = m.index

def MManeuver(m):
    mTaskState.maneuver = m.index
    if(mTaskState.maneuver == 1):
        mSimulationConfig.performTask = True
    else:
        mSimulationConfig.performTask = False

def MTask(m):
        mTaskState.task = m.index

def MRanking(m):
    mTaskState.rank[mTaskState.maneuver][mTaskState.task] = m.index
    mTaskState.unsavedData = True

def SaveAll(m):
    for maneuver in range(0, num_maneuvers):
        for task in range(0, num_tasks):
            task_row = multi_user_data.loc[
                (multi_user_data['User'] == mTaskState.user) & (multi_user_data['Maneuver'] == maneuver) & (
                            multi_user_data['Task'] == task)].index
            multi_user_data.at[task_row.values[0], 'Ranking'] = mTaskState.rank[maneuver][task]
    multi_user_data.to_csv('UserOutput/multi_user_df.csv', index=False)
    mTaskState.unsavedData = False

def display_setup(drone_radius, rod_length, rod_radius, load_radius, multi_user_data):

    scene1 = vp.canvas()

    scene1.width = 725
    scene1.height = 600
    scene1.range = 1.8
    scene1.forward = vp.vector(0., 1., 0.)
    scene1.center = vp.vector(0., 0., 0.)
    scene1.up = vp.vector(0., 0., 1.)
    scene1.camera.pos = vp.vector(0., -0.7, -0.1)
    scene1.background = vp.color.gray(0.2)
    scene1.align = 'left'

    vp.scene.width = 725
    vp.scene.height = 1
    vp.background = vp.color.white

    # Sphere at base of the double pendulum which will serve as the interface body, represents simple spherical drone model
    drone = vp.sphere(pos=vp.vector(0., 0., 0.), axis=vp.vector(0.,0.,-1.), radius=drone_radius, color=vp.color.blue, opacity=0.5)
    drone_com = np.array([0., 0., 0.])
    drone_com.shape = (3, 1)

    end_effector = vp.sphere(pos=vp.vector(0., 0., 0.), axis=vp.vector(0.,0.,-1), radius=drone_radius, color=vp.color.yellow, opacity=0.5)
    end_effector_label = vp.label(pos = end_effector.pos, text=f'{end_effector.pos.x:.2f},{end_effector.pos.y:.2f},{end_effector.pos.z:.2f}', xoffset=-10, yoffset=20, opacity=0, height=10)

    taskball = vp.sphere(pos=vp.vector(0., 0., 0.), axis=vp.vector(0.,0.,-1.), radius = 0.2*drone_radius, color=vp.color.black, opacity=0)

    # Make two rods that constitute the double pendulum and set their initial orientation relative to the sphere which
    # is initialized to be at the origin
    rod1 = vp.cylinder(pos=vp.vector(0, 0., -drone_radius), axis=vp.vector(0.,0.,-1.), length=rod_length, radius=rod_radius, color=vp.color.white)
    rod1_com = np.array([0., 0., -drone_radius-rod_length/2.])
    rod1_com.shape = (3, 1)

    rod2 = vp.cylinder(pos=vp.vector(0., 0., -drone_radius-rod_length), axis=vp.vector(0.,0.,-1.), length=rod_length, radius=rod_radius, color=vp.color.white)
    rod2_com = np.array([0., 0., -drone_radius-3.*rod_length/2.])
    rod2_com.shape = (3, 1)

    load = vp.sphere(pos=vp.vector(0., 0., -drone_radius-2*rod_length-load_radius), axis=vp.vector(0.,0.,-1.), radius=load_radius, color=vp.color.red)
    load_com = np.array([0., 0., -drone_radius-2.*rod_length-load_radius])
    load_com.shape = (3, 1)

    # Simulation control
    task_choices = [str(x) for x in range(0, num_tasks)]
    maneuver_choices = [maneuver_dict(x) for x in range(0, num_maneuvers)]
    user_choices = [str(x) for x in range(0, num_users)]
    rank_choices = [str(x) for x in range (0, 11)]

    scene1.append_to_title('<b>Delay RIM Demo</b>\n')
    vp.button(canvas=scene1, bind=B, text='\n  Start  \n\n', color=vp.color.black, background=vp.color.green, pos=vp.scene.title_anchor)
    scene1.append_to_title('   \n')
    vp.wtext(canvas=scene1, text=' Baseline:', pos=vp.scene.title_anchor)
    vp.checkbox(canvase=scene1,bind=MBaseline, pos=vp.scene.title_anchor)
    vp.wtext(canvas=scene1, text=' User:', pos=vp.scene.title_anchor)
    vp.menu(canvas=scene1, bind=MUser, choices=user_choices, pos=vp.scene.title_anchor)
    vp.wtext(canvas=scene1, text='  Maneuver:', pos=vp.scene.title_anchor)
    vp.menu(canvas=scene1, bind=MManeuver, choices=maneuver_choices, pos=vp.scene.title_anchor)
    vp.wtext(canvas=scene1, text='  Task:', pos=vp.scene.title_anchor)
    vp.menu(canvas=scene1, bind=MTask, choices=task_choices, pos=vp.scene.title_anchor)
    vp.scene.append_to_title('\n\n')
    vp.wtext(canvas=scene1, text='  Rank:', pos=vp.scene.title_anchor)
    vp.menu(canvas=scene1, bind=MRanking, choices=rank_choices, pos=vp.scene.title_anchor)
    saveButton = vp.button(canvas=scene1, bind=SaveAll, text='  Save  ', color=vp.color.black, background=vp.color.green,
              pos=vp.scene.title_anchor)
    vp.scene.append_to_title('\n\n')
    ranking_text = get_rank_text()
    ranks = vp.wtext(canvas=scene1, text=ranking_text, pos=vp.scene.title_anchor)

    return drone, rod1, rod2, load, end_effector, end_effector_label, scene1, taskball, saveButton, ranks

class VirtualPacket:
    def __init__(self, effective_params=virtual_environment.EffectiveParams(), virtual_state=virtual_environment.VirtualState()):
        self.effective_params = effective_params
        self.virtual_state = virtual_state

def VirtualEnvironmentProcess(connection, hv, mass_drone, drone_radius, mass_density, rod_length, rod_radius, mass_load,load_radius):
    #Create virtual environment
    mVirtualEnvironment = virtual_environment.VirtualEnvironment(hv=hv, mass_drone=mass_drone, drone_radius=drone_radius,
                                                                 mass_density=mass_density, rod_length=rod_length, rod_radius=rod_radius,
                                                                 mass_load=mass_load,load_radius=load_radius)
    effective_params = mVirtualEnvironment.get_effective_params()
    virtual_state = mVirtualEnvironment.get_virtual_environment_state()
    virtual_packet = VirtualPacket(effective_params, virtual_state)
    connection.send(virtual_packet)
    while True:
        effective_params = mVirtualEnvironment.get_effective_params()
        virtual_state = mVirtualEnvironment.get_virtual_environment_state()
        virtual_packet = VirtualPacket(effective_params, virtual_state)
        connection.send(virtual_packet)
        inverse_state = connection.recv() # waits until the information is received
        if inverse_state != None:
            #Take the next step
            mVirtualEnvironment.virtual_main(inverse_state)
        else:
            #We're done, leave the function so the process can be terminated.
            stuff = 1
            break

    print("Virtual Process Ending")

def send_force_to_Inverse3(Inverse3, rendered_force, inverse3_initial_position):

    effective_force_to_send = [rendered_force[0,0], rendered_force[1,0], rendered_force[2,0]]
    inverse3_raw_position, inverse3_raw_velocity = Inverse3.end_effector_force(effective_force_to_send)
    inverse3_position = np.asarray([inverse3_raw_position[0] - inverse3_initial_position[0], inverse3_raw_position[1] - inverse3_initial_position[1], inverse3_raw_position[2] - inverse3_initial_position[2]])
    inverse3_position.shape = (3, 1)
    inverse3_velocity = np.asarray(inverse3_raw_velocity)
    inverse3_velocity.shape = (3, 1)

    return virtual_environment.Inverse3State(inverse3_position, inverse3_velocity)

def saveData(trajectory, forces):
    #Create filename
    if mProgramState.isSafe:
        stable = ""
    else:
        stable = "Unsafe"
    now = datetime.now()
    current_time = now.strftime("%H%M%S")
    filename = f"UserOutput/User_{mTaskState.user}/Task_{mTaskState.task}_Maneuver_{mTaskState.maneuver}_{stable}_{current_time}.csv"
    #Create header row
    headerRow = ['X', 'Y', 'Z', 'Fx', 'Fy', 'Fz']

    # open the file in the write mode
    with open(filename, 'w', newline='') as f:
        # create the csv writer
        writer = csv.writer(f)
        # write rows to the csv file
        writer.writerow(headerRow)
        for i in range(0,len(trajectory)):
            writer.writerow([trajectory[i][0][0], trajectory[i][1][0], trajectory[i][2][0], forces[i][0], forces[i][1], forces[i][2]])

def main():

    mModel = None

    hl = 0.001
    fGraphics = 100
    vp.rate(fGraphics)

    multirate_factor = 4
    multi_user_data = pd.read_csv('multi_user_df.csv')

    mass_drone = 2.5
    drone_radius = 0.05
    mass_density = 10000
    rod_length = 0.1
    rod_radius = 0.01
    mass_load = 2.5
    load_radius = 0.03
    mCap = 7.5

    drone, rod1, rod2, load, end_effector, end_effector_label, scene1, taskball, saveButton, ranks = display_setup(drone_radius, rod_length, rod_radius, load_radius, multi_user_data)
    sleep(2)
    # Wakeup Inverse3 and get the position and velocity of the end effector
    port = "COM6"
    com = HaplyHardwareAPI.SerialStream(port)
    Inverse3 = HaplyHardwareAPI.Inverse3(com)
    Inverse3.device_wakeup()
    print("Made Handshake with device")

    count = 0
    tic = 0
    trajectoryForSaving = []
    forcesForSaving = []

    while True:

        tic = time.perf_counter()

        #If simulation is starting, then we do the initialization
        if mProgramState.isStarting:

            communicationModel, latencyModel, constantDelay = get_config(multi_user_data, mTaskState.user, mTaskState.maneuver, mTaskState.task)
            mSimulationConfig.latencyModel = latencyModel
            mSimulationConfig.communicationModel = communicationModel
            mSimulationConfig.constantDelay = constantDelay

            communicationLine = communication_line.CommunicationLine()
            connectionVirtual, connectionLocal = Pipe()
            virtualProcess = Process(target=VirtualEnvironmentProcess,
                                     args=(connectionVirtual, hl * multirate_factor, mass_drone, drone_radius,
                                           mass_density, rod_length, rod_radius, mass_load, load_radius))
            virtualProcess.start()
            sleep(1)
            Inverse3.device_wakeup()
            initial_force = [0., 0., 0.]
            inverse3_raw_position, inverse3_raw_velocity = Inverse3.end_effector_force(initial_force)
            reduced_model_force = np.zeros((3, 1))
            reduced_model_force.shape = (3, 1)
            inverse3_initial_position = inverse3_raw_position
            inverse3_position = np.asarray([inverse3_raw_position[0] - inverse3_initial_position[0],
                                            inverse3_raw_position[1] - inverse3_initial_position[1],
                                            inverse3_raw_position[2] - inverse3_initial_position[2]])
            inverse3_position.shape = (3, 1)
            inverse3_velocity = np.asarray(inverse3_raw_velocity)
            inverse3_velocity.shape = (3, 1)
            inverse3_state = virtual_environment.Inverse3State(inverse3_position, inverse3_velocity)

            mModel = local_processes.delayRIMModel(4)

            mProgramState.isStarting = False
            mProgramState.isRunning = True

            count = 0

            trajectoryForSaving = []
            forcesForSaving = []
            savedFlag = False

        if mProgramState.isRunning:
            # Get information from each environment
            if (count % multirate_factor == 0):
                # recieve state from connection
                virtual_packet = connectionLocal.recv()
                effective_params = virtual_packet.effective_params
                virtual_state = virtual_packet.virtual_state
                # send inverse state
                connectionLocal.send(inverse3_state)
            else:
                effective_params = None

            communicationLine.addToLine(effective_params, count)
            # Update positions for the graphics (note that this isn't a draw call, the graphics is drawing stuff independently
            update_graphics(end_effector, drone, rod1, rod2, load, inverse3_state.inverse3_position,
                            virtual_state.rod1_com, virtual_state.rod2_com, virtual_state.drone_com,
                            virtual_state.load_com,
                            virtual_state.drone_axis, virtual_state.rod1_axis, virtual_state.rod2_axis,
                            virtual_state.load_axis,
                            end_effector_label, scene1)

            if mSimulationConfig.performTask:
                taskball.opacity = 1
                if count*hl > 2 and savedFlag == False:
                    scene1.background = vp.color.gray(0.7)
                else:
                    scene1.background = vp.color.gray(0.2)
                if count*hl < 4:
                    taskball.pos = vp.vector(0.,0.,0,)
                elif count*hl < 6:
                    taskball.pos = vp.vector(0.,0.,0.25*math.sin((count*hl-6)*math.pi))
                elif count*hl < 8:
                    taskball.pos = vp.vector(0.,0.,0.)
                elif count*hl < 10:
                    taskball.pos = vp.vector(0.,0.25*math.sin((count*hl-8)*math.pi),0.)
                elif count*hl < 12:
                    taskball.pos = vp.vector(0.,0.,0.)
                elif count*hl < 14:
                    taskball.pos = vp.vector(0.25*math.sin((count*hl-10)*math.pi),0.,0.)
                elif count*hl < 16:
                    taskball.pos = vp.vector(0.,0.,0.)
                elif (savedFlag == False and mTaskState.baseline == False):
                    saveData(trajectoryForSaving, forcesForSaving)
                    savedFlag = True
            else:
                taskball.opacity = 0
                if count * hl > 2 and savedFlag == False:
                    scene1.background = vp.color.gray(0.7)
                else:
                    scene1.background = vp.color.gray(0.2)
                if (count*hl > 16 and savedFlag == False and mTaskState.baseline == False):
                    saveData(trajectoryForSaving, forcesForSaving)
                    savedFlag = True

            if (count % 10 == 0 and count*hl > 2 and count*hl <= 16 and mTaskState.baseline == False):
                #add the current state and force to the list for saving
                trajectoryForSaving.append(inverse3_state.inverse3_position)
                forcesForSaving.append([rendered_force[0, 0], rendered_force[1, 0], rendered_force[2, 0]])

            #Get the packet for this step from the communication line
            cur_packet = communicationLine.getCurrent()
            print(cur_packet)
            if cur_packet.info is not None:
                #Send the packet to the model and get the force
                print(f'timeStamp: {count}')
                tic2 = time.perf_counter()
                rendered_force = mModel.getForceFromData(cur_packet, hl, cur_packet.delay, mSimulationConfig.latencyModel, inverse3_state.inverse3_position, inverse3_state.inverse3_velocity)
                print(f'obtained force time: {time.perf_counter()-tic}')

                if mTaskState.baseline == True:
                    #(self, packet, hl, delay, RIMType, key):
                    mPacket = local_processes.RIMPacket(cur_packet, hl, cur_packet.delay, mSimulationConfig.latencyModel, 0)
                    mPacket.reduced_model.reduced_model_position.shape = (3, 1)
                    mPacket.reduced_model.reduced_model_velocity.shape = (3, 1)

                    mPacket.reduced_model.phi_position = mPacket.reduced_model.reduced_model_position - inverse3_state.inverse3_position
                    mPacket.reduced_model.phi_velocity = mPacket.reduced_model.reduced_model_velocity - inverse3_state.inverse3_velocity
                    raw_force = - mPacket.reduced_model.stiffness * mPacket.reduced_model.phi_position - mPacket.reduced_model.damping * mPacket.reduced_model.phi_velocity - inverse3_state.inverse3_velocity
                    raw_force.shape = (3, 1)
                    rendered_force = (-1) * 0.02 * raw_force

                #print(rendered_force)
                #rendered_force = [0,0,0]
                #Check rendered force is below the cap
                if abs(rendered_force[0]) > mCap or abs(rendered_force[1]) > mCap or abs(rendered_force[2]) > mCap:
                    mProgramState.isSafe = False

                #Apply the rendered force, and update the inverse3 state queues with the new state
                if (mProgramState.isSafe):
                    inverse3_state = send_force_to_Inverse3(Inverse3, rendered_force, inverse3_initial_position)
                else:
                    rendered_force[0, 0] = 0
                    rendered_force[1, 0] = 0
                    rendered_force[2, 0] = 0
                    inverse3_state = send_force_to_Inverse3(Inverse3, rendered_force, inverse3_initial_position)

            #Apply the force to the inverse

        if mProgramState.isStopping:
            mProgramState.isStopping = False
            mProgramState.isRunning = False
            mProgramState.isSafe = True
            connectionLocal.send(None)
            virtualProcess.join()
            mModel.endAllProcesses()

        # Wait for the time step to finish
        endTime = time.perf_counter()
        while (endTime - tic) < hl:
            endTime = time.perf_counter()
        print(f'step time: {endTime - tic}')
        count = count + 1

        update_gui(saveButton, ranks)

if __name__ == '__main__':
    main()