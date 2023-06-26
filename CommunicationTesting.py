'''
TODO:
Figure out how V2V messages are sent
'''

import os
import random
import sys
import traci, utils
from vehicle_message import VehicleMessage
from plexe import Plexe, DRIVER, ACC, CACC, RPM, GEAR, RADAR_REL_SPEED, \
    SPEED, RADAR_DISTANCE, ACCELERATION, TIME, U, INDEX
from utils import *
import matplotlib.pyplot as plt
from vehicle_data import VehicleData

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

MAX_STEP = 2500
CLAIMING_VEHICLE = 'v.0'
VERIFYING_VEHICLE = 'v.1'

# cruising speed
velocity = 30

LENGTH = 4

ACC_HEADWAY=1.5
HEADWAY_DISTANCE=ACC_HEADWAY * velocity

# inter-vehicle distance

DISTANCE = ACC_HEADWAY * velocity + 2

# falsified message
false_message = VehicleMessage()

# variables for plotting
vehicle_ids = [CLAIMING_VEHICLE, VERIFYING_VEHICLE]
data = {vid: {"times": [], "accelerations": [], "velocities": []} for vid in vehicle_ids}

def plot_data():
    fig, axs = plt.subplots(2)  # create a figure with 2 subplots

    # plot the acceleration data for each vehicle
    for vid in vehicle_ids:
        if (vid == CLAIMING_VEHICLE):
            name = "Claimer"
        else:
            name = "Verifier"
        axs[0].plot(data[vid]["times"], data[vid]["accelerations"], label=f"{name} Acceleration")

    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Acceleration (m/s^2)')
    axs[0].set_title('Vehicle Acceleration')
    axs[0].legend()  # add a legend to distinguish the different vehicles

    # plot the velocity data for each vehicle
    for vid in vehicle_ids:
        if (vid == CLAIMING_VEHICLE):
            name = "Claimer"
        else:
            name = "Verifier"
        axs[1].plot(data[vid]["times"], data[vid]["velocities"], label=f"{name} Velocity")

    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Speed (m/s)')
    axs[1].set_title('Vehicle Speed')
    axs[1].legend()  # add a legend to distinguish the different vehicles

    plt.tight_layout()  # adjust the subplot layout to make it more readable
    plt.show()

def add_vehicles(plexe, n, real_engine=False):
    # add n vehicles
    for i in range(n):
        vid = "v.%d" % i
        add_vehicle(plexe, vid, (n - i + 1) * (DISTANCE + LENGTH) / 1.4, 0, velocity)
        plexe.set_fixed_lane(vid, 0, safe=False)
        traci.vehicle.setSpeedMode(vid, 0)
        plexe.use_controller_acceleration(vid, False)
        plexe.set_active_controller(vid, ACC)
        plexe.set_acc_headway_time(vid, ACC_HEADWAY)
        plexe.set_cc_desired_speed(vid, 30)
        '''
        if i > 0:
            plexe.enable_auto_feed(vid, True, CLAIMING_VEHICLE, "v.%d" % (i-1))
            plexe.set_cc_desired_speed(vid, 50)'''

def build_message(plexe, vid):
    # builds a message with current data
    vd = plexe.get_vehicle_data(vid)
    posX = vd.__getitem__(POS_X)
    posY = vd.__getitem__(POS_Y)
    acceleration = vd.__getitem__(ACCELERATION)
    speed = vd.__getitem__(SPEED)
    timestamp = vd.__getitem__(TIME)
    newMessage = VehicleMessage(posX, posY, speed, acceleration, timestamp)
    return newMessage

def send_message(plexe, claim, verifier):
    # Take info from message and broadcast to target vehicle
    fd = VehicleData(pos_x=claim.posX, pos_y=claim.posY, acceleration=claim.acceleration, \
                     speed=claim.speed, time=claim.timestamp)
    plexe.set_front_vehicle_data(verifier, fd)

def braking_attack(plexe, message, vid):
    '''
    simulates the kinematics of a vehicle suddenly braking
    # vid: attacker vehicle
    '''
    newTime = plexe.get_vehicle_data(vid).__getitem__(TIME)
    t = (newTime - message.timestamp)
    message.acceleration = -6
    if message.speed <= 0:
        message.speed = 0
    else:
        message.speed += message.acceleration * t
        message.posX += message.speed * t + message.acceleration / 2 * t * t
    message.timestamp = newTime

def main():
    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0
    random.seed()
    while running(False, step, max_step=MAX_STEP):

        traci.simulationStep()
        if step > 1:
            time = traci.simulation.getTime()
            for vid in vehicle_ids:
                data[vid]["times"].append(time)
                data[vid]["accelerations"].append(traci.vehicle.getAcceleration(vid))
                data[vid]["velocities"].append(traci.vehicle.getSpeed(vid))
        elif step == 0:
            add_vehicles(plexe, 2)
            traci.gui.trackVehicle("View #0", VERIFYING_VEHICLE)
            traci.gui.setZoom("View #0", 50000)
            traci.vehicle.setColor(CLAIMING_VEHICLE, (255,0,0)) 
            traci.vehicle.setColor(VERIFYING_VEHICLE, (255,255,255))
            '''
        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            # print(plexe.get_vehicle_data(CLAIMING_VEHICLE).__getitem__(ACCELERATION))
            send_message(plexe, CLAIMING_VEHICLE, VERIFYING_VEHICLE) 
            '''
        if step == 500:
            false_message = build_message(plexe, CLAIMING_VEHICLE)
        if step > 500:
            braking_attack(plexe, false_message, CLAIMING_VEHICLE)
            false_vehicle_data = VehicleData(length=4)
            false_vehicle_data.pos_x = false_message.posX; false_vehicle_data.pos_y = false_message.posY; \
                false_vehicle_data.speed = false_message.speed; false_vehicle_data.acceleration = false_message.acceleration
            plexe.set_front_vehicle_data(VERIFYING_VEHICLE, false_vehicle_data)
            plexe.set_vehicle_data(CLAIMING_VEHICLE, false_vehicle_data)
            plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, -6)
        if step % 20 == 1:
            cd = plexe.get_vehicle_data(VERIFYING_VEHICLE)
            print(f"Index = {cd.__getitem__(INDEX)}\tU = {cd.__getitem__(U)}")
        step += 1

    traci.close()

if __name__ == "__main__":
    main()
    plot_data()

