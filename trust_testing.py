#!/usr/bin/env python

import os
import random
import sys
import traci, utils
from plexe import Plexe, ACC, GEAR, RPM
from utils import *
import matplotlib.pyplot as plt

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

MAX_STEP = 4000
CLAIMING_VEHICLE = 'v0'
VERIFYING_VEHICLE = 'v1'

vehicle_ids = [CLAIMING_VEHICLE, VERIFYING_VEHICLE]
data = {vid: {"times": [], "accelerations": [], "velocities": []} for vid in vehicle_ids}

def plot_data():
    fig, axs = plt.subplots(2)  # create a figure with 2 subplots

    # plot the acceleration data for each vehicle
    for vid in vehicle_ids:
        axs[0].plot(data[vid]["times"], data[vid]["accelerations"], label=f"{vid} Acceleration")

    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Acceleration (m/s^2)')
    axs[0].set_title('Vehicle Acceleration')
    axs[0].legend()  # add a legend to distinguish the different vehicles

    # plot the velocity data for each vehicle
    for vid in vehicle_ids:
        axs[1].plot(data[vid]["times"], data[vid]["velocities"], label=f"{vid} Velocity")

    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Speed (m/s)')
    axs[1].set_title('Vehicle Speed')
    axs[1].legend()  # add a legend to distinguish the different vehicles

    plt.tight_layout()  # adjust the subplot layout to make it more readable
    plt.show()

def choose_behavior(plexe, vid):
    # vid: vehicle id
    # rand number decides which action the vehicle takes
    plexe.set_fixed_acceleration(vid, True, 0) # remove any acc

    action = random.randint(0, 3)
    if (action == 0):
        # Change lane
        new_lane = random.randint(0, 3)
        print(f"changing to lane {new_lane}")
        plexe.set_fixed_lane(vid, new_lane, False)
    elif (action == 1):
        # accelerate
        print("accelerating")
        plexe.set_fixed_acceleration(vid, True, 6)
    elif (action == 2):
        # decelerate
        print("decelerating")
        plexe.set_fixed_acceleration(vid, True, -6)
    elif (action == 3):
        # do nothing
        print("no action")
        return

def main(real_engine, setter=None):

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
        elif step == 1:
            add_vehicle(plexe, CLAIMING_VEHICLE, 140, 1, 25, "passenger")
            add_vehicle(plexe, VERIFYING_VEHICLE, 250, 0, 20, "passenger")
            traci.gui.trackVehicle("View #0", VERIFYING_VEHICLE)
            traci.gui.setZoom("View #0", 60000)

        # insert vehicle behaviors every interval
        if step % 400 == 1:
            choose_behavior(plexe, VERIFYING_VEHICLE)

        step += 1

    traci.close()

if __name__ == "__main__":
    main(True)
    plot_data()

