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

MAX_STEP = 1000
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

def choose_behavior(vid):
    plexe = Plexe()
    # vid: vehicle id
    # Change lane
    new_lane = random.randint(0, 2)
    plexe.set_fixed_lane(vid, new_lane, False)
    #Plexe.plexe.set_fixed_lane(CLAIMING_VEHICLE, lane=new_lane, safe=False)
    # speed up
    # slow down

def main(real_engine, setter=None):

    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0
    random.seed(1)
    while running(False, step, max_step=MAX_STEP):

        traci.simulationStep()
        if step > 1:
            time = traci.simulation.getTime()
            for vid in vehicle_ids:
                data[vid]["times"].append(time)
                data[vid]["accelerations"].append(traci.vehicle.getAcceleration(vid))
                data[vid]["velocities"].append(traci.vehicle.getSpeed(vid))
        elif step == 1:
            add_vehicle(plexe, CLAIMING_VEHICLE, 140, 1, 25, "vtypeauto")
            add_vehicle(plexe, VERIFYING_VEHICLE, 250, 0, 20, "passenger2")
            traci.gui.trackVehicle("View #0", VERIFYING_VEHICLE)
            traci.gui.setZoom("View #0", 50000)

        # insert vehicle behaviors every interval
        if step % 400 == 1:
            #choose_behavior(VERIFYING_VEHICLE)
            plexe.set_fixed_lane(VERIFYING_VEHICLE, 1, False)

        step += 1

    traci.close()

if __name__ == "__main__":
    main(True)
    plot_data()

