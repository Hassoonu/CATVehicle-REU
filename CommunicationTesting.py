import os
import random
import sys
import traci, utils
from vehicle_message import VehicleMessage
from plexe import Plexe, ACC, SPEED, ACCELERATION, TIME
from utils import *
import matplotlib.pyplot as plt
from vehicle_data import VehicleData
from attacks import Attacks
from vehicles import Vehicles

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


MAX_STEP = 1000
CLAIMING_VEHICLE = 'v.0'
VERIFYING_VEHICLE = 'v.1'
attack = Attacks()

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
    fig, axs = plt.subplots(2)  # create a figure with 2 subplots, NEED 'fig', don't delete it.

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
    global vehicles
    vehicles = [Vehicles("v.%d"%i, (n - i + 1) * (DISTANCE + LENGTH) / 2, 0, velocity,plexe) for i in range(n)]
    for i in range(n):
        vid = "v.%d" % i
        add_vehicle(plexe, vid, (n - i + 1) * (DISTANCE + LENGTH) / 2, 0, velocity)
        plexe.set_fixed_lane(vid, 0, safe=False)
        traci.vehicle.setSpeedMode(vid, 0)
        plexe.use_controller_acceleration(vid, False)
        plexe.set_active_controller(vid, ACC)
        plexe.set_acc_headway_time(vid, ACC_HEADWAY)
        plexe.set_cc_desired_speed(vid, 30)

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
            traci.gui.setZoom("View #0", 45000)
            traci.vehicle.setColor(CLAIMING_VEHICLE, (255,0,0)) 
            traci.vehicle.setColor(VERIFYING_VEHICLE, (255,255,255))

        if step % 20 == 1 and step < 200:
            
            v2_data = plexe.get_vehicle_data(CLAIMING_VEHICLE)
            claim_lane = vehicles[0].getLane()
            desiredAcceleration = vehicles[1].getDesiredAcceleration(VERIFYING_VEHICLE, v2_data, claim_lane)
            vehicles[1].setAcceleration(desiredAcceleration)
            print(f"Desired Acceleration = {desiredAcceleration}")
        if step == 200:

            false_message = vehicles[0].buildMessage()
            vehicles[0].setAcceleration(0)

        if step % 20 == 1 and step > 200:
            # BEGIN ATTACK!!
            attack.teleportationAttack(plexe, false_message, CLAIMING_VEHICLE, VERIFYING_VEHICLE)
            false_vehicle_data = VehicleData()
            vehicles[0].copyDataFromMessage(false_vehicle_data, false_message)
            
            # find acceleration based on false message
            claim_lane = vehicles[1].getLane()
            desiredAcceleration = vehicles[1].getDesiredAcceleration(VERIFYING_VEHICLE, false_vehicle_data, claim_lane)
            vehicles[1].setAcceleration(desiredAcceleration)
            
            print(f"Desired Acceleration = {desiredAcceleration:.2f}")
            
        step += 1

    traci.close()

if __name__ == "__main__":
    main()
    plot_data()

