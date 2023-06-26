import os
import random
import sys
import traci, utils
from plexe import Plexe, DRIVER, ACC, CACC, RPM, GEAR, RADAR_REL_SPEED, SPEED, RADAR_DISTANCE, ACCELERATION
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
        plexe.set_fixed_acceleration(vid, True, 0.5)
    elif (action == 2):
        # decelerate
        print("decelerating")
        plexe.set_fixed_acceleration(vid, True, -0.5)
    elif (action == 3):
        # do nothing
        print("no action")
        return

def add_vehicles(plexe, n, real_engine=False):
    # add n vehicles
    for i in range(n):
        vid = "v.%d" % i
        add_vehicle(plexe, vid, (n - i + 1) * (DISTANCE + LENGTH) / 2, 0, velocity)
        plexe.set_fixed_lane(vid, 0, safe=False)
        traci.vehicle.setSpeedMode(vid, 0)
        plexe.use_controller_acceleration(vid, False)
        if i == 0:
            plexe.set_active_controller(vid, ACC)
            plexe.set_acc_headway_time(vid, ACC_HEADWAY)
        else:
            plexe.set_active_controller(vid, ACC)
            plexe.set_acc_headway_time(vid, ACC_HEADWAY)
        if i > 0:
            plexe.enable_auto_feed(vid, True, CLAIMING_VEHICLE, "v.%d" % (i-1))
            plexe.set_cc_desired_speed(vid, 50)

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

        # insert vehicle behaviors every interval
        if step % 400 == 200:
            choose_behavior(plexe, CLAIMING_VEHICLE)
            '''
            false_data = VehicleData(index=0, speed=0, acceleration=0, pos_x=0, pos_y=0, time=0, length=0, u=0)
            plexe.set_vehicle_data(VERIFYING_VEHICLE, false_data)'''
        '''    
        if step % 100 == 1:
            vd = plexe.get_vehicle_data(CLAIMING_VEHICLE)
            print(vd.__getitem__(SPEED))
            print(vd.__getitem__(ACCELERATION))
            plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, -6)'''

        step += 1

    traci.close()

if __name__ == "__main__":
    main()
    plot_data()

