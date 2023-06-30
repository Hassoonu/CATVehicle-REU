'''
TODO:
Send V2V messages through controller
'''

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

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

MAX_STEP = 3000
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
        add_vehicle(plexe, vid, (n - i + 1) * (DISTANCE + LENGTH) / 2, 0, velocity)
        plexe.set_fixed_lane(vid, 0, safe=False)
        traci.vehicle.setSpeedMode(vid, 0)
        plexe.use_controller_acceleration(vid, False)
        plexe.set_active_controller(vid, ACC)
        plexe.set_acc_headway_time(vid, ACC_HEADWAY)
        plexe.set_cc_desired_speed(vid, 20)

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

def desired_acceleration(plexe, v1, v2_data, v2_lane):
    '''
    :param v1: Verifying Vehicle
    :param v2_data: Attacker False VehicleData object

    un = K1 * s∆ + K2 * ∆v * R(s), if s <= rFRACC
         K1 * (v0 - vn) * td, if s > rFRACC

    un: desired acceleration
    s: forward space gap
    K1, K2: control feedback coefficients
    rFRACC: detection range of forward sensor
    ∆v: relative speed wrt the preceding vehicle
    s∆: spacing error given by:
    s∆ = min{s - s0 - vn * td, (v0 - vn) * td}
    s0: minimum space gap between vehicles at standstill
    td: desired time gap
    v0: free-flow mode velocity (desired velocity)
    vn: longitudinal vehicle velocity
    R(s): space gap-dependent velocity-error response for forward
        collision avoidance
    R(s) = 1 - [1 / (1 + Q * e^-(s / P))]
    Q: aggressiveness coefficient
    P: perception range coefficient based on detection range
        of the forward sensors
    '''
    td = 1.2; s0 = 3; v0 = velocity; Q = 1; P = 100; K1 = 0.18; K2 = 1.93     # params
    d1 = plexe.get_vehicle_data(v1); # get vehicle info

    # check if cars are in the same lane
    if (traci.vehicle.getLaneID(v1)[-1] == v2_lane):
        s = v2_data.__getitem__(POS_X) - d1.__getitem__(POS_X) - LENGTH # calculate space gap
        vn = d1.__getitem__(SPEED); vn2 = v2_data.__getitem__(SPEED) # vehicle speeds
    else:
        s = 100 # calculate space gap
        vn = d1.__getitem__(SPEED); vn2 = velocity # vehicle speeds      
    del_s = min(s - s0 - vn * td, (v0 - vn) * td)   # calculate spacing error
    R_s = 1 - (1 / (1 + Q * math.pow(math.e, -1 * (s / P))))    # calculate error response for collision avoidance

    des_acc = K1 * del_s + K2 * (vn2 - vn) * R_s    # finally, calculate desired acceleration
    print(f"Desired accel: {des_acc}")
    return des_acc

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
            #plexe.set_fixed_lane(CLAIMING_VEHICLE, 1, False)

        # if step == 400:
        #     false_message = build_message(plexe, CLAIMING_VEHICLE)
        #     plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, -8)
        # elif step == 1200:
        #     plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, 3)
        # elif step == 2000:
        #     plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, -4)
        if step % 20 == 1 and step < 200:
            v2_data = plexe.get_vehicle_data(CLAIMING_VEHICLE)
            claim_lane = traci.vehicle.getLaneID(CLAIMING_VEHICLE)[-1]
            des_acc = desired_acceleration(plexe, VERIFYING_VEHICLE, v2_data, claim_lane)
            plexe.set_fixed_acceleration(VERIFYING_VEHICLE, True, des_acc)
        if step == 200:
            false_message = build_message(plexe, CLAIMING_VEHICLE)
        if step % 20 == 1 and step > 200:
            # BEGIN ATTACK!!
            false_vehicle_data = VehicleData()
            #attack.falseLaneAttack(plexe, CLAIMING_VEHICLE, VERIFYING_VEHICLE)
            print(f"Verifier accel. = {plexe.get_vehicle_data(VERIFYING_VEHICLE).__getitem__(ACCELERATION)}" )
            false_vehicle_data.pos_x = false_message.posX; false_vehicle_data.pos_y = false_message.posY; \
            false_vehicle_data.speed = false_message.speed; false_vehicle_data.acceleration = false_message.acceleration
            claim_lane = int(traci.vehicle.getLaneID(CLAIMING_VEHICLE)[-1]) + 1
            claim_lane = str(claim_lane)
            # find acceleration based on false message
            des_acc = desired_acceleration(plexe, VERIFYING_VEHICLE, false_vehicle_data, attack.falseLaneAttack(plexe, CLAIMING_VEHICLE, VERIFYING_VEHICLE))
            plexe.set_fixed_acceleration(VERIFYING_VEHICLE, True, des_acc)
        step += 1

    traci.close()

if __name__ == "__main__":
    main()
    plot_data()
