'''
TODO:
Send V2V messages through controller
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

MAX_STEP = 3000
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
        add_vehicle(plexe, vid, (n - i + 1) * (DISTANCE + LENGTH) / 2, 0, velocity)
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

def desired_acceleration(plexe, v1, v2_data):
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
    s = v2_data.__getitem__(POS_X) - d1.__getitem__(POS_X) - LENGTH # calculate space gap
    vn = d1.__getitem__(SPEED); vn2 = v2_data.__getitem__(SPEED) # vehicle speeds

    del_s = min(s - s0 - vn * td, (v0 - vn) * td)   # calculate spacing error
    R_s = 1 - (1 / (1 + Q * math.pow(math.e, -1 * (s / P))))    # calculate error response for collision avoidance

    des_acc = K1 * del_s + K2 * (vn2 - vn) * R_s    # finally, calculate desired acceleration
    return des_acc

def check_safety(v1, v2):
    '''
    v1: verifier vehicle
    v2: claiming vehicle
    '''
    plexe = Plexe()
    # Get distance from cars A and B
    d1 = plexe.get_vehicle_data(v1)
    d2 = plexe.get_vehicle_data(v2)
    # print(f"Distance between = {abs(abs(d2 - d1) - LENGTH):.2f}")
    dist_between = abs(abs(d2.__getitem__(POS_X) - d1.__getitem__(POS_X)) - LENGTH)

    # Calculate stopping distance
    max_deceleration = -8    # in m/s
    '''
    Stopping distance is the dist required to make a complete stop
    '''
    stopping_dist = d1.__getitem__(SPEED) * d1.__getitem__(SPEED) / (-2 * max_deceleration)
    relative_acc = d2.__getitem__(ACCELERATION) - d1.__getitem__(ACCELERATION)

    print(f"Distance between = {dist_between:.2f} Stopping dist = {stopping_dist:.2f} Speed = {d1.__getitem__(SPEED)} Relative Acc = {relative_acc}")

    # check if there is relative acceleration between the two vehicles
    if (dist_between <= stopping_dist and d2.__getitem__(ACCELERATION) < 0):
        if (relative_acc < -2):
            plexe.set_fixed_acceleration(v1, True, max_deceleration)
            print("Braking")
        elif (relative_acc < -1):
            plexe.set_fixed_acceleration(v1, True, -1)
            print("Slowing")
    # # if distance is too close, increase distance
    # elif (dist_between < stopping_dist):
    #     plexe.set_fixed_acceleration(v1, True, -1)
    #     print("Increasing distance")
    else:
        sp = d2.__getitem__(SPEED)
        if (sp < velocity):
            plexe.set_cc_desired_speed(v1, sp)
        else:
            plexe.set_cc_desired_speed(v1, velocity)
            print("Set speed to cruising velocity")
    return

def defensive_action():
    return

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
            '''
        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            # print(plexe.get_vehicle_data(CLAIMING_VEHICLE).__getitem__(ACCELERATION))
            send_message(plexe, CLAIMING_VEHICLE, VERIFYING_VEHICLE) 
            '''
        # if step == 700:
        #     false_message = build_message(plexe, CLAIMING_VEHICLE)
        #     plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, -8)
        # elif step == 1200:
        #     plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, 3)
        # elif step == 2000:
        #     plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, -4)
        if step % 20 == 1 and step < 700:
            # check_safety(VERIFYING_VEHICLE, CLAIMING_VEHICLE)
            v2_data = plexe.get_vehicle_data(CLAIMING_VEHICLE)
            des_acc = desired_acceleration(plexe, VERIFYING_VEHICLE, v2_data)
            plexe.set_fixed_acceleration(VERIFYING_VEHICLE, True, des_acc)
            print(f"Desired Acceleration = {des_acc}")
        if step == 700:
            false_message = build_message(plexe, CLAIMING_VEHICLE)
        if step % 20 == 1 and step > 700:
            # BEGIN ATTACK!!
            braking_attack(plexe, false_message, CLAIMING_VEHICLE)
            false_vehicle_data = VehicleData()
            false_vehicle_data.pos_x = false_message.posX; false_vehicle_data.pos_y = false_message.posY; \
                false_vehicle_data.speed = false_message.speed; false_vehicle_data.acceleration = false_message.acceleration
            des_acc = desired_acceleration(plexe, VERIFYING_VEHICLE, false_vehicle_data)
            plexe.set_fixed_acceleration(VERIFYING_VEHICLE, True, des_acc)
            print(f"Desired Acceleration = {des_acc}")
        step += 1

    traci.close()

if __name__ == "__main__":
    main()
    plot_data()

