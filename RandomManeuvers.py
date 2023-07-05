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

MAX_STEP = 6000
# Vehicle IDs
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
    newMessage = VehicleMessage(posX=posX, posY=posY, speed=speed, acceleration=acceleration, \
                                timestamp=timestamp, id=vid)
    return newMessage

def choose_behavior(plexe, vid, dist_between):
    # vid: vehicle id
    # rand number decides which action the vehicle takes
    plexe.set_fixed_acceleration(vid, True, 0) # remove any acc

    action = random.randint(0, 3)
    if (action == 0):
        # Change lane
        new_lane = random.randint(0, 3)
        print(f"changing to lane {new_lane}")
        plexe.set_fixed_lane(vid, new_lane, False)
    elif (action == 1 or dist_between < -20):
        # accelerate
        print("accelerating")
        plexe.set_fixed_acceleration(vid, True, 0.5)
    if (action == 2 or dist_between > 50):
        # decelerate
        print("decelerating")
        plexe.set_fixed_acceleration(vid, True, -0.5)
    if (action == 3):
        # do nothing
        print("no action")
        return

def desired_acceleration(plexe, v1, v2, v2_data):
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
    if (traci.vehicle.getLaneID(v1)[-1] == traci.vehicle.getLaneID(v2)[-1]):
        s = v2_data.__getitem__(POS_X) - d1.__getitem__(POS_X) - LENGTH # calculate space gap
        vn = d1.__getitem__(SPEED); vn2 = v2_data.__getitem__(SPEED) # vehicle speeds
    else:
        s = 100 # calculate space gap
        vn = d1.__getitem__(SPEED); vn2 = velocity # vehicle speeds      
    del_s = min(s - s0 - vn * td, (v0 - vn) * td)   # calculate spacing error
    R_s = 1 - (1 / (1 + Q * math.pow(math.e, -1 * (s / P))))    # calculate error response for collision avoidance

    des_acc = K1 * del_s + K2 * (vn2 - vn) * R_s    # finally, calculate desired acceleration
    return des_acc

def main():
    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0
    random.seed(2)
    #random.seed(3)
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
        if step % 20 == 1:
            v2_data = plexe.get_vehicle_data(CLAIMING_VEHICLE)
            des_acc = desired_acceleration(plexe, VERIFYING_VEHICLE, CLAIMING_VEHICLE, v2_data)
            plexe.set_fixed_acceleration(VERIFYING_VEHICLE, True, des_acc)
            if step % 40 == 1:
                print(f"Desired Acceleration = {des_acc:.3f}")
        if step % 400 == 1:
            dist = plexe.get_vehicle_data(CLAIMING_VEHICLE).__getitem__(POS_X) - plexe.get_vehicle_data(VERIFYING_VEHICLE).__getitem__(POS_X)
            choose_behavior(plexe, CLAIMING_VEHICLE, dist)
            # ranAcc = random.randint(-15, 10)
            # plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, ranAcc)
            # print(f"Random acceleration: = {ranAcc}")
        step += 1

    traci.close()

if __name__ == "__main__":
    main()
    plot_data()

