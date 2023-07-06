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
from reputation import Reputation

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
SENSOR_REFRESH = 20 # milliseconds

# inter-vehicle distance
DISTANCE = ACC_HEADWAY * velocity + 2

# falsified message
false_message = VehicleMessage()
trust_score = Reputation(0.5)

# variables for plotting
vehicle_ids = [CLAIMING_VEHICLE, VERIFYING_VEHICLE]
data = {vid: {"times": [], "accelerations": [], "velocities": []} for vid in vehicle_ids}
trust_data = {"time": [], "trust": []}

def plot_data():
    fig, axs = plt.subplots(3)  # create a figure with 2 subplots

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

    # plot the trust data
    axs[2].plot(trust_data["time"], trust_data["trust"], label=f"Trust Score")
    axs[2].set_xlabel('Time')
    axs[2].set_ylabel('Trust')
    axs[2].set_title('Trust Score')

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
            trust_data["time"].append(time)
            trust_data["trust"].append(trust_score.trust)
            # trust_score.trust = abs(math.sin(step / (2 * math.pi)) / 2 * math.pow(math.e, -1 * step / 100) + 0.5)
        elif step == 0:
            trust_score = Reputation(0.5)
            add_vehicles(plexe, 2)
            traci.gui.trackVehicle("View #0", VERIFYING_VEHICLE)
            traci.gui.setZoom("View #0", 45000)
            traci.vehicle.setColor(CLAIMING_VEHICLE, (255,0,0)) 
            traci.vehicle.setColor(VERIFYING_VEHICLE, (255,255,255))
        if step % SENSOR_REFRESH == 1 and step < 200:  # sensor refresh rate
            false_message = build_message(plexe, CLAIMING_VEHICLE)
            sensor_info = build_message(plexe, CLAIMING_VEHICLE)
            v2_data = plexe.get_vehicle_data(CLAIMING_VEHICLE)
            claim_lane = traci.vehicle.getLaneID(CLAIMING_VEHICLE)[-1]
            des_acc = desired_acceleration(plexe, VERIFYING_VEHICLE, v2_data, claim_lane)
            plexe.set_fixed_acceleration(VERIFYING_VEHICLE, True, des_acc)
            # print(f"Desired Acceleration = {des_acc:.2f}")
        if step == 200:
            false_message = build_message(plexe, CLAIMING_VEHICLE)
        if step > 200 and step % SENSOR_REFRESH == 1:
            sensor_info = build_message(plexe, CLAIMING_VEHICLE)
            attack.falseBrake(plexe, false_message, CLAIMING_VEHICLE)
            false_vehicle_data = VehicleData()
            false_vehicle_data.pos_x = false_message.posX; false_vehicle_data.pos_y = false_message.posY; \
            false_vehicle_data.speed = false_message.speed; false_vehicle_data.acceleration = false_message.acceleration
            claim_lane = traci.vehicle.getLaneID(CLAIMING_VEHICLE)[-1]
            des_acc = desired_acceleration(plexe, VERIFYING_VEHICLE, false_vehicle_data, claim_lane)
            plexe.set_fixed_acceleration(VERIFYING_VEHICLE, True, des_acc)
        if step % SENSOR_REFRESH == 1:
            trust_score.UpdateTrustScore(sensor_info, false_message)
        step += 1

    traci.close()

if __name__ == "__main__":
    main()
    plot_data()

