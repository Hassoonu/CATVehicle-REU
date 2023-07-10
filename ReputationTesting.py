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
from Sensors import addNoise, kalmanFilter

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

MAX_STEP = 2000
CLAIMING_VEHICLE = 'v.0'
VERIFYING_VEHICLE = 'v.1'
attack = Attacks()


# cruising speed
velocity = 30
LENGTH = 4
ACC_HEADWAY=1.5
HEADWAY_DISTANCE=ACC_HEADWAY * velocity
SENSOR_REFRESH = 10 # centiseconds

# inter-vehicle distance
DISTANCE = ACC_HEADWAY * velocity + 2

# falsified message
false_message = VehicleMessage()
trust_score = Reputation(0.5)

# variables for plotting
vehicle_ids = [CLAIMING_VEHICLE, VERIFYING_VEHICLE]
data = {vid: {"times": [], "accelerations": [], "velocities": []} for vid in vehicle_ids}
sensor_data = {"times": [], "sensor_velocities": [], "filtered_velocities": [], "sensor_accelerations": [], \
               "filtered_accelerations": []}
trust_data = {"times": [], "trust": []}

def plot_data():
    fig, axs = plt.subplots(2, 3, figsize=(15, 8))

    # plot the acceleration data for each vehicle
    for vid in vehicle_ids:
        if (vid == CLAIMING_VEHICLE):
            name = "Claimer"
        else:
            name = "Verifier"
        axs[0, 0].plot(data[vid]["times"], data[vid]["accelerations"], label=f"{name} Acceleration")
    
    # axs[0, 0].plot(sensor_data["times"], sensor_data["sensor_accelerations"], label=f"Sensor Accl")

    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Acceleration (m/s^2)')
    axs[0, 0].set_title('Vehicle Acceleration')
    axs[0, 0].legend()  # add a legend to distinguish the different vehicles

    # plot the velocity data for each vehicle
    for vid in vehicle_ids:
        if (vid == CLAIMING_VEHICLE):
            name = "Claimer"
        else:
            name = "Verifier"
        axs[0, 1].plot(data[vid]["times"], data[vid]["velocities"], label=f"{name} Velocity")
   
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Speed (m/s)')
    axs[0, 1].set_title('Vehicle Speed')
    axs[0, 1].legend()  # add a legend to distinguish the different vehicles

    # plot the trust data
    axs[1, 0].plot(trust_data["times"], trust_data["trust"], label=f"Trust Score")
    axs[1, 0].set_xlabel('Time')
    axs[1, 0].set_ylabel('Trust')
    axs[1, 0].set_title('Trust Score')

    axs[0, 2].plot(data[CLAIMING_VEHICLE]["times"], data[CLAIMING_VEHICLE]["velocities"], label=f"{name} Velocity")
    axs[0, 2].plot(sensor_data["times"], sensor_data["sensor_velocities"], label=f"Sensor Velocity")
    axs[0, 2].plot(sensor_data["times"], sensor_data["filtered_velocities"], label=f"Filtered Velocity")
    axs[0, 2].set_xlabel('Time (s)')
    axs[0, 2].set_ylabel('Speed (m/s)')
    axs[0, 2].set_title('Vehicle Speed')
    axs[0, 2].legend()  # add a legend to distinguish the different vehicles

    axs[1, 2].plot(data[CLAIMING_VEHICLE]["times"], data[CLAIMING_VEHICLE]["accelerations"], label=f"{name} Accel")
    axs[1, 2].plot(sensor_data["times"], sensor_data["sensor_accelerations"], label=f"Sensor Accel")
    axs[1, 2].plot(sensor_data["times"], sensor_data["filtered_accelerations"], label=f"Filtered Accel")
    axs[1, 2].set_xlabel('Time (s)')
    axs[1, 2].set_ylabel('Speed (m/s)')
    axs[1, 2].set_title('Vehicle Acceleration')
    axs[1, 2].legend()  # add a legend to distinguish the different vehicles
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
    sensor_info = VehicleData(speed=None)
    while running(False, step, max_step=MAX_STEP):
        traci.simulationStep()
        if step == 0:
            trust_score = Reputation(0.5)
            add_vehicles(plexe, 2)
            traci.gui.trackVehicle("View #0", VERIFYING_VEHICLE)
            traci.gui.setZoom("View #0", 45000)
            traci.vehicle.setColor(CLAIMING_VEHICLE, (255,0,0)) 
            traci.vehicle.setColor(VERIFYING_VEHICLE, (255,255,255))

            est = velocity; pred = 0; accel_est = 0; accel_pred = 0
        if step % SENSOR_REFRESH == 1:  # sensor refresh rate
            time = traci.simulation.getTime()

            # create the sensor readings
            if sensor_info.speed == None:
                prev_est = velocity
            else:
                prev_est = est
            claim_speed_sensor = addNoise(traci.vehicle.getSpeed(CLAIMING_VEHICLE), 0.4)
            est, pred = kalmanFilter(claim_speed_sensor, state_est=est, prediction=pred)
            sensor_info = plexe.get_vehicle_data(CLAIMING_VEHICLE)
            v2_data = build_message(plexe, CLAIMING_VEHICLE)

            # calculate acceleration
            accel = (est - prev_est) / (SENSOR_REFRESH / 100)
            sensor_info.speed = est

            accel_est, accel_pred = kalmanFilter(accel, R=1, state_est=accel_est, prediction=accel_pred)
            sensor_info.acceleration = accel_est

            claim_lane = traci.vehicle.getLaneID(CLAIMING_VEHICLE)[-1]
            des_acc = desired_acceleration(plexe, VERIFYING_VEHICLE, sensor_info, claim_lane)
            plexe.set_fixed_acceleration(VERIFYING_VEHICLE, True, des_acc)
            # print(f"Desired Acceleration = {des_acc:.2f}")

            trust_score.UpdateTrustScore(sensor_info, v2_data)

            # graphing info
            for vid in vehicle_ids:
                data[vid]["times"].append(time)
                data[vid]["accelerations"].append(traci.vehicle.getAcceleration(vid))
                data[vid]["velocities"].append(traci.vehicle.getSpeed(vid))
            trust_data["times"].append(time)
            trust_data["trust"].append(trust_score.trust)
            sensor_data["times"].append(time)
            sensor_data["sensor_velocities"].append(claim_speed_sensor)
            sensor_data["filtered_velocities"].append(est)
            sensor_data["sensor_accelerations"].append(accel)
            sensor_data["filtered_accelerations"].append(sensor_info.acceleration)
        if step == 600:
            false_message = build_message(plexe, CLAIMING_VEHICLE)
            plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, 1)
        if step == 900:
            plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, -4)
        if step == 1100:
            plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, 1)
        if step == 1300:
            plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, 0.1)
        if step == 1700:
            plexe.set_fixed_acceleration(CLAIMING_VEHICLE, True, -6)
        # if step > 200 and step % SENSOR_REFRESH == 1:
        #     sensor_info = build_message(plexe, CLAIMING_VEHICLE)
        #     attack.falseBrake(plexe, false_message, CLAIMING_VEHICLE)
        #     false_vehicle_data = VehicleData()
        #     false_vehicle_data.pos_x = false_message.posX; false_vehicle_data.pos_y = false_message.posY; \
        #     false_vehicle_data.speed = false_message.speed; false_vehicle_data.acceleration = false_message.acceleration
        #     claim_lane = traci.vehicle.getLaneID(CLAIMING_VEHICLE)[-1]
        #     des_acc = desired_acceleration(plexe, VERIFYING_VEHICLE, false_vehicle_data, claim_lane)
        #     plexe.set_fixed_acceleration(VERIFYING_VEHICLE, True, des_acc)
        step += 1

    traci.close()

if __name__ == "__main__":
    main()
    plot_data()

