import os
import random, numpy
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
from vehicles import Vehicles
import json

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

SIMULATION_SECONDS = 20
MAX_STEP = 100 * SIMULATION_SECONDS
CLAIMING_VEHICLE = 'v.0'
VERIFYING_VEHICLE = 'v.1'
attack = Attacks()
ATTACK_STEP = 1000


# cruising speed
velocity = 30
estimatedVelocity = velocity
pred = 0
accel_est = 0
accel_pred = 0
LENGTH = 4
ACC_HEADWAY=1.5
HEADWAY_DISTANCE=ACC_HEADWAY * velocity
SENSOR_REFRESH = 10 # centiseconds
beginSendingMessages = 100 * 1

# inter-vehicle distance
DISTANCE = ACC_HEADWAY * velocity + 2

# plotting information
vehicle_ids = [CLAIMING_VEHICLE, VERIFYING_VEHICLE]
data = {vid: {"times": [], "accelerations": [], "velocities": []} for vid in vehicle_ids}
sensor_data = {"times": [], "sensor_velocities": [], "filtered_velocities": [], "sensor_accelerations": [], \
               "filtered_accelerations": [], "message_speed": [], "message_acceleration": [], "time_delay": [], \
                "des_time_delay": []}
trust_data = {"times": [], "trust": []}

def plot_data():
    fig, axs = plt.subplots(2, 3, figsize=(15, 7))

    axs[0, 0].plot(sensor_data["times"], sensor_data["time_delay"], color="black", label="Actual Time Delay")
    axs[0, 0].plot(sensor_data["times"], sensor_data["des_time_delay"], color="green", label="Desired Time Delay")
    axs[0, 0].set_xlabel('Simulation Time (s)')
    axs[0, 0].set_ylabel('Delay Time (s)')
    axs[0, 0].set_title('Intervehicular Delay Time')
    axs[0, 0].legend()

    # plot the acceleration data for each vehicle
    for vid in vehicle_ids:
        if (vid == CLAIMING_VEHICLE):
            name = "Claimer"
            color = "black"
        else:
            name = "Verifier"
            color = "gray"
        axs[1, 1].plot(data[vid]["times"], data[vid]["accelerations"], label=f"{name} Accel", color = color)
    axs[1, 1].plot(sensor_data["times"], sensor_data["message_acceleration"], label=f"Message Accel", linestyle="dashed", color="red")
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Acceleration (m/s^2)')
    axs[1, 1].set_title('Vehicle Acceleration')
    axs[1, 1].legend()  # add a legend to distinguish the different vehicles

    # plot the velocity data for each vehicle
    for vid in vehicle_ids:
        if (vid == CLAIMING_VEHICLE):
            name = "Claimer"
            color = "black"
        else:
            name = "Verifier"
            color = "gray"

        axs[0, 1].plot(data[vid]["times"], data[vid]["velocities"], label=f"{name} Velocity", color=color)
    # plot the message information
    axs[0, 1].plot(sensor_data["times"], sensor_data["message_speed"], label=f"Message Velocity", linestyle="dashed", color="red")
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Speed (m/s)')
    axs[0, 1].set_title('Vehicle Speed')
    axs[0, 1].legend()  # add a legend to distinguish the different vehicles

    # plot the velocity sensor information
    axs[0, 2].scatter(sensor_data["times"], sensor_data["sensor_velocities"], label=f"Sensor Velocity", color="gray", s=1)
    axs[0, 2].plot(data[CLAIMING_VEHICLE]["times"], data[CLAIMING_VEHICLE]["velocities"], label=f"{name} Velocity", color="black")
    axs[0, 2].scatter(sensor_data["times"], sensor_data["filtered_velocities"], label=f"Filtered Velocity", color="red", s=1)
    axs[0, 2].set_xlabel('Time (s)')
    axs[0, 2].set_ylabel('Speed (m/s)')
    axs[0, 2].set_title('Sensor Speed')
    axs[0, 2].legend()  # add a legend to distinguish the different vehicles

    # plot the trust data
    axs[1, 0].scatter(trust_data["times"], trust_data["trust"], color='black', s=1)
    axs[1, 0].set_xlabel('Time')
    axs[1, 0].set_ylabel('Trust')
    axs[1, 0].set_ylim([0, 1])
    axs[1, 0].set_title('Trust Score')

    # plot the acceleration sensor information
    axs[1, 2].plot(sensor_data["times"], sensor_data["sensor_accelerations"], label=f"Sensor Accel", color="gray")
    axs[1, 2].plot(data[CLAIMING_VEHICLE]["times"], data[CLAIMING_VEHICLE]["accelerations"], label=f"{name} Accel", color="black")
    axs[1, 2].plot(sensor_data["times"], sensor_data["filtered_accelerations"], label=f"Filtered Accel", color="red")
    axs[1, 2].set_xlabel('Time (s)')
    axs[1, 2].set_ylabel('Acceleration (m/s^2)')
    axs[1, 2].set_title('Sensor Acceleration')
    axs[1, 2].legend()  # add a legend to distinguish the different vehicles
    plt.tight_layout()  # adjust the subplot layout to make it more readable
    plt.show()



def append_data(message_data):
    # get information
    trust_score = vehicles[1].getTrustScore()
    claim_speed_sensor = vehicles[1].getSensorClaimedSpeed()
    accel = vehicles[1].getAccelFromSensor()
    sensor_info = vehicles[1].getSensorData()
    time = traci.simulation.getTime()

    for vid in vehicle_ids:
        data[vid]["times"].append(time)
        data[vid]["accelerations"].append(traci.vehicle.getAcceleration(vid))
        data[vid]["velocities"].append(traci.vehicle.getSpeed(vid))
    trust_data["times"].append(time)
    trust_data["trust"].append(trust_score)
    sensor_data["times"].append(time)
    sensor_data["sensor_velocities"].append(claim_speed_sensor)
    sensor_data["filtered_velocities"].append(sensor_info.speed)
    sensor_data["sensor_accelerations"].append(accel)
    sensor_data["filtered_accelerations"].append(sensor_info.acceleration)
    sensor_data["time_delay"].append(vehicles[1].getTimeDelay())
    sensor_data["des_time_delay"].append(vehicles[1].getdesTimeDelay())
    sensor_data["message_acceleration"].append(message_data.acceleration)
    sensor_data["message_speed"].append(message_data.speed)


def add_vehicles(plexe, n, real_engine=False):
    global vehicles
    vehicles = [Vehicles("v.%d"%i, (n - i + 1) * (DISTANCE + LENGTH) / 2, 0, velocity,plexe) for i in range(n)]


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

            # for random behaviors

            behavior_interval = 200

        # # Use this for benign scenarios
        # if (step % behavior_interval == 1 and step < ATTACK_STEP):
        #     vehicles[0].setAcceleration(numpy.random.normal(0, 1.5))
        #     behavior_interval = int(numpy.random.normal(500, 100))
        #     if behavior_interval == 0:
        #         behavior_interval == 1

        # use this to test a benign rapid braking
        # if (step > 1000 and step < 1600):
        #     vehicles[0].setAcceleration(-6)
        # elif (step == 1600):
        #     vehicles[0].setAcceleration(3)
        # elif (step == 2400):
        #     vehicles[0].setAcceleration(0)

        if (step > 0 and step < ATTACK_STEP):
            v2_data = vehicles[0].buildMessage()
            claim_lane = vehicles[0].getLane()
    
            vehicles[0].sendMessage(v2_data, vehicles[1], vehicles[0], claim_lane, trust_score.trust, step)

            # get info for graphing
            trust_score.trust = vehicles[1].getTrustScore()
            append_data(v2_data)

        if (step >= ATTACK_STEP):
            claim_lane = vehicles[0].getLane()
            attack.teleportationAttack(plexe, v2_data, CLAIMING_VEHICLE, VERIFYING_VEHICLE)
            vehicles[0].sendMessage(v2_data, vehicles[1], vehicles[0], claim_lane, trust_score.trust, step)
            trust_score.trust = vehicles[1].getTrustScore()
            append_data(v2_data)

        # end the simulation if a crash occurs
        if (plexe.get_crashed(CLAIMING_VEHICLE)):
            traci.close()
            return
        step += 1

    traci.close()

if __name__ == "__main__":
    main()
    plot_data()
    with open(f'sensor_data.json', 'w') as f:
        json.dump(sensor_data, f)
    with open(f'vehicle_data.json', 'w') as f:
        json.dump(data, f)

