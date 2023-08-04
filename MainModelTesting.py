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
from showCaseVehicles import Vehicles
import json

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
RANDSEED = 2
SIMULATION_SECONDS = 120
MAX_STEP = 100 * SIMULATION_SECONDS
CLAIMING_VEHICLE = 'v.0'
VERIFYING_VEHICLE = 'v.1'
attack = Attacks()
ATTACK_STEP = 100 * 160
SCENARIO_STEP = 100 * 160
FONTSIZE = 24
file_path = 'output.txt'

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
sensor_data = {"times": [], "times0": [], "times1": [], "times2": [], "sensor_velocities": [], "filtered_velocities": [], "sensor_accelerations": [], \
               "filtered_accelerations": [], "message_speed": [], "message_acceleration": [], "time_delay": [], \
                "des_time_delay": [], "dist_between0": [], "dist_between1": [], "dist_between2": []}
trust_data = {"times": [], "trust": []}

# fig, axs = plt.subplots(2, 3, figsize=(15, 7))

def plot_data(i=0):
    global axs
    global fig
    #Trust acceleration time graph and distance trust time graph
    axs2 = axs[1, 1].twinx()

    axs[0, 0].scatter(sensor_data["times"], sensor_data["time_delay"], color="black", label="Actual Time Delay", s=1)
    axs[0, 0].scatter(sensor_data["times"], sensor_data["des_time_delay"], color="green", label="Desired Time Delay", s=1)
    axs[0, 0].set_xlabel('Simulation Time (s)')
    axs[0, 0].set_ylabel('Delay Time (s)')
    axs[0, 0].set_title('Intervehicular Delay Time')
    axs[0, 0].legend()
    colors = ["red", "blue", "green", "purple", "yellow"]
    print(f"MY COLOR IS: {colors[i]}")

    # plot the acceleration data for each vehicle
    for vid in vehicle_ids:
        if (vid == CLAIMING_VEHICLE):
            pass
            name = "Claimer"
            color = "black"
        else:
            name = "Verifier"
            color = "gray"
            axs[1, 1].scatter(data[vid]["times"], data[vid]["accelerations"], label=f"{name} Accel", color = color, s=1)
    axs2.scatter(trust_data["times"], trust_data["trust"], label=f"Vehicle Trust", color="blue", s=1)
    #axs[1, 1].plot(sensor_data["times"], sensor_data["message_acceleration"], label=f"Message Accel", linestyle="dashed", color="red")

    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Acceleration (m/s^2)')
    axs2.set_ylabel("Trust")
    axs2.set_ylim([0, 1])
    axs[1, 1].set_title('Vehicle Acceleration')
    axs[1, 1].legend(loc=0)  # add a legend to distinguish the different vehicles
    axs2.legend()

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
    axs[1, 0].scatter(sensor_data["times0"], sensor_data["dist_between0"], color = colors[0], s=1)
    axs[1, 0].scatter(sensor_data["times1"], sensor_data["dist_between1"], color = colors[1], s=1)
    axs[1, 0].scatter(sensor_data["times2"], sensor_data["dist_between2"], color = colors[2], s=1)
    axs[1, 0].set_xlabel('Time')
    axs[1, 0].set_ylabel("Distance Between Vehicles (m)")
    axs[1, 0].set_ylim([0, 100])
    axs[1, 0].legend()

    # plot the acceleration sensor information
    axs[1, 2].plot(sensor_data["times"], sensor_data["sensor_accelerations"], label=f"Sensor Accel", color="gray")
    axs[1, 2].plot(data[CLAIMING_VEHICLE]["times"], data[CLAIMING_VEHICLE]["accelerations"], label=f"{name} Accel", color="black")
    axs[1, 2].plot(sensor_data["times"], sensor_data["filtered_accelerations"], label=f"Filtered Accel", color="red")
    axs[1, 2].set_xlabel('Time (s)')
    axs[1, 2].set_ylabel('Acceleration (m/s^2)')
    axs[1, 2].set_title('Sensor Acceleration')
    axs[1, 2].legend()  # add a legend to distinguish the different vehicles
    plt.tight_layout()  # adjust the subplot layout to make it more readable
    #plt.show()

def trustOverTimeGraph(i=0):
    fig, axs = plt.subplots(figsize=(9, 7))
    global trust_data
    axs.scatter(trust_data["times"], trust_data["trust"], color='#DBE2EB', s=15, label = "Trust Score")
    axs.set_xlabel('Time (s)', fontsize = FONTSIZE)
    axs.set_ylabel('Trust', fontsize = FONTSIZE)
    axs.set_facecolor("#3C4F67")
    axs.tick_params(colors='white')
    axs.xaxis.label.set_color('white')
    axs.yaxis.label.set_color('white')
    axs.spines['bottom'].set_color('white')
    axs.spines['top'].set_color('white')
    axs.spines['left'].set_color('white')
    axs.spines['right'].set_color('white')
    axs.set_title("Trust Score Over Time", color='white', fontsize=FONTSIZE)

    if ATTACK_STEP < 120:
        axs.axvline(x=ATTACK_STEP / 100, color='r', linestyle='--', label='Attack Begins')
    axs.set_ylim([0, 1])
    axs.tick_params(axis = 'x', labelsize = FONTSIZE)
    axs.tick_params(axis = 'y', labelsize = FONTSIZE)
    legend = axs.legend(loc = 0, fontsize = FONTSIZE, scatterpoints=1, markerscale=3, facecolor='#393939')
    for text in legend.get_texts():
        text.set_color('white')
    plt.savefig("/Users/kylemack/Downloads/SIMULATION_DATA/Trust.png", transparent = True)

def distanceComparisonGraph():
    fig, axs = plt.subplots(figsize=(9, 7))
    global sensor_data
    colors = ["#84BF40", "#FCB900", "#DBE2EB"]
    axs.scatter(sensor_data["times0"], sensor_data["dist_between0"], color = colors[0], s=1, label = "Messaging Only")
    axs.scatter(sensor_data["times1"], sensor_data["dist_between1"], color = colors[1], s=1, label = "Sensors Only")
    axs.scatter(sensor_data["times2"], sensor_data["dist_between2"], color = colors[2], s=1, label = "Reputation + Sensors")
    if ATTACK_STEP < 120:
        axs.axvline(x=ATTACK_STEP / 100, color='r', linestyle='--', label='Attack Begins')
    axs.set_facecolor("#3C4F67")
    axs.tick_params(colors='white')
    axs.xaxis.label.set_color('white')
    axs.yaxis.label.set_color('white')
    axs.spines['bottom'].set_color('white')
    axs.spines['top'].set_color('white')
    axs.spines['left'].set_color('white')
    axs.spines['right'].set_color('white')
    axs.set_title("Distance Between Vehicles Over Time", color='white', fontsize=FONTSIZE)
    axs.set_xlabel('Time (s)', fontsize = FONTSIZE)
    axs.set_ylabel("Distance Between Vehicles (m)", fontsize = FONTSIZE)
    axs.set_ylim([0, 125])
    axs.tick_params(axis = 'x', labelsize = FONTSIZE)
    axs.tick_params(axis = 'y', labelsize = FONTSIZE)
    legend = axs.legend(loc = 0, fontsize = FONTSIZE, scatterpoints=1, markerscale=15, facecolor='#393939')
    for text in legend.get_texts():
        text.set_color('white')
    plt.savefig("/Users/kylemack/Downloads/SIMULATION_DATA/Dist.png", transparent = True)    
    plt.show()

def accelerationPlotGraph(i =0):
    fig, axs = plt.subplots(figsize=(9, 7))
    global data
    colors = ["red", "green", "black"]
    labels = ["Messaging Only", "Sensors Only", "Sensors + Reputation"]
    axs.scatter(data[CLAIMING_VEHICLE]["times"], data[CLAIMING_VEHICLE]["accelerations"], color = colors[i], s=1, label = labels[i])
    #axs.axvline(x=60, color='r', linestyle='--', label='Attack Begins')
    axs.set_xlabel('Time (s)', fontsize = 14)
    axs.set_ylabel("Claimer Acceleration (m/s^2)", fontsize = 14)
    axs.tick_params(axis = 'x', labelsize = 14)
    axs.tick_params(axis = 'y', labelsize = 14)
    axs.legend(loc = 0, fontsize = 14, scatterpoints=1, markerscale=15)

def append_data(message_data, i):
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
    if i == 0:
        sensor_data["dist_between0"].append(traci.vehicle.getPosition(CLAIMING_VEHICLE)[0] - traci.vehicle.getPosition(VERIFYING_VEHICLE)[0])
        sensor_data["times0"].append(time)
    elif i == 1:
        sensor_data["dist_between1"].append(traci.vehicle.getPosition(CLAIMING_VEHICLE)[0] - traci.vehicle.getPosition(VERIFYING_VEHICLE)[0])
        sensor_data["times1"].append(time)
    else:
        sensor_data["dist_between2"].append(traci.vehicle.getPosition(CLAIMING_VEHICLE)[0] - traci.vehicle.getPosition(VERIFYING_VEHICLE)[0])
        sensor_data["times2"].append(time)

def write_array_to_file(file_path, array):
    with open(file_path, 'a') as file:
        output_line = ' '.join(str(item) for item in array)
        file.write(output_line)
        file.write("\n")

def add_vehicles(plexe, n, real_engine=False):
    global vehicles
    vehicles = [Vehicles("v.%d"%i, (n - i + 1) * (DISTANCE + LENGTH) * 2, 0, velocity,plexe) for i in range(n)]
    for i in range(n):
        print((n - i + 1) * (DISTANCE + LENGTH) * 2)

def main(i=0):
    results = []
    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0

    #sensor_info = VehicleData(speed=None)
    while running(False, step, max_step=MAX_STEP):
        traci.simulationStep()
        if step == 0:
            trust_score = Reputation(0.5)
            add_vehicles(plexe, 2)
            traci.gui.trackVehicle("View #0", VERIFYING_VEHICLE)
            traci.gui.setZoom("View #0", 45000)
            traci.vehicle.setColor(CLAIMING_VEHICLE, (255,0,0)) 
            traci.vehicle.setColor(VERIFYING_VEHICLE, (255,255,255))
            vehicles[1].setModel(i)
        vehicles[1].setStep(step)

        behavior_interval = 200
        if (step > 0 and step < ATTACK_STEP):
            v2_data = vehicles[0].buildMessage()
            claim_lane = vehicles[0].getLane()
            vehicles[0].sendMessage(v2_data, vehicles[1], vehicles[0], claim_lane, trust_score.trust, step)
            # get info for graphing
            trust_score.trust = vehicles[1].getTrustScore()
            append_data(v2_data, i)
            # if (step % behavior_interval == 1 and step < SCENARIO_STEP):
            #     vehicles[0].setAcceleration(numpy.random.normal(0, 1.5))
            #     behavior_interval = int(numpy.random.normal(500, 100))
            #     if behavior_interval == 0:
            #         behavior_interval = 1

        if (step == ATTACK_STEP):
            vehicles[1].startTimer(step)
            vehicles[1].initialVelocity()

        if (step > ATTACK_STEP):
            v2_data = vehicles[0].buildMessage()
            claim_lane = vehicles[0].getLane()
            # attack.falseLaneAttack(plexe, CLAIMING_VEHICLE)
            # attack.falseBrake(plexe, v2_data, CLAIMING_VEHICLE)
            # attack.phantomBraking(plexe, v2_data, CLAIMING_VEHICLE)
            # attack.teleportationAttack(plexe, v2_data, CLAIMING_VEHICLE, VERIFYING_VEHICLE)
            vehicles[0].sendMessage(v2_data, vehicles[1], vehicles[0], claim_lane, trust_score.trust, step)
            trust_score.trust = vehicles[1].getTrustScore()
            append_data(v2_data, i)

        # end the simulation if a crash occurs
        if (plexe.get_crashed(CLAIMING_VEHICLE)):
            traci.vehicle.setSpeed(VERIFYING_VEHICLE, 0)
            traci.close()
            return
        step += 1
    
    print(f"model: {vehicles[1].getModel()}")
    print(f"max accel: {vehicles[1].getMaxAcc()}")
    #print(f"max distance: {vehicles[1].getMaxDistanceBetween()}")
    #print(f"min distance: {vehicles[1].getMinDistanceBetween()}")
    #print(f"time accel stabilize: {vehicles[1].getTimeTillStable()}")
    print(f"trust: {vehicles[1].getTrustScore()}")

    results.append(vehicles[1].getModel())
    results.append(vehicles[1].getMaxAcc())
    results.append(vehicles[1].getMaxDistanceBetween())
    results.append(vehicles[1].getMinDistanceBetween())
    # results.append(vehicles[1].getTimeTillStable())
    write_array_to_file(file_path, results)

    traci.close()

if __name__ == "__main__":
    for i in range(3):
        random.seed(RANDSEED)
        numpy.random.seed(RANDSEED)
        trust_data = {"times": [], "trust": []}
        data = {vid: {"times": [], "accelerations": [], "velocities": []} for vid in vehicle_ids}
        main(i)
        if i == 2:
            trustOverTimeGraph(i)
        # accelerationPlotGraph(i)
        with open(f'sensor_data.json', 'w') as f:
            json.dump(sensor_data, f)
        with open(f'vehicle_data.json', 'w') as f:
            json.dump(data, f)
    
    distanceComparisonGraph()