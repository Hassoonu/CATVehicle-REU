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
from vehicles import Vehicles

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

MAX_STEP = 2000
CLAIMING_VEHICLE = 'v.0'
VERIFYING_VEHICLE = 'v.1'
attack = Attacks()
ATTACK_STEP = 1500

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
trust_threshold = 0.4

# plotting information
vehicle_ids = [CLAIMING_VEHICLE, VERIFYING_VEHICLE]
data = {vid: {"times": [], "accelerations": [], "velocities": []} for vid in vehicle_ids}
sensor_data = {"times": [], "sensor_velocities": [], "filtered_velocities": [], "sensor_accelerations": [], \
               "filtered_accelerations": [], "message_speed": [], "message_acceleration": []}
trust_data = {"times": [], "trust": []}

def plot_data():
    fig, axs = plt.subplots(2, 3, figsize=(15, 8))

    # plot the acceleration data for each vehicle
    for vid in vehicle_ids:
        if (vid == CLAIMING_VEHICLE):
            name = "Claimer"
        else:
            name = "Verifier"
        axs[1, 1].plot(data[vid]["times"], data[vid]["accelerations"], label=f"{name} Acceleration")
    
    # axs[0, 0].plot(sensor_data["times"], sensor_data["sensor_accelerations"], label=f"Sensor Accl")

    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Acceleration (m/s^2)')
    axs[1, 1].set_title('Vehicle Acceleration')
    axs[1, 1].legend()  # add a legend to distinguish the different vehicles

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

    # plot the velocity sensor information
    axs[0, 2].plot(data[CLAIMING_VEHICLE]["times"], data[CLAIMING_VEHICLE]["velocities"], label=f"{name} Velocity")
    axs[0, 2].plot(sensor_data["times"], sensor_data["sensor_velocities"], label=f"Sensor Velocity")
    axs[0, 2].plot(sensor_data["times"], sensor_data["filtered_velocities"], label=f"Filtered Velocity")
    axs[0, 2].set_xlabel('Time (s)')
    axs[0, 2].set_ylabel('Speed (m/s)')
    axs[0, 2].set_title('Sensor Speed')
    axs[0, 2].legend()  # add a legend to distinguish the different vehicles

    # plot the trust data
    axs[1, 0].plot(trust_data["times"], trust_data["trust"], label=f"Trust Score")
    axs[1, 0].set_xlabel('Time')
    axs[1, 0].set_ylabel('Trust')
    axs[1, 0].set_title('Trust Score')

    # plot the message information
    axs[0, 0].plot(sensor_data["times"], sensor_data["message_speed"], label=f"Message Velocity")
    # axs[1, 1].plot(sensor_data["times"], sensor_data["message_acceleration"], label=f"Message Accel")
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Message Information')
    axs[0, 0].set_title('Messages')
    axs[0, 0].legend()  # add a legend to distinguish the different vehicles
    axs[0, 0].set_ylim(axs[0, 1].get_ylim())

    # plot the acceleration sensor information
    axs[1, 2].plot(data[CLAIMING_VEHICLE]["times"], data[CLAIMING_VEHICLE]["accelerations"], label=f"{name} Accel")
    axs[1, 2].plot(sensor_data["times"], sensor_data["sensor_accelerations"], label=f"Sensor Accel")
    axs[1, 2].plot(sensor_data["times"], sensor_data["filtered_accelerations"], label=f"Filtered Accel")
    axs[1, 2].set_xlabel('Time (s)')
    axs[1, 2].set_ylabel('Speed (m/s)')
    axs[1, 2].set_title('Sensor Acceleration')
    axs[1, 2].legend()  # add a legend to distinguish the different vehicles
    plt.tight_layout()  # adjust the subplot layout to make it more readable
    plt.show()

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

            estimatedVelocity = velocity; pred = 0; accel_est = 0; accel_pred = 0
        if step % SENSOR_REFRESH == 1:  # sensor refresh rate
            time = traci.simulation.getTime()

            # create the sensor readings
            if sensor_info.speed == None:
                prev_est = velocity
            else:
                prev_est = estimatedVelocity
            claim_speed_sensor = addNoise(traci.vehicle.getSpeed(CLAIMING_VEHICLE), 0.4)
            estimatedVelocity, pred = kalmanFilter(claim_speed_sensor, state_est=estimatedVelocity, prediction=pred)
            sensor_info = vehicles[0].buildMessage()
            v2_data = vehicles[0].buildMessage()

            # calculate acceleration
            accel = (estimatedVelocity - prev_est) / (SENSOR_REFRESH / 100)
            sensor_info.speed = estimatedVelocity

            accel_est, accel_pred = kalmanFilter(accel, R=1, state_est=accel_est, prediction=accel_pred)
            sensor_info.acceleration = accel_est

            # start attack
            if step > ATTACK_STEP:
                attack.falseBrake(plexe, false_message, CLAIMING_VEHICLE)

                false_vehicle_data = VehicleData()
                false_vehicle_data.pos_x = false_message.pos_x; false_vehicle_data.pos_y = false_message.pos_y; \
                false_vehicle_data.speed = false_message.speed; false_vehicle_data.acceleration = false_message.acceleration

                trust_score.UpdateTrustScore(sensor_info, false_vehicle_data, SENSOR_REFRESH / 100)
            else:
                # sensor info needs to be VehicleData object
                sensor_object = VehicleData(acceleration=sensor_info.acceleration, speed=sensor_info.speed, \
                                                pos_x=sensor_info.pos_x, pos_y=sensor_info.pos_y)
                claim_lane = vehicles[0].getLane()
                trust_score.UpdateTrustScore(sensor_info, v2_data, SENSOR_REFRESH / 100)

            # check if trust score is too low
            if trust_score.trust < trust_threshold:
                # ignore message, only use sensor information
                sensor_object = VehicleData(acceleration=sensor_info.acceleration, speed=sensor_info.speed, \
                                            pos_x=sensor_info.pos_x, pos_y=sensor_info.pos_y)
                des_acc = vehicles[1].getDesiredAcceleration(sensor_object, claim_lane)
            else:
                if step > ATTACK_STEP:
                    des_acc = vehicles[1].getDesiredAcceleration(false_vehicle_data, claim_lane)
                else:
                    des_acc = vehicles[1].getDesiredAcceleration(sensor_object, claim_lane)
            vehicles[1].setAcceleration(des_acc)

            # graphing info
            for vid in vehicle_ids:
                data[vid]["times"].append(time)
                data[vid]["accelerations"].append(traci.vehicle.getAcceleration(vid))
                data[vid]["velocities"].append(traci.vehicle.getSpeed(vid))
            trust_data["times"].append(time)
            trust_data["trust"].append(trust_score.trust)
            sensor_data["times"].append(time)
            sensor_data["sensor_velocities"].append(claim_speed_sensor)
            sensor_data["filtered_velocities"].append(sensor_info.speed)
            sensor_data["sensor_accelerations"].append(accel)
            sensor_data["filtered_accelerations"].append(sensor_info.acceleration)
            if step > ATTACK_STEP:
                sensor_data["message_acceleration"].append(false_vehicle_data.acceleration)
                sensor_data["message_speed"].append(false_vehicle_data.speed)
            else:
                sensor_data["message_acceleration"].append(vehicles[0].getAcceleration())
                sensor_data["message_speed"].append(traci.vehicle.getSpeed(CLAIMING_VEHICLE))

        if step == 200:
            vehicles[0].setAcceleration(-3)
        if step == 400:
            vehicles[0].setAcceleration(6)
        if step == 1000:
            vehicles[0].setAcceleration(-1)
        # if step == 1400:
        #     vehicles[0].setAcceleration(-6)
        # if step == 2000:
        #     vehicles[0].setAcceleration(6)
        if step == ATTACK_STEP:
            vehicles[0].setAcceleration(0)
            false_message = vehicles[0].buildMessage()
        step += 1

    traci.close()

if __name__ == "__main__":
    main()
    plot_data()

