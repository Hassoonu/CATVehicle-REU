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

ACC_HEADWAY=1.5
class Vehicles:

    def __init__(self, vehicleID, position, lane, speed, plexe):
        self.plexe = plexe
        add_vehicle(plexe, vehicleID, position, lane, speed)
        self.ID = vehicleID
        self.LANE = lane
        self.plexe.set_fixed_lane(vehicleID, 0, safe=False)
        traci.vehicle.setSpeedMode(vehicleID, 0)
        self.plexe.use_controller_acceleration(vehicleID, False)
        self.plexe.set_active_controller(vehicleID, ACC)
        self.plexe.set_acc_headway_time(vehicleID, ACC_HEADWAY)
        self.plexe.set_cc_desired_speed(vehicleID, 20)


    def getAcceleration(self):
        pass

    def setAcceleration(self, acceleration):
        self.plexe.set_fixed_acceleration(self.ID, True, acceleration)
        

    def getLane(self):
        lane = traci.vehicle.getLaneID(self.id)[-1]
        return lane

    def buildFalseVehicle(self, falseVehicleObject, falseMessage):
        falseVehicleObject.pos_x = falseMessage.posX
        falseVehicleObject.pos_y = falseMessage.posY
        falseVehicleObject.speed = falseMessage.speed
        falseVehicleObject.acceleration = falseMessage.acceleration

    def getDesiredAcceleration(self, vehicle1, vehicle2Data, vehicle2Lane):
        cruisingVelocity = 30
        td = 1.2; s0 = 3; v0 = cruisingVelocity; Q = 1; P = 100; K1 = 0.18; K2 = 1.93     # params
        d1 = self.plexe.get_vehicle_data(vehicle1); # get vehicle info
        
        if (vehicle1.getLane() == v2_lane):
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

    def getVehicleData(self, ):
        pass

    def buildMessage(self, vehicleID):
        vd = self.plexe.get_vehicle_data(vehicleID)
        posX = vd.__getitem__(POS_X)
        posY = vd.__getitem__(POS_Y)
        acceleration = vd.__getitem__(ACCELERATION)
        speed = vd.__getitem__(SPEED)
        timestamp = vd.__getitem__(TIME)
        newMessage = VehicleMessage(posX, posY, speed, acceleration, timestamp)
        return newMessage


