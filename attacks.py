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



class Attacks:
    """
    Class to store data of a vehicle. Fields can either be accessed directly
    (e.g., data.acceleration) or using a dictionary-like access (e.g.,
    data[ACCELERATION]) depending on convenience
    """
    def __init__(self, index=None, u=None, acceleration=None, speed=None,
                 pos_x=None, pos_y=None, time=None, length=None):
        self.index = index
        self.u = u
        self.acceleration = acceleration
        self.speed = speed
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.time = time
        self.length = length

    def __init__(self, plexe):
        self.plexe = plexe

    def falseBrake(self, plexe, falseMessage, claimerID):
        '''
        Sends message of braking while not physically braking
        # claimerID: attacker vehicle
        '''
        plexe = Plexe()
        newTime = plexe.get_vehicle_data(claimerID).__getitem__(TIME)
        t = (newTime - falseMessage.timestamp)
        falseMessage.acceleration = -6
        if falseMessage.speed <= 0:
            falseMessage.speed = 0
        else:
            falseMessage.speed += falseMessage.acceleration * t
            falseMessage.pos_x += falseMessage.speed * t + falseMessage.acceleration / 2 * t * t
        falseMessage.timestamp = newTime
        plexe.set_fixed_acceleration(claimerID, True, 0)


    def notBraking(self, plexe, falseMessage, claimerID):
        '''
        Sends message of not braking while physically braking
        # claimerID: attacker vehicle
        '''
        newTime = plexe.get_vehicle_data(claimerID).__getitem__(TIME)
        t = (newTime - falseMessage.timestamp)
        falseMessage.acceleration = 6
        if falseMessage.speed <= 0:
            falseMessage.speed = 60
        else:
            falseMessage.speed += falseMessage.acceleration * t
            falseMessage.pos_x += falseMessage.speed * t + falseMessage.acceleration / 2 * t * t
        falseMessage.timestamp = newTime
        plexe.set_fixed_acceleration(claimerID, True, -6)


    def teleportationAttack(self, plexe, falseMessage, claimerID, targetID):
        newTime = plexe.get_vehicle_data(claimerID).__getitem__(TIME)
        t = (newTime - falseMessage.timestamp)
        falsePositionX = plexe.get_vehicle_data(targetID).__getitem__(POS_X) + 4
        falsePositionY = plexe.get_vehicle_data(targetID).__getitem__(POS_Y)
        falseMessage.pos_x = falsePositionX
        falseMessage.pos_y = falsePositionY
        falseMessage.speed  = plexe.get_vehicle_data(targetID).__getitem__(SPEED)
        falseMessage.timestamp = newTime

    def falseLaneAttack(self, plexe, claimerID, targetID):
        falseLane = int(traci.vehicle.getLaneID(claimerID)[-1]) + 1
        falseLane = str(falseLane)
        return falseLane


        

    def mergerAttack(self, plexe, falseMessage, claimerID, targetID):
        claimerX = plexe.get_vehicle_data(claimerID).__getitem__(POS_X)
        targetX = plexe.get_vehicle_data(targetID).__getitem__(POS_X)
        distance = claimerX - targetX
        distanceThreshold = 3
        targetSpeed = plexe.get_vehicle_data(targetID).__getitem__(SPEED)
        #Declare Variables
        
        td = 0; s0 = 0; v0 = targetSpeed; Q = 1; P = 100; K1 = 0.18 ; K2 = 1.93     # params k1 = 0.18, k2 = 1.93
        d1 = plexe.get_vehicle_data(claimerID); # get vehicle info
        s = distance # calculate space gap
        vn = d1.__getitem__(SPEED); vn2 = targetSpeed # vehicle speeds      
        del_s = (s0 - s - vn * td)   # calculate spacing error
        R_s = 1 - (1 / (1 + Q * math.pow(math.e, -1 * (s / P))))    # calculate error response for collision avoidance
        des_acc = K1 * del_s + K2 * (vn2 - vn) * R_s
        #Find desired acc.
        
        plexe.set_fixed_acceleration(claimerID, True, des_acc)
        targetLane = traci.vehicle.getLaneID(targetID)[-1]
        targetLane = int(targetLane)
        plexe.set_fixed_lane(claimerID, 1, False)
        #change accel. and lane

        claimerSpeed = plexe.get_vehicle_data(claimerID).__getitem__(SPEED)
        targetSpeed = plexe.get_vehicle_data(targetID).__getitem__(SPEED)
        speedDiff = claimerSpeed - targetSpeed
        speedThreshold = 1
        #merge attack variable declarations

        if( (distance < distanceThreshold) and ( speedDiff < speedThreshold)):
            plexe.set_fixed_lane(claimerID, 0, False) #NOT FINISHED, LINE PRODUCES ERROR + NOT MERGING?!?!?!?
            print("Im working lmao")
        #Meant to merge lanes and cause crash

        
    def rearAttack(self, plexe, falseMessage, claimerID, targetID):

        '''
        Sends message of not accelerating while physically accelerating
        # claimerID: attacker vehicle
        '''

        claimerX = plexe.get_vehicle_data(claimerID).__getitem__(POS_X)
        targetX = plexe.get_vehicle_data(targetID).__getitem__(POS_X)

        plexe.set_fixed_lane(claimerID, 1, False)

        targetSpeed = plexe.get_vehicle_data(targetID).__getitem__(SPEED)
        plexe.set_fixed_acceleration(claimerID, True, -2)
        targetLane = traci.vehicle.getLaneID(targetID)[-1]
        targetLane = int(targetLane)
        

        if((claimerX + 5) < targetX ):
            plexe.set_fixed_lane(claimerID, targetLane, False)
            newTime = plexe.get_vehicle_data(claimerID).__getitem__(TIME)
            t = (newTime - falseMessage.timestamp)
            falseMessage.acceleration = 0
            if falseMessage.speed <= 0: #check if this conditional is needed.
                falseMessage.speed = targetSpeed
            else:
                falseMessage.speed += falseMessage.acceleration * t
                falseMessage.pos_x += falseMessage.speed * t + falseMessage.acceleration / 2 * t * t
            falseMessage.timestamp = newTime
            plexe.set_fixed_acceleration(claimerID, True, 10)
            print(plexe.get_vehicle_data(claimerID).__getitem__(ACCELERATION))

        