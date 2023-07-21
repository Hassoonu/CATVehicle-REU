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
        falseMessage.acceleration = 0
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

    def falseLaneAttack(self, plexe, claimerID):
        falseLane = int(traci.vehicle.getLaneID(claimerID)[-1]) + 1
        falseLane = str(falseLane)
        return falseLane


    def falsifyPosition(self, plexe, falseMessage, claimerID, targetID, switchCase, positionDisturbance):
        match switchCase:
            case 0:
                newTime = plexe.get_vehicle_data(claimerID).__getitem__(TIME)
                t = (newTime - falseMessage.timestamp)
                falsePositionX = plexe.get_vehicle_data(targetID).__getitem__(POS_X) + positionDisturbance
                falsePositionY = plexe.get_vehicle_data(targetID).__getitem__(POS_Y)
                falseMessage.pos_x = falsePositionX
                falseMessage.pos_y = falsePositionY
                falseMessage.speed  = plexe.get_vehicle_data(targetID).__getitem__(SPEED)
                falseMessage.timestamp = newTime
                return
            
            case 1:
                falseLane = int(traci.vehicle.getLaneID(claimerID)[-1]) + positionDisturbance
                falseLane = str(falseLane)
                return falseLane
            
            case _:
                return

    def falsifyVelocity(self, falseMessage, switchCase, velocityFalse):
        match switchCase:
            case 0:
                falseMessage.speed = velocityFalse
                return

    def falsifyAcceleration(self, plexe, falseMessage, claimerID, switchCase, accelerationReal, accelerationFalse):
        match switchCase:
            case 0:
                newTime = plexe.get_vehicle_data(claimerID).__getitem__(TIME)
                t = (newTime - falseMessage.timestamp)
                falseMessage.acceleration = accelerationFalse
                if falseMessage.speed <= 0:
                    falseMessage.speed = 60
                else:
                    falseMessage.speed += falseMessage.acceleration * t
                    falseMessage.pos_x += falseMessage.speed * t + falseMessage.acceleration / 2 * t * t
                falseMessage.timestamp = newTime
                plexe.set_fixed_acceleration(claimerID, True, accelerationReal)
                return
            
            case 1:
                newTime = plexe.get_vehicle_data(claimerID).__getitem__(TIME)
                t = (newTime - falseMessage.timestamp)
                falseMessage.acceleration = accelerationFalse
                if falseMessage.speed <= 0:
                    falseMessage.speed = 0
                else:
                    falseMessage.speed += falseMessage.acceleration * t
                    falseMessage.pos_x += falseMessage.speed * t + falseMessage.acceleration / 2 * t * t
                falseMessage.timestamp = newTime
                plexe.set_fixed_acceleration(claimerID, True, accelerationReal)
                return
            
            case _:
                return

        