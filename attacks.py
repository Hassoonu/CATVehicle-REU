#
# Copyright (c) 2018-2022 Michele Segata <segata@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

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
        Sends message of not braking while physically braking
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
            falseMessage.posX += falseMessage.speed * t + falseMessage.acceleration / 2 * t * t
        falseMessage.timestamp = newTime

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
            falseMessage.posX += falseMessage.speed * t + falseMessage.acceleration / 2 * t * t
        falseMessage.timestamp = newTime

    def teleportationAttack(self, plexe, falseMessage, claimerID):
        pass

    def falseLaneAttack(self, plexe, falseMessage, claimerID):
        pass

    def mergerAttack(self, plexe, falseMessage, claimerID):
        pass

    def rearAttack(self, plexe, falseMessage, claimerID):

        '''
        Sends message of not accelerating while physically accelerating
        # claimerID: attacker vehicle
        '''
        newTime = plexe.get_vehicle_data(claimerID).__getitem__(TIME)
        t = (newTime - falseMessage.timestamp)
        falseMessage.acceleration = 0
        if falseMessage.speed <= 0: #check if this conditional is needed.
            falseMessage.speed = 60
        else:
            falseMessage.speed += falseMessage.acceleration * t
            falseMessage.posX += falseMessage.speed * t + falseMessage.acceleration / 2 * t * t
        falseMessage.timestamp = newTime