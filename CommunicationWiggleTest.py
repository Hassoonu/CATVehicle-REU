import traci
from vehicle_message import VehicleMessage
from plexe import ACC, SPEED, ACCELERATION, TIME
from utils import *

ACC_HEADWAY=1.5
LENGTH = 4
velocity = 30


class CommunicationWiggleTest:

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
        lane = traci.vehicle.getLaneID(self.ID)[-1]
        return lane

    def copyDataFromMessage(self, vehicleObject, message):
        vehicleObject.pos_x = message.posX
        vehicleObject.pos_y = message.posY
        vehicleObject.speed = message.speed
        vehicleObject.acceleration = message.acceleration

    def getDesiredAcceleration(self, vehicle1, vehicle2Data, vehicle2Lane):
        
        timeConstant = 0.5
        changeInStep = 0.01
        desiredAcceleration = 0
        velocity = 0
        distanceBetweenCars = 20
        T= distanceBetweenCars/vehicle2Data.speed

        desiredAcceleration = -(1/T) * (velocityDiff + 

        return desiredAcceleration

    def getSensorD(self, ):
        pass

    def buildMessage(self):
        vd = self.plexe.get_vehicle_data(self.ID)
        posX = vd.__getitem__(POS_X)
        posY = vd.__getitem__(POS_Y)
        acceleration = vd.__getitem__(ACCELERATION)
        speed = vd.__getitem__(SPEED)
        timestamp = vd.__getitem__(TIME)
        newMessage = VehicleMessage(posX, posY, speed, acceleration, timestamp)
        return newMessage


