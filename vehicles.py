import traci
from vehicle_message import VehicleMessage
from plexe import ACC, SPEED, ACCELERATION, TIME
from utils import *
from Sensors import addNoise, kalmanFilter
from vehicle_data import VehicleData
import math

ACC_HEADWAY=1.5
LENGTH = 4
velocity = 30
GracefulDeceleration = 7
maxDeceleration = 10
SENSOR_REFRESH = 10


class Vehicles:


    def __init__(self, vehicleID, position, lane, speed, plexe):
        self.plexe = plexe
        add_vehicle(plexe, vehicleID, position, lane, speed)
        self.ID = vehicleID
        self.LANE = lane
        self.myAcceleration = 0
        self.plexe.set_fixed_lane(vehicleID, 0, safe=False)
        traci.vehicle.setSpeedMode(vehicleID, 0)
        self.plexe.use_controller_acceleration(vehicleID, False)
        self.plexe.set_active_controller(vehicleID, ACC)
        self.plexe.set_acc_headway_time(vehicleID, ACC_HEADWAY)
        self.plexe.set_cc_desired_speed(vehicleID, 20)
        self.gotAMessage = False
        self.sensorObject = VehicleData(speed=None)
        self.estimatedVelocity = velocity
        self.pred = 0
        self.accel_est = 0
        self.accel_pred = 0
        self.claim_speed_sensor = 0
        self.accel_est = 0
        self.accel_pred = 0
        self.timeSinceLastMessage = 0
        self.MessageTime = 0




    def setAcceleration(self, acceleration):
        self.plexe.set_fixed_acceleration(self.ID, True, acceleration)
        self.myAcceleration = acceleration




    def getAcceleration(self):
        return self.myAcceleration
        




    def getLane(self):
        lane = traci.vehicle.getLaneID(self.ID)[-1]
        return lane




    def copyDataFromMessage(self, vehicleObject, message):
        vehicleObject.pos_x = message.pos_x
        vehicleObject.pos_y = message.pos_y
        vehicleObject.speed = message.speed
        vehicleObject.acceleration = message.acceleration




    def getDesiredAcceleration(self, vehicle2Data, vehicle2Lane, trustScore):
        
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
        cruisingVelocity = 100
        timeDelayN = 1.8
        
        td = -timeDelayN*trustScore + 3 #use case switch
        s0 = 3
        v0 = cruisingVelocity
        Q = 1
        P = 100
        K1 = 0.18
        K2 = 1.93     # params
        d1 = self.plexe.get_vehicle_data(self.ID) # get vehicle info
        
        if (self.getLane() == vehicle2Lane):
            s = vehicle2Data.__getitem__(POS_X) - d1.__getitem__(POS_X) - LENGTH # calculate space gap
            vn = d1.__getitem__(SPEED); vn2 = vehicle2Data.__getitem__(SPEED) # vehicle speeds
        else:
            s = 100 # calculate space gap
            vn = d1.__getitem__(SPEED); vn2 = velocity # vehicle speeds      
        del_s = min(s - s0 - vn * td, (v0 - vn) * td)   # calculate spacing error
        R_s = 1 - (1 / (1 + Q * math.pow(math.e, -1 * (s / P))))    # calculate error response for collision avoidance

        des_acc = K1 * del_s + K2 * (vn2 - vn) * R_s    # finally, calculate desired acceleration
        #print(f"Desired accel: {des_acc}")
        return des_acc




    def updateSensorData(self, targetVehicleObject):
        message = self.getTrueMessage(targetVehicleObject)
        if self.sensorObject.speed == None:
                self.prev_est = velocity
        else:
            self.prev_est = self.estimatedVelocity

        self.claim_speed_sensor = addNoise(traci.vehicle.getSpeed(targetVehicleObject.getID()), 0.4)
        
        self.sensorObject = message
        
        self.estimatedVelocity, self.pred = kalmanFilter(self.claim_speed_sensor, state_est=self.estimatedVelocity, prediction=self.pred)
        
        self.accel = (self.estimatedVelocity - self.prev_est) / (SENSOR_REFRESH / 100)
        
        self.sensorObject.speed = self.estimatedVelocity
        
        self.accel_est, self.accel_pred = kalmanFilter(self.accel, R=1, state_est=self.accel_est, prediction=self.accel_pred)
        
        self.sensorObject.acceleration = self.accel_est




    def getSensorData(self):
        
        accelerationFromSensor=self.sensorObject.acceleration 
        
        sensorSpeed=self.sensorObject.speed 
        
        x_position=self.sensorObject.pos_x 
        
        y_position=self.sensorObject.pos_y
        
        sensorData = VehicleData(acceleration=accelerationFromSensor, speed=sensorSpeed, pos_x=x_position, pos_y=y_position)
        
        return sensorData




    def getSensorClaimedSpeed(self):
        
        return self.claim_speed_sensor




    def getAccelFromSensor(self):
        
        return self.accel




    def buildMessage(self):
        
        vd = self.plexe.get_vehicle_data(self.ID)
        
        posX = vd.__getitem__(POS_X)
        
        posY = vd.__getitem__(POS_Y)
        
        acceleration = vd.__getitem__(ACCELERATION)
        
        speed = vd.__getitem__(SPEED)
        
        timestamp = vd.__getitem__(TIME)
        
        newMessage = VehicleMessage(posX, posY, speed, acceleration, timestamp)
        
        return newMessage

    def getID(self):
        
        return self.ID

    def sendMessage(self, message, targetOfMessage, creatorOfMessage, vehicleLane, trust, step):
            #change this to actually send a message
            targetOfMessage.recieveMessage(creatorOfMessage, message, vehicleLane, trust, step)

    def recieveMessage(self, sender, message, vehicleLane, trust, step):

        self.timeSinceLastMessage = step - self.MessageTime
        self.MessageTime = step
        self.getDesiredAcceleration(message, vehicleLane, trust)

        if(self.canUpdateSensor(step)):
            self.updateSensorData(sender)
            trustworthy = self.verifyMessageIntegrity(message, self.timeSinceLastMessage)
            if(trustworthy):
                pass
            else:
                pass
    
    def canUpdateSensor(self, step):
        return True if(step % SENSOR_REFRESH == 1) else False
        

    def verifyMessageIntegrity(self, message, time_interval):
        threshold = 1
        suspicious = False
        deviation = 0
        if abs(self.sensorObject.acceleration - message.acceleration) > 2:
            suspicious = True
            deviation += abs(self.sensorObject.acceleration - message.acceleration) - 2
        if abs(self.sensorObject.speed - message.speed) > threshold:
            suspicious = True
            deviation += abs(self.sensorObject.speed - message.speed) - threshold
        
        self.decay(time_interval)

        self.updateTrustScore(suspicious, deviation)
    
    def decay(self, interval):
        '''
        Adds exponential time decay based on the length of the intervals between messages
        '''
        decay_rate = .001
        self.trust *= math.exp(-decay_rate * interval)
        return
    
    def getTrueMessage(self, sender):
        return sender.buildMessage()
    
    def increase(self, inc):
        '''
        Increases the trust score based on a factor
        '''
        increase_factor = .1
        self.trust += increase_factor * math.log(1 + inc)
        return
    
    def updateTrustScore(self, suspicious, deviation):
        if suspicious:
            self.trust -= deviation / 100
        else:
            self.increase(.01)
        # boundary conditions
        if self.trust > 1:
            self.trust = 1
        elif self.trust < 0:
            self.trust = 0
        return