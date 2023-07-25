import traci
from vehicle_message import VehicleMessage
from reputation import Reputation
from plexe import ACC, SPEED, ACCELERATION, TIME
from utils import *
from Sensors import addNoise, kalmanFilter
from vehicle_data import VehicleData
import math

ACC_HEADWAY=1.5
LENGTH = 4
velocity = 30
GracefulDeceleration = 6
maxDeceleration = 10
SENSOR_REFRESH = 10


class modelTesting:

    def __init__(self, vehicleID, position, lane, speed, plexe):
        self.plexe = plexe
        add_vehicle(plexe, vehicleID, position, lane, speed)
        self.ID = vehicleID
        self.LANE = lane
        self.myAcceleration = 0
        self.plexe.set_fixed_lane(vehicleID, 0, safe=False)
        traci.vehicle.setSpeedMode(vehicleID, 0)
        if (vehicleID == 'v.0'):
            self.plexe.set_active_controller(vehicleID, ACC)
            self.plexe.set_acc_headway_time(vehicleID, 0.1)
            self.plexe.set_cc_desired_speed(vehicleID, 30)
            self.plexe.use_controller_acceleration(vehicleID, False)
        # else:
        #     self.plexe.set_active_controller(vehicleID, ACC)
        #     self.plexe.set_acc_headway_time(vehicleID, 0.1)
        #     self.plexe.use_controller_acceleration(vehicleID, False)
        self.gotAMessage = False
        self.sensorObject = VehicleData(speed=None)
        self.estimatedVelocity = velocity
        self.pred = 0
        self.accel_est = 0
        self.accel_pred = 0
        self.claim_speed_sensor = 0
        self.timeSinceLastMessage = 0
        self.MessageTime = 0
        self.trust = 0.5
        self.trustPenalty = 0
        self.trustworthy = True
        self.timeDelay = 0
        self.desTimeDelay = 0
        #for testing
        self.model = 1
        self.beginningStep = 0
        self.step = 0
        self.maxDesiredAccelerationOutput = 0
        self.currentMaxDistanceGap = 0
        self.currentMinDistanceGap = None
        self.beginningVelocity = 0
        self.stopGettingData = False
        self.previousVelocities = []




    def setAcceleration(self, acceleration):
        if (self.ID == 'v.0'):
            self.plexe.set_fixed_acceleration(self.ID, True, acceleration)
        else:
            traci.vehicle.setAcceleration(self.ID, acceleration, 1)
        self.myAcceleration = traci.vehicle.getAcceleration(self.ID)

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
        minTD = 0.7 # min time delay in seconds
        maxTD = 3 # seconds
        a = -4
        x1 = 0.81
        match self.model:
            case 0:
                #linear model
                td = (minTD-maxTD) * trustScore + maxTD
            case 1:
                #Exponential Model 1
                a = 0.00001
                td = ((maxTD-minTD)*a**(-trustScore + 1) + a*minTD - maxTD) / (a - 1)
            case 2:
                #Exponential Model 2
                a = 5.2
                td = ((maxTD-minTD)*a**(-trustScore + 1) + a*minTD - maxTD) / (a - 1)
            case 3:
                #Cubic Model 1
                a = 3.3
                b = 0.5
                td = a*(trustScore**3) + (-3*a*b)*(trustScore**2) + (minTD-maxTD-a+3*a*b)*trustScore + maxTD
            case 4:
                #Cubic Model 2
                a = -6.8
                b = 0.418
                td = a*(trustScore**3) + (-3*a*b)*(trustScore**2) + (minTD-maxTD-a+3*a*b)*trustScore + maxTD
            case 5:
                #Step Model
                a = 0.65
                b = 0
                if (trustScore < a):
                    b = 0
                    td = maxTD - (maxTD - minTD)*b
                elif(trustScore >= a):
                    b = 1
                    td = maxTD - (maxTD - minTD)*b
            case _:
                td = a*math.pow(trustScore, 3) - 3*a*x1*math.pow(trustScore, 2) + (minTD - maxTD - a + 3*a*x1) * trustScore + maxTD

        s0 = 3.5
        desiredVelocity = 60
        Q = 5
        P = 100
        K1 = 0.18 #0.18
        K2 = 1.93 #1.93    # params
        d1 = self.plexe.get_vehicle_data(self.ID) # get vehicle info
        if (self.getLane() == vehicle2Lane):
            s = vehicle2Data.__getitem__("pos_x") - d1.__getitem__(POS_X) - LENGTH # calculate space gap
            vn = d1.__getitem__(SPEED); vn2 = vehicle2Data.__getitem__(SPEED) # vehicle speeds
        else:
            s = 100 # calculate space gap
            vn = d1.__getitem__(SPEED); vn2 = velocity # vehicle speeds    

        del_s = min(s - s0 - vn *td, (desiredVelocity - vn) * td)   # calculate spacing error
        #del_s = s - s0 - vn * td
        R_s = 1 - (1 / (1 + Q * math.pow(math.e, -1 * (s / P))))    # calculate error response for collision avoidance
        des_acc = K1 * del_s + K2 * (vn2 - vn) * R_s    # finally, calculate desired acceleration
        #print(f"Desired accel: {des_acc}")
        s = traci.vehicle.getPosition('v.0')[0] - traci.vehicle.getPosition('v.1')[0] - LENGTH
        #print(f"del_s: {del_s}\nvehicle 0 pos: {traci.vehicle.getPosition('v.0')[0]}\nvehicle 1 pos: {traci.vehicle.getPosition('v.1')[0]}")
        actualDelay = s / traci.vehicle.getSpeed('v.1')
        #print(f"vehicle speed: {traci.vehicle.getSpeed('v.1')}\nposition: {s}\nactual delay: {actualDelay}")
        self.timeDelay = actualDelay

        #print(f"Desired delay: {td:.3f}, Actual delay: {actualDelay:.3f}, Desired accel: {des_acc:.3f}, Trust score: {trustScore:.3f}, flag: {self.stopGettingData}, step: {self.getStep()}, initial velocity: {self.beginningVelocity}, current velocity: {traci.vehicle.getSpeed('v.1')}")
        #print(f"Model being tested is: {self.getModel()}")
        self.desTimeDelay = td
        if(abs(des_acc) > abs(self.maxDesiredAccelerationOutput)):
            self.maxDesiredAccelerationOutput = des_acc
        return des_acc

    def updateSensorData(self, targetVehicleObject):
        '''
        Moving average for data to prevent large amounts of noise in acceleration
        Trust penalty response depends on following time, smaller time delay = larger penalty 
        '''
        message = self.getTrueMessage(targetVehicleObject)
        if self.sensorObject.speed == None:
                self.prev_est = velocity
        else:
            self.prev_est = self.estimatedVelocity

        self.claim_speed_sensor = addNoise(traci.vehicle.getSpeed(targetVehicleObject.getID()))
        
        self.sensorObject = message
        
        Q = 1; R = 0.001
        self.estimatedVelocity, self.pred = kalmanFilter(self.claim_speed_sensor, state_est=self.estimatedVelocity, prediction=self.pred, Q=Q, R=R)
        
        self.accel = (self.estimatedVelocity - self.prev_est) / (SENSOR_REFRESH / 100)
        
        # self.sensorObject.speed = self.estimatedVelocity
        self.sensorObject.speed = self.claim_speed_sensor
        Q = 1; R = 1.14 * Q
        self.accel_est, self.accel_pred = kalmanFilter(self.accel, R=R, Q=Q, state_est=self.accel_est, prediction=self.accel_pred)
        
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
        targetOfMessage.recieveMessage(creatorOfMessage, message, vehicleLane, trust, step)

    def recieveMessage(self, sender, message, vehicleLane, trust, step):
        self.timeSinceLastMessage = step - self.MessageTime
        self.MessageTime = step
        if(self.canUpdateSensor(step)):
            self.updateSensorData(sender)
            self.trustworthy = self.verifyMessageIntegrity(message, self.timeSinceLastMessage, sender, vehicleLane)
        if(self.trustworthy):
            des_acc = self.getDesiredAcceleration(message, vehicleLane, trust)
        else:   # use sensor information instead
            vehicleLane = self.getTrueLane(sender)
            des_acc = self.getDesiredAcceleration(self.sensorObject, vehicleLane, trust)
        self.setAcceleration(des_acc)
        vehicleLane = self.getTrueLane(sender)
        desiredAcceleration = self.getDesiredAcceleration(self.sensorObject, vehicleLane, trust)
        self.collectData(sender, desiredAcceleration, step)

    def collectData(self, sender, desiredAcceleration, step):
        myData = self.buildMessage()
        self.previousVelocities.append(myData.speed)
        print(f"Desired accel: {desiredAcceleration:.3f}, Trust score: {self.trust:.3f}, step: {self.getStep()}, beginningStep: {self.beginningStep}")
        if(self.trust == 0 and (abs(desiredAcceleration) < 1) and step - self.beginningStep > 50):
            step = self.getStep()
            self.endTimer(step, self.stopGettingData)
            self.stopGettingData = True
            #print(f"Desired accel: {desiredAcceleration:.3f}, Trust score: {self.trust:.3f}, flag: {self.stopGettingData}, step: {self.getStep()}, initial velocity: {self.beginningVelocity}, current velocity: {traci.vehicle.getSpeed('v.1')}")
            #pause = input("press key when ready.")
        targetInfo = self.getTrueMessage(sender)
        myInfo = self.plexe.get_vehicle_data(self.ID)
        distanceBetween = targetInfo.__getitem__("pos_x") - myInfo.__getitem__(POS_X) - LENGTH
        self.checkMaxDistanceBetween(distanceBetween)
        self.checkMinDistanceBetween(distanceBetween)


# get rid of vehicle lane parameter in most code and replace with "getTrueLane()"
    def canUpdateSensor(self, step):
        return True if(step % SENSOR_REFRESH == 1) else False 

    def verifyMessageIntegrity(self, message, time_interval, sender, claimedLane):
        acc_std = 0.412
        vel_std = 0.03
        vel_threshold = 3 * vel_std
        acc_threshold = 2 * acc_std
        suspicious = False
        deviation = 0
        if abs(self.sensorObject.acceleration - message.acceleration) > acc_threshold:
            suspicious = True
            deviation += abs(self.sensorObject.acceleration - message.acceleration) - acc_threshold
        if abs(self.sensorObject.speed - message.speed) > vel_threshold:
            suspicious = True
            deviation += abs(self.sensorObject.speed - message.speed) - vel_threshold
        if(claimedLane != self.getTrueLane(sender)):
            suspicious = True
            deviation += 10 #just a number right now, will update later
        
        self.decay(time_interval)
        self.updateTrustScore(suspicious, deviation)

        return False if (self.trust < 0.5) else True
    
    def decay(self, interval):
        '''
        Adds exponential time decay based on the length of the intervals between messages
        '''
        decay_rate = .0011
        self.trust *= math.exp(-decay_rate * interval)
        return
    
    def getTrueMessage(self, sender):
        return sender.buildMessage()
    
    def getTrueLane(self, sender):
        return sender.getLane()
    
    def updateTrustScore(self, suspicious, deviation):
        if (self.trust == 0):
            pass
        elif suspicious: # decrease score
            distanceMultiplier = 100 * math.exp(1.0/self.getTimeDelay()) #PRONE TO CHANGE
            self.trust -= (deviation / 100) * math.pow(math.e, self.trustPenalty) * distanceMultiplier
            self.trustPenalty += 0.2
        else:   # increase score
            increase_factor = .1; inc = 0.011
            self.trustPenalty = self.trustPenalty/2
            self.trust += increase_factor * math.log(1 + inc)
        # boundary conditions
        if self.trust > 0.95:
            self.trust = 0.95
        elif self.trust < 0:
            self.trust = 0
        return
    
    def getTrustScore(self):
        return self.trust
    
    def getTimeDelay(self):
        return self.timeDelay
    
    def getdesTimeDelay(self):
        return self.desTimeDelay
    



    #FOR TESTING

    def getModel(self):
        return self.model

    def getMaxAcc(self):
        return self.maxDesiredAccelerationOutput
    
    def checkMaxDistanceBetween(self, distanceBetween):
        if(distanceBetween > self.currentMaxDistanceGap):
            self.currentMaxDistanceGap = distanceBetween

    def checkMinDistanceBetween(self, distanceBetween):
        if(self.currentMinDistanceGap == None):
            self.currentMinDistanceGap = distanceBetween
        if(distanceBetween < self.currentMinDistanceGap):
            self.currentMinDistanceGap = distanceBetween

    def getMaxDistanceBetween(self):
        return self.currentMaxDistanceGap
    
    def getMinDistanceBetween(self):
        return self.currentMinDistanceGap
        
    def initialVelocity(self):
        myInfo = self.buildMessage()
        velocityInitial = myInfo.speed
        self.beginningVelocity = velocityInitial

    def endTimer(self, step, flag = False):
        if(flag == False):
            self.endStep = step
        else:
            pass

    def startTimer(self, step):
        self.beginningStep = step

    def getTimeTillStable(self):
        return((self.endStep - self.beginningStep) / 100)
    
    def setStep(self, step):
        self.step = step

    def getStep(self):
        return self.step