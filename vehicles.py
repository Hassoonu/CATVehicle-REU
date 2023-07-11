import traci
from vehicle_message import VehicleMessage
from plexe import ACC, SPEED, ACCELERATION, TIME
from utils import *
from Sensors import addNoise, kalmanFilter

ACC_HEADWAY=1.5
LENGTH = 4
velocity = 30


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
        lane = traci.vehicle.getLaneID(self.ID)[-1]
        return lane

    def copyDataFromMessage(self, vehicleObject, message):
        vehicleObject.pos_x = message.posX
        vehicleObject.pos_y = message.posY
        vehicleObject.speed = message.speed
        vehicleObject.acceleration = message.acceleration

    def getDesiredAcceleration(self, vehicle2Data, vehicle2Lane):
        
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

        cruisingVelocity = 30
        td = 1.2; s0 = 3; v0 = cruisingVelocity; Q = 1; P = 100; K1 = 0.18; K2 = 1.93     # params
        d1 = self.plexe.get_vehicle_data(self.ID); # get vehicle info
        
        if (self.getLane() == vehicle2Lane):
            s = vehicle2Data.__getitem__(POS_X) - d1.__getitem__(POS_X) - LENGTH # calculate space gap
            vn = d1.__getitem__(SPEED); vn2 = vehicle2Data.__getitem__(SPEED) # vehicle speeds
        else:
            s = 100 # calculate space gap
            vn = d1.__getitem__(SPEED); vn2 = velocity # vehicle speeds      
        del_s = min(s - s0 - vn * td, (v0 - vn) * td)   # calculate spacing error
        R_s = 1 - (1 / (1 + Q * math.pow(math.e, -1 * (s / P))))    # calculate error response for collision avoidance

        des_acc = K1 * del_s + K2 * (vn2 - vn) * R_s    # finally, calculate desired acceleration
        # print(f"Desired accel: {des_acc}")
        return des_acc

    def getSensorData(self, ):
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
    
    def getID(self):
        return self.ID


