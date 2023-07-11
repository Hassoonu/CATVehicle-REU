import math

class Reputation:
    def __init__(self, trust = 0.5):
        self.trust = trust
        return
    def isTrustworthyMessage():
        '''
        Returns whether the message is trustworthy
        '''
        return True
    def decay(self, interval):
        '''
        Adds exponential time decay based on the length of the intervals between messages
        '''
        decay_rate = .001
        self.trust *= math.exp(-decay_rate * interval)
        return
    def increase(self, inc):
        '''
        Increases the trust score based on a factor
        '''
        increase_factor = .1
        self.trust += increase_factor * math.log(1 + inc)
        return
    def UpdateTrustScore(self, sensorData, message, time_interval):
        threshold = 1
        suspicious = False
        deviation = 0
        if abs(sensorData.acceleration - message.acceleration) > 2:
            suspicious = True
            deviation += abs(sensorData.acceleration - message.acceleration) - 2
        if abs(sensorData.speed - message.speed) > threshold:
            suspicious = True
            deviation += abs(sensorData.speed - message.speed) - threshold
        # if abs(sensorData.pos_y - message.pos_y) > threshold:
        #     suspicious = True
        #     deviation += abs(sensorData.pos_y - message.pos_y) - threshold
        # if abs(sensorData.pos_x - message.pos_x) > threshold:
        #     deviation += abs(sensorData.pos_x - message.pos_x) - threshold
        #     suspicious = True
    
        self.decay(time_interval)

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