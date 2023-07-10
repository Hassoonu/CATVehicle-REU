class Reputation:
    def __init__(self, trust = 0.5):
        self.trust = trust
        return
    def isTrustworthyMessage():
        '''
        Returns whether the message is trustworthy
        '''
        return True
    def UpdateTrustScore(self, sensorData, message):
        threshold = 1
        suspicious = False
        if (sensorData.acceleration - message.acceleration) > threshold:
            self.trust -= abs(message.acceleration - threshold) / 100
            suspicious = True
        if (sensorData.speed - message.speed) > threshold:
            self.trust -= abs(message.speed - threshold) / 100
            suspicious = True
        if (sensorData.pos_y - message.pos_y) > threshold:
            self.trust -= abs(message.pos_y - threshold) / 100
            suspicious = True
        if (sensorData.pos_x - message.pos_x) > threshold:
            self.trust -= abs(message.pos_x - threshold) / 100
            suspicious = True
        if not suspicious:
            self.trust += .005
            
        # boundary conditions
        if self.trust > 1:
            self.trust = 1
        elif self.trust < 0:
            self.trust = 0
        return