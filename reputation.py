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
        threshold = 0.5
        suspicious = False
        if (sensorData.acceleration - message.acceleration) > threshold:
            self.trust -= abs(message.acceleration - threshold)
            suspicious = True
        if (sensorData.speed - message.speed) > threshold:
            self.trust -= abs(message.speed - threshold)
            suspicious = True
        if (sensorData.posY - message.posY) > threshold:
            self.trust -= abs(message.posY - threshold)
            suspicious = True
        if (sensorData.posX - message.posX) > threshold:
            self.trust -= abs(message.posX - threshold)
            suspicious = True
        if not suspicious:
            self.trust += .02
            
        # boundary conditions
        if self.trust > 1:
            self.trust = 1
        elif self.trust < 0:
            self.trust = 0
        return