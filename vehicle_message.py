'''
Message class for the V2V communication
'''

class VehicleMessage():
    def __init__(self, posX = 0, posY = 0, speed = 0, acceleration = 0, timestamp = 0):
        self.posX = posX
        self.posY = posY
        self.speed = speed
        self.acceleration = acceleration
        self.timestamp = timestamp
    
    def get(self, item):
        if item == 'posX':
            return self.posX
        elif item == 'posY':
            return self.posY
        elif item == 'speed':
            return self.speed
        elif item == 'acceleration':
            return self.acceleration
        elif item == 'timestamp':
            return self.timestamp
        