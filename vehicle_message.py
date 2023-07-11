'''
Message class for the V2V communication
'''

class VehicleMessage():
    def __init__(self, pos_x = 0, pos_y = 0, speed = 0, acceleration = 0, timestamp = 0):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.speed = speed
        self.acceleration = acceleration
        self.timestamp = timestamp
    
    def get(self, item):
        if item == 'pos_x':
            return self.pos_x
        elif item == 'pos_y':
            return self.pos_y
        elif item == 'speed':
            return self.speed
        elif item == 'acceleration':
            return self.acceleration
        elif item == 'timestamp':
            return self.timestamp
        
    def __getitem__(self, item):
        if item == 'pos_x':
            return self.pos_x
        elif item == 'pos_y':
            return self.pos_y
        elif item == 'speed':
            return self.speed
        elif item == 'acceleration':
            return self.acceleration
        elif item == 'timestamp':
            return self.timestamp
        