class GPSStub:
    def __init__(self):
        self.latitude = 30.0
        self.longitude = -88.0
        self.altitude = 0.0
        self.speed = 0.0

class BaroStub:
    def __init__(self):
        self.ALT = 0.0
        self.PRES = 1013.25

class IMUStub:
    def __init__(self):
        self.rpy = [0.0, 0.0, 0.0]
        self.g = [0.0, 0.0, 0.0]
