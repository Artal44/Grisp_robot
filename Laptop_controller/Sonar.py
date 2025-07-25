import time

class Sonar :
    def __init__(self, IP, Port, Role):
        self.ip = IP
        self.port = Port
        self.role = Role
        self.last_seen = int(time.time())
        self.distance = 0.0

    def update_distance(self, distance, seq):
        self.distance = distance
        self.seq = seq
        self.last_seen = int(time.time())
