class Robot:
    pos = (0,0)
    real_pos = (0,0)
    angle = 0
    room = -1
    ip = "1"
    port = 0

    def __init__(self):
        self.confirmed = False

    def update_adress(self, ip: str, port: int) -> None:
        self.ip = ip
        self.port = port

    def update_pos(self, pos: tuple, real_pos: tuple, angle: int, room: int) -> None:
        print("[ROBOT] is at " + str(real_pos) + " with an angle of " + str(angle) + " in room " + str(room.room_num))
        self.pos = pos
        self.real_pos = real_pos
        self.angle = angle
        self.room = room