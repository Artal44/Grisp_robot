import socket
import threading
from Sensor import Sensor
from Robot import Robot
from Sonar import Sonar
import time

class Server:
    # Automatically get the local IP address
    # Get the local IP address by connecting to a public address (does not send data)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(('8.8.8.8', 80))
            HOST = s.getsockname()[0]
        except Exception:
            HOST = '127.0.0.1'
    PORT = 5000  # Use the same port on both the server and the GRiSP device
    buffer = []
    sensors = {}
    robot_sonar = {}
    robot = Robot()
    started = False

    def __init__(self):
        self.rcvServer = threading.Thread(target=self.rcv_server, daemon=True)
        self.rcvServer.start()
        self.pinger = threading.Thread(target=self.ping_server, daemon=True)
        self.pinger.start()

    def ping_server(self):
        while not self.started : 
            time.sleep(3)
            message = "ping : server , " + self.HOST + " , " + str(self.PORT)
            self.send(message, "brd")
            
    def rcv_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
            server_socket.bind((self.HOST, self.PORT))
            print(f"[SERVER] Listening for UDP packets on {self.HOST}:{self.PORT}")

            while True:
                data, addr = server_socket.recvfrom(1024)
                try :
                    data = data.decode()
                    if data[:5] == "Hello":
                        if data[11:17] == "ROBOT_":
                            role = data[17:]
                            self.robot_sonar[role] = Sonar(addr[0], addr[1], role)
                            print("[SERVER] Received hello from ROBOT_" + role)
                        else :
                            id = int(data[11:])
                            self.sensors[id] = Sensor(addr[0], addr[1], id)
                            print("[SERVER] Received hello from " + str(id) + " on (" + str(addr[0]) + ", " + str(addr[1]) + ")")
                        self.send("Ack , server", "uni", id)
                    elif data[:8] == "Distance":
                        self.sensors[addr[0]].update_data(float(data[9:]))
                except : 
                    pass

    def send(self, message, type, id=None):
        if type == "brd":
            threading.Thread(target=self.brd_server, args=(message,), daemon=True).start()
        elif type == "uni":
            threading.Thread(target=self.uni_server, args=(message, id), daemon=True).start()

    def brd_server(self, message):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv_socket:
            srv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            broadcast_ip = '172.20.10.15'
            port = 9000

            srv_socket.sendto(message.encode(), (broadcast_ip, port))
            print(f"[SERVER] Broadcasted to ({broadcast_ip}, {port}): {message}")

    def uni_server(self, message, id):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv_socket:
            sensor = self.sensors.get(id)
            ip = sensor.ip
            port = sensor.port
            srv_socket.sendto(message.encode(), (ip, port))
            print("[SERVER] Sent to " + str(sensor.id) + " on (" + str(ip) + ", " + str(port) + ") : " + str(message))

    def get_sensors(self):
        return self.sensors.keys()

    def update_sens(self, id, side, room, x, y):
        sens = self.sensors.get(id)
        sens.set_angle(side)
        sens.update_pos(room, x, y)
        
    def update_sens_height(self, id, height):
        self.sensors.get(id).update_height(height)

    def update_robot(self, pos, real_pos, angle, room):
        self.robot.update_pos(pos, real_pos, angle, room)
        
    def send_pos(self):
        self.started = True
        for sensor in self.sensors.values() :
            if sensor.x != -1 : 
                message = "Add_Device : sensor_" + str(sensor.id) + " , " + sensor.ip + " , " + str(sensor.port)
                self.send(message, "brd")
                time.sleep(0.5)
                message = "Pos " + str(sensor.id) + " : " + str(sensor.x) + " , " + str(sensor.y) + " , " + str(sensor.height) + " , " + str(sensor.angle) + " , " + str(sensor.room)
                self.send(message, "brd")
                time.sleep(0.5)

        if self.robot.ip != "0":
            message = "Add_Device : robot , " + self.robot.ip + " , " + str(self.robot.port)
            self.send(message, "brd")
            time.sleep(0.5)
            message = "Init_pos : " + str(self.robot.real_pos[0]) + " , " + str(self.robot.real_pos[1]) + " , " + str(self.robot.angle) + " , " + str(self.robot.room.room_num)
            self.send(message, "brd")
        time.sleep(1)
        message = "Start " + self.HOST
        self.send(message, "brd")

if __name__ == '__main__' :
    serv = Server()
