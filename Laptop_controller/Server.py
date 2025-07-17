import socket
import threading
from Sonar import Sonar
import time

class Server:
    HOST = "172.20.10.4"
    
    ui_port = 5001          # Port sur lequel le UI écoute
    PORT = 5000  # Use the same port on both the server and the GRiSP device
    robot_sonar = {}
    last_seen = {}  # role -> last timestamp
    lost_robots = set()

    def __init__(self):
        # Create robot_debug.txt if it doesn't exist otherwise clear it
        with open("robot_debug.txt", "w") as log_file:
            log_file.write("LOGGING STARTED\n")
        self.rcvServer = threading.Thread(target=self.rcv_server, daemon=True)
        self.rcvServer.start()
        self.pinger = threading.Thread(target=self.ping_server, daemon=True)
        self.pinger.start()
        self.alive_checker = threading.Thread(target=self.check_alive_loop, daemon=True)
        self.alive_checker.start()

    def ping_server(self):
        while not len(self.robot_sonar) == 3:
            time.sleep(5)
            message = "ping : server , " + self.HOST + " , " + str(self.PORT)
            self.send(message, "brd")

    def broadcast_devices(self):
        for role, sonar in self.robot_sonar.items():
            msg = f"Add_Device , {role} , {sonar.ip} , {sonar.port}"
            self.send(msg, "brd")
            self.log(f"[SERVER] Broadcasted device info for {role}")

    def uni_devices(self, receiver):
        for role, sonar in self.robot_sonar.items():
            msg = f"Add_Device , {role} , {sonar.ip} , {sonar.port}"
            self.send(msg, "uni_sonar", role=receiver)
            self.log(f"[SERVER] Broadcasted device info for {role}")

    def rcv_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
            server_socket.bind((self.HOST, self.PORT))
            self.log(f"[SERVER] Listening for UDP packets on {self.HOST}:{self.PORT}")

            while True:
                data, addr = server_socket.recvfrom(1024)
                try :
                    data = data.decode()
                    if data[:16] == "Hello from robot":
                        role = data[11:]
                        is_reconnect = role in self.robot_sonar
                        self.robot_sonar[role] = Sonar(addr[0], addr[1], role)
                        self.last_seen[role] = int(time.time())
                        self.send("Ack , server", "uni_sonar", role=role)
                        self.log("[SERVER] Received hello from " + role)
                        
                        if len(self.robot_sonar) == 3:
                            self.broadcast_devices()
                        elif is_reconnect:
                            self.lost_robots.discard(role)
                            self.log(f"[SERVER] Reconnected robot {role}, rebroadcasting peers")
                            self.uni_devices(role)
                    elif data.startswith("log :"):
                        log_message = data[6:].strip()
                        with open("robot_debug.txt", "a") as log_file:
                            log_file.write(log_message + "\n")
                    elif data.startswith("alive :"):
                        role = data.split(":", 1)[1].strip()
                        self.last_seen[role] = int(time.time())
                        if role not in self.robot_sonar:
                            self.robot_sonar[role] = Sonar(addr[0], addr[1], role)
                            if len(self.robot_sonar) == 3:
                                self.broadcast_devices()
                        self.log(f"[SERVER] Received ALIVE from {role} at {self.last_seen[role]}")
                    else :
                        self.log(f"[SERVER] Received unknown message: {data}")
                except : 
                    pass

    def send(self, message, type, id=None, role=None):
        if type == "brd":
            threading.Thread(target=self.brd_server, args=(message,), daemon=True).start()
        elif type == "uni_sonar":
            threading.Thread(target=self.uni_server_sonar, args=(message, role), daemon=True).start()
        elif type == "uni_main":
            threading.Thread(target=self.uni_server_main, args=(message,), daemon=True).start()

    def brd_server(self, message):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv_socket:
            srv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            broadcast_ip = '172.20.10.15'
            port = 9000

            srv_socket.sendto(message.encode(), (broadcast_ip, port))
            self.log(f"[SERVER] Broadcasted to ({broadcast_ip}, {port}): {message}")

    def uni_server_sonar(self, message, role):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv_socket:
            sonar = self.robot_sonar.get(role)
            ip = sonar.ip
            port = sonar.port
            srv_socket.sendto(message.encode(), (ip, port))
            self.log("[SERVER] Sent to " + str(sonar.role) + " on (" + str(ip) + ", " + str(port) + ") : " + str(message))

    def uni_server_main(self, message):
        robot_main = self.robot_sonar.get("robot_main")
        if robot_main:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv_socket:
                ip = robot_main.ip
                port = robot_main.port
                srv_socket.sendto(message.encode(), (ip, port))
                self.log(f"[SERVER] Sent to robot_main ({ip}, {port}) : {message}")

    def check_alive_loop(self):
        while True:
            now = int(time.time()) # Current time in seconds
            for role, last_time in self.last_seen.items():
                if now - last_time > 30:
                    self.log(f"[SERVER] Lost contact with {role}! Last seen {now - last_time} seconds ago.")
                    if role not in self.lost_robots:
                        self.lost_robots.add(role)
                    self.try_reconnect(role)
                else:
                    self.lost_robots.discard(role)  # It's alive again
            time.sleep(5)

    def try_reconnect(self, role):
        self.log(f"[SERVER] Attempting to reconnect {role}...")
        msg = "ping : server , " + self.HOST + " , " + str(self.PORT)
        self.send(msg, "brd")

    def log(self, msg):
        print(msg)
        self.send_to_ui(msg)

    def send_to_ui(self, message):
        if self.HOST and self.ui_port:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.sendto(message.encode(), (self.HOST, self.ui_port))


if __name__ == '__main__' :
    serv = Server()

