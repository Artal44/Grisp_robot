import socket
import threading
from Sonar import Sonar
import time

class Server:
    HOST = "172.20.10.4"
  
    PORT = 5000  # Use the same port on both the server and the GRiSP device
    buffer = []
    robot_sonar = {}
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
                            self.send("Ack , server", "uni_sonar", role=role)
                except : 
                    pass

    def send(self, message, type, id=None, role= None):
        if type == "brd":
            threading.Thread(target=self.brd_server, args=(message,), daemon=True).start()
        elif type == "uni_sonar":
            threading.Thread(target=self.uni_server_sonar, args=(message, role), daemon=True).start()

    def brd_server(self, message):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv_socket:
            srv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            broadcast_ip = '172.20.10.15'
            port = 9000

            srv_socket.sendto(message.encode(), (broadcast_ip, port))
            print(f"[SERVER] Broadcasted to ({broadcast_ip}, {port}): {message}")

    def uni_server_sonar(self, message, role):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv_socket:
            sonar = self.robot_sonar.get(role)
            ip = sonar.ip
            port = sonar.port
            srv_socket.sendto(message.encode(), (ip, port))
            print("[SERVER] Sent to " + str(sonar.role) + " on (" + str(ip) + ", " + str(port) + ") : " + str(message))

    def send_start(self):
        self.started = True
        time.sleep(1)
        message = "Start " + self.HOST
        self.send(message, "brd")

if __name__ == '__main__' :
    serv = Server()

