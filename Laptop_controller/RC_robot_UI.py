#!/usr/bin/env python3
#!/usr/bin/env python3
import pygame
import pygame_gui
import sys
import numpy as np
import serial
from Server import Server
import socket
import threading

class User_interface:
    # App General State
    WIDTH, HEIGHT = 1920, 540 # Screen Size 
    running = True
    in_popup = False
    active_popup = None
    temp_origin = None
    x = 0
    string = ""
    image_dict = {}
    rect_dict = {}
    current_action = ""

    # Log settings
    logs = []
    MAX_LOGS = 10
    log_messages = []
    LOG_PORT = 5001  # Port for receiving logs from the robot
    
    # Robot state
    message = 0  # Message to send to the robot
    run = True 
    stand = False
    kalman = True  # Kalman filter is always enabled and cannot be disabled
    release_space = True
    release_enter = True
    release_t = True

    def __init__(self):

        pygame.init()
        self.ser = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
        
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT), pygame.RESIZABLE)
        pygame.display.set_caption("Robot Controller")

        self.manager = pygame_gui.UIManager((self.WIDTH, self.HEIGHT))
        self.clock = pygame.time.Clock()
        self.clock.tick(200)

        self.server = Server()

        self.load_figures()

        self.ui_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ui_socket.bind(("0.0.0.0", 6000))  # Port UI listens on

        self.log_thread = threading.Thread(target=self.listen_to_logs, daemon=True)
        self.log_thread.start()

    def load_figures(self):
        arrow_img = pygame.image.load('./img/arrow.png')
        arrow_img = pygame.transform.scale(arrow_img, (arrow_img.get_width() // 4, arrow_img.get_height() // 4))

        circle_img = pygame.image.load('./img/point.png')
        circle_img = pygame.transform.scale(circle_img, (circle_img.get_width() // 2, circle_img.get_height() // 2))

        stop_img = pygame.image.load('./img/Stop_sign.png')
        stop_img = pygame.transform.scale(stop_img, (stop_img.get_width() // 10, stop_img.get_height() // 10))

        self.image_dict["stop"] = stop_img
        self.image_dict["arrow"] = arrow_img
        self.image_dict["circle"] = circle_img

######################################################### TRIGGER CHECK #################################################

    def event_handler(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            self.manager.process_events(event)

 
######################################################### KEYBOARD FUNCTIONS #################################################

    def check_keys_movement(self, keys):
        if keys[pygame.K_SPACE]:
            if self.release_space:
                self.release_space = False
                if self.message < 10000000:
                    self.run = True
                else:
                    self.run = False
                    self.current_action = ""
        elif keys[pygame.K_z] or keys[pygame.K_UP] or self.current_action == "front":
            self.x += -1
        elif keys[pygame.K_s] or keys[pygame.K_DOWN] or self.current_action == "back":
            self.x += 1
        elif keys[pygame.K_q] or keys[pygame.K_LEFT] or self.current_action == "left":
            self.x += 1j
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT] or self.current_action == "right":
            self.x += -1j
        elif keys[pygame.K_ESCAPE]:
            self.running = False
        
        else:
            self.release_space = True

    def check_keys_kalman(self, keys):
        # Kalman filter is always enabled and cannot be disabled
        self.kalman = True

    def check_test(self, keys):
        if keys[pygame.K_t] and self.release_t:
                self.test, self.release_t = True, False
        else:
            self.test, self.release_t =  False, True

    def check_standing(self, keys):
        if keys[pygame.K_RETURN] or self.current_action == "stand":
            if self.release_enter:
                self.stand = not self.stand
                self.release_enter = False       
        else:
            self.release_enter = True

######################################################### DRAWING FUNCTIONS #################################################

    def update_screen_size(self):   
        self.WIDTH, self.HEIGHT = self.screen.get_size()
        self.manager.set_window_resolution((self.WIDTH, self.HEIGHT))

    def draw_move_ctrl(self):
        if self.message < 10000000:
            self.draw_image("stop", self.WIDTH //2, 100)
        elif abs(self.x) == 0:
            self.draw_image("circle", self.WIDTH //2, 100)
        else:
            angle = np.angle(-1*self.x, deg=True)
            rotated_arrow = pygame.transform.rotate(self.image_dict.get("arrow"), angle)
            rotated_rect = rotated_arrow.get_rect(center = (self.WIDTH//2, 100))
            self.screen.blit(rotated_arrow, rotated_rect.topleft)

    def draw_string(self):
        font = pygame.font.Font(None, 28)
        self.string += "DYNAMIC\n" if not self.stand else "STATIC\n"
        self.string += "Kalman filter\n" if self.kalman else ""
        self.string += "Running\n" if self.run else "Stopped\n"
        self.string += "Message: " + str(self.message) + "\n"

        lines = self.string.split("\n") + ["--- LOGS ---"] + self.log_messages

        for i, line in enumerate(lines):
            text = font.render(line, True, (0, 128, 0))
            self.screen.blit(text, (10, 10 + i * 24))


######################################################### SERIAL COMM FUNCTIONS #################################################

    def serial_comm(self):
        data = self.run << 7 | self.kalman << 6 | self.test << 5 | self.stand << 4 | (self.x.real == 1) << 3 | (self.x.real == -1) << 2 | (
                    self.x.imag == 1) << 1 | (self.x.imag == -1)
        self.ser.write(bytes([data]))

        Content = self.ser.readline()
        Content = Content.decode().replace("\r\n", "")
        self.message = int(Content)

############################################################ HELPER FUNCTIONS #####################################################
    def is_click_image(self, name, event):
        return self.rect_dict.get(name) != None and self.rect_dict.get(name).collidepoint(event.pos)
    
    def load_image(self, room_num, object, side):
        img = pygame.image.load(object.img)
        img = pygame.transform.scale(img, (img.get_width() // 5, img.get_height() // 5))

        name = object.type + "_" + side + "_" + str(room_num)
        self.image_dict[name] = img

    def draw_image(self, name, x, y):
        plus_rect = self.image_dict.get(name).get_rect(center = (x, y))
        self.screen.blit(self.image_dict.get(name), self.image_dict.get(name).get_rect(center=plus_rect.center))
        self.rect_dict[name] = plus_rect
    
    def close_popup(self):
        self.active_popup.kill()
        self.active_popup = None
    
############################################ LOG LISTENER ################################################

    def listen_to_logs(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind(('', self.LOG_PORT))
            except OSError as e:
                print(f"[ERROR] Cannot bind UDP log socket on port {self.LOG_PORT}: {e}")
                return

            while True:
                try:
                    data, _ = s.recvfrom(1024)
                    msg = data.decode().strip()
                    self.logs.append(msg)
                    if len(self.logs) > self.MAX_LOGS:
                        self.logs.pop(0)
                except:
                    continue

    def draw_logs(self):
        font = pygame.font.Font(None, 24)
        base_y = self.HEIGHT - 25 * self.MAX_LOGS - 10
        for i, line in enumerate(self.logs):
            text = font.render(line, True, (0, 0, 0))
            self.screen.blit(text, (10, base_y + i * 25))


######################################################### MAIN LOOP ############################################################

    def main_loop(self):
        while self.running:
            self.screen.fill((255, 255, 255))

            self.event_handler()
            self.update_screen_size()
            
            keys = pygame.key.get_pressed()
            self.x = 0
            self.string = ""

            if not self.in_popup:            

                self.check_keys_movement(keys)
                self.check_keys_kalman(keys)
                self.check_test(keys)
                self.check_standing(keys)

            self.draw_move_ctrl()   
            self.draw_string()
            self.draw_logs()

            self.manager.update(self.clock.tick(60)/1000)
            self.manager.draw_ui(self.screen)
            pygame.display.flip()
            self.serial_comm()

        # Quit
        pygame.quit()
        sys.exit()

if __name__ == '__main__':
    ui = User_interface()
    ui.main_loop()
