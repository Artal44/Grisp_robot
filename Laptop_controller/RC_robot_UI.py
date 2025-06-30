import pygame
import pygame_gui
import sys
import numpy as np
import serial
from Server import Server

class User_interface:

    # App General State
    WIDTH, HEIGHT = 1920, 540 # Screen Size
    RESIZE = 2 # Resizing factor for the rooms
    running = True
    in_popup = False
    active_popup = None
    UI_elements = {}
    temp_origin = None
    x = 0
    string = ""
    image_dict = {}
    rect_dict = {}
    current_action = ""


    # Robot state
    message = 0  #Message to send to the robot
    run = True 
    stand = False
    kalman = True
    release_space = True
    release_enter = True
    release_t = True
    release_tab = True

    # Saved Files
    saved_files = []
    
    def __init__(self, trajectory):

        pygame.init()
        self.ser = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
        
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT), pygame.RESIZABLE)
        pygame.display.set_caption("Robot Controller")
²
        self.manager = pygame_gui.UIManager((self.WIDTH, self.HEIGHT))
        self.clock = pygame.time.Clock()
        self.clock.tick(200)

        self.server = Server()

        self.load_figures()

    def load_figures(self):
        arrow_img = pygame.image.load('./img/arrow.png')
        arrow_img = pygame.transform.scale(arrow_img, (arrow_img.get_width() // 4, arrow_img.get_height() // 4))

        circle_img = pygame.image.load('./img/point.png')
        circle_img = pygame.transform.scale(circle_img, (circle_img.get_width() // 2, circle_img.get_height() // 2))

        stop_img = pygame.image.load('./img/Stop_sign.png')
        stop_img = pygame.transform.scale(stop_img, (stop_img.get_width() // 10, stop_img.get_height() // 10))

        robot = pygame.image.load('./img/Robot.png')
        robot = pygame.transform.scale(robot, (robot.get_width()//4, robot.get_height()//4))

        minus_img = pygame.image.load('./img/minus.png')
        minus_img = pygame.transform.scale(minus_img, (minus_img.get_width() // 5, minus_img.get_height() // 5))

        start_img = pygame.image.load('./img/button_start.png')
        start_img = pygame.transform.scale(start_img, (start_img.get_width(), start_img.get_height()))

        start_img_pressed = pygame.image.load('./img/start_pressed.png')
        start_img_pressed = pygame.transform.scale(start_img_pressed, (start_img_pressed.get_width(), start_img_pressed.get_height()))

        save_img = pygame.image.load('./img/button_save.png')
        save_img = pygame.transform.scale(save_img, (save_img.get_width(), save_img.get_height()))

        load_img = pygame.image.load('./img/button_load.png')
        load_img = pygame.transform.scale(load_img, (load_img.get_width(), load_img.get_height()))

        zoom_in = pygame.image.load('./img/zoom_in.png')
        zoom_in = pygame.transform.scale(zoom_in, (zoom_in.get_width()//8, zoom_in.get_height()//8))

        zoom_out = pygame.image.load('./img/zoom_out.png')
        zoom_out = pygame.transform.scale(zoom_out, (zoom_out.get_width()//8, zoom_out.get_height()//8))

        self.image_dict["arrow"] = arrow_img
        self.image_dict["circle"] = circle_img
        self.image_dict["stop"] = stop_img
        self.image_dict["minus"] = minus_img
        self.image_dict["start"] = start_img
        self.image_dict["save"] = save_img
        self.image_dict["load"] = load_img
        self.image_dict["start_pressed"] = start_img_pressed
        self.image_dict["zoom_in"] = zoom_in
        self.image_dict["zoom_out"] = zoom_out
        self.image_dict["robot"] = robot

######################################################### TRIGGER CHECK #################################################

    def event_handler(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.server.send("Exit", "brd")
                self.running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                self.event_click(event)

            self.manager.process_events(event)

    def event_click(self, event):
        if self.in_popup:
            return
        elif self.is_click_image("start", event):
            self.is_trajectory_started = True
            self.server.send_start()
            self.timer = pygame.time.get_ticks()/1000                                         
        self.in_popup = False

######################################################### KEYBOARD FUNCTIONS #################################################

    def check_keys_movement(self, keys):
        if keys[pygame.K_SPACE]:
            if self.release_space:
                self.release_space = False
                if self.message < 10000000:
                    self.run = True
                else:
                    self.run = False
                    self.is_trajectory_started = False
                    self.current_action = ""
                    self.action_duration = 0
                    self.trajectory_idx = 0
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
        if keys[pygame.K_k]:
            self.kalman = True
        elif keys[pygame.K_c]:
            self.kalman = False
        elif keys[pygame.K_TAB]:
            if self.release_tab:
                self.kalman = not self.kalman
                self.release_tab = False
        else:
            self.release_tab = True

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
        font = pygame.font.Font(None, 36)
        self.string += "DOWN \n" if not self.stand else "UP \n"
        self.string += "Kalman filter\n" if self.kalman else "Complementary filter\n"
        self.string += "Running\n" if self.run else "Stopped\n"
        self.string += "Message: " + str(self.message) + "\n"
        self.string += "Timer : 0"

        for i, line in enumerate(self.string.split("\n")):
            text = font.render(line, True, (0, 128, 0))
            self.screen.blit(text, (10, 10 + i * 30))

    def draw_buttons(self):
        self.draw_image("start", self.WIDTH-200, 100)

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
    
    def compute_screen_size(self, width, height):
        return int(width * (self.HEIGHT//self.RESIZE)), int(height * (self.HEIGHT//self.RESIZE))
    
    def close_popup(self):
        self.active_popup.kill()
        self.active_popup = None
    
######################################################### MAIN LOOP ############################################################

    def main_loop(self):
        while self.running:
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

            self.screen.fill((255, 255, 255))

            self.draw_move_ctrl()
            self.draw_buttons()
            self.draw_string()

            self.manager.update(self.clock.tick(60)/1000)
            self.manager.draw_ui(self.screen)
            pygame.display.flip()

            self.serial_comm()

        # Quit
        pygame.quit()
        sys.exit()

if __name__ == '__main__':
    if len(sys.argv) == 1:
        ui = User_interface(None)
    else :
        ui = User_interface(sys.argv[1])
    ui.main_loop()