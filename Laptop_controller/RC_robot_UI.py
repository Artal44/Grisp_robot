import pygame
import pygame_gui
import sys
import numpy as np
import serial
from Server import Server
from Room import Room
from Robot import Robot
from pathlib import Path
import SaveParser
import time

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

    # App Room state
    room_grid = ((0,0),(0,0))
    rooms = []
    sensor = []

    # Predefined trajectory
    trajectory = []
    trajectory_idx = 0
    current_action = ""
    action_start_time = 0
    action_duration = 0
    is_trajectory_started = False
    timer = 0

    # Robot UI
    robot = None

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

        self.manager = pygame_gui.UIManager((self.WIDTH, self.HEIGHT))
        self.clock = pygame.time.Clock()
        self.clock.tick(200)

        self.server = Server()
        self.defined_trajectory(trajectory)

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

        plus_img = pygame.image.load('./img/plus.png')
        plus_img = pygame.transform.scale(plus_img, (plus_img.get_width() // 5, plus_img.get_height() // 5))

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
        self.image_dict["plus_L_0"] = plus_img
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
                
            if event.type == pygame_gui.UI_BUTTON_PRESSED:
                self.event_interact_popup(event)

            self.manager.process_events(event)

    def event_click(self, event):
        if self.in_popup:
            return
        
        robot_room = None
        for room in range(len(self.rooms)):

            for side in ["L", "R", "T", "B"]:
                name = "plus_" + side + "_" + str(room)
                if self.is_click_image(name, event) :
                    self.in_popup = True
                    self.temp_origin = name
                    self.create_choice_popup()

            for corner in ["TL", "TR", "BL", "BR"]:
                name = "plus_" + corner + "_" + str(room)
                if self.is_click_image(name, event) :
                    self.in_popup = True
                    self.temp_origin = name
                    self.create_sensor_popup()

            room_obj = self.rooms[room]
            
            room_rect = pygame.Rect(0, 0, room_obj.width, room_obj.height)
            room_rect.center = room_obj.pos

            if room_rect.collidepoint(event.pos):
                robot_room = room_obj

        if robot_room != None and self.robot == None:
            self.in_popup = True
            x, y = self.get_real_pos(event.pos[0], event.pos[1])
            self.server.update_robot(event.pos, (x,y), 0, robot_room)
            self.robot = self.server.robot
            self.create_robot_popup()
        
        elif len(self.rooms) == 0 and self.is_click_image("plus_L_0", event):
            self.in_popup = True
            self.temp_origin = "plus_L_0"
            self.create_room_popup()

        elif self.is_click_image("start", event) :
            self.server.send_pos()
            time.sleep(2)
            self.is_trajectory_started = True
            self.timer = pygame.time.get_ticks()/1000

        elif self.is_click_image("save", event):
            self.in_popup = True
            self.create_save_popup()

        elif self.is_click_image("load", event):
            self.in_popup = True
            self.create_load_popup()

        elif self.is_click_image("zoom_in", event):
            if(self.RESIZE != 1):            
                for room in self.rooms:
                    room.update_size(self.RESIZE, self.RESIZE-1, self.HEIGHT)
                self.RESIZE -= 1

        elif self.is_click_image("zoom_out", event):  

            for room in self.rooms:
                room.update_size(self.RESIZE, self.RESIZE+1, self.HEIGHT)
            self.RESIZE += 1

    def event_interact_popup(self, event):
        if event.ui_element == self.UI_elements.get("Room_Submit"):
            width = self.UI_elements.get("Width").get_text()
            height = self.UI_elements.get("Height").get_text()
            if width != "" and height != "":
                try :
                    screen_width, screen_height = self.compute_screen_size(float(width), float(height))
                    side = self.temp_origin[5]
                    x, y = self.compute_pos_room(screen_width, screen_height, side, len(self.rooms))

                    room = Room(screen_width, screen_height, x, y, len(self.rooms))
                    self.add_sides(room)
                    self.rooms.append(room)
                    self.get_new_grid()
                except :
                    print("[ERROR] : Problem with width and height values")
            self.close_popup()
            self.temp_origin = None
        elif event.ui_element == self.UI_elements.get("Save_Submit"):
            filename = self.UI_elements.get("Filename").get_text()
            self.create_save_file(filename)
            self.close_popup()
        elif event.ui_element == self.UI_elements.get("Sensor"):
            self.close_popup()
            self.create_sensor_popup()
        elif event.ui_element == self.UI_elements.get("SensH_Submit"):
            height = self.UI_elements.get("Sens_height").get_text()

            height = float(height)
            self.server.update_sens_height(self.temp_origin, height)

            self.close_popup()
            self.temp_origin = None

        elif event.ui_element == self.UI_elements.get("Room"):
            self.close_popup()
            self.create_room_popup()
        elif event.ui_element == self.UI_elements.get("yes"):
            self.robot.confirmed = True
            self.close_popup()
        elif event.ui_element == self.UI_elements.get("no"):
            self.robot = None
            self.close_popup()
        
        #Check sensor choice 
        sensors = self.server.get_sensors()
        for id in sensors:
            if event.ui_element == self.UI_elements.get("Sensor choice " + str(id)):
                self.close_popup()
                splitted_origin = self.temp_origin.split("_")
                room = int(splitted_origin[2])
                side = splitted_origin[1]

                room = self.rooms[room]

                ix, iy = room.compute_pos(side)
                
                x, y = self.get_real_pos(ix, iy)
                self.create_sensor_height_popup()
                self.server.update_sens(id, side, room.room_num, x, y)
                self.draw_sensor()
                self.temp_origin = id

        #File loader choice
        for filename in self.saved_files:
            if event.ui_element == self.UI_elements.get("SavedFile" + filename[:-4]) :
                self.rooms = SaveParser.parse(filename, self.HEIGHT, self.RESIZE)    
                self.close_popup()
                                            
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
        if self.is_trajectory_started :
            self.string += "Timer : " + str(round((pygame.time.get_ticks()/1000) - self.timer, 1))
        else : 
            self.string += "Timer : 0"

        for i, line in enumerate(self.string.split("\n")):
            text = font.render(line, True, (0, 128, 0))
            self.screen.blit(text, (10, 10 + i * 30))

    def draw_add_room(self):
        if len(self.rooms) == 0:
            self.draw_image("plus_L_0", self.WIDTH//2, self.HEIGHT//2)

    def draw_image(self, name, x, y):
        plus_rect = self.image_dict.get(name).get_rect(center = (x, y))
        self.screen.blit(self.image_dict.get(name), self.image_dict.get(name).get_rect(center=plus_rect.center))
        self.rect_dict[name] = plus_rect

    def draw_room(self, room):
        room_rect = pygame.Rect(0, 0, room.width, room.height)
        room_rect.center = (room.pos[0], room.pos[1])

        pygame.draw.rect(self.screen, (0, 0, 0), room_rect, width=10)
        self.draw_room_sides(room)

    def draw_room_sides(self, room):
        for side in ["L", "R", "T", "B"]:
            if type(room.sides[side]) != Room:
                self.load_image(room.room_num, room.sides[side], side)
                self.draw_image(room.sides[side].type + "_" + side + "_" + str(room.room_num), room.sides[side].pos[0], room.sides[side].pos[1])
            else:
                x, y = room.compute_pos(side)
                self.draw_door(x, y)
        for corner in ["TL", "TR", "BL", "BR"]:
            self.load_image(room.room_num, room.corners[corner], corner)
            self.draw_image(room.corners[corner].type + "_" + corner + "_" + str(room.room_num), room.corners[corner].pos[0], room.corners[corner].pos[1])

    def draw_sensor(self):
        if self.temp_origin[7] =="_":
            room_origin = int(self.temp_origin[8])
            side_origin = self.temp_origin[5:7]
            self.sensor.append(self.rooms[room_origin].corners[side_origin])
        else :
            room_origin = int(self.temp_origin[7])
            side_origin = self.temp_origin[5]
            self.sensor.append(self.rooms[room_origin].sides[side_origin])
        self.rooms[room_origin].modify_side(side_origin, "./img/sensor.png", "sensor")
        
    def draw_door(self, x, y):
        width, height = self.compute_screen_size(0.3, 0.3)
        door_rect = pygame.Rect(0, 0, width, height)
        door_rect.center = (x, y)
        pygame.draw.rect(self.screen, (255, 255, 255), door_rect)

    def draw_grid(self):
        RED = (255, 0, 0)

        point1 = (self.room_grid[0][0], self.room_grid[1][0])
        point2 = (self.room_grid[0][1], self.room_grid[1][0])
        point3 = (self.room_grid[0][1], self.room_grid[1][1])

        pygame.draw.line(self.screen, RED, point1, point2, width=10)
        pygame.draw.line(self.screen, RED, point2, point3, width=10)

    def draw_buttons(self):
        if self.is_trajectory_started :
            self.draw_image("start_pressed", self.WIDTH-200, 100)
        else :
            self.draw_image("start", self.WIDTH-200, 100)
        self.draw_image("save", self.WIDTH-375, 100)
        self.draw_image("load", self.WIDTH-550, 100)

        # Draw zoom
        self.draw_image("zoom_in", self.WIDTH - 200, self.HEIGHT - 100)
        self.draw_image("zoom_out", self.WIDTH - 375, self.HEIGHT - 100)

    def draw_robot(self):
        if self.robot != None and self.robot.confirmed:
            self.draw_image("robot", self.robot.pos[0], self.robot.pos[1])
######################################################## POPUPS CREATORS #####################################################
    
    def create_choice_popup(self):
        button_width = self.WIDTH // 2 - self.WIDTH // 20
        button_height = min(self.HEIGHT // 20, 60)
        popup_width = self.WIDTH // 2
        popup_height = self.HEIGHT // 3
        margin_left = (self.WIDTH - button_width)//20
        margin = 20

        # Center the popup on the screen
        popup_rect = pygame.Rect(
            (self.WIDTH - popup_width) // 2,
            (self.HEIGHT - popup_height) // 2,
            popup_width,
            popup_height
        )

        popup_window = pygame_gui.elements.UIWindow(
            rect=popup_rect,
            manager=self.manager,
            window_display_title='Add Component'
        )

        self.active_popup = popup_window

        current_y = margin

        header_label = pygame_gui.elements.UILabel(
            
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Do you Want to add a Room or a sensor ?",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        self.UI_elements["Sensor"] = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Sensor",
            manager=self.manager,
            container=popup_window
        )

        current_y += button_height + margin

        self.UI_elements["Room"] = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Room",
            manager=self.manager,
            container=popup_window
        )

    def create_room_popup(self):
        # Calculate sizes for buttons and popup dimensions
        button_width = self.WIDTH // 2 - self.WIDTH // 20
        button_height = min(self.HEIGHT // 20, 60)
        popup_width = self.WIDTH // 2
        popup_height = self.HEIGHT // 2
        margin_left = (self.WIDTH - button_width)//20
        margin = 20

        # Center the popup on the screen
        popup_rect = pygame.Rect(
            (self.WIDTH - popup_width) // 2,
            (self.HEIGHT - popup_height) // 2,
            popup_width,
            popup_height
        )

        popup_window = pygame_gui.elements.UIWindow(
            rect=popup_rect,
            manager=self.manager,
            window_display_title='Room Creation'
        )

        self.active_popup = popup_window

        current_y = margin

        header_label = pygame_gui.elements.UILabel(
            
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Enter the room size:",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        width_label = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Width (m):",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        self.UI_elements["Width"] = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            manager=self.manager,
            container=popup_window
        )

        current_y += button_height + margin

        height_label = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Height (m):",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        self.UI_elements["Height"] = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            manager=self.manager,
            container=popup_window
        )

        current_y += int(1.75 * button_height) + margin

        self.UI_elements["Room_Submit"] = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Submit",
            manager=self.manager,
            container=popup_window
        )

        self.manager.draw_ui(self.screen)
        pygame.display.update()

    def create_sensor_popup(self):
        # Calculate sizes for buttons and popup dimensions
        button_width = self.WIDTH // 2 - self.WIDTH // 20
        button_height = min(self.HEIGHT // 20, 60)
        popup_width = self.WIDTH // 2
        popup_height = self.HEIGHT // 3
        margin_left = (self.WIDTH - button_width)//20
        margin = 20

        # Retrieve sensors Ids
        sensors = self.server.get_sensors()

        # Center the popup on the screen
        popup_rect = pygame.Rect(
            (self.WIDTH - popup_width) // 2,
            (self.HEIGHT - popup_height) // 2,
            popup_width,
            popup_height
        )

        popup_window = pygame_gui.elements.UIWindow(
            rect=popup_rect,
            manager=self.manager,
            window_display_title='Sensor Choice'
        )

        self.active_popup = popup_window

        current_y = margin

        header_label = pygame_gui.elements.UILabel(
            
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Choose a sensor:",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        for Id in sensors :
            self.UI_elements["Sensor choice " + str(Id)] = pygame_gui.elements.UIButton(
                relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
                text=str(Id),
                manager=self.manager,
                container=popup_window
            )

            current_y += button_height + margin

        self.manager.draw_ui(self.screen)
        pygame.display.update()

    def create_sensor_height_popup(self):
        # Calculate sizes for buttons and popup dimensions
        button_width = self.WIDTH // 2 - self.WIDTH // 20
        button_height = min(self.HEIGHT // 20, 60)
        popup_width = self.WIDTH // 2
        popup_height = self.HEIGHT // 5
        margin_left = (self.WIDTH - button_width)//20
        margin = 20

        # Center the popup on the screen
        popup_rect = pygame.Rect(
            (self.WIDTH - popup_width) // 2,
            (self.HEIGHT - popup_height) // 2,
            popup_width,
            popup_height
        )

        popup_window = pygame_gui.elements.UIWindow(
            rect=popup_rect,
            manager=self.manager,
            window_display_title='Sensor Height'
        )

        self.active_popup = popup_window

        current_y = margin

        header_label = pygame_gui.elements.UILabel(
            
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="At what height is the sensor (count to the center of the sonar):",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        self.UI_elements["Sens_height"] = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        self.UI_elements["SensH_Submit"] = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Submit",
            manager=self.manager,
            container=popup_window
        )

        self.manager.draw_ui(self.screen)
        pygame.display.update()

    def create_save_popup(self):
        # Calculate sizes for buttons and popup dimensions
        button_width = self.WIDTH // 2 - self.WIDTH // 20
        button_height = min(self.HEIGHT // 20, 60)
        popup_width = self.WIDTH // 2
        popup_height = self.HEIGHT // 3
        margin_left = (self.WIDTH - button_width)//20
        margin = 20

        # Center the popup on the screen
        popup_rect = pygame.Rect(
            (self.WIDTH - popup_width) // 2,
            (self.HEIGHT - popup_height) // 2,
            popup_width,
            popup_height
        )

        popup_window = pygame_gui.elements.UIWindow(
            rect=popup_rect,
            manager=self.manager,
            window_display_title='Save Config'
        )

        self.active_popup = popup_window

        current_y = margin

        header_label = pygame_gui.elements.UILabel(
            
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Save as :",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        self.UI_elements["Filename"] = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        self.UI_elements["Save_Submit"] = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Submit",
            manager=self.manager,
            container=popup_window
        )

        self.manager.draw_ui(self.screen)
        pygame.display.update()

    def create_load_popup(self):
        # Calculate sizes for buttons and popup dimensions
        button_width = self.WIDTH // 2 - self.WIDTH // 20
        button_height = min(self.HEIGHT // 20, 60)
        popup_width = self.WIDTH // 2
        popup_height = self.HEIGHT // 3
        margin_left = (self.WIDTH - button_width)//20
        margin = 20

        # Retrieve sensors Ids
        self.saved_files = self.get_saved_files()

        # Center the popup on the screen
        popup_rect = pygame.Rect(
            (self.WIDTH - popup_width) // 2,
            (self.HEIGHT - popup_height) // 2,
            popup_width,
            popup_height
        )

        popup_window = pygame_gui.elements.UIWindow(
            rect=popup_rect,
            manager=self.manager,
            window_display_title='Saved Configs'
        )

        self.active_popup = popup_window

        current_y = margin

        header_label = pygame_gui.elements.UILabel(
            
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Select a file to load:",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin


        for name in self.saved_files :
            self.UI_elements["SavedFile" + name[:-4]] = pygame_gui.elements.UIButton(
                relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
                text=name[:-4],
                manager=self.manager,
                container=popup_window
            )

            current_y += button_height + margin

        self.manager.draw_ui(self.screen)
        pygame.display.update()

    def create_robot_popup(self):
        # Calculate sizes for buttons and popup dimensions
        button_width = self.WIDTH // 2 - self.WIDTH // 20
        button_height = min(self.HEIGHT // 20, 60)
        popup_width = self.WIDTH // 2
        popup_height = self.HEIGHT // 6
        margin_left = (self.WIDTH - button_width)//20
        margin = 20

        # Center the popup on the screen
        popup_rect = pygame.Rect(
            (self.WIDTH - popup_width) // 2,
            (self.HEIGHT - popup_height) // 2,
            popup_width,
            popup_height
        )

        popup_window = pygame_gui.elements.UIWindow(
            rect=popup_rect,
            manager=self.manager,
            window_display_title='Place the robot ?'
        )

        self.active_popup = popup_window

        current_y = margin

        header_label = pygame_gui.elements.UILabel(
            
            relative_rect=pygame.Rect(margin_left, current_y, button_width, button_height),
            text="Do you want to place the robot here ?",
            manager=self.manager,
            container=popup_window
        )
        current_y += button_height + margin

        self.UI_elements["yes"] = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(margin_left, current_y, button_width//2 - margin_left, button_height),
            text="Yes",
            manager=self.manager,
            container=popup_window
        )

        self.UI_elements["no"] = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(2*margin_left + button_width//2, current_y, button_width//2 - margin_left, button_height),
            text="No",
            manager=self.manager,
            container=popup_window
        )

        self.manager.draw_ui(self.screen)
        pygame.display.update()

######################################################### SERIAL COMM FUNCTIONS #################################################

    def serial_comm(self):
        data = self.run << 7 | self.kalman << 6 | self.test << 5 | self.stand << 4 | (self.x.real == 1) << 3 | (self.x.real == -1) << 2 | (
                    self.x.imag == 1) << 1 | (self.x.imag == -1)
        self.ser.write(bytes([data]))

        Content = self.ser.readline()
        Content = Content.decode().replace("\r\n", "")
        self.message = int(Content)

    def defined_trajectory(self, file):
        if file != None:
            with open("./trajectories/" + file + ".txt") as file:
                lines = file.readlines()
                for line in lines:
                    action, duration = line.split(" : ")
                    self.trajectory.append((action, duration))

        else:
            self.trajectory = None

############################################################ HELPER FUNCTIONS #####################################################
    def is_click_image(self, name, event):
        return self.rect_dict.get(name) != None and self.rect_dict.get(name).collidepoint(event.pos)
    
    def load_image(self, room_num, object, side):
        img = pygame.image.load(object.img)
        img = pygame.transform.scale(img, (img.get_width() // 5, img.get_height() // 5))

        name = object.type + "_" + side + "_" + str(room_num)
        self.image_dict[name] = img

    def compute_pos_room(self, adapted_height, adapted_width, side, room_num):
        x, y = self.rect_dict[self.temp_origin].center[0], self.rect_dict[self.temp_origin].center[1]
        if room_num == 0 :
         return x, y
        else :
            match side :
                case "L":
                    return (x - adapted_width//2)+10, y
                case "R":
                    return (x + adapted_width//2)-10, y
                case "T": 
                    return x, (y + adapted_height//2)-10
                case "B":
                    return x, (y - adapted_height//2)+10 
    
    def compute_screen_size(self, width, height):
        return int(width * (self.HEIGHT//self.RESIZE)), int(height * (self.HEIGHT//self.RESIZE))
    
    def close_popup(self):
        self.active_popup.kill()
        self.active_popup = None

    def add_sides(self, room):
        sides = ["L", "R", "T", "B"]
        corners = ["TL", "TR", "BL", "BR"]
        room_origin = int(self.temp_origin[7])
        side_origin = self.temp_origin[5]

        if len(self.rooms) !=0:
            if side_origin == "L":
                sides.remove("R")
                room.add_room("R", self.rooms[room_origin])
            elif side_origin == "R":
                sides.remove("L")
                room.add_room("L", self.rooms[room_origin])
            elif side_origin == "T":
                sides.remove("B")
                room.add_room("B", self.rooms[room_origin])
            elif side_origin == "B":
                sides.remove("T")
                room.add_room("T", self.rooms[room_origin])

            self.rooms[room_origin].add_room(side_origin, room)

        for side in sides:
            room.modify_side(side, "./img/plus.png", "plus")
        for corner in corners:
            room.modify_side(corner, "./img/plus.png", "plus")

    def get_new_grid(self):
        leftmost_room = None
        rightmost_room = None
        
        upmost_room = None
        downmost_room = None

        x_min = self.WIDTH+1
        x_max = 0
        y_min = self.HEIGHT+1
        y_max = 0
        for room in self.rooms:

            if room.pos[0] < x_min:                
                x_min = room.pos[0]
                leftmost_room = room
            if room.pos[0] > x_max:                
                x_max = room.pos[0]
                rightmost_room = room

            if room.pos[1] < y_min:
                y_min = room.pos[1]
                upmost_room = room
            if room.pos[1] > y_max:
                y_max = room.pos[1]
                downmost_room = room
        
        x_min -= leftmost_room.width//2
        x_max += rightmost_room.width//2

        y_min -= upmost_room.height//2
        y_max += downmost_room.height//2

        self.room_grid = ((x_min, x_max), (y_min, y_max))

    def get_real_pos(self, x, y):
        grid_x = round((x - self.room_grid[0][0])/(self.HEIGHT/self.RESIZE), 2)
        grid_y = round((y - self.room_grid[1][0])/(self.HEIGHT/self.RESIZE), 2)

        return grid_x, grid_y
    
    def check_trajectory(self):
        if self.is_trajectory_started:
                if self.trajectory != None :
                    current_time = pygame.time.get_ticks() / 1000.0
                    if self.action_start_time + self.action_duration <= current_time and len(self.trajectory) >= self.trajectory_idx +1: 
                        self.current_action = self.trajectory[self.trajectory_idx][0]
                        self.action_duration = float(self.trajectory[self.trajectory_idx][1])
                        self.action_start_time = pygame.time.get_ticks() / 1000.0
                        self.trajectory_idx += 1

    def create_save_file(self, filename):
        with open('./saves/' + filename+'.txt', "w") as file:
            file.write("RESIZE_FACTOR : " + str(self.RESIZE) +"\n")
            file.write("HEIGHT : " + str(self.HEIGHT) +"\n")
            for room in self.rooms:
                line = str(room.width) + ", " + str(room.height) + ", " + str(room.pos) + ", " + str(room.room_num)+"\n"
                for side in room.sides:
                    pass
                    """ side_obj = room.sides.get(side)
                    line += ", " + str(side_obj.pos) + ";" +  str(side_obj.img) + ";" + str(side_obj.type) """
                file.write(line)

    def get_saved_files(self):
        directory = Path("./saves")
        files = [f.name for f in directory.iterdir() if f.is_file()]
        return files
    
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

            for room in self.rooms:
                self.draw_room(room)

            self.draw_move_ctrl()
            self.draw_buttons()
            self.draw_add_room()
            self.draw_string()
            self.draw_robot()

            self.check_trajectory()

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