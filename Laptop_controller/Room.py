from Side import Side
import help_fun

class Room:

    def __init__(self, width, height, x, y, room_num):
        self.width = width
        self.height = height
        self.pos = (x,y)
        self.sides = {
            "L":None,
            "R":None,
            "T":None,
            "B":None
                    }
        self.corners = {
            "TL":None,
            "TR":None,
            "BL":None,
            "BR":None
        }
        self.room_num = room_num
        
    def modify_side(self, side, img, img_type):
        x, y = self.compute_pos(side)
        if side in self.sides.keys() :            
            self.sides[side] = Side(x, y, img, img_type)
        else :
            self.corners[side] = Side(x, y, img, img_type)

    def compute_pos(self, side):
        match side :
            case "L":
                side_x, side_y = self.pos[0] - self.width//2, self.pos[1]  
            case "R":
                side_x, side_y = self.pos[0] + self.width//2, self.pos[1]  
            case "T":
                side_x, side_y = self.pos[0], self.pos[1] + self.height//2
            case "B":
                side_x, side_y = self.pos[0], self.pos[1] - self.height//2
            case "TL":
                side_x, side_y = self.pos[0] - self.width//2, self.pos[1] + self.height//2
            case "TR":
                side_x, side_y = self.pos[0] + self.width//2, self.pos[1] + self.height//2
            case "BL":
                side_x, side_y = self.pos[0] - self.width//2, self.pos[1] - self.height//2
            case "BR":
                side_x, side_y = self.pos[0] + self.width//2, self.pos[1] - self.height//2

        return side_x, side_y

        
    def add_room(self, side, room):
        self.sides[side] = room

    def update_size(self, resize, new_resize, height):
        self.width, self.height = help_fun.compute_current_size(self.width, self.height, height, height, resize, new_resize)
        for side in ["L", "R", "T", "B"]:
            x, y = self.compute_pos(side)
            self.sides[side].pos = (x,y)


    

