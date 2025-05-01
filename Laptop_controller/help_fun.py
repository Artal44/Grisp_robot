def compute_current_size(width, height, past_height, current_height,  past_resize, current_resize):
   new_width, new_height = int(width / (past_height//past_resize)), int(height / (past_height//past_resize))
   new_width, new_height = int(new_width * (current_height//current_resize)), int(new_height * (current_height//current_resize))
   return new_width, new_height