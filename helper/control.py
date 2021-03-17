import numpy as np

class Robot_movement:

    def __init__(self, starting_x, starting_y):
        self.x = starting_x
        self.y = starting_y
        self.noise = 0.1
        self.x_path = np.array([starting_x])
        self.y_path = np.array([starting_y])

    def get_position(self):
        return self.x,self.y
    
    def get_path(self):
        return self.x_path,self.y_path

    def take_step(self,commanded_x,commanded_y):
        self.x += commanded_x+np.random.random()*self.noise
        self.y += commanded_y+np.random.random()*self.noise
        self.x_path = np.append(self.x_path,self.x)
        self.y_path = np.append(self.y_path,self.y)





