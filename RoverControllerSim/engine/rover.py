from engine.utils import *
class Rover:
    def __init__(self, name:str, startingPos:Vec2, size:Vec2, 
                 colour:tuple[int,int,int] = (255,0,0)):
        """Starting pos is the center of the rover, size is the size of the rover in mm"""
        
        self.name = name
        #center of rover
        self.position = startingPos
        self.size = size
        self.colour = colour
        self.facing = 0
        self.servoAngle = 0 #-90 to 90 degrees
        
        