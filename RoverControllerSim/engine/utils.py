import numpy as np
class Vec2:
    def __init__(self, x = 0, y = 0):
        if isinstance(x, tuple):
            self.x, self.y = x
        else:
            self.x = x
            self.y = y
    
    def AsNumpy(self):
        return np.array([self.x, self.y])
    
    def angle(self):
        if self.x == 0 and self.y == 0:
            return None
        return np.atan2(self.y, self.x)

    def angleDegrees(self):
        angle_rad = self.angle()
        if angle_rad is None:
            return None
        return np.degrees(angle_rad)
    
    def AsTuple(self):
        return (self.x, self.y)
    
    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar:int):
        return Vec2(self.x * scalar, self.y * scalar)
    
    def __truediv__(self, scalar:int):
        return Vec2(self.x / scalar, self.y / scalar)
    
    def __repr__(self):
        return f"Vec2({self.x}, {self.y})"
    
    def __str__(self):
        return f"Vec2({self.x}, {self.y})"
    

#for the line intersection calcs as lines arent stored as vectors
def cross(v1, v2):
    return v1[0]*v2[1] - v1[1]*v2[0]

def subtract(p1, p2):
    return (p1[0] - p2[0], p1[1] - p2[1])

#returns (did intersect?, where at, how far)
def lineIntersect(a1, a2, b1, b2) -> tuple[bool,Vec2,float]:
    r = subtract(a2, a1)
    s = subtract(b2, b1)
    rxS = cross(r,s)
    
    if rxS == 0:
        return (False,Vec2(0,0), 0.0)
    
    t = cross(subtract(b1, a1), s) / rxS  
    u = cross(subtract(b1, a1), r) / rxS

    if 0 <= t <= 1 and 0 <= u <= 1:
        intersection = Vec2(a1[0] + t*r[0], a1[1] + t*r[1])
        dist = np.hypot(intersection.x - a1[0], intersection.y - a1[1])
        return (True, intersection, dist)
    
    return (False,Vec2(0,0), 0.0)
    
   