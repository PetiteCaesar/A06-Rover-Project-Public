import pygame
import sys
import threading
from typing import Callable, Any, Optional
from roverMaze import Maze 
from engine.rover import Rover
from engine.utils import *
import time
import copy
import numpy as np


def getRotationMatrix(degrees:float):
    rads = np.deg2rad(degrees)
    return np.array([[np.cos(rads), -np.sin(rads)],
                     [np.sin(rads), np.cos(rads)]])

def getMazeLineCoords(maze:Maze, scaleX:float, scaleY:float):    
    walls = []
    
    horizOff = 0
    vertOff = 0
    
    #vertical walls
    for y in range(0, maze.gridHeight):
        for x in range(0, maze.gridWidth+1):
            elem = maze.vertical[y][x]
            if elem == 0: continue
            v1 = [x * maze.gridSize + horizOff, y * maze.gridSize + vertOff]
            v2 = [x * maze.gridSize + horizOff, y * maze.gridSize + maze.gridSize + vertOff]
            if elem == 2: #positive slope
                v1[0] += maze.gridSize
            elif elem == 3: #negative slope
                v2[0] += maze.gridSize
            v1[0] *= scaleX; v2[0] *= scaleX
            v1[1] *= scaleY; v2[1] *= scaleY
            walls.append([v1,v2])
            
            
    for y in range(0, maze.gridWidth+1):
        for x in range(0, maze.gridHeight):
            elem = maze.horizontal[y][x]
            if elem == 0: continue
            v1 = [x * maze.gridSize + horizOff, y * maze.gridSize + vertOff]
            v2 = [x * maze.gridSize + maze.gridSize + horizOff, y * maze.gridSize + vertOff]
            if elem == 2: #positive slope
                v1[1] += maze.gridSize
            elif elem == 3: #negative slope
                v2[1] += maze.gridSize
            v1[0] *= scaleX; v2[0] *= scaleX
            v1[1] *= scaleY; v2[1] *= scaleY
            walls.append([v1,v2])
    
    return walls
        


class Engine:
    def __init__(self,screenSize:tuple[int,int], maze:Maze) -> None:
        pygame.init()
        self.maze = maze
        self.screenWidth = screenSize[0]  
        self.screenHeight = screenSize[1]
        self.__running = True
        self.screen = pygame.display.set_mode((self.screenWidth, self.screenHeight))
        self.clock = pygame.time.Clock()
        self.fps = 60
        pygame.display.set_caption("Rover simulator")
        
        
        realWidth = maze.gridWidth * maze.gridSize
        realHeight = maze.gridHeight * maze.gridSize
        self.scaleX = self.screenWidth / realWidth
        self.scaleY = self.screenHeight / realHeight
        
        self.mazeRealWidth = maze.gridSize * maze.gridWidth
        self.mazeRealHeight = maze.gridSize * maze.gridHeight
        
        self.mazeLineCoords = getMazeLineCoords(self.maze, self.scaleX, self.scaleY)
        self.__logicThread = None
        self.logicThreadRunning = False
        self.lock = threading.Lock()
        self.__deltaTime = -1.0
        self.lastTick = 0.0
        
        self.SENSOR_DISTANCE = 60
        self.__sensorDistance = 4000
        
        self.rover = Rover("Default rover", Vec2(250,250), Vec2(115,115), (255,0,0))

    
    def GetDistance(self) -> float:
        return self.__sensorDistance
    
    
    def MeasureDistance(self) -> float:
        with self.lock:
            pos = copy.deepcopy(self.rover.position)
            pos.x *= self.scaleX
            pos.y *= self.scaleY
            
            size = copy.deepcopy(self.rover.size)
            size.x *= self.scaleX
            size.y *= self.scaleY
            
            roverPos = np.array([pos.x, pos.y])
            
            rotMatrix = getRotationMatrix(self.rover.facing)
            usRotMatrix = getRotationMatrix(self.rover.servoAngle);
            
            points = np.array([[pos.x + size.x/2, pos.y + size.y/2],
                [pos.x, pos.y - size.y/2],
                [pos.x - size.x/2, pos.y + size.y/2]]) - roverPos
            
            rotatedPoints = np.dot(rotMatrix, points.T).T
            rotatedPoints += roverPos
            rotatedPoints = list(map(tuple, rotatedPoints))
            
            sensorLookDir = (np.array(rotatedPoints[1]) - roverPos)
            sensorLookDir = usRotMatrix @ sensorLookDir
            ultraSonicSensorRayEnd = np.array(rotatedPoints[1]) + self.SENSOR_DISTANCE*sensorLookDir
            
            #could do when rendering maze
            distances = []
            for l in self.mazeLineCoords:
                intersect = lineIntersect(rotatedPoints[1], ultraSonicSensorRayEnd, l[0], l[1])
                if intersect[0]:
                    distances.append((intersect[1], intersect[2]))
            
            if len(distances) > 0:
                minDistance = min(distances, key=lambda x: x[1])
                self.__sensorDistance = minDistance[1]
                return self.__sensorDistance
        
        return 4000
    
    def SetRover(self, rover:Rover) -> None:
        self.rover = copy.deepcopy(rover)
    
    def SetSimulationFPS(self, fps:int) -> None:
        if(fps <= 0):
            raise ValueError("FPS must be greater than 0")
        self.fps = fps
    
    def Running(self) -> bool:
        return self.__running
    
    def GetDeltaTime(self) -> float:
        # if(time.time() - self.lastTick == 0 or self.lastTick == 0): return 0.000001
        # return time.time() - self.lastTick
        return self.__deltaTime
    
    def Run(self, task: Callable[..., Any]) -> None:
        if self.logicThreadRunning:
            print("Logic thread already running")
            return
        
        font = pygame.font.SysFont(None, 24)
        self.__running = True
        self.__logicThread = threading.Thread(target=task)
        
        self.__logicThread.start()
        self.logicThreadRunning = True
        
        while self.__running:
            
            self.__deltaTime = self.clock.tick(self.fps)
            self.lastTick = time.time()
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.__running = False


            self.screen.fill((0, 0, 0))

            fps = self.clock.get_fps()
            fps_text = font.render(f"FPS: {fps:.2f}", True, pygame.Color('white'))
            self.screen.blit(fps_text, (10, 10))  # Top-left corner
            
           
            
            #draw grid
            for i in range(0,5):
                hOff = 0
                p = np.array([[0,self.maze.gridSize * i], [self.maze.gridWidth * self.maze.gridSize, self.maze.gridSize * i],
                              [self.maze.gridSize * i + hOff,0], [self.maze.gridSize * i + hOff, self.maze.gridSize * self.maze.gridHeight]], dtype=np.float64)
                p *= np.array([self.scaleX,self.scaleY])
                pygame.draw.line(self.screen, (50, 50, 50), p[0],p[1])
                pygame.draw.line(self.screen, (50, 50, 50), p[2],p[3])
                
            #draw maze
            for l in self.mazeLineCoords:
                pygame.draw.line(self.screen, (255, 255, 255), l[0],l[1], 3)
            
            
            #draw rover (triangle)
            # should move drawing outside of with
            with self.lock:
                pos = copy.deepcopy(self.rover.position)
                pos.x *= self.scaleX
                pos.y *= self.scaleY
                
                size = copy.deepcopy(self.rover.size)
                size.x *= self.scaleX
                size.y *= self.scaleY
                
                roverPos = np.array([pos.x, pos.y])
                
                rotMatrix = getRotationMatrix(self.rover.facing)
                usRotMatrix = getRotationMatrix(self.rover.servoAngle);
                
                points = np.array([[pos.x + size.x/2, pos.y + size.y/2],
                    [pos.x, pos.y - size.y/2],
                    [pos.x - size.x/2, pos.y + size.y/2]]) - roverPos
                
                rotatedPoints = np.dot(rotMatrix, points.T).T
                rotatedPoints += roverPos
                rotatedPoints = list(map(tuple, rotatedPoints))
                
                sensorLookDir = (np.array(rotatedPoints[1]) - roverPos)
                sensorLookDir = usRotMatrix @ sensorLookDir
                ultraSonicSensorRayEnd = np.array(rotatedPoints[1]) + self.SENSOR_DISTANCE*sensorLookDir
                
                #could do when rendering maze
                distances = []
                for l in self.mazeLineCoords:
                    intersect = lineIntersect(rotatedPoints[1], ultraSonicSensorRayEnd, l[0], l[1])
                    if intersect[0]:
                        distances.append((intersect[1], intersect[2]))
                        pygame.draw.circle(self.screen,(255,0,255), intersect[1].AsNumpy(), 4)
                
                if len(distances) > 0:
                    minDistance = min(distances, key=lambda x: x[1])
                    self.__sensorDistance = minDistance[1]
                    pygame.draw.circle(self.screen,(0,0,255), minDistance[0].AsNumpy(), 5)
                    
                pygame.draw.polygon(self.screen, self.rover.colour, rotatedPoints)
                
                pygame.draw.line(self.screen, (0, 255, 0), rotatedPoints[1], ultraSonicSensorRayEnd, 1)
                
                #nose of the rover
                pygame.draw.circle(self.screen, (255, 255, 255), rotatedPoints[1], 2)
                pygame.draw.circle(self.screen, (0, 255, 255), roverPos, 2)
                
            pygame.display.flip()
            
        print("Exiting main loop")
        
        self.__logicThread.join()
        self.logicThreadRunning = False
        print("Logic thread joined")
        
        pygame.quit()
        print("Exited safely")
        sys.exit()
