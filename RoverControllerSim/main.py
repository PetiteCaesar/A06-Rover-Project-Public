from engine.engine import Engine
from roverMaze import defaultMaze
from engine.utils import Vec2
import time

engine = Engine((800, 600), maze=defaultMaze)
engine.SetSimulationFPS(120000)



class Logic:
    def __init__(self):
        self.sDir = 1
        
    def Start(self):
        with engine.lock:
            engine.rover.position.x = engine.mazeRealWidth - engine.maze.gridSize
            engine.rover.position.y = engine.mazeRealHeight - engine.maze.gridSize/2
    
    def Loop(self):
        with engine.lock:
            engine.rover.facing += 0.05 * engine.GetDeltaTime() # rotate rover to the right
            if(engine.rover.facing > 360):
                engine.rover.facing = 0
            engine.rover.servoAngle += 0.2 * engine.GetDeltaTime() * self.sDir
            if(engine.rover.servoAngle > 90 or engine.rover.servoAngle < -90):
                self.sDir *=-1

   

logic = Logic()


def run():
        logic.Start()
    
        while engine.Running():
            s = time.time()
            logic.Loop()
            
            delay = 1/engine.fps - (time.time() - s)
            if delay > 0:
                time.sleep(delay)

engine.Run(run)    