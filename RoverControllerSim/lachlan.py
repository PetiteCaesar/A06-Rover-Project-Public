from engine.engine import Engine
from roverMaze import defaultMaze
from engine.utils import Vec2
import time
import numpy as np
import functools
import operator
import sys
import math

#I was testing a different algorithm for the rover that never ended up being used
#also it was way to complex, and I should have spent my time focusing on implementing the easier (hand hug) 
#algorithm on the rover itself, though, this was still fun to do


engine = Engine((800, 800), maze=defaultMaze)
engine.SetSimulationFPS(240)


class Cell:
    def __init__(self,u,d,l,r):
        self.up = u
        self.down = d
        self.left = l
        self.right = r
        
    def __repr__(self):
        return f"Cell({self.up},{self.down},{self.left},{self.right})"
#two elements per cell horiz, 4 vertical        
#top left to bottom right (x,y)
c00 = Cell(0,0,1,1)
c10 = Cell(1,0,1,0)
c20 = Cell(1,0,0,0)
c30 = Cell(1,1,0,1)

c01 = Cell(0,0,1,1)
c11 = Cell(0,0,1,1)
c21 = Cell(0,1,1,0)
c31 = Cell(1,0,0,1)

c02 = Cell(0,0,1,1)
c12 = Cell(0,0,1,3)
c22 = Cell(1,0,3,1)#as this cell has a negative wall in it
c32 = Cell(0,0,1,1)

c03 = Cell(0,1,1,0)
c13 = Cell(0,1,0,2)
c23 = Cell(0,1,2,1)
c33 = Cell(0,0,1,1)


maze = [
    [c00,c10,c20,c30],
    [c01,c11,c21,c31],
    [c02,c12,c22,c32],
    [c03,c13,c23,c33],
]

def getMazeList(_maze:list[list[Cell]]):
    nm = []
    for i,y in enumerate(_maze):
        r1 = []
        r2 = []
        r3 = []
        ix = 0
        for ix, x in enumerate(y):
            if ix == 0:
                r1.append([5,x.up,5])
                r3.append([5,x.down,5])
                r2.append([x.left,0,x.right])
            else:
                r1.append([x.up,5])
                r3.append([x.down,5])
                r2.append([0,x.right])
            
        
        if i == 0:
            r1 = functools.reduce(operator.iconcat, r1, [])
            nm.append(r1)
        r2 = functools.reduce(operator.iconcat, r2, [])
        nm.append(r2)
        
        r3 = functools.reduce(operator.iconcat, r3, [])
        nm.append(r3)
        
        
    return nm

# print(getMazeList(maze))
#getMazeList(maze)



temp__= [
    [5, 0, 5, 5, 1, 5, 5, 1, 5, 5, 1, 5], #up
    [1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1], 
    [5, 0, 5, 5, 0, 5, 5, 0, 5, 5, 1, 5], #down 2
    [5, 0, 5, 5, 0, 5, 5, 0, 5, 5, 1, 5], #up
    [1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1], 
    [5, 0, 5, 5, 0, 5, 5, 1, 5, 5, 0, 5], #down 5
    [5, 0, 5, 5, 0, 5, 5, 1, 5, 5, 0, 5], #up
    [1, 0, 1, 1, 0, 0, 3, 0, 1, 1, 0, 1], 
    [5, 0, 5, 5, 0, 5, 5, 0, 5, 5, 0, 5], #down 8
    [5, 0, 5, 5, 0, 5, 5, 0, 5, 5, 0, 5], #up
    [1, 0, 0, 0, 0, 0, 2, 0, 1, 1, 0, 1], 
    [5, 1, 5, 5, 1, 5, 5, 1, 5, 5, 0, 5] # down
]

#final representation before A*
temp2_ = [
    [5, 0, 5, 1, 5, 1, 5, 1, 5], 
    [1, 0, 1, 0, 0, 0, 0, 0, 1], 
    [5, 0, 5, 0, 5, 0, 5, 1, 5], 
    [1, 0, 1, 0, 1, 0, 0, 0, 1], 
    [5, 0, 5, 0, 5, 1, 5, 0, 5], 
    [1, 0, 1, 0, 3, 0, 1, 0, 1], 
    [5, 0, 5, 0, 5, 0, 5, 0, 5], 
    [1, 0, 0, 0, 2, 0, 1, 0, 1], 
    [5, 1, 5, 1, 5, 1, 5, 0, 5]
]

    
class Node:
    def __init__(self, position, cost, heuristic):
        self.position = position
        self.cost = cost
        self.heuristic = heuristic
        self.parent = None

    def total_cost(self):
        return self.cost + self.heuristic

#https://www.alps.academy/a-star-algorithm-python/
def astar_search(graph, start, goal):
    open_set = []
    closed_set = set()

    start_node = Node(start, 0, heuristic(start, goal))
    open_set.append(start_node)

    while open_set:
        # Find the node in open_set with the lowest total cost (f = g + h)
        current_node = min(open_set, key=lambda node: node.total_cost())
        open_set.remove(current_node)

        if current_node.position == goal:
            # Path found, reconstruct and return it
            path = []
            while current_node:
                path.insert(0, current_node.position)
                current_node = current_node.parent
            return path

        closed_set.add(current_node.position)

        for neighbor in graph[current_node.position]:
            if neighbor in closed_set:
                continue

            cost = current_node.cost + graph[current_node.position][neighbor]
            heuristic_val = heuristic(neighbor, goal)
            new_node = Node(neighbor, cost, heuristic_val)
            new_node.parent = current_node

            # Check if a better node for this position is already in open_set
            existing_node = next((node for node in open_set if node.position == neighbor), None)
            if existing_node:
                if existing_node.cost <= cost:
                    continue
                else:
                    open_set.remove(existing_node)

            open_set.append(new_node)

    return None  # No path found

#https://www.alps.academy/a-star-algorithm-python/
def heuristic(node, goal):
    # Manhattan distance
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

#https://www.alps.academy/a-star-algorithm-python/
def createGraph(maze, wallPositions):
    graph = {}
    height = len(maze)
    width = len(maze[0])
    for y in range (0, height):
        for x in range(0, width):
            if (x,y) not in wallPositions:
                neighbours = {}
                if x > 0 and (x - 1, y) not in wallPositions:
                    neighbours[(x - 1, y)] = 1
                if y > 0 and (x, y - 1) not in wallPositions:
                    neighbours[(x, y - 1)] = 1
                if x < width - 1 and (x + 1, y) not in wallPositions:
                    neighbours[(x + 1, y)] = 1
                if y < height - 1 and (x, y + 1) not in wallPositions:
                    neighbours[(x, y + 1)] = 1
                graph[(x, y)] = neighbours
    return graph


def wallPositions(maze):
    walls = []
    for iy, y in enumerate(maze):
        for ix, x in enumerate(y):     
            if x != 0: walls.append((ix,iy))
    return walls

def coordToMazeCoord(coord:tuple[int, int]):
    return (max(0,math.ceil(coord[0]/2 - 1)),max(0,math.ceil(coord[1]/2 - 1)))

def mazeCoordToCoord(coord:tuple[int,int]):
    return (coord[0] * 2 + 1, coord[1] * 2 + 1)

graph = createGraph(temp2_, wallPositions(temp2_))
print(list(dict.fromkeys(map(coordToMazeCoord, astar_search(graph, (7,8),(1,0))))))




class Logic:
    def __init__(self):
        self.sDir = 1
        self.lastDistance = 0
        #as 9x9 for one maze
        self.maze = [[1 for x in range(0,9)] for y in range(0,9)]
        self.currentPos = (7,7)
        self.referencePos = (7,7)
        self.maze[self.currentPos[1]][self.currentPos[0]] = 0
        print(np.array(self.maze))
        print(f"Size in bytes = {sum(map(lambda x: len(x),self.maze))*4}")
        
        self.prevDestinations = [self.currentPos]
        
        
        
        
        
    def Start(self):
        with engine.lock:
            engine.rover.position.x = engine.mazeRealWidth - engine.maze.gridSize/2
            engine.rover.position.y = engine.mazeRealHeight - engine.maze.gridSize/2
            # engine.rover.position.x = engine.mazeRealWidth - 7*engine.maze.gridSize/2
            # engine.rover.position.y = engine.mazeRealHeight - 7*engine.maze.gridSize/2
            engine.rover.facing = 0
    
    def Forward(self):
        rads = np.radians(engine.rover.facing - 90)
        dir = Vec2(np.cos(rads), np.sin(rads))
        speed = 0.5
        with engine.lock:
            engine.rover.position += dir * engine.GetDeltaTime() * speed

    def MoveDistance(self, distance):
        rads = np.radians(engine.rover.facing - 90)
        dir = Vec2(np.cos(rads), np.sin(rads))
        with engine.lock:
            engine.rover.position += dir * distance
    
    
    def Algo1(self):
        self.Forward()
        
        distance = engine.MeasureDistance()
        # print(f"Initial distance {distance}")
        delay = 0.5
        if distance < 30:
            time.sleep(delay)
            with engine.lock:
                engine.rover.servoAngle = -90
                
            distance = engine.MeasureDistance()
            print(f"Left distance {distance}")
            time.sleep(delay)
            
            #slope wall if distance is > 30 < 30 + self.gridSize
                
            #works on known maze, in a perfect world with perfect sensors and movement
            if distance < 30*3 or (distance > 30 and distance < engine.maze.gridSize):
                with engine.lock:
                    engine.rover.servoAngle = 90
                distance = engine.MeasureDistance()
                print(f"Right distance {distance}")
                time.sleep(delay)
                if distance < 30*3 or (distance > 30 and distance < engine.maze.gridSize):
                    with engine.lock:
                        engine.rover.facing -= 180
                else:
                    with engine.lock:
                        engine.rover.facing += 90
            else:
                with engine.lock:
                    engine.rover.facing -= 90
            with engine.lock:        
                engine.rover.servoAngle = 0
            time.sleep(delay)
            # print("distance less than 30")

            # time.sleep(0.5)


    #only kinda works when starting at left hand corner
    # doesnt know when it has ended
    def Algo2(self):
        delay = 0.05
        with engine.lock:
            engine.rover.servoAngle = -90
        distanceLeft = engine.MeasureDistance()
        # print(f"Left distance {distanceLeft}")
        time.sleep(delay)
        with engine.lock:
            engine.rover.servoAngle = 0
        distanceForward = engine.MeasureDistance()
        # print(f"Forward distance {distanceForward}")
        time.sleep(delay)
        with engine.lock:
            engine.rover.servoAngle = 90
        distanceRight = engine.MeasureDistance()
        # print(f"Right distance {distanceRight}")
        time.sleep(delay)
        #all relative
        up = 1 if (distanceForward < engine.maze.gridSize) else 0
        left = 1 if (distanceLeft < engine.maze.gridSize) else 0
        right = 1 if (distanceRight < engine.maze.gridSize) else 0
        
        # print(Cell(up,0,left,right))#for printing
        
            
        print(engine.rover.facing)
        print(f"Rover position before setting maze square {self.currentPos}")
        if(engine.rover.facing == 0): #forward 
            if self.currentPos[0] % 2 != 0:
                self.maze[self.currentPos[1]][self.currentPos[0] - 1] = left
                if left == 0:
                    self.maze[self.currentPos[1]][self.currentPos[0] - 2] = 0
                if self.currentPos[0] != 7:
                    self.maze[self.currentPos[1]][self.currentPos[0] + 1] = right
                    if right == 0:
                        self.maze[self.currentPos[1]][self.currentPos[0] + 2] = 0
                 
            self.maze[self.currentPos[1] - 1][self.currentPos[0]] = up
            if up == 0:
                self.maze[self.currentPos[1] - 2][self.currentPos[0]] = 0
        elif(engine.rover.facing == 90): #right
            
            if self.currentPos[0] % 2 != 0:
                if self.currentPos[0] != 7:
                    self.maze[self.currentPos[1]][self.currentPos[0] + 1] = up#right is now up
                    if up == 0:
                        self.maze[self.currentPos[1]][self.currentPos[0] + 2] = 0
                 
            self.maze[self.currentPos[1] - 1][self.currentPos[0]] = left #up now left
            if left == 0:
                self.maze[self.currentPos[1] - 2][self.currentPos[0]] = 0
            
            self.maze[self.currentPos[1] + 1][self.currentPos[0]] = right #down is now right
            if right == 0:
                self.maze[self.currentPos[1] + 2][self.currentPos[0]] = 0
                
        elif(engine.rover.facing == 180): # down
            if self.currentPos[0] % 2 != 0:
                self.maze[self.currentPos[1]][self.currentPos[0] - 1] = right
                if right == 0:
                    self.maze[self.currentPos[1]][self.currentPos[0] - 2] = 0
                if self.currentPos[0] != 7:
                    self.maze[self.currentPos[1]][self.currentPos[0] + 1] = left
                    if left == 0:
                        self.maze[self.currentPos[1]][self.currentPos[0] + 2] = 0
                 
            self.maze[self.currentPos[1] + 1][self.currentPos[0]] = up
            if up == 0:
                self.maze[self.currentPos[1] + 2][self.currentPos[0]] = 0
                
        elif(engine.rover.facing == 270): #left
            # print(f"Maze before")
            # print(np.array(self.maze))
            # print(f"Self.currentPos {self.currentPos}")
            if self.currentPos[0] % 2 != 0:
                self.maze[self.currentPos[1]][self.currentPos[0] - 1] = up
                if up == 0:
                    self.maze[self.currentPos[1]][self.currentPos[0] - 2] = 0
                #no left
                 
            self.maze[self.currentPos[1] - 1][self.currentPos[0]] = right #up now left
            if right == 0:
                self.maze[self.currentPos[1] - 2][self.currentPos[0]] = 0
                
            self.maze[self.currentPos[1] + 1][self.currentPos[0]] = left #down is now right
            if left == 0:
                self.maze[self.currentPos[1] + 2][self.currentPos[0]] = 0
            # print(f"Maze after")
            # print(np.array(self.maze))
            
        print("Maze after looking 👀")
        print(np.array(self.maze))
        print(f"Maze after looking position: {self.currentPos}")
        
        #get destination
        possible = []
        for iy, y in enumerate(self.maze):
            for ix, x in enumerate(y):     
                if x == 0: possible.append((ix,iy))
        diff = list(set(possible) ^ set(self.prevDestinations))
        print(f"Possible {possible}")
        print(f"prev destinations {self.prevDestinations}")
        if len(diff) == 0: 
            raise Exception("There are no possible destinations")
        destination = diff[0]
        
        print(f"Destination: {destination}")
        graph = createGraph(self.maze, wallPositions(self.maze))
        # movements = list(dict.fromkeys(map(coordToMazeCoord, astar_search(graph, self.currentPos,destination))))
        movements = astar_search(graph, self.currentPos,destination)
        print(movements)
        movementsReal = list(dict.fromkeys(map(coordToMazeCoord, movements)))
        print(f"Movements real {movementsReal}")
        
        #now move
        for m in movementsReal:
            currentPos = Vec2(coordToMazeCoord(self.currentPos))
            direction = Vec2(m) - currentPos
            print(f"direction: {direction}")
            if(direction.x == 0 and direction.y == 0): continue
            with engine.lock:
                # engine.rover.facing = math.atan2(direction.y,direction.x)
                engine.rover.facing = direction.angleDegrees() + 90
            self.MoveDistance(engine.maze.gridSize)
            
            #here to do deviation correction, could first check has left and right deviation
            #then do what am already doing
            
            # with engine.lock:
            #     engine.rover.position += direction * engine.maze.gridSize
            self.currentPos = mazeCoordToCoord((currentPos + direction).AsTuple())
            self.maze[self.currentPos[1]][self.currentPos[0]] = 0
            print(f"new current pos{self.currentPos}")
            time.sleep(delay)

        self.prevDestinations.append(destination)
        # if(len(movementsReal) > 1):
        #     self.prevDestinations.append(destination)
        
       
        # #temp
        # self.MoveDistance(engine.maze.gridSize)
        # self.currentPos = (self.currentPos[0], self.currentPos[1]-2)
        # time.sleep(delay)
        
    
    def Loop(self):
        #self.Algo1()
        self.Algo2()

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