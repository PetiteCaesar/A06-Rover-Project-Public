

# 5 rows 5 cols, 4x4 grid
# 0 = empty space, 1 = wall, 2 is a angled wall with a positive slope 
# 3 is a angled wall with a negative slope )
#imaging each ',' in the list is a grid spot and the number is the walls around it
class Maze:
    def __init__(self):
        self.w = 4
        #vertical Walls
        self.vertical = [
            [1,1,0,0,1],
            [1,1,1,0,1],
            [1,1,3,1,1], #since "3,1" this means that the space between the slope and wall is not free
            [1,0,2,1,1]
        ]
        #horizontal walls
        self.horizontal = [
            [0,1,1,1],
            [0,0,0,1],
            [0,0,1,0],
            [0,0,0,0],
            [1,1,1,0],
        ]
        
        
        # self.vertical = [
        #     [1,1,0,0,1],
        #     [1,1,1,0,1],
        #     [1,1,1,1,1], #since "3,1" this means that the space between the slope and wall is not free
        #     [1,0,1,1,1]
        # ]
        # #horizontal walls
        # self.horizontal = [
        #     [0,1,1,1],
        #     [0,0,0,1],
        #     [0,0,1,0],
        #     [0,0,0,0],
        #     [1,1,1,0],
        # ]
        
        # self.vertical = [
        #     [1,0,0,0,0],
        #     [1,0,1,0,0],
        #     [1,1,0,1,0],
        #     [1,3,0,0,1]
        # ]
        # #horizontal walls
        # self.horizontal = [
        #     [1,1,1,1],
        #     [0,1,1,1],
        #     [0,0,1,0],
        #     [0,0,0,1],
        #     [1,1,1,0],
        # ]
        
        #empty
        # self.vertical = [
        #     [0,0,0,0,0],
        #     [0,0,0,0,0],
        #     [0,0,0,0,0],
        #     [0,0,0,0,0],
        # ]
        # #horizontal walls
        # self.horizontal = [
        #     [0,0,0,0],
        #     [0,0,0,0],
        #     [0,0,0,0],
        #     [0,0,0,0],
        #     [0,0,0,0],
        # ]
          
        self.gridSize = 195 #mm
        self.gridWidth = 4
        self.gridHeight = 4


defaultMaze = Maze()

