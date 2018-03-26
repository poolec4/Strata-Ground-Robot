import numpy as np
mport random
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap




class world:
    def __init__(self, gridSize, numObjects, maxSize):
        self.grid = self.generate(gridSize, numObjects, maxSize)

    def generate(self, gridSize, numObjects, maxSize):
        grid = np.zeros(gridSize)
        for i in range(0, numObjects):
            center = [random.randint(0.1*gridSize[0], 0.9*gridSize[0]), random.randint(0.1*gridSize[1], 0.9*gridSize[1])]
            # center = [random.randint(0.2*gridSize[0], 0.8*gridSize[0]), random.randint(0.2*gridSize[1], 0.8*gridSize[1])]
            size = random.randint(0, maxSize)
            grid[center[0], center[1]] = 1
            loc = center
            for j in range(0, size):
                index = random.randint(0, 3)
                loc, grid = self.growObject(grid, loc, index, gridSize)

        return grid


    def growObject(self, grid, location, index, gridSize):
        if index == 0: # Up
            location[1] = location[1]+1
        elif index == 1: # Right
            location[0] = location[0]+1
        elif index == 2: # Down
            location[1] = location[1]-1
        elif index == 3: # Left
            location[0] = location[0]-1
        if location[0] < gridSize[0] and location[1] < gridSize[1]:
            grid[location[0], location[1]] = 1

        return location, grid


class robot:
    def __init__(self, grid, gridSize):
        self.position = self.init_position(grid, gridSize)
        self.goal = self.goal_position(grid, gridSize, self.Position)

    def init_position(self, grid, gridSize):
        valid = False
        while valid == False:
            temp_pos = [random.randint(0.1*gridSize[0], 0.9*gridSize[0]), random.randint(0.1*gridSize[1], 0.9*gridSize[1])]
            if grid[temp_pos[0], temp_pos[1]] == 0:
                valid = True
                position = temp_pos

        return position

    def goal_position(self, grid, gridSize, startPosition):
        valid = False
        while valid == False:
            temp_goal = [random.randint(0.1*gridSize[0], 0.9*gridSize[0]), random.randint(0.1*gridSize[1], 0.9*gridSize[1])]
            if grid[temp_pos[0], temp_pos[1]] == 0 and temp_goal[0] != startPosition[0] and temp_goal[1] != startPosition[1]:
                valid = True
                goal = temp_goal

        return goal

class search: # A* Search (Greedy)
    def __init__(start):
        self.open = start
        self.closed = []
        self.cameFrom = []

    def heuristic(a, b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    def a_star(grid, start, goal):


class render:
