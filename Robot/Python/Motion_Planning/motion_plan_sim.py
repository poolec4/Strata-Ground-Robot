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

class aStar: # A* Search
    def __init__(start, gridSize, numObjects, maxSize):
        self.open = open(start)
        self.closed = closed()
        self.cameFrom = cameFrom()
        self.f = 10e6
        self.g = 0
        self.h = 10e6
        self.grid = world(gridSize, numObjects, maxSize)

    def search(self, start, goal):
        while self.open.location != goal:
            current = self.open[self.minF()] # Select current location
            self.closed.add(current, self.grid) # Add current to closed 
            self.open.remove(current, self.grid) # Remove current from open
            self.h = self.heuristic(current, goal) # Calculate predicted fugure cost
            self.g = self.cameFrom.getValue(current) + self.grid[current[0], current[1]] # calculate cost to get to current (this is incorrect - only checking last cameFrom, should check all)
            self.f = self.h + self.g # Calculate f
            self.open = self.open.findNeighbors(current) # Find neighbors
            

    def heuristic(current, goal):
        (x1, y1) = current
        (x2, y2) = goal
        return abs(x1 - x2) + abs(y1 - y2)

    def cost():

        return g

    def minF(self):
        ind = np.argmin(np.asarray(self.open.value))
        return ind


class open:
    def __init__(self, start, grid):
        self.locations = start
        self.values = grid[start[0], start[1]]

    def add(self, neighbor):
        self.locations.append(current)
        self.values.append(grid[current[0], current[1])
        return self

    def remove(self, current, grid):
        self.locations.remove(current)
        self.values.remove(grid[current[0], current[1]])
        return self

    def findNeighbors(self, current):
        for i in (-1, 0, 1):
            for j in range(-1, 0, 1):
                if grid[current[0]+i, current[1]+j] not in open:
                    self.open.locations.append([current[0]+i, current[1]+j])
                    self.open.values.append(grid[current[0]+i, current[1]+j])
        return self
                


class closed:
    def __init__(self):
        self.locations = []
        self.values = []
        
    def add(self, current, grid):
        self.locations.append(current)
        self.values.append(grid[current[0], current[1]])
        return self
                           

class cameFrom:
    def __init__(self, gridSize):
        self.locations = np.empty(gridSize[0], gridSize[1])
    
    def update(self, current, previous, grid):
        self.locations[current[0], current[1]] = previous
        return self

    def getValue(self, current, closed):
        cameFrom = self.locations[current[0], current[1]]
        ind = cameFrom in closed.locations
        if not ind:
            raise ValueError('Could not find cameFrom in closed.locations')
        value = closed.values[ind]
        return value
        
        

class render:
