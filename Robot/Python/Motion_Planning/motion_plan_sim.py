import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import time

# Grid Number Key
# 0: Open
# 1: Obstructed
# 2: Robot
# 3: Goal

class world:
    def __init__(self, gridSize, numObjects, maxSize):
        self.grid = self.generate(gridSize, numObjects, maxSize)

    def generate(self, gridSize, numObjects, maxSize):
        grid = np.zeros(gridSize)
        for i in range(0, numObjects):
            center = [random.randint(0.1*gridSize[0], 0.9*gridSize[0]), random.randint(0.2*gridSize[1], 0.9*gridSize[1])]
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
        self.start = self.init_position(grid, gridSize)
        self.goal = self.goal_position(grid, gridSize, self.start)

    def init_position(self, grid, gridSize):
        position = [gridSize[0]/2, 0]
        return position

    def goal_position(self, grid, gridSize, startPosition):
        goal = [random.randint(0.1*gridSize[0], 0.9*gridSize[0]), (gridSize[1]-1)]
        return goal

class myLazySearch:
    def __init__(self, start, grid, gridSize):
        self.grid = grid
        self.gridSize = gridSize
        self.start = start

    def search(self, start, goal):
        self.goal = goal
        self.path = []
        self.path.append(start)
        current = start
        while current[1] != goal[1]:
            dist = self.getDists(current)
            old_pos = current
            current = self.updatePosition(dist, current, goal)
            print('Current: ', current)
            self.path = self.addPath(dist, current, old_pos)
        self.showPath()
        return self

    def getDists(self, current):
        dist = []
        for i in range(self.gridSize[0]):
            end = False
            j = 0
            while end == False:
                if j+current[1] == self.gridSize[1]-1 or self.grid[i, j+current[1]] != 0:
                    dist.append(j+current[1])
                    end = True
                    break
                j += 1
        return dist

    def updatePosition(self, dist, current, goal):
        maxDist = 0
        lateral = 10
        for i in range(gridSize[0]):
            if dist[i] > maxDist:
                maxInd = i
                maxDist = dist[i]
                lateral = abs(i-goal[0])
            elif dist[i] == maxDist and abs(i-goal[0]) < lateral:
                maxInd = i
                maxDist = dist[i]
                lateral = abs(i-goal[0])

        current = [maxInd, current[1]+maxDist]
        return current

    def addPath(self, dist, new, old):
        current = old
        mag = self.planPath(new, old)
        print(mag)
        i = 0
        j = 0
        while(current[1] != new[1]):
            if current[0] < new[0]:
                i = int(round(1.1*mag))
            elif current[0] > new [0]:
                i = int(round(-1.1*mag))
            else:
                i = 0
            if current[1] < new[1]:
                j = 1
            else:
                j = 0
            current = [current[0]+i, current[1]+j]
            self.path.append(current)
            if current[0] == new[0] and current[1] == new[1]:
                break

        return self.path

    def planPath(self, new, old):
        run = True
        angles = []
        if old[0] > new[0]:
            xvals = list(range(new[0], old[0]))
        else:
            xvals = list(range(old[0], new[0]))
        for i in xvals:
            for j in range(self.gridSize[1]):
                current = self.grid[i, j]
                if current == 1:
                    break

            angles.append(j/i)

        return max(angles)


    def showPath(self):
        plotter = plot()
        print(self.path)
        for i in range(len(self.path)):
            self.grid[self.path[i][0], self.path[i][1]] = 2
            plotter.render(self.grid)
            self.grid[self.path[i][0], self.path[i][1]] = 0



class plot:
    def __init__(self):
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.cmap = ListedColormap(['w', 'k', 'r'])

    def render(self, grid):
        self.ax.cla()
        self.ax.matshow(grid, cmap=self.cmap)
        plt.draw()
        plt.pause(0.0000001)



class aStar: # A* Search
    def __init__(self, start, grid, gridSize):
        self.open = opn(start, grid)
        self.closed = closed()
        self.cameFrom = cameFrom(gridSize, start)
        self.f = 10e6
        self.g = 0
        self.h = 10e6
        self.grid = grid
        self.gridSize = gridSize
        self.start = start

    def search(self, start, goal):
        current = self.open.locations[0]
        self.grid[goal[0]][goal[1]] = 3
        count = 0
        while current != goal:
            if count > 0:
                current = self.open.locations[self.minF()] # Select current location
                count = 1
            print('Current:')
            print(current)
            self.grid[current[0], current[1]] = 2 # Update grid with robot's position
            print(self.grid)
            self.open = self.open.findNeighbors(current, self.grid, self.gridSize) # Find neighbors
            self.closed.add(current, self.grid) # Add current to closed
            self.open.remove(current, self.grid) # Remove current from open
            self.cost = self.heuristic(current, goal) + self.cameFrom.getValue(current, self.closed) + self.grid[current[0], current[1]]
            self.grid[current[0], current[1]] = 0 # Remove robot's position before updating
            print('Open: ')
            print(self.open.locations)
            print('Closed: ')
            print(self.closed.locations)
            print(len(self.open.values))
            for i in range(len(self.open.values)):
                # Calculate Cost of Moving to Each Neighbor in Open
                h = self.heuristic(self.open.locations[i], goal) # Calculate predicted fugure cost
                # if
                g = self.cameFrom.getValue(self.open.locations[i][0], self.closed, previous=current) + self.grid[self.open.locations[i][0], self.open.locations[i][1]] # calculate cost to get to current (this is incorrect - only checking last cameFrom, should check all)
                f = h + g # Calculate Total Cost
                if f < self.cost: # If cost of moving to this neighbor less than cost of current position
                    temp_current = self.open.locations[i]
                    self.cost = self.heuristic(current, goal) + self.cameFrom.getValue(current, self.closed) + self.grid[current[0], current[1]]
            self.cameFrom.update(temp_current, current, self.grid)
            current = temp_current



        self.path = self.reconstruct(current, self.start)
        return self


    def heuristic(self, current, goal): # Prediction
        (x1, y1) = current
        (x2, y2) = goal
        return abs(x1 - x2) + abs(y1 - y2)

    def cost(): # Cost of all

        return g

    def minF(self):
        ind = np.argmin(np.asarray(self.open.values))
        return ind

    def reconstruct(self, current, start):
        print('Reconstructing Path')
        print('Start: ', start)
        print('Goal: ', current)
        path = []
        while current != start:
            self.grid[current[0]][current[1]] == 4
            self.path = [self.cameFrom.locations[current[0]][current[1]], current]
            current = self.cameFrom.locations[current[0]][current[1]]
            print(current)
            time.sleep(0.5)
        return path




class opn:
    def __init__(self, start, grid):
        self.locations = []
        self.values = []
        self.locations.append(start)
        self.values.append(grid[start[0], start[1]])

    def add(self, neighbor):
        self.locations.append(current)
        self.values.append(grid[current[0], current[1]])
        return self

    def remove(self, current, grid):
        ind = self.locations.index(current)
        del self.locations[ind]
        del self.values[ind]
        return self

    def findNeighbors(self, current, grid, gridSize):
        for i in (-1, 0, 1):
            for j in range(-1, 0, 1):
                if current[0]+i >= 0 and current[0]+i < gridSize[0] and current[1]+j >= 0 and current[1]+j < gridSize[1]:
                    if grid[current[0]+i, current[1]+j] != 1 and [current[0]+i, current[1]+j] not in self.locations:
                        self.locations.append([current[0]+i, current[1]+j])
                        self.values.append(grid[current[0]+i, current[1]+j])
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
    def __init__(self, gridSize, start):
        # self.locations = np.empty([gridSize[0], gridSize[1]])
        self.locations = [[None for _ in range(gridSize[0])] for _ in range(gridSize[1])]
        self.locations[start[0]][start[1]] = start

    def update(self, current, previous, grid):
        self.locations[current[0]][current[1]] = previous
        return self

    def getValue(self, current, closed, previous = [-1, -1]):
        if previous == [-1, -1]:
            cameFrom = self.locations[current[0]][current[1]]
        else:
            cameFrom = previous
        ind = closed.locations.index(cameFrom)
        # if not ind:
        #     raise ValueError('Could not find cameFrom in closed.locations')
        value = closed.values[ind]
        return value



# class render:
#     def showGrid(self):

#     def showSearch(self):

#     def showPath(self):


def run(gridSize, numObjects, maxSize):
    grid = world(gridSize, numObjects, maxSize)
    agent = robot(grid.grid, gridSize)
    print('Start: ', agent.start)
    print('Goal: ', agent.goal)
    runner = myLazySearch(agent.start, grid.grid, gridSize)
    runner = runner.search(agent.start, agent.goal)

    return runner.grid, runner.path

if __name__ == "__main__":
    gridSize = [100, 100]
    numObjects = 50
    maxSize = 5
    grid, path = run(gridSize, numObjects, maxSize)
    print(path)
