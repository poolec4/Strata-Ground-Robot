from collections import defaultdict, deque
import numpy as np
import math
import pdb

class Planner:
    def __init__(self):
        self.graph = Graph()
        self.world = World([7, 7], 1, 1)
        self.graph = self.graph.add_nodes_and_edges(self.world)
        print('Encoded Grid')
        print(self.world.enc)

    def plan(self, start, dest, nextGrid=[-1, -1]):
        self.abs_goal = dest
	    start = self.coords2enc(start)
        dest = self.coords2enc(dest)
        print('start: ', start)
        print('dest: ', dest)
        path_obj = shortest_path(self.graph, start, dest)
        path_cost = path_obj[0]
        path = path_obj[1]
        x_coords, y_coords = path2coords(path, self.world)
        x_coords.append(self.abs_goal[0])
	    y_coords.append(self.abs_goal[1])
        angles = getAngles(path, self.world, nextGrid=nextGrid)
        return x_coords, y_coords, angles, path # coodinates in meters, angles in radians

    def coords2enc(self, coords):
        grid_coords = self.coords2grid(coords)
        print(grid_coords)
        grid_enc = self.world.enc[grid_coords[0]][grid_coords[1]]
        return grid_enc

    def coords2grid(self, coords):
        grid = [-1, -1]
	    coords = invTransform(coords)
	    print('transformed: ', coords)
        for i in range(len(coords)):
            grid[i] = int((1.0/2.0)*(coords[i]/0.3048 - 1))
        return grid

    def planWaypoints(self, start, goals):
        x_coords = []
	y_coords = []
        angles = []
        path = []
	current = start
        # current = invTransform(start)
        for i in range(len(goals)):
            # if i < len(goals)-1:
            #     nextGrid = self.coords2grid(goals[i+1])
            # else:
            nextGrid = [-1, -1]
            x_coords_, y_coords_, angles_, path_ = self.plan(current, goals[i], nextGrid=nextGrid)
            x_coords.append(x_coords_)
	    y_coords.append(y_coords_)
            angles.append(angles_)
            path.append(path_)
            current = goals[i]
        return x_coords, y_coords, angles, path


class World:
    def __init__(self, gridSize, numObjects, maxSize):
        self.grid = self.generate(gridSize, numObjects, maxSize)
	    print(self.grid)
        self.gridSize = gridSize
        self.enc = self.encode()
        self.neighbors = self.findNeighbors()

    def generate(self, gridSize, numObjects, maxSize, random=False):
        grid = np.zeros(gridSize)
        for i in range(0, numObjects):
            if random == True:
                center = [random.randint(0.1*gridSize[0], 0.9*gridSize[0]), random.randint(0.2*gridSize[1], 0.9*gridSize[1])]
                size = random.randint(0, maxSize)
		# for i in ([-1, 0, 1]):
		#   for j in range ([-1, 0, 1]):
		#	grid[center[0]+i, center[0]+j] = 1
                grid[center[0], center[1]] = 1
                loc = center
                for j in range(0, size):
                    index = random.randint(0, 3)
                    loc, grid = self.growObject(grid, loc, index, gridSize)
            else:
                center = [int(round(gridSize[0]/2.0)), int(round(gridSize[1]/2.0))]
                size = maxSize
		for i in ([-1, 0, 1]):
		   for j in ([-1, 0, 1]):
			grid[center[0]+i][center[1]+j] = 1
                grid[int(center[0]), int(center[1])] = 2

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

    def encode(self):
        enc_val = 0
        enc_grid = np.empty(self.gridSize)
        for i in range(self.gridSize[0]):
            for j in range(self.gridSize[1]):
                enc_grid[i, j] = int(enc_val)
                enc_val += 1
        return enc_grid

    def decode(self, encVal):
        for i in range(self.gridSize[0]):
            for j in range(self.gridSize[1]):
                if self.enc[i, j] == encVal:
                    coords = [i, j]
                    return coords


    def findNeighbors(self):
        neighbors = []
        for n in range(self.gridSize[0]*self.gridSize[1]):
            neighbors.append([])
        for i in range(self.gridSize[0]):
            for j in range(self.gridSize[1]):
                for x in ([-1, 0, 1]):
                    for y in ([-1, 0, 1]):
                        potentialNeighbor = [i+x, j+y]
                        if self.isNeighbor([i, j], potentialNeighbor) is True:
                            neighbors[int(self.enc[i, j])].append(potentialNeighbor)

        return neighbors

    def isNeighbor(self, current, potentialNeighbor):
        if potentialNeighbor[0] < 0 or potentialNeighbor[1] < 0 or potentialNeighbor[0] > self.gridSize[0] or potentialNeighbor[1] > self.gridSize[1]:
            return False
        elif current[0] == potentialNeighbor[0] and current[1] == potentialNeighbor[1]:
            return False
        else:
            return True


    def cost(self, current, destination):
        move_cost = self.grid[destination[0], destination[1]]
        if current[0]-destination[0] == 0 or current[1]-destination[1] == 0:
            move_cost += 1
        else:
            move_cost += 1.2
        return move_cost


class Graph(object):
    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.distances[(from_node, to_node)] = distance

    def add_nodes_and_edges(self, world):
        for i in range(world.gridSize[0]):
            for j in range(world.gridSize[1]):
                node = world.enc[i, j]
                self.add_node(node)
        for i in range(world.gridSize[0]):
            for j in range(world.gridSize[1]):
                for k in range(len(world.neighbors[int(world.enc[i, j])])):
                    enc_ind = world.neighbors[int(world.enc[i, j])][k]
                    if enc_ind[0] < world.gridSize[0] and enc_ind[1] < world.gridSize[1]:
                        if int(world.enc[enc_ind[0]][enc_ind[1]]) not in self.edges[int(world.enc[i, j])]:
                            current = [i, j]
                            edge = enc_ind
                            self.add_edge(int(world.enc[i, j]), int(world.enc[enc_ind[0]][enc_ind[1]]), world.cost(current, edge))
        return self


def dijkstra(graph, initial):
    visited = {initial: 0}
    path = {}

    nodes = set(graph.nodes)

    while nodes:
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node
        if min_node is None:
            break

        nodes.remove(min_node)
        current_weight = visited[min_node]

        for edge in graph.edges[min_node]:
            try:
                if (min_node < edge):
                    weight = current_weight + graph.distances[(min_node, edge)]
                else:
                     weight = current_weight + graph.distances[(edge, min_node)]
            except:
                continue
            if edge not in visited or weight < visited[edge]:
                visited[edge] = weight
                path[edge] = min_node

    return visited, path


def shortest_path(graph, origin, destination):
    visited, paths = dijkstra(graph, origin)
    full_path = deque()
    _destination = paths[destination]

    while _destination != origin:
        full_path.appendleft(_destination)
        _destination = paths[_destination]

    full_path.appendleft(origin)
    full_path.append(destination)

    return visited[destination], list(full_path)

def path2coords(path, world):
    x_coords = []
    y_coords = []
    for i in range(len(path)-1):
        grid_coords = world.decode(path[i])
        coords_meters = [0.3048*(grid_coords[0]*2+1), 0.3048*(grid_coords[1]*2+1)]
        coords_meters_offset = coordTransform(coords_meters)
        x_coords.append(coords_meters_offset[0])
    	y_coords.append(coords_meters_offset[1])
    return x_coords, y_coords

def getAngles(path, world, nextGrid=[-1, -1]): # angle in radians
    angle = []
    i = 0
    while(i < len(path)-1):
        current_coords = world.decode(path[i])
        next_coords = world.decode(path[i+1])
        theta = math.atan2(next_coords[1]-current_coords[1], next_coords[0]-current_coords[0]) # numpy grid flips x and y coords
        angle.append(theta)
        i += 1
    if nextGrid[0] == -1:
        angle.append(0.0) # Final angle of zero if no next grid given
    else:
        current_coords = world.decode(path[len(path)-1])
        next_coords = nextGrid
        theta = math.atan2(next_coords[1]-current_coords[1], next_coords[0]-current_coords[0]) # numpy grid flips x and y coords
        angle.append(theta)
    return angle

def coordTransform(coords, offset=[-2.1336, -2.1336]):
    return [coords[0] + offset[0], coords[1] + offset[1]]

def invTransform(coords, offset=[2.1336, 2.1336]):
    return [coords[0] + offset[0], coords[1] + offset[1]]

if __name__ == '__main__':
    planner = Planner()
    start = [-1.85, -1.85]
    goals = [[1.2, 3], [3.524, 4], [4, 1]]
    coords, angles, path = planner.planWaypoints(start, goals)
    print('coords: ', coords)
    print('angles: ', angles)
    print('path: ', path)
