import numpy as np
import time
from collections import defaultdict, deque
import math
from matplotlib import pyplot as plt

# Small grids with cost >half width of robot around obstacle
# Consider all grids behind obstacle occupied
# Replan when parallel with furthest detected obstable

class World:
    def __init__(self, depth_map, world_size=[100, 100]):
        self.world = self.generate(world_size)
        self.enc_world = self.encode(world_size)
        bounds = np.asarray(getBounds(depth_map))
        # print('bounds: ', bounds)
        self.grid_size = self.gridSize(world_size, bounds)
        self.world = self.addObstacles(depth_map, bounds, world_size)
        self.world_size = world_size
        self.neighbors = self.findNeighbors()

    def generate(self, world_size):
        world = np.zeros(world_size)
        return world

    def encode(self, world_size):
        enc_val = 0
        enc_world = np.empty(world_size)
        for i in range(world_size[0]):
            for j in range(world_size[1]):
                enc_world[i, j] = int(enc_val)
                enc_val += 1
        return enc_world

    def decode(self, encVal):
        for i in range(self.world_size[0]):
            for j in range(self.world_size[1]):
                if self.enc_world[i, j] == encVal:
                    coords = [i, j]
                    return coords

    def addObstacles(self, depth_map, bounds, world_size):
        dones = np.ones(world_size)
        for i in range(depth_map.shape[0]):
            x = depth_map[i, 0]-bounds[0, 0]
            y = depth_map[i, 2]-bounds[1, 0]
            z = depth_map[i, 1]-bounds[2, 0]
            # print('x, y: ', x, y)
            index = self.coords2grid(x, y)
            # print('index: ', index)
            if z > self.world[index[0], index[1]]:
                self.world[index[0], index[1]] = z
            else:
                dones[index[0], index[1]] = 0
                if np.max(dones) == 0:
                    print('Exited at Sample: ', i)
                    self.dones = dones
                    break
        self.dones = dones
        return self.world

    def gridSize(self, world_size, bounds): # Returns size of each grid in meters
        x_factor = (bounds[0, 1] - bounds[0, 0])/world_size[0]
        y_factor = (bounds[1, 1] - bounds[1, 0])/world_size[1]
        return  x_factor, y_factor

    def coords2grid(self, x, y):
        x_index = int(x/self.grid_size[0]-0.5)
        y_index = int(y/self.grid_size[1]-0.5)
        return x_index, y_index

    def findNeighbors(self):
        neighbors = []
        for n in range(self.world_size[0]*self.world_size[1]):
            neighbors.append([])
        for i in range(self.world_size[0]):
            for j in range(self.world_size[1]):
                for x in ([-1, 0, 1]):
                    for y in ([-1, 0, 1]):
                        potentialNeighbor = [i+x, j+y]
                        if self.isNeighbor([i, j], potentialNeighbor) is True:
                            neighbors[int(self.enc_world[i, j])].append(potentialNeighbor)

        return neighbors

    def isNeighbor(self, current, potentialNeighbor):
        if potentialNeighbor[0] < 0 or potentialNeighbor[1] < 0 or potentialNeighbor[0] > self.world_size[0] or potentialNeighbor[1] > self.world_size[1]:
            return False
        elif current[0] == potentialNeighbor[0] and current[1] == potentialNeighbor[1]:
            return False
        else:
            return True

    def cost(self, current, destination):
        move_cost = self.world[destination[0], destination[1]]
        if current[0]-destination[0] == 0 or current[1]-destination[1] == 0:
            move_cost += 1
        else:
            move_cost += 1.2
        return move_cost

    def addBuffer(self, depth_map, threshold): # Adds buffer around obstacle
        # To be completed
        return 0

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
        for i in range(world.world_size[0]):
            for j in range(world.world_size[1]):
                node = world.enc_world[i, j]
                self.add_node(node)
        for i in range(world.world_size[0]):
            for j in range(world.world_size[1]):
                for k in range(len(world.neighbors[int(world.enc_world[i, j])])):
                    enc_ind = world.neighbors[int(world.enc_world[i, j])][k]
                    if enc_ind[0] < world.world_size[0] and enc_ind[1] < world.world_size[1]:
                        if int(world.enc_world[enc_ind[0]][enc_ind[1]]) not in self.edges[int(world.enc_world[i, j])]:
                            current = [i, j]
                            edge = enc_ind
                            self.add_edge(int(world.enc_world[i, j]), int(world.enc_world[enc_ind[0]][enc_ind[1]]), world.cost(current, edge))
        return self

# Depth Map Functions
def coordTransform(depth_map):
    # depth_map[:, 1] *= -1 # Uncomment for execution
    depth_map = np.asarray(depth_map[depth_map[:,1].argsort()]) # Sort by y column
    depth_map = depth_map[::-1] # Flip Order (High to Low)
    return depth_map

def getBounds(depth_map): # Translates to robot's frame
    x_bounds = np.asarray([np.min(depth_map[:, 0]), np.max(depth_map[:, 0])]).flatten()
    y_bounds = np.asarray([np.min(depth_map[:, 2]), np.max(depth_map[:, 2])]).flatten()
    z_bounds = np.asarray([np.min(depth_map[:, 1]), np.max(depth_map[:, 1])]).flatten()
    return x_bounds, y_bounds, z_bounds

# Search Algorithm Functions
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

# Path Planning Functions
def coords2enc(coords, world): # returns encoded grid value
    grid_coords = world.coords2grid(coords[0], coords[1])
    grid_enc = world.enc_world[grid_coords[0]][grid_coords[1]]
    return grid_enc

def coords2dist(grid_coords, grid_size): # returns coordinates in meters
    coords = []
    for i in range(len(grid_coords)):
        coords.append(grid_size[i]*(grid_coords[i]+0.5))
    return coords

def path2coords(path, world, grid_size):
    x_coords = []
    y_coords = []
    for i in range(len(path)-1):
        grid_coords = world.decode(path[i])
        print(grid_coords)
        coords = coords2dist(grid_coords, grid_size)
        x_coords.append(coords[0])
        y_coords.append(coords[1])
    return x_coords, y_coords

def getAngles(path, world):
    angles = []
    for i in range(len(path)-1):
        current_coords = world.decode(path[i])
        next_coords = world.decode(path[i+1])
        theta = math.atan2(next_coords[1]-current_coords[1], next_coords[0]-current_coords[0]) # numpy grid flips x and y coords
        angles.append(theta)
    angles.append(math.pi/2.0)
    return angles

def local2world(x_coords, y_coords, angles, start, start_angle): # start in global frame
    x_global = []
    y_global = []
    angles_global = []
    for i in range(len(x_coords)):
        x_global.append(x_coords[i]+start[0])
        y_global.append(y_coords[i]+start[1])
        angles_global.append(angles[i]+start_angle-math.pi/2.0)

    return x_global, y_global, angles_global



def plan(start, dest, depth_map, world_size): # Called to plan trajectory
    graph = Graph()
    world = World(depth_map, world_size=world_size)
    graph = graph.add_nodes_and_edges(world)
    start_enc = coords2enc(start, world)
    dest_enc = coords2enc(dest, world)
    path_obj = shortest_path(graph, start_enc, dest_enc)
    path_cost = path_obj[0]
    path = path_obj[1]
    x_coords, y_coords = path2coords(path, world, world.grid_size)
    angles = getAngles(path, world)
    return x_coords, y_coords, angles, path, path_cost, world


if __name__ == '__main__':
    # Start: Robot's Current Location in Vicon
    # Dest: Goal Location in Vicon
    # These will need to be translated to current kinect frame
    # Start_kinect: [0, 0] grid
    # Dest_kinect: grid in kinect world closest to goal location
    # Should end with angle theta such that it faces the destination
    depth_map = np.random.rand(300000, 3)
    global_start = [2, 3]
    global_angle = math.pi/4.0
    local_start = np.asarray([0.5, 0]) # should be 0, 0 in implementation
    local_dest = np.random.rand(1, 2).flatten()
    world_size = [50, 50]
    depth_map = coordTransform(depth_map)
    t = time.time()
    x_coords, y_coords, angles, path, path_cost, world = plan(local_start, local_dest, depth_map, world_size)
    x_global, y_global, angles_global = local2world(x_coords, y_coords, angles, global_start, global_angle)
    print('Execution Time: ', time.time()-t)
    print(world.enc_world)
    print(path)
    print(angles)
    print(angles_global)
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(x_coords, y_coords)
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(x_global, y_global)
    plt.show()
