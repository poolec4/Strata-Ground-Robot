import numpy as np
import time
from collections import defaultdict, deque
import math
from matplotlib import pyplot as plt

import pdb

# Small grids with cost >half width of robot around obstacle
# Consider all grids behind obstacle occupied
# Replan when parallel with furthest detected obstable

class World:
    def __init__(self, depth_map, world_size=[100, 100]):
        self.world = self.generate(world_size)
        self.enc_world = self.encode(world_size)
        self.world_size = world_size
        bounds = np.asarray(getBounds(depth_map))
        print('bounds: ', bounds)
        self.grid_size = self.gridSize(world_size, bounds)
        print('grid size: ', self.grid_size)
        self.world = self.addObstacles(depth_map, bounds, world_size)
        self.old_world = self.world
	plt.ion()
	# fig = plt.figure()
	# ax = fig.add_subplot(111)
	# ax.matshow(self.world, cmap='gray')
	# plt.show()
	# plt.ioff()
        self.world = self.addBuffer()
        # print(self.world)
        self.neighbors = self.findNeighbors()
        self.bounds = bounds

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
	    if (depth_map[i, 0]+depth_map[i, 2]+depth_map[i, 1] != 0):
		    x = depth_map[i, 0]-bounds[0, 0]
		    y = depth_map[i, 2]-bounds[1, 0]
		    z = depth_map[i, 1]-bounds[2, 0]
		    # print('x, y: ', x, y)
		    index = self.coords2grid(x, y)
		    # print('index: ', index)
		    if z > self.world[index[0], index[1]]:
			print('object placed at: ', index)
		        self.world[index[0], index[1]] = z
		        # print('added object')
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
        x = x/self.grid_size[0]-0.49
        y = y/self.grid_size[1]-0.49
        if x > self.world_size[0]-1:
            x_index = int(x)
        else:
            x_index = int(round(x))
        if y > self.world_size[1]-1:
            y_index = int(y)
        else:
            y_index = int(round(y))
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
        move_cost = 10*self.world[destination[0], destination[1]]
        if current[0]-destination[0] == 0 or current[1]-destination[1] == 0:
            move_cost += 1
        else:
            move_cost += 1.2
        return move_cost

    def addBuffer(self): # Adds buffer around obstacle
        threshold = 0.0
        new_world = np.copy(self.world)
        buff_width = int(0.25/self.grid_size[0]) # half of robot's width in grids
        print('buff_width: ', buff_width)
        for i in range(self.world_size[0]):
            for j in range(self.world_size[1]):
                if self.world[i, j] > threshold: # object here
                    height = self.world[i, j]
                    for k in range(i-buff_width, i+buff_width):
                        for m in range(j, self.world_size[1]):
                            if k >= 0 and k < self.world_size[0]:
                                if height > self.world[k, m] and self.world[k, m] < height:
                                    new_world[k, m] = height
                                    print('adding buffer at: ', [k, m])

        return new_world

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
    depth_map[:, 1] *= -1 # Uncomment for execution
    # depth_map = np.asarray(depth_map[depth_map[:,1].argsort()]) # Sort by y column
    # depth_map = depth_map[::-1] # Flip Order (High to Low)
    return depth_map

def getBounds(depth_map): # Translates to robot's frame
    x_max = np.max(abs(depth_map[:, 0]))
    x_bounds = np.asarray([-x_max, x_max]).flatten()
    # x_bounds = np.asarray([np.min(depth_map[:, 0]), np.max(depth_map[:, 0])]).flatten()
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
    print(coords)
    grid_coords = world.coords2grid(coords[0], coords[1])
    grid_enc = world.enc_world[grid_coords[0]][grid_coords[1]]
    print('grid_enc ', grid_enc)
    return grid_enc

def coords2dist(grid_coords, grid_size, local_start): # returns coordinates in meters
    coords = []
    for i in range(len(grid_coords)):
        coords.append(grid_size[i]*(grid_coords[i]+0.5-local_start[i]))
    return coords

def path2coords(path, world, grid_size, local_start):
    x_coords = []
    y_coords = []
    for i in range(len(path)):
        grid_coords = world.decode(path[i])
        print(grid_coords)
        coords = coords2dist(grid_coords, grid_size, local_start)
        x_coords.append(coords[0])
        y_coords.append(coords[1])
    return x_coords, y_coords, grid_coords

def getAngles(path, world):
    angles = []
    for i in range(len(path)-1):
        current_coords = world.decode(path[i])
        next_coords = world.decode(path[i+1])
        theta = math.atan2(next_coords[1]-current_coords[1], next_coords[0]-current_coords[0]) # numpy grid flips x and y coords
        angles.append(theta)
    angles.append(0.0)
    return angles

def getLocalGoal(global_start, global_goal, global_angle, world, local_start):
    global_angle = global_angle - math.pi/2
    dist_to_goal = [abs(global_goal[0]-global_start[0]), abs(global_goal[1]-global_start[1])]
    if dist_to_goal[0] <= world.bounds[0, 1] and dist_to_goal[1] <= world.bounds[1, 1]: # goal in current world
        R_gl = [[math.cos(global_angle), math.sin(global_angle)], [-math.sin(global_angle), math.cos(global_angle)]]
        P_g = [[global_goal[0]], [global_goal[1]]]
        P_v = [[global_start[0]], [global_start[1]]]
        print(R_gl)
        local_coords = (np.matmul(R_gl, P_v)-np.matmul(R_gl, P_g)).flatten()
        print('local coords ', local_coords)
        # local_coords = [(global_goal[0]-global_start[0]-local_start[0]*world.grid_size[0], global_goal[1]-global_start[1]]
        # local_coords = coords2enc(local_goal, world)
    else:
        print('out of bounds')
        shortest_path_angle = math.atan2(global_goal[1]-global_start[1], global_goal[0]-global_start[0])  - global_angle # local angle to shortest path
        print('shortest path angle: ', shortest_path_angle)
        local_coords = [0.5*(world.bounds[0, 1]-world.bounds[0, 0])*math.cos(shortest_path_angle)+local_start[0]*world.grid_size[0], (world.bounds[1, 1]-world.bounds[1, 0])*math.sin(shortest_path_angle)]
        print('local coords ', local_coords)
    return local_coords

def local2global(x_coords, y_coords, angles, global_start, global_start_angle, world):
    global_x_coords = []
    global_y_coords = []
    global_angles = []
    global_start_angle -= math.pi/2.0
    for i in range(len(x_coords)):
        global_x_coords.append(x_coords[i]*math.cos(global_start_angle)+y_coords[i]*-math.sin(global_start_angle)+global_start[0])
        global_y_coords.append(x_coords[i]*math.sin(global_start_angle)+y_coords[i]*math.cos(global_start_angle)+global_start[1])
        global_angles.append(angles[i]+global_start_angle)
    return global_x_coords, global_y_coords, global_angles

def plan(start, dest, depth_map, world): # Called to plan trajectory
    graph = Graph()
    graph = graph.add_nodes_and_edges(world)
    # start_enc = coords2enc(start, world)
    start_enc = world.enc_world[start[0], start[1]]
    print('start_enc ', start_enc)
    dest_enc = coords2enc(dest, world)
    print('dest enc ', dest_enc)
    path_obj = shortest_path(graph, start_enc, dest_enc)
    path_cost = path_obj[0]
    path = path_obj[1]
    x_coords, y_coords, grid_coords = path2coords(path, world, world.grid_size, start)
    angles = getAngles(path, world)
    return x_coords, y_coords, angles, path, path_cost, world, grid_coords


if __name__ == '__main__':
    # depth_map = np.asarray([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]).reshape(2, 3)
    depth_map = np.asarray([2.*np.random.rand(100, 1)-1, np.random.rand(100, 1) ,np.random.rand(100, 1)])
    depth_map = np.swapaxes(depth_map, 0, 2)
    depth_map = depth_map[0]
    global_start = [1, 1]
    print(depth_map.shape)
    global_dest = [2, 2]
    global_angle = 2.0*math.pi/4.0 # -math.pi/2
    local_start = [5, 0]
    world_size = [100, 100]
    world = World(depth_map, world_size=world_size)
    # print('depth map: ', depth_map)
    depth_map = coordTransform(depth_map)
    local_dest = getLocalGoal(global_start, global_dest, global_angle, world, local_start)
    print('local dest: ', local_dest)
    # print('sorted: ', depth_map)
    t = time.time()
    # world = World(depth_map)
    x_coords, y_coords, angles, path, path_cost, world, grid_coords = plan(local_start, local_dest, depth_map, world)
    global_x_coords, global_y_coords, global_angles = local2global(x_coords, y_coords, angles, global_start, global_angle, world)
    print('Execution Time: ', time.time()-t)
    print(world.enc_world)
    print(path)
    print(global_angles)
    print(x_coords[0:3])
    print(global_x_coords[0:3])
    print(y_coords[0:3])
    print(global_y_coords[0:3])
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(x_coords, y_coords, 'o')
    ax1.set_xlim([-1, 1])
    ax1.set_ylim([-1, 1])
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(global_x_coords, global_y_coords, 'o')
    ax2.set_xlim([0, 2])
    ax2.set_ylim([0, 2])
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.matshow(np.transpose(world.world), cmap='gray')
    # ax3.matshow(np.transpose(
    plt.show()
    # print(world.dones)
    # print(world.world)
