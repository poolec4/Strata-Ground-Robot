from collections import defaultdict, deque
import numpy as np

class world:
    def __init__(self, gridSize, numObjects, maxSize):
        self.grid = self.generate(gridSize, numObjects, maxSize)
        self.gridSize = gridSize
        self.enc = self.encode()
        self.neighbors = self.findNeighbors()

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

    def encode(self):
        enc_val = 0
        enc_grid = np.empty(self.gridSize)
        for i in range(self.gridSize[0]):
            for j in range(self.gridSize[1]):
                enc_grid[i, j] = int(enc_val)
                enc_val += 1
        return enc_grid

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
        if potentialNeighbor[0] < 0 or potentialNeighbor[1] < 0 or potentialNeighbor[0] >= self.gridSize[0] or potentialNeighbor[1] >= self.gridSize[1]:
            return False
        elif current[0] == potentialNeighbor[0] and current[1] == potentialNeighbor[1]:
            return False
        else:
            return True


    def cost(self, gridSize):

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
                weight = current_weight + graph.distances[(min_node, edge)]
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

if __name__ == '__main__':
    graph = Graph()
    world = world([10, 10], 0, 0)
    for i in range(world.gridSize[0]):
        for j in range(world.gridSize[1]):
            node = world.enc[i, j]
            graph.add_node(node)
    # print(graph.nodes)
    # print(world.neighbors[0])
    print('Encoded World')
    print(world.enc)
    # print([int(world.enc[0, 0]), 0])
    # print('--')
    # ind = world.neighbors[int(world.enc[0, 0])][0]
    # print(ind)
    # print(ind[0])
    # print(world.neighbors[ind[0]][ind[1]])
    # print('Right Val')
    # enc_ind = world.neighbors[ind[0]][ind[1]]
    # print(world.enc[enc_ind[0]][enc_ind[1]])
    # print(world.enc[(world.neighbors[ind[0]][ind[1]])])
    for i in range(world.gridSize[0]-1):
        for j in range(world.gridSize[1]-1):
            for k in range(len(world.neighbors[int(world.enc[i, j])])):
                # print('k', k)
                ind = world.neighbors[int(world.enc[i, j])][k]
                # print('ind: ', ind)
                # enc_ind = world.neighbors[ind[0]][ind[1]]
                enc_ind = ind
                # print('enc_ind: ', enc_ind)
                if int(world.enc[enc_ind[0]][enc_ind[1]]) not in graph.edges[int(world.enc[i, j])]:
                    graph.add_edge(int(world.enc[i, j]), int(world.enc[enc_ind[0]][enc_ind[1]]), 1)
                # print('Current: ', int(world.enc[i, j]))
                # print('Edge: ', int(world.enc[enc_ind[0]][enc_ind[1]]))
    # graph.add_edge('A', 'B', 10)
    # graph.add_edge('A', 'C', 20)
    # graph.add_edge('B', 'D', 15)
    # graph.add_edge('C', 'D', 30)
    # graph.add_edge('B', 'E', 50)
    # graph.add_edge('D', 'E', 30)
    # graph.add_edge('E', 'F', 5)
    # graph.add_edge('F', 'G', 2)
    # print(graph.edges[13])
    print(shortest_path(graph, 18, 99)) # output: (25, ['A', 'B', 'D'])
