import numpy as np
from motion_plan import World

# Small worlds with cost >half width of robot around obstacle
# Consider all worlds behind obstable occupied
# Replan when parallel with furthest detected obstable


# Depth Map Functions
def coordTransform(self, depth_map):
    depth_map[:, 1] *= -1
    return depth_map

def bounds(depth_map): # Translates to robot's frame
    x_bounds = [np.min(depth_map[:, 0]), np.max(depth_map[:, 0])]
    y_bounds = [np.min(depth_map[:, 2]), np.max(depth_map[:, 2])]
    z_bounds = [np.min(depth_map[:, 1]), np.max(depth_map[:, 1])]
    return x_bounds, y_bounds, z_bounds


class World:
    def __init__(self, world_size=[100, 100]):
        self.world = self.generate(world_size)
        self.enc_world = self.encode(world_size)
        self.grid_size = self.gridSize(world_size, bounds)

    def generate(self, world_size):
        world = np.zeros(world_size)
        return world

    def encode(self):
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
                if self.enc[i, j] == encVal:
                    coords = [i, j]
                    return coords


    def addObstacles(self, depth_map, offset=0):
        for i in depth_map.shape(0):
            x = depth_map[i, 0]
            y = depth_map[i, 2]
            z = depth_map[i, 1]
            if z > offset:



    def gridSize(self, world_size, bounds): # Returns size of each grid in meters
        x_factor = (bounds[0, 1] - bounds[0, 0])/world_size[0]
        y_factor = (bounds[1, 1] - bounds[1, 0])/world_size[1]
        return  x_factor, y_factor

    def coords2grid(self, x, y):
        x_index = int(x/self.grid_size[0]-0.5)
        y_index = int(y/self.grid_size[1]-0.5)






    def addBuffer(self, depth_map):




if __name__ == '__main__':
    depth_map - coordTransform(depth_map)
