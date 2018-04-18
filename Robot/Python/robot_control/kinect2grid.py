import numpy as np
import time

# Small worlds with cost >half width of robot around obstacle
# Consider all worlds behind obstable occupied
# Replan when parallel with furthest detected obstable


# Depth Map Functions
def coordTransform(depth_map):
    # depth_map[:, 1] *= -1
    # depth_map = np.sort(depth_map.view('float16,float16,float16'), order=['f1'], axis=0) # sort y column
    depth_map = np.asarray(depth_map[depth_map[:,1].argsort()])
    depth_map = depth_map[::-1]
    return depth_map

def getBounds(depth_map): # Translates to robot's frame
    x_bounds = np.asarray([np.min(depth_map[:, 0]), np.max(depth_map[:, 0])]).flatten()
    y_bounds = np.asarray([np.min(depth_map[:, 2]), np.max(depth_map[:, 2])]).flatten()
    z_bounds = np.asarray([np.min(depth_map[:, 1]), np.max(depth_map[:, 1])]).flatten()
    return x_bounds, y_bounds, z_bounds


class World:
    def __init__(self, depth_map, world_size=[100, 100]):
        self.world = self.generate(world_size)
        self.enc_world = self.encode(world_size)
        bounds = np.asarray(getBounds(depth_map))
        # print('bounds: ', bounds)
        self.grid_size = self.gridSize(world_size, bounds)
        # print('grid_size: ', self.grid_size)
        self.world = self.addObstacles(depth_map, bounds, world_size)

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
                if self.enc[i, j] == encVal:
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
                    print('Broke at Sample: ', i)
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

    def addBuffer(self, depth_map):
        return 0




if __name__ == '__main__':
    # depth_map = np.asarray([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]).reshape(2, 3)
    depth_map = np.random.rand(300000, 3)
    # print('depth map: ', depth_map)
    depth_map = coordTransform(depth_map)
    # print('sorted: ', depth_map)
    start = time.time()
    world = World(depth_map)
    print('Execution Time: ', time.time()-start)
    # print(world.dones)
    # print(world.world)
