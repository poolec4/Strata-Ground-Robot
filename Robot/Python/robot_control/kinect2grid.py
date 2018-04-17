import numpy as np
from motion_plan import World

# Small grids with cost >half width of robot around obstacle
# Consider all grids behind obstable occupied
# Replan when parallel with furthest detected obstable

class DepthMap:
    def __init__(self, depth_map):
        self.depth_map = self.coordTransform(depth_map)
        self.map_shape = self.depth_map.shape()

    def coordTransform(self, depth_map):
        depth_map[:, 1] *= -1
        return depth_map

    def

class World:
    def __init__(self, gridSize=[100, 100]):

    def obstacles(self, depth_map)
