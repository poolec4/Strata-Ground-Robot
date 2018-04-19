#!/usr/bin/env python
"""This goes through each kinect on your system, grabs one frame and
displays it.  Uncomment the commented line to shut down after each frame
if your system can't handle it (will get very low FPS but it should work).
This will keep trying indeces until it finds one that doesn't work, then it
starts from 0.
"""
import freenect
#import cv2
import pdb
import math
import numpy as np
import time
#import random
from kinect import Kinect
#from mpl_toolkits.mplot3d import Axes3D
#from matplotlib import cm
import matplotlib.pyplot as plt
from kinect2path import plan, World, coordTransform, getLocalGoal, local2global

kinect = Kinect()
#cv2.namedWindow('Depth')

#plt.ion()

# fig1 = plt.figure()
#ax = fig.add_subplot(111, projection='3d')

fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
fig2 = plt.figure()
ax2 = fig2.add_subplot(111)
fig3 = plt.figure()
ax3 = fig3.add_subplot(111)

global_start = [-1, -1]
global_dest = [1, 1]
global_angle = 1.0*math.pi/4.0 # -math.pi/2
world_size = [30, 15]
local_start = [int(world_size[0]/2.0), 0]

for i in range(5):
    raw_depth = kinect.get_raw_depth()
    time.sleep(0.1)

# for i in range(1):
while 1:
    raw_depth = kinect.get_raw_depth()
    # raw_depth = np.random.rand(kinect.height,kinect.width)*2047
    # raw_depth = cv2.imread('test_depth6_640x480.png',0)*(2047.0/255.0)
    print(np.max(raw_depth),np.min(raw_depth))
    pcl = kinect.get_point_cloud(raw_depth)

    #plt.imshow(raw_depth)
    print(pcl.shape)
    depth_map = coordTransform(pcl)
    world = World(depth_map, world_size=world_size)
    if world.bounds[1, 0] > -1:
    	local_dest = getLocalGoal(global_start, global_dest, global_angle, world, local_start)
    	x_coords, y_coords, angles, path, path_cost, world, grid_coords = plan(local_start, local_dest, 	depth_map, world)
    	global_x_coords, global_y_coords, global_angles = local2global(x_coords, y_coords, angles, 		global_start, global_angle, world)
    
        ax1.cla()
        ax2.cla()
        ax3.cla()
        ax1.plot(x_coords, y_coords, 'o')
        ax1.set_xlim([-2, 2])
        ax1.set_ylim([0, 4])
        ax2.plot(global_x_coords, global_y_coords, 'o')
        ax2.set_xlim([-2, 2])
        ax2.set_ylim([-2, 2])
        ax3.matshow(world.world, cmap='gray')
        plt.ion()
        plt.show()
        plt.pause(0.01)
    else:
	print('Threw out bad data')
 #   ax.clear()
 #  ax.scatter(pcl[:,0], pcl[:,2], -pcl[:,1])
 #   ax.set_xlabel('X')
 #   ax.set_ylabel('Y')
 #   ax.set_zlabel('Z')
 #  ax.set_xlim(-3,3)
 #   ax.set_ylim(0,6)
 #   ax.set_zlim(-2,4)
 #   ax.view_init(elev=0., azim=-90)

 #   plt.pause(0.01)
 #   pdb.set_trace()
 #   cv2.imshow('Depth', raw_depth/2047.0)
 #   if cv2.waitKey(10) == 27:
 #       break

#plt.show()

#freenect.sync_stop()  # NOTE: Uncomment if your machine can't handle it
