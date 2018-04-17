#!/usr/bin/env python
"""This goes through each kinect on your system, grabs one frame and
displays it.  Uncomment the commented line to shut down after each frame
if your system can't handle it (will get very low FPS but it should work).
This will keep trying indeces until it finds one that doesn't work, then it
starts from 0.
"""
import freenect
import cv2
import pdb
import numpy as np
import random
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from kinect import Kinect

kinect = Kinect()
cv2.namedWindow('Depth')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(1):
    # depth = kinect.get_raw_depth()
    # depth = np.random.rand(kinect.height,kinect.width)*2047
    depth = cv2.imread('test_depth6_640x480.png',0)*(2047.0/255.0)
    
    print(depth)
    # pdb.set_trace()
    p_w = np.empty([3,kinect.width*kinect.height])
    
    dpx = 10

    for x in dpx*np.array(range(kinect.width/dpx)):
        for y in dpx*np.array(range(kinect.height/dpx)):
            ind = x + y*kinect.width
            p_w[:,ind] = kinect.raw_depth_to_world(x,y,depth[y,x])

    ax.scatter(p_w[0,:], p_w[1,:], p_w[2,:])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

    # cv2.imshow('Depth', depth/2047.0)
    # if cv2.waitKey(10) == 27:
    #     break

#freenect.sync_stop()  # NOTE: Uncomment if your machine can't handle it