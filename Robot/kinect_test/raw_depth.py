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
from kinect import Kinect
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt

kinect = Kinect()
cv2.namedWindow('Depth')

plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# for i in range(1):
while 1:
    raw_depth = kinect.get_raw_depth()
    # raw_depth = np.random.rand(kinect.height,kinect.width)*2047
    # raw_depth = cv2.imread('test_depth6_640x480.png',0)*(2047.0/255.0)
    
    print(raw_depth)
    pcl = kinect.get_point_cloud(raw_depth)

    ax.clear()
    ax.scatter(pcl[:,0], pcl[:,2], -pcl[:,1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-3,3)
    ax.set_ylim(0,6)
    ax.set_zlim(-2,4)
    ax.view_init(elev=0., azim=-90)

    plt.pause(0.01)
    pdb.set_trace()
    cv2.imshow('Depth', raw_depth/2047.0) 
    if cv2.waitKey(10) == 27:
        break

plt.show()

#freenect.sync_stop()  # NOTE: Uncomment if your machine can't handle it