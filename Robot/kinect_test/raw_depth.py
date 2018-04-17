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
import matplotlib.pyplot as plt

kinect = Kinect()
cv2.namedWindow('Depth')

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# for i in range(1):
while 1:
    raw_depth = kinect.get_raw_depth()
    # raw_depth = np.random.rand(kinect.height,kinect.width)*2047
    # raw_depth = cv2.imread('test_depth6_640x480.png',0)*(2047.0/255.0)
    
    print(raw_depth)
    p_w = []
    
    dpx = 1
    ind = 0

    for x in dpx*np.array(range(kinect.width/dpx)):
        for y in [479]: #dpx*np.array(range(kinect.height/dpx)):
            if raw_depth[y,x] != 0:
                # depth_meters[y,x] = kinect.raw_depth_to_meters(raw_depth[y,x])
                p = kinect.raw_depth_to_world(x,y,raw_depth[y,x])
                p_w.append(p)
                ind = ind + 1

    if not p_w:
        p_w = [[0, 0, 0],[0,0,0]]

    p_wmat = np.asarray(p_w)

    plt.clf()
    plt.scatter(p_wmat[:,0], p_wmat[:,2], s=1, alpha=0.5)
    plt.axis([-3, 3, 0, 6])
    plt.pause(0.01)

    cv2.imshow('Depth', raw_depth/2047.0) 
    if cv2.waitKey(10) == 27:
        break

plt.show()

#freenect.sync_stop()  # NOTE: Uncomment if your machine can't handle it