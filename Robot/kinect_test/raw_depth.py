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
from kinect import Kinect

kinect = Kinect()
cv2.namedWindow('Depth')

while 1:
    depth = kinect.get_raw_depth()
    print(depth)
    # pdb.set_trace()
    p_w = np.empty([3,kinect.width*kinect.height])
    for x in range(kinect.width):
        for y in range(kinect.height):
            ind = x + y*kinect.width
            p_w[:,ind] = kinect.raw_depth_to_world(x,y,depth[y,x])

    cv2.imshow('Depth', depth/2047.0)
    if cv2.waitKey(10) == 27:
        break

#freenect.sync_stop()  # NOTE: Uncomment if your machine can't handle it