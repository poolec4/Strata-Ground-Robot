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

cv2.namedWindow('Depth')
ind = 0

def get_depth(ind):
    return freenect.sync_get_depth(ind)[0]/2047.0

while 1:
    try:
        depth = get_depth(ind)
        print(depth)
        # pdb.set_trace()
        cv2.imshow('Depth',(depth))
        if cv2.waitKey(10) == 27:
            break

    except TypeError:
        ind = 0
        continue
#freenect.sync_stop()  # NOTE: Uncomment if your machine can't handle it