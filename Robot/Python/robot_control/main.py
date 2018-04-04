import serial
import time
import math
import numpy as np
from robot import robot

# motor vals here are going to be defined as 0 to 255 for forwards and 0 to -255 for backwards
# The conversions will be done in the robot class before writing to arduino

robot = robot('/dev/ttyACM1', 19200)
robot = robot.write_motors()

robot = robot.set_goal(0, 0, 0)
t_init = time.time()

while (time.time()-t_init) < 10.0:
	# get vicon data
	robot = robot.P_control()
	robot = robot.write_motors()

robot.stop_robot()

