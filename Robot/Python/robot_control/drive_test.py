import serial
import time
import math
import numpy as np
from vicon import vicon
from robot import robot

TCP_IP = '127.0.0.1'
TCP_PORT = 5000
BUFFER_SIZE = 1024

vicon = vicon(TCP_IP, TCP_PORT, BUFFER_SIZE)

# motor vals here are going to be defined as 0 to 255 for forwards and 0 to -255 for backwards
# The conversions will be done in the robot class before writing to arduino

robot = robot('/dev/ttyACM1', 19200)
robot = robot.write_motors()

t_init = time.time()

while (time.time()-t_init) < 10.0:
	vicon.get_state();

	print(vicon.x_v)
	robot.motor_vals = 50*np.ones(6)
	robot = robot.write_motors()

robot.stop_robot()

