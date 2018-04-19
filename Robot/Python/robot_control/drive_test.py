import time
import math
import numpy as np
from robot import Robot

TCP_IP = '192.168.10.31'
TCP_PORT = 5000
BUFFER_SIZE = 1024


# motor vals here are going to be defined as 0 to 255 for forwards and 0 to -255 for backwards
# The conversions will be done in the robot class before writing to arduino

robot = Robot()
robot.open('/dev/ttyTHS2', 19200)
robot = robot.write_motors()

t_init = time.time()

while (time.time()-t_init) < 5.0:
	robot.motor_vals = 45*np.ones(6)
	robot = robot.write_motors()
	time.sleep(0.05)

robot.stop_robot()

