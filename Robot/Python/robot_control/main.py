import sys
import select
import serial
import time
import math
import termios
import tty
import numpy as np
from vicon import vicon
from robot import robot

TCP_IP = '192.168.10.31'
TCP_PORT = 5000
BUFFER_SIZE = 1024

x_g = [0, 0];
th_g = 0;

vicon = vicon(TCP_IP, TCP_PORT, BUFFER_SIZE)

robot = robot()

robot.open('/dev/ttyACM0', 19200)
robot.write_motors()

robot.set_goal(x_g, th_g)
t_init = time.time()

RUN_ROBOT = True

while (RUN_ROBOT == True):
	vicon.get_state();

	robot = robot.P_control(vicon.x_v, vicon.q_v)
	robot = robot.write_motors()

	time.sleep(0.05)

robot.stop_robot()
