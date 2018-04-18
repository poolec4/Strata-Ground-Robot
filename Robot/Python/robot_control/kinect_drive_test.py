import serial
import time
import math
import freenect
import pdb

import numpy as np
from vicon import Vicon
from robot import Robot

TCP_IP = '192.168.10.7'
TCP_PORT = 50000
BUFFER_SIZE = 1024

PLAN_TIME = 5;

x_g = [2, 2] # Goal in global coordinates
th_g = 0 # Goal in global coordinates (+CCW)

vicon = Vicon(TCP_IP, TCP_PORT, BUFFER_SIZE)
robot = Robot()

robot.open('/dev/serial0', 19200)
robot.write_motors() # Writes initial value (0) to motors

traj_count = 0

t_init = time.time()
t_plan = 0

RUN_ROBOT = True

try:
	while RUN_ROBOT:
		print("\n")
	
		## GET KINECT DATA AND REPLAN PATH
		if t_plan - time.time() > PLAN_TIME:
			raw_depth = kinect.get_raw_depth() 
			pcl = kinect.get_point_cloud(raw_depth) # list of points that has to go into path planner

			# do some planning with global goal (x_g, th_g) and depth data (pcl)

			# get some trajectories
			[x_traj, y_traj, th_traj] = ...
			traj_length = length(x_traj);
			traj_count = 0;

		## GET VICON DATA
		vicon.get_state()
		t_vicon = time.time()

		## CONTROL ROBOT
		robot.set_goal([x_traj[traj_count],y_traj[traj_count]], th_traj[traj_count])
		robot = robot.PI_control(vicon.x_v, vicon.q_v, 1-0.01)
		robot = robot.write_motors()
		t_send = time.time()		

		if robot.p < 0.1:
			traj_count = traj_count + 1
			robot.integral_reset()

		g_err = math.sqrt((x_g[0]-vicon.x_v[0])**2 + (x_g[1]-vicon.x_v[1])**2)

	 	if g_err < 0.1
	 		RUN_ROBOT = false

except KeyboardInterrupt:
	robot.stop_robot()
	print("Robot stopped.. hopefully")
