import serial
import time
import math
import freenect
import pdb

import numpy as np
from vicon import Vicon
from kinect import Kinect
from robot import Robot
from kinect2path import plan, World, coordTransform, getLocalGoal, local2global

TCP_IP = '192.168.10.7'
TCP_PORT = 50000
BUFFER_SIZE = 1024

PLAN_TIME = 2.0;

x_g = [0, 0] # Goal in global coordinates
th_g = math.pi # Goal in global coordinates (+CCW)

vicon = Vicon(TCP_IP, TCP_PORT, BUFFER_SIZE)
kinect = Kinect()
robot = Robot()

robot.open('/dev/ttyTHS2', 19200)
robot.write_motors() # Writes initial value (0) to motors

traj_count = 0

t_init = time.time()
t_plan = t_init

world_size = [30, 15]
local_start = [int(world_size[0]/2.0), 0]

for i in range(5):
    raw_depth = kinect.get_raw_depth()
    time.sleep(0.1)

RUN_ROBOT = True
FIRST_LOOP = True

try:
	while RUN_ROBOT:
		print("\n")

		## GET VICON DATA
		vicon.get_state()
		print(vicon.x_v[0:2].shape)
		t_vicon = time.time()
	
		## GET KINECT DATA AND REPLAN PATH
		if time.time() - t_plan >= PLAN_TIME or FIRST_LOOP == True:
			print("Planning...")
			raw_depth = kinect.get_raw_depth() 
			pcl = kinect.get_point_cloud(raw_depth) # list of points that has to go into path planner
    			depth_map = coordTransform(pcl)
    			world = World(depth_map, world_size=world_size)
    			if world.bounds[1, 0] > -1:
				global_start = np.asarray(vicon.x_v[0:2]).flatten()
    				local_dest = getLocalGoal(global_start, x_g, vicon.th_r, world, local_start)
    				x_coords, y_coords, angles, path, path_cost, world, grid_coords = plan(local_start, local_dest,depth_map, world)
    				x_traj, y_traj, th_traj = local2global(x_coords, y_coords, angles,vicon.x_v[0:2], vicon.th_r, world)

				traj_length = len(x_traj);
				traj_count = 0;
				t_plan = time.time()
				FIRST_LOOP = False

		## CONTROL ROBOT
		robot.set_goal([x_traj[traj_count],y_traj[traj_count]], th_traj[traj_count])
		robot = robot.PI_control(vicon.x_v, vicon.q_v, 1-0.01)
		robot = robot.write_motors()
		t_send = time.time()		

		if robot.p < 0.1:
			traj_count = traj_count + 1
			robot.integral_reset()

		g_err = math.sqrt((x_g[0]-vicon.x_v[0])**2 + (x_g[1]-vicon.x_v[1])**2)

	 	if g_err < 0.1:
	 		RUN_ROBOT = False

except KeyboardInterrupt:
	robot.stop_robot()
	print("Robot stopped.. hopefully")
